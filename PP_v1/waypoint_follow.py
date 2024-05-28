import time
from f110_gym.envs.base_classes import Integrator
from f110_gym.envs.f110_env import F110Env
import yaml
import gym
import numpy as np
from argparse import Namespace

from numba import njit

from pyglet.gl import GL_POINTS

import math

import csv

Output_to = 'Results/Output.csv'


with open(Output_to, mode='w', newline='') as file:
    writer = csv.writer(file)

def add_row(csv_data):
    with open(Output_to, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(csv_data)

"""
Planner Helpers
"""
@njit(fastmath=False, cache=True)
def nearest_point_on_trajectory(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.

    Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
    not be an issue so long as trajectories are not insanely long.

        Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)

    point: size 2 numpy array
    trajectory: Nx2 matrix of (x,y) trajectory waypoints
        - these must be unique. If they are not unique, a divide by 0 error will destroy the world
    """
    diffs = trajectory[1:,:] - trajectory[:-1,:]
    l2s   = diffs[:,0]**2 + diffs[:,1]**2
    # this is equivalent to the elementwise dot product
    # dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
    dots = np.empty((trajectory.shape[0]-1, ))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t<0.0] = 0.0
    t[t>1.0] = 1.0
    # t = np.clip(dots / l2s, 0.0, 1.0)
    projections = trajectory[:-1,:] + (t*diffs.T).T
    # dists = np.linalg.norm(point - projections, axis=1)
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp*temp))
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment

@njit(fastmath=False, cache=True)
def first_point_on_trajectory_intersecting_circle(point, radius, trajectory, t=0.0, wrap=False):
    """
    starts at beginning of trajectory, and find the first point one radius away from the given point along the trajectory.

    Assumes that the first segment passes within a single radius of the point

    http://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
    """
    start_i = int(t)
    start_t = t % 1.0
    first_t = None
    first_i = None
    first_p = None
    trajectory = np.ascontiguousarray(trajectory)
    for i in range(start_i, trajectory.shape[0]-1):
        start = trajectory[i,:]
        end = trajectory[i+1,:]+1e-6
        V = np.ascontiguousarray(end - start)

        a = np.dot(V,V)
        b = 2.0*np.dot(V, start - point)
        c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
        discriminant = b*b-4*a*c

        if discriminant < 0:
            continue
        #   print "NO INTERSECTION"
        # else:
        # if discriminant >= 0.0:
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2.0*a)
        t2 = (-b + discriminant) / (2.0*a)
        if i == start_i:
            if t1 >= 0.0 and t1 <= 1.0 and t1 >= start_t:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            if t2 >= 0.0 and t2 <= 1.0 and t2 >= start_t:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break
        elif t1 >= 0.0 and t1 <= 1.0:
            first_t = t1
            first_i = i
            first_p = start + t1 * V
            break
        elif t2 >= 0.0 and t2 <= 1.0:
            first_t = t2
            first_i = i
            first_p = start + t2 * V
            break
    # wrap around to the beginning of the trajectory if no intersection is found1
    if wrap and first_p is None:
        for i in range(-1, start_i):
            start = trajectory[i % trajectory.shape[0],:]
            end = trajectory[(i+1) % trajectory.shape[0],:]+1e-6
            V = end - start

            a = np.dot(V,V)
            b = 2.0*np.dot(V, start - point)
            c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
            discriminant = b*b-4*a*c

            if discriminant < 0:
                continue
            discriminant = np.sqrt(discriminant)
            t1 = (-b - discriminant) / (2.0*a)
            t2 = (-b + discriminant) / (2.0*a)
            if t1 >= 0.0 and t1 <= 1.0:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            elif t2 >= 0.0 and t2 <= 1.0:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break

    return first_p, first_i, first_t

@njit(fastmath=False, cache=True)
def get_actuation(pose_theta, lookahead_point, position, lookahead_distance, wheelbase):
    """
    Returns actuation
    """
    waypoint_y = np.dot(np.array([np.sin(-pose_theta), np.cos(-pose_theta)]), lookahead_point[0:2]-position)
    speed = lookahead_point[2]
    if np.abs(waypoint_y) < 1e-6:
        return speed, 0.
    radius = 1/(2.0*waypoint_y/lookahead_distance**2)
    steering_angle = np.arctan(wheelbase/radius)
    return speed, steering_angle

def calcSpeed(flag, flagMax, z_2, z_1, minSpeed, maxSpeed):
        speed = minSpeed
        traj_speed_test = 0

        test_flag = (flag == flagMax)

        if test_flag:
            traj_speed_test = abs((z_2 - z_1)*100)

            if traj_speed_test < 10:
                flag = 1
                speed = minSpeed
            else:
                speed = maxSpeed
        else:
            flag = flag + 1
            speed = minSpeed
        return flag, speed

def straight_Test(straight_Mode, currentTime):
    speed = 8

    match straight_Mode:

        case 1:

            if(currentTime >= 5):
                speed = 0

        case 2:

            if(currentTime >= 5 and currentTime < 8):
                speed = 4
            elif(currentTime >= 8 and currentTime < 10):
                speed = 0
            elif(currentTime >= 10 and currentTime < 13):
                speed = 5
            elif(currentTime >= 13 and currentTime < 20):
                speed = 8

        case 3:

            if(currentTime >= 2 and currentTime < 2.5):
                speed = 0
            elif(currentTime >= 2.5 and currentTime < 2.8):
                speed = 8
            elif(currentTime >= 2.8 and currentTime < 2.9):
                speed = 0
            elif(currentTime >= 2.9 and currentTime < 5):
                speed = 8
            elif(currentTime >= 5 and currentTime < 5.1):
                speed = 0
            elif(currentTime >= 5.1 and currentTime < 6):
                speed = 8
            elif(currentTime >= 6):
                speed = 0

    return speed

class PurePursuitPlanner:
    """
    Example Planner
    """
    def __init__(self, conf, wb):
        self.wheelbase = wb
        self.conf = conf
        self.load_waypoints(conf)
        self.max_reacquire = 20.

        self.drawn_waypoints = []

        self.prevWP = 0

        self.drive_Mode = 2
        #Straight Modes: 1,2,3
        self.straight_Mode = 1
        self.start = 0
        self.currentTime = 0
        self.start_pos = np.array([0.0, 0.0])
        self.minSpeed = 2.5
        self.maxSpeed = 4
        self.flagSpeed = 200
        self.flagSpeedMax = self.flagSpeed
        self.lookahead_data = 30

    def load_waypoints(self, conf):
        """
        loads waypoints
        """
        self.waypoints = np.loadtxt(conf.wpt_path, delimiter=conf.wpt_delim, skiprows=conf.wpt_rowskip)

    def render_waypoints(self, e):
        """
        update waypoints being drawn by EnvRenderer
        """

        #points = self.waypoints

        points = np.vstack((self.waypoints[:, self.conf.wpt_xind], self.waypoints[:, self.conf.wpt_yind])).T
        
        scaled_points = 50.*points

        for i in range(points.shape[0]):
            if len(self.drawn_waypoints) < points.shape[0]:
                b = e.batch.add(1, GL_POINTS, None, ('v3f/stream', [scaled_points[i, 0], scaled_points[i, 1], 0.]),
                                ('c3B/stream', [183, 193, 222]))
                self.drawn_waypoints.append(b)
            else:
                self.drawn_waypoints[i].vertices = [scaled_points[i, 0], scaled_points[i, 1], 0.]
        
    def _get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
        """
        gets the current waypoint to follow
        """

        x_temp = self.waypoints[:, self.conf.wpt_xind]
        y_temp = self.waypoints[:, self.conf.wpt_yind]
        wpts = np.vstack((x_temp, y_temp)).T
        nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(position, wpts)

        #speed

        match self.drive_Mode:
            case 1:

                one_test = i+self.lookahead_data
                two_test = i+self.lookahead_data+1

                if one_test >= len(x_temp):
                    one_test = len(x_temp) - one_test

                if two_test >= len(x_temp):
                    two_test = len(x_temp) - two_test

                x_1 = x_temp[one_test]
                x_2 = x_temp[two_test]

                y_1 = y_temp[one_test]
                y_2 = y_temp[two_test]

                z_1 = math.sqrt(x_1**2 + y_1**2)
                z_2 = math.sqrt(x_2**2 + y_2**2)

                x_1_t = x_temp[i]
                x_2_t = x_temp[i+1]

                y_1_t = y_temp[i]
                y_2_t = y_temp[i+1]

                z_1_t = math.sqrt(x_1_t**2 + y_1_t**2)
                z_2_t = math.sqrt(x_2_t**2 + y_2_t**2)

                self.flagSpeed, setSpeed = calcSpeed(self.flagSpeed, self.flagSpeedMax, z_2, z_1, self.minSpeed, self.maxSpeed)
            case 2:

                setSpeed = straight_Test(self.straight_Mode, self.currentTime)

        if nearest_dist < lookahead_distance:
            lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(position, lookahead_distance, wpts, i+t, wrap=True)
            if i2 == None:
                return None
            current_waypoint = np.empty((3, ))
            # x, y
            current_waypoint[0:2] = wpts[i2, :]
            # speed
            current_waypoint[2] = setSpeed

            #print((abs(z_2_t - z_1_t))*100, setSpeed)
            #add_row([(abs(z_2_t - z_1_t))*100])
            #current_waypoint[2] = waypoints[i, self.conf.wpt_vind]
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            #return np.append(wpts[i, :], waypoints[i, self.conf.wpt_vind])
            return np.append(wpts[i, :], waypoints[i, setSpeed])
        else:
            return None

    def plan(self, pose_x, pose_y, pose_theta, lookahead_distance, vgain):
        """
        gives actuation given observation
        """
        position = np.array([pose_x, pose_y])

        lookahead_point = self._get_current_waypoint(self.waypoints, lookahead_distance, position, pose_theta)

        if lookahead_point is None:
            return 4.0, 0.0

        speed, steering_angle = get_actuation(pose_theta, lookahead_point, position, lookahead_distance, self.wheelbase)
        speed = vgain * speed

        return speed, steering_angle
    
    def setStart(self, startTime):
        self.start = startTime

    def setCurrent(self, currentTime):
        self.currentTime = currentTime


class FlippyPlanner:
    """
    Planner designed to exploit integration methods and dynamics.
    For testing only. To observe this error, use single track dynamics for all velocities >0.1
    """
    def __init__(self, speed=1, flip_every=1, steer=2):
        self.speed = speed
        self.flip_every = flip_every
        self.counter = 0
        self.steer = steer
    
    def render_waypoints(self, *args, **kwargs):
        pass

    def plan(self, *args, **kwargs):
        if self.counter%self.flip_every == 0:
            self.counter = 0
            self.steer *= -1
        return self.speed, self.steer


def main():
    """
    main entry point
    """

    work = {'mass': 3.463388126201571, 'lf': 0.15597534362552312, 'tlad': 0.82461887897713965, 'vgain': 1.375}#0.90338203837889}
    
    with open('./config_Straight_map.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)

    planner = PurePursuitPlanner(conf, (0.17145+0.15875)) #FlippyPlanner(speed=0.2, flip_every=1, steer=10)

    def render_callback(env_renderer):
        # custom extra drawing function

        e = env_renderer

        # update camera to follow car
        x = e.cars[0].vertices[::2]
        y = e.cars[0].vertices[1::2]
        top, bottom, left, right = max(y), min(y), min(x), max(x)
        e.score_label.x = left
        e.score_label.y = top - 700
        e.left = left - 800
        e.right = right + 800
        e.top = top + 800
        e.bottom = bottom - 800

        planner.render_waypoints(env_renderer)

    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=1, timestep=0.01, integrator=Integrator.RK4)
    env.add_render_callback(render_callback)
    
    obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))
    env.render()

    laptime = 0.0
    
    data_csv = ["t_s", "v_in", "v_out"]
    add_row(data_csv)

    start = time.time()

    planner.setStart(start)

    while not done:
        speed, steer = planner.plan(obs['poses_x'][0], obs['poses_y'][0], obs['poses_theta'][0], work['tlad'], work['vgain'])
        obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
        #add_row(data_csv)
        #print(obs)
        #print(time.time() - start)
        laptime += step_reward
        data_csv = [laptime, speed, obs['linear_vels_x'][0]]
        planner.setCurrent(laptime)
        add_row(data_csv)
        env.render(mode='human')

        # if((time.time() - start) >= 30):
        #     done = True
        
    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time()-start)

if __name__ == '__main__':
    main()
