import time
from f110_gym.envs.base_classes import Integrator
from f110_gym.envs.f110_env import F110Env
from f110_gym.utils.load_parameters import readYaml
import yaml
import gym
import numpy as np
from argparse import Namespace
from Utils import PlannerHelpers_1

from numba import njit

from pyglet.gl import GL_POINTS

from Run_Scripts.straight_run import find_Speed

import math
from pathlib import Path
import csv

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
        self.drive_Mode = 3
        self.straight_Mode = 4
        self.flagTime = 0
        self.setSpeed_raw = 8

        self.start = 0
        self.currentTime = 0
        self.start_pos = np.array([0.0, 0.0])
        self.minSpeed = 2.5
        self.maxSpeed = 10
        self.flagSpeed = 0
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
        nearest_point, nearest_dist, t, i = PlannerHelpers_1.nearest_point_on_trajectory(position, wpts)

        #speed

        setSpeed, self.flagSpeed = find_Speed(self.drive_Mode, self.lookahead_data, i, x_temp, y_temp, self.flagSpeed, self.flagSpeedMax, self.flagTime, self.minSpeed, self.maxSpeed, self.setSpeed_raw, self.currentTime, self.straight_Mode)

        if nearest_dist < lookahead_distance:
            lookahead_point, i2, t2 = PlannerHelpers_1.first_point_on_trajectory_intersecting_circle(position, lookahead_distance, wpts, i+t, wrap=True)
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

        speed, steering_angle = PlannerHelpers_1.get_actuation(pose_theta, lookahead_point, position, lookahead_distance, self.wheelbase)
        speed = vgain * speed

        if self.drive_Mode == 3:
            steering_angle = -0.4

        return speed, steering_angle
    
    def setStart(self, startTime):
        self.start = startTime

    def setCurrent(self, currentTime):
        self.currentTime = currentTime

    def setRunConfig(self, runConfig):
        self.drive_Mode = runConfig['drive_Mode']
        self.straight_Mode = runConfig['straight_Mode']
        self.setSpeed_raw = runConfig['setSpeed_raw']

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