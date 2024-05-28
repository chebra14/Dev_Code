import time
from f110_gym.envs.base_classes import Integrator
from f110_gym.envs.f110_env import F110Env
from f110_gym.utils.load_parameters import readYaml
import yaml
import gym
import numpy as np
from argparse import Namespace
from Planners.PurePursuitPlanner import PurePursuitPlanner

from numba import njit

from pyglet.gl import GL_POINTS

import math
from pathlib import Path
import csv

Output_to = 'Results/Output.csv'


with open(Output_to, mode='w', newline='') as file:
    writer = csv.writer(file)

def add_row(csv_data):
    with open(Output_to, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(csv_data)


def main():
    """
    main entry point
    """

    work = {'mass': 3.463388126201571, 'lf': 0.15597534362552312, 'tlad': 0.82461887897713965, 'vgain': 1}#1.375}#0.90338203837889}
    
    with open('./Sim_Config/Track_Parameters/Straight/config_Straight_map.yaml') as file:
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
    # env.render()

    laptime = 0.0
    
    #Create baseline to match with Output_test, caused byt env.reset()
    add_row([0, 0])

    start = time.time()

    planner.setStart(start)

    #Get sim config settings
    run_config_path = Path(__file__).parent/'Run_Scripts/run_config.yaml'
    run_config = readYaml(run_config_path)
    planner.setRunConfig(run_config)


    stop_Time = 0

    while not done:

        speed, steer = planner.plan(obs['poses_x'][0], obs['poses_y'][0], obs['poses_theta'][0], work['tlad'], work['vgain'])

        data_csv = [speed, steer]
        add_row(data_csv)

        obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
        laptime += step_reward

        planner.setCurrent(laptime)

        # env.render(mode='human')

        if speed == 0:
            stop_Time = stop_Time + step_reward

        if stop_Time >= 5:
            done = True
        
    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time()-start)

if __name__ == '__main__':
    main()
