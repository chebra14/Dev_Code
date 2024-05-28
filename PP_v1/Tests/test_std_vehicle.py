"""
Functions for testing different vehicle models (see examples in chap. 11 of documentation)
"""

from scipy.integrate import odeint
import numpy
import matplotlib.pyplot as plt
from matplotlib.pyplot import title, legend
import math
from pathlib import Path

from Utils.vehicle_parameters import setup_vehicle_parameters
from f110_gym.envs.dynamic_models import vehicle_dynamics_std

def func_STD(x, t, u, p):
    f = vehicle_dynamics_std(x, u, p)
    return f

def init_std(init_state, p):
    """
    init_std generates the initial state vector for the drift single track model

    Syntax:
        x0 = init_std(init_state, p)

    Inputs:
        :param init_state: core initial states
        :param p: parameter vector

    Outputs:
        :return x0: initial state vector

    Author:         Gerald Würsching
    Written:        23-October-2020
    Last update:    23-October-2020
    Last revision:  ---
    """

    # ------------- BEGIN CODE --------------
    # states
    # x1 = x-position in a global coordinate system
    # x2 = y-position in a global coordinate system
    # x3 = steering angle of front wheels
    # x4 = velocity at vehicle center
    # x5 = yaw angle
    # x6 = yaw rate
    # x7 = slip angle at vehicle center
    # x8 = front wheel angular speed
    # x9 = rear wheel angular speed

    # create initial state vector
    x0 = init_state.copy()
    x0.append(x0[3]*math.cos(x0[6])*math.cos(x0[2])/p.R_w)  # init front wheel angular speed
    x0.append(x0[3]*math.cos(x0[6])/p.R_w)  # init rear wheel angular speed

    return x0


# load parameters
parameters_path = Path(__file__).parent / 'Vehicle_Parameters'
p = setup_vehicle_parameters(vehicle_id=2, dir_params = parameters_path)
g = 9.81  # [m/s^2]

# set options --------------------------------------------------------------
tStart = 0  # start time
tFinal = 1  # start time

delta0 = 0
vel0 = 15
Psi0 = 0
dotPsi0 = 0
beta0 = 0
sy0 = 0
initialState = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]  # initial state for simulation
x0_STD = init_std(initialState, p)  # initial state for single-track drift model
# --------------------------------------------------------------------------

def oversteer_understeer_STD():
    t = numpy.arange(0, tFinal, 0.01)
    v_delta = 0.15

    # coasting
    u = [v_delta, 0]
    x_coast = odeint(func_STD, x0_STD, t, args=(u, p))

    # braking
    u = [v_delta, -0.75 * g]
    x_brake = odeint(func_STD, x0_STD, t, args=(u, p))

    # accelerating
    u = [v_delta, 0.63 * g]
    x_acc = odeint(func_STD, x0_STD, t, args=(u, p))

    # position
    title('position comparison STD')
    plt.plot([tmp[0] for tmp in x_coast], [tmp[1] for tmp in x_coast])
    plt.plot([tmp[0] for tmp in x_brake], [tmp[1] for tmp in x_brake])
    plt.plot([tmp[0] for tmp in x_acc], [tmp[1] for tmp in x_acc])
    legend(['coasting', 'braking', 'accelerating'])
    plt.show()
    # compare slip angles
    title('slip angle comparison STD')
    plt.plot(t, [tmp[6] for tmp in x_coast])
    plt.plot(t, [tmp[6] for tmp in x_brake])
    plt.plot(t, [tmp[6] for tmp in x_acc])
    legend(['coasting', 'braking', 'accelerating'])
    plt.show()
    # orientation
    title('orientation comparison STD')
    plt.plot(t, [tmp[4] for tmp in x_coast])
    plt.plot(t, [tmp[4] for tmp in x_brake])
    plt.plot(t, [tmp[4] for tmp in x_acc])
    legend(['coasting', 'braking', 'accelerating'])
    plt.show()


# run simulations *****************
if __name__ == '__main__':
    oversteer_understeer_STD()
    
    # print("Hello")
    # print(p.a)
