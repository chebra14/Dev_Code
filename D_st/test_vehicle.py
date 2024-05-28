"""
Functions for testing different vehicle models (see examples in chap. 11 of documentation)
"""

from scipy.integrate import odeint
import numpy
import matplotlib.pyplot as plt
from matplotlib.pyplot import title, legend
# import math

from vehiclemodels.vehicle_parameters import setup_vehicle_parameters
from vehiclemodels.init_std import init_std
from vehiclemodels.vehicle_dynamics_std import vehicle_dynamics_std

def func_STD(x, t, u, p):
    f = vehicle_dynamics_std(x, u, p)
    return f


# load parameters
p = setup_vehicle_parameters(vehicle_id=2)
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
