import matplotlib.pyplot as plt 
import csv 
from pathlib import Path
import numpy as np
from scipy.integrate import cumtrapz
import math

time_N = []
accel_N_x = []
accel_N_y = []
gyro_N_z = []
r_ws_N = []
l_ws_N = []
yaw_model_N = []

threshold = 0.6

# time_S = []
# accel_S_x = []
# accel_S_y = []
# accel_S_z = []
# gyro_S_x = []
# gyro_S_y = []
# gyro_S_z = []
# r_ws_S = []
# l_ws_S = []

path_root_N = Path(__file__).parent  / 'Data/No Slip/BV_N_Test4.csv'
  
with open(path_root_N ,'r') as csvfile1: 
    plots1 = csv.reader(csvfile1, delimiter = ',') 
      
    for row in plots1: 
            time_N.append(float(row[0]))
            accel_N_y.append(float(row[1]))
            accel_N_x.append(float(row[2]))
            gyro_N_z.append(float(row[3]))
            r_ws_N.append(float(row[4]))
            l_ws_N.append(float(row[5]))
            yaw_model_N.append(float(row[6]))


# path_root_S = Path(__file__).parent  / 'Data/Slip/BV_Slip_Test2.csv'
  
# with open(path_root_S ,'r') as csvfile1: 
#     plots1 = csv.reader(csvfile1, delimiter = ',') 
      
#     for row in plots1: 
#         time_S.append(float(row[0]))

#         accel_S_x.append(float(row[1]))
#         accel_S_y.append(float(row[2]))
#         accel_S_z.append(float(row[3]))

#         gyro_S_x.append(float(row[4]))
#         gyro_S_y.append(float(row[5]))
#         gyro_S_z.append(float(row[6]))

#         r_ws_N.append(float(row[7]))
#         l_ws_N.append(float(row[8]))

# Normalise Time
t_s_N = np.array(time_N)
t_s_N = t_s_N/1000
t_s_N = t_s_N - t_s_N[0]

# t_s_S = np.array(time_S)
# t_s_S = t_s_S/1000
# t_s_S = t_s_S - t_s_S[0]

maxTime_N = round(t_s_N[len(t_s_N)-1])
# maxTime_S = round(t_s_S[len(t_s_S)-1])

v_x = np.array(cumtrapz(accel_N_x, t_s_N))
yaw_rate = np.array(gyro_N_z)
# yaw = cumtrapz(yaw_rate, t_s_N) * math.pi

a_centr = v_x * yaw_rate[1::]

temp_a_y = np.array(accel_N_y[1::] )
v_y = np.array(cumtrapz(temp_a_y, t_s_N[1::]))

v_x = yaw_rate * 0.065

fig, (Accel, WS) = plt.subplots(2)
Accel.plot(t_s_N, v_x)
# Accel.plot(t_s_N, accel_N_y)
# Accel.plot(t_s_N[1::], a_centr)
Accel.set_xlabel('Time (s)') 
Accel.set_ylabel('Speed (m/s)') 
Accel.set_title('Speed') 
Accel.legend(["X", "Y"])
Accel.set_xlim([0, maxTime_N])
Accel.grid(visible=True)

WS.plot(t_s_N, l_ws_N)
WS.plot(t_s_N, r_ws_N)
WS.set_xlabel('Time (s)') 
WS.set_ylabel('Wheel Speed') 
WS.set_title('Wheel Speed') 
WS.legend(["Left", "Right"])
WS.set_xlim([0, maxTime_N])
WS.grid(visible=True)

plt.subplots_adjust(hspace=0.3)
    

# plt.plot(t_s_S, accel_S_x)
# plt.plot(t_s_S, accel_S_y)
# plt.plot(t_s_S, accel_S_z)
# plt.set_xlabel('Time (s)') 
# plt.set_ylabel('Acceleration (m/s^2)') 
# plt.set_title('Baby Voyager IMU Test (Slip)') 
# plt.legend(["X", "Y", "Z"])
# plt.set_xlim([0, maxTime_S])
# plt.grid(visible=True) 

plt.show()

# fig2, (NSlip_gyro, w_speed) = plt.subplots(2)

# NSlip_gyro.plot(t_s_N, yaw_model_N)
# NSlip_gyro.plot(t_s_N, gyro_N_z)
# NSlip_gyro.set_xlabel('Time (s)') 
# NSlip_gyro.set_ylabel('Yaw (raw/s)') 
# NSlip_gyro.set_title('Baby Voyager YAW Test (Model vs Real)') 
# NSlip_gyro.legend(["Yaw Model", "Yaw Gyro"])
# NSlip_gyro.set_xlim([0, maxTime_N])
# NSlip_gyro.grid(visible=True) 

# w_speed.plot(t_s_N, l_ws_N)
# w_speed.plot(t_s_N, r_ws_N)
# w_speed.set_xlabel('Time (s)') 
# w_speed.set_ylabel('Pulse per 250ms') 
# w_speed.set_title('Baby Voyager YAW Test (Wheel Speed)') 
# w_speed.legend(["Left Wheel", "Right Wheel"])
# w_speed.set_xlim([0, maxTime_N])
# w_speed.grid(visible=True) 

# plt.show()