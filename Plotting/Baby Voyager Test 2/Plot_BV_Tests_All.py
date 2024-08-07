import matplotlib.pyplot as plt 
import csv 
from pathlib import Path
import numpy as np

time_N = []
accel_N_x = []
accel_N_y = []
gyro_N_z = []
r_ws_N = []
l_ws_N = []
yaw_model_N = []

# time_S = []
# accel_S_x = []
# accel_S_y = []
# accel_S_z = []
# gyro_S_x = []
# gyro_S_y = []
# gyro_S_z = []
# r_ws_S = []
# l_ws_S = []

path_root_N = Path(__file__).parent  / 'Data/Slip/BV_S_Test1.csv'
  
with open(path_root_N ,'r') as csvfile1: 
    plots1 = csv.reader(csvfile1, delimiter = ',') 
      
    for row in plots1: 
            time_N.append(float(row[0]))
            accel_N_x.append(float(row[1]))
            accel_N_y.append(float(row[2]))
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

temp_yaw_m = np.array(yaw_model_N)
temp_yaw_r = np.array(gyro_N_z)

up_yaw = temp_yaw_m + 0.5
low_yaw = temp_yaw_m - 0.5

# for i in range(67):

#     print(t_s_N[i], (abs(temp_yaw_m[i] - temp_yaw_r[i])))

plt.plot(t_s_N, gyro_N_z)
plt.plot(t_s_N, up_yaw)
plt.plot(t_s_N, low_yaw)
plt.xlabel('Time (s)') 
plt.ylabel('Yaw rate (rad/s)') 
plt.title('Baby Voyager YAW Test (Pushing the Car Around)') 
plt.legend(["Yaw Gyro", "Upper Limit", "Lower Limit"])
plt.xlim([0, maxTime_N])
plt.grid(visible=True) 

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