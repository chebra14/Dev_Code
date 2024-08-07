import matplotlib.pyplot as plt 
import csv 
from pathlib import Path
import numpy as np

time_N = []
accel_N_x = []
accel_N_y = []
accel_N_z = []
gyro_N_x = []
gyro_N_y = []
gyro_N_z = []
r_ws_N = []
l_ws_N = []

time_S = []
accel_S_x = []
accel_S_y = []
accel_S_z = []
gyro_S_x = []
gyro_S_y = []
gyro_S_z = []
r_ws_S = []
l_ws_S = []

path_root_N = Path(__file__).parent  / 'Data/No Slip/BV_NSlip_Test3.csv'
  
with open(path_root_N ,'r') as csvfile1: 
    plots1 = csv.reader(csvfile1, delimiter = ',') 
      
    for row in plots1: 
        time_N.append(float(row[0]))

        accel_N_x.append(float(row[1]))
        accel_N_y.append(float(row[2]))
        accel_N_z.append(float(row[3]))

        gyro_N_x.append(float(row[4]))
        gyro_N_y.append(float(row[5]))
        gyro_N_z.append(float(row[6]))

        r_ws_N.append(float(row[7]))
        l_ws_N.append(float(row[8]))

path_root_S = Path(__file__).parent  / 'Data/Slip/BV_Slip_Test4.csv'
  
with open(path_root_S ,'r') as csvfile1: 
    plots1 = csv.reader(csvfile1, delimiter = ',') 
      
    for row in plots1: 
        time_S.append(float(row[0]))

        accel_S_x.append(float(row[1]))
        accel_S_y.append(float(row[2]))
        accel_S_z.append(float(row[3]))

        gyro_S_x.append(float(row[4]))
        gyro_S_y.append(float(row[5]))
        gyro_S_z.append(float(row[6]))

        r_ws_N.append(float(row[7]))
        l_ws_N.append(float(row[8]))

# Normalise Time
t_s_N = np.array(time_N)
t_s_N = t_s_N/1000
# t_s_N = t_s_N - t_s_N[0]

t_s_S = np.array(time_S)
t_s_S = t_s_S/1000
# t_s_S = t_s_S - t_s_S[0]

minTime = min(t_s_N[0], t_s_S[0])
t_s_N = t_s_N - minTime
t_s_S = t_s_S - minTime
maxTime_N = round(t_s_N[len(t_s_N)-1])
maxTime_S = round(t_s_S[len(t_s_S)-1])
maxTime = max(maxTime_N, maxTime_S)

N_accel = (np.array(accel_N_x) ** 2 + np.array(accel_N_y) ** 2) ** 0.5
S_accel = (np.array(accel_S_x) ** 2 + np.array(accel_S_y) ** 2) ** 0.5

# plt.plot(t_s_N, accel_N_x)
# plt.plot(t_s_N, accel_N_y)
# plt.plot(t_s_N, accel_N_z)

plt.figure()
plt.plot(t_s_N, N_accel)
plt.plot(t_s_S, S_accel)
plt.xlabel('Time (s)') 
plt.ylabel('Acceleration (m/s^2)') 
plt.title('Baby Voyager IMU Test (Acceleration)') 
plt.legend(["No Slip", "Slip"])
plt.xlim([0, maxTime])
plt.ylim([0, 8])
plt.grid(visible=True) 

plt.show()

plt.figure()

plt.plot(t_s_N, gyro_N_z)
plt.plot(t_s_S, gyro_S_z)
plt.xlabel('Time (s)') 
plt.ylabel('Yaw (m/s^2)') 
plt.title('Baby Voyager IMU Test (Gyro)') 
plt.legend(["No Slip", "Slip"])
plt.xlim([0, maxTime])
plt.ylim([-5.5, 5])
plt.grid(visible=True) 

plt.show()