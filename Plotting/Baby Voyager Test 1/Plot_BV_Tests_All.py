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

path_root_N = Path(__file__).parent  / 'Data/No Slip/BV_NSlip_Test1.csv'
  
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

path_root_S = Path(__file__).parent  / 'Data/Slip/BV_Slip_Test2.csv'
  
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
t_s_N = t_s_N - t_s_N[0]

t_s_S = np.array(time_S)
t_s_S = t_s_S/1000
t_s_S = t_s_S - t_s_S[0]

maxTime_N = round(t_s_N[len(t_s_N)-1])
maxTime_S = round(t_s_S[len(t_s_S)-1])

# plt.plot(t_s_N, accel_N_x)
# plt.plot(t_s_N, accel_N_y)
# plt.plot(t_s_N, accel_N_z)

fig1, (NSlip_accl, Slip_accl) = plt.subplots(2)

NSlip_accl.plot(t_s_N, accel_N_x)
NSlip_accl.plot(t_s_N, accel_N_y)
NSlip_accl.plot(t_s_N, accel_N_z)
NSlip_accl.set_xlabel('Time (s)') 
NSlip_accl.set_ylabel('Acceleration (m/s^2)') 
NSlip_accl.set_title('Baby Voyager IMU Test (No Slip)') 
NSlip_accl.legend(["X", "Y", "Z"])
NSlip_accl.set_xlim([0, maxTime_N])
NSlip_accl.grid(visible=True) 

Slip_accl.plot(t_s_S, accel_S_x)
Slip_accl.plot(t_s_S, accel_S_y)
Slip_accl.plot(t_s_S, accel_S_z)
Slip_accl.set_xlabel('Time (s)') 
Slip_accl.set_ylabel('Acceleration (m/s^2)') 
Slip_accl.set_title('Baby Voyager IMU Test (Slip)') 
Slip_accl.legend(["X", "Y", "Z"])
Slip_accl.set_xlim([0, maxTime_S])
Slip_accl.grid(visible=True) 

plt.show()

fig2, (NSlip_gyro, Slip_gyro) = plt.subplots(2)

NSlip_gyro.plot(t_s_N, gyro_N_x)
NSlip_gyro.plot(t_s_N, gyro_N_y)
NSlip_gyro.plot(t_s_N, gyro_N_z)
NSlip_gyro.set_xlabel('Time (s)') 
NSlip_gyro.set_ylabel('Yaw (m/s^2)') 
NSlip_gyro.set_title('Baby Voyager IMU Test (No Slip)') 
NSlip_gyro.legend(["X", "Y", "Z"])
NSlip_gyro.set_xlim([0, maxTime_N])
NSlip_gyro.grid(visible=True) 

Slip_gyro.plot(t_s_S, gyro_S_x)
Slip_gyro.plot(t_s_S, gyro_S_y)
Slip_gyro.plot(t_s_S, gyro_S_z)
Slip_gyro.set_xlabel('Time (s)') 
Slip_gyro.set_ylabel('Yaw (m/s^2)') 
Slip_gyro.set_title('Baby Voyager IMU Test (Slip)') 
Slip_gyro.legend(["X", "Y", "Z"])
Slip_gyro.set_xlim([0, maxTime_S])
Slip_gyro.grid(visible=True) 

plt.show()