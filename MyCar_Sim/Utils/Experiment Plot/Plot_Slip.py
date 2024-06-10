import matplotlib.pyplot as plt 
import csv 
from pathlib import Path
import numpy as np

ref_speed = []

# Base Class CSV (Output_Test):
# add_row([self.state[7]*self.params_std.R_w - self.state[3], self.state[8]*self.params_std.R_w - self.state[3]])
  
f_wslip = []
r_wslip = []

avg_slip = []

t_s = []
time = 0
time_step = 0.01

path_root_test = Path(__file__).parent  /'..'/'..'/ 'Results/Output_Test.csv'
path_root = Path(__file__).parent  /'..'/'..'/ 'Results/Output.csv'
  
with open(path_root_test ,'r') as csvfile1: 
    plots1 = csv.reader(csvfile1, delimiter = ',') 
      
    for row in plots1: 
        t_s.append(float(time))
        time = time + time_step

        f_wslip.append(float(row[0]))
        r_wslip.append(float(row[1]))

with open(path_root ,'r') as csvfile2: 
    plots2 = csv.reader(csvfile2, delimiter = ',') 
      
    for row in plots2: 
        ref_speed.append(float(row[0]))

for i in range(0, np.size(t_s)):
    avg_slip.append((f_wslip[i] + r_wslip[i])/2)


# plt.figure()
# plt.plot(t_s, ref_speed)
# plt.plot(t_s, f_wslip)

# plt.xlabel('Time (s)') 
# plt.ylabel('Speed (m/s)') 
# plt.title('Front Wheel Slip') 
# # plt.legend(["Front Wheel Speed", "Rear Wheel Speed", "PID Command"])
# plt.legend(["Reference", "Wheelslip"])
# plt.xlim([0,time]) 
# plt.show() 

# plt.figure()
# plt.plot(t_s, ref_speed)
# plt.plot(t_s, r_wslip)

# plt.xlabel('Time (s)') 
# plt.ylabel('Speed (m/s)') 
# plt.title('Rear Wheel Slip') 
# # plt.legend(["Front Wheel Speed", "Rear Wheel Speed", "PID Command"])
# plt.legend(["Reference", "Wheelslip"])
# plt.xlim([0,time]) 
# plt.show()

t_s_real = []
slip_real = []
expected_speed = []

path_root = Path(__file__).parent  /'..'/'..'/ 'Results/Fourth_Straight_Test/Wheel_Slip.csv'

with open(path_root ,'r') as csvfile1: 
    plots1 = csv.reader(csvfile1, delimiter = ',') 
      
    for row in plots1: 
        t_s_real.append(float(row[0]))
        slip_real.append(float(row[1]))
        expected_speed.append(float(row[2]))

# plt.figure()
# plt.plot(t_s, ref_speed)
# plt.plot(t_s, avg_slip)

# plt.xlabel('Time (s)') 
# plt.ylabel('Speed (m/s)') 
# plt.title('Straight Test - Simulation Wheelslip Result') 
# plt.legend(["Reference", "Wheel-slip (Actual Speed - Wheel Speed)"])
# plt.xlim([0,7])
# plt.ylim([-4, 5]) 
# plt.grid(visible=True)
# plt.show() 

# plt.figure()
# plt.plot(t_s_real, expected_speed)
# plt.plot(t_s_real, slip_real)

# plt.xlabel('Time (s)') 
# plt.ylabel('Speed (m/s)') 
# plt.title('Straight Test - Real Car Wheelslip Result') 
# plt.legend(["Reference", "Wheel-slip (Actual Speed - Wheel Speed)"])
# plt.xlim([0,7])
# plt.ylim([-4, 5]) 
# plt.grid(visible=True)
# plt.show()

fig, axs = plt.subplots(2)

axs[0].plot(t_s, ref_speed)
axs[0].plot(t_s, avg_slip)
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Speed (m/s)') 
axs[0].set_title('Straight Test - Simulation Wheelslip Result') 
axs[0].legend(["Reference", "Wheel-slip (Actual Speed - Wheel Speed)"])
axs[0].set_xlim([0,5])
axs[0].set_ylim([-4, 5]) 
axs[0].grid(visible=True)

axs[1].plot(t_s_real, expected_speed)
axs[1].plot(t_s_real, slip_real)
axs[1].set_xlabel('Time (s)') 
axs[1].set_ylabel('Speed (m/s)') 
axs[1].set_title('Straight Test - Real Car Wheelslip Result') 
axs[1].legend(["Reference", "Wheel-slip (Actual Speed - Wheel Speed)"])
axs[1].set_xlim([0,5])
axs[1].set_ylim([-4, 5]) 
axs[1].grid(visible=True)

plt.show()