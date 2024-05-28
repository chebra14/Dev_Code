import matplotlib.pyplot as plt 
import csv 
from pathlib import Path

ref_speed = []

# Base Class CSV (Output_Test):
# add_row([self.state[7]*self.params_std.R_w - self.state[3], self.state[8]*self.params_std.R_w - self.state[3]])
  
f_wslip = []
r_wslip = []

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


plt.figure()
plt.plot(t_s, ref_speed)
plt.plot(t_s, f_wslip)

plt.xlabel('Time (s)') 
plt.ylabel('Speed (m/s)') 
plt.title('Front Wheel Slip') 
# plt.legend(["Front Wheel Speed", "Rear Wheel Speed", "PID Command"])
plt.legend(["Reference", "Wheelslip"])
plt.xlim([0,time]) 
plt.show() 

plt.figure()
plt.plot(t_s, ref_speed)
plt.plot(t_s, r_wslip)

plt.xlabel('Time (s)') 
plt.ylabel('Speed (m/s)') 
plt.title('Rear Wheel Slip') 
# plt.legend(["Front Wheel Speed", "Rear Wheel Speed", "PID Command"])
plt.legend(["Reference", "Wheelslip"])
plt.xlim([0,time]) 
plt.show() 
