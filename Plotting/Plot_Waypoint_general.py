import matplotlib.pyplot as plt 
import csv 
from pathlib import Path

accl = []
f_ws = []
r_ws = []

t_s = []
time = 0
time_step = 0.01

path_root_test = Path(__file__).parent  /'..'/'..'/ 'Results/Output_Test.csv'
  
with open(path_root_test ,'r') as csvfile1: 
    plots1 = csv.reader(csvfile1, delimiter = ',') 
      
    for row in plots1: 
        t_s.append(float(time))
        time = time + time_step

        accl.append(float(row[0]))
        f_ws.append(float(row[1]))
        r_ws.append(float(row[2]))


plt.plot(t_s, accl)
plt.plot(t_s, f_ws)
plt.plot(t_s, r_ws)

plt.xlabel('Time (s)') 
plt.ylabel('Speed (m/s) / Acceleration (m/s^2)') 
plt.title('Start and Stop Straight Test (No LPF)') 
# plt.legend(["Front Wheel Speed", "Rear Wheel Speed", "PID Command"])
plt.legend(["Acceleration", "Front Wheel Speed", "Rear Wheel Speed"])
plt.xlim([0,time]) 
plt.show() 
