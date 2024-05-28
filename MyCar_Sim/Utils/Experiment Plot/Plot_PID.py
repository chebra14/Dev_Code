import matplotlib.pyplot as plt 
import csv 
from pathlib import Path

# Base Class CSV (Output_Test):
# add_row([accl, self.state[7]*self.params_std.R_w, self.state[8]*self.params_std.R_w])

accl = []
  
f_ws = []
r_ws = []

t_s = []
time = 0
time_step = 0.01

path_root_test = Path(__file__).parent  /'..'/'..'/ 'Results/Output_Test.csv'
  

with open(path_root_test ,'r') as csvfile: 
    plots = csv.reader(csvfile, delimiter = ',') 
      
    for row in plots:
        t_s.append(time)
        time = time + time_step

        accl.append(float(row[0]))
        f_ws.append(float(row[1]))
        r_ws.append(float(row[2]))

plt.figure()
plt.plot(t_s, accl)
plt.plot(t_s, f_ws)
plt.plot(t_s, r_ws)

plt.xlabel('Time (s)') 
plt.ylabel('Speed (m/s) / Acceleration (m/s^2)') 
plt.title('PID Command and Wheel Speed (Kp_a = 4.0, Kp_b = 3.5)') 
plt.legend(["PID Command", "Front Wheel Speed", "Rear Wheel Speed"])
plt.xlim([0,time]) 
plt.show()