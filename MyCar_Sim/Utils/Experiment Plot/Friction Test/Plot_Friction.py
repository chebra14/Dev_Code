import matplotlib.pyplot as plt 
import csv 
from pathlib import Path

Fs = []

t_s = []
time = 0
time_step = 0.1

m = 2.74
g = 9.81

path_root = Path(__file__).parent  / 'output'

with open(path_root ,'r') as csvfile2: 
    plots2 = csv.reader(csvfile2, delimiter = ',') 
      
    for row in plots2: 
        t_s.append(time)
        time = time + time_step

        Fs.append(float(row[0]))

mu = max(Fs) / ((m*g) / 4)

print("Estimated mu: (lateral)", mu)


plt.figure()
plt.plot(t_s, Fs)

plt.xlabel('Time (s)') 
plt.ylabel('Force (N)') 
plt.title('Push Test of Car on Lab Floor') 
plt.xlim([0,time]) 
plt.show() 
