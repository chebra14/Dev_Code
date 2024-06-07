import matplotlib.pyplot as plt 
import csv 
from pathlib import Path
# import math
import numpy as np
from scipy.signal import lfilter
from scipy.interpolate import interp1d
from scipy.ndimage import gaussian_filter
import noisereduce as nr


def main():
    pose_raw = []
    t_s_1 = []

    wheel_speed = []
    expected_speed = []
    t_s_2 = []

    path_root_test = Path(__file__).parent  /'..'/'..'/ 'Results/First_Straight_Test/Results_Odom.csv'


    with open(path_root_test ,'r') as csvfile: 
        plots = csv.reader(csvfile, delimiter = ',') 
        
        for row in plots:

            if float(row[2]) > 0.0:
                wheel_speed.append(float(row[0]))
                expected_speed.append(float(row[1]))
                t_s_2.append(float(row[2]))

    path_root_laser = Path(__file__).parent  /'..'/'..'/ 'Results/First_Straight_Test/Results_Laser.csv'

    with open(path_root_laser ,'r') as csvfile2: 
        plots2 = csv.reader(csvfile2, delimiter = ',') 
        
        for row in plots2:

            if float(row[1]) > 0.0:
                pose_raw.append(float(row[0]))
                t_s_1.append(float(row[1]))

    #Normalize the Positions
    t_s_1 = np.array(t_s_1)
    start_pos = max(pose_raw)
    pose = -1.0 *( np.array(pose_raw) - start_pos )
    temp_counter = 0
    while pose[temp_counter] != 0:
        pose[temp_counter] = 0

        temp_counter +=1
    # Smoothen the Noise
    # p = np.polyfit(t_s_1, pose, 4)
    # polyfit_pose = t_s_1**4 * p[0] + t_s_1**3 * p[1] + t_s_1**2 * p[2] + t_s_1**2 * p[3] + p[4]
    # Smoothen the Noise
    # sigma = 5
    # smoothed_pose = gaussian_filter(pose, sigma)
    # Smoothen the Noise
    # n = 100
    # b = [1.0/n] * n
    # a = 1
    # yy = lfilter(b,a,pose)
    # Smoothen the Noise
    # lpf_step = np.zeros(np.size(pose))
    # gain1 = 0.5
    # for i in range(1, np.size(pose)):
    #     lpf_step[i] = lpf_step[i-1]*gain1 + (1-gain1)*(pose[i])
    # Smoothen the Noise
    # reduced_noise = nr.reduce_noise(y=pose, sr=t_s_1)
    #Plot to test
    # plt.figure()
    # # plt.plot(t_s_1, smoothed_pose)
    # # plt.plot(t_s_1, polyfit_pose)
    # # plt.plot(t_s_1, yy)
    # # plt.plot(t_s_1, lpf_step)
    # # plt.plot(t_s_1, reduced_noise)
    # plt.plot(t_s_1, pose)

    # plt.xlabel('Time (s)') 
    # plt.ylabel('Position (m)') 
    # plt.title('Straight Test - Position Measurements (Squat Fix)') 
    # plt.grid(visible=True)
    # plt.show()

    #Calculate Velocity of the car in terms of the original time
    # vel = np.gradient(smoothed_pose, t_s_1)
    vel = np.gradient(pose, t_s_1)
    #Hardcode Smoothening
    # np.savetxt(f"Results/First_Straight_Test/Vel.csv", vel, delimiter = ',')
    m1 = (vel[69] - vel[57]) / (69 - 57)
    c1 = vel[69] - m1*69
    # for i in range(57,70):
    #     print(m1*i + c1)
    m2 = (vel[103] - vel[100]) / (103 - 100)
    c2 = vel[103] - m2*103
    # for i in range(100,104):
    #     print(m2*i + c2)
    # m3 = (vel[59] - vel[54]) / (59 - 54)
    # c3 = vel[59] - m3*59
    # # for i in range(54,60):
    # #     print(m3*i + c3)
    # m4 = (vel[80] - vel[77]) / (80 - 77)
    # c4 = vel[80] - m4*80
    # # for i in range(77,81):
    # #     print(m4*i + c4)
    # m5 = (vel[99] - vel[96]) / (99 - 96)
    # c5 = vel[99] - m5*99
    # # for i in range(96,100):
    # #     print(m5*i + c5)
    # m6 = (vel[107] - vel[104]) / (107 - 104)
    # c6 = vel[107] - m6*107
    # # for i in range(104,108):
    # #     print(m6*i + c6)
    path_root_vel = Path(__file__).parent  /'..'/'..'/ 'Results/First_Straight_Test/Vel.csv'
    new_vel_1 = []
    with open(path_root_vel ,'r') as csvfile: 
        plots = csv.reader(csvfile, delimiter = ',') 
        
        for row in plots:
            new_vel_1.append(float(row[0]))
    new_vel_1 = np.array(new_vel_1)

    # Smoothen the Velocity
    smoothened_vel = smoothen(new_vel_1)
    #Interpolate the Velocity
    interpolate_vel = interp1d(t_s_1, smoothened_vel, kind= 'linear', fill_value= 'extrapolate')
    # interpolate_vel = interp1d(t_s_1, vel, kind= 'linear', fill_value= 'extrapolate')
    vel_interpolated = interpolate_vel(t_s_2)
    #Plot to test
    # plt.figure()
    # # plt.plot(t_s_1, smoothed_vel)
    # plt.plot(t_s_1, vel)
    # # plt.plot(t_s_1, smoothened_vel)
    # plt.plot(t_s_2, vel_interpolated)

    # plt.xlabel('Time (s)') 
    # plt.ylabel('Speed (m/s)') 
    # plt.title('Straight Test - Velocity Measurement') 
    # plt.legend(["Raw Derivative", "Filtered Values"])
    # plt.grid(visible=True)
    # plt.show()

    #Calculate the Longitudinal Slip Velocity
    slip = wheel_speed - vel_interpolated

    plt.figure()
    plt.plot(t_s_2, wheel_speed)
    plt.plot(t_s_2, expected_speed)
    plt.plot(t_s_2, vel_interpolated)
    plt.plot(t_s_2, slip)
    # plt.plot(t_s_1, pose)

    plt.xlabel('Time (s)') 
    plt.ylabel('Speed (m/s)') 
    plt.title('Straight Test - Result') 
    plt.legend(["Wheel Speed", "Expected Speed", "Actual Speed", "Slip"])#, "Actual Speed", "Position (m)"])
    plt.xlim([0,6])
    # plt.ylim([-3, 10]) 
    plt.grid(visible=True)
    plt.show()

def smoothen(vel):
    #Step 1
    sigma = 1
    smoothed_vel = gaussian_filter(vel, sigma)
    lpf_step = np.zeros(np.size(vel))
    lpf_step2 = np.zeros(np.size(vel))
    lpf_step3 = np.zeros(np.size(vel))
    lpf_step4 = np.zeros(np.size(vel))
    gain1 = 0.1
    for i in range(1, np.size(vel)):
        lpf_step[i] = lpf_step[i-1]*gain1 + (1-gain1)*(smoothed_vel[i])
    for i in range(1, np.size(vel)):
        lpf_step2[i] = lpf_step2[i-1]*gain1 + (1-gain1)*(lpf_step[i])
    for i in range(1, np.size(vel)):
        lpf_step3[i] = lpf_step3[i-1]*gain1 + (1-gain1)*(lpf_step2[i])
    for i in range(1, np.size(vel)):
        lpf_step4[i] = lpf_step4[i-1]*gain1 + (1-gain1)*(lpf_step3[i])

    #Step 2
    smoothed_vel2 = gaussian_filter(lpf_step4, sigma)
    lpf_step5 = np.zeros(np.size(vel))
    lpf_step6 = np.zeros(np.size(vel))
    lpf_step7 = np.zeros(np.size(vel))
    lpf_step8 = np.zeros(np.size(vel))
    gain2 = 0.1
    for i in range(1, np.size(vel)):
        lpf_step5[i] = lpf_step5[i-1]*gain2 + (1-gain2)*(smoothed_vel2[i])
    for i in range(1, np.size(vel)):
        lpf_step6[i] = lpf_step6[i-1]*gain2 + (1-gain2)*(lpf_step5[i])
    for i in range(1, np.size(vel)):
        lpf_step7[i] = lpf_step7[i-1]*gain2 + (1-gain2)*(lpf_step6[i])
    for i in range(1, np.size(vel)):
        lpf_step8[i] = lpf_step8[i-1]*gain2 + (1-gain2)*(lpf_step7[i])

    #Step 3
    smoothed_vel3 = gaussian_filter(lpf_step8, sigma)
    lpf_step9 = np.zeros(np.size(vel))
    lpf_step10 = np.zeros(np.size(vel))
    lpf_step11 = np.zeros(np.size(vel))
    lpf_step12 = np.zeros(np.size(vel))
    gain3 = 0.1
    for i in range(1, np.size(vel)):
        lpf_step9[i] = lpf_step9[i-1]*gain3 + (1-gain3)*(smoothed_vel3[i])
    for i in range(1, np.size(vel)):
        lpf_step10[i] = lpf_step10[i-1]*gain3 + (1-gain3)*(lpf_step9[i])
    for i in range(1, np.size(vel)):
        lpf_step11[i] = lpf_step11[i-1]*gain3 + (1-gain3)*(lpf_step10[i])
    for i in range(1, np.size(vel)):
        lpf_step12[i] = lpf_step12[i-1]*gain3 + (1-gain3)*(lpf_step11[i])

    #Step 4
    smoothed_vel4 = gaussian_filter(lpf_step12, sigma)
    lpf_step13 = np.zeros(np.size(vel))
    lpf_step14 = np.zeros(np.size(vel))
    lpf_step15 = np.zeros(np.size(vel))
    lpf_step16 = np.zeros(np.size(vel))
    gain4 = 0.1
    for i in range(1, np.size(vel)):
        lpf_step13[i] = lpf_step13[i-1]*gain4 + (1-gain4)*(smoothed_vel4[i])
    for i in range(1, np.size(vel)):
        lpf_step14[i] = lpf_step14[i-1]*gain4 + (1-gain4)*(lpf_step13[i])
    for i in range(1, np.size(vel)):
        lpf_step15[i] = lpf_step15[i-1]*gain4 + (1-gain4)*(lpf_step14[i])
    for i in range(1, np.size(vel)):
        lpf_step16[i] = lpf_step16[i-1]*gain4 + (1-gain4)*(lpf_step15[i])

    return lpf_step16

if __name__ == '__main__':
    main()