import math

def find_Speed(drive_Mode, lookahead_data, i, x_temp, y_temp, flagSpeed, flagSpeedMax, flagTime, minSpeed, maxSpeed, setSpeed_raw, currentTime, straight_Mode):

    match drive_Mode:
            case 1:

                one_test = i+lookahead_data
                two_test = i+lookahead_data+1

                if one_test >= len(x_temp):
                    one_test = len(x_temp) - one_test

                if two_test >= len(x_temp):
                    two_test = len(x_temp) - two_test

                x_1 = x_temp[one_test]
                x_2 = x_temp[two_test]

                y_1 = y_temp[one_test]
                y_2 = y_temp[two_test]

                z_1 = math.sqrt(x_1**2 + y_1**2)
                z_2 = math.sqrt(x_2**2 + y_2**2)

                x_1_t = x_temp[i]
                x_2_t = x_temp[i+1]

                y_1_t = y_temp[i]
                y_2_t = y_temp[i+1]

                z_1_t = math.sqrt(x_1_t**2 + y_1_t**2)
                z_2_t = math.sqrt(x_2_t**2 + y_2_t**2)

                flagSpeed, setSpeed = calcSpeed(flagSpeed, flagSpeedMax, z_2, z_1, minSpeed, maxSpeed)
            case 2:

                setSpeed_raw, flagTime = straight_Test(straight_Mode, currentTime, setSpeed_raw, flagTime)

                setSpeed  = setSpeed_raw
            
            case 3:
                setSpeed = donut(setSpeed_raw)
            case 4:
                setSpeed = setSpeed_raw
    
    return setSpeed, flagSpeed

def calcSpeed(flag, flagMax, z_2, z_1, minSpeed, maxSpeed):
        speed = minSpeed
        traj_speed_test = 0

        test_flag = (flag == flagMax)

        if test_flag:
            traj_speed_test = abs((z_2 - z_1)*100)

            if traj_speed_test < 10:
                flag = 1
                speed = minSpeed
            else:
                speed = maxSpeed
        else:
            flag = flag + 1
            speed = minSpeed
        return flag, speed

def straight_Test(straight_Mode, currentTime, speed, flagTime):

        match straight_Mode:

            case 1:

                if(currentTime >= 2):
                    speed = 0

            case 2:

                if(currentTime >= 5 and currentTime < 8):
                    speed = 4
                elif(currentTime >= 8 and currentTime < 10):
                    speed = 0
                elif(currentTime >= 10 and currentTime < 13):
                    speed = 5
                elif(currentTime >= 13 and currentTime < 20):
                    speed = 8

            case 3:

                if(currentTime >= 2 and currentTime < 2.5):
                    speed = 0
                elif(currentTime >= 2.5 and currentTime < 2.8):
                    speed = 8
                elif(currentTime >= 2.8 and currentTime < 2.9):
                    speed = 0
                elif(currentTime >= 2.9 and currentTime < 5):
                    speed = 8
                elif(currentTime >= 5 and currentTime < 5.1):
                    speed = 0
                elif(currentTime >= 5.1 and currentTime < 6):
                    speed = 8
                elif(currentTime >= 6):
                    speed = 0
            case 4:

                if(currentTime > flagTime):
                    speed = speed + 0.05
                    flagTime += 0.1

        # print(speed)

        return speed, flagTime

def donut(speed):
        return speed