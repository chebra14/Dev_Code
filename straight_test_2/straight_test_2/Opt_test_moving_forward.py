#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import math
from ackermann_msgs.msg import AckermannDriveStamped
#Change this line (1) from gym
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu

import time

class TESTNode(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        self.ds = dataSave()
        
        #Change this line (2) from gym to /pf/pose/odom
        # self.pose_subscriber = self.create_subscription(Odometry, '/pf/pose/odom', self.callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.pose_subscriber = self.create_subscription(LaserScan, '/scan', self.callback_scan, 10)
        self.raw_car = self.create_subscription(Odometry, '/odom', self.callback_wheelspeed, 10)
        self.imu_car = self.create_subscription(Imu, '/sensors/imu/raw', self.callback_IMU, 10)
        #Change this line (3) from gym
        self.joy_sub = self.create_subscription(Joy, "/joy", self.callbackJoy, 10)
        self.Joy7 = 0

        self.x0 = [0.0] * 3    #wheelspeed, setspeed, time  
        self.start_time = time.perf_counter()
        self.go_time = 2.0
        self.set_speed = 4.0

        self.get_logger().info("Initlialized")

    def callback_wheelspeed(self, msg:Odometry):

        current_time = time.perf_counter() - self.start_time
        
        cmd = AckermannDriveStamped()

        quat_ori = [msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w]
        
        yaw = self.euler_from_quaternion(quat_ori[0], quat_ori[1], quat_ori[2], quat_ori[3])

        if (current_time) < self.go_time:
            cmd.drive.speed = self.set_speed
            self.get_logger().info("GO")
        else:
            cmd.drive.speed = 0.0
            self.get_logger().info("STOP")

            if (current_time) > 10.0:
                self.ds.savefile_odom()
                self.ds.savefile_IMU()
                self.ds.savefile_las()
                self.get_logger().info("Odom, Imu and pf saved")
                rclpy.shutdown()

        speed_publish = cmd.drive.speed

        self.drive_pub.publish(cmd)

        self.x0 = [msg.twist.twist.linear.x,
                   speed_publish,
                   current_time]
        
        self.ds.saveStates(self.x0)

    def callback_IMU(self, msg:Imu):

        current_time = time.perf_counter() - self.start_time

        linear_x = msg.linear_acceleration.x
        linear_y = msg.linear_acceleration.y

        self.get_logger().info(f'{linear_x}, {linear_y}')

        self.ds.saveIMU(linear_x, linear_y, current_time)


    def callback_scan(self, msg:LaserScan):

        current_time = time.perf_counter() - self.start_time

        laserScan = np.array(msg.ranges)

        mid_las = laserScan[540]

        self.ds.saveLaser(mid_las, current_time)

    def callback(self, msg:Odometry):

        cmd = AckermannDriveStamped()

        quat_ori = [msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w]
        
        yaw = self.euler_from_quaternion(quat_ori[0], quat_ori[1], quat_ori[2], quat_ori[3])

        current_time = time.perf_counter() - self.start_time

        if (current_time) < self.go_time:
            cmd.drive.speed = self.set_speed
            self.get_logger().info("GO")
        else:
            cmd.drive.speed = 0.0
            self.get_logger().info("STOP")

            if (current_time) > 10.0:
                self.ds.savefile_odom()
                self.ds.savefile_las()
                self.ds.savefile_IMU()
                self.get_logger().info("Odom, IMU and Laser saved")
                rclpy.shutdown()

        speed_publish = cmd.drive.speed

        self.drive_pub.publish(cmd)

        self.x0 = [msg.twist.twist.linear.x,
                   speed_publish,
                   current_time]
        
        self.ds.saveStates(self.x0)

    #Change this function (4) from gym
    def callbackJoy(self, msg: Joy):
        self.Joy7 = msg.buttons[7]


    def euler_from_quaternion(self,x, y, z, w):  
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z # in radians
    
class dataSave:
    def __init__(self):
        self.rowSize = 50000
        self.stateCounter = 0
        self.laserCounter = 0
        self.IMUCounter = 0
        self.txt_x0 = np.zeros((self.rowSize,3))
        self.txt_ls0 = np.zeros((self.rowSize,2))
        self.txt_IMU0 = np.zeros((self.rowSize,3))

    def saveLaser(self, las, currentTime):
        self.txt_ls0[self.laserCounter,0] = las
        self.txt_ls0[self.laserCounter,1] = currentTime

        self.laserCounter += 1
        #Pose, time

    def saveIMU(self, linear_x, linear_y, current_time):
        self.txt_IMU0[self.IMUCounter,0] = linear_x
        self.txt_IMU0[self.IMUCounter,1] = linear_y
        self.txt_IMU0[self.IMUCounter,2] = current_time

        self.IMUCounter += 1
        #linear acceleration (x), linear acceleration (y), time

    def saveStates(self, x0):
        self.txt_x0[self.stateCounter,0] = x0[0]
        self.txt_x0[self.stateCounter,1] = x0[1]
        self.txt_x0[self.stateCounter,2] = x0[2]

        self.stateCounter += 1
        #actual_speed, expected_speed, time

    def savefile_odom(self):

        np.savetxt(f"Results_Odom.csv", self.txt_x0, delimiter = ',', header="Wheelspeed, Setspeed, Time", fmt="%-10f")

        self.txt_x0 = np.zeros((self.rowSize,3))
        self.stateCounter = 0

    def savefile_las(self):

        np.savetxt(f"Results_Laser.csv", self.txt_ls0, delimiter = ',', header="Pose, Time", fmt="%-10f")

        self.txt_ls0 = np.zeros((self.rowSize,2))
        self.laserCounter = 0

    def savefile_IMU(self):
        np.savetxt(f"Results_IMU.csv", self.txt_IMU0, delimiter = ',', header="Linear_X, Linear_Y, Time", fmt="%-10f")

        self.txt_IMU0 = np.zeros((self.rowSize,3))
        self.IMUCounter = 0
    
def main(args = None):
    
    #use - ros2 run teleop_twist_keyboard teleop_twist_keyboard 
    #to move the car manually around the map
    rclpy.init(args = args)
    controller_node = TESTNode()
 
    rclpy.spin(controller_node)          #allows the node to always been running 
    rclpy.shutdown()                    #shut dowwn the node


if __name__ == "__main__":
    main()
