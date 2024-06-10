#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu

import time

class TESTNode(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        self.ds = dataSave()
        
        self.imu_car = self.create_subscription(Imu, '/sensors/imu/raw', self.callback_IMU, 10)

        self.start_time = time.perf_counter()

        self.get_logger().info("Initlialized")


    def callback_IMU(self, msg:Imu):
        linear_x = msg.linear_acceleration.x
        linear_y = msg.linear_acceleration.y

        self.get_logger().info("Polling...")

        current_time = time.perf_counter() - self.start_time

        self.ds.saveIMU(linear_x, linear_y, current_time)

        if (current_time) > 10.0:
                self.ds.savefile_IMU()
                self.get_logger().info("IMU saved")
                rclpy.shutdown()

    
class dataSave:
    def __init__(self):
        self.rowSize = 50000
        self.IMUCounter = 0
        self.txt_IMU0 = np.zeros((self.rowSize,3))

    def saveIMU(self, linear_x, linear_y, current_time):
        self.txt_IMU0[self.IMUCounter,0] = linear_x
        self.txt_IMU0[self.IMUCounter,1] = linear_y
        self.txt_IMU0[self.IMUCounter,2] = current_time

        self.IMUCounter += 1
        #linear acceleration (x), linear acceleration (y), time

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
