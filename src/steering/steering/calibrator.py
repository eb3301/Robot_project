#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.logging

import time
from geometry_msgs.msg import Twist

import numpy as np


class Calibrator(Node):

    def __init__(self):
        super().__init__('Calibrator')
        self.get_logger().info(f"Starting calibrator")

        # Init publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Robot paramters
        self.wheel_radius = 0.046 # 0.04915
        self.base = 0.3 # 0.30


    def turn(self):
        # Turn clockwise
        print('Turn')
        msg = Twist()
        msg.angular.z = ((self.wheel_radius / self.base) / (np.pi/2)) * np.pi/2 / 8
        self.cmd_vel_pub.publish(msg)
        time.sleep(8)
        # Turn anti-clockwise
        print('Turn')
        msg.angular.z = ((self.wheel_radius / self.base) / (np.pi/2)) * -np.pi/2 / 8
        self.cmd_vel_pub.publish(msg)
        time.sleep(8)
        # Stop
        print('Stop')
        msg_2 = Twist()
        self.cmd_vel_pub.publish(msg_2)

    def straight(self):
        # Move forward
        print('Drive')
        time.sleep(1)
        msg = Twist()
        msg.linear.x = self.wheel_radius / 8
        self.cmd_vel_pub.publish(msg)
        time.sleep(1)
        time.sleep(16)
        # Stop
        print('Stop')
        msg_2 = Twist()
        self.cmd_vel_pub.publish(msg_2)



def main():
    rclpy.init()
    node = Calibrator()
    try:
        node.straight()
        # node.turn()
        # rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        msg_2 = Twist()
        node.cmd_vel_pub.publish(msg_2)
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()