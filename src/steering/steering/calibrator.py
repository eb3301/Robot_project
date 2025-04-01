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
        self.wheel_radius = 0.04915 # m
        self.base = 0.31 # m


    def turn(self):
        # Turn clockwise
        msg = Twist()
        msg.angular.z = ((self.wheel_radius / self.base) / (np.pi/2)) * np.pi/2 / 8
        self.cmd_vel_pub.publish(msg)
        time.sleep(seconds=8)
        # Turn anti-clockwise
        msg.angular.z = ((self.wheel_radius / self.base) / (np.pi/2)) * -np.pi/2 / 8
        self.cmd_vel_pub.publish(msg)
        time.sleep(seconds=8)
        # Stop
        msg_2 = Twist()
        self.cmd_vel_pub.publish(msg_2)

    def straight(self):
        # Move forward
        msg = Twist()
        msg.linear.x = self.wheel_radius / 8
        self.cmd_vel_pub.publish(msg)
        time.sleep(seconds=8)
        # Stop
        msg_2 = Twist()
        self.cmd_vel_pub.publish(msg_2)



def main():
    rclpy.init()
    node = Calibrator()
    try:
        rclpy.spin(node)
        node.straight()
        # node.turn()
    except rclpy.exceptions.ROSInterruptException:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()