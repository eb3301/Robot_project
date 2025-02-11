#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robp_interfaces.msg import DutyCycles
import numpy as np

class WheelController(Node):

    def __init__(self):
        super().__init__("Wheel_Controller")   

        # Create subscription to /cmd_vel for Twist messages
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        # Create publisher to send duty cycle commands to the motors
        self.duty_pub = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)

        # Create a timer to send duty cycles at a regular interval
        self.timer = self.create_timer(0.4, self.publish_duty_cycles)  # 10Hz frequency

        # Initialize some variables for the robot's movement
        self.linear_vel = 0.05  # Default linear velocity
        self.rot = 1.0  # Default rotation velocity

    def twist_callback(self, msg: Twist):
        # Update linear and rotational velocities based on cmd_vel message
        self.linear_vel = msg.linear.x
        self.rot = msg.angular.z

    def publish_duty_cycles(self):
        duty_cycles_msg = DutyCycles()
        
        rot_speed = 0.08
        binary_rot = self.rot
        
        
        if binary_rot == 0.0:
            print("Driving straight")
            # Move straight
            duty_cycles_msg.duty_cycle_left = self.linear_vel
            duty_cycles_msg.duty_cycle_right = self.linear_vel
        else:
            print("Turning")
            duty_cycles_msg.duty_cycle_left = -rot_speed
            duty_cycles_msg.duty_cycle_right = rot_speed

        # Publish the duty cycle message to control motors
        self.duty_pub.publish(duty_cycles_msg)
       

def main():
    rclpy.init()

    node = WheelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
