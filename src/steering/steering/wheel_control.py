#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robp_interfaces.msg import DutyCycles
import numpy as np

class WheelController(Node):

    def __init__(self):
        super().__init__("Wheel_Controller") 
        self.get_logger().info("Start wheel control")

        # Create subscription to /cmd_vel for Twist messages
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        # Create publisher to send duty cycle commands to the motors
        self.duty_pub = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)

        # Create a timer to send duty cycles at a regular interval
        self.timer = self.create_timer(0.2, self.publish_duty_cycles)  # 5 Hz Frequency

        # Initialize some variables for the robot's movement
        self.vel_x = 0.0 # Default linear velocity
        self.rot_z = 0.0  # Default rotation velocity
        self.max_factor = 0.0 # Dedault max factor

        # Given parameters
        self.wheel_radius = 0.046 # 0.04915
        self.base = 0.30 # 0.30

    def twist_callback(self, msg: Twist):
        # Update linear and rotational velocities based on cmd_vel message
        self.vel_x = msg.linear.x*0.6
        self.max_factor = msg.linear.z
        self.rot_z = msg.angular.z*0.9

    def publish_duty_cycles(self):
        # Steer geometry, from linear velocity to wheel velocity
        u_w = self.vel_x / (self.wheel_radius)
        u_phi = self.rot_z * self.base / self.wheel_radius 
        if np.abs(u_phi) >= self.max_factor:
            if u_phi >= 0:
                u_phi = 2*self.max_factor - u_phi
            if u_phi < 0:
                u_phi = -(2*self.max_factor + u_phi)

        # Wheel velocities
        w_l = u_w - u_phi/2
        w_r = u_w + u_phi/2

        # Corrections for uneven motors
        correct_factor = 0.0 # 0.006
        if u_w >= 0:
            if -0.25 < u_phi < 0:
                w_r = w_r - (correct_factor)/(0.25)*u_phi - correct_factor
            elif 0 < u_phi < 0.25:
                w_r = w_r + (correct_factor)/(0.25)*u_phi - correct_factor
            elif u_phi == 0 and u_w != 0:
                w_r = w_r - correct_factor
        elif u_w < 0:
            if -0.25 < u_phi < 0:
                w_r = w_r + (correct_factor)/(0.25)*u_phi + correct_factor
            elif 0 < u_phi < 0.25:
                w_r = w_r - (correct_factor)/(0.25)*u_phi + correct_factor
            elif u_phi == 0 and u_w != 0:
                w_r = w_r + correct_factor

        # Create message
        duty_cycles_msg = DutyCycles()        
        duty_cycles_msg.duty_cycle_left = w_l
        duty_cycles_msg.duty_cycle_right = w_r
        
        # self.get_logger().info(f"Dutycycles \n Left: {w_l}, right: {w_r}")

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