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
        self.timer = self.create_timer(0.5, self.publish_duty_cycles)  # 2 Hz Frequency

        # Initialize some variables for the robot's movement
        self.vel_x = 0.0 # Default linear velocity
        # self.vel_y = 0.0 # Default linear velocity
        # self.theta = 0.0 # Default orientation
        self.rot_z = 0.0  # Default rotation velocity

    def twist_callback(self, msg: Twist):
        # Update linear and rotational velocities based on cmd_vel message
        self.vel_x = msg.linear.x
        # self.vel_y = msg.linear.y
        self.rot_z = msg.angular.z

    def publish_duty_cycles(self):
        # Given parameters
        wheel_radius = 0.04915 # 0.04921
        base = 0.31 # 0.30

        # # Steer geometry, from velocity to wheel velocity
        # if np.abs(self.vel_x) > 0 or np.abs(self.vel_y) > 0:
        #     self.theta = np.arctan2(self.vel_y, self.vel_x)

        u_w = self.vel_x / (wheel_radius)
        u_phi = self.rot_z * base / wheel_radius 
        if np.abs(u_phi) >= 0.5:
            if u_phi >= 0:
                u_phi = 1 - u_phi
            if u_phi < 0:
                u_phi = 1 + u_phi 
        print(u_w)
        print(u_phi)

        # Wheel angular velocity
        w_l = u_w + u_phi/2
        w_r = u_w - u_phi/2

        print(w_l)
        print(w_r)
        print('------')

        # Create message
        duty_cycles_msg = DutyCycles()
              
        duty_cycles_msg.duty_cycle_left = w_l
        duty_cycles_msg.duty_cycle_right = w_r

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