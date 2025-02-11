#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robp_interfaces.msg import DutyCycles
import numpy as np

class WheelController(Node):

    def __init__(self):
        super().__init__("Wheel_Controller")   

        # Init publisher
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        # Init subscriber
        self.duty_pub = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)

    def twist_callback(self, msg : Twist):

        linear_vel = 0.1
        rot = msg.angular.z
        rot = (rot + np.pi) % (2 * np.pi) - np.pi

        duty_cycles_msg = DutyCycles()
        if rot == 0:
            #drive straight
            duty_cycles_msg.duty_cycle_left = linear_vel
            duty_cycles_msg.duty_cycle_right = linear_vel
        else: 
            
            normalised_rot = rot / np.pi
            custom_factor = 0.8 #decice empircally
            turn_factor = normalised_rot * custom_factor
            duty_cycles_msg.duty_cycle_left = -normalised_rot * turn_factor
            duty_cycles_msg.duty_cycle_right = normalised_rot * turn_factor

        print("Publishing duty cycle msg")
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