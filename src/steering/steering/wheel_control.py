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
        max_vel = 1 #m/s --- (made it up)

        # Velocity and rotation
        vel = msg.linear.x # m/s
        stearing = msg.angular.z # rad

        vel_factor = vel / max_vel

        # GPT -- Duty Cycle Turning Factor
        rot_factor = np.tan(stearing)

        # Message
        duty_cycles_msg = DutyCycles()

        duty_cycles_msg.duty_cycle_left = vel + vel_factor * rot_factor
        duty_cycles_msg.duty_cycle_right = vel + vel_factor * (1-rot_factor)

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