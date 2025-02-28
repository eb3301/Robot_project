#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np

class Controller(Node):

    def __init__(self):
        super().__init__('Joy_Controller')
        
        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        #Subscribe to encoder topic and call callback function on each recieved message
        self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        
        # Publish to cmd_vel topic
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)


    def joy_callback(self, msg: Joy):
        # Joy stick returns float in [-1,1]
        joy_vel_x = msg.axes[3]
        joy_vel_y = msg.axes[2] 
        
        # Transform joy stick reading to velocity
        max_vel = 0.5 #m/s 
        max_rot = 0.25 # rad/s
        vel = np.sqrt(joy_vel_x**2 + joy_vel_y**2)

        if vel != 0:
            vel_x = joy_vel_x / vel * max_vel
            vel_y = joy_vel_y / vel * max_vel
        else:
            vel_x = 0
            vel_y = 0

        if np.abs(joy_vel_y) >= 0.95:
            rot = max_rot
            vel_x = 0
            vel_y = 0
        else:
            rot = 0
            
        print(vel_x)
        print(vel_y)
        # Create Twist msg
        cmd_msg = Twist()
        cmd_msg.linear.x = vel_x
        cmd_msg.linear.y = vel_y
        cmd_msg.angular.z = rot
        
        # Publish message
        self.cmd_pub.publish(cmd_msg)

def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()