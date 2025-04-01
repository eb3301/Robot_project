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
        # Joystick returns float in [-1, 1]
        joy_vel_x = msg.axes[3]
        joy_vel_y = msg.axes[2] 

        # Robot paramters
        wheel_radius = 0.04915 # m
        base = 0.31 # m
        
        # Maximum velocities
        max_factor = 1 / 4
        max_vel = wheel_radius * max_factor # m/s
        max_rot = ((wheel_radius / base) / (np.pi/2)) * max_factor # rad/s

        # Calculate raw velocities
        vel_x = joy_vel_x * max_vel
        vel_y = joy_vel_y * max_vel

        # Calculate the magnitude of the velocity vector
        velocity_magnitude = np.sqrt(vel_x**2 + vel_y**2)

        # If the magnitude exceeds max_vel, normalize it
        if velocity_magnitude > max_vel:
            vel_x = vel_x / velocity_magnitude * max_vel
            vel_y = vel_y / velocity_magnitude * max_vel

        # Calculate rotation change
        if np.abs(vel_x) > 0 or np.abs(vel_y) > 0:
            rot = np.arctan2(vel_y, vel_x) * max_rot 
        else:
            rot = 0

        # Create Twist msg
        cmd_msg = Twist()
        cmd_msg.linear.x = float(vel_x)
        # cmd_msg.linear.y = float(vel_y)
        cmd_msg.linear.z = float(max_factor)
        cmd_msg.angular.z = float(rot)
        
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