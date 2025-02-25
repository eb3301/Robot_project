#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
#from tf_transformations import quaternion_from_euler, euler_from_quaternion

from sensor_msgs.msg import Joy
from robp_interfaces.msg import DutyCycles

class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        
        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        #Subscribe to encoder topic and call callback function on each recieved message
        self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        
        # self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.duty_pub = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)

    def joy_callback(self, msg: Joy):

        
        header = msg.header

        #Joy stick returns float in [-1,1]
        joy_vel = msg.axes[3]
        joy_rot = msg.axes[2] 

        #Transform joy stick reading to velocity
        max_vel = 0.5 #m/s 
        max_rot = 30 #degrees
        
        vel = joy_vel * max_vel
        rotation = joy_rot * max_rot

        #Create DutyCycles msg
        duty_cycles_msg = DutyCycles()

        duty_cycles_msg.header = header
        if joy_rot > 0: 
            duty_cycles_msg.duty_cycle_left = vel * 0.5
            duty_cycles_msg.duty_cycle_right = vel
        elif joy_rot < 0:
            duty_cycles_msg.duty_cycle_right = vel * 0.5
            duty_cycles_msg.duty_cycle_left = vel
        else:
            duty_cycles_msg.duty_cycle_right = vel
            duty_cycles_msg.duty_cycle_left = vel

        self.duty_pub.publish(duty_cycles_msg)

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