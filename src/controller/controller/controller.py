#!/usr/bin/env python
import numpy as np
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
        
        rot = abs(joy_rot)

        #Transform joy stick reading to velocity
        max_vel = 0.3 #m/s 
        
        vel = joy_vel * max_vel

        #Create DutyCycles msg
        duty_cycles_msg = DutyCycles()



        duty_cycles_msg.header = header

        if joy_rot > 0.05:
            joy_rot = np.abs(joy_rot/3) 
            duty_cycles_msg.duty_cycle_left = -0.1
            duty_cycles_msg.duty_cycle_right = 0.1
        elif joy_rot < -0.05:
            joy_rot = np.abs(joy_rot/3)
            duty_cycles_msg.duty_cycle_left = 0.1
            duty_cycles_msg.duty_cycle_right = -0.1
        else:
            if joy_vel > 0.0: 
                duty_cycles_msg.duty_cycle_left = 0.2
                duty_cycles_msg.duty_cycle_right = 0.185
            if joy_vel < 0.0:
                duty_cycles_msg.duty_cycle_left = -0.2
                duty_cycles_msg.duty_cycle_right = -0.185
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