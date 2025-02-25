#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension


class PickupPublisher(Node):

    def __init__(self):
        super().__init__('Publish_joint_states')
        print('Node started')

        self.joint_pub = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        

    def publish(self, answer):
        joint_msg = Int16MultiArray()
       
        #Fix weird datatype... 
        layout = MultiArrayLayout()
        dim = MultiArrayDimension()
        dim.label = ""
        dim.size = 0
        dim.stride = 0
        layout.dim = [dim]
        joint_msg.layout = layout

        #data: [pos1, pos2, pos3, pos4, pos5, pos6, time1, time2,time3,time4,time5,time6]
        if answer == "1":
            joint_msg.data = [12000,12000,12000,12000,12000,12000,1500,1500,1500,1500,1500,1500]
        elif answer == "2":
            joint_msg.data = [3000,12000,7000,14000,7000,12000,1500,1500,1500,1500,1500,1500]
        elif answer == "3":
            joint_msg.data = [7400,12000,7000,14000,7000,12000,1500,1500,1500,1500,1500,1500]
        else: 
            print("Wrong input...")

        self.joint_pub.publish(joint_msg)

def main():
    rclpy.init()
    node = PickupPublisher()
    try:
        while rclpy.ok():
            answer = input("Where do you want to move it? \n 1. Stand up \n 2. Move to Object \n 3. Grasp Object \n" )
            node.publish(answer)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()