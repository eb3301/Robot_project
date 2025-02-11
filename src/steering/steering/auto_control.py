#!/usr/bin/env python

import numpy as np

import rclpy
import rclpy.logging
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf_transformations import euler_from_quaternion
import time
import random


class AutoControll(Node):

    def __init__(self):
        super().__init__('Auto_Controller')

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.pose_sub = self.create_subscription(PoseStamped, "/odom_pose",
                                                  self.pose_callback, 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.goal_pose = (0, 0)
        self.pose = Pose()
        self.arrived = True


    #Updates current pose
    def pose_callback(self, msg: PoseStamped):
        # latest_PoseStamped = msg.poses[-1] #PoseStamped
        self.pose = msg.pose #Pose

    
    #Calculates path to destination
    def calculate_path(self):
        curr_x, curr_y = self.pose.position.x, self.pose.position.y
        goal_x, goal_y = self.goal_pose[0], self.goal_pose[1]
        print(f"x: {self.pose.position.x}, y: {self.pose.position.y}")
        #2D stearing twist msg
        twist_msg = Twist()
        #All 0 for 2D stearing
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0

        #check if robot arrived at goal posititon
        min_error = 0.1
        dist = np.sqrt((goal_x - curr_x)**2 +
                       (goal_y - curr_y)**2)
        if dist < min_error:
            self.arrived = True
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            return 
        
        #Compute path to destination

        dx, dy = goal_x - curr_x, goal_y - curr_y
        heading  = self.compute_heading(self.pose.orientation)
        angle = np.arctan2(dy, dx)
        stearing = angle - heading
        vel_x = 0.1 #m/s --- IDK, does not really matter I guess...

        #Set velocity and stearing for twist msg
        twist_msg.linear.x = vel_x
        twist_msg.angular.z = stearing 

        self.get_logger().info("Publishing new Twist msg")
        self.cmd_vel_pub.publish(twist_msg)

    def compute_heading(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        _, _, yaw = euler_from_quaternion((x, y, z, w))
        return yaw
    
    def generate_point(self):
        x, y = (random.uniform(0, 20), random.uniform(0, 20))
        # y = (random.uniform(0, 2), random.uniform(0, 2))
        return x, y


def main():
    rclpy.init()
    node = AutoControll()
    _ = input("Press Enter to start driving! \n")
    try:
        while True:
            #Sample new destination point
            if node.arrived:
                node.arrived = False
                #sample random point within x:[0, 2], y:[0, 2]
                x, y = node.generate_point()
                
                node.goal_pose = (x, y)
                print(f"Sampled point: (x,y) = ({x,y})")
            else: 
                rclpy.spin_once(node)
                node.calculate_path()
                time.sleep(3) #Waiting 3s before calculating new path
                
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()