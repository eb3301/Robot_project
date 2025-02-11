#!/usr/bin/env python

import numpy as np

import rclpy
import rclpy.logging
from rclpy.node import Node
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
        self.pose = msg.pose #Pose

        #Create twist msg
        twist_msg = Twist()
        #All 0 for 2D steering
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        #Arbitrary velocity
        twist_msg.linear.x = 0.1

        #Calculate correct desired steering
        curr_x, curr_y = self.pose.position.x, self.pose.position.y
        goal_x, goal_y = self.goal_pose[0], self.goal_pose[1]
        dx, dy = goal_x - curr_x, goal_y - curr_y
        heading  = self.compute_heading(self.pose.orientation)
        print(f"curr_x: {curr_x}" + "curr_y", curr_y)
        print(f"goal_x: {goal_x}" + "goal_y", goal_y)

        angle = np.arctan2(dy, dx)
        steering = angle - heading
        print(f"angle: {angle}")
        print(f"Heading: {heading}")
        print(f"Steering: {steering}")

        if steering > 0.1 or  steering < -0.1: 
            #Turn
            twist_msg.angular.z = steering
            self.cmd_vel_pub.publish(twist_msg)
            print("published turned")
            time.sleep(1)
            return
        else: 
            if np.linalg.norm(np.array([dx,dy])) > 0.1:
                #We drive
                twist_msg.angular.z = 0
                self.cmd_vel_pub.publish(twist_msg)
                print("published straight")
                time.sleep(1)
            else:
                self.generate_point()
                time.sleep(1)



    # def calculate_path(self):

    #     #2D stearing twist msg
    #     twist_msg = Twist()
    #     #All 0 for 2D stearing
    #     twist_msg.linear.y = 0.0
    #     twist_msg.linear.z = 0.0
    #     twist_msg.angular.x = 0.0
    #     twist_msg.angular.y = 0.0

    #     #Set velocity and stearing for twist msg
    #     twist_msg.linear.x = 0.1
    #     twist_msg.angular.z = np.pi/6 

    #     self.get_logger().info("Turning 30deg")
    #     self.cmd_vel_pub.publish(twist_msg)


    def compute_heading(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        _, _, yaw = euler_from_quaternion((x, y, z, w))
        return yaw
    
    
    def generate_point(self):
        x, y = (random.uniform(0, 2), random.uniform(0, 2))
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        self.goal_pose_pub.publish(goal_pose)
        return x, y

def main():
    rclpy.init()
    node = AutoControll()
    node.goal_pose = (5, 5)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()