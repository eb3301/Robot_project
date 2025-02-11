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

        self.pub_goal_pose()

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

        angle = np.arctan2(dy, dx)
        steering = angle - heading

        #Check if we are at goal
        if np.linalg.norm(np.array([dx,dy])) < 0.1:
            print(f"Arrived at destination!")
            twist_msg.angular.z = 1.0
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(3)
            self.generate_point()
            return

        print(f"Angle Difference: {np.abs(steering - heading)}")
        if steering > 0.2 or steering < -0.2: 
            #Turn
            twist_msg.angular.z = 1.0
            self.cmd_vel_pub.publish(twist_msg)
            #time.sleep(0.1)
            return
        else: 
            #We drive
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)



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
        x, y = (random.uniform(0, 3), random.uniform(0, 3))
        print(f"Moving to marker at:({x, y})")
        return x, y

    def pub_goal_pose(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "odom"
        goal_pose.pose.position.x = self.goal_pose[0]
        goal_pose.pose.position.y = self.goal_pose[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        self.goal_pose_pub.publish(goal_pose)

def main():
    rclpy.init()
    node = AutoControll()
    x, y = node.generate_point()
    node.goal_pose = (x, y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()