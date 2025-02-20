#!/usr/bin/env python

import numpy as np

import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf_transformations import euler_from_quaternion
import time
import random
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

class AutoControll(Node):

    def __init__(self):
        super().__init__('Auto_Controller')

        # Init publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribe to current pose
        self.pose_sub = self.create_subscription(PoseStamped, "/odom_pose",
                                                   self.pose_callback, 10)

        # Init publisher of goal marker     
        self.goal_marker_pub = self.create_publisher(Marker, "/goal_marker", 10) 

        # Preallocation
        self.goal_pose = (0, 0)
        self.pose = Pose()
        self.arrived = True


    # Updates current pose
    def pose_callback(self, msg: PoseStamped):
        # Pose
        self.pose = msg.pose 

        # Publish goal marker
        self.pub_goal_marker()

        # Create twist msg
        twist_msg = Twist()

        # All 0 for 2D steering
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0

        # Arbitrary velocity
        twist_msg.linear.x = 0.20 # Change

        # Calculate correct desired steering
        curr_x, curr_y = self.pose.position.x, self.pose.position.y
        goal_x, goal_y = self.goal_pose[0], self.goal_pose[1]
        heading  = self.compute_heading(self.pose.orientation)
        dx, dy = goal_x - curr_x, goal_y - curr_y
        angle = np.arctan2(dy, dx)
        steering = angle - heading

        # Check if we are at goal
        if np.linalg.norm(np.array([dx,dy])) < 0.1:
            print(f"Arrived at destination!")
            twist_msg.angular.z = 1.0
            self.cmd_vel_pub.publish(twist_msg)
            x, y = self.generate_point()
            self.goal_pose = (x, y)
            time.sleep(3)
            return

        if steering > 0.2 or steering < -0.2: 
            # Turn
            if steering <= 0:
                twist_msg.angular.z = -1.0
                self.cmd_vel_pub.publish(twist_msg)
            if steering > 0:
                twist_msg.angular.z = 1.0
                self.cmd_vel_pub.publish(twist_msg)
            return
        else: 
            # Drive forward
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)

    def compute_heading(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        _, _, yaw = euler_from_quaternion((x, y, z, w))
        return yaw
    
    def generate_point(self): # Random samled point
        x, y = (random.uniform(0, 2), random.uniform(0, 2))
        print(f"Moving to marker at:({x, y})")
        return x, y

    def pub_goal_marker(self):
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "odom"
        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE  # Use sphere to represent the goal
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_pose[0]
        marker.pose.position.y = self.goal_pose[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Radius of the sphere
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.goal_marker_pub.publish(marker)

def main():
    rclpy.init()
    node = AutoControll()
    x, y = node.generate_point()
    node.goal_pose = (x, y)
    _ = input("Press enter to start moving!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()