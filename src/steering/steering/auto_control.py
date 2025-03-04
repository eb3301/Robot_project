#!/usr/bin/env python

import numpy as np

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Path

class AutoControll(Node):

    def __init__(self):
        super().__init__('Auto_Controller')

        qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE, # Does not lose msg
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, # New subscribers get msg after pub
            depth = 1
        )

        # Init publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribe to current pose
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.pose_callback, 10) # This might need to be restricted

        # Subscribe to goal pose
        self.path_sub = self.create_subscription(Path, "/planned_path", self.path_callback, qos) # latched topic

        # Timer
        self.timer = self.create_timer(0.2, self.timer_callback)

        # Preallocation
        self.lookahead_distance = 3
        self.current_position = (0.0)
        self.current_heading = 0
        self.pose_list = []

    def timer_callback(self):
        if self.pose_list:
            # Calculate the distance to the final point
            final_point = self.pose_list[-1]
            distance_to_goal = np.linalg.norm(np.array(self.current_position) - np.array(final_point))

            # If the robot is close enough to the final point, stop publishing
            if distance_to_goal < 0.1:  # You can adjust this threshold
                self.get_logger().info("Goal reached! Stopping.")
                twist_msg = Twist()  # Publish zero velocity to stop the robot
                self.cmd_vel_pub.publish(twist_msg)
                return  # Stop further processing
            
            # Compute the velocity command using the Pure Pursuit algorithm
            twist_msg = pure_pursuit_velocity(self.current_position, self.current_heading, self.pose_list, self.lookahead_distance)
            
            # Publish the twist message
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info(f"Published velocity: linear = {twist_msg.linear.x}, angular = {twist_msg.angular.z}")

    # Get position of robot
    def pose_callback(self, msg : PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_position = (x, y)
        self.current_heading = self.compute_heading(msg.pose.pose.orientation)

    # Updates path, extract position (x, y) and add to list
    def path_callback(self, msg: Path):
        self.pose_list = []
        
        for pose_msg in msg.poses:
            
            position = (pose_msg.pose.position.x, pose_msg.pose.position.y)
            self.pose_list.append(position)

    # Compute heading from Loke
    def compute_heading(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        _, _, yaw = euler_from_quaternion((x, y, z, w))
        return yaw

        
def pure_pursuit_velocity(current_position, current_heading, path, lookahead_distance):
    # Convert path to a NumPy array
    path = np.array(path)

    # Calculate the vector from the robot to each point in the path
    dx = path[:, 0] - current_position[0]
    dy = path[:, 1] - current_position[1]

    # Calculate the Euclidean distance from the current position to each waypoint
    distances = np.sqrt(dx**2 + dy**2)

    # Find the first target point within the lookahead distance
    target_point_idx = np.argmax(distances >= lookahead_distance)
    if target_point_idx == 0:
        target_point_idx = 1  # Skip the first point, as it is the vehicle's current position

    # Get the target point
    target_point = path[target_point_idx]

    # Calculate the steering angle to the target point
    steering_angle = calculate_steering_angle(current_position, current_heading, target_point)

    # Robot paramters
    wheel_radius = 0.04915 # m
    base = 0.31 # m
    
    # Maximum velocities
    max_factor = 1 / 4
    max_vel = wheel_radius * max_factor # m/s
    max_rot = ((wheel_radius / base) / (np.pi/2)) * max_factor # rad/s

    # Use max linear velocity
    linear_velocity = max_vel

    # Calculate the angular velocity using the steering angle and a gain factor
    angular_velocity = max_rot * steering_angle

    # Create a ROS Twist message
    twist_msg = Twist()
    twist_msg.linear.x = linear_velocity
    twist_msg.linear.z = max_factor
    twist_msg.angular.z = angular_velocity
    return twist_msg

def calculate_steering_angle(current_position, current_heading, target_point):
    # Vector from current position to target point
    vector_to_target = target_point - np.array(current_position)

    # Calculate the angle to the target point
    angle_to_target = np.arctan2(vector_to_target[1], vector_to_target[0])

    # Steering angle is the difference between the vehicle's heading and the angle to the target
    steering_angle = angle_to_target - current_heading

    return steering_angle



def main():
    rclpy.init()
    node = AutoControll()
    # _ = input("Press enter to start moving!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()