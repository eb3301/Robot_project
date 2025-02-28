#!/usr/bin/env python

import numpy as np

import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Path

class AutoControll(Node):

    def __init__(self):
        super().__init__('Auto_Controller')

        # Init publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribe to current pose
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10) # This might need to be restricted

        # Subscribe to goal pose
        self.path_sub = self.create_subscription(Path, "/planned_path", self.path_callback, 1) # latch topic

        # Timer
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Preallocation
        self.lookahead_distance = 10

    def timer_callback(self):
        # Compute the velocity command using the Pure Pursuit algorithm
        twist_msg = pure_pursuit_velocity(self.current_position, self.current_heading, self.pose_list, self.lookahead_distance)
        
        # Publish the twist message
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Published velocity: linear = {twist_msg.linear.x}, angular = {twist_msg.angular.z}")

    def pose_callback(self, msg : PoseWithCovarianceStamped):
        # Get posion of robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_position = (x, y)
        self.current_heading = self.compute_heading(msg.pose.pose.orientation)

    # Updates current path
    def path_callback(self, msg: Path):
        # Create an empty list to hold the extracted poses
        self.pose_list = []
        
        # Iterate through the list of PoseStamped messages in the Path
        for pose_msg in msg.poses:
            # Extract the position (x, y) and orientation (yaw) from each PoseStamped message
            position = (pose_msg.pose.position.x, pose_msg.pose.position.y)
            # orientation = self.compute_heading(orientation) 
            
            # Create a tuple containing the position and orientation
            # pose = position #, orientation)
            
            # Append the pose to the pose list
            self.pose_list.append(position)

    def compute_heading(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        _, _, yaw = euler_from_quaternion((x, y, z, w))
        return yaw

        

def pure_pursuit_velocity(current_position, current_heading, path, lookahead_distance, max_linear_velocity=0.5, steering_gain=1.0):
    """
    Pure Pursuit algorithm that calculates linear and angular velocities for ROS 2.

    Parameters:
    - current_position: Tuple (x, y) representing the current position of the vehicle.
    - current_heading: The current heading (orientation) of the vehicle in radians.
    - path: List of waypoints [(x1, y1), (x2, y2), ..., (xn, yn)] representing the path to follow.
    - lookahead_distance: The lookahead distance (in meters) from the current position to the target point.
    - max_linear_velocity: The maximum linear velocity of the robot.
    - steering_gain: A gain factor to adjust how aggressively the robot turns.

    Returns:
    - twist_msg: A ROS `Twist` message with the calculated linear and angular velocities.
    """
    # Convert path to a NumPy array
    path = np.array(path)

    # Calculate the vector from the vehicle to each point in the path
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

    # Calculate the linear velocity (keep constant or adjust based on the distance)
    # We use a constant linear velocity for simplicity, but you can adjust it based on the distance
    linear_velocity = max_linear_velocity

    # Calculate the angular velocity using the steering angle and a gain factor
    angular_velocity = steering_gain * steering_angle

    # Create a ROS Twist message
    twist_msg = Twist()
    twist_msg.linear.x = linear_velocity
    twist_msg.angular.z = angular_velocity

    return twist_msg

def calculate_steering_angle(current_position, current_heading, target_point):
    """
    Calculate the steering angle to the target point using the Pure Pursuit algorithm.
    """
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
    _ = input("Press enter to start moving!")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()