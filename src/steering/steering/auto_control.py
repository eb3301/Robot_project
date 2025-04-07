#!/usr/bin/env python

import numpy as np

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Path
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs

class AutoControll(Node):

    def __init__(self):
        super().__init__('Auto_Controller')
        self.get_logger().info(f"Starting controller")

        qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE, # Does not lose msg
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, # New subscribers get msg after pub
            depth = 1
        )

        # Init publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribe to current pose
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.pose_callback, 10)

        # Subscribe to goal pose
        self.path_sub = self.create_subscription(Path, "/planned_path", self.path_callback, qos)

        # Create timer
        self.timer = self.create_timer(0.2, self.execution)  # 5 Hz Frequency

        # TF2
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self, spin_thread = True)

        # Preallocation
        self.current_position = (0.0, 0.0)
        self.current_heading = 0
        self.pose_list = []
        self.start = False

    # Get position of robot
    def pose_callback(self, msg : PoseWithCovarianceStamped):
        # Init transform
        to_frame_rel = 'map'
        from_frame_rel = 'odom'
        time = rclpy.time.Time().from_msg(msg.header.stamp) # Maybe change?

        # Wait for the transform asynchronously
        tf_future = self.buffer.wait_for_transform_async(
        target_frame=to_frame_rel,
        source_frame=from_frame_rel,
        time=time
        )
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        # Lookup tansform
        try:
            t = self.buffer.lookup_transform(to_frame_rel,
                                        from_frame_rel,
                                        time)
            # Do the transform
            map_pose = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, t)

            # Get position of robot
            x = map_pose.position.x
            y = map_pose.position.y

            self.current_position = (x, y)
            self.current_heading = self.compute_heading(map_pose.orientation)
        except TransformException:
            self.get_logger().info('No transform found')

    # Compute heading from Loke
    def compute_heading(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        _, _, yaw = euler_from_quaternion((x, y, z, w))
        return yaw
            
        
    # Updates path, extract position (x, y) and add to list
    def path_callback(self, msg: Path):
        self.get_logger().info('Path received')
        self.pose_list = []
        
        for pose_msg in msg.poses:
            position = (pose_msg.pose.position.x, pose_msg.pose.position.y)
            self.pose_list.append(position)
        
        if self.pose_list:
            # Set the
            self.get_logger().info('Executing Path')
            self.start = True

            # Calculate the distance to the final point
            self.final_point = self.pose_list[-1]

            # Calculate the lookahead distance
            self.resolution = np.sqrt((self.pose_list[0][0] - self.pose_list[1][0])**2 + (self.pose_list[0][1] - self.pose_list[1][1])**2)
            self.lookahead_distance = 6*self.resolution

            # Convert path to a NumPy array
            self.pose_list = np.array(self.pose_list)
        else:
            self.start = False
            self.get_logger().info("Path empty, stopping")
            # Publish 0, to stop
            twist_msg = Twist()
            self.cmd_vel_pub.publish(twist_msg)

        
    def execution(self):
        if self.start:
            # Calculate distance to goal
            distance_to_goal = np.linalg.norm(np.array(self.current_position) - np.array(self.final_point))

            if distance_to_goal > self.resolution:
                # Compute the velocity command using the Pure Pursuit algorithm
                twist_msg = self.pure_pursuit_velocity(self.current_position, self.current_heading, self.pose_list, self.lookahead_distance)
                
                # Publish the twist message
                self.cmd_vel_pub.publish(twist_msg)
                # self.get_logger().info(f"Published velocity: linear = {twist_msg.linear.x}, angular = {twist_msg.angular.z}")
            else:  
                self.start = False
                self.get_logger().info("Goal reached! Stopping.")
                # Publish 0, to stop
                twist_msg = Twist()
                self.cmd_vel_pub.publish(twist_msg)

        
    def pure_pursuit_velocity(self, current_position, current_heading, path, lookahead_distance):
        # Calculate the vector from the robot to each point in the path
        dx = path[:, 0] - current_position[0]
        dy = path[:, 1] - current_position[1]

        # Calculate the Euclidean distance from the current position to each waypoint
        distances = np.sqrt(dx**2 + dy**2)

        # Find the current point index
        current_point_idx = np.argmin(distances)
        target_point_idx = current_point_idx
        curr_x = path[current_point_idx][0]
        curr_y = path[current_point_idx][1]

        # Find target index
        while target_point_idx + 1 < len(path) and np.sqrt((path[target_point_idx + 1][0] - curr_x)**2 + (path[target_point_idx + 1][1] - curr_y)**2) < 2*lookahead_distance:
            target_point_idx += 1

        if target_point_idx == 0:
            target_point_idx = 1  # Skip the first point, as it is the vehicle's current position

        # Get the target point
        target_point = path[target_point_idx]

        # Calculate the steering angle to the target point
        steering_angle = calculate_steering_angle(current_position, current_heading, target_point)

        # Robot paramters
        wheel_radius = 0.046 # 0.04915
        base = 0.3 # 0.30
        
        # Maximum velocities
        max_factor = 1 / 6
        max_vel = wheel_radius * max_factor # m/s
        max_rot = ((wheel_radius / base) / (np.pi/2)) * max_factor # rad/s

        # Use max linear velocity
        linear_velocity = (1 - abs(steering_angle) / (np.pi / 2)) * max_vel

        # Calculate the angular velocity using the steering angle and a gain factor
        angular_velocity = max_rot * steering_angle
        # print(f'Linear: {linear_velocity}')
        # print(f'Angular: {angular_velocity}')

        # Create a ROS Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.linear.z = max_factor
        twist_msg.angular.z = angular_velocity
        
        return twist_msg

def calculate_steering_angle(current_position, current_heading, target_point):
    # Vector from current position to target point
    vector_to_target = target_point - np.array(current_position)
    # print(f'Vector to target: {vector_to_target}')

    # Calculate the angle to the target point
    angle_to_target = np.arctan2(vector_to_target[1], vector_to_target[0])
    # print(f'Angle to target: {angle_to_target}')

    # Steering angle is the difference between the vehicle's heading and the angle to the target
    steering_angle = angle_to_target - current_heading

    # Normalize the steering angle to be in the range of -pi to pi
    steering_angle = (steering_angle + np.pi) % (2 * np.pi) - np.pi
    # print(f'Steering angle: {steering_angle}')

    # Constrain the steering angle to be within the range of -pi/2 to pi/2 to not drive backwards
    if steering_angle > np.pi / 2:
        steering_angle = np.pi / 2
    elif steering_angle < -np.pi / 2:
        steering_angle = -np.pi / 2

    return steering_angle



def main():
    rclpy.init()
    node = AutoControll()
    try:
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()