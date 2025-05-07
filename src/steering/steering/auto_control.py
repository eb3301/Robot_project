#!/usr/bin/env python

import numpy as np

import rclpy
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

        # Publish velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribe to current pose
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.pose_callback, 10)

        # Subscribe to goal pose
        self.path_sub = self.create_subscription(Path, "/planned_path", self.path_callback, qos)

        # Create timer for execution
        self.timer = self.create_timer(0.2, self.execution)  # 5 Hz Frequency

        # TF2 - to handle transforms from map to odom
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self, spin_thread = True)

        # Preallocation
        self.current_position = (0.0, 0.0)
        self.current_heading = 0
        self.pose_list = []
        self.start = False
        self.count = 0

        # Robot paramters
        wheel_radius = 0.046 # 0.04915
        base = 0.3 # 0.30

        # Maximum velocity parameters
        self.max_factor = 1 / 6
        self.max_vel = wheel_radius * self.max_factor # m/s
        self.max_rot = ((wheel_radius / base) / (np.pi/2)) * self.max_factor # rad/s


    # To get the position of the robot at all times
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

            # Save position of robot
            x = map_pose.position.x
            y = map_pose.position.y

            self.current_position = (x, y)
            self.current_heading = self.compute_heading(map_pose.orientation)
        except TransformException:
            # self.get_logger().info('No transform found')
            return

    # Compute heading from Loke
    def compute_heading(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        _, _, yaw = euler_from_quaternion((x, y, z, w))
        return yaw
            
        
    # Updates path if new is recived, extract position (x, y) from data and add to list
    def path_callback(self, msg: Path):
        self.get_logger().info('Path received')
        self.pose_list = []
        
        # Add positions to a list
        for pose_msg in msg.poses:
            position = (pose_msg.pose.position.x, pose_msg.pose.position.y)
            self.pose_list.append(position)
        
        if self.pose_list:
            # Set the path to be executed
            self.get_logger().info('Executing Path')
            self.start = True

            # Calculate the distance to the final point
            self.final_point = self.pose_list[-1]

            # Calculate the lookahead distance
            self.resolution = np.sqrt((self.pose_list[0][0] - self.pose_list[1][0])**2 + (self.pose_list[0][1] - self.pose_list[1][1])**2)
            self.lookahead_distance = 6*self.resolution

            # Smooth the path to avoid strange steering - jumping
            smoothed_path = create_god_path(self.pose_list, self.resolution)

            # Convert path to a NumPy array
            self.pose_list = np.array(smoothed_path)
        else:
            self.start = False
            self.get_logger().info("Path empty, stopping")
            # Publish 0, to stop
            twist_msg = Twist()
            self.cmd_vel_pub.publish(twist_msg)

    # Generate actual driving commands
    def execution(self):
        if self.start:
            # Calculate distance to goal
            distance_to_goal = np.linalg.norm(np.array(self.current_position) - np.array(self.final_point))

            if distance_to_goal > 0.1: # self.resolution:
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

        # Find the current pose's index in the path list
        current_point_idx = np.argmin(distances)
        target_point_idx = current_point_idx
        curr_x = path[current_point_idx][0]
        curr_y = path[current_point_idx][1]

        # Find target index in the list that is closest to the lookahead distance
        while target_point_idx + 1 < len(path) and np.sqrt((path[target_point_idx + 1][0] - curr_x)**2 + (path[target_point_idx + 1][1] - curr_y)**2) < 2*lookahead_distance:
            target_point_idx += 1

        if target_point_idx == 0:
            target_point_idx = 1  # Skip the first point

        # Get the target point that we want to drive to
        target_point = path[target_point_idx]

        # Calculate the steering angle to the target point
        steering_angle = self.calculate_steering_angle(current_position, current_heading, target_point)
        
        # Calculate linear velocity
        linear_velocity = (1 - abs(steering_angle) / (np.pi / 2)) * self.max_vel

        # Calculate the angular velocity
        angular_velocity = steering_angle * self.max_rot

        # Create a ROS Twist message
        twist_msg = Twist()

        # Added a stop if ther robot turn to much, to still detect things in turns
        if self.count <= 5:
            twist_msg.linear.x = linear_velocity
            twist_msg.linear.z = self.max_factor
            twist_msg.angular.z = angular_velocity
        else:
            # If the robot has turn a while, send a pause of 0 velocity
            self.count = 0
        
        return twist_msg

    def calculate_steering_angle(self, current_position, current_heading, target_point):
        # Vector from current position to target point
        vector_to_target = target_point - np.array(current_position)

        # Calculate the angle to the target point
        angle_to_target = np.arctan2(vector_to_target[1], vector_to_target[0])

        # Steering angle is the difference between the vehicle's heading and the angle to the target
        steering_angle = angle_to_target - current_heading

        # Normalize the steering angle to be in the range of -pi to pi
        steering_angle = (steering_angle + np.pi) % (2 * np.pi) - np.pi
        if abs(steering_angle) > np.pi/4:
            self.count += 1

        # Constrain the steering angle to be within the range of -pi/2 to pi/2 to not drive backwards
        if steering_angle > np.pi / 2:
            steering_angle = np.pi / 2
        elif steering_angle < -np.pi / 2:
            steering_angle = -np.pi / 2
        # Constrain the steering angle to not turn as much
        elif np.pi / 4 < steering_angle <= np.pi / 2:
            steering_angle = np.pi / 2
        elif -np.pi / 2 <= steering_angle < -np.pi / 4:
            steering_angle = -np.pi / 2

        return steering_angle

# Smoothing with bezier curves
def cubic_bezier(t, P0, P1, P2, P3):
    return (1 - t)**3 * P0 + 3 * (1 - t)**2 * t * P1 + 3 * (1 - t) * t**2 * P2 + t**3 * P3

# Perform smoothing for a segment
def smooth(points):
    points = np.array(points)
    number = len(points)
    smoothed_path = []

    P0 = points[0]
    P1 = points[int(number / 4)]
    P2 = points[int(3 * number / 4) - 1]
    P3 = points[-1]

    t_values = np.linspace(0, 1, 20)

    for t in t_values:
        smoothed_point = cubic_bezier(t, P0, P1, P2, P3)
        smoothed_path.append(tuple(smoothed_point))  # Convert the NumPy array to a tuple

    return smoothed_path

# Group similar adjecent poses that has a pattern into a segment
def extract_segments(path, resolution):
    horizontal_segments = []
    vertical_segments = []
    diagonal_segments = []
    
    i = 0
    while i < len(path) - 2:
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        x3, y3 = path[i + 2]

        if y1 == y2 == y3 and np.isclose(abs(x2 - x1), resolution) and np.isclose(abs(x3 - x2), resolution):
            start = i
            while i + 2 < len(path) and np.isclose(path[i + 1][1], path[i][1]) and \
                (np.isclose(path[i + 1][0], path[i][0] + resolution) or np.isclose(path[i + 1][0], path[i][0] - resolution)) and \
                (np.isclose(path[i + 2][0], path[i + 1][0] + resolution) or np.isclose(path[i + 2][0], path[i + 1][0] - resolution)):
                i += 1
            horizontal_segments.append((path[start:i + 2], i))

        elif x1 == x2 == x3 and np.isclose(abs(y2 - y1), resolution) and np.isclose(abs(y3 - y2), resolution):
            start = i
            while i + 2 < len(path) and np.isclose(path[i + 1][0], path[i][0]) and \
                (np.isclose(abs(path[i + 1][1] - path[i][1]), resolution) or np.isclose(abs(path[i + 1][1] - path[i][1]), -resolution)) and \
                (np.isclose(abs(path[i + 2][1] - path[i + 1][1]), resolution) or np.isclose(abs(path[i + 2][1] - path[i + 1][1]), -resolution)):
                i += 1
            vertical_segments.append((path[start:i + 2], i))

        elif (np.isclose(abs(x2 - x1), resolution) and np.isclose(abs(y2 - y1), 0) and np.isclose(abs(x3 - x2), 0) and np.isclose(abs(y3 - y2), resolution)) or \
             (np.isclose(abs(x2 - x1), 0) and np.isclose(abs(y2 - y1), resolution) and np.isclose(abs(x3 - x2), resolution) and np.isclose(abs(y3 - y2), 0)):        
            start = i
            while i + 2 < len(path) and (
                (np.isclose(abs(path[i + 1][0] - path[i][0]), resolution) and np.isclose(abs(path[i + 1][1] - path[i][1]), 0) and np.isclose(abs(path[i + 2][0] - path[i + 1][0]), 0) and np.isclose(abs(path[i + 2][1] - path[i + 1][1]), resolution)) or
                (np.isclose(abs(path[i + 1][0] - path[i][0]), 0) and np.isclose(abs(path[i + 1][1] - path[i][1]), resolution) and np.isclose(abs(path[i + 2][0] - path[i + 1][0]), resolution) and np.isclose(abs(path[i + 2][1] - path[i + 1][1]), 0))
            ):
                i += 1
            diagonal_segments.append((path[start:i + 2], i))

        i += 1

    return horizontal_segments, vertical_segments, diagonal_segments


# Function to combine the joint of adjacent segments
def combine_segments(sorted_segments):
    # Combine so every segment is longer than 8
    i = 0
    while i < len(sorted_segments) - 1:
        # Check if the current segment has less than 8 elements
        if len(sorted_segments[i]) < 8:
            # How many elements we need to reach 8
            needed = 8 - len(sorted_segments[i])
            # Extend the current segment with elements from the next one
            sorted_segments[i].extend(sorted_segments[i + 1][:needed])
            # Remove the elements we just took from the next segment
            sorted_segments[i + 1] = sorted_segments[i + 1][needed:]
            
            # If the next segment becomes empty, remove it
            if len(sorted_segments[i + 1]) == 0:
                sorted_segments.pop(i + 1)
        else:
            # Move to the next segment
            i += 1

    new_segments = []
    # Merging the joints of the segments
    for i in range(1, len(sorted_segments)):
        # Get the last two points of the previous segment
        prev_segment = sorted_segments[i - 1]
        next_segment = sorted_segments[i]
        
        # Take the last two points from the previous segment
        prev_end_points = prev_segment[-3:]

        # Take the first two points from the next segment
        next_start_points = next_segment[:3]

        # Create a new segment by combining these 4 points
        merge_segment = prev_end_points + next_start_points

        # Add the non-modified segments to the list, if the segment is long enough
        new_segments.append(prev_segment[2:-2])

        # Add the merge segment
        new_segments.append(merge_segment)

    # Add the last segment and update the first to include the start
    new_segments[0] = sorted_segments[0][:-2]
    if len(sorted_segments[-1]) > 3:
        new_segments.append(sorted_segments[-1][2:])
    return new_segments

# Create the smooth path
def create_god_path(path, resolution):
    # Extract segments
    horizontal, vertical, diagonal = extract_segments(path, resolution)

    # Combine segments
    combined_segments = diagonal + horizontal + vertical

    # Sort the segments by the first point of each segment
    combined_segments_sorted = sorted(combined_segments, key=lambda x: x[1])

    # Remove the sorting index (only keep the segments)
    sorted_segments = [segment[0] for segment in combined_segments_sorted]

    if len(sorted_segments) > 2:
        # Combine adjacent segments by taking out the last two and first two points
        combined_segments = combine_segments(sorted_segments)
    else:
        combined_segments = sorted_segments

    # Now, smooth the segments and create the new path (as a list of tuples)
    smoothed_path = []
    for segment in combined_segments:
        smoothed_segment = smooth(segment)
        smoothed_path.extend(smoothed_segment)  # Add the smoothed points to the final list

    return smoothed_path

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