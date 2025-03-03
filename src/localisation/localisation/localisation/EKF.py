#!/usr/bin/env python
import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
import tf2_ros
from builtin_interfaces.msg import Time

from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformException, TransformBroadcaster

import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Pose, TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path

import numpy as np
#from filterpy.kalman import ExtendedKalmanFilter as EKF

from scipy.spatial.transform import Rotation as R

class EKF_Algorithm(Node):
    def __init__(self):
        super().__init__("EKF")
        
        # ROS topics
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)
        self.create_subscription(
            Encoders, '/motor/encoders', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.ekfPath_pub = self.create_publisher(Path, 'EKF_Path', 10)

        #TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread = True)
        self._tf_broadcaster = TransformBroadcaster(self)


        # EKF Parameters
        self.ekf_x = np.array([0.0, 0.0, 0.0]) # Initial state
        self.ekf_cov = np.diag([1e-4, 1e-4, 1e-4]) # Initial Uncertainty (Covariannce matrix)

        # Paths
        self.ekf_path = Path()

        # Time
        self.time = None
        self.imu_counter = 0

        # None
        self.yaw_offset = None
        self.imu_yaw = 0

        self.get_logger().info("Initialised EKF node...")


    def odom_callback(self, msg: Encoders):
        '''Prediction step based on encoders'''
        # The kinematic parameters for the differential configuration
        t = rclpy.time.Time().from_msg(msg.header.stamp)
        
        if self.time is None:
            self.time = t
            return
        else: 
            dt = (t - self.time).nanoseconds * 1e-9
        
        ticks_per_rev = 48 * 64
        wheel_radius = 0.045 # 9.6cm
        base = 0.27 # 29.7cm 

        # Ticks since last message
        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        K = 1/ticks_per_rev * 2*np.pi
        
        # Control Parameters
        v = wheel_radius / 2 * (K * delta_ticks_right + K*delta_ticks_left) 
        w = wheel_radius / base * (K * delta_ticks_right - K * delta_ticks_left)

        if dt > 0:
            x, y, yaw = self.ekf_x.flatten()
            # # Update states
            x_bar = x +  v*np.cos(yaw) 
            y_bar = y + v*np.sin(yaw) 
            yaw_bar = yaw + w 
            
            predicted_pose = np.array([[x_bar, y_bar, yaw_bar]])

            self.time = t

            # Jacobian
            G = np.array([[1.0, 0.0,-v * np.sin(yaw_bar)],
                          [0.0, 1.0, v * np.cos(yaw_bar)],
                          [0.0, 0.0, 1.0                ]])
            
            # Set noise to 1% of distance traveled
            motion_noise = np.diag([0.01 * v**2, 0.01 * v**2, 0.01 * w**2])
            imu_noise = np.array([[0.01]])

            # Predict step
            cov_bar = G @ self.ekf_cov @ G.T + motion_noise

            # Update step
            H = np.array([[0.0, 0.0, 1.0]]) # Measurement jacobian (Only yaw)
            kalman_gain = cov_bar @ H.T @ np.linalg.inv(H @ cov_bar @ H.T + imu_noise)
            yaw_meas = self.imu_yaw

            self.ekf_cov = cov_bar - kalman_gain @ H @ cov_bar
            yaw_error = self.normalize_angle(yaw_meas - yaw_bar)
            self.ekf_x = predicted_pose.T + kalman_gain * yaw_error
            
            x, y, yaw = self.ekf_x.flatten()
            yaw = self.normalize_angle(yaw)
            self.broadcast_transform(t, x, y, yaw)
            self.publish_pose(x, y, yaw)


    def imu_callback(self, msg: Imu):
        '''Run update step using IMU orientaiton. Imu publishes with freq 250 hz'''   
        # Change freq to 50 hz       
        if self.imu_counter < 4:
            self.imu_counter += 1 
            return
        self.imu_counter= 0
        '''Process imu data and perform update step '''


        # Transform data to correct orientation
        q = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        q_x = R.from_euler('x', 180, degrees = True).as_quat()  # Rotation by 180° around x-axis
        q_z = R.from_euler('z', -90, degrees = True).as_quat()

        q_rot = R.from_quat(q)  # scipy uses (x, y, z, w) format

        # Rotate 180 degrees around x-axis
        q1_rot = R.from_quat(q_x) * q_rot

        # Rotate -90 degrees around z-axis
        q2_rot = R.from_quat(q_z) * q1_rot

        # Transform to quarternion
        final_q = q2_rot.as_quat()
        
        # Calculate imu offset
        if self.yaw_offset is None:
            self.yaw_offset = self.compute_heading(final_q)
        
        self.imu_yaw = self.compute_heading(final_q) - self.yaw_offset
        self.imu_yaw = self.normalize_angle(self.imu_yaw)

    def publish_pose(self, x, y, theta):
        '''Publish estimated pose with corresponding covariance'''
        if self.time is None:
            return
        
        # Create message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.time.to_msg()
        pose_msg.header.frame_id = 'odom'

        # Set Position
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # Set Orientation   
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, theta)
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        
        # Set Covariance
        pose_msg.pose.covariance = [0] * 36 # Flat 6x6 matrix
        pose_msg.pose.covariance[0]  = self.ekf_cov[0, 0]
        pose_msg.pose.covariance[1]  = self.ekf_cov[0, 1]
        pose_msg.pose.covariance[6]  = self.ekf_cov[1, 0]
        pose_msg.pose.covariance[7]  = self.ekf_cov[1, 1]
        pose_msg.pose.covariance[35] = self.ekf_cov[2, 2]

        PathPose = PoseStamped()
        PathPose.header = pose_msg.header
        PathPose.pose = pose_msg.pose.pose
        
        self.ekf_path.poses.append(PathPose) 

        self.ekf_path.header.stamp = self.time.to_msg()
        self.ekf_path.header.frame_id = 'odom'
        self.ekfPath_pub.publish(self.ekf_path) 
         
        self.pose_pub.publish(pose_msg)
        

    def broadcast_transform(self, time, x, y, yaw):
        """Takes a 2D pose and broadcasts it as a ROS transform. """
        t = TransformStamped()
        t.header.stamp = time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, the robot can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self._tf_broadcaster.sendTransform(t)


    def compute_heading(self, orientation):
            if isinstance(orientation, np.ndarray):
                x, y, z, w = orientation[0], orientation[1], orientation[2], orientation[3]
            else:
                x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
            
            _, _, yaw = euler_from_quaternion((x, y, z, w))
            return yaw


    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi] range."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

def main():
    rclpy.init()
    node = EKF_Algorithm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()