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
from filterpy.kalman import ExtendedKalmanFilter as EKF

from scipy.spatial.transform import Rotation as R

class EKF_Algorithm(Node):
    def __init__(self):
        super().__init__("EKF")
        
        # ROS topics
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)
        self.create_subscription(
            Encoders, '/motor/encoders', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback,10)
        self.OdomPath_pub = self.create_publisher(Path, 'odom_path', 10)
        self.ekfPath_pub = self.create_publisher(Path, 'EKF_Path', 10)

        #TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Timers
        self.ICP_timer = self.create_timer(1, self.ICP_update)
        self.pub_timer = self.create_timer(0.1, self.publish_pose)
        self.broadcast_timer = self.create_timer(0.1, self.broadcast_transform)

        # KF Parameters
        self.ekf = EKF(dim_x =3, dim_z = 3) # x: state, z: measurement 
        self.ekf.x = np.array([0.0, 0.0, 0.0]) # Initial state

        self.ekf.F = np.eye(3) # Transition Matrix (Jacobian of motion model)
        self.ekf.H = np.eye(3) # Meausrement model
         
        self.ekf.P = np.eye(3) * 0.1# Covariance matrix
        self.ekf.Q = np.diag([0.08, 0.08, 0.08]) # Odom noise
        self.ekf.R = np.eye(3) * 0.1 # Measurement noise
        
        # Paths
        self.odom_path = Path()
        self.ekf_path = Path()

        #Time
        self.time = rclpy.time.Time()

        self.imu_counter = 0

        self.get_logger().info("Initialised EKF node...")


    def odom_callback(self, msg: Encoders):
        '''Prediction step based on encoders'''
        # The kinematic parameters for the differential configuration
        dt = 50 / 1000
        t = rclpy.time.Time().from_msg(msg.header.stamp)

        # First time it is called, only update time
        if self.time.nanoseconds == 0.0:
                self.time = t
                return
        
        ticks_per_rev = 48 * 64
        wheel_radius = 0.04915 # 0.04921
        base = 0.31 # 0.30

        # Ticks since last message
        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        K = 1/ticks_per_rev * 2*np.pi
        
        # Control Parameters
        v = wheel_radius/2 * (K*delta_ticks_right + K*delta_ticks_left) 
        w = wheel_radius/base * (K*delta_ticks_right - K*delta_ticks_left)

 
        if dt > 0.0:
            yaw = self.ekf.x[2]

            # Update states
            self.ekf.x[0] += v*np.cos(yaw)
            self.ekf.x[1] += v*np.sin(yaw)
            self.ekf.x[2] += w
            self.time = t

            # Update transition matrix
            self.ekf.F = np.array([[1, 0,-v * np.sin(yaw) * dt],
                                   [0, 1, v * np.cos(yaw) * dt],
                                   [0, 0, 1                   ]])

            self.publish_odom_path(msg.header.stamp)
            # Perform prediction step
            self.ekf.predict()
    

    def imu_callback(self, msg: Imu):
        '''Run update step using IMU orientaiton. Imu publishes with freq 250 hz'''
        # Change freq to 50 hz       
        if self.imu_counter != 5:
            self.imu_counter += 1 
            return

        '''Process imu data and perform update step '''
        t = rclpy.time.Time().from_msg(msg.header.stamp)

        # Orientation and corresponding covariance
        q = msg.orientation
        cov = msg.orientation_covariance 

        if cov[0] == -1.0:
            return

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

        yaw = self.compute_heading(final_q)

        z_obs = np.array([yaw]) # Measured pose

        # Noise -- 0 Covariance leads to singular values
        if cov[8] == 0:
            cov_yaw = 0.05
        else:
            cov_yaw = cov[8]
        R_imu = np.array([[cov_yaw]]) * 0.1 # Noise -- Adapt
    
        self.ekf.update(z_obs, # measurement
                        HJacobian = self.imu_jacobian, # measurement jacobian
                        Hx = self.imu_model, # Prediction
                        R = R_imu, # Noise model
                        residual = np.subtract) # residual function(z_obs - z_pred)

    def imu_model(self, x):
        return np.array([x[2]])
    
    def imu_jacobian(self, x):
        H = np.zeros((1,3))
        H[0, 2] = 1
        return H


    def ICP_update(self):
        '''Perform update step based on transform from ICP algorithm'''
        try:

            # Lookup Transform
            tf_future = self.tf_buffer.wait_for_transform_async(
                target_frame = "map",
                source_frame = "odom",
                time = rclpy.time.Time()
            )
            
            rclpy.spin_until_future_complete(self, tf_future, timeout_sec = 1)

            icp_tf = self.tf_buffer.lookup_transform(
                "map",  
                'odom', 
                time = rclpy.time.Time(),
                timeout = rclpy.duration.Duration(seconds=0.1)
            )
            
            # Convert current pose to type: Pose()
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = self.ekf.x[0], self.ekf.x[1], 0.0
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.ekf.x[2])
            pose.orientation.x, pose.orientation.y = qx, qy 
            pose.orientation.z, pose.orientation.w = qz, qw

            # Transform pose based on ICP transform
            icp_pose = tf2_geometry_msgs.do_transform_pose(pose, icp_tf)        

            # Extract observed measurements
            x_obs = icp_pose.position.x
            y_obs = icp_pose.position.y
            theta_obs = self.compute_heading(icp_pose.orientation)

            z_obs = np.array([x_obs, y_obs, theta_obs]) # Measured pose

            R_icp = np.eye(3) * 0.4 # Noise -- Adapt

            # Perform EKF update step

            self.ekf.update(z_obs, # measurement
                            HJacobian = self.JacobianICP, # measurement jacobian
                            Hx = self.ICP_measurement, # Prediction
                            R = R_icp, # Noise model
                            residual = np.subtract) # residual function(z_obs - z_pred)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {'odom'} to {"map"}: {ex}')

    
    def ICP_measurement(self, x):
        '''Returns expected pose at time of measurement update'''
        return x
    

    def JacobianICP(self, x):
        '''Return Jacobian for ICP measurement function'''
        return np.eye(3)


    def publish_pose(self):
        '''Publish estimated pose with corresponding covariance'''
        # Current pose
        x, y, theta = self.ekf.x[0], self.ekf.x[1], self.ekf.x[2]
        
        # Create message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.time.to_msg()
        pose_msg.header.frame_id = 'map'

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
        pose_msg.pose.covariance[0]  = self.ekf.P[0, 0]
        pose_msg.pose.covariance[1]  = self.ekf.P[0, 1]
        pose_msg.pose.covariance[6]  = self.ekf.P[1, 0]
        pose_msg.pose.covariance[7]  = self.ekf.P[1, 1]
        pose_msg.pose.covariance[35] = self.ekf.P[2, 2]

        PathPose = PoseStamped()
        PathPose.header = pose_msg.header
        PathPose.pose = pose_msg.pose.pose
        
        self.ekf_path.poses.append(PathPose) 

        self.ekf_path.header.stamp = self.time.to_msg()
        self.ekf_path.header.frame_id = 'map'
        self.ekfPath_pub.publish(self.ekf_path) 
         
        self.pose_pub.publish(pose_msg)
        

    def broadcast_transform(self):
        """Takes a 2D pose and broadcasts it as a ROS transform. """
        t = TransformStamped()
        t.header.stamp = self.time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = self.ekf.x[0]
        t.transform.translation.y = self.ekf.x[1]
        t.transform.translation.z = self.ekf.x[2]

        # For the same reason, the robot can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0.0, 0.0, self.ekf.x[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self._tf_broadcaster.sendTransform(t)


    def publish_odom_path(self, stamp):
        """Takes a 2D pose appends it to the path and publishes the whole path.

        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        self.odom_path.header.stamp = stamp
        self.odom_path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = self.odom_path.header

        pose.pose.position.x = self.ekf.x[0]
        pose.pose.position.y = self.ekf.x[1]
        pose.pose.position.z = 0.01  # 1cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, self.ekf.x[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.odom_path.poses.append(pose)
        self.OdomPath_pub.publish(self.odom_path)



    def compute_heading(self, orientation):
            if isinstance(orientation, np.ndarray):
                x, y, z, w = orientation[0], orientation[1], orientation[2], orientation[3]
            else:
                x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
            
            _, _, yaw = euler_from_quaternion((x, y, z, w))
            return yaw

    


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