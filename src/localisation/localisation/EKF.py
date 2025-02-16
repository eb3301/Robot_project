#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros

from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformException

import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Pose

import numpy as np
from filterpy.kalman import ExtendedKalmanFilter as EKF


class EKF_Algorithm(Node):
    def __init__(self):
        super().__init__("EKF")
        
        # ROS topics
        self.odom_pose_sub = self.create_subscription(PoseStamped, '/odom_pose', self.odom_callback, 10)
        self.icp_sub = self.create_subscription(PoseStamped, '/icp_pose', self.icp_callback)
        self.describe_parameterose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)

        #TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timers
        self.ICP_timer = self.create_timer(1, self.ICP_update)
        self.pub_timer = self.create_timer(0.1, self.publish_pose)

        # KF Parameters
        self.ekf = EKF(dim_x =3, dim_z =2) # x: state, z: measurement 
        self.ekf.x = np.array([0, 0, 0]) # Initial state

        self.ekf.F = np.eye(3) # Transition Matrix (Jacobian of motion model)
        self.ekf.H = np.eye(3) # Meausrement model
         
        self.ekf.P *= 1000 # Covariance matrix
        self.ekf.Q = np.eye(3) * 0.1 # Odom noise
        self.ekf.R = np.eye(3) * 0.1 # Measurement noise

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        #Time
        self.time = rclpy.time.Time()


    def odom_callback(self, msg: PoseStamped):
        '''Prediction step based on odometry data'''

        t = rclpy.time.Time().from_msg(msg.header.stamp)

        # First time it is called, only update time
        if self.time.nanoseconds == 0:
                self.time = t
                return

        dt = (t - self.time).nanoseconds * 1e-9 # nano seconds --> seconds

        dx = msg.pose.position.x - self.x
        dy = self.y - msg.pose.position.y

        theta = self.compute_heading(msg.pose.orientation)
        dtheta = self.heading - theta

        if dt > 0:
            # Update states
            self.x += dx
            self.y += dy
            self.heading += dtheta
            self.time = t

            # Update EKF state  
            self.ekf.x = np.array([self.x, self.y, self.heading])

            # Update control parameters
            v = np.sqrt((dx)**2 + (dy)**2) / dt

            # Update transition matrix
            self.ekf.F = np.array([1, 0, -v* np.sin(self.heading) * dt],
                                  [0, 1, v * np.cos(self.heading) * dt],
                                  [0, 0, 1                            ])

            # Perform prediction step
            self.ekf.predict()


    def ICP_update(self):
        '''Perform update step based on transform from ICP algorithm'''
        try:
            # Lookup Transform
            tf_future = self.tf_buffer.wait_for_transform_async(
                target_frame = "base_link",
                source_frame = "icp_odom",
                time = self.time
            )
            
            rclpy.spin_until_future_complete(self, tf_future, timeout_sec = 1)

            icp_tf = self.tf_buffer.lookup_transform(
                "base_link",  
                'icp_odom', 
                time = self.time,
                timeout = rclpy.duration.Duration(seconds=0.1)
            )
            
            # Convert current pose to type: Pose()
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = self.x, self.y, 0
            qx, qy, qz, qw = quaternion_from_euler(0, 0, self.heading)
            pose.orientation.x, pose.orientation.y = qx, qy 
            pose.orientation.z, pose.orientation.w = qz, qw

            # Transform pose based on ICP transform
            icp_pose = tf2_geometry_msgs.do_transform_pose(pose, icp_tf)        

            # Extract observed measurements
            x_obs = icp_pose.position.x
            y_obs = icp_pose.position.y
            theta_obs = self.compute_heading(icp_pose.orientation)

            z_pred = self.ekf.x # Predicted pose based on odometry
            z_obs = np.array([x_obs, y_obs, theta_obs]) # Measured pose


            H = np.eye(3) # Measurement model jacobian
            R = np.eye(3) * 0.1 # Noise -- Adapt

            # Perform EKF update step
            self.ekf.update(z_obs, # measurement
                            HJacobian = H, # measurement jacobian
                            Hx = z_pred, # Prediction
                            R = R, # Noise model
                            residual = np.subtract) # residual function(z_obs - z_pred)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {'base_link'} to {"icp_odom"}: {ex}')


    def publish_pose(self):
        '''Publish estimated pose with corresponding covariance'''

        # Create message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.time
        pose_msg.header.frame_id = 'map'

        # Set Position
        pose_msg.pose.pose.position.x = self.x
        pose_msg.pose.pose.position.y = self.y
        pose_msg.pose.pose.position.z = 0

        # Set Orientation
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.heading)
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

        self.pose_pub.publish(pose_msg)


    def compute_heading(self, orientation):
            x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
            _, _, yaw = euler_from_quaternion((x, y, z, w))
            return yaw