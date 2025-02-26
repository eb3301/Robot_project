#!/usr/bin/env python
import rclpy
import rclpy.duration
import rclpy.logging
from rclpy.node import Node
import rclpy.time
import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from laser_geometry import LaserProjection
from robp_interfaces.msg import Encoders

from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf2_ros.transform_listener import TransformListener


import numpy as np
import open3d as o3d
import time

class ICPNode(Node):
    def __init__(self):
        super().__init__('ICP_Node')

        # ROS Topics
        scan_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT, # Ok to drop scans
            durability = QoSDurabilityPolicy.VOLATILE, # Does not store old scans
            depth = 1 
        )
        lidar_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE, # Does not lose msg
            durability = QoSDurabilityPolicy.VOLATILE, # Does not store old scans
            depth = 1 
        )

        ref_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE, # Does not lose msg
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, # New subscribers get msg after pub
            depth = 1
        )

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, scan_qos)
        self.lidar_pub = self.create_publisher(PointCloud2, '/lidar', lidar_qos)
        self.ref_cloud_pub = self.create_publisher(PointCloud2, 'reference_PointCloud', ref_qos)
        self.encoder_sub = self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, scan_qos)

        # TF2 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread = True)

        # Timers
        self.broadcast_timer = self.create_timer(0.05, self.broadcast_transform)

        # Initialise Cloud Variables
        self.target_pcd = None  

        self.transform = np.identity(4)

        # Reference Cloud Variables
        self.proj = LaserProjection()
        self.counter = 0
        self.ref_counter = 0
        self.start_time = None
        self.create_ref = True
        self.accumulated_scans = []

        self.stamp = None

        self.rotating  = False
        
        self.get_logger().info("Initialised ICP node...")
        

    def encoder_callback(self, msg: Encoders):
        '''Tell the program when the robot is rotating...'''
        ticks_per_rev = 48 * 64
        wheel_radius = 0.045 # 9.6cm
        base = 0.27 # 29.7cm 
        # Ticks since last message
        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        K = 1/ticks_per_rev * 2*np.pi
        
        # Control Parameters
        w = wheel_radius / base * (K * delta_ticks_right - K * delta_ticks_left)
        #self.get_logger().info(f"w: {w}")
        if np.abs(w) > 0.01:
            self.rotating = True
        else:
            self.rotating = False

    def scan_callback(self, msg: LaserScan):
        # if self.rotating:
        #     return
        self.stamp = msg.header.stamp

        # Time of message
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)

        # Take time at first scan
        if self.start_time is None:
            self.start_time = time.time()      
        
        to_frame_rel = 'odom'
        from_frame_rel = msg.header.frame_id

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                msg_time
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')     
            return
        
        # Filter out scans too close
        min_range = 0.25
        msg.ranges = [r if r >= min_range else float('nan') for r in msg.ranges]
        
        # Project lidar to point cloud
        cloud = self.proj.projectLaser(msg)

        # Transform point cloud
        cloud_out = do_transform_cloud(cloud, t)

        # Extract points 
        points = pc2.read_points_numpy(cloud_out, field_names=("x", "y", "z"), skip_nans=True)

        # Create reference pointcloud for ICP
        if self.ref_counter <= 10:
            self.accumulated_scans.append(points)
            self.ref_counter += 1
            self.ref_cloud_pub.publish(cloud_out)
            
            if self.create_ref:
                ref_time = time.time() - self.start_time
                self.create_ref = False
                merged_points = np.vstack(self.accumulated_scans)
                merged_ref = pc2.create_cloud_xyz32(cloud_out.header, merged_points)
                self.target_pcd = self.pointcloud_2_open3d(merged_ref)
                self.get_logger().info(f"Reference cloud accumulated in {ref_time} seconds")
                
        else:
            if not self.create_ref:
                self.accumulated_scans = []
                self.create_ref = True 

            else:
                self.accumulated_scans.append(points)
                self.lidar_pub.publish(cloud_out)
                if self.counter == 5:
                    merged_points = np.vstack(self.accumulated_scans)
                    merged_ref = pc2.create_cloud_xyz32(cloud_out.header, merged_points)

                    # Save and publish pointcloud
                    source_pcd =  self.pointcloud_2_open3d(merged_ref)
                    self.ICP(source_pcd)

                    self.accumulated_scans = []
                    self.counter = 0
                else: self.counter += 1

    def compute_heading(self, orientation):
            x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
            _, _, yaw = euler_from_quaternion((x, y, z, w))
            return yaw


    def pointcloud_2_open3d(self, pcd2):
        '''Convert PointCloud2 message to an array'''
        #Convert PointCloud2 to Numpy array
        points = pc2.read_points_numpy(pcd2, field_names = ('x' ,'y', 'z'), skip_nans = True)

        #Convert Numpy array to o3d pointcloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        return pcd


    def ICP(self, source_pcd):
        '''ICP Algorithm Implementation'''
        if  source_pcd is None or self.target_pcd is None:
            return
        start_time = time.time()
        
        # Compute normals for Point-to-Plane
        radius = 0.1 # max range for neighbour search
        max_nn = 15 # max amount of neighbours

        source_pcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius ,max_nn))  
        self.target_pcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius ,max_nn))  

        
        # Algorithm Parameters
        threshold = 0.1 # Max distance to be a 'match'
        trans_init = self.transform.copy() # Initial transformation estimate

        #Convergence Criterias
        max_iteration, relative_fitness, relative_rmse = 300, 1e-6, 1e-6
        # Run algorithm 
        result = o3d.pipelines.registration.registration_icp(
            source_pcd, self.target_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness, relative_rmse, max_iteration)
        )


        translation = result.transformation[:3, 3]
        dist = np.linalg.norm(translation)
        if result.fitness > 0.3 and result.inlier_rmse < 0.1: # and dist < 1: 
            #alpha = 0.3  # Small smoothing factor
            #self.transform = alpha * result.transformation + (1 - alpha) * self.transform
            self.ttransform = result.transformation

            # self.get_logger().info(f"ICP transform: \n Distance moved: {dist} \n fitness: {result.fitness} \n Inlier rmse: {result.inlier_rmse}")

            # end_time = time.time()
            # self.get_logger().info(f"ICP algorithm took {end_time - start_time:.6f} seconds")
        else: 
            #self.get_logger().info(f"Ignoring ICP result, fitness: {result.fitness}, Inlier rmse: {result.inlier_rmse}")
            pass

    def broadcast_transform(self):
        if not self.stamp:
            return
        
        t_icp = self.transform
        t = TransformStamped()
        #self.get_logger().info("Broadcasting transform...")
        t.header.stamp = self.stamp
        t.header.frame_id = 'map' 
        t.child_frame_id = 'odom' 
        
        t.transform.translation.x = t_icp[0, 3]
        t.transform.translation.y = t_icp[1, 3]
        t.transform.translation.z = t_icp[2, 3]

        q = quaternion_from_matrix(t_icp)
        q = q / np.linalg.norm(q)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ICPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
