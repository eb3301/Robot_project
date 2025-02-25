#!/usr/bin/env python
import rclpy
import rclpy.duration
import rclpy.logging
from rclpy.node import Node
import rclpy.time
import tf2_ros
from tf2_ros import TransformException

from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from laser_geometry import LaserProjection

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
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/lidar', 10)
        self.ref_cloud_pub = self.create_publisher(PointCloud2, 'reference_PointCloud', 10)

        # TF2 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread = True)

        # Timers
        self.broadcast_timer = self.create_timer(0.05, self.broadcast_transform)
        self.ICP_timer = self.create_timer(0.5, self.ICP)

        # Initialise Cloud Variables
        self.source_pcd = None
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
        
        self.get_logger().info("Initialised ICP node...")
        

    def scan_callback(self, msg: LaserScan):
        self.stamp = msg.header.stamp

        # Only publish every tenth scan
        if not self.create_ref:
            if self.counter < 2:
                self.counter += 1
                return
            else:
                self.counter = 0

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
        
        # Project lidar to point cloud
        cloud = self.proj.projectLaser(msg)

        # Transform point cloud
        cloud_out = do_transform_cloud(cloud, t)

        # Extract points 
        points = pc2.read_points_numpy(cloud_out, field_names=("x", "y", "z"), skip_nans=True)

        # Create reference pointcloud for ICP
        if self.ref_counter <= 30:
            self.accumulated_scans.append(points)
            self.ref_counter += 1
            self.ref_cloud_pub.publish(cloud_out)
            return
        else: 
            if self.create_ref:
                ref_time = time.time() - self.start_time
                self.create_ref = False
                merged_points = np.vstack(self.accumulated_scans)
                merged_ref = pc2.create_cloud_xyz32(cloud_out.header, merged_points)
                self.source_pcd = self.pointcloud_2_open3d(merged_ref)
                self.get_logger().info(f"Reference cloud accumulated in {ref_time} seconds")
                
        # Save Pointcloud for ICP
        self.target_pcd = self.pointcloud_2_open3d(cloud_out)
        # Publish every 10th Pointcloud
        if self.counter == 0:  
            self.lidar_pub.publish(cloud_out)



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


    def ICP(self):
        '''ICP Algorithm Implementation'''
        if self.source_pcd is None or self.target_pcd is None:
            return
        start_time = time.time()
        
        # Compute normals for Point-to-Plane
        radius = 0.05 # max range for neighbour search
        max_nn = 20 # max amount of neighbours

        self.source_pcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius ,max_nn))  
        self.target_pcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius ,max_nn))  

        
        # Algorithm Parameters
        threshold = 0.1 # Max distance to be a 'match'
        trans_init = self.transform.copy() # Initial transformation estimate

        #Convergence Criterias
        max_iteration, relative_fitness, relative_rmse = 200, 1e-5, 1e-5
        # Run algorithm 
        result = o3d.pipelines.registration.registration_icp(
            self.source_pcd, self.target_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness, relative_rmse, max_iteration)
        )

        if result.fitness > 0.5 and result.inlier_rmse < 0.07: 
            self.transform = np.dot(self.transform, result.transformation)

            translation = self.transform[:3, 3]
            dist = np.linalg.norm(translation)

            self.get_logger().info(f"ICP transform: \n Distance moved: {dist} \n fitness: {result.fitness} \n Inlier rmse: {result.inlier_rmse}")

            # end_time = time.time()
            # self.get_logger().info(f"ICP algorithm took {end_time - start_time:.6f} seconds")
        else: 
            self.get_logger().info(f"Ignoring ICP result, fitness: {result.fitness}, Inlier rmse: {result.inlier_rmse}")


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
