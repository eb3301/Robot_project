#!/usr/bin/env python
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
import tf2_ros

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
from geometry_msgs.msg import TransformStamped, PoseStamped

import numpy as np
import open3d as o3d

from localisation.srv import EstimatePose


class ICPservice(Node):
    def __init__(self):
        super().__init__('ICP_Service')
        
        # Create Service
        self.srv = self.create_service(EstimatePose, 'ICP_Service', self.ICP)

        # ROS Topics
        self.ref_sub = self.create_subscription(PointCloud2, '/lidar_ref', self.ref_callback, 10)
        self.pointcloud = self.create_subscription(PointCloud2, '/lidar', self.cloud_callback, 10)
        
        #TF2 Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialise Cloud Variables
        self.source_pcd = None
        self.target_pcd = None  

        self.current_time = 0.0
        self.get_logger().info("Initialised ICP node...")
        

    def ref_callback(self, msg: PointCloud2):
        self.target_pcd = self.pointcloud_2_open3d(msg)


    def cloud_callback(self, msg: PointCloud2):
        self.current_time = rclpy.time.Time().from_msg(msg.header.stamp)
        self.source_pcd = self.pointcloud_2_open3d(msg)


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


    def ICP(self, request, response):
        '''ICP Algorithm Implementation'''
        # If pointclouds have no arrived, return failed  
        if self.target_pcd is None or self.source_pcd is None:
            self.get_logger().info("Pointclouds not available yet")
            return response
        
        # Compute normals for Point-to-Plane
        radius = 0.1 # max range for neighbour search
        max_nn = 30 # max amount of neighbours

        self.target_pcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius ,max_nn))  
        
        # Algorith m Parameters
        threshold = 0.02 # Max distance to be a 'match'
        trans_init = np.identity(4) # Initial estimate

        #Convergence Criterias
        max_iteration, relative_fitness, relative_rmse = 50, 1e-6, 1e-6
        # Run algorithm 
        result = o3d.pipelines.registration.registration_icp(
            self.source_pcd, self.target_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness, relative_rmse, max_iteration)
        )

        transform = result.transformation
        response.transformation_matrix = transform.flatten().tolist()
        return response
    


def main():
    rclpy.init()
    node = ICPservice()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()