#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped, PoseStamped

import numpy as np
import open3d as o3d

class ICPNode(Node):
    def __init__(self):
        super().__init__('ICP_Node')

        # ROS Topics
        self.ref_sub = self.create_subscription(PointCloud2, '/Ref_PointCloud', self.ref_callback, 10)
        self.pointcloud = self.create_subscription(PointCloud2, '/lidar', self.cloud_callback, 10)
        
        #TF2 Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialise Cloud Variables
        self.source_pcd = None
        self.target_pcd = None  

        self.current_time = 0.0
        

    def ref_callback(self, msg: PointCloud2):
        self.target_pcd = self.pointcloud_2_open3d(msg)


    def cloud_callback(self, msg: PointCloud2):
        self.current_time = rclpy.time.Time().from_msg(msg.header.stamp)
        self.source_pcd = self.pointcloud_2_open3d(msg)
        if self.target_pcd is not None:
            self.ICP()


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
        self.get_logger().info('Running ICP algorithm...')

        # Compute normals for Point-to-Plane
        radius = 0.1 # max range for neighbour search
        max_nn = 30 # max amount of neighbours

        self.target_pcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius ,max_nn))  
        
        # Algorithm Parameters
        threshold = 0.02 # Max distance to be a 'match'
        trans_init = self.pose # Initial estimate

        #Convergence Criterias
        max_iteration, relative_fitness, relative_rmse = 50, 1e-6, 1e-6

        # Run algorithm 
        result = o3d.pipelines.registration.registration_icp(
            self.source_pcd, self.target_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration, relative_fitness, relative_rmse)
        )

        transform = result.transformation
        self.publish_tf(transform)
        self.get_logger().info('Recieved transformation from ICP algorithm')
        

    def publish_tf(self, t_icp):
        t = TransformStamped()
        t.header.stamp = self.current_time
        t.header.frame_id = 'icp_odom' 
        t.child_frame_id = 'base_linK'

        t.transform.translation.x = t_icp[0, 3]
        t.transform.translation.y = t_icp[1, 3]
        t.transform.translation.z = t_icp[2, 3]

        qx, qy, qz, qw = quaternion_from_euler(*euler_from_quaternion(t_icp[:3, :3]))
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published ICP transform')


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