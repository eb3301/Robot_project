#!/usr/bin/env python
import rclpy
import rclpy.duration
import rclpy.logging
from rclpy.node import Node
import rclpy.time
import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import tf2_geometry_msgs 

from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from laser_geometry import LaserProjection
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from std_msgs.msg import String
from tf2_geometry_msgs import Pose

from tf_transformations import euler_from_quaternion, quaternion_from_matrix
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
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
        self.ref_msg_sub = self.create_subscription(String, '/ref_msg', self.ref_msg_callback, 1)


        # Map Pose
        odom_pose_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE, 
            depth = 1
        )
        self.odom_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'ekf_pose', self.odom_pose_callback, odom_pose_qos)
        
        self.map_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'map_pose', lidar_qos)
        self.map_path_pub = self.create_publisher(Path, 'map_path', 10)


        # TF2 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread = True)

        # Timers
        self.broadcast_timer = self.create_timer(0.05, self.broadcast_transform)

        # Initialise Cloud Variables
        self.target_pcd_list = []  

        self.transform = np.identity(4)

        # Reference Cloud Variables
        self.proj = LaserProjection()
        self.counter = 0
        self.ref_counter = 0
        self.start_time = None
        self.create_ref = True
        self.accumulated_scans = []
        self.pub_ref = True
        self.fitness_counter = 0

        self.stamp = None
        self.rotating = False
        
        self.pose = Pose()
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.position.z = 0.0
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = 1.0 

        self.path = Path()

        self.n_ref_clouds = 0

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
            #self.get_logger().info(
                #f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')     
            return
        
        # Filter scans based on range
        min_range, max_range = 0.25, 3
        msg.ranges = [r if max_range >= r >= min_range else float('nan') for r in msg.ranges]

        # Project lidar to point cloud
        cloud = self.proj.projectLaser(msg)

        # Transform point cloud
        cloud_out = do_transform_cloud(cloud, t)

        # Extract points 
        points = pc2.read_points_numpy(cloud_out, field_names=("x", "y", "z"), skip_nans=True)

        # If it is time to create a reference cloud
        if not self.rotating:
            if self.pub_ref:
                # Create reference pointcloud for ICP 
                # First reference cloud it accumulates N scans. Other clouds it uses only 1
                if self.ref_counter == 10:
                    merged_points = np.vstack(self.accumulated_scans)
                    merged_ref = pc2.create_cloud_xyz32(cloud_out.header, merged_points)
                    self.target_pcd_list.append( (self.pointcloud_2_open3d(merged_ref), self.pose) )
                    self.pub_ref = False
                    self.ref_counter = 8
                else:
                    self.ref_counter += 1
                    self.accumulated_scans.append(points)
                    self.ref_cloud_pub.publish(cloud_out)
            else:
                if self.counter % 4 == 0:
                    self.counter += 1
                    self.lidar_pub.publish(cloud_out)
                    source_pcd = self.pointcloud_2_open3d(cloud_out)
                    self.ICP(source_pcd)
                else:
                    self.counter += 1

    def ref_msg_callback(self, msg):
        self.pub_ref = True


    def odom_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.publish_pose(msg)


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
        # return
        '''ICP Algorithm Implementation'''
        if source_pcd is None or not self.target_pcd_list:
            return
        start_time = time.time()

        # Determine which point cloud to use
        if len(self.target_pcd_list) == 1:
            target_pcd = self.target_pcd_list[0][0]
        else:
            curr_pos = np.array([self.pose.position.x, self.pose.position.y])

            # Compute distances to all clouds
            distances = []
            for i, (_, cloud_pose) in enumerate(self.target_pcd_list):  
                cloud_pos = np.array([cloud_pose.position.x, cloud_pose.position.y])
                dist = np.linalg.norm(curr_pos - cloud_pos)
                distances.append((dist, i))

            # Find the closest cloud
            closest_index = min(distances, key=lambda x: x[0])[1]
            #sself.get_logger().info(f'Using reference cloud {closest_index} at distance {distances[closest_index][0]}')

            target_pcd = self.target_pcd_list[closest_index][0]             

        # Downsample clouds using Voxel Filter
        voxel_size = 0.02  
        source_pcd = source_pcd.voxel_down_sample(voxel_size)
        target_pcd = target_pcd.voxel_down_sample(voxel_size)

        # Compute normals for Point-to-Plane
        radius = 0.10 # max range for neighbour search
        max_nn = 10 # max amount of neighbours

        source_pcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))  
        target_pcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn))  

        
        # Algorithm Parameters
        threshold = 0.05 # Max distance to be a 'match'
        trans_init = self.transform.copy() # Initial transformation estimate

        #Convergence Criterias
        max_iteration, relative_fitness, relative_rmse = 200, 1e-6, 1e-6
        # Run algorithm 
        result = o3d.pipelines.registration.registration_icp(
            source_pcd, target_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness, relative_rmse, max_iteration)
        )

        translation = result.transformation[:3, 3]
        dist = np.linalg.norm(translation)
        if result.fitness > 0.02 and result.inlier_rmse < 0.04 and dist < 0.5: 
            self.fitness_counter = 0
            self.transform = result.transformation

            #self.get_logger().info(f"RUNNING ICP: \n Distance moved: {dist} \n fitness: {result.fitness} \n Inlier rmse: {result.inlier_rmse}")

            end_time = time.time()
            #self.get_logger().info(f"ICP algorithm took {end_time - start_time:.6f} seconds")
        else:
            #self.get_logger().info(f"IGNORING ICP: \n Distance moved: {dist} \n fitness: {result.fitness} \n Inlier rmse: {result.inlier_rmse} ") 
            self.fitness_counter += 1
            
            if self.fitness_counter == 3:
                min_distance = float('inf')
                curr_pos = np.array([self.pose.position.x, self.pose.position.y])

                # Check distance to all reference clouds
                for _, cloud_pose in self.target_pcd_list:
                    cloud_pos = np.array([cloud_pose.position.x, cloud_pose.position.y])
                    dist = np.linalg.norm(curr_pos - cloud_pos)
                    min_distance = min(min_distance, dist)

                if min_distance < 1.5:  # Adjust threshold as needed
                    # self.get_logger().info(f"Skipping new reference cloud: closest existing cloud is {min_distance:.2f}m away")
                    pass
                else:
                    pass
                    self.pub_ref = True
                    # self.get_logger().info(f"Creating new reference cloud: nearest cloud was {min_distance:.2f}m away")

                self.fitness_counter = 0

    
    def publish_pose(self, pose_msg):
            stamp = pose_msg.header.stamp
            #time = rclpy.time.Time().from_msg(stamp)
            time = rclpy.time.Time(seconds = 0)
            try:
                tf_future = self.tf_buffer.wait_for_transform_async(
                target_frame = "map",
                source_frame = 'odom',
                time = time
                )
            
                rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

                base_2_odom_tf = self.tf_buffer.lookup_transform(
                    target_frame = "map", 
                    source_frame = 'odom', 
                    time = time,
                    timeout = rclpy.duration.Duration(seconds=0.1)
                )
                
                map_pose = tf2_geometry_msgs.do_transform_pose(pose_msg.pose.pose, base_2_odom_tf)

                # Pose
                pose_msg.pose.pose = map_pose
                pose_msg.header.frame_id = 'map'
                self.map_pose_pub.publish(pose_msg)

                # Path
                PathPose = PoseStamped()
                PathPose.header = pose_msg.header
                PathPose.pose = pose_msg.pose.pose
                self.path.poses.append(PathPose) 

                self.path.header.stamp = stamp
                self.path.header.frame_id = 'map'    
                self.map_path_pub.publish(self.path)

                self.pose = map_pose

            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {'odom'} to {"map"}: {ex}')



    def broadcast_transform(self):
        if not self.stamp:
            return
        
        t_icp = self.transform
        t = TransformStamped()
        #self.get_logger().info("Broadcasting transform...")
        t.header.stamp = self.stamp
        t.header.frame_id = 'map' 
        t.child_frame_id = 'odom' 
        
        t.transform.translation.x = t_icp[0,3]
        t.transform.translation.y = t_icp[1,3]
        t.transform.translation.z = t_icp[2,3]

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
