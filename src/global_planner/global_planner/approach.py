import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_matrix, quaternion_from_euler, euler_from_quaternion
import tf2_geometry_msgs
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist


class Approaching(Node):
    def __init__(self):
        super().__init__('detection')

        # Publishers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  
            depth=10
            ) 

        # Subscriber to the point cloud topic
        self._sub = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)
        # Subscriber to Odom topic
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.pose_callback, 10)
        
        # Publisher to velocity command topic
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        


        # Call control algorithm
        self.create_timer(0.2, self.control)  # ogni 100ms


        # Initialize the transform buffer
        self.tf_buffer = Buffer()
        # Initialize the transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x = 0
        self.y = 0
        self.distance_to_target = None
    
    def pose_callback(self, msg : PoseWithCovarianceStamped):
        # Init transform
        to_frame_rel = 'map'
        from_frame_rel = 'base_link'
        time = rclpy.time.Time().from_msg(msg.header.stamp)

        # Wait for the transform asynchronously
        tf_future = self.tf_buffer.wait_for_transform_async(
        target_frame=to_frame_rel,
        source_frame=from_frame_rel,
        time=time
        )
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        # Lookup tansform
        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel,
                                            from_frame_rel,
                                            time)
            # Do the transform
            map_pose = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, t)

            # Get position of robot
            self.x = map_pose.position.x
            self.y = map_pose.position.y
            
            q = [
                map_pose.orientation.x,
                map_pose.orientation.y,
                map_pose.orientation.z,
                map_pose.orientation.w
            ]
            angles = euler_from_quaternion(q)
            self.theta=angles[2]
        except TransformException:
            self.get_logger().info('No transform found')

    def voxel_grid_filter(self, points, leaf_size=0.05):
        """Downsamples the point cloud using a voxel grid filter."""
        # Create voxel grid by downsampling points to grid size
        # Points are binned by floor division on leaf size.
        grid_indices = np.floor(points[:, :3] / leaf_size).astype(int)
        unique_grid_indices = np.unique(grid_indices, axis=0)
        
        # For each grid cell, compute the centroid
        downsampled_points = []
        for idx in unique_grid_indices:
            mask = np.all(grid_indices == idx, axis=1)
            points_in_cell = points[mask]
            centroid = np.mean(points_in_cell, axis=0)
            downsampled_points.append(centroid)

        return np.array(downsampled_points)
    
    def cloud_callback(self, msg: PointCloud2):
        """
        Processes the latest point cloud and stores detected objects.
        Detects objects using DBSCAN clustering, volume and shape detection, and publishes bounding boxes.
        """
        
        # Transformation
        to_frame_rel = 'map'
        from_frame_rel = msg.header.frame_id

        time = rclpy.time.Time().from_msg(msg.header.stamp)

        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                time)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        

        # Cloud Processing
        self.latest_cloud = msg  # Store latest cloud
        points = pc2.read_points_numpy(msg, skip_nans=True)[:, :3]
        colors = pc2.read_points_numpy(msg, skip_nans=True)[:, 3]

        distances = np.linalg.norm(points[:, :3], axis=1)
        offset = 0.089
        mask = (distances <= 1) & (distances >= 0.2) & (points[:, 1] < offset) & (points[:, 1] > offset - 0.3)
        filtered_points = points[mask]
        filtered_distances=distances[mask]

        if filtered_points.shape[0] == 0:
            # self.get_logger().info("No points after filtering")
            return
        
        # Stimiamo il piano del pavimento dai punti più bassi (es. y vicino a 0)
        floor_mask = filtered_points[:, 1] < (offset - 0.25)  # Prendiamo i più bassi
        floor_points = filtered_points[floor_mask]

        if floor_points.shape[0] > 50:
            # Fit piano al pavimento con RANSAC
            from sklearn.linear_model import RANSACRegressor

            # Piano z = ax + by + c (y è asse verticale in ROS)
            X = floor_points[:, [0, 2]]
            y = floor_points[:, 1]
            ransac = RANSACRegressor(residual_threshold=0.01, max_trials=100)
            ransac.fit(X, y)
            a, b = ransac.estimator_.coef_
            c = ransac.estimator_.intercept_

            # Normale al piano del pavimento
            floor_normal = np.array([-a, 1.0, -b])
            floor_normal /= np.linalg.norm(floor_normal)
        else:
            floor_normal = np.array([0, 1, 0])  # fallback a verticale pura

        # Downsample using voxel grid filter
        #filtered_points = self.voxel_grid_filter(filtered_points)  
        
        # Clustering
        db = DBSCAN(eps=0.015, min_samples=80)
        labels = db.fit_predict(filtered_points)

        _, counts = np.unique(labels, return_counts=True)
        max_cluster_size = 8000

        filt_labels = np.array([
            -1 if counts[label] > max_cluster_size else label   
            for label in labels
        ])

        updated_target = False 

        unique_labels = set(filt_labels)

        for label in unique_labels:
            if label == -1:
                continue

            cluster_mask = labels == label
            cluster_points = filtered_points[cluster_mask]
            cluster_distances=filtered_distances[cluster_mask]
            cluster_distance=np.mean(cluster_distances)

            #self.get_logger().info(f"Cluster #{label} → {cluster_points.shape[0]} punti")

            if np.sum(cluster_points[:,1] < (offset - 0.125)) > 10: #skip cluster if too high (20 points are above the max)
                continue

            if cluster_points.shape[0] < 10 and cluster_points.shape[0] > max_cluster_size:
                continue

            if cluster_points.shape[0] > 3:
                centroid = np.mean(cluster_points, axis=0)
                centered = cluster_points - centroid
                centered = self.voxel_grid_filter(centered)

                # Safety check: SVD needs at least 3 points in 3D
                if centered.shape[0] >= 3 and centered.shape[1] == 3:
                    try:
                        _, _, vh = np.linalg.svd(centered)
                        cluster_normal = vh[2, :]
                        cluster_normal /= np.linalg.norm(cluster_normal)

                        cos_angle = np.dot(cluster_normal, floor_normal)
                        angle = np.arccos(np.clip(abs(cos_angle), -1.0, 1.0)) * 180 / np.pi

                        if angle < 50:
                            continue
                    except np.linalg.LinAlgError as e:
                        self.get_logger().warn(f"SVD fallita per cluster con shape {centered.shape}: {e}")
                        continue
                else:
                    self.get_logger().warn(f"Cluster saltato: shape dopo voxel filtering = {centered.shape}")
                    continue


            bbox_min = np.min(cluster_points, axis=0)
            bbox_max = np.max(cluster_points, axis=0)
            bbox_size = bbox_max - bbox_min
            bbox_center = (bbox_min + bbox_max) / 2

            #self.get_logger().info('Deteced')
            
            x_lim = 0.5 # to skip everithing not centered
            x_min, x_max = -x_lim, x_lim
            if (bbox_min[0] < x_min or bbox_max[0] > x_max):
                continue        

            bbox_pose = PoseStamped()
            bbox_pose.header.frame_id = msg.header.frame_id
            bbox_pose.header.stamp = msg.header.stamp
            bbox_pose.pose.position.x = bbox_center[0]
            bbox_pose.pose.position.y = bbox_center[1]
            bbox_pose.pose.position.z = bbox_center[2]
            bbox_pose.pose.orientation.w = 1.0

            try:
                map_pose = tf2_geometry_msgs.do_transform_pose(bbox_pose.pose, t)
                x_obj = map_pose.position.x
                y_obj = map_pose.position.y

                x = x_obj - self.x
                y = y_obj - self.y

                self.desired_angle = math.atan2(y, x)
                self.distance_to_target = math.sqrt(x**2 + y**2)

                updated_target = True

            except TransformException as ex:
                self.get_logger().warn(f"Trasformazione bbox fallita: {ex}")
                return            
        
        if not updated_target and self.distance_to_target is not None:
            self.get_logger().info("Nessun oggetto valido rilevato. Continuo verso l'ultimo target.")




    def transform_to_matrix(self,t):
        trans = t.transform.translation
        rot = t.transform.rotation

        translation = np.array([trans.x, trans.y, trans.z])
        quaternion = [rot.x, rot.y, rot.z, rot.w]

        transform_mat = quaternion_matrix(quaternion)
        transform_mat[0:3, 3] = translation
        return transform_mat
    
    def control(self):

        if not hasattr(self, 'theta') or not hasattr(self, 'desired_angle'):
            return

        angular_error = math.atan2(math.sin(self.theta - self.desired_angle),
                           math.cos(self.theta - self.desired_angle))
        
        angular_threshold=math.radians(15)

 
        wheel_radius = 0.046 # 0.04915
        base = 0.3 # 0.30
        max_factor = 1 / 6

        max_vel = wheel_radius * max_factor # m/s
        max_rot = ((wheel_radius / base) / (np.pi/2)) * max_factor # rad/s

        kw=50
        kv=0.9

        v=0
        w=0

        # self.get_logger().info(f'Angular: {angular_error}')
        if self.distance_to_target is not None and self.distance_to_target < 0.05:
            v = 0.0
            w = 0.0
        # elif abs(angular_error) > angular_threshold:
        #     w=kw*angular_error
        #     v=0
        # else:
        #     w=0 #-kw*angular_error*max_rot
        #     v=kv*max_vel

        w=kw*angular_error
        v=kv*max_vel

        w=max(min(w,max_rot),-max_rot)
        v=max(min(v,max_vel),-max_vel)

        # Create a ROS Twist message
        twist_msg = Twist()
        twist_msg.linear.x = float(v)
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = float(max_factor)
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = float(w)
        self.cmd_vel_pub.publish(twist_msg)
        

        

        
def main():
    rclpy.init()
    node = Approaching()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
