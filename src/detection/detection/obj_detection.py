import math
import numpy as np
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN


class Detection(Node):
    def __init__(self):
        super().__init__('detection')

        self._pub = self.create_publisher(PointCloud2, '/detected_objects', 10)
        
        self.create_subscription(PointCloud2,
             '/camera/camera/depth/color/points', self.cloud_callback, 10)
        
        self.get_logger().info(f"node started")

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
        """Detects objects using DBSCAN clustering from scikit-learn."""
        # self.get_logger().info(f"start callback")
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]  # Extract XYZ

        # **Filter points based on distance**
        distances = np.linalg.norm(points[:, :2], axis=1)  # XY distance
        mask = (distances <= 1) & (points[:, 2] > 0)  # Within 1m and above ground
        filtered_points = points[mask]
        # self.get_logger().info(f"mask applied")

        if filtered_points.shape[0] == 0:
            self.get_logger().info(f"No points after filtering")
            return

        # Downsample using voxel grid filter
        filtered_points = self.voxel_grid_filter(filtered_points, leaf_size=0.1)  
        
        # **Step 1: DBSCAN Clustering**
        # self.get_logger().info(f"proceed with DBSCAN")
        db = DBSCAN(eps=0.5, min_samples=10)  # eps defines the neighborhood size, min_samples defines minimum points per cluster
        labels = db.fit_predict(filtered_points)
        # self.get_logger().info(f"done")

        detected_objects = []
        classified_labels = []
       
        unique_labels = set(labels)
        # self.get_logger().info(f"unique labels: {unique_labels}")
        
        if len(unique_labels) > 0:
            for label in unique_labels:
                if label == -1:
                    continue  # Ignore noise (DBSCAN labels noise as -1)

                # Get points belonging to the current cluster
                cluster_points = filtered_points[labels == label]

                if cluster_points.shape[0] < 10:
                    self.get_logger().info(f"Neglecting small cluster")
                    continue  # Ignore small clusters

                # **Step 2: Compute Bounding Box**
                bbox_min = np.min(cluster_points, axis=0)
                bbox_max = np.max(cluster_points, axis=0)
                bbox_size = bbox_max - bbox_min
                volume = np.prod(bbox_size)

                # **Step 3: Classify Objects**
                if volume < 0.004:  # Small object (Cube/Sphere)
                    self.get_logger().info(f'Detected Cube or Sphere at {np.mean(cluster_points, axis=0)}')
                    classified_labels.append(1)  # 1 for small objects
                else:  # Larger object (Box)
                    self.get_logger().info(f'Detected Large Box at {np.mean(cluster_points, axis=0)}')
                    classified_labels.append(2)  # 2 for large objects

                detected_objects.append(cluster_points)
        else:
                self.get_logger().info("No labels found, no clusters to process.")

        # **Step 4: Publish Detected Objects**
        self.publish_detected_objects(detected_objects, classified_labels, msg.header)

    def publish_detected_objects(self, clusters, labels, header):
        """Publishes detected objects as PointCloud2."""
        detected_points = []
        colors = []

        for i, cluster in enumerate(clusters):
            for point in cluster:
                detected_points.append(point)
                colors.append([255, 0, 0] if labels[i] == 1 else [0, 255, 0])  # Red = Cube/Sphere, Green = Box

        if not detected_points:
            return

        detected_points = np.array(detected_points)
        colors = np.array(colors)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='b', offset=14, datatype=PointField.UINT8, count=1)
        ]

        message_data = [struct.pack('fffBBB', *detected_points[i], *colors[i]) for i in range(len(detected_points))]

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(detected_points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 15
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b''.join(message_data)

        self._pub.publish(msg)

def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
