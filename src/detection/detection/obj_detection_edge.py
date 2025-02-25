import math
import numpy as np
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import KDTree
import cv2

class Detection(Node):
    def __init__(self):
        super().__init__('detection')

        self._pub = self.create_publisher(PointCloud2, '/detected_objects', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/bounding_boxes', 10)
        
        self.create_subscription(PointCloud2,
             '/camera/camera/depth/color/points', self.cloud_callback, 10)
        
        self.get_logger().info(f"Node started")
        self._pub_edges = self.create_publisher(PointCloud2, '/edge_points', 10)

    def cloud_callback(self, msg: PointCloud2):
        """Detects objects using DBSCAN clustering and applies edge detection."""
        header = msg.header
        points = pc2.read_points_numpy(msg, skip_nans=True)[:, :3]

        # Filter points based on distance and height
        distances = np.linalg.norm(points[:, :3], axis=1)
        offset = 0.09
        mask = (distances <= 1) & (points[:, 1] < offset) & (points[:, 1] > offset - 0.3)
        filtered_indices = np.where(mask)[0]
        filtered_points = points[mask]

        if filtered_points.shape[0] == 0:
            self.get_logger().info("No points after filtering")
            return

        # Detect edges
        edge_indices = self.depth_gradient_edge_detection(filtered_points)

        # Apply DBSCAN for clustering
        db = DBSCAN(eps=0.1, min_samples=70)
        labels = db.fit_predict(filtered_points)

        detected_clusters = []
        classified_labels = []
        unique_labels = set(labels)

        for label in unique_labels:
            if label == -1:
                continue  # Ignore noise

            cluster_mask = labels == label
            cluster_points = filtered_points[cluster_mask]
            cluster_indices = filtered_indices[cluster_mask]

            # Ensure the cluster is large enough
            if cluster_points.shape[0] < 10:
                continue

            # Compute bounding box
            bbox_min = np.min(cluster_points, axis=0)
            bbox_max = np.max(cluster_points, axis=0)
            bbox_size = bbox_max - bbox_min
            volume = np.prod(bbox_size)

            # Classify detected objects based on volume
            if volume < 0.002:
                self.get_logger().info(f'Detected Small Object at {np.mean(cluster_points, axis=0)}')
                classified_labels.append(1)
            elif volume < 0.01:
                self.get_logger().info(f'Detected Large Object at {np.mean(cluster_points, axis=0)}')
                classified_labels.append(2)

            detected_clusters.append(cluster_indices)

        # Publish detected objects with edge refinement
        self.publish_detected_objects(detected_clusters, msg)
        self.publish_bounding_boxes(detected_clusters, points, msg.header)

        # Publish edges as a separate topic (optional)
        self.publish_edges(edge_indices, points, msg.header)

    """    
    def cloud_callback(self, msg: PointCloud2):
        #Detects objects using DBSCAN clustering and publishes bounding boxes.
        header = msg.header
        points = pc2.read_points_numpy(msg, skip_nans=True)[:, :3]

        distances = np.linalg.norm(points[:, :3], axis=1)
        offset = 0.09
        mask = (distances <= 1) & (points[:, 1] < offset) & (points[:, 1] > offset - 0.3)
        filtered_indices = np.where(mask)[0]
        filtered_points = points[mask]

        if filtered_points.shape[0] == 0:
            self.get_logger().info("No points after filtering")
            return

        db = DBSCAN(eps=0.1, min_samples=70)
        labels = db.fit_predict(filtered_points)

        detected_indices = []
        classified_labels = []
        unique_labels = set(labels)
        
        for label in unique_labels:
            if label == -1:
                continue

            cluster_mask = labels == label
            cluster_points = filtered_points[cluster_mask]
            cluster_indices = filtered_indices[cluster_mask]
            
            if cluster_points.shape[0] < 10:
                continue

            bbox_min = np.min(cluster_points, axis=0)
            bbox_max = np.max(cluster_points, axis=0)
            bbox_size = bbox_max - bbox_min
            volume = np.prod(bbox_size)

            if volume < 0.002:
                self.get_logger().info(f'Detected Objects at {np.mean(cluster_points, axis=0)}')
                classified_labels.append(1)
            elif volume < 0.01:
                self.get_logger().info(f'Detected Large Box at {np.mean(cluster_points, axis=0)}')
                classified_labels.append(2)
            
            detected_indices.append(cluster_indices)

        self.publish_detected_objects(detected_indices, msg)
        self.publish_bounding_boxes(detected_indices, points, msg.header)
        """

    def depth_gradient_edge_detection(self, points, grid_size=(200, 200)):

        """Project points to a 2D depth map and apply Sobel edge detection."""
        
        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])
        
        # Create an empty depth map
        depth_map = np.zeros(grid_size, dtype=np.float32)
        point_indices = np.full(grid_size, -1, dtype=int)  # Track original indices

        # Fill the depth map
        for i, p in enumerate(points):
            x_idx = int((p[0] - x_min) / (x_max - x_min) * (grid_size[0] - 1))
            z_idx = int((p[2] - z_min) / (z_max - z_min) * (grid_size[1] - 1))
            
            if depth_map[x_idx, z_idx] == 0 or p[1] < depth_map[x_idx, z_idx]:  
                depth_map[x_idx, z_idx] = p[1]  # Store min depth (closer points)
                point_indices[x_idx, z_idx] = i  # Store index of corresponding point
        
        # Apply Sobel edge detection
        sobel_x = cv2.Sobel(depth_map, cv2.CV_64F, 1, 0, ksize=5)
        sobel_y = cv2.Sobel(depth_map, cv2.CV_64F, 0, 1, ksize=5)
        edges = np.sqrt(sobel_x**2 + sobel_y**2)

        # Select top 5% of edges
        edge_threshold = np.percentile(edges, 30)
        edge_pixels = np.argwhere(edges > edge_threshold)

        # Map edge pixels back to 3D points
        edge_indices = [point_indices[x, z] for x, z in edge_pixels if point_indices[x, z] != -1]

        return np.array(edge_indices)


    def publish_detected_objects(self, clusters, original_msg):
        """Publishes detected objects as PointCloud2 using original indices."""
        if not clusters:
            return

        selected_indices = np.concatenate(clusters) if clusters else []
        selected_data = bytearray()
        point_step = original_msg.point_step

        for idx in selected_indices:
            offset = idx * point_step
            selected_data.extend(original_msg.data[offset:offset + point_step])

        new_msg = PointCloud2()
        new_msg.header = original_msg.header
        new_msg.height = 1
        new_msg.width = len(selected_indices)
        new_msg.fields = original_msg.fields
        new_msg.is_bigendian = original_msg.is_bigendian
        new_msg.point_step = original_msg.point_step
        new_msg.row_step = new_msg.point_step * new_msg.width
        new_msg.is_dense = True
        new_msg.data = bytes(selected_data)

        self._pub.publish(new_msg)

    def publish_bounding_boxes(self, clusters, points, header):
        """Publishes bounding boxes as MarkerArray."""
        marker_array = MarkerArray()

        for i, cluster_indices in enumerate(clusters):
            cluster_points = points[cluster_indices]
            bbox_min = np.min(cluster_points, axis=0)
            bbox_max = np.max(cluster_points, axis=0)
            bbox_size = bbox_max - bbox_min
            bbox_center = (bbox_min + bbox_max) / 2

            marker = Marker()
            marker.header = header
            marker.ns = "bounding_boxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(bbox_center[0])
            marker.pose.position.y = float(bbox_center[1])
            marker.pose.position.z = float(bbox_center[2])
            marker.scale.x = float(bbox_size[0])
            marker.scale.y = float(bbox_size[1])
            marker.scale.z = float(bbox_size[2])
            marker.color.a = 0.5  # Semi-transparent
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime.sec = 1  # Auto-delete if not updated

            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)

    def publish_edges(self, edge_indices, points, header):
        """Publishes detected edges as a PointCloud2 message for visualization."""
        if not len(edge_indices):
            return

        edge_points = points[edge_indices]  # Extract edge points

        edge_msg = PointCloud2()
        edge_msg.header = header
        edge_msg.height = 1
        edge_msg.width = len(edge_points)
        edge_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        edge_msg.is_bigendian = False
        edge_msg.point_step = 12  # 3 floats * 4 bytes each
        edge_msg.row_step = edge_msg.point_step * edge_msg.width
        edge_msg.is_dense = True
        edge_msg.data = struct.pack(f'{len(edge_points) * 3}f', *edge_points.flatten())

        self._pub_edges.publish(edge_msg)

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
