import math
import numpy as np
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import numpy as np

class Detection(Node):
    def __init__(self):
        super().__init__('detection')

        self._pub = self.create_publisher(PointCloud2, '/detected_objects', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/bounding_boxes', 10)
        
        self.create_subscription(PointCloud2,
             '/camera/camera/depth/color/points', self.cloud_callback, 10)
        
        self.get_logger().info(f"Node started")

    def pointcloud_to_image(self, points):
        """Converts a point cloud to a 2D grayscale depth image."""
        resolution = 0.005  # 5mm per pixel
        x_range = (-1, 1)  # x-axis limits in meters
        z_range = (0, 2)  # z-axis (depth) limits

        img_width = int((x_range[1] - x_range[0]) / resolution)
        img_height = int((z_range[1] - z_range[0]) / resolution)
        depth_image = np.zeros((img_height, img_width), dtype=np.uint8)

        for point in points:
            x, y, z = point
            if x_range[0] <= x <= x_range[1] and z_range[0] <= z <= z_range[1]:
                img_x = int((x - x_range[0]) / resolution)
                img_y = int((z - z_range[0]) / resolution)
                depth_value = int((1 - (y + 0.5)) * 255)  # Normalize depth

                depth_image[img_y, img_x] = depth_value

        return depth_image

    def cloud_callback(self, msg: PointCloud2):
        """Detects objects, creates a depth image, and applies edge detection."""
        header = msg.header
        points = pc2.read_points_numpy(msg, skip_nans=True)[:, :3]

        depth_image = self.pointcloud_to_image(points)

        # Apply Canny edge detection
        edges = cv2.Canny(depth_image, 50, 150)

        # Show the edge-detected image
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)

        # Continue with DBSCAN clustering...
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
