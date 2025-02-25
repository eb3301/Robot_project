import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import sensor_msgs.msg as sensor_msgs
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import KDTree


class CannyEdgePointCloud(Node):
    def __init__(self):
        super().__init__('canny_edge_pointcloud')
        self.subscription = self.create_subscription(
            sensor_msgs.PointCloud2,
            '/camera/camera/depth/color/points',  # Change to your actual topic
            self.pointcloud_callback,
            10)
        self.publisher = self.create_publisher(sensor_msgs.PointCloud2, '/detected_objects', 10)
        self.bridge = CvBridge()
        self.get_logger().info("Node started")

    def pointcloud_callback(self, msg):
        points = self.pointcloud2_to_array(msg)
        
        if points is None or points.shape[0] == 0:
            self.get_logger().warn("Received empty or invalid point cloud.")
            return
        
        depth_image = self.create_depth_image(points, msg.width, msg.height)
        
        if depth_image is None:
            self.get_logger().warn("Failed to generate depth image.")
            return
        
        # Apply Gaussian blur to reduce noise before edge detection
        depth_image = cv2.GaussianBlur(depth_image, (5, 5), 1.5)

        # Adaptive Canny edge detection
        mean_intensity = np.mean(depth_image)
        lower_threshold = max(0, mean_intensity * 0.06)
        upper_threshold = min(255, mean_intensity * 1.33)
        edges = cv2.Canny(depth_image, lower_threshold, upper_threshold)

        edge_points = self.extract_edge_points(points, edges)
        
        if edge_points.size == 0:
            self.get_logger().warn("No edge points detected.")
            return
        
        edge_pc_msg = self.array_to_pointcloud2(edge_points, msg.header)
        self.publisher.publish(edge_pc_msg)

    def pointcloud2_to_array(self, msg):
        try:
            points = np.array([
                [p[0], p[1], p[2]]
                for p in pc2.read_points(msg, field_names=("x", "z", "y"), skip_nans=False)
            ], dtype=np.float32)

            # Remove NaN points
            points = points[~np.isnan(points).any(axis=1)]

            return points
        except Exception as e:
            self.get_logger().error(f"Error processing PointCloud2: {e}")
            return None

    def create_depth_image(self, points, width, height):
        try:
            depth = points[:, 2]  # Extract Z-coordinates

            # Replace NaNs and normalize depth values
            depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
            depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            # Apply median filter to reduce noise
            depth = cv2.medianBlur(depth, 5)

            # Reshape depth to image size (may require adjustment)
            if width * height == depth.shape[0]:
                return depth.reshape((height, width))
            else:
                self.get_logger().warn(f"Depth size mismatch: {depth.shape[0]} vs {width}x{height}")
                return None
        except Exception as e:
            self.get_logger().error(f"Error creating depth image: {e}")
            return None

    def extract_edge_points(self, points, edges):
        try:
            edge_indices = np.argwhere(edges > 0)

            if len(edge_indices) == 0:
                return np.array([])

            # Ensure valid indexing
            valid_indices = edge_indices[:, 0] < points.shape[0]
            edge_points = points[edge_indices[:, 0][valid_indices]]

            # Filter edge points by depth range
            depth_threshold_min = 0.0  # Min depth (meters)
            depth_threshold_max = 5.0  # Max depth (meters)
            valid_points = (edge_points[:, 2] > depth_threshold_min) & (edge_points[:, 2] < depth_threshold_max)
            edge_points = edge_points[valid_points]

            return edge_points
        except Exception as e:
            self.get_logger().error(f"Error extracting edge points: {e}")
            return np.array([])

    def array_to_pointcloud2(self, points, header):
        try:
            if points.size == 0:
                return None

            # Create structured point cloud
            cloud_msg = pc2.create_cloud_xyz32(header, points)
            cloud_msg.is_dense = True  # Mark as dense
            return cloud_msg
        except Exception as e:
            self.get_logger().error(f"Error converting to PointCloud2: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = CannyEdgePointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
