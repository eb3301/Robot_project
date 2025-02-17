import math
import numpy as np
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull


class Detection(Node):
    def __init__(self):
        super().__init__('detection')

        self._pub = self.create_publisher(PointCloud2, '/detected_objects', 10)
        self.create_subscription(PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)

    def cloud_callback(self, msg: PointCloud2):
        """Detect objects based on their area and publish them as PointCloud2."""
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]  # Extract XYZ

        # Filter points based on distance
        distances = np.linalg.norm(points[:, :2], axis=1)  # XY distance
        mask = (distances <= 2) & (points[:, 2] > 0)  # Within 2m and above ground
        filtered_points = points[mask]

        if filtered_points.shape[0] == 0:
            return

        # Cluster Points
        clustering = DBSCAN(eps=0.1, min_samples=10).fit(filtered_points)
        labels = clustering.labels_

        detected_objects = []
        classified_labels = []  # Stores classification (Cube/Sphere or Box)

        for cluster_id in np.unique(labels):
            if cluster_id == -1:
                continue  # Ignore noise

            cluster_points = filtered_points[labels == cluster_id]
            if cluster_points.shape[0] < 10:
                continue  # Ignore small clusters

            # Compute Object Size
            hull = ConvexHull(cluster_points)
            bbox_min = np.min(cluster_points, axis=0)
            bbox_max = np.max(cluster_points, axis=0)
            bbox_size = bbox_max - bbox_min
            volume = np.prod(bbox_size)

            # Classify Objects
            if volume < 0.04:  # Small object (Cube/Sphere)
                self.get_logger().info(f'Detected Cube or Sphere at {np.mean(cluster_points, axis=0)}')
                classified_labels.append(1)  # 1 for small objects
            else:  # Larger object (Box)
                self.get_logger().info(f'Detected Large Box at {np.mean(cluster_points, axis=0)}')
                classified_labels.append(2)  # 2 for large objects

            detected_objects.append(cluster_points)

        # **Step 4: Publish Detected Objects as PointCloud2**
        self.publish_detected_objects(detected_objects, classified_labels, msg.header)

    def publish_detected_objects(self, clusters, labels, header):
        """Publishes a PointCloud2 message with detected objects."""

        detected_points = []
        colors = []

        for i, cluster in enumerate(clusters):
            for point in cluster:
                detected_points.append(point)

                if labels[i] == 1:
                    colors.append([255, 0, 0])  # Red for Cube/Sphere
                else:
                    colors.append([0, 255, 0])  # Green for Large Box


        if not detected_points:
            return

        detected_points = np.array(detected_points)
        colors = np.array(colors)

        # Create PointCloud2 message

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='b', offset=14, datatype=PointField.UINT8, count=1)
        ]


        # Pack data
        message_data = []
        for i in range(len(detected_points)):
            message_data.append(struct.pack('fffBBB', 
                                            *detected_points[i], 
                                            *colors[i]))


        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(detected_points)
        msg.fields = fields
        msg.is_bigendian = False

        msg.point_step = 15  # 12 bytes for XYZ + 3 bytes for RGB

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