import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from detect_interfaces.srv import DetectObjects
from std_srvs.srv import SetBool, Trigger



class Detection(Node):
    def __init__(self):
        super().__init__('detection')

        # Publishers
        self._pub = self.create_publisher(PointCloud2, '/detected_objects', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/bounding_boxes', 10)
        
        # Service definition
        self.srv = self.create_service(DetectObjects, 'detect_objects', self.detect_objects_callback)

        # Subscriber to the point cloud topic
        self._sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            10
        )

        self.latest_cloud = None
        self.ObjectList = []  # Stores detected objects

        self.active = False  # Nodo inattivo finché non viene avviato

        # Servizi per attivare/disattivare detection
        self.create_service(SetBool, 'start_detection', self.start_detection_callback)
        self.create_service(SetBool, 'stop_detection', self.stop_detection_callback)
        self.create_service(Trigger, 'reset_detected_objects', self.reset_callback)



        self.get_logger().info("Node started and service ready")

    def detect_objects_callback(self, request, response):
        """Service callback that returns the detected objects and their positions."""
        self.get_logger().info("Detecting objects...")

        if not self.ObjectList:
            self.get_logger().warn("No detected objects available.")
            return response  # Return empty response if no objects detected

        # Populate response with object types and positions
        response.object_types = []
        response.object_positions = []

        for obj in self.ObjectList:
            obj_type, obj_position = obj
            response.object_types.append(obj_type)

            # position = Point()
            # position.x, position.y, position.z = obj_position
            position = Point()
            position.x = float(obj_position[0])
            position.y = float(obj_position[1])
            position.z = float(obj_position[2])

            response.object_positions.append(position)

        return response

    def cloud_callback(self, msg: PointCloud2):
        """
        Processes the latest point cloud and stores detected objects.
        Detects objects using DBSCAN clustering, volume and shape detection, and publishes bounding boxes.
        """
        if not self.active:
            return  # Ignora frame se il nodo è disattivato

        self.latest_cloud = msg  # Store latest cloud
        points = pc2.read_points_numpy(msg, skip_nans=True)[:, :3]

        distances = np.linalg.norm(points[:, :3], axis=1)
        offset = 0.089
        mask = (distances <= 1.5) & (points[:, 1] < offset) & (points[:, 1] > offset - 0.3)
        filtered_indices = np.where(mask)[0]
        filtered_points = points[mask]

        if filtered_points.shape[0] == 0:
            self.get_logger().info("No points after filtering")
            return

        db = DBSCAN(eps=0.15, min_samples=70)
        labels = db.fit_predict(filtered_points)

        _, counts = np.unique(labels, return_counts=True)
        max_cluster_size = 8000
        filt_labels = np.array([
            -1 if counts[label] > max_cluster_size else label   
            for label in labels
        ])

        detected_indices = []
        unique_labels = set(filt_labels)

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
            bbox_center = (bbox_min + bbox_max) / 2
            volume = np.prod(bbox_size)

            x_lim = 0.35
            x_min, x_max = -x_lim, x_lim
            if (bbox_min[0] < x_min or bbox_max[0] > x_max):
                continue

            obj_type = "trash"  # Default category

            if volume < 0.00012:  # Small objects (cube, sphere)
                curvatures = []
                for p in cluster_points:
                    cov_matrix = np.cov(cluster_points.T)
                    if np.any(np.isnan(cov_matrix)) or np.any(np.isinf(cov_matrix)):
                        continue
                    eigenvalues, _ = np.linalg.eig(cov_matrix)
                    curvature = eigenvalues.min() / np.sum(eigenvalues)
                    curvatures.append(curvature)

                avg_curvature = np.mean(curvatures) if curvatures else 0

                if avg_curvature > 0.08:
                    obj_type = "Cube"
                else:
                    obj_type = "Sphere"

            elif volume < 0.002:
                obj_type = "Fluffy_animal"
            elif volume < 0.01:
                obj_type = "Box"

            # Store detected object
            if not self.ObjectList:
                self.ObjectList.append([obj_type, np.mean(cluster_points, axis=0)])
            else:
                new_obj_position = np.mean(cluster_points, axis=0)
                should_add = True

                for obj in self.ObjectList:
                    obj_position = obj[1]  # Prendi solo la posizione dell'oggetto
                    if np.linalg.norm(new_obj_position - obj_position) < 0.2:
                        should_add = False
                        break

                if should_add:
                    self.ObjectList.append([obj_type, new_obj_position])
            self.get_logger().info(f'Detected {len(self.ObjectList)} objects from the detection node start')

            detected_indices.append(cluster_indices)

        self.publish_detected_objects(detected_indices, msg)
        self.publish_bounding_boxes(detected_indices, points, msg.header)

    def publish_detected_objects(self, clusters, original_msg):
        """Publishes detected objects as PointCloud2."""
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

    def start_detection_callback(self, request, response):
        self.active = True
        #self.ObjectList = []  # Riparti pulito se vuoi
        self.get_logger().info("Detection attivata.")
        response.success = True
        response.message = "Detection avviata."
        return response

    def stop_detection_callback(self, request, response):
            self.active = False
            self.get_logger().info("Detection disattivata.")
            response.success = True
            response.message = "Detection fermata."
            return response
    
    def reset_callback(self, request, response):
            self.ObjectList = []
            response.success = True
            response.message = "Lista oggetti rilevati svuotata."
            return response

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
