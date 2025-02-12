import rclpy
import json
import os
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
class ObjectMapNode(Node):
    def __init__(self):
        super().__init__('object_map_node')        # Subscribe to /detected_objects topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.detected_objects_callback,
            10
        )        # Map storage
        self.map_file = 'map.json'
        self.object_map = self.load_map()        
        self.get_logger().info("Object Map Node Started!")    
    
    def load_map(self):
        """Load the map file if it exists, otherwise return an empty dictionary"""
        self.get_logger().info("loading")
        try:
            with open(self.map_file, 'r') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}    
    
    def save_map(self):
        """Save the object map to a JSON file"""
        self.get_logger().info("saving")
        with open(self.map_file, 'w') as f:
            json.dump(self.object_map, f, indent=4)        
            abs_path = os.path.abspath(self.map_file)
        self.get_logger().info(f"Map saved at: {abs_path}")    
    
    def detected_objects_callback(self, msg):
        """Callback function to process detected objects from PointCloud2"""
        detected_objects = []
        self.get_logger().info("callback")        
        field_names = [field.name for field in msg.fields]
        self.get_logger().info(f"Available fields: {field_names}")
        self.get_logger().info(f"Point step: {msg.point_step}")
        if "rgb" in field_names:
            gen = pc2.read_points_numpy(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        else:
            gen = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)        
            points = gen[:, :4]        # Parse the PointCloud2 message
        for n, point in enumerate(points):
            x, y, z = point[:3]  # Extract x, y, z
            rgb = point[3] if "rgb" in field_names else None  # Only extract RGB if available
            detected_objects.append({
                "x": x,
                "y": y,
                "z": z,
                "rgb": rgb
            })
        self.get_logger().info("adding obj")
        # Update object map
        for obj in detected_objects:
            key = f"{obj['x']:.2f}_{obj['y']:.2f}_{obj['z']:.2f}"  # Unique key for each object
        self.object_map[key] = obj  # Update or add new objects        
        self.save_map()  # Save updated map to file        
        self.get_logger().info(f"Updated map with {len(detected_objects)} objects.")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
