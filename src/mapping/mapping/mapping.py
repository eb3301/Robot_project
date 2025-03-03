import rclpy
import json
import os
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from custom_msgs.msg import DetectedObject
from custom_msgs.msg import DetectedObjects

class ObjectMapNode(Node):
    def __init__(self):
        super().__init__('object_map_node')

        # Subscribe to detected objects, not raw PointCloud2
        self.subscription = self.create_subscription(
            DetectedObject,
            '/detected_object_info',  # Change topic to filtered object points
            self.detected_objects_callback,
            10
        )

        # Map storage
        self.map_file = 'map_file.json'
        self.object_map = self.load_map()

        self.get_logger().info("Object Map Node Started! loool")

    def load_map(self):
        """Load the map file if it exists, otherwise return an empty dictionary."""
        # Check if the file exists and delete it if it does
        if os.path.exists(self.map_file):
            self.get_logger().info(f"Deleting old map file: {self.map_file}")
            os.remove(self.map_file)  # Delete the old file
        
        # Return an empty dictionary if no map file exists or after deletion
        return {}
        
    def save_map(self):
        """Save the object map to a JSON file"""
        def convert_to_native(obj):
            if isinstance(obj, np.float32):
                return float(obj)  # Convert to native Python float
            elif isinstance(obj, tuple):
                return tuple(convert_to_native(x) for x in obj)
            elif isinstance(obj, list):
                return [convert_to_native(x) for x in obj]
            elif isinstance(obj, dict):
                return {k: convert_to_native(v) for k, v in obj.items()}
            return obj  # Return as is if no conversion is needed

        #self.get_logger().info("Saving map...")
        converted_map = convert_to_native(self.object_map)  # Convert before saving
        with open(self.map_file, 'w') as f:
            json.dump(converted_map, f, indent=4)

    def detected_objects_callback(self, msg: DetectedObject):
        """Store detected object positions in the object map."""
        object_x = msg.x
        object_y = msg.y
        object_z = msg.z
        object_label = msg.label

        # Store the object position using the (x, y, z) as a key
        # We'll store the label as part of the data in the map
        key = f"{round(object_x, 4)}_{round(object_y, 4)}_{round(object_z, 4)}"

        self.get_logger().info(f"Object: {object_label} @ ({object_x}, {object_y}, {object_z})")
        
        if key not in self.object_map:
            self.object_map[key] = {
                'x': object_x,
                'y': object_y,
                'z': object_z,
                'label': object_label
            }

        self.save_map()  # Save after processing

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices using rounding instead of resolution."""
        grid_x = round(x)  # Round to nearest integer
        grid_y = round(y)
        return grid_x, grid_y

def main(args=None):
    rclpy.init(args=args)
    node = ObjectMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
