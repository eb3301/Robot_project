import rclpy
import json
import os
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class ObjectMapNode(Node):
    def __init__(self):
        super().__init__('object_map_node')

        """     
        # Subscribe to PointCloud2
            self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',  # Adjust topic name as needed
            self.detected_objects_callback,
            10
        )
        """

        # Subscribe to detected objects, not raw PointCloud2
        self.subscription = self.create_subscription(
            PointCloud2,
            '/detected_objects',  # Change topic to filtered object points
            self.detected_objects_callback,
            10
        )

        # Map storage
        self.map_file = 'map_file.json'
        self.object_map = self.load_map()

        self.get_logger().info("Object Map Node Started!")

    def load_map(self):
        """Load the map file if it exists, otherwise return an empty dictionary"""
        self.get_logger().info("Loading saved map...")
        try:
            with open(self.map_file, 'r') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

        """    
        def convert_to_native_float(self, obj):
        #Convert NumPy floats to native Python floats for JSON storage
        if isinstance(obj, np.float32):
            return float(obj)
        elif isinstance(obj, dict):
            return {key: self.convert_to_native_float(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            return [self.convert_to_native_float(item) for item in obj]
        else:
            return obj
        """

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

        self.get_logger().info("Saving map...")
        converted_map = convert_to_native(self.object_map)  # Convert before saving
        with open(self.map_file, 'w') as f:
            json.dump(converted_map, f, indent=4)
        self.get_logger().info(f"Map saved at: {os.path.abspath(self.map_file)}")

    def detected_objects_callback(self, msg: PointCloud2):
        """Store only the detected objects by storing their centroids."""
        self.get_logger().info("Processing detected objects...")

        points = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # We'll store centroids or bounding box centers instead of all points
        for point in points:
            # Round point to a certain precision to avoid duplicates
            x, y, z = map(lambda v: round(v, 2), point[:3])  
            key = f"{x}_{y}_{z}"  # Unique key for the point
            if key not in self.object_map:
                self.object_map[key] = (x, y, z)

        self.save_map()  # Save after processing
        self.get_logger().info(f"Stored {len(points)} detected objects.")

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
