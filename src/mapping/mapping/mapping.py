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
            '/camera/camera/depth/color/points',#swith to /camera/depth/color/ds_points or /detected/objects
            self.detected_objects_callback,
            10
        )        # Map storage
        self.map_file = 'map_file.json'
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
    
    def convert_to_native_float(self, obj):
        if isinstance(obj, np.float32):
            return float(obj)
        elif isinstance(obj, dict):
            return {key: self.convert_to_native_float(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            return [self.convert_to_native_float(item) for item in obj]
        else:
            return obj
    
    def save_map(self):
        """Save the object map to a JSON file"""
        converted_map = self.convert_to_native_float(self.object_map)
        self.get_logger().info("saving")
        with open(self.map_file, 'w') as f:
            json.dump(converted_map, f, indent=4)        
            abs_path = os.path.abspath(self.map_file)
        self.get_logger().info(f"Map saved at: {abs_path}")    
    
    def detected_objects_callback(self, msg: PointCloud2):
        """Callback function to process detected objects from PointCloud2"""
        self.get_logger().info("callback")      
        #print(msg.fields)
        points = pc2.read_points_numpy(msg, field_names=("x","y","z","rgb"), skip_nans=True)

        detected_objects = []
        for point in points:
            x, y, z, rgb = point  # Extract x, y, z coordinates
            #self.get_logger().info(f"Point: x={x}, y={y}, z={z}")
            detected_objects.append([x,y,z])
        #points = gen[:, :3]

        
        
    
        # Iterate over the points
        for point in points:
            x, y, z = point[:3]  # Extract x, y, z
            
            self.get_logger().info("adding obj")
            # Update object map
            if len(detected_objects) != 0:
                for obj in detected_objects:
                    key = f"{obj[0]:.2f}_{obj[1]:.2f}_{obj[2]:.2f}"  # Unique key for each object
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
