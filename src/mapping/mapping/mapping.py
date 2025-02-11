import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)  # Publish at 1 Hz

    def publish_map(self):
        msg = OccupancyGrid()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Grid metadata
        msg.info.resolution = 0.1  # 1 meter per cell
        msg.info.width = 200  # 5x5 m grid
        msg.info.height = 200
        msg.info.origin.position.x = 10.0
        msg.info.origin.position.y = 10.0
        msg.info.origin.position.z = 0.1
        msg.info.origin.orientation.w = 1.0  # No rotation

        # Grid data (-1 = unknown, 0 = free, 100 = occupied)
        msg.data = np.zeros()
        self.publisher.publish(msg)
        self.get_logger().info("Published occupancy grid")

def main():
    rclpy.init()
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
