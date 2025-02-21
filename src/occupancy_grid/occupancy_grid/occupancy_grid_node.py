#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.timer = self.create_timer(5.0, self.publish_map)
        self.get_logger().info("Occupancy Grid Node Started")
        self.size = 100
        self.workspace = 2

    def generate_room_map(self, obstacle_chance=0.2):
        """
        Generates a simple room with walls and random obstacles.
        - Walls: 100
        - Free space: 0
        - Obstacles: 100 (randomly placed)
        """
        grid = np.zeros((self.size, self.size), dtype=int)

        # Add walls (borders)
        grid[0, :] = 100
        grid[-1, :] = 100
        grid[:, 0] = 100
        grid[:, -1] = 100

        # # Add random obstacles inside the room
        # num_obstacles = int((self.size * self.size) * obstacle_chance)
        # obstacle_positions = np.random.choice(size * self.size, num_obstacles, replace=False)

        # for pos in obstacle_positions:
        #     x, y = divmod(pos, self.size)
        #     if grid[x, y] == 0:  # Only place obstacles in free space
        #         grid[x, y] = 100

        return grid.flatten().tolist()

    def publish_map(self):
        map_data = self.generate_room_map()

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.workspace / self.size  # cm per cell
        msg.info.width = self.size
        msg.info.height = self.size
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.data = map_data

        self.publisher_.publish(msg)
        self.get_logger().info("Published occupancy grid with a simple room")

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
