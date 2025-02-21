import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid')
        
        # Occupancy Grid Publisher
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        
        # LiDAR Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar',
            self.lidar_callback,
            10)
        
        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Grid Parameters
        self.grid_size = 150  # 15x15 grid (m)
        self.resolution = 0.05  # 5 cm per cell
        self.origin_x = - (self.grid_size * self.resolution) / 2
        self.origin_y = - (self.grid_size * self.resolution) / 2

        # Initialize map
        self.map_data = self.generate_room_map(size=self.grid_size)

        # Timer to publish occupancy grid
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info("Occupancy Grid Node Started")

    def generate_room_map(self, size=10, obstacle_chance=0.2):
        grid = np.zeros((size, size), dtype=int)
        grid[0, :] = 100
        grid[-1, :] = 100
        grid[:, 0] = 100
        grid[:, -1] = 100
        return grid.flatten().tolist()

    def lidar_callback(self, msg):
        """ Process LiDAR scan and update the occupancy grid """
        try:
            # Transform PointCloud2 to the map frame
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
            cloud_transformed = do_transform_cloud(msg, transform)
            
            # Convert PointCloud2 to numpy array
            points = np.array(list(pc2.read_points(cloud_transformed, field_names=("x", "y"), skip_nans=True)))

            # Update the occupancy grid
            self.update_occupancy_grid(points)

        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")

    """
    def update_occupancy_grid(self, points):
        #Mark obstacles and construct room boundaries using the furthest LiDAR scans
        grid = np.array(self.map_data).reshape((self.grid_size, self.grid_size))

        # Dictionary to store the furthest point for each angle sector
        max_range_points = {}

        for point in points:
            gx = int((point[0] - self.origin_x) / self.resolution)
            gy = int((point[1] - self.origin_y) / self.resolution)

            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                angle = np.arctan2(point[1], point[0])  # Angle of the scan point
                dist = np.linalg.norm(point)  # Distance of point from the robot

                # Store the furthest point for each angle sector
                if angle not in max_range_points or dist > max_range_points[angle][0]:
                    max_range_points[angle] = (dist, gx, gy)

        # Mark the furthest detected points as walls
        for _, (_, gx, gy) in max_range_points.items():
            grid[gy, gx] = 100  # Mark as obstacle

        self.map_data = grid.flatten().tolist()
    """

    def update_occupancy_grid(self, points, inflation_radius=2, obstacle_value=100, buffer_value=20):
        # Mark free space and obstacles in the occupancy grid
        grid = np.array(self.map_data).reshape((self.grid_size, self.grid_size))
        
        # Convert world coordinates to grid coordinates and apply inflation with opacity
        for point in points:
            gx = int((point[0] - self.origin_x) / self.resolution)
            gy = int((point[1] - self.origin_y) / self.resolution)

            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                # Mark the obstacle cell with full opacity
                grid[gy, gx] = obstacle_value

                # Apply inflation by marking surrounding cells as a buffer with lower opacity
                for dx in range(-inflation_radius, inflation_radius + 1):
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        # Check if the cell is within the grid bounds
                        nx = gx + dx
                        ny = gy + dy

                        # Calculate the squared distance to avoid unnecessary sqrt calculation
                        if (dx ** 2 + dy ** 2) <= inflation_radius ** 2:
                            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                                # Avoid overwriting actual obstacle cells with buffer value
                                if grid[ny, nx] != obstacle_value:
                                    grid[ny, nx] = buffer_value  # Mark as buffer/low-opacity

        # Update the map data with the inflated grid
        self.map_data = grid.flatten().tolist()

    
    def publish_map(self):
        """ Publish updated occupancy grid """
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_size
        msg.info.height = self.grid_size
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.data = self.map_data

        self.publisher_.publish(msg)
        self.get_logger().info("Updated Occupancy Grid Published")


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
