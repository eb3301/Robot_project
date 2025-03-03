import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sklearn.neighbors import NearestNeighbors
from visualization_msgs.msg import Marker

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid')
        
        # Occupancy Grid Publisher
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.marker_publisher = self.create_publisher(Marker, 'workspace_marker', 10)
        
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
        self.resolution = 0.05  # 5 cm per cell

        # Initialize map
        # Adjust grid size based on workspace dimensions
        self.workspace_coordinates = self.read_workspace_coordinates("/home/kristoffer-germalm/dd2419_ws/src/occupancy_grid/occupancy_grid/workspace_2.tsv")
        self.grid_size_x, self.grid_size_y, self.origin_x, self.origin_y = self.calculate_grid_size_and_origin(self.workspace_coordinates)

        # Initialize map data
        self.map_data = self.generate_room_map()

        # Timer to publish occupancy grid
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info("Occupancy Grid Node Started")

        self.publish_workspace_marker()  # Add this line to call the function

    def calculate_grid_size_and_origin(self, coordinates):
        """Calculate grid size and origin based on workspace coordinates."""
        # Find the bounding box of the workspace (min and max x and y values)
        min_x = min(coord[0] for coord in coordinates)
        max_x = max(coord[0] for coord in coordinates)
        min_y = min(coord[1] for coord in coordinates)
        max_y = max(coord[1] for coord in coordinates)

        # Calculate the width and height of the workspace
        workspace_width = max_x - min_x
        workspace_height = max_y - min_y

        # Calculate the grid size (number of cells) based on the workspace size and resolution
        grid_size_x = int(np.ceil(workspace_width / self.resolution))
        grid_size_y = int(np.ceil(workspace_height / self.resolution))

        # Set the origin to the bottom-left corner of the grid
        origin_x = min_x
        origin_y = min_y

        return grid_size_x, grid_size_y, origin_x, origin_y

    def generate_room_map(self):
        """Generate room map based on workspace size."""
        grid = np.zeros((self.grid_size_y, self.grid_size_x), dtype=int)  # Use grid_size_x and grid_size_y
        
        # Mark the boundary of the workspace as obstacles
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

            # Get the robot's position in the map frame
            robot_transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = robot_transform.transform.translation.x
            robot_y = robot_transform.transform.translation.y
            
            # Convert PointCloud2 to numpy array
            points = np.array(list(pc2.read_points(cloud_transformed, field_names=("x", "y"), skip_nans=True)))

            # Distance thresholding relative to the robot's position
            min_distance = 0.4 # Minimum distance, 0.35 enough to remove the arm from the lidar
            max_distance = 2.0  # Maximum distance (e.g., 10 meters)

            # Filter points based on distance threshold relative to the robot
            filtered_points = []
            for point in points:
                # Calculate distance from the robot (in the map frame)
                distance = np.sqrt((point[0] - robot_x)**2 + (point[1] - robot_y)**2)

                # Apply the distance thresholding (ignore points too close or too far from the robot)
                if min_distance <= distance <= max_distance:
                    filtered_points.append(point)

            # Update the occupancy grid with filtered points
            self.update_occupancy_grid(filtered_points)

        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")

    def update_occupancy_grid(self, points, inflation_radius=2, obstacle_value=100, buffer_value=20):
        # Mark free space and obstacles in the occupancy grid
        grid = np.array(self.map_data).reshape((self.grid_size_y, self.grid_size_x))  # Use grid_size_y and grid_size_x
        
        # Convert world coordinates to grid coordinates and apply inflation with opacity
        for point in points:
            gx = int((point[0] - self.origin_x) / self.resolution)
            gy = int((point[1] - self.origin_y) / self.resolution)

            if 0 <= gx < self.grid_size_x and 0 <= gy < self.grid_size_y:  # Check against grid_size_x and grid_size_y
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
                            if 0 <= nx < self.grid_size_x and 0 <= ny < self.grid_size_y:  # Check against grid_size_x and grid_size_y
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
        msg.info.width = self.grid_size_x  # Use grid_size_x
        msg.info.height = self.grid_size_y  # Use grid_size_y
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.data = self.map_data

        self.publisher_.publish(msg)
        self.get_logger().info("Updated Occupancy Grid Published")

    def read_workspace_coordinates(self, file_path):
        """Reads workspace coordinates from a file."""
        coordinates = []
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    values = line.strip().split()
                    if len(values) == 2:  # Ensure two values (x, y)
                        x, y = map(float, values)
                        coordinates.append((x / 100.0, y / 100.0))
        except Exception as e:
            self.get_logger().error(f"Error reading workspace file: {e}")
        return coordinates

    def publish_workspace_marker(self):
        """Reads workspace coordinates from a file and publishes a marker."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        file_path = "/home/kristoffer-germalm/dd2419_ws/src/occupancy_grid/occupancy_grid/workspace_2.tsv"  # Update this path if necessary
        coordinates = self.read_workspace_coordinates(file_path)

        first_point = None

        for coord in coordinates:
            # Convert from mm to meters (divide by 1000)
            point = Point()
            point.x, point.y = coord[0], coord[1]  # Convert to meters (?)
            point.z = 0.0  # Assuming it's a 2D workspace, set z to 0
            marker.points.append(point)
            
            # Save the first point
            if first_point is None:
                first_point = point

        # Append the first point again to close the loop
        if first_point is not None:
            marker.points.append(first_point)

        self.marker_publisher.publish(marker)
        self.get_logger().info("Published workspace marker from file")


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()