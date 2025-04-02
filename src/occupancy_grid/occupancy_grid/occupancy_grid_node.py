import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import matplotlib.path as mpl_path
from scipy.ndimage import gaussian_filter
from detect_interfaces.srv import DetectObjects
from tf_transformations import euler_from_quaternion

class OccupancyGridPublisher(Node):
    # Constants for grid and object properties
    OBSTACLE_VALUE = 100
    INFLATION_VALUE = 40
    INFLATION_RADIUS = 2
    SEEN_VALUE = 0
    BORDER_THICKNESS = 0.2
    OBJECT_VALUE = 80
    OBJECT_INFLATION_RADIUS = 2
    OBJECT_INFLATION_VALUE = 40

    def __init__(self):
        super().__init__('occupancy_grid')

        # Publishers
        self.marker_publisher = self.create_publisher(Marker, 'workspace_marker', 10)
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        
        self.workspace_explored = False  # Initially assume the workspace is not fully explored


        # Add service client to call the detect_objects service
        self.detect_objects_client = self.create_client(DetectObjects, 'detect_objects')
        # Wait for the service to be available
        if not self.detect_objects_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DetectObjects service not available...')
        # Add a timer to periodically get the detected objects
        self.timer = self.create_timer(1.0, self.fetch_detected_objects)
        

        
        # LiDAR Subscribers
        self.subscription = self.create_subscription(PointCloud2, '/lidar', self.lidar_callback, 10)
        #self.cloud_sub = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)
        
        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Grid Parameters
        self.resolution = 0.03

        # Initialize map
        # Adjust grid size based on workspace dimensions
        self.workspace_coordinates = self.read_workspace_coordinates("/home/group1/dd2419_ws/src/occupancy_grid/occupancy_grid/workspace_2.tsv")
        self.grid_size_x, self.grid_size_y, self.origin_x, self.origin_y = self.calculate_grid_size_and_origin(self.workspace_coordinates)

        # Initialize map data
        self.map_data = self.generate_room_map()
        
        # Timer to publish occupancy grid
        #self.marker_timer = self.create_timer(1.0, self.publish_workspace_marker)
        #self.publish_workspace_marker()

        self.timer = self.create_timer(1.0, self.publish_map)
        self.grid_update_timer = self.create_timer(0.5, self.update_grid_regularly)
        self.get_logger().info("Occupancy Grid Node Started")

    def update_grid_regularly(self):
        """Update the occupancy grid based on the robot's rotation for the "seen" mechanic. """
        try:
            # Get the robot's position and orientation from TF
            robot_transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_x = robot_transform.transform.translation.x
            robot_y = robot_transform.transform.translation.y
            robot_quaternion = robot_transform.transform.rotation

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            _, _, robot_yaw = euler_from_quaternion([robot_quaternion.x, robot_quaternion.y, robot_quaternion.z, robot_quaternion.w])

            self.mark_area_in_front_of_robot(robot_x, robot_y, robot_yaw)

        except Exception as e:
            self.get_logger().warn(f"Failed to update grid regularly: {e}")

    def fetch_detected_objects(self):
        """ Call the 'detect_objects' service to get the detected objects. """
        req = DetectObjects.Request()
        future = self.detect_objects_client.call_async(req)
        future.add_done_callback(self.on_detect_objects_response)

    def on_detect_objects_response(self, future, obstacle_value=OBJECT_VALUE, border_value=OBJECT_INFLATION_VALUE, object_inflation_radius=OBJECT_INFLATION_RADIUS):
        """ Callback to handle the response from the detect_objects service. """
        try:
            response = future.result()

            # Process the detected objects from the response
            if response.object_types:
                self.detected_objects = list(zip(response.object_types, response.object_positions))
                # self.get_logger().info(f"Detected {len(self.detected_objects)} objects.")

                # Convert flat map to 2D grid for easier manipulation
                grid = np.array(self.map_data).reshape((self.grid_size_y, self.grid_size_x))

                # Parameters
                obstacle_value = obstacle_value
                border_value = border_value
                inflation_radius = object_inflation_radius  # Border thickness in cells

                for obj_type, obj_position in self.detected_objects:
                    object_x = obj_position.x
                    object_y = obj_position.y

                    gx = int((object_x - self.origin_x) / self.resolution)
                    gy = int((object_y - self.origin_y) / self.resolution)

                    if 0 <= gx < self.grid_size_x and 0 <= gy < self.grid_size_y:
                        # Mark the object cell as occupied
                        grid[gy, gx] = obstacle_value

                        # Inflate border around the object
                        for dx in range(-inflation_radius, inflation_radius + 1):
                            for dy in range(-inflation_radius, inflation_radius + 1):
                                nx = gx + dx
                                ny = gy + dy

                                if 0 <= nx < self.grid_size_x and 0 <= ny < self.grid_size_y:
                                    distance_sq = dx**2 + dy**2
                                    if distance_sq <= inflation_radius**2:
                                        if grid[ny, nx] != obstacle_value:
                                            grid[ny, nx] = border_value

                        # self.get_logger().info(f"Object {obj_type} placed at grid cell ({gx}, {gy})")

                # Update the flattened map data
                self.map_data = grid.flatten().tolist()

                self.get_detected_objects_info()

            else:
                pass
                #self.get_logger().warn("No objects detected.")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def get_detected_objects_info(self):
        """ Print out the information of detected objects. """
        for i, (obj_type, obj_position) in enumerate(self.detected_objects):
            pass
            # self.get_logger().info(f"Object {i + 1}: Type: {obj_type}, Position: {obj_position}")

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

    def generate_room_map(self, obstacle_value=OBSTACLE_VALUE, wall_inflation_value= INFLATION_VALUE,border_thickness=BORDER_THICKNESS):
        """Generate room map with unexplored cells (-1)."""
        grid = np.full((self.grid_size_y, self.grid_size_x), -1, dtype=int)  # Unexplored by default

        # Create a path from the workspace coordinates (polygon of the workspace perimeter)
        workspace_polygon = mpl_path.Path(self.workspace_coordinates)

        # BORDER PARAMETERS (WALL INFLATION)
        # -----------------------------------
        border_thickness = border_thickness
        # -----------------------------------

        for gy in range(self.grid_size_y):
            for gx in range(self.grid_size_x):
                # Convert grid cell center (gx, gy) to world coordinates
                world_x = self.origin_x + gx * self.resolution
                world_y = self.origin_y + gy * self.resolution

                if not workspace_polygon.contains_point((world_x, world_y)):
                    grid[gy, gx] = obstacle_value  # Outside workspace = wall
                else:
                    # Check if this cell is close to the edge (inside the polygon, but near a wall)
                    # Sample some surrounding points slightly further out to test if they are outside
                    is_border = False
                    for dx in [-border_thickness, 0, border_thickness]:
                        for dy in [-border_thickness, 0, border_thickness]:
                            if dx == 0 and dy == 0:
                                continue
                            test_x = world_x + dx
                            test_y = world_y + dy
                            if not workspace_polygon.contains_point((test_x, test_y)):
                                is_border = True
                                break
                        if is_border:
                            break

                    if is_border:
                        grid[gy, gx] = wall_inflation_value # Border wall
                    else:
                        grid[gy, gx] = -1  # Free (unexplored) space

        # self.get_logger().info(f"Value at grid[10,10] before = {grid[10,10]}")
        #grid[20,30] = 100

        return grid.flatten().tolist()
    
        """    def cloud_callback(self, msg: PointCloud2):
                #Process depth camera PointCloud2 message and update the occupancy grid.
                points = pc2.read_points_numpy(msg, skip_nans=True)[:, :3]  # Extract points (x, y, z)

                # Calculate the min and max x, y values from the point cloud
                min_x = np.min(points[:, 0])
                max_x = np.max(points[:, 0])
                min_y = np.min(points[:, 1])
                max_y = np.max(points[:, 1])

                # Convert the min and max x, y into grid coordinates (cell indices)
                min_gx = int((min_x - self.origin_x) / self.resolution)
                max_gx = int((max_x - self.origin_x) / self.resolution)
                min_gy = int((min_y - self.origin_y) / self.resolution)
                max_gy = int((max_y - self.origin_y) / self.resolution)

                # Make sure the grid indices are within bounds
                min_gx = max(min_gx, 0)
                max_gx = min(max_gx, self.grid_size_x - 1)
                min_gy = max(min_gy, 0)
                max_gy = min(max_gy, self.grid_size_y - 1)

                # Update the grid within the bounding box to mark it as "seen"
                self.update_grid_from_bounding_box(min_gx, max_gx, min_gy, max_gy)"""

        """    def update_grid_from_bounding_box(self, min_gx, max_gx, min_gy, max_gy, seen_value=50):
                
                #Update the occupancy grid for the region defined by the bounding box (min_x, max_x, min_y, max_y).
                
                grid = np.array(self.map_data).reshape((self.grid_size_y, self.grid_size_x))  # Use grid_size_y and grid_size_x

                # Mark all cells within the bounding box as "seen" (free space)
                for gy in range(min_gy, max_gy + 1):
                    for gx in range(min_gx, max_gx + 1):
                        grid[gy, gx] = seen_value

                # Update the map data with the modified grid
                self.map_data = grid.flatten().tolist()
                self.get_logger().info(f"Updated grid from bounding box: min({min_gx}, {min_gy}), max({max_gx}, {max_gy})")"""

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
            
            # # Distance thresholding relative to the robot's position
            # min_distance = 0.4 # Minimum distance, 0.35 enough to remove the arm from the lidar
            # max_distance = 2.5  # Maximum distance (e.g., 10 meters)

            # # Filter points based on distance threshold relative to the robot
            # filtered_points = []
            # for point in points:
            #     # Calculate distance from the robot (in the map frame)
            #     distance = np.sqrt((point[0] - robot_x)**2 + (point[1] - robot_y)**2)

            #     # Apply the distance thresholding (ignore points too close or too far from the robot)
            #     if min_distance <= distance <= max_distance:
            #         filtered_points.append(point)

            # Update the occupancy grid with filtered points
            self.update_occupancy_grid(points)

        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")

    def update_occupancy_grid(self, points, inflation_radius=INFLATION_RADIUS, obstacle_value=OBSTACLE_VALUE, inflation_value=INFLATION_VALUE):
        # Mark free space and obstacles in the occupancy grid
        grid = np.array(self.map_data).reshape((self.grid_size_y, self.grid_size_x))  # Use grid_size_y and grid_size_x
        
        # Flag to track if any unexplored cells are still present
        unexplored_found = False

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
                                    grid[ny, nx] = inflation_value  # Mark as buffer/low-opacity

        # After processing all points, check if there are still any unexplored cells (-1)
        unexplored_found = np.any(grid == -1)  # Check if there are any unexplored cells left

        # Update the workspace_explored flag
        self.workspace_explored = not unexplored_found

        # Update the map data with the inflated grid
        self.map_data = grid.flatten().tolist()

        # Optionally, log if the workspace is fully explored
        if self.workspace_explored:
            self.get_logger().info("The entire workspace has been explored!")

    def mark_area_in_front_of_robot(self, robot_x, robot_y, robot_yaw, length=1, width=0.6, seen_value=SEEN_VALUE):
        """
        Mark a rectangular area in front of the robot as seen.
        - `length`: how far in front of the robot to mark [m]
        - `width`: how wide the area is [m]
        """
        grid = np.array(self.map_data).reshape((self.grid_size_y, self.grid_size_x))

        # Generate a grid of points inside the rectangle
        dys = np.linspace(-width / 2, width / 2, int(width / self.resolution))
        start_offset = 0.2  # Start 20 cm in front of the robot
        dxs = np.linspace(start_offset, length + start_offset, int(length / self.resolution))

        # Rotation matrix from robot orientation (yaw)
        cos_yaw = np.cos(robot_yaw)
        sin_yaw = np.sin(robot_yaw)

        for dy in dys:
            for dx in dxs:
                # Calculate the world coordinates of the point in front of the robot
                x = robot_x + dx * cos_yaw - dy * sin_yaw  # Corrected to rotate around the yaw axis
                y = robot_y + dx * sin_yaw + dy * cos_yaw  # Corrected to rotate around the yaw axis

                gx = int((x - self.origin_x) / self.resolution)
                gy = int((y - self.origin_y) / self.resolution)

                if 0 <= gx < self.grid_size_x and 0 <= gy < self.grid_size_y:
                    # Only overwrite if not an obstacle
                    if grid[gy, gx] < 10:  # Check if not an obstacle
                        grid[gy, gx] = seen_value

        # Update the map data with the modified grid
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
        #self.get_logger().info("Updated Occupancy Grid Published")

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

        file_path = "/home/group1/dd2419_ws/src/occupancy_grid/occupancy_grid/workspace_2.tsv"  # Update this path if necessary
        coordinates = self.read_workspace_coordinates(file_path)

        first_point = None

        for coord in coordinates:
            # Convert from mm to meters (divide by 1000)
            point = Point()
            point.x, point.y = coord[0], coord[1]  # Convert to meters (?)
            point.z = 1.0  # Assuming it's a 2D workspace, set z to 0
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
