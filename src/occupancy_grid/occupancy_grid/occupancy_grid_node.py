#!/usr/bin/env python

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import matplotlib.path as mpl_path
from scipy.ndimage import gaussian_filter
from detect_interfaces.srv import DetectObjects
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from ament_index_python import get_package_share_directory
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from skimage.draw import line
from matplotlib.path import Path
from shapely.geometry import Polygon
from shapely.geometry import Point

class OccupancyGridPublisher(Node):
    # Constants for grid and object properties
    OBSTACLE_VALUE = 100
    INFLATION_VALUE = 80
    INFLATION_RADIUS = 8
    SEEN_VALUE = 0
    BORDER_THICKNESS = 0.2
    OBJECT_VALUE = 100
    OBJECT_INFLATION_RADIUS = 7
    OBJECT_INFLATION_VALUE = 80

    def __init__(self):
        super().__init__('occupancy_grid')

        # Publishers
        self.marker_publisher = self.create_publisher(Marker, 'workspace_marker', 10)
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)

        qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,  # Ensures message delivery
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Keeps the last message for new subscribers
                depth=10  # Stores up to 10 messages in queue
                )
        
        self.objects_pub = self.create_publisher(Marker, '/object_markers', qos)
        self.workspace_explored = False  # Initially assume the workspace is not fully explored

        # Add service client to call the detect_objects service
        self.detect_objects_client = self.create_client(DetectObjects, 'detect_objects')
        self.timer = self.create_timer(1.0, self.fetch_detected_objects)
        
        # LiDAR Subscribers
        self.subscription = self.create_subscription(PointCloud2, '/lidar', self.lidar_callback, 10)
        
        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Grid Parameters
        self.resolution = 0.03

        # Initialize map
        package_share_dir = get_package_share_directory('occupancy_grid')
        ws_path = os.path.join(package_share_dir, 'data', 'workspace_3.tsv')
        if os.path.exists(ws_path):
            
            self.workspace_coordinates = self.read_workspace_coordinates(ws_path)
            shapely_polygon = Polygon(self.workspace_coordinates)
            shapely_polygon = Polygon(self.workspace_coordinates)
            self.workspace_polygon = Path(self.workspace_coordinates)
            self.shrunk_polygon = shapely_polygon.buffer(-self.BORDER_THICKNESS)
            self.obj_polygon = shapely_polygon.buffer(-0.04)

            #self.workspace_polygon = Path(self.workspace_coordinates)
            #self.small_workspace_polygon = Path(list(self.workspace_polygon.buffer(-self.BORDER_THICKNESS).exterior.coords))
        else:
            self.get_logger().error(f"Workspace file {ws_path} not found.")

        # Adjust grid size based on workspace dimensions
        self.grid_size_x, self.grid_size_y, self.origin_x, self.origin_y = self.calculate_grid_size_and_origin(self.workspace_coordinates)

        # Initialize map data
        self.map_data = self.generate_room_map()
        
        # Timer to publish occupancy grid
        #self.marker_timer = self.create_timer(1.0, self.publish_workspace_marker)
        #self.publish_workspace_marker()

        self.timer = self.create_timer(1.0, self.publish_map)
        #self.grid_update_timer = self.create_timer(0.5, self.update_grid_regularly)
        self.get_logger().info("Occupancy Grid Node Started")

    """    def update_grid_regularly(self):
            #Update the occupancy grid based on the robot's rotation for the "seen" mechanic.
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
                self.get_logger().warn(f"Failed to update grid regularly: {e}")"""

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

                for i, (obj_type, obj_pos) in enumerate(self.detected_objects):
                    if obj_type == 'trash':
                        self.detected_objects.pop(i)
                        #self.get_logger().info(f"Removed: {i}, type: {obj_type}")
                    if not self.obj_polygon.contains(Point(obj_pos.x, obj_pos.y)):
                        self.detected_objects.pop(i)
                        self.get_logger().info(f"Removed a object outside obj_polygon")

                self.publish_objects(self.detected_objects)

                # Save to map file
                with open("Generated_map.tsv", "w") as file:  # filter on trash?
                    for obj_type, pos in self.detected_objects:
                        x, y, z = float(pos.x), float(pos.y), float(pos.z)
                        file.write(f"{obj_type} \t {x:.2f} \t {y:.2f} \t {z:.2f}" + "\n")

                # Convert flat map to 2D grid for easier manipulation
                grid = np.array(self.map_data).reshape((self.grid_size_y, self.grid_size_x))

                # Parameters
                inflation_radius = object_inflation_radius  # Solid border thickness in cells
                gradient_radius = inflation_radius + 6       # Gradient fade-out radius

                for obj_type, obj_position in self.detected_objects:
                    object_x = obj_position.x
                    object_y = obj_position.y

                    gx = int((object_x - self.origin_x) / self.resolution)
                    gy = int((object_y - self.origin_y) / self.resolution)

                    if 0 <= gx < self.grid_size_x and 0 <= gy < self.grid_size_y:
                        # Mark the object cell as occupied
                        grid[gy, gx] = obstacle_value

                        # Inflate border with gradient
                        for dx in range(-gradient_radius, gradient_radius + 1):
                            for dy in range(-gradient_radius, gradient_radius + 1):
                                nx = gx + dx
                                ny = gy + dy

                                if 0 <= nx < self.grid_size_x and 0 <= ny < self.grid_size_y:
                                    distance_sq = dx**2 + dy**2
                                    distance = np.sqrt(distance_sq)

                                    if distance <= gradient_radius:
                                        if distance <= inflation_radius:
                                            # Solid inflation
                                            if grid[ny, nx] != obstacle_value:
                                                grid[ny, nx] = border_value
                                        else:
                                            # Gradient inflation
                                            decay = 1 - ((distance - inflation_radius) / (gradient_radius - inflation_radius))
                                            value = int(border_value * decay)
                                            if grid[ny, nx] < value:
                                                grid[ny, nx] = value  # Only update if value is stronger

                        # self.get_logger().info(f"Object {obj_type} placed at grid cell ({gx}, {gy})")

                # Update the flattened map data
                self.map_data = grid.flatten().tolist()

                # self.get_detected_objects_info()

            # else:
            #     self.get_logger().warn("No objects detected.")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

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

    def lidar_callback(self, msg: PointCloud2):
        """ Process LiDAR scan and update the occupancy grid """

        # Transformation
        to_frame_rel = 'map'
        from_frame_rel = msg.header.frame_id

        time = rclpy.time.Time().from_msg(msg.header.stamp)

        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            transform = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                time)
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # Transform PointCloud2 to the map frame
        cloud_transformed = do_transform_cloud(msg, transform)

        # Get the robot's position in the map frame
        # Transformation
        to_frame_rel = 'map'
        from_frame_rel = 'base_link'

        time = rclpy.time.Time().from_msg(msg.header.stamp)

        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            robot_transform = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                time)
        except Exception as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        robot_x = robot_transform.transform.translation.x
        robot_y = robot_transform.transform.translation.y
            
        # Convert PointCloud2 to numpy array
        points = np.array(list(pc2.read_points(cloud_transformed, field_names=("x", "y"), skip_nans=True)))

        # Distance thresholding relative to the robot's position
        min_distance = 0.4 # Minimum distance, 0.35 enough to remove the arm from the lidar
        max_distance = 4  # Maximum distance (e.g., 10 meters)

        # Filter points based on distance threshold relative to the robot
        filtered_points = []
        for point in points:
            # Calculate distance from the robot (in the map frame)
            distance = np.sqrt((point[0] - robot_x)**2 + (point[1] - robot_y)**2)

            # Apply the distance thresholding (ignore points too close or too far from the robot)
            if min_distance <= distance <= max_distance:
                filtered_points.append(point)

        # Update the occupancy grid with filtered points
        self.update_occupancy_grid(filtered_points, robot_x=robot_x, robot_y=robot_y)

    def update_occupancy_grid(self, points, robot_x=None, robot_y=None,
                            inflation_radius=INFLATION_RADIUS,
                            obstacle_value=OBSTACLE_VALUE,
                            inflation_value=INFLATION_VALUE):

        if robot_x is None or robot_y is None:
            self.get_logger().warn("Robot position not provided for occupancy update.")
            return

        # Reshape the map for 2D grid processing
        grid = np.array(self.map_data).reshape((self.grid_size_y, self.grid_size_x))

        # Convert robot world position to grid coordinates
        robot_gx = int((robot_x - self.origin_x) / self.resolution)
        robot_gy = int((robot_y - self.origin_y) / self.resolution)

        for point in points:
            # Convert obstacle world position to grid coordinates
            gx = int((point[0] - self.origin_x) / self.resolution)
            gy = int((point[1] - self.origin_y) / self.resolution)

            if 0 <= gx < self.grid_size_x and 0 <= gy < self.grid_size_y:
                # Clear cells between robot and obstacle using raytracing
                rr, cc = line(robot_gy, robot_gx, gy, gx)  # (row, col) => (y, x)

                for y, x in list(zip(rr, cc))[:-1]:
                    if 0 <= x < self.grid_size_x and 0 <= y < self.grid_size_y:
                        world_x = self.origin_x + x * self.resolution
                        world_y = self.origin_y + y * self.resolution

                        if self.shrunk_polygon.contains(Point(world_x, world_y)):
                        #if self.shrunk_polygon.contains_point((world_x, world_y)) : 
                            grid[y, x] = -1

                # Mark obstacle cell
                grid[gy, gx] = obstacle_value

                # Inflate obstacle
                for dx in range(-inflation_radius, inflation_radius + 1):
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        nx = gx + dx
                        ny = gy + dy
                        if 0 <= nx < self.grid_size_x and 0 <= ny < self.grid_size_y:
                            if (dx ** 2 + dy ** 2) <= inflation_radius ** 2:
                                if grid[ny, nx] != obstacle_value:
                                    grid[ny, nx] = inflation_value

        # Check if any unexplored cells remain
        unexplored_found = np.any(grid == -1)
        self.workspace_explored = not unexplored_found

        # Flatten and update map data
        self.map_data = grid.flatten().tolist()

        if self.workspace_explored:
            self.get_logger().info("The entire workspace has been explored!")

    def mark_area_in_front_of_robot(self, robot_x, robot_y, robot_yaw, length=1.5, width=0.6, seen_value=SEEN_VALUE, cone_angle=np.radians(30)):
        """
        Mark a cone-shaped area in front of the robot as seen.
        - `length`: how far in front of the robot to mark [m]
        - `cone_angle`: the full angle of the cone [radians]
        """
        grid = np.array(self.map_data).reshape((self.grid_size_y, self.grid_size_x))
        
        # Angular resolution and radial resolution
        angle_steps = int(cone_angle / np.radians(2))  # e.g., 2 degree steps
        distance_steps = int(length / self.resolution)

        angles = np.linspace(-cone_angle / 2, cone_angle / 2, angle_steps)
        distances = np.linspace(0.2, length, distance_steps)  # Start slightly ahead of the robot

        for angle in angles:
            abs_angle = robot_yaw + angle
            cos_a = np.cos(abs_angle)
            sin_a = np.sin(abs_angle)
            for dist in distances:
                x = robot_x + dist * cos_a
                y = robot_y + dist * sin_a

                gx = int((x - self.origin_x) / self.resolution)
                gy = int((y - self.origin_y) / self.resolution)

                if 0 <= gx < self.grid_size_x and 0 <= gy < self.grid_size_y:
                    if grid[gy, gx] < 10:  # Don't overwrite obstacles
                        grid[gy, gx] = seen_value

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

        package_share_dir = get_package_share_directory('occupancy_grid')
        ws_path = os.path.join(package_share_dir, 'data', 'workspace_3.tsv')
        if os.path.exists(ws_path):
            self.workspace_coordinates = self.read_workspace_coordinates(ws_path)
        else:
            self.get_logger().error(f"Workspace file {ws_path} not found.")

        coordinates = self.read_workspace_coordinates(ws_path)

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
        # self.get_logger().info("Published workspace marker from file")


    def publish_objects(self, objects):
        for idx, (obj_type, pos) in enumerate(objects): 
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'

            marker.id = idx + 1
            marker.ns = f'Object Markers'
            marker.lifetime = Duration(sec=10000)  # Keep marker visible for 10 seconds
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = pos.x
            marker.pose.position.y = pos.y
            marker.pose.position.z = 0.0

            # Orientation
            q = quaternion_from_euler(0, 0, pos.z)

            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]

            # Marker Colour
            marker.color.a = 1.0  # Alpha
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0

            # Scale marker size
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            if obj_type == '1': # Cube
                marker.type = marker.CUBE
            elif obj_type == '2': # Sphere
                marker.type = marker.SPHERE
            elif obj_type == '3': # Plushie
                marker.type = marker.CYLINDER
            elif obj_type == 'B': # Box
                marker.type = marker.CUBE
                marker.scale.x = 0.3
                marker.scale.y = 0.15
                marker.scale.z = 0.1
            else:
                return
            self.objects_pub.publish(marker)
            #claerprint(f'Publishing marker ID: {marker.id} at x={marker.pose.position.x}, y={marker.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()