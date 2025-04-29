#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import os
import math
import py_trees as pt
import py_trees_ros as ptr
from builtin_interfaces.msg import Duration

from ament_index_python import get_package_share_directory
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, Twist
from sensor_msgs.msg import PointCloud2

from tf_transformations import quaternion_from_euler
import numpy as np
from arm_interface.srv import Arm
from arm_service.arm_client import call_arm_client
import matplotlib.pyplot as plt

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from tf2_ros import TransformException

from std_srvs.srv import SetBool
from detect_interfaces.srv import DetectObjects
import time

import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN


class BehaviourTree(Node):
    def __init__(self):
        super().__init__('Behaviour_Tree_Node')
        self.get_logger().info("Initialising Behaviour Tree")
        
        # Initialise blackboard variables
        self.blackboard = pt.blackboard.Blackboard()

        # Initialise Behaviours
        create_ws = Create_ws(self) # Read workspace and map file
        load_map = Load_Map(self)



        # Get target coordinates
        ''' Possible arguments:
            'object' -- Any object
            '1' -- Cube
            '2' -- Sphere
            '3' -- Plushie
            'B' -- Box        
            '''
        goto_target = Goto_Target(self, 'object')
        approach_object = Approach_Object(self)
        goto_box = Goto_Target(self, 'B')

        pickup = Pickup(self) # Pickup object
        check_map = Check_Map(self)
        
        # create_ws, load_map, goto_target, create_ws, load_map, goto_target, 


        test_seq = pt.composites.Sequence(name = 'Test Sequence', 
                                          memory = False,
                                          children = [goto_target, approach_object, pickup, goto_box]
                                          )

        test_sel = pt.composites.Selector(name = 'Test Selector',
                                          memory = False,
                                          children = [check_map, test_seq]
                                          )

        base_seq = pt.composites.Sequence(name = 'Main Sequence',
                                          memory = False,
                                          children = [create_ws, load_map, test_sel]
                                          )


        self.BT = pt.trees.BehaviourTree(root = base_seq)

        self.timer = self.create_timer(0.5, self.tick_tree)


    def tick_tree(self):
        '''Ticks the tree and stops if sequence is complete'''
        status = self.BT.tick()


class Create_ws(pt.behaviour.Behaviour):
    '''Processes map file and workspace file'''
    def __init__(self, node):
        super().__init__('Create Workspace')
        self.node = node
        self.blackboard = pt.blackboard.Blackboard()
        self.coords = None

        self.done = False
        qos = QoSProfile(
                        reliability=QoSReliabilityPolicy.RELIABLE,  # Ensures message delivery
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Keeps the last message for new subscribers
                        depth=10  # Stores up to 10 messages in queue
                        )

        self.ws_pub = self.node.create_publisher(Marker, '/Workspace', qos)
        print('Initialised Read Files Behaviour')
        
    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        self.done = True

        package_share_dir = get_package_share_directory('global_planner')
        ws_path = os.path.join(package_share_dir, 'data', 'workspace_3.tsv')
        
        if not os.path.exists(ws_path):
                self.node.get_logger().error(f"Workspace file {ws_path} not found.")
                return pt.common.Status.FAILURE
        
        self.coords = []
        with open(ws_path, 'r') as file:
            for line in file:
                values = line.strip().split()
                if len(values) == 2:  # Ensure two values (x, y)
                    x, y = map(float, values)
                    self.coords.append((x / 100.0, y / 100.0))
        
        self.blackboard.set('workspace_coordinates', self.coords)
        print(self.coords)
        self.publish_workspace()
        return pt.common.Status.SUCCESS
    
    def publish_workspace(self):    
        """Reads workspace coordinates from a file and publishes a marker."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        coordinates = self.coords

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

        self.ws_pub.publish(marker)
        print("Published workspace marker from file")


class Load_Map(pt.behaviour.Behaviour):
    '''Load in the map file'''
    def __init__(self, node):
        super().__init__('Load Map ')
        self.node = node
        self.blackboard = pt.blackboard.Blackboard()
        self.done = False
        qos = QoSProfile(
                        reliability=QoSReliabilityPolicy.RELIABLE,  # Ensures message delivery
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Keeps the last message for new subscribers
                        depth=20  # Stores up to 10 messages in queue
                        )

        self.marker_pub = self.node.create_publisher(Marker, '/object_markers', qos)
        print('Initialised Load Map Behaviour')

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        self.done = True

        package_share_dir = get_package_share_directory('global_planner')
        map_path = os.path.join(package_share_dir, 'data', 'map_3.tsv')

        if not os.path.exists(map_path):
            print(f"Map file {map_path} not found.")
            return pt.common.Status.FAILURE

        objects = []
        with open(map_path, 'r') as file:
            for item in file:
                obj = item.strip().split()
                obj_tuple = (str(obj[0]), float(obj[1])/100, float(obj[2])/100, float(obj[3]))
                objects.append(obj_tuple)
        self.blackboard.set('objects', objects)
        self.publish_objects(objects)

        return pt.common.Status.SUCCESS

    def publish_objects(self, objects):
        for idx, object in enumerate(objects): 
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.header.frame_id = 'map'

            marker.id = idx + 1
            marker.ns = f'Object Markers'
            marker.lifetime = Duration(sec=10000)  # Keep marker visible for 10 seconds
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = object[1]
            marker.pose.position.y = object[2]
            marker.pose.position.z = 0.0

            # Orientation
            q = quaternion_from_euler(0, 0, object[3])

            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]

            # Scale marker size
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            if object[0] == '1': # Cube
                # Marker Colour
                marker.color.a = 1.0  # Alpha
                marker.color.r = 1.0  # Red
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker.type = marker.CUBE
                marker.scale.x = 0.1
                marker.scale.y = 0.11
                marker.scale.z = 0.1

            elif object[0] == '2': # Sphere
                # Marker Colour
                marker.color.a = 1.0  # Alpha
                marker.color.r = 0.2  # Puprle
                marker.color.g = 0.2
                marker.color.b = 0.8

                marker.type = marker.SPHERE
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

            elif object[0] == '3': # Plushie
                # Marker Colour
                marker.color.a = 1.0  # Alpha
                marker.color.r = 0.0  # Green
                marker.color.g = 1.0
                marker.color.b = 0.0

                marker.type = marker.CYLINDER
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.2

            elif object[0] == 'B': # Box
                # Marker Colour
                marker.color.a = 1.0  # Alpha
                marker.color.r = 1.0  # Black
                marker.color.g = 1.0
                marker.color.b = 1.0

                marker.type = marker.CUBE
                marker.scale.x = 0.3
                marker.scale.y = 0.15
                marker.scale.z = 0.1
            self.marker_pub.publish(marker)
            print(f'Publishing marker ID: {marker.id} at x={marker.pose.position.x}, y={marker.pose.position.y}')


class Goto_Target(pt.behaviour.Behaviour):
    '''Retrieves coordinates '''
    def __init__(self, node, object: str):
        super().__init__("Go to Target")
        self.node = node
        self.object = object
        self.blackboard = pt.blackboard.Blackboard()

        self.objects = None
        self.targets = None
        self.target = None
        self.sampled_point = None
        self.target_point = None
        self.candidates = None
        self.arrived = False
        self.rotated = False

        # Initialise grid
        self.grid = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None

        self.blackboard.set('reset goto target', False)

        self.grid_sub = self.node.create_subscription(OccupancyGrid, '/map', self.grid_callback, 10)
        self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, '/map_pose', self.pose_callback, 10)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Ensures message delivery
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Keeps the last message for new subscribers
            depth=10  # Stores up to 10 messages in queue
            )
        self.waypoint_pub = self.node.create_publisher(Marker, '/goal_marker', qos)
        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

        self.done = False
        print('Initialising Coordinate Retriever...')

    def update(self):

        reset = self.blackboard.get('reset goto target')
        if self.status == pt.common.Status.INVALID or reset:
            self.reset()

            self.objects = self.blackboard.get('objects')
        
            # Save targets in list
            if self.object == 'object': # Any object (not box)
                self.targets = [obj for obj in self.objects if obj[0] != 'B']
            else:    
                self.targets = [obj for obj in self.objects if obj[0] == self.object]
            
            return pt.common.Status.RUNNING

        if self.done:
            return pt.common.Status.SUCCESS
        if self.grid is None:
            self.node.get_logger().info("No grid recieved")
            return pt.common.Status.RUNNING
        if self.objects is None:
            self.node.get_logger().info("No objects in list!")
            return pt.common.Status.FAILURE
        if self.arrived:
            if self.rotated:
                self.blackboard.set('reset approach', True)
                self.done = True
                self.node.get_logger().info("Go To Target behavior done. Continuing...")
                return pt.common.Status.SUCCESS
            else:
                self.node.get_logger().info("Rotating...")
                return pt.common.Status.RUNNING

        self.sample_point()
        return pt.common.Status.RUNNING

    def sample_point(self):
        if not self.sampled_point and self.pos:
            # Pick target out of list
            closest_target = None
            min_dist = np.inf
            for target in self.targets:
                ds = np.sqrt((target[1] - self.pos[0])**2 + (target[2] - self.pos[1])**2)
                if ds < min_dist:
                    closest_target = target
                    min_dist = ds
            self.target = closest_target
            
            if self.object != 'B':
                # Remove target from list of objects
                new_objects = []
                for tpl in self.objects:
                    if tpl != self.target:
                        new_objects.append(tpl)
                self.blackboard.set('objects', new_objects)

            # list of tuples (grid_x, grid_y) on a circle around target
            candidates = self.candidate_points(self.target) 
            
            # Target in grid indices
            t_x = int((self.target[1] - self.origin_x) / self.resolution)
            t_y = int((self.target[2] - self.origin_y) / self.resolution)
            self.target_point = (t_x, t_y)

            # Find the best candidate
            best_candidate = None
            max_free_cells = -1
            for point in candidates:
                # Count free cells around this candidate
                free_cells = self.count_free_cells_around(point, radius = 5)

                # Update best candidate
                if free_cells > max_free_cells:
                    max_free_cells = free_cells
                    best_candidate = point
            x = (best_candidate[0] * self.resolution) + self.origin_x    
            y = (best_candidate[1] * self.resolution) + self.origin_y
            z = np.arctan2(self.target[2] - y, self.target[1] - x)  

            self.sampled_point = (x, y, z)
            self.pub_goal_marker()

            self.node.get_logger().info("Go To Target: Sampled Point")
            # self.visualise_grid_and_targets()

    def grid_callback(self, msg: OccupancyGrid):
        if self.done:
            return

        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8)  
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # 2D np.array(). Unknown space = -1, free space  = 0, occupied = 100
        self.grid = data.reshape((height, width))  

    def candidate_points(self, target, distance = 0.50, angle_step = 15):
        '''Sample points on a circle with X cells radius around the target'''
        x0, y0 = target[1], target[2]
        self.candidates = []
        for angle_deg in range(0, 360, angle_step):
            angle_rad = np.radians(angle_deg)
            x = x0 + distance * np.cos(angle_rad)
            y = y0 + distance * np.sin(angle_rad)
            grid_x = int((x - self.origin_x) / self.resolution)
            grid_y = int((y - self.origin_y) / self.resolution)
            self.candidates.append((grid_x, grid_y))
        return self.candidates
    
    def count_free_cells_around(self, point, radius):
        free_count = 0
        x, y = point  # candidate = (grid_x, grid_y)

        # Iterate over the neighborhood in a square defined by the radius
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                ny, nx = y + dy, x + dx  # Check this neighboring cell (y, x)

                # Ensure the cell is within bounds and free
                if 0 <= ny < self.grid.shape[0] and 0 <= nx < self.grid.shape[1]:
                    if self.grid[ny, nx] <= 0:  # Free space
                        free_count += 1
        return free_count

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if self.done:
            return
        if self.target is None:
            return
        if self.sampled_point:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            self.pos = (x, y)

            heading = self.compute_heading(msg.pose.pose.orientation)
            goal_heading = self.sampled_point[2]

            if not self.arrived:
                dx, dy = np.abs(self.sampled_point[0] - x), np.abs(self.sampled_point[1] - y)
                dist = np.linalg.norm(np.array([dx, dy]))
                if dist < 0.1:
                    self.node.get_logger().info('Arrived at target!')    
                    self.arrived = True
            else:   
                if not self.rotated:
                    # Rotate 
                    d_z = goal_heading - heading
                    # self.node.get_logger().info(f'Dz = {d_z}')    
                    if np.abs(d_z) > np.deg2rad(5):
                        # Maximum velocities
                        wheel_radius = 0.046 # 0.04915
                        base = 0.3 # 0.30
                        max_factor = 1 / 6
                        max_rot = ((wheel_radius / base) / (np.pi/2)) * max_factor # rad/s
                        twist_msg = Twist()
                        twist_msg.angular.z = d_z / np.abs(d_z) * np.pi / 2 * max_rot
                        # self.node.get_logger().info(f"Sending twist msg {d_z} rad/s")
                        twist_msg._linear.z = max_factor
                        self.cmd_vel_pub.publish(twist_msg)
                    else: 
                        twist_msg = Twist()
                        self.cmd_vel_pub.publish(twist_msg)
                        self.rotated = True
                        self.node.get_logger().info('Robot is in position to look for the object!')

    def compute_heading(self, orientation):
        if isinstance(orientation, np.ndarray):
            x, y, z, w = orientation[0], orientation[1], orientation[2], orientation[3]
        else:
            x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        
        _, _, yaw = euler_from_quaternion((x, y, z, w))
        return yaw

    def pub_goal_marker(self, stop = False):
        if stop is True:
            z = -1.0
        else:
            z = 0.0
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.ns = "Target_Marker"
        marker.id = 0
        marker.type = Marker.SPHERE  # Use sphere to represent the goal
        marker.action = Marker.ADD
        marker.pose.position.x = self.sampled_point[0]
        marker.pose.position.y = self.sampled_point[1]
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Radius of the sphere
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.node.get_logger().info(f"Publishing goal marker!!!")
        self.waypoint_pub.publish(marker)
    
    def visualise_grid_and_targets(self):
        """
        Visualises occupancy grid and overlays targets and origin on top.
        """
        if self.grid is None:
            print("Grid is empty, can't visualise.")
            return

        fig, ax = plt.subplots(figsize=(10, 10))

        # Show the occupancy grid
        display_grid = self.grid.copy()
        display_grid = np.where(display_grid == -1, 127, display_grid)  # Make unknown gray
        ax.imshow(display_grid, cmap='gray', origin='lower')

        # Plot origin
        origin_col = int((0 - self.origin_x) / self.resolution)
        origin_row = int((0 - self.origin_y) / self.resolution)
        if 0 <= origin_row < self.grid.shape[0] and 0 <= origin_col < self.grid.shape[1]:
            ax.plot(origin_col, origin_row, 'b*', markersize=12, label='Origin')
            ax.text(origin_col + 1, origin_row + 1, 'Origin', color='blue', fontsize=8)
        else:
            print("Origin is out of grid bounds, skipping.")

        # Plot targets
        for name, x, y, yaw in self.targets:
            col = int((x - self.origin_x) / self.resolution)
            row = int((y - self.origin_y) / self.resolution)

            if 0 <= row < self.grid.shape[0] and 0 <= col < self.grid.shape[1]:
                ax.plot(col, row, 'ro')  # red dot
                ax.text(col + 1, row + 1, name, color='red', fontsize=8)
            else:
                print(f"Target {name} is out of grid bounds, skipping.")

        # Plot the sampled point
        if self.sampled_point:
            sp_x = int((self.sampled_point[0] - self.origin_x) / self.resolution) 
            sp_y = int((self.sampled_point[1] - self.origin_y) / self.resolution)
            ax.plot(sp_x, sp_y, 'go', markersize=8, label='Sampled Point')  # green dot
            ax.text(sp_x + 1, sp_y + 1, 'Sampled', color='green', fontsize=8)

        # Plot the target point
        if self.target_point:
            tp_x, tp_y = self.target_point[0], self.target_point[1]  # (col, row)
            ax.plot(tp_x, tp_y, 'bx', markersize=8, label='Target Grid')  # blue X
            ax.text(tp_x + 1, tp_y + 1, 'Target', color='blue', fontsize=8)

        # Plot all candidate points
        if self.candidates:
            for gx, gy in self.candidates:  # grid_x, grid_y
                if 0 <= gy < self.grid.shape[0] and 0 <= gx < self.grid.shape[1]:
                    ax.plot(gx, gy, 'y.', markersize=4)  # yellow dots


        ax.set_title("Occupancy Grid with Targets and Origin")
        plt.grid(False)
        plt.tight_layout()
        plt.show()

    def reset(self):
        self.objects = None
        self.targets = None
        self.target = None
        self.sampled_point = None
        self.target_point = None
        self.candidates = None
        self.arrived = False
        self.rotated = False

        # Initialise grid
        self.grid = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None

        self.done = False
        self.pos = None
        self.blackboard.set('reset goto target', False)


class Approach_Object(pt.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__('Approaching Object')

        self.node = node
        
        self.x = 0
        self.y = 0
        self.distance_to_target = None

        self.object_found = False
        self.done = True

        self.blackboard = pt.blackboard.Blackboard()
        self.blackboard.set('reset approach', False)

        # Subscriber to the point cloud topic
        self._sub = self.node.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)
        # Subscriber to Odom topic
        self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.pose_callback, 10)
        
        # Publisher to velocity command topic   
        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        
        # Call control algorithm
        self.timer = self.node.create_timer(0.2, self.control)  # ogni 100ms

        # Initialize the transform buffer
        self.tf_buffer = Buffer()

        # Initialize the transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.node.get_logger().info("Approaching object...")
     
    def update(self):
        reset = self.blackboard.get('reset approach')
        if self.status == pt.common.Status.INVALID or reset:
            self.reset()
            self.node.get_logger().info("Running Approach Object Node")
            return pt.common.Status.RUNNING

        # Check if done
        if self.done:
            self.node.get_logger().info("Ready for pickup service!")
            return pt.common.Status.SUCCESS

        return pt.common.Status.RUNNING
    
    def pose_callback(self, msg : PoseWithCovarianceStamped):
        # Init transform
        # to_frame_rel = 'base_link'
        # from_frame_rel = 'odom'
        # time = rclpy.time.Time().from_msg(msg.header.stamp)

        # # Wait for the transform asynchronously
        # tf_future = self.tf_buffer.wait_for_transform_async(
        # target_frame=to_frame_rel,
        # source_frame=from_frame_rel,
        # time=time
        # )
        # rclpy.spin_until_future_complete(self.node, tf_future, timeout_sec=1)

        # # Lookup tansform
        # try:
        #     t = self.tf_buffer.lookup_transform(to_frame_rel,
        #                                     from_frame_rel,
        #                                     time)
        #     # Do the transform
        #     map_pose = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, t)

        #     # Get position of robot
        #     self.x = map_pose.position.x
        #     self.y = map_pose.position.y
            
        #     q = [
        #         map_pose.orientation.x,
        #         map_pose.orientation.y,
        #         map_pose.orientation.z,
        #         map_pose.orientation.w
        #     ]
        #     angles = euler_from_quaternion(q)
        #     self.theta=angles[2]
        # except TransformException:
        #     self.node.get_logger().info('No transform found')

        #Get position of robot

        if self.done:
            return
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        angles = euler_from_quaternion(q)
        self.theta = angles[2]
                
    def voxel_grid_filter(self, points, leaf_size=0.05):
        """Downsamples the point cloud using a voxel grid filter."""
        # Create voxel grid by downsampling points to grid size
        # Points are binned by floor division on leaf size.
        grid_indices = np.floor(points[:, :3] / leaf_size).astype(int)
        unique_grid_indices = np.unique(grid_indices, axis=0)
        
        # For each grid cell, compute the centroid
        downsampled_points = []
        for idx in unique_grid_indices:
            mask = np.all(grid_indices == idx, axis=1)
            points_in_cell = points[mask]
            centroid = np.mean(points_in_cell, axis=0)
            downsampled_points.append(centroid)

        return np.array(downsampled_points)      

    def cloud_callback(self, msg: PointCloud2):
        """
        Processes the latest point cloud and stores detected objects.
        Detects objects using DBSCAN clustering, volume and shape detection, and publishes bounding boxes.
        """

        if self.object_found or self.done:
            return
        # Transformation
        to_frame_rel = 'odom'
        from_frame_rel = msg.header.frame_id

        time = rclpy.time.Time().from_msg(msg.header.stamp)

        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        rclpy.spin_until_future_complete(self.node, tf_future, timeout_sec=1)

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                time)
        except TransformException as ex:
            self.node.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        

        # Cloud Processing
        self.latest_cloud = msg  # Store latest cloud
        points = pc2.read_points_numpy(msg, skip_nans=True)[:, :3]
        colors = pc2.read_points_numpy(msg, skip_nans=True)[:, 3]

        distances = np.linalg.norm(points[:, :3], axis=1)
        offset = 0.089
        mask = (distances <= 1) & (distances >= 0.2) & (points[:, 1] < offset) & (points[:, 1] > offset - 0.3)
        filtered_points = points[mask]
        filtered_distances=distances[mask]

        if filtered_points.shape[0] == 0:
            # self.node.get_logger().info("No points after filtering")
            return
        
        # Stimiamo il piano del pavimento dai punti più bassi (es. y vicino a 0)
        floor_mask = filtered_points[:, 1] < (offset - 0.25)  # Prendiamo i più bassi
        floor_points = filtered_points[floor_mask]

        if floor_points.shape[0] > 50:
            # Fit piano al pavimento con RANSAC
            from sklearn.linear_model import RANSACRegressor

            # Piano z = ax + by + c (y è asse verticale in ROS)
            X = floor_points[:, [0, 2]]
            y = floor_points[:, 1]
            ransac = RANSACRegressor(residual_threshold=0.01, max_trials=100)
            ransac.fit(X, y)
            a, b = ransac.estimator_.coef_
            c = ransac.estimator_.intercept_

            # Normale al piano del pavimento
            floor_normal = np.array([-a, 1.0, -b])
            floor_normal /= np.linalg.norm(floor_normal)
        else:
            floor_normal = np.array([0, 1, 0])  # fallback a verticale pura

        # Downsample using voxel grid filter
        #filtered_points = self.voxel_grid_filter(filtered_points)  
        
        # Clustering
        db = DBSCAN(eps=0.015, min_samples=80)
        labels = db.fit_predict(filtered_points)

        _, counts = np.unique(labels, return_counts=True)
        max_cluster_size = 8000

        filt_labels = np.array([
            -1 if counts[label] > max_cluster_size else label   
            for label in labels
        ])

        updated_target = False 

        unique_labels = set(filt_labels)

        for label in unique_labels:
            if label == -1:
                continue

            cluster_mask = labels == label
            cluster_points = filtered_points[cluster_mask]
            cluster_distances=filtered_distances[cluster_mask]
            cluster_distance=np.mean(cluster_distances)

            #self.get_logger().info(f"Cluster #{label} → {cluster_points.shape[0]} punti")

            if np.sum(cluster_points[:,1] < (offset - 0.125)) > 10: #skip cluster if too high (20 points are above the max)
                continue

            if cluster_points.shape[0] < 10 and cluster_points.shape[0] > max_cluster_size:
                continue

            if cluster_points.shape[0] > 3:
                centroid = np.mean(cluster_points, axis=0)
                centered = cluster_points - centroid
                centered = self.voxel_grid_filter(centered)

                # Safety check: SVD needs at least 3 points in 3D
                if centered.shape[0] >= 3 and centered.shape[1] == 3:
                    try:
                        _, _, vh = np.linalg.svd(centered)
                        cluster_normal = vh[2, :]
                        cluster_normal /= np.linalg.norm(cluster_normal)

                        cos_angle = np.dot(cluster_normal, floor_normal)
                        angle = np.arccos(np.clip(abs(cos_angle), -1.0, 1.0)) * 180 / np.pi

                        if angle < 50:
                            continue
                    except np.linalg.LinAlgError as e:
                        self.node.get_logger().warn(f"SVD fallita per cluster con shape {centered.shape}: {e}")
                        continue
                else:
                    self.node.get_logger().warn(f"Cluster saltato: shape dopo voxel filtering = {centered.shape}")
                    continue


            bbox_min = np.min(cluster_points, axis=0)
            bbox_max = np.max(cluster_points, axis=0)
            bbox_size = bbox_max - bbox_min
            bbox_center = (bbox_min + bbox_max) / 2

            #self.node.get_logger().info('Deteced')
            
            x_lim = 0.5 # to skip everithing not centered
            x_min, x_max = -x_lim, x_lim
            if (bbox_min[0] < x_min or bbox_max[0] > x_max):
                continue        

            bbox_pose = PoseStamped()
            bbox_pose.header.frame_id = msg.header.frame_id
            bbox_pose.header.stamp = msg.header.stamp
            bbox_pose.pose.position.x = bbox_center[0]
            bbox_pose.pose.position.y = bbox_center[1]
            bbox_pose.pose.position.z = bbox_center[2]
            bbox_pose.pose.orientation.w = 1.0

            try:
                map_pose = tf2_geometry_msgs.do_transform_pose(bbox_pose.pose, t)
                self.x_obj = map_pose.position.x
                self.y_obj = map_pose.position.y

                updated_target = True
                self.node.get_logger().info("object found")
                self.object_found = True

            except TransformException as ex:
                self.node.get_logger().warn(f"Trasformazione bbox fallita: {ex}")
                return            
        
        if not updated_target and self.distance_to_target is not None:
            pass
            #self.node.get_logger().info("no valid object found. continuing to the last target.")
    
    def transform_to_matrix(self,t):
        trans = t.transform.translation
        rot = t.transform.rotation

        translation = np.array([trans.x, trans.y, trans.z])
        quaternion = [rot.x, rot.y, rot.z, rot.w]

        transform_mat = quaternion_matrix(quaternion)
        transform_mat[0:3, 3] = translation
        return transform_mat
    
    def control(self):
        if self.done:   
            return

        if self.theta is None or self.x_obj is None or self.y_obj is None:
            return

        x = self.x_obj - self.x
        y = self.y_obj - self.y

        self.distance_to_target = math.sqrt(x**2 + y**2)

        wheel_radius = 0.046 # 0.04915
        base = 0.3 # 0.30
        max_factor = 1 / 6

        max_vel = wheel_radius * max_factor # m/s
        max_rot = ((wheel_radius / base) / (np.pi/2)) * max_factor # rad/s
        self.node.get_logger().info(f'Distance to target: {self.distance_to_target}')
        if self.distance_to_target is not None and self.distance_to_target < 0.20: # ----------------------------------------- 
            twist_msg = Twist()
            self.cmd_vel_pub.publish(twist_msg)
            self.done = True    
        else:
            # Calculate the steering angle to the target point
            steering_angle = self.calculate_steering_angle((self.x, self.y), self.theta, (self.x_obj, self.y_obj))
            
            # Calculate linear velocity
            linear_velocity = (1 - abs(steering_angle) / (np.pi / 2)) * max_vel

            # Calculate the angular velocity
            angular_velocity = steering_angle * max_rot

            # Create a ROS Twist message
            twist_msg = Twist()

            twist_msg.linear.x = linear_velocity
            twist_msg.linear.z = max_factor
            twist_msg.angular.z = angular_velocity

            self.cmd_vel_pub.publish(twist_msg)

    def calculate_steering_angle(self, current_position, current_heading, target_point):
        # Vector from current position to target point
        vector_to_target = target_point - np.array(current_position)

        # Calculate the angle to the target point
        angle_to_target = np.arctan2(vector_to_target[1], vector_to_target[0])

        # Steering angle is the difference between the vehicle's heading and the angle to the target
        steering_angle = angle_to_target - current_heading

        # Normalize the steering angle to be in the range of -pi to pi
        steering_angle = (steering_angle + np.pi) % (2 * np.pi) - np.pi

        # Constrain the steering angle to be within the range of -pi/2 to pi/2 to not drive backwards
        if steering_angle > np.pi / 2:
            steering_angle = np.pi / 2
        elif steering_angle < -np.pi / 2:
            steering_angle = -np.pi / 2
        # Constrain the steering angle to not turn as much
        elif np.pi / 4 < steering_angle <= np.pi / 2:
            steering_angle = np.pi / 2
        elif -np.pi / 2 <= steering_angle < -np.pi / 4:
            steering_angle = -np.pi / 2

        return steering_angle

        # x = self.x_obj - self.x
        # y = self.y_obj - self.y

        # self.desired_angle = math.atan2(y, x)
        # self.distance_to_target = math.sqrt(x**2 + y**2)

        # angular_error = self.theta - self.desired_angle       
 
        # wheel_radius = 0.046 # 0.04915
        # base = 0.3 # 0.30
        # max_factor = 1 / 6

        # max_vel = wheel_radius * max_factor # m/s
        # max_rot = ((wheel_radius / base) / (np.pi/2)) * max_factor # rad/s

        # kw = -0.02 
        # kv = 0.5

        # v = 0
        # w = 0
        # self.node.get_logger().info(f'Distance to target: {self.distance_to_target}')

        # if self.distance_to_target is not None and self.distance_to_target < 0.25:
        #     v = 0.0
        #     w = 0.0
        #     self.node.get_logger().info('Arrived at target.')

        #     # Create a ROS Twist message
        #     twist_msg = Twist()
        #     twist_msg.linear.x = float(v)
        #     twist_msg.linear.y = 0.0
        #     twist_msg.linear.z = float(max_factor)
        #     twist_msg.angular.x = 0.0
        #     twist_msg.angular.y = 0.0
        #     twist_msg.angular.z = float(w)
        #     self.cmd_vel_pub.publish(twist_msg)

        #     a = str('1') - 1
        #     return pt.common.Status.SUCCESS
        # # elif abs(angular_error) > angular_threshold:
        # #     w=kw*angular_error
        # #     v=0
        # # else:
        # #     w=0 #-kw*angular_error*max_rot
        # #     v=kv*max_vel

        # w=kw*angular_error
        # v=kv*max_vel

        # w=max(min(w,max_rot),-max_rot)
        # v=max(min(v,max_vel),-max_vel)

        # # Create a ROS Twist message
        # twist_msg = Twist()
        # twist_msg.linear.x = float(v)
        # twist_msg.linear.y = 0.0
        # twist_msg.linear.z = float(max_factor)
        # twist_msg.angular.x = 0.0
        # twist_msg.angular.y = 0.0
        # twist_msg.angular.z = float(w)
        # self.cmd_vel_pub.publish(twist_msg)

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.x_obj = None
        self.y_obj = None

        self.distance_to_target = None
        self.latest_cloud = None

        self.object_found = False
        self.done = False

        self.blackboard.set('reset approach', False)


class Pickup(pt.behaviour.Behaviour):
    def __init__(self,node):
        super().__init__("Pickup")
        self.node = node
        self.cli = self.node.create_client(Arm, 'arm')
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Arm service not available, waiting again...')
        self.req = Arm.Request()
        self.blackboard = pt.blackboard.Blackboard()
        self.request_sent = False
        self.look_response = None
        self.grab_pos_response = None
        self.grab_response = None
        self.drive_response = None
        self.sent_look = False
        self.sent_pick = False
        self.sent_grab = False
        self.progress = 0

        self.test = True

    def update(self):
        if self.test:
            self.test = False
            self.blackboard.set('reset goto target', True)
        return pt.common.Status.RUNNING

        if self.progress == 0:
            obj_class = self.blackboard.get('Target_type')
            
            if obj_class is None:
                self.node.get_logger().error("No object class specified on blackboard.")
                return pt.common.Status.FAILURE
            self.req.obj_class = obj_class
            self.progress = 1
        elif self.progress == 1: # Here the robot will look and return if it sees
            print("6")
            if not self.sent_look:
                self.look_response = self.cli.send_request(2,[],[]) 
                self.sent_look = True
            if self.look_response != None and self.look_response.success:
                self.progress = 3
                print("success: "+ str(self.look_response.success))
                self.look_response.success = False
        elif self.progress == 3:
            self.obj_grabbed = False
            print("going to pick now")
            time.sleep(2.5) # might swith to checking that time is 2.5 more than at start
            if not self.sent_pick:
                self.grab_pos_response = self.cli.send_request(6,[],[]) 
                self.sent_pick = True
            if self.grab_pos_response != None and self.grab_pos_response.success:
                self.progress = 4
            elif self.look_response != None and not self.look_response.success:
                self.progress = 5
        elif self.progress == 4:
            time.sleep(2.0)
            print("Grabbing now")
            time.sleep(1.0)
            if not self.sent_grab:
                self.grab_response = self.cli.send_request(7,self.grab_pos_response.arm_pos,[])
                self.sent_grab = True
            if self.grab_response != None and self.grab_response.success:
                self.progress = 6
                response = self.cli.send_request(8,[], self.grab_response.xyfix)
        elif self.progress == 5:
            print("Driving now, error is: " + self.grab_response.message + " and obj is at: " + str(self.grab_response.xyfix))

            if not self.sent_drive:
                self.drive_response = self.cli.send_request(8,[],self.grab_response.xyfix) 
                self.sent_drive = True
            
            if self.drive_response != None and self.drive_response.success:
                self.progress == 3
        elif self.progress == 6:
            return pt.common.Status.SUCCESS
        

        if self.future.done():
            if self.future.result() is not None:
                response = self.future.result()
                if response.success:
                    self.node.get_logger().info("pickup successful")
                    return pt.common.Status.SUCCESS
                else:
                    self.node.get_logger().info("pickup failed: " + response.message)
                    return pt.common.Status.FAILURE
            else:
                self.node.get_logger().error(f"Service call failed:")
                return pt.common.Status.FAILURE
            
        
        # waiting
        return pt.common.Status.RUNNING

    def send_request(self, command, obj_class):
        self.req.xy[0] = command
        self.req.obj_class = obj_class
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def pickup(self,obj='1'): # cube
        response = self.send_request(2,obj)
        self.node.get_logger().info('Response from arm is: ' + str(response.success))
           

class Place(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Pickup")
    def update(self):
        pass


class Detection(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Detection")
    def update(self):
        pass


class Check_Map(pt.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__("Detection")
        self.node = node
        self.blackboard = pt.blackboard.Blackboard()

        self.node.get_logger().info("Checking if there are objects in the map")

    def update(self):
        objects = self.blackboard.get('objects')

        obj_counter = 0
        for obj in objects:
            if obj[0] != 'B':
                obj_counter += 1

        if obj_counter > 0:
            return pt.common.Status.FAILURE
        else:
            return pt.common.Status.SUCCESS

def main():
    rclpy.init()
    node = BehaviourTree()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()