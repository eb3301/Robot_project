import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import os

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import py_trees as pt
import py_trees_ros as ptr

from ament_index_python import get_package_share_directory

from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

from std_srvs.srv import SetBool
from detect_interfaces.srv import DetectObjects

from shapely.geometry import Polygon, LineString
from shapely.geometry import Point as shapelyPoint
import random

class BehaviourTree(Node):
    def __init__(self):
        super().__init__('Behaviour_Tree_Node')
        self.get_logger().info("Initialising Behaviour Tree")
        
        # Initialise blackboard variables
        self.blackboard = pt.blackboard.Blackboard()

        # Initialise Behaviours
        create_ws = Create_ws(self)
        waypoints = Sample_Waypoints(self)
        explore_samples = ExploreSamples(self)

        #explore_unknown = ExploreUknownSpace((self)) --- Break in case of emergency

        test_seq = pt.composites.Sequence(name = 'Test Sequence', 
                                          memory = bool,
                                          children = [create_ws, waypoints, explore_samples]
                                          )

        self.BT = pt.trees.BehaviourTree(root = test_seq)

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
        
        self.blackboard.set('workspace', self.coords)
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
        #print("Published workspace marker from file")


class Sample_Waypoints(pt.behaviour.Behaviour):
    '''Samples waypoints over the workspace to be used for exploration'''
    def __init__(self, node):
        super().__init__('Sample Waypoints')
        self.node = node
        self.blackboard = pt.blackboard.Blackboard()
        self.waypoints = []
        

    def update(self):
        if self.status == pt.common.Status.INVALID:
            self.ws = self.blackboard.get('workspace')
            print('Sampling Waypoints...')
            return pt.common.Status.RUNNING
        
        if not self.ws:
            print('No workspace loaded...')
            return pt.common.Status.FAILURE

        vertices = list(self.ws)
        if vertices[0] != vertices[-1]:
            vertices.append(vertices[0])
        polygon = Polygon(vertices)

        # Add padding to the walls
        PADDING_VALUE = 0.17
        polygon = polygon.buffer(-PADDING_VALUE)

        UNIFORMITY_RADIUS = 0.7  # distanza minima tra candidati (Poisson disk)
        NUM_UNCOVERED_SAMPLES = 100

        candidates = self.poisson_disk_sample(polygon, radius=UNIFORMITY_RADIUS)
        self.waypoints = self.greedy_visibility_coverage(polygon, candidates, resolution=NUM_UNCOVERED_SAMPLES)
        #self.node.get_logger().info(f"{self.waypoints}")
        
        #self.plot_workspace()
        
        # Save waypoints
        self.blackboard.set('waypoints', self.waypoints)
        return pt.common.Status.SUCCESS
    

    def poisson_disk_sample(self,polygon, radius, k = 30):
        minx, miny, maxx, maxy = polygon.bounds
        cell_size = radius / np.sqrt(2)
        grid = {}
        points = []
        active = []


        def grid_coords(p):
            return int((p.x - minx) / cell_size), int((p.y - miny) / cell_size)


        def fits(p):
            gx, gy = grid_coords(p)
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    neighbor = grid.get((gx + dx, gy + dy))
                    if neighbor and p.distance(neighbor) < radius:
                        return False
            return polygon.contains(p)

        # first casual point
        p0 = None
        while not p0 or not polygon.contains(p0):
            p0 = shapelyPoint(random.uniform(minx, maxx), random.uniform(miny, maxy))
        points.append(p0)
        active.append(p0)
        grid[grid_coords(p0)] = p0

        while active:
            idx = random.randint(0, len(active) - 1)
            center = active[idx]
            found = False
            for _ in range(k):
                angle = random.uniform(0, 2 * np.pi)
                r = random.uniform(radius, 2 * radius)
                px = center.x + r * np.cos(angle)
                py = center.y + r * np.sin(angle)
                p = shapelyPoint(px, py)
                if fits(p):
                    points.append(p)
                    active.append(p)
                    grid[grid_coords(p)] = p
                    found = True
            if not found:
                active.pop(idx)
        return points

    # === 3. Campiona i punti ===
    def sample_points_within_polygon(self,polygon, num_points):
        minx, miny, maxx, maxy = polygon.bounds
        points = []
        while len(points) < num_points:
            p = shapelyPoint(random.uniform(minx, maxx), random.uniform(miny, maxy))
            if polygon.contains(p):
                points.append(p)
        return points

    # === 4. Algoritmo greedy con raggio limitato ===
    def greedy_visibility_coverage(self,polygon, candidates, resolution=600):
        uncovered = self.sample_points_within_polygon(polygon, resolution)
        self.waypoints = []
        MAX_RANGE = 0.2  

        while uncovered:
            best_point = None
            best_covered = []

            for p in candidates:
                visible = []
                for u in uncovered:
                    if p.distance(u) <= MAX_RANGE:
                        line = LineString([p, u])
                        if line.within(polygon):
                            visible.append(u)

                if len(visible) > len(best_covered):
                    best_covered = visible
                    best_point = p

            if not best_point:
                break

            self.waypoints.append((best_point.x, best_point.y))
            #self.node.get_logger().info(f"{self.waypoints}")
            uncovered = [u for u in uncovered if u not in best_covered]
        return self.waypoints

    def plot_workspace(self):
        '''Plots the workspace boundary and sampled waypoints'''
        if not self.ws:
            print("No workspace to plot.")
            return
        
        # Extract coordinates
        x_vals, y_vals = zip(*self.ws + [self.ws[0]])  # Close the shape
        # way_x, way_y = zip(*self.waypoints) if self.waypoints else ([], [])

        if self.waypoints:
            way_x = [p[0] for p in self.waypoints]
            way_y = [p[1] for p in self.waypoints]
        else:
            way_x, way_y = [], []

        
        # # Plot
        plt.figure(figsize=(6,6))
        plt.plot(x_vals, y_vals, marker='o', linestyle='-', color='b', label='Workspace Boundary')
        plt.scatter(way_x, way_y, color='r', marker='x', label='Sampled Waypoints')
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")
        plt.title("Workspace and Sampled Waypoints")
        plt.grid(True, linestyle="--", linewidth=0.5)
        plt.legend()
        plt.show()



class ExploreSamples(pt.behaviour.Behaviour):
    '''Explores waypoints'''
    def __init__(self, node):
        super().__init__('Explore Pre-Sampled Points')
        self.node = node
        self.blackboard = pt.blackboard.Blackboard

        self.target = None
        self.target_in_occupied = False
        self.detection_active = False

    def update(self):
        if self.status == pt.common.Status.INVALID:
            # Get waypoints
            self.waypoints = self.blackboard.get('waypoints')
            
            # Initialise sub/pub
            qos = QoSProfile(
                            reliability=QoSReliabilityPolicy.RELIABLE,  # Ensures message delivery
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Keeps the last message for new subscribers
                            depth=10  # Stores up to 10 messages in queue
                            )
            
            self.target_pub = self.node.create_publisher(Marker, '/goal_marker', qos)
            self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, 'map_pose', self.pose_callback, 10)
            self.grid_sub = self.node.create_subscription(OccupancyGrid, 'map', self.grid_callback, 10)
            
            # Initialise detection
            self.start_client = self.node.create_client(SetBool, '/start_detection')
            self.stop_client = self.node.create_client(SetBool, '/stop_detection')
            self.detect_client = self.node.create_client(DetectObjects, '/detect_objects')

            if not self.waypoints:
                print("No waypoints sampled...")
                return pt.common.Status.FAILURE
            else:
                # Start detection service
                print('Time to explore - Starting detection client')
                self.call_set_bool(self.start_client, True)
                self.detection_active = True
                return pt.common.Status.RUNNING

        if len(self.waypoints) == 0:
            print("All waypoints have been visited - Stopping detection client")
            self.call_set_bool(self.stop_client, True)
            self.pub_goal_marker(stop = True)
            return pt.common.Status.SUCCESS

        # Pick out target
        if self.target is None or self.target_in_occupied:
            self.target_in_occupied = False
            self.target = self.waypoints.pop(0)
            self.blackboard.set('waypoints', self.waypoints)
            self.pub_goal_marker()

        print('Exploring...')        
        return pt.common.Status.RUNNING
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if self.target:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            
            dx, dy = np.abs(self.target[0] - x), np.abs(self.target[1] - y)
            dist = np.linalg.norm(np.array([dx, dy]))

            if dist < 0.1:
                print('Arrived at target!')
                self.target = None   

    def grid_callback(self, msg: OccupancyGrid):
        if self.target:
            x, y = self.target[0], self.target[1]
        if True: # CHECK IF TARGET IS IN OCCUPIED SPACE
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.int8)  
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y

            # 2D np.array(). Unknown space = -1, free space  = 0, occupied = 100
            grid = data.reshape((height, width))  

            gx, gy = int((x - origin_x) / resolution), int((y - origin_y) / resolution)

            if grid[gy, gx] > 0:
                self.node.get_logger().info("Target is in occupied space. Skipping to next sampled waypoint")
                self.target_in_occupied = True

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
        marker.pose.position.x = self.target[0]
        marker.pose.position.y = self.target[1]
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
        #print('Published goal marker')
        self.target_pub.publish(marker)








class ExploreUknownSpace(pt.behaviour.Behaviour):
    '''Check if there is remaining unknown space and explore it'''
    def __init__(self, node):
        super().__init__('Explore Unknown Space')
        self.node = node
        self.blackboard = pt.blackboard.Blackboard
        
        self.grid = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None

        self.pose = None
        self.target = None

        self.unknown = 1 # Initialise as 100% unknown space

    def update(self):
        # Check if it is first time running 
        if self.status == pt.common.Status.INVALID:

            # Initialise sub/pub
            qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  
            depth=10        ) 
            self.grid_sub = self.node.create_subscription(OccupancyGrid, '/map', self.grid_callback, 10)
            self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, '/map_pose', self.pose_callback, 10)
            self.target_pub = self.node.create_publisher(Marker, '/goal_marker', qos)

            # Initialise detection
            self.start_client = self.node.create_client(SetBool, '/start_detection')
            self.stop_client = self.node.create_client(SetBool, '/stop_detection')
            self.detect_client = self.node.create_client(DetectObjects, '/detect_objects')
            print('Exploring Remaining unknown space -- Starting detection client')

            # Start detection
            print('Time to explore - Starting detection client')
            self.call_set_bool(self.start_client, True)
            return pt.common.Status.RUNNING
        
        # Check if everything is explored
        if self.unknown < 0.05: # 5%
            print("Good job! The entire workspace has been explored -- Stopping detection client")
            self.call_set_bool(self.stop_client, True)
            self.pub_goal_marker(stop = True)
            return pt.common.Status.SUCCESS

        # Check if grid as been recieved
        if self.grid is None:
            print('No grid recieved')
            return pt.common.Status.RUNNING # Idk maybe return failre instead?
    
        if not self.pose:
            print('No current pose recieved...')
            return pt.common.Status.RUNNING
        
        if self.target is not None:
            print('No target...')
            return pt.common.Status.RUNNING

        # Sample closest unknown space
        unknown_coords = []
        for gy in range(self.grid.shape[0]):
            for gx in range(self.grid.shape[1]):
                if self.grid[gy, gx] == 0:
                    # Convert from cell index to world coordinates
                    wx = self.origin_x + gx * self.resolution
                    wy = self.origin_y + gy * self.resolution

                    unknown_coords.append([wx, wy])
        unknown_coords = np.array(unknown_coords)
        pose = np.array(self.pose)

        dist_array = unknown_coords - pose
        min_dist_idx = np.argmin(np.linalg.norm(dist_array, axis = 1))
        
        self.target = tuple(unknown_coords[min_dist_idx])
        self.pub_goal_marker()
        return pt.common.Status.RUNNING
    

    def grid_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data, dtype=np.int8)  
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        # 2D np.array(). Unknown space = -1, free space  = 0, occupied = 100
        self.grid = data.reshape((height, width))  


    def pose_callback(self, msg: PoseWithCovarianceStamped):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.pose = (x, y)

        if self.target:
            dx, dy = np.abs(self.target[0] - x), np.abs(self.target[1] - y)
            dist = np.linalg.norm(np.array([dx, dy]))

            if dist < 0.1:
                print('Arrived at target!')
                self.target = None 
    
    def check_uknown_space(self):
        # Count cells in grid
        n_free_cells = sum(1 for cell in self.grid if cell == 0)
        n_unknown_cells = sum(1 for cell in self.grid if cell == -1)
        n_total = n_free_cells + n_unknown_cells

        # Calculate percentage unknown cells
        self.unknown = n_unknown_cells / n_total

        
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
        marker.pose.position.x = self.target[0]
        marker.pose.position.y = self.target[1]
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
        #print('Published goal marker')
        self.target_pub.publish(marker)

    def call_set_bool(self, client, value: bool):
        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Service not available")
            return
        req = SetBool.Request()
        req.data = value
        client.call_async(req)
           
        
        


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