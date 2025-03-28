import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import os

import numpy as np
import matplotlib.pyplot as plt

import py_trees as pt
import py_trees_ros as ptr

from ament_index_python import get_package_share_directory

from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

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
        explore_unknown = ExploreUknownSpace((self)) 

        test_seq = pt.composites.Sequence(name = 'Test Sequence', 
                                          memory = bool,
                                          children = [create_ws, waypoints, explore_samples, explore_unknown]
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
        ws_path = os.path.join(package_share_dir, 'data', 'workspace_2.tsv')
        
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

        # Find 'center' of workspace
        x_cords = [point[0] for point in self.ws]
        y_cords = [point[1] for point in self.ws]
        center = (np.mean(x_cords), np.mean(y_cords))


        for idx, corner in enumerate(self.ws):
            # Sample every corner and add offset to move it towards center
            # if idx == len(self.ws) - 1:
            #     next_corner = self.ws[0]
            # else:
            #     next_corner = self.ws[idx + 1]
            
            # dx, dy = next_corner[0] - corner[0], next_corner[1] - corner[1] 
            # orthogonal = (dx, -dy) / np.linalg.norm(np.array([dx, -dy])) + (dx/2, dy/2)

            # sampled_point = (orthogonal[0] - corner[0], orthogonal[1] - corner[1])
            # self.waypoints.append(tuple(sampled_point))

            offset = np.array(center) - np.array(corner) 
            offset /= np.linalg.norm(offset) # Normalise to 1m
            sampled_point = np.array(corner) + offset * 0.5
            self.waypoints.append(tuple(sampled_point))

        self.plot_workspace()
        # Save waypoints
        self.blackboard.set('waypoints', self.waypoints)
        return pt.common.Status.SUCCESS
    

    def plot_workspace(self):
        '''Plots the workspace boundary and sampled waypoints'''
        if not self.ws:
            print("No workspace to plot.")
            return
        
        # Extract coordinates
        x_vals, y_vals = zip(*self.ws + [self.ws[0]])  # Close the shape
        way_x, way_y = zip(*self.waypoints) if self.waypoints else ([], [])
        
        # Plot
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

        qos = QoSProfile(
                        reliability=QoSReliabilityPolicy.RELIABLE,  # Ensures message delivery
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Keeps the last message for new subscribers
                        depth=10  # Stores up to 10 messages in queue
                        )
        
        self.target_pub = self.node.create_publisher(Marker, '/goal_marker', qos)
        self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, 'map_pose', self.pose_callback, 10)
        self.target = None

    def update(self):
        if self.status == pt.common.Status.INVALID:
            self.waypoints = self.blackboard.get('waypoints')
            if not self.waypoints:
                print("No waypoints sampled...")
                return pt.common.Status.FAILURE
            else:
                print('Exploring lots!')
                return pt.common.Status.RUNNING

        if len(self.waypoints) == 0:
            print("All waypoints have been visited!")
            return pt.common.Status.SUCCESS

        # Pick out target
        if self.target is None:
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

    def pub_goal_marker(self):
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
        marker.pose.position.z = 0.0
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
            qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  
            depth=10        ) 
            self.grid_sub = self.node.create_subscription(OccupancyGrid, '/map', self.grid_callback, 10)
            self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, '/map_pose', self.pose_callback, 10)
            self.target_pub = self.node.create_publisher(Marker, '/goal_marker', qos)
            print('Exploring Remaining unknown space')
            return pt.common.Status.RUNNING
        
        # Check if everything is explored
        if self.unknown < 0.05: # 5%
            print("Good job! The entire workspace has been explored :)")
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

        
    def pub_goal_marker(self):
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
        marker.pose.position.z = 0.0
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