#!/usr/bin/env python3

import rclpy
import numpy as np
import tf2_ros
from tf2_ros import TransformException
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist

# Only for test function
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

class Planner(Node):

  def __init__(self):
    super().__init__('Astar_Planner')
    self.get_logger().info(f"Starting planner")

    qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE, # Does not lose msg
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, # New subscribers get msg after pub
            depth = 1
        )

    # Subscribe to occupancy grid and costmap
    self.costmap_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)

    # Subscribe to current pose
    self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.pose_callback, 10) # This might need to be restricted
    
    # Subscribe to goal pose
    self.goal_sub = self.create_subscription(Marker, "/goal_marker", self.goal_callback, 1)

    # Publish the planned path
    self.path_pub = self.create_publisher(Path, "/planned_path", qos)

    # Publish the zero velocity when planning
    self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    # TF2 - to handle transforms from map to odom
    self.buffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.buffer, self, spin_thread = True)

    # Initiation of robot pose coordnates
    self.x0 = 0.0
    self.y0 = 0.0
    self.theta0 = 0

    # Initiation of target coordinates
    self.goal_received = False
    self.xt = 4.5
    self.yt = 0.0

    # Initiation of path
    self.planned = False
    self.path = []


  # Get the most recent map all the time. Also logic for plan and replan
  def map_callback(self, msg : OccupancyGrid):
    if self.goal_received:
      # Get important data from message
      map_data = msg.data
      map_data = np.reshape(map_data, (msg.info.height, msg.info.width))
      resolution = msg.info.resolution
      origin_x = msg.info.origin.position.x
      origin_y = msg.info.origin.position.y
      self.origin = (origin_x, origin_y)
      time = msg.header.stamp

      # Logic for deciding plan and when to replan
      if self.planned: 
        try:
          if not self.path:
            self.get_logger().info(f"Path is empty, should replan")
          else:
            for i in range(len(self.path)):
              x_index = int((self.path[i][0] - origin_x) / resolution)  
              y_index = int((self.path[i][1] - origin_y) / resolution)
              if map_data[y_index][x_index] >= 80:
                self.planned = False
                # self.get_logger().info(f"Path obstructed at ({self.path[i][0]}, {self.path[i][1]})")
                break
        except:
          self.get_logger().info(f"Path is empty")

      # Replan
      if not self.planned:
        self.get_logger().info("Planning new path")
        self.plan_path(map_data, resolution, time)
        self.planned = True


  # The actual path planning and publishing of the path
  def plan_path(self, map_data, resolution, time):
    # Check if the current pose is occupierd or not, to be able to plan
    x_index = int((self.x0 - self.origin[0]) / resolution)  
    y_index = int((self.y0 - self.origin[1]) / resolution)
    if map_data[y_index][x_index] >= 80:
      # Current pose occupied, search for candidates around
      radius = 1  # Start expanding from radius 1
      x0, y0 = None, None
      while x0 is None and y0 is None:
        for dy in range(-radius, radius + 1):
          for dx in range(-radius, radius + 1):
            if abs(dy) + abs(dx) == radius:
              ny, nx = y_index + dy, x_index + dx
              if map_data[ny][nx] < 80:
                y0 = self.origin[1] + ny * resolution
                x0 = self.origin[0] + nx * resolution
                break # Return the closest free world coordinates
        
        radius += 1
    else:
      # Return the free world coordinates
      x0 = self.x0
      y0 = self.y0
      
    # Path planning algortim
    path = solution(x0, y0, self.theta0, self.xt, self.yt, map_data, resolution, self.origin)
    self.path = path # Store path to be able to determine when to replan
    
    # Path message
    message = Path()
    message.header.stamp = time
    message.header.frame_id = '/map'
    
    if path:# If the path exists, publish it
      for i, pose in enumerate(path):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = message.header.stamp
        pose_msg.header.frame_id = message.header.frame_id
        # Set the position 
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        
        message.poses.append(pose_msg)

      self.get_logger().info("Publishing path")
      self.path_pub.publish(message)
    else: 
      # If no path exist, publish empty path
      self.get_logger().info('Publishing empty path')


  # To get the position of the robot at all times
  def pose_callback(self, msg : PoseWithCovarianceStamped):
    # Init transform
    to_frame_rel = 'map'
    from_frame_rel = 'odom'
    time = rclpy.time.Time().from_msg(msg.header.stamp)

    # Wait for the transform asynchronously
    tf_future = self.buffer.wait_for_transform_async(
      target_frame=to_frame_rel,
      source_frame=from_frame_rel,
      time=time
    )
    rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

    # Lookup tansform
    try:
      t = self.buffer.lookup_transform(to_frame_rel,
                                      from_frame_rel,
                                      time)
      # Do the transform
      map_pose = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, t)

      # Save position of robot
      self.x0 = map_pose.position.x
      self.y0 = map_pose.position.y
    except TransformException:
      # self.get_logger().info('No transform found')
      return


  # To get the position of the target when it is published
  def goal_callback(self, msg : Marker):
    # Get position of goal
    self.xt = msg.pose.position.x
    self.yt = msg.pose.position.y
    self.goal_received = True
    self.planned = False

    # Publish zero velocity when planning
    twist_msg = Twist()
    self.cmd_vel_pub.publish(twist_msg)
    self.get_logger().info('Goal recived')

    # Stop planning
    if msg.pose.position.z == -1:
      self.goal_received = False



# Path planning
class Plan_node(object):
  def __init__(self, x, y, theta, parent = None):
    self.x = x
    self.y = y
    self.theta = theta
    self.phi = 0
    self.parent = parent 
    self.g = 0  # Travel cost
    self.h = 0  # Distance cost
    self.f = 0  # Total cost
    self.c = 0  # Cost of cell
    self.feasible = True

def reached_target(x, y, xt, yt, resolution):
  # Check distance to target - maybe add start_center to this?
  if np.sqrt(((x - xt)**2 + (y - yt)**2)) <= resolution/2:
    return True
  return False

def find_cell_index(x, y, resolution, origin):
  # Find grid index from map coordinate
  x_index = int((x - origin[0]) / resolution)
  y_index = int((y - origin[1]) / resolution)
  return (x_index, y_index)

def start_center(x, y, resolution):
  # Adjust the coordinates to the center of the grid cell
  x_center = (x - x % resolution) + (0.5 * resolution)
  y_center = (y - y % resolution) + (0.5 * resolution)
  return x_center, y_center

def step(x, y, theta, phi, resolution):
  # Take a cell step
  dx = resolution * np.cos(theta + phi)
  dy = resolution * np.sin(theta + phi)
  dtheta = phi

  # New cell positions
  xn = x + dx
  yn = y + dy
  thetan = (dtheta + theta) % (2 * np.pi)
  return xn, yn, thetan

def step_collided_with_obsticale(obsticales, x, y, resolution, origin):
  # Check if cell is occupied
  x_index, y_index = find_cell_index(x, y, resolution, origin)
  if obsticales[y_index, x_index] >= 80:
    return True
  return False
    

def get_new_nodes(current_node, open_set, closed_set, steps, xt, yt, obsticales, resolution, origin):
  # For the start node explore all directions then only in front of pose
  steer_angels = [-np.pi/2, 0, np.pi/2, np.pi] if len(closed_set) == 0 else [-np.pi/2, 0, np.pi/2]

  # Calculate adjecent positions for different steering angles
  for phi in steer_angels: 
    xn = current_node.x
    yn = current_node.y
    thetan = current_node.theta
    feasible = True

    # Take steps in the direction of travel
    for i in range(steps):
      xn, yn, thetan = step(xn, yn, thetan, phi, resolution)
            
      if step_collided_with_obsticale(obsticales, xn, yn, resolution, origin):
        feasible = False
        break
    
    # Create new node
    new_node = Plan_node(xn, yn, thetan)
    new_node_key = (round(new_node.x*10000)/10000, round(new_node.y*10000)/10000)
    
    # Check if the point already is visited
    if new_node_key not in closed_set:
      # Check if the point is free or occupied
      if not feasible:
        new_node.feasible = False
        closed_set[new_node_key] = new_node 
      else:
        # Assinge cost functions
        new_node.g = current_node.g + resolution 
        x_index, y_index = find_cell_index(xn, yn, resolution, origin) 
        cost = obsticales[y_index, x_index]
        if cost == -1:
          cost = 0 # - resolution
        new_node.c = cost
        
        # Check if the point is in the open set 
        if new_node_key in open_set:
          node = open_set[new_node_key]
          # Check if the cost is smaller for the new node than the one in open set, then replace it
          if node.g + node.c > new_node.g + new_node.c:
            new_node.h = np.sqrt(((new_node.x - xt)**2 + (new_node.y - yt)**2)) 
            new_node.f = new_node.g + new_node.h * 2 + new_node.c
                
            new_node.parent = current_node
            new_node.phi = phi        
            open_set[new_node_key] = new_node
        else:
          # If the point is free and not visited, add to open set
          new_node.h = np.sqrt(((new_node.x - xt)**2 + (new_node.y - yt)**2))
          new_node.f = new_node.g + new_node.h * 2 + new_node.c
              
          new_node.parent = current_node
          new_node.phi = phi        
          open_set[new_node_key] = new_node


def solution(x0, y0, theta0, xt, yt, obsticales, resolution, origin):
  # Parameters
  steps = 1

  # Ensure grid compatibility, start at a center cell
  x0, y0 = start_center(x0, y0, resolution)
  xt, yt = start_center(xt, yt, resolution)

  # Init the first node
  start_node = Plan_node(x0, y0, theta0) 
  start_node.h = np.sqrt(((start_node.x - xt)**2 + (start_node.y - yt)**2))
  start_node.f = start_node.g + start_node.h * 2
  start_node_key = (round(start_node.x*10000)/10000, round(start_node.y*10000)/10000)
  
  # Innit and preallocate sets
  open_set = dict()
  closed_set = dict()
  
  # Put in the start node in the open set
  open_set[start_node_key] = start_node

  # For avalible points
  while open_set:   
    # Find and take out the lowest cost point
    current_node_key = min(open_set, key=lambda node: open_set[node].f)
    current_node = open_set[current_node_key]
    del open_set[current_node_key]
    
    # Add visited point to closed set to never go there again
    closed_set[current_node_key] = current_node
       
    if reached_target(current_node.x, current_node.y, xt, yt, resolution):
      path=[]
      
      # Take out the path
      while current_node:
          path.append((round(current_node.x*10000)/10000, round(current_node.y*10000)/10000))#, current_node.theta)) # /(resolution*10))*(resolution*10)
          current_node = current_node.parent
      return path[::-1] #, closed_set, open_set

    # Get new nodes
    get_new_nodes(current_node, open_set, closed_set, steps, xt, yt, obsticales, resolution, origin)
  
# Test function
def main2():
  grid = np.zeros((100, 100), dtype=int)
  
  grid[0, :] = 100
  grid[-1, :] = 100
  grid[:, 0] = 100
  grid[:, -1] = 100

  x0 = 0.1 # start x position
  y0 = 0.1 # start y position
  xt = 0.9 # target x position
  yt = 0.3 # target y position

  # Mark the start (x0, y0) and goal (xt, yt) points with green (value 50)
  start_x_index = int(x0 * 100)
  start_y_index = int(y0 * 100)
  goal_x_index = int(xt * 100)
  goal_y_index = int(yt * 100)

  # # Add some custom patterns or corridors
  # for i in range(0, 70):
  #   grid[i, 20:30] = 100  # Add vertical wall
  # for i in range(20, 70):
  #   grid[70:80, i] = 100  # Add horizontal wall

  # # Add random obstacles inside the room
  # num_obstacles = int((100 * 100) * 0.3)
  # obstacle_positions = np.random.choice(100 * 100, num_obstacles, replace=False)

  # for pos in obstacle_positions:
  #     x, y = divmod(pos, 100)
  #     if grid[x, y] == 0 and (x, y) != (start_x_index, start_y_index) and (x, y) != (goal_x_index, goal_y_index):  # Only place obstacles in free space
  #         grid[x, y] = 100

  start_time = time.time()
  path = solution(x0, y0, 0, xt, yt, grid, 1/100, (0,0)) # , closed_set, open_set
  end_time = time.time()
  elapsed_time = end_time - start_time

  if not path:
    print("No possible path")
  else:
    # Plot path
    print('Path found')
    for node in path:
      (x,y) = node #, theta
      x_index = int(x * 100)
      y_index = int(y * 100)
      grid[x_index, y_index] = 5

  # Create a custom colormap with white, red, green, and black
  cmap = ListedColormap(['white', 'red', 'green', 'black'])

  # Adjust the bounds to make sure 5, 50, and 100 are mapped correctly
  bounds = [0, 5, 50, 100]
  norm = BoundaryNorm(bounds, cmap.N)

  # Plot the grid
  plt.imshow(grid, cmap=cmap, norm=norm, interpolation='nearest')

  # Plot start and goal as green points
  plt.scatter(start_y_index, start_x_index, c='green', marker='o', s=50, label=f"Start ({start_x_index}, {start_y_index})")
  plt.scatter(goal_y_index, goal_x_index, c='green', marker='o', s=50, label=f"Goal ({goal_x_index}, {goal_y_index})")
  
  # print('Path is ' + str(len(path)))
  # print('Closed set is ' + str(len(closed_set)))
  # print('Open set is ' + str(len(open_set)))
  # for node_key in closed_set:
  #   node = closed_set[node_key]
  #   closed_x_index = int(node.x * 100)
  #   closed_y_index = int(node.y * 100)
  #   plt.scatter(closed_y_index, closed_x_index, c='red', marker='x', s=10)

  # for node_key in open_set:
  #   node = open_set[node_key]
  #   open_x_index = int(node.x * 100)
  #   open_y_index = int(node.y * 100)
  #   plt.scatter(open_y_index, open_x_index, c='blue', marker='o', s=20)

  plt.gca().invert_yaxis()
  
  # Plot titles
  plt.legend()
  plt.title(f"Path calculation at time: {elapsed_time:.4f} seconds")
  plt.show()

# main2() # need to change x and y index in find_grid_index


def main():
    rclpy.init()
    node = Planner()
    try:
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()