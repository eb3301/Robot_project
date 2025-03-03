#!/usr/bin/env python3

import rclpy
import rclpy.logging
import numpy as np
import time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

class Planner(Node):

  def __init__(self):
    super().__init__('Initialize_Planner')

    # Subscribe to costmap
    self.costmap_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)

    # Subscribe to current pose
    self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/ekf_pose', self.pose_callback, 10) # This might need to be restricted
    
    # Subscribe to goal pose
    self.goal_sub = self.create_subscription(Marker, "/goal_marker", self.goal_callback, 1)

    # Publish the planned path
    self.path_pub = self.create_publisher(Path, "/planned_path", 1)

    # Robot pose coordnates
    self.x0 = 0.1
    self.y0 = 0.1
    self.theta0 = 0

    # Target coordinates
    self.xt = 0.9
    self.yt = 0.9

  def map_callback(self, msg : OccupancyGrid):
    # Get data from message
    map_data = msg.data
    map_data = np.reshape(map_data, (msg.info.height, msg.info.width))
    resolution = msg.info.resolution

    # Path planning algortim
    path = solution(self.x0, self.y0, self.theta0, self.xt, self.yt, map_data, resolution)
    
    # Path message
    message = Path()
    message.header.stamp = self.get_clock().now().to_msg()
    message.header.frame_id = 'odom'
    
    if path:# If the path exists, publish it
      for i, pose in enumerate(path):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = message.header.stamp
        pose_msg.header.frame_id = message.header.frame_id
        # Set the position and orientation (phi in your case)
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        # Convert the orientation from phi (heading) to a quaternion
        quaternion = quaternion_from_euler(0, 0, pose[2])  # Use phi as the yaw angle
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        message.poses.append(pose_msg)
    # else: # If no path exist, publish empty path
    #   pose_msg = PoseStamped()
    #   pose_msg.header.stamp = message.header.stamp
    #   pose_msg.header.frame_id = message.header.frame_id
    #   message.poses.append(pose_msg)

    print('Publish path')
    self.path_pub.publish(message)

  def pose_callback(self, msg : PoseWithCovarianceStamped):
    # Get posion of robot
    self.x0 = msg.pose.pose.position.x
    self.y0 = msg.pose.pose.position.y
    # self.theta0 = self.compute_heading(msg.pose.pose.orientation)
    # thetao = 0 Do it need to be zero to work?

  def goal_callback(self, msg : Marker):
    # Get position of goal
    self.xt = msg.pose.position.x
    self.yt = msg.pose.position.y

  # def compute_heading(self, orientation):
  #   # Compute origentation of robot
  #   x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
  #   _, _, yaw = euler_from_quaternion((x, y, z, w))
  #   return yaw
  

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
    self.c = 0  # Cost of kernel
    self.feasible = True


def reached_target(x, y, xt, yt, resolution):
  if np.sqrt(((x - xt)**2 + (y - yt)**2)) <= resolution/2: # change distance to target np.abs(x - xt) < 0.01 and np.abs(y - yt) < 0.01:
    return True
  return False

def find_cell_index(x, y, resolution):
  # Convert to integers to avoid floating-point precision issues
  x_index = int(x // resolution)  # Integer division to get the grid index
  y_index = int(y // resolution)  # Integer division to get the grid index
  
  # # Calculate the 1D index assuming a 2D grid
  # index = y_index * int(1 / resolution) + x_index
  return (x_index, y_index)

def start_center(x, y, resolution):
  x_index, y_index = find_cell_index(x, y, resolution)
  
  # Adjust the coordinates to the center of the grid cell
  x_center = (x_index + 0.5) * resolution
  y_center = (y_index + 0.5) * resolution
  return x_center, y_center


def step(x, y, theta, phi, resolution):
  # Take a one cell step
  dx = resolution * np.cos(theta + phi)
  dy = resolution * np.sin(theta + phi)
  dtheta = phi

  # New cell positions
  xn = x + dx
  yn = y + dy
  thetan = (dtheta + theta) % (2 * np.pi)
  return xn, yn, thetan


def step_collided_with_obsticale(obsticales, x, y, resolution):
  x_index, y_index = find_cell_index(x, y, resolution) # Maybe need to be center?
  if obsticales[x_index, y_index] == 100:
    return True
  return False
    

def get_new_nodes(current_node, open_set, closed_set, steps, xt, yt, obsticales, resolution, directions):
  # Get steering angles, only grid coordinates
  if directions == 4:
    steer_angels = [-np.pi/2, 0, np.pi/2, np.pi]
  else:
    steer_angels = [-np.pi/2, 0, np.pi/2]

  # Calculate different steering angles
  for phi in steer_angels: 
    xn = current_node.x
    yn = current_node.y
    thetan = current_node.theta
    feasible = True

    # Take steps in the direction of travel
    for i in range(steps):
      xn, yn, thetan = step(xn, yn, thetan, phi, resolution)
            
      if step_collided_with_obsticale(obsticales, xn, yn, resolution): # chatgpt fail
        feasible = False
        break
    
    # Calculate new node
    new_node = Plan_node(xn, yn, thetan)
    new_node_key = (new_node.x, new_node.y)
        
    if new_node_key not in closed_set:
      if not feasible:
        # If node in obsticle
        new_node.feasible = False
        closed_set[new_node_key] = new_node 
      else:
        # Cost functions
        new_node.g = current_node.g + resolution #+ math.sqrt((new_node.x - current_node.x)**2 + (new_node.y - current_node.y)**2) # change to Mahalanobis distance
        new_node.h = np.sqrt(((new_node.x - xt)**2 + (new_node.y - yt)**2)) # np.abs(new_node.x - xt) + np.abs(new_node.y - yt)
        x_index, y_index = find_cell_index(xn, yn, resolution) 
        new_node.c = obsticales[x_index, y_index]
        new_node.f = new_node.g + new_node.h * 2 + new_node.c
        
        new_node.parent = current_node
        new_node.phi = phi
        open_set[new_node_key] = new_node
    # else:
    #   if closed_set[new_node_key].feasible:
    #     pass # Look if g is higher?


def solution(x0, y0, theta0, xt, yt, obsticales, resolution):
  steps = 1
  start = 0

  # Ensure grid compatibility
  x0, y0 = start_center(x0, y0, resolution)
  xt, yt = start_center(xt, yt, resolution)

  # Start node
  start_node = Plan_node(x0, y0, theta0) 
  start_node.h = np.sqrt(((start_node.x - xt)**2 + (start_node.y - yt)**2)) # np.abs(start_node.x - xt) + np.abs(start_node.y - yt) #
  start_node.f = start_node.g + start_node.h * 2
  start_node_key = (start_node.x, start_node.y)
  
  # Innit and preallocate lists
  open_set = dict()
  closed_set = dict()
  
  # Put in the start node in the open list
  open_set[start_node_key] = start_node

  # For avalible paths
  while open_set:
    # Find and take out the lowest cost node
    current_node_key = min(open_set, key=lambda node: open_set[node].f)
    current_node = open_set[current_node_key]
    del open_set[current_node_key]
    
    # Add visited nodes
    closed_set[current_node_key] = current_node
       
    if reached_target(current_node.x, current_node.y, xt, yt, resolution):
      path=[]
      
      # Take out hte path
      while current_node:
          path.append((current_node.x, current_node.y, current_node.theta))
          current_node = current_node.parent
      path.pop(-1) # The robots position, should be included?
      return path[::-1]

    # For the start, explore all directions
    if start == 0:
      directions = 4
      start = 1
    else:
      directions = 3

    # Get new nodes
    get_new_nodes(current_node, open_set, closed_set, steps, xt, yt, obsticales, resolution, directions)


def main2():
  grid = np.zeros((100, 100), dtype=int)
  
  grid[0, :] = 100
  grid[-1, :] = 100
  grid[:, 0] = 100
  grid[:, -1] = 100
  # grid = grid.flatten().tolist()

  x0 = 0.1 # start x position
  y0 = 0.1 # start y position
  xt = 0.9 # target x position
  yt = 0.9 # target y position

  # Mark the start (x0, y0) and goal (xt, yt) points with green (value 50)
  start_x_index = int(x0 * 100)
  start_y_index = int(y0 * 100)
  goal_x_index = int(xt * 100)
  goal_y_index = int(yt * 100)

  # Add some custom patterns or corridors
  # for i in range(0, 60):
  #   grid[i, 20:30] = 100  # Add vertical wall
  # for i in range(20, 50):
  #   grid[70:80, i] = 100  # Add horizontal wall

  # # Add random obstacles inside the room
  # num_obstacles = int((100 * 100) * 0.3)
  # obstacle_positions = np.random.choice(100 * 100, num_obstacles, replace=False)

  # for pos in obstacle_positions:
  #     x, y = divmod(pos, 100)
  #     if grid[x, y] == 0 and (x, y) != (start_x_index, start_y_index) and (x, y) != (goal_x_index, goal_y_index):  # Only place obstacles in free space
  #         grid[x, y] = 100

  start_time = time.time()
  path = solution(x0, y0, 0, xt, yt, grid, 1/100)
  end_time = time.time()
  elapsed_time = end_time - start_time

  if not path:
    print("No possible path")
  else:
    # Plot path
    print('Path found')
    for node in path:
      (x,y, theta) = node
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

  plt.gca().invert_yaxis()
  
  # Plot titles
  plt.legend()
  plt.title(f"Path calculation at time: {elapsed_time:.4f} seconds")
  plt.show()

 

# main2()


def main():
    rclpy.init()
    node = Planner()
    try:
        rclpy.spin(node)
        time.sleep(3)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
