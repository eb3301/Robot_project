#!/usr/bin/env python3

import rclpy
import rclpy.logging
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion

class Planner(Node):

  def __init__(self):
    super().__init__('Planner')

    # Subscribe to costmap
    self.costmap_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

    # Subscribe to current pose
    self.pose_sub = self.create_subscribtion(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
    
    # Subscribe to goal pose
    self.goal_sub = self.create_subscription(Marker, "/goal_marker", self.goal_callback, 10)

  def map_callback(self, msg : OccupancyGrid):
    obsticales = msg.data
    resolution = msg.info.resolution
    grid_width = msg.info.width
    grid_height = msg.info.height
    origo_x = msg.info.origin.position.x
    origo_y = msg.info.origin.position.x

  def pose_callback(self, msg : PoseWithCovarianceStamped):
    x0 = msg.pose.pose.position.x
    y0 = msg.pose.pose.position.y
    theta0 = self.compute_heading(msg.pose.pose.orientation)

  def goal_callback(self, msg : Marker):
    xt = msg.pose.position.x
    yt = msg.pose.position.y

  def compute_heading(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        _, _, yaw = euler_from_quaternion((x, y, z, w))
        return yaw


class Node(object):
  def __init__(self, x, y, theta, parent = None):
    self.x = x
    self.y = y
    self.theta = theta
    self.phi = 0
    self.parent = parent 
    self.g = 0
    self.h = 0 
    self.f = 0  

    self.time = 0


def find_obstacles(map_data, grid_size=10):
    obstacles = []
    
    for index, value in enumerate(map_data):
        if value == 100:  # Check if the cell is an obstacle (value 100)
            # Convert index to (x, y) coordinates
            x = index // grid_size
            y = index % grid_size
            obstacles.append((x, y))
    
    return obstacles

def reached_target(x, y, xt, yt):
  if math.sqrt(((x - xt)**2 + (y - yt)**2)) <= 0.5:
    return True
  return False

def step(x, y, theta, phi, dt=0.01):
  # State change rate
  dx = math.cos(theta)
  dy = math.sin(theta)
  dtheta = math.tan(phi)

  # New state, euler
  xn = x + dx*dt
  yn = y + dy*dt
  thetan = theta + dt*dtheta
  return xn, yn, thetan

def step_collided_with_obsticale(obsticales, x, y, marign=0.2):
  for obs in obsticales:
    if math.sqrt((x - obs[0])**2 + (y - obs[1])**2) <= (obs[2] + marign):
      return True
  return False

def grid(node, resolution=0.3):
    # Mapping to right position
    x_key = round(node.x / resolution)
    y_key = round(node.y / resolution)
    
    # Calculate heading
    theta_pos = node.theta % (2 * math.pi)
    theta_key = round(theta_pos / (2 * math.pi / 6))
    return (x_key, y_key, theta_key)
    
def get_new_nodes(current_node, open_set, closed_set, steps, xt, yt, obsticales): #, xlb, xub, ylb, yub):
  # Calculate different steering angles
  for phi in [-math.pi/2, 0, math.pi/2]: # only grid coordinates
    xn = current_node.x
    yn = current_node.y
    thetan = current_node.theta
    feasible = True

    # Take steps in the best direction
    for i in range(steps):
      xn, yn, thetan = step(xn, yn, thetan, phi)
            
      if step_collided_with_obsticale(obsticales, xn, yn):
        feasible = False
        break
        
    new_node = Node(xn, yn, thetan)
    new_node_key = grid(new_node)
        
    if not new_node_key in closed_set:
      if not feasible:
        closed_set[new_node_key] = new_node 
      else:
        new_node.g = current_node.g + math.sqrt((new_node.x - current_node.x)**2 + (new_node.y - current_node.y)**2) # change to Mahalanobis distance
        new_node.h = math.sqrt(((new_node.x - xt)**2 + (new_node.y - yt)**2))
        new_node.f = new_node.g + 2 * new_node.h
        new_node.parent = current_node
        new_node.time = current_node.time + steps * 0.01
        new_node.phi = phi
        open_set[new_node_key] = new_node

# x0 - robot x position
# y0 - robot y position
# xt - target x position
# yt - target y position
# obsticales - list of obsticales [x,y,radius]

def solution(x0, y0, theta0, xt, yt, obsticales): #, xlb, xub, ylb, yub):
  steps = 80
  start_node = Node(x0, y0, theta0) 
  start_node.h = math.sqrt(((start_node.x - xt)**2 + (start_node.y - yt)**2))
  start_node.f = start_node.g + 2 * start_node.h
  start_node_key = grid(start_node)
  
  # Innit and preallocate
  open_set = dict()
  closed_set = dict()
    
  open_set[start_node_key] = start_node

  # For avalible paths
  while open_set:
    current_node_key = min(open_set, key=lambda node: open_set[node].f)
    current_node = open_set[current_node_key]

    del open_set[current_node_key]
       
    closed_set[current_node_key] = current_node
       
    if reached_target(current_node.x, current_node.y, xt, yt):
      path=[] 
      time=[]

      while current_node:
          time.append(current_node.time)
          path.append(current_node.phi)
          current_node = current_node.parent
      path.pop(-1)
      return path[::-1], time[::-1]
  
    get_new_nodes(current_node, open_set, closed_set, steps, xt, yt, obsticales) #, xlb, xub, ylb, yub)


def main():
    rclpy.init()
    print('Hi from Planning.')
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
