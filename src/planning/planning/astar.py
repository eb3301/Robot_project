#!/usr/bin/env python3

import rclpy
import rclpy.logging
import math
import time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class Planner(Node):

  def __init__(self):
    super().__init__('Initialize_Planner')

    # Subscribe to costmap
    self.costmap_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

    # Subscribe to current pose
    self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
    
    # Subscribe to goal pose
    self.goal_sub = self.create_subscription(Marker, "/goal_marker", self.goal_callback, 10)

    # Publish the planned path
    self.path_pub = self.create_publisher(Path, "/planned_path", 10)

    # Prealocation
    self.obsticales = []
    self.resolution = 0

    # Robot pose coordnates
    self.x0 = 0.25
    self.y0 = 0.25
    self.theta0 = 0

    # Target coordinates
    self.xt = 0.75
    self.yt = 0.75

  def map_callback(self, msg : OccupancyGrid):
    map_data = msg.data
    self.resolution = msg.info.resolution
    grid_size = msg.info.height
    # origo_x = msg.info.origin.position.x
    # origo_y = msg.info.origin.position.x

    self.obsticales = self.find_obstacles(map_data, grid_size, self.resolution)

    path, time = solution(self.x0, self.y0, self.theta0, self.xt, self.yt, self.obsticales)
    print('Path found')
    message = Path()
    message.header.stamp = self.get_clock().now().to_msg()
    message.header.frame_id = 'map'
    print(path[2][-1])
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

    print('Publish path')
    self.path_pub.publish(message)

  def pose_callback(self, msg : PoseWithCovarianceStamped):
    self.x0 = msg.pose.pose.position.x
    self.y0 = msg.pose.pose.position.y
    self.theta0 = self.compute_heading(msg.pose.pose.orientation)

  def goal_callback(self, msg : Marker):
    self.xt = msg.pose.position.x
    self.yt = msg.pose.position.y

  def compute_heading(self, orientation):
        x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
        _, _, yaw = euler_from_quaternion((x, y, z, w))
        return yaw
  
  def find_obstacles(self, map_data, grid_size, resolution):
    obstacles = []
    
    for index, value in enumerate(map_data):
        if value == 100:  # Check if the cell is an obstacle (value 100)
            # Convert index to (x, y) coordinates
            x = index // grid_size
            y = index % grid_size
            radius = resolution
            obstacles.append((x, y, radius))
    
    return obstacles



class Node(object):
  def __init__(self, x, y, theta, parent = None):
    self.x = x
    self.y = y
    self.theta = theta
    self.phi = 0
    self.parent = parent 
    self.g = 0  # Travel cost
    self.h = 0  # Distance cost
    self.f = 0  # Total cost

    self.time = 0


def reached_target(x, y, xt, yt):
  if math.sqrt(((x - xt)**2 + (y - yt)**2)) <= 0.5: # change distance to target
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


def step_collided_with_obsticale(obsticales, x, y, marign=0.1):
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
    

def get_new_nodes(current_node, open_set, closed_set, steps, xt, yt, obsticales):
  # Calculate different steering angles
  print('New paths')
  for phi in [-math.pi/4, 0, math.pi/4]: # only grid coordinates
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


def solution(x0, y0, theta0, xt, yt, obsticales):
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
    print('In loop')
    current_node_key = min(open_set, key=lambda node: open_set[node].f)
    current_node = open_set[current_node_key]

    del open_set[current_node_key]
       
    closed_set[current_node_key] = current_node
       
    if reached_target(current_node.x, current_node.y, xt, yt):
      path=[]
      time=[]

      while current_node:
          time.append(current_node.time)
          path.append((current_node.x, current_node.y, current_node.phi))
          current_node = current_node.parent
      path.pop(-1)
      return path[::-1], time[::-1]
  
    get_new_nodes(current_node, open_set, closed_set, steps, xt, yt, obsticales)




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
