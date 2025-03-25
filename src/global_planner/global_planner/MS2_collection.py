#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import os

import py_trees as pt
import py_trees_ros as ptr
from builtin_interfaces.msg import Duration

from ament_index_python import get_package_share_directory

from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped

from tf_transformations import quaternion_from_euler
import numpy as np
from arm_interface.srv import Arm
from arm_service.arm_client import call_arm_client

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
        '''Mapping: Box:     'B'
                    Cube:    '1'
                    Sphere:  '2'
                    Plushie: '3'
                    ICP:    'ICP'    '''
        cube_coor = Get_Coor('ICP', self)


        drive_to_obj_1 = Drive_to_Obj() # Plan and execute path to coordinates
        drive_to_obj_2 = Drive_to_Obj() # Plan and execute path to coordinates
        pickup = Pickup(self) # Pickup object
        place = Place() # Place object


        test_seq = pt.composites.Sequence(name = 'Test Sequence', 
                                          memory = bool,
                                          children = [create_ws, load_map, cube_coor, pickup]
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
                        depth=10  # Stores up to 10 messages in queue
                        )

        self.marker_pub = self.node.create_publisher(Marker, '/object_markers', qos)
        print('Initialised Load Map Behaviour')

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        self.done = True

        package_share_dir = get_package_share_directory('global_planner')
        map_path = os.path.join(package_share_dir, 'data', 'map_1.tsv')

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
            marker.lifetime = Duration(sec=100)  # Keep marker visible for 10 seconds
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

            # Marker Colour
            marker.color.a = 1.0  # Alpha
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0

            # Scale marker size
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            if object[0] == '1': # Cube
                marker.type = marker.CUBE
            elif object[0] == '2': # Sphere
                marker.type = marker.SPHERE
            elif object[0] == '3': # Plushie
                marker.type = marker.CYLINDER
            elif object[0] == 'B': # Box
                marker.type = marker.CUBE
                marker.scale.x = 0.3
                marker.scale.y = 0.15
                marker.scale.z = 0.1
            self.marker_pub.publish(marker)
            print(f'Publishing marker ID: {marker.id} at x={marker.pose.position.x}, y={marker.pose.position.y}')


class Get_Coor(pt.behaviour.Behaviour):
    '''Retrieves coordinates '''
    def __init__(self, object_type: str, node):
        super().__init__("Retrieving Coordinate")
        self.node = node

        self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, '/map_pose', self.pose_callback, 10)

        self.blackboard = pt.blackboard.Blackboard()
        self.target = object_type
        print('Initialising Coordinate Retriever...')

    def update(self):
        # Get list of objects
        #self.blackboard.register_key('objects', access = pt.common.Access.READ_WRITE)
        objects = self.blackboard.get('objects')
        print(objects)
        # First time ticking update
        if self.status == pt.common.Status.INVALID:
            print(f"Retrieving coordinate for {self.target}")
            return pt.common.Status.RUNNING
        
        #Extract boxes and target object
        boxes = [obj for obj in objects if "B" in obj[0]]
        targets = [obj for obj in objects if self.target in obj[0]]

        # If target is ICP reference cloud
        if self.target == 'ICP':
            coor = (6.7, 3.0)
            self.blackboard.set('Target', coor)
            return pt.common.Status.SUCCESS

        # If target is a box: choose closest box
        if self.target == 'B':
            closest_box = min(boxes, key=lambda box: np.linalg.norm(np.array([self.x, self.y]) - np.array([box[1], box[2]])) )
            coor = (closest_box[1], closest_box[2])
            self.blackboard.set('Target', coor)
            return pt.common.Status.SUCCESS

        # Determine what object to pick up
        distances = []
        for target in targets: 
            # Distance between robot and target
            targ_to_robot = np.linalg.norm(np.array([self.x, self.y]) - np.array([target[1], target[2]]))

            # Find box closets to target
            closest_box = min(boxes, key=lambda box: np.linalg.norm(np.array([target[1], target[2]]) - np.array([box[1], box[2]])) )
            
            # Distance from target to closest box
            closest_box_dist = np.linalg.norm(np.array([target[1], target[2]]) - np.array([closest_box[1], closest_box[2]]))

            distances.append(targ_to_robot + closest_box_dist)

        i = np.argmin(distances)
        min_targ = targets[i]
        coor = (min_targ[1], min_targ[2], min_targ)
        
        self.blackboard.set('Target', coor)
        return pt.common.Status.SUCCESS
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        self.x, self.y = pose.position.x, pose.position.y



class Drive_to_Obj(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__('Driving to Object')
        self.target = None
        self.blackboard = pt.blackboard.Blackboard()
        print('Driving behaviour initialised')
        
    def update(self):
        # First time ticking update
        if self.status == pt.common.Status.INVALID:
            print(f"Retrieving coordinates for {self.target}")
            self.target = self.blackboard.get('Target')
            return pt.common.Status.RUNNING
        
        if self.target is None:
            print('No target coordinates available...')
            return pt.common.Status.FAILURE 
        else: 
            return pt.common.Status.SUCCESS
            # TODO: IMPLEMENT DRIVING!
        

class Pickup(pt.behaviour.Behaviour):
    def __init__(self,node):
        super().__init__("Pickup")
        self.node = node
        self.cli = self.node.create_client(Arm, 'arm')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.req = Arm.Request()
        self.blackboard = pt.blackboard.Blackboard()
        self.request_sent = False
        self.response = None
    
    def initialise(self):
        self.request_sent = False
        self.response = None
        self.req.xy[0] = 6 #  
        self.req.obj_class = '1' #self.blackboard.get('pickup_obj_class')[0]
        self.future = self.cli.call_async(self.req)
        return self.future.result()
        rclpy.spin_until_future_complete(self, self.future)
        #return self.future.result()

    def update(self):

        obj_class = '1'#self.blackboard.get('pickup_obj_class')[0]
        if obj_class is None:
            self.node.get_logger().error("No object class specified on blackboard.")
            return pt.common.Status.FAILURE

        # Send pickup request once
        if not self.request_sent:
            try:
                self.req = Arm.Request()
                self.req.xy[0] = 2 # sending a look command
                self.req.obj_class = obj_class
                self.future = self.cli.call_async(self.req)
                rclpy.spin_until_future_complete(self, self.future)
                # req.object_id = target  # or whatever field your service requires
                # self.response = self.client.call(req)
                self.request_sent = True
            except: #rclpy.ServiceException as e:
                self.node.get_logger().error(f"Service call failed:")# {e}")
                return pt.common.Status.FAILURE

        # Process the response
        if self.response.success:
            self.node.get_logger().info("Pickup successful.")
            return pt.common.Status.SUCCESS
        else:
            self.node.get_logger().error("Pickup failed, " + self.response.message)
            return pt.common.Status.FAILURE
        pass

    
    
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