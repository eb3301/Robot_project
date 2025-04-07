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

from std_srvs.srv import SetBool
from detect_interfaces.srv import DetectObjects
import time

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


        drive_to_obj_1 = Drive_to_Obj(self) # Plan and execute path to coordinates ### this to obj
        drive_to_obj_2 = Drive_to_Obj(self) # Plan and execute path to coordinates ### this one to box?
        pickup = Pickup(self) # Pickup object
        place = Place() # Place object
        detection=Detection()

        drive_parallell = pt.composites.Parallel(name='Drive parallell',
                                                 policy=pt.common.ParallelPolicy.SuccessOnSelected(drive_to_obj_1),
                                                 children=[collect_seq]
                                                 )
        collect_seq = pt.composites.Sequence(name = 'Collection Sequence', 
                                          memory = bool,
                                          children = [cube_coor, drive_to_obj_1, pickup, drive_to_obj_2, place]
                                          )
        
        test_seq = pt.composites.Sequence(name = 'Test Sequence', 
                                          memory = bool,
                                          children = [create_ws, load_map, drive_parallell, pickup]
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
            self.blackboard.set('Target_type', "ICP")
            return pt.common.Status.SUCCESS

        # If target is a box: choose closest box
        if self.target == 'B':
            closest_box = min(boxes, key=lambda box: np.linalg.norm(np.array([self.x, self.y]) - np.array([box[1], box[2]])) )
            coor = (closest_box[1], closest_box[2])
            self.blackboard.set('Target', coor)
            self.blackboard.set('Target_type', "B")
            return pt.common.Status.SUCCESS
        
        # Determine what object to pick up
        distances = []
        for target in targets: 
            # Distance between robot and target
            targ_to_robot = np.linalg.norm(np.array([self.x, self.y]) - np.array([target[1], target[2]]))

            # Find closest box to target
            closest_box = min(boxes, key=lambda box: np.linalg.norm(np.array([target[1], target[2]]) - np.array([box[1], box[2]])) )
            
            # Distance from target to closest box
            closest_box_dist = np.linalg.norm(np.array([target[1], target[2]]) - np.array([closest_box[1], closest_box[2]]))

            distances.append(targ_to_robot + closest_box_dist)

        i = np.argmin(distances)
        min_targ = targets[i]
        coor = (min_targ[1], min_targ[2], min_targ)
        
        self.blackboard.set('Target', coor)
        self.blackboard.set('Target_type', min_targ[0])

        return pt.common.Status.SUCCESS
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        self.x, self.y = pose.position.x, pose.position.y

class Drive_to_Obj(pt.behaviour.Behaviour):
    def __init__(self,node):
        super().__init__('Driving to Object')
        self.node = node

        self.target = None
        self.blackboard = pt.blackboard.Blackboard()

        self.target = None
        self.detection_active = False

        print('Driving behaviour initialised')

        
    def update(self):
        # First time ticking update
        if self.status == pt.common.Status.INVALID:
            print(f"Retrieving coordinates for {self.target}")
            self.target = self.blackboard.get('Target')
            self.objects = self.blackboard.get('objects')
            
            # Initialise detection
            self.start_client = self.node.create_client(SetBool, '/start_detection')
            self.stop_client = self.node.create_client(SetBool, '/stop_detection')
            self.detect_client = self.node.create_client(DetectObjects, '/detect_objects')

            if self.target is None:
                print('No target coordinates available...')
                return pt.common.Status.FAILURE 
            else: 
                print('Starting detection client')
                self.call_set_bool(self.start_client, True)
                self.detection_active = True

                # TODO: IMPLEMENT DRIVING!
                # Path definition
                # Motor commands
                
                # Detection for loop
                if not self.detect_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn("Service detect objects not available")
                    return pt.common.Status.FAILURE
                        
                self.detect_req = DetectObjects.Request()
                self.future = self.detect_client.call_async(self.detect_req)
                return pt.common.Status.RUNNING
        
        if hasattr(self, 'future') and self.future.done():
                res = self.future.result()
                if res is None:
                    self.get_logger().error("Service call returned None")
                    return pt.common.Status.FAILURE
                try:
                    for i, (obj_type, pos) in enumerate(zip(res.object_types, res.object_positions)):
                        for obj in self.objects:
                            detected_pos = np.array([pos.x, pos.y])
                            known_pos = np.array([obj[1], obj[2]])
                            if np.linalg.norm(detected_pos - known_pos) > 0.15:
                                # if a new object is found and it is far from all the other known object --> re-plan the trajectory
                                self.node.get_logger().info("New object found - Re-planning needed") 
                                return pt.common.Status.FAILURE
                    return pt.common.Status.SUCCESS
                except Exception as e:
                    self.get_logger().error(f"Error response detect_objects: {e}")
                    return pt.common.Status.FAILURE
                
        return pt.common.Status.RUNNING
                
        

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

    def update(self):
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