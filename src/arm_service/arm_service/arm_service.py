from arm_interface.srv import Arm

import os
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import ikpy.chain
import ikpy.utils.plot as plot_utils

import numpy as np
import math

import ipywidgets as widgets
import serial


class MinimalService(Node):

    def __init__(self):
        super().__init__('arm_service')
        self.srv = self.create_service(Arm, 'arm', self.arm_callback)
        self.publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)

    def pos_ok_check(self,target_position):
        # x1_limits = [0.0,0.18] # The limits for two boxes in m
        # x2_limits = [-0.1,0.0]
        # y1_limits = [-0.05,0.15]
        # y2_limits = [-0.15,-0.05]
        # z_limits = [0.0,0.1]
        # x = target_position[0]
        # y = target_position[1]
        # z = target_position[2]
        # if y1_limits[0] <= y <= y1_limits[1]:
        #     if x1_limits[0] <= x <= x1_limits[1]:
        #         if z_limits[0] <= z <= z_limits[1]:
        #             return True
        # elif y2_limits[0] <= y <= y2_limits[1]:
        #     if x2_limits[0] <= x <= x1_limits[1]:
        #         if z_limits[0] <= z <= z_limits[1]:
        #             return True
        # return False
        x, y, z = target_position
        return (
            (np.all([-0.05 <= y <= 0.15, 0.0 <= x <= 0.18]) or 
            np.all([-0.15 <= y <= -0.05, -0.1 <= x <= 0.18]))
            and (0.0 <= z <= 0.2)
        )


    def arm_callback(self, request, response):
        # Load the URDF file
        urdf_file = os.path.join(os.path.dirname(__file__), "armpi_fpv.urdf")
        my_chain = ikpy.chain.Chain.from_urdf_file(urdf_file,active_links_mask=[True, True, True, True, True, True])
        
        target_position = [ 0.00, 0.15, 0.04]
        # if not self.pos_ok_check(target_position):
        #     self.get_logger().info("Position not within limits")
        #     response.success = False # should add something like reason for fail maybe
        #     return response

        target_orientation = [0, 0, 0]

        data_sets = [[12000,12000,12000,12000,12000,12000,2000,2000,2000,2000,2000,2000],
                    [2000,12000,3000,12000,4000,12000,2000,2000,2000,2000,2000,2000],
                    [11000,12000,10000,15000,4000,12000,2000,2000,2000,2000,2000,2000],
                    [2000,12000,8000,16000,10000,12000,2000,2000,2000,2000,2000,2000]]
        test_set = []
        i=0
        for val in data_sets[1]:
            if i<6:
                test_set.append((val - 12000)/100)
            else: 
                break
            i += 1

        ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")
        ik_deg = list(map(lambda r:math.degrees(r),ik.tolist()))
        print("The angles of each joints are : ", ik_deg)
        print("raw ik: "+ str(ik))
        computed_data_set = []
        for ang in ik_deg:
            new_ang = round(ang*100) + 12000
            computed_data_set.append(new_ang)
        time_data_set = [2000,2000,2000,2000,2000,2000]
        computed_data_set = np.concatenate((computed_data_set,time_data_set))
        self.get_logger().info("Computed data set is: " + str(computed_data_set))

        computed_position = my_chain.forward_kinematics(ik)
        computed_position2 = my_chain.forward_kinematics(test_set)

        print("Computed position: %s, original position : %s" % (computed_position[:3, 3], target_position))
        print("Computed position (readable) : %s" % [ '%.2f' % elem for elem in computed_position[:3, 3] ])

        print("Computed top position: %s, original position : %s" % (computed_position2[:3, 3], target_position))
        print("Computed top position (readable) : %s" % [ '%.2f' % elem for elem in computed_position2[:3, 3] ])


        
        msg = Int16MultiArray()
        

        if request.xy[0] == 1: # this is command from client
            self.get_logger().info('moving arm to top')
            msg.data = data_sets[int(request.xy[0]-1)]
            self.publisher.publish(msg)
        elif request.xy[0] == 2:
            self.get_logger().info('moving arm to look')
            msg.data = data_sets[int(request.xy[0]-1)]
            self.publisher.publish(msg)
        elif request.xy[0] == 3: 
            self.get_logger().info('moving arm to grab')
            msg.data = data_sets[int(request.xy[0]-1)]
            self.publisher.publish(msg)
        elif request.xy[0] == 4: 
            self.get_logger().info('moving arm to drop')
            msg.data = data_sets[int(request.xy[0]-1)]
            self.publisher.publish(msg)
        elif request.xy[0] == 5:
            self.get_logger().info('moving arm to set point')
            msg.data = computed_data_set
            self.publisher.publish(msg)
        
        
        #self.get_logger().info('Incoming request\na: %d b: %d' % (request.xy[0]))
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()
    
    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()