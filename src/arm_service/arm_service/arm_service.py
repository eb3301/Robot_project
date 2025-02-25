from arm_interface.srv import Arm

import rclpy
from rclpy.node import Node

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import time

# class MinimalPublisher(Node):

#     def __init__(self):
#         super().__init__('move_servos_publisher')
#         self.publisher_ = self.create_publisher(Int64MultiArray, 'multi_servo_cmd_sub', 10)
#         timer_period = 2  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.i = 0

#     def timer_callback(self):
#         # pick up stuff
#         # data_sets = [[2000,12000,20000,3000,-1,3000,500,500,500,500,500,500],
#         #             [16000,-1,-1,-1,-1,-1,500,500,500,500,500,500],
#         #             [16000,12000,12000,12000,12000,12000,500,500,500,500,500,500],
#         #             [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]]
        
#         # random movement
#         data_sets = [[7000,13000,14000,10000,16000,8000,2000,2000,2000,2000,2000,2000],
#                     [12000,12000,12000,12000,12000,12000,2000,2000,2000,2000,2000,2000]]
        
#         msg = Int16MultiArray()
#         msg.data = data_sets[self.i]
#         self.publisher_.publish(msg)

#         self.i += 1
#         if self.i == 2:
#             self.i = 0

        
#         # self.get_logger().info('Publishing: "%s"' % str(msg.data))
#         # self.i += 1


class MinimalService(Node):

    def __init__(self):
        super().__init__('arm_service')
        self.srv = self.create_service(Arm, 'arm', self.arm_callback)
        self.publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)

    def arm_callback(self, request, response):
        data_sets = [[12000,12000,12000,12000,12000,12000,2000,2000,2000,2000,2000,2000],
                    [2000,12000,3000,12000,4000,12000,2000,2000,2000,2000,2000,2000],
                    [11000,12000,10000,15000,4000,12000,2000,2000,2000,2000,2000,2000],
                    [2000,12000,8000,16000,10000,12000,2000,2000,2000,2000,2000,2000]]
        msg = Int16MultiArray()
        msg.data = data_sets[int(request.xy[0]-1)]

        if request.xy[0] == 1: # this is command from client
            self.get_logger().info('moving arm to top')
            self.publisher.publish(msg)
        elif request.xy[0] == 2:
            self.get_logger().info('moving arm to look')
            self.publisher.publish(msg)
        elif request.xy[0] == 3: 
            self.get_logger().info('moving arm to grab')
            self.publisher.publish(msg)
        elif request.xy[0] == 4: 
            self.get_logger().info('moving arm to drop')
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