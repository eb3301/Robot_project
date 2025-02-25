import sys

from arm_interface.srv import Arm
import rclpy
from rclpy.node import Node
from arm_service.arm_client import call_arm_client

def main(args=None):
    rclpy.init(args=args)
    response = call_arm_client(int(sys.argv[1])) # sys.argv is from terminal
    
    print(str(response))
    rclpy.shutdown()


if __name__ == '__main__':
    main()