import sys # used to get input

from arm_interface.srv import Arm
import rclpy
from rclpy.node import Node


class ArmClient(Node):

    def __init__(self):
        super().__init__('arm_client')
        self.cli = self.create_client(Arm, 'arm')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Arm.Request()

    def send_request(self, command):
        self.req.xy[0] = command
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def call_arm_client(command):
    arm_client = ArmClient()

    success = arm_client.send_request(command)

    return success


def main(args=None):
    rclpy.init(args=args)

    arm_client = ArmClient()
    response = arm_client.send_request(int(sys.argv[1])) # sys.argv is from terminal
    arm_client.get_logger().info(
        'Response from arm is: ' + str(response.success) + " and " + response.message)

    arm_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()