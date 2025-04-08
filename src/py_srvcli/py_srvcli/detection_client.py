import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from detect_interfaces.srv import DetectObjects
from nav_msgs.msg import Odometry
from robp_interfaces.msg import Encoders
from geometry_msgs.msg import Twist
import math

class DetectionManager(Node):
    def __init__(self):
        super().__init__('detection_manager')

        self.angular_velocity = 0.0
        self.linear_velocity = 0.0
        self.detection_active = False  

        # Subscriber a /odom
        #self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Subscribe to encoder
        self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 10)

        # Service client
        self.start_client = self.create_client(SetBool, '/start_detection')
        self.stop_client = self.create_client(SetBool, '/stop_detection')
        self.detect_client = self.create_client(DetectObjects, '/detect_objects')

        # Timer
        self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("Start detection")

    
    def encoder_callback(self, msg: Encoders):
        """Takes encoder readings and updates the odometry.

        This function is called every time the encoders are updated (i.e., when a message is published on the '/motor/encoders' topic).

        Your task is to update the odometry based on the encoder data in 'msg'. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- An encoders ROS message. To see more information about it 
        run 'ros2 interface show robp_interfaces/msg/Encoders' in a terminal.
        """

        # The kinematic parameters for the differential configuration
        dt = 50 / 1000
        ticks_per_rev = 48 * 64
        wheel_radius = 0.04915 # 0.04921
        base = 0.31 # 0.30

        # Ticks since last message
        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        K = 1/ticks_per_rev * 2*math.pi

        v = wheel_radius/2 * (K*delta_ticks_right + K*delta_ticks_left) 
        w = wheel_radius/base * (K*delta_ticks_right - K*delta_ticks_left)

        # Threshold to avoid numerical errors
        rot_threshold = 0.005
        vel_threshold = 1

        if abs(w) > rot_threshold or abs(v) > vel_threshold: 
            if self.detection_active:
                self.get_logger().info("Rotating robot: stop detection")
                self.call_set_bool(self.stop_client, True)
                self.detection_active = False
                self.get_logger().info(f"omega: {w}")
        elif not self.detection_active:
                self.get_logger().info("Robot still or in linear motion: start detection.")
                self.call_set_bool(self.start_client, True)
                self.detection_active = True




    def timer_callback(self):
        # Ogni secondo leggi la lista oggetti rilevati
        if not self.detect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service detect objects not available")
            return

        req = DetectObjects.Request()
        future = self.detect_client.call_async(req)
        future.add_done_callback(self.handle_detect_response)

    c=0
    def handle_detect_response(self, future):
        try:
            res = future.result()
            #fself.get_logger().info(f" Oggetti rilevati: {len(res.object_types)}")
            for i, (obj_type, pos) in enumerate(zip(res.object_types, res.object_positions)):
                self.get_logger().info(f"  #{i+1}: {obj_type} @ ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
                c=0
        except Exception as e:
            self.get_logger().error(f"Error response detect_objects: {e}")

    def call_set_bool(self, client, value: bool):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available")
            return
        req = SetBool.Request()
        req.data = value
        client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
