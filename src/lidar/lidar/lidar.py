import rclpy
import rclpy.duration
from rclpy.node import Node

import rclpy.time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

from tf2_geometry_msgs import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class Timestamp(Node):

    def __init__(self):

        print("LE----------------------------------------------------------------------------------------------------------------------------")
        super().__init__('timestamp')

        self.publisher = self.create_publisher(PointCloud2, '/lidar', 10)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning

        # Initialize the transform buffer
        self.tf_buffer = Buffer()

        # Initialize the transform listener
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.broadcaster = TransformBroadcaster(self)

        self.proj = LaserProjection()

    def listener_callback(self, msg):

        to_frame_rel = 'odom'
        from_frame_rel = msg.header.frame_id

        #Create a transform between LiDAR to base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = from_frame_rel

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.10
        transform.transform.translation.z = 0.10    

        self.broadcaster.sendTransform(transform)

        time = rclpy.time.Time().from_msg(msg.header.stamp)

        # Wait for the transform asynchronously
        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        # Spin until transform found or `timeout_sec` seconds has passed
        rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
                # timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        # Project lidar to point cloud
        cloud = self.proj.projectLaser(msg)

        # Transform point cloud
        cloud_out = do_transform_cloud(cloud, t)

        self.publisher.publish(cloud_out)


def main():
    rclpy.init()
    node = Timestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()

