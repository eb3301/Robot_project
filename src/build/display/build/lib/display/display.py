#!/usr/bin/env python

import rclpy
import math
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
import tf_transformations

from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped


class DisplayMarkers(Node):

    def __init__(self):
        super().__init__('display_markers')

        # Initialize the transform listener and assign it a buffer
        self.buffer = Buffer()
        self.tf_listner = TransformListener(self.buffer, self, spin_thread=True)

        # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # Subscribe to aruco marker topic and call callback function on each received message
        self.subscription = self.create_subscription(MarkerArray, '/aruco/markers', self.aruco_callback, 10)

    def aruco_callback(self, msg: MarkerArray):
        # msg.header.frame_id
        for marker in msg.markers:
            to_frame_rel = 'map'
            from_frame_rel = 'camera_depth_optical_frame'
            time = rclpy.time.Time().from_msg(msg.header.stamp) # time.time(seconds=0)

            # Wait for the transform asynchronously
            tf_future = self.buffer.wait_for_transform_async(
                target_frame=to_frame_rel,
                source_frame=from_frame_rel,
                time=time
            )

            rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1)

            try:
                t = self.buffer.lookup_transform(to_frame_rel,
                            from_frame_rel,
                            time)
            except TransformException:
                self.get_logger().info('No transform')
            
            
            try:
                transformed_point = tf2_geometry_msgs.do_transform_pose(marker.pose.pose, t)
            
                message = TransformStamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = 'map'
                message.child_frame_id = f'/aruco/detected{marker.id}'
                
                if marker.id == 1:
                    rotation1 = tf_transformations.quaternion_from_euler(0, 0, -(math.pi / 2))
                    rotation2 = tf_transformations.quaternion_from_euler(-(math.pi / 2), 0, 0)
                elif marker.id == 2:
                    rotation1 = tf_transformations.quaternion_from_euler(0, 0, -(math.pi / 2))
                    rotation2 = tf_transformations.quaternion_from_euler((math.pi / 2), 0, 0)

                message.transform.translation.x = transformed_point.position.x
                message.transform.translation.y = transformed_point.position.y
                message.transform.translation.z = transformed_point.position.z

                orientation_as_list = [transformed_point.orientation.x, 
                        transformed_point.orientation.y, 
                        transformed_point.orientation.z, 
                        transformed_point.orientation.w]

                rotation_quaternion = tf_transformations.quaternion_multiply(rotation2, rotation1)
                q = tf_transformations.quaternion_multiply(rotation_quaternion, orientation_as_list)
                
                message.transform.rotation.x = q[0]
                message.transform.rotation.y = q[1]
                message.transform.rotation.z = q[2]
                message.transform.rotation.w = q[3]
            
                self.broadcaster.sendTransform(message)
            except Exception:
                self.get_logger().info('No transfrom')
            

            
            # Broadcast/publish the transform between the map frame and the detected aruco marker



def main():
    rclpy.init()
    node = DisplayMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()