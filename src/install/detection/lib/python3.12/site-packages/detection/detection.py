#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import ctypes
import struct


class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        self.broadcaster = TransformBroadcaster(self)
        self.buffer = Buffer()

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)

        # Subscribe to point cloud topic and call callback function on each received message
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)

    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
    """
        

        # Convert ROS -> NumPy

        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        colors = np.empty(points.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        colors = colors.astype(np.float32) / 255

        target_red = np.array([0.57, 0.04, 0])
        target_green = np.array([0, 0.23, 0.13])
        color_threshold = 0.1

        objects = []
        color = []
        for i, point in enumerate(points):
            # if point[2] > -0.115:
            distance = math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)

            if distance < 1:
                # red
                if np.linalg.norm(colors[i]-target_red) < color_threshold:
                    self.get_logger().info('Detected red cube!')
                    objects.append(point)
                    color.append(target_red)

                # green
                if np.linalg.norm(colors[i]-target_green) < color_threshold:
                    self.get_logger().info('Detected green cube!')
                    objects.append(point)
                    color.append(target_green)
        
        message = PointCloud2()
        message.header = msg.header
        message.height = 1 # unordered
        message.width = len(objects)
        #print(msg.fields)
        # x_field = PointField()
        # x_field.name = 'x'
        # x_field.point_step = 0
        # x_field.row_step = PointField.FLOAT32
        # x_field.data = 1
        # fields = [
        #     PointField(name='x', 0, PointField.FLOAT32, 1),
        #     PointField('y', 4, PointField.FLOAT32, 1),
        #     PointField('z', 8, PointField.FLOAT32, 1),
        #     PointField('r', 12, PointField.UINT8, 1),
        #     PointField('g', 13, PointField.UINT8, 1),
        #     PointField('b', 14, PointField.UINT8, 1),
        #     PointField('intensity', 15, PointField.FLOAT32, 1)
        # ]


        message.fields = msg.fields # PointField msg

        message.is_bigendian = False
        message.point_step = 16
        message.row_step = message.width * message.point_step
        message.is_dense = True

        message_data = []
        for i, obj in enumerate(objects):
            # Append (x, y, z)
            message_data.append(struct.pack('fff', *obj))

            red = int(color[i][0] * 255)  # Convert back to 0-255 range
            green = int(color[i][1] * 255)  # Convert back to 0-255 range
            blue = int(color[i][2] * 255)  # Convert back to 0-255 range

            # message_data.append(struct.pack('B', red))  # Red channel
            # message_data.append(struct.pack('B', green))  # Green channel
            # message_data.append(struct.pack('B', blue))  # Blue channel

            packed_color = (red << 24) | (green << 16) | (blue << 8) | 255
            intensity = float(sum(color[i]))  # Just an example; you can use another formula for intensity
            message_data.append(struct.pack('f', intensity))
        
        message.data = b''.join(message_data)

        self._pub.publish(message)
                


def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()