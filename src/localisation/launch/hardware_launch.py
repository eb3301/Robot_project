import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Start Rviz (LiDAR also starts rviz so probably not needed....)
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/group1//dd2419_ws/src/default.rviz'],
            output='screen',
            name='rviz'
        ),
        # args="-d $(find-pkg-share robp_launch)/rviz/default.rviz"


        # Start Motor
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robp_launch', 'phidgets_launch.py'],
            output='screen',
            name='motor_launch'
        ),
        
        
        # Start joystick
        ExecuteProcess(
            cmd=['ros2', 'launch', 'teleop_twist_joy', 'teleop-launch.py'],
            output='screen',
            name='joystick_launch'
        ),

        
        # Static transform broadcaster for base_link to imu_link
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '0.00107', '--y', '0', '--z', '0.02361', '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
            output='screen',
            name='base_to_imu'
        ),
        
        # Static transform broadcaster for base_link to camera_link (rs_d435i)
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', '0.08987', '--y', '0.0175', '--z', '0.10456', '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
            output='screen',
            name='base_to_rs_d435i'
        ),
        
        # Static broadcaster
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'odom'],
            output='screen',
            name='static_transform_broadcaster'
        ),
        

        #Lidar Launch
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robp_launch', 'lidar_launch.yaml'],
            output='screen',
            name='lidar_launch'
        ),


    ])
