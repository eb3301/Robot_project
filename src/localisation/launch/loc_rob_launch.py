import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

    # #Start Rviz
    # ExecuteProcess(
    #     cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/group1/rviz.rviz'],
    #     output='screen',
    #     name='rviz'
    # ),

    #EKF Node
    ExecuteProcess(
        cmd=['ros2', 'run', 'localisation', 'EKF'],
        output='screen',
        name='EKF_Node'
    ),

    #ICP Node
    ExecuteProcess(
        cmd=['ros2', 'run', 'localisation', 'ICP'],
        output='screen',
        name='ICP_Node'
    ),

    #Static broadcaster: LiDAR - base_link
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher','--x', '0.0', '--y', '0.815', '--z', '0.12', '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
            output='screen',
            name='base_to_lidar'
        ),
    ])

















