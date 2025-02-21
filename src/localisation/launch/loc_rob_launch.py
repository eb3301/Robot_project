import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

    # Start controller_node
    ExecuteProcess(
        cmd=['ros2', 'run', 'controller', 'controller'],
        output='screen',
        name='controller_node'
    ),

    #Lidar node
    ExecuteProcess(
        cmd=['ros2', 'run', 'localisation', 'lidar'],
        output='screen',
        name='lidar_node'
    ),

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
    ])

















