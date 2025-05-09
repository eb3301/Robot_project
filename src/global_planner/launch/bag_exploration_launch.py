import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

    # Start Rviz
    ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/robot/dd2419_ws/src/global_planner/launch/rviz/exploration_bag.rviz'],
        output='screen',
        name='rviz'
    ),

    # EKF Node
    ExecuteProcess(
        cmd=['ros2', 'run', 'localisation', 'EKF'],
        output='screen',
        name='EKF_Node'
    ),

    # ICP Node
    ExecuteProcess(
        cmd=['ros2', 'run', 'localisation', 'ICP'],
        output='screen',
        name='ICP_Node'
    ),

    # Occupancy grid node
    ExecuteProcess(
        cmd=['ros2', 'run', 'occupancy_grid', 'occupancy_grid'],
        output='screen',
        name='Occupancy_Grid'
    ),

    # Mapping node
    ExecuteProcess(
        cmd=['ros2', 'run', 'mapping', 'mapping'],
        output='screen',
        name='Mapping_Node'
    ),


    # Detection Service
    ExecuteProcess(
        cmd=['ros2', 'run', 'py_srvcli', 'detection_service'],
        output='screen',
        name='Detection_Service'
    ) 


    ])

















