import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

    #Start Rviz
    ExecuteProcess(
        cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/robot/dd2419_ws/src/localisation/launch/rviz/loc.rviz'],
        output='screen',
        name='rviz'
    ),

    # Start controller_node
    ExecuteProcess(
        cmd=['ros2', 'run', 'controller', 'controller'],
        output='screen',
        name='controller_node'
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

        # Path Execution Node
    ExecuteProcess(
        cmd=['ros2', 'run', 'steering', 'wheel_control'],
        output='screen',
        name='Wheel_Controller_Node'
    )
    ])

















