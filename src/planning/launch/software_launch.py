import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Start Rviz
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/group1//.rviz2/default.rviz'],
            output='screen',
            name='rviz'
        ),
        
        # # Start controller joystick 
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'controller', 'controller'],
        #     output='screen',
        #     name='controller_node'
        # ),

        # # Start auto control
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'steering', 'auto_control'],
        #     output='screen',
        #     name='auto_control_node'
        # ),
        
        # Start costmap
        ExecuteProcess(
            cmd=['ros2', 'run', 'occupancy_grid', 'occupancy_grid'],
            output='screen',
            name='occupancy_grid_node'
        ),

        # Start wheel control
        ExecuteProcess(
            cmd=['ros2', 'run', 'steering', 'wheel_control'],
            output='screen',
            name='wheel_control_node'
        ),

        # Start path planner
        ExecuteProcess(
            cmd=['ros2', 'run', 'planning', 'astar'],
            output='screen',
            name='path_plan_node'
        ),

    ])