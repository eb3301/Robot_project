import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Start Rviz
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/robot//dd2419_ws/src/default.rviz'],
            output='screen',
            name='rviz'
        ),
        
        # Start controller joystick 
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller', 'controller'],
            output='screen',
            name='controller_node'
        ),

        # # Start auto control
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'steering', 'auto_control'],
        #     output='screen',
        #     name='auto_control_node'
        # ),
        
        # # Start odometry
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'odometry', 'odometry'],
        #     output='screen',
        #     name='odometry_node'
        # ),

        # Start wheel control
        ExecuteProcess(
            cmd=['ros2', 'run', 'steering', 'wheel_control'],
            output='screen',
            name='wheel_control_node'
        ),

        # # Start camera
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'detection', 'detection'],
        #     output='screen',
        #     name='camera_node'
        # ),
       
        # # Start arm
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/hiwonder_arm', '-v6'],
        #     output='screen',
        #     name='arm_launch'
        # ),
        
        # Arm controller
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'pickup', 'pickup'],
        #     output='screen',
        #     name='arm_controller'
        # ),

        # #Lidar node
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'lidar', 'lidar'],
        #     output='screen',
        #     name='lidar_node'
        # ),

    ])