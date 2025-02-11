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
        
        # Start controller_node
        ExecuteProcess(
            cmd=['ros2', 'run', 'controller', 'controller'],
            output='screen',
            name='controller_node'
        ),
        
        # Start joystick
        ExecuteProcess(
            cmd=['ros2', 'launch', 'teleop_twist_joy', 'teleop-launch.py'],
            output='screen',
            name='joystick_launch'
        ),
        
        # Start odometry
        ExecuteProcess(
            cmd=['ros2', 'run', 'odometry', 'odometry'],
            output='screen',
            name='odometry_node'
        ),

        #Start camera
        ExecuteProcess(
            cmd=['ros2', 'run', 'detection', 'detection'],
            output='screen',
            name='camera_node'
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
        
        # Start arm
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/hiwonder_arm', '-v6'],
            output='screen',
            name='arm_launch'
        ),
        
        # Arm controller
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'pickup', 'pickup'],
        #     output='screen',
        #     name='arm_controller'
        # ),

        #Lidar node
        ExecuteProcess(
            cmd=['ros2', 'run', 'lidar', 'lidar'],
            output='screen',
            name='lidar_node'
        ),

        #Lidar Launch
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robp_launch', 'lidar_launch.yaml'],
            output='screen',
            name='lidar_launch'
        ),

        #Arm camera Launch
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robp_launch', 'arm_camera_launch.yaml'],
            output='screen',
            name='arm_camera_launch'
        ),

        # Camera Launch
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robp_launch', 'rs_d435i_launch.py'],
            output='screen',
            name='camera_launch'
        ),
    ])
