import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),

        #Start Rviz
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', '/home/robot/dd2419_ws/src/localisation/launch/rviz/loc.rviz'],
            output='screen',
            name='rviz'
        ),
        # args="-d $(find-pkg-share robp_launch)/rviz/default.rviz"
        
        # # Start odometry
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'odometry', 'odometry'],
        #     output='screen',
        #     name='odometry_node'
        # ),

        #Start camera
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'detection', 'detection'],
        #     output='screen',
        #     name='camera_node'
        # ),
        
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
        
        #Static broadcaster: LiDAR - base_link
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher','--x', '0.0', '--y', '0.815', '--z', '0.12', '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
            output='screen',
            name='base_to_lidar'
        ),

        # #Lidar node
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'localisation', 'lidar'],
        #     output='screen',
        #     name='lidar_node'
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

        #Occupancy grid node
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
        )
    ])
