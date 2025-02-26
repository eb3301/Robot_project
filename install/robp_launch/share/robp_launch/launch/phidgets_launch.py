"""Launch phidgets devices in a container."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robp_phidgets',
            namespace='robp_phidgets',
            executable='robp_phidgets',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'motors_acceleration': 100.0,
                'motors_braking_strength': 1.0,
                'motors_data_rate': 10.0,
                'motors_failsafe_timeout_ms': 500,
                'motors_native_failsafe': True,
                'motors_native_failsafe_extra_timeout_ms': 100
            }]
        )
    ])
