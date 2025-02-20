"""Launch phidgets devices in a container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='robp_phidget_container',
        namespace='',
        package='rclcpp_components',
        # Could also try 'component_container_mt' ('component_container' is not a good idea since it is single-threaded)
        executable='component_container_isolated',
        composable_node_descriptions=[
                ComposableNode(
                    package='robp_phidgets_encoders',
                    plugin='robp::phidgets::Encoders',
                    name='robp_phidgets_encoders'),
                ComposableNode(
                    package='robp_phidgets_motors',
                    plugin='robp::phidgets::Motors',
                    name='robp_phidgets_motors'),
                ComposableNode(
                    package='robp_phidgets_spatial',
                    plugin='robp::phidgets::Spatial',
                    name='robp_phidgets_spatial'),
                ComposableNode(
                    package='robp_phidgets_temperature',
                    plugin='robp::phidgets::Temperature',
                    name='robp_phidgets_temperature')
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
