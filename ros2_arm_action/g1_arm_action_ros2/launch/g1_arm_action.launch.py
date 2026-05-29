#!/usr/bin/env python3
"""
Launch file for G1 Arm Action ROS2 Node

This launch file starts the g1_arm_action_node with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='',
        description='DDS network interface name (e.g., eth0, enp2s0). '
                    'Leave empty for default interface.'
    )

    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='10.0',
        description='Timeout for arm action execution in seconds. '
                    'Custom actions may require longer timeout.'
    )

    # Create the node
    g1_arm_action_node = Node(
        package='g1_arm_action_ros2',
        executable='g1_arm_action_node',
        name='g1_arm_action_node',
        output='screen',
        parameters=[{
            'network_interface': LaunchConfiguration('network_interface'),
            'timeout': LaunchConfiguration('timeout'),
        }],
        # Remap if needed
        # remappings=[
        #     ('~/execute_arm_action', '/g1/arm/execute_action'),
        #     ('~/get_arm_action_list', '/g1/arm/get_action_list'),
        # ]
    )

    return LaunchDescription([
        network_interface_arg,
        timeout_arg,
        g1_arm_action_node,
    ])
