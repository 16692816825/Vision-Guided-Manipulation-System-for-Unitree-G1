#!/usr/bin/env python3
"""
Launch file for G1 Arm Action ROS2 Node with YAML config file

This launch file loads parameters from a YAML configuration file.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('g1_arm_action_ros2')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'arm_action_params.yaml')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the parameter configuration file'
    )

    # Create the node
    g1_arm_action_node = Node(
        package='g1_arm_action_ros2',
        executable='g1_arm_action_node',
        name='g1_arm_action_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        g1_arm_action_node,
    ])
