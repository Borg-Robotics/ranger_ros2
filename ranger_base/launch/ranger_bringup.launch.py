#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ranger_base')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'ranger_base_params.yaml'),
        description='Full path to the ROS2 parameters file to load'
    )

    # ranger_base_node
    ranger_node = Node(
        package='ranger_base',
        executable='ranger_base_node',
        name='ranger_base_node',
        output='both',
        parameters=[
            LaunchConfiguration('config_file')
        ]
    )

    # Create launch description and populate
    ld = LaunchDescription([
        config_file_arg,
        ranger_node
    ])

    return ld