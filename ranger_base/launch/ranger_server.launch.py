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
        default_value=os.path.join(pkg_dir, 'config', 'ranger_server_params.yaml'),
        description='Full path to the ROS2 parameters file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # ranger_server node
    ranger_server_node = Node(
        package='ranger_base',
        executable='ranger_server',
        name='ranger_server',
        output='both',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    # Create launch description and populate
    ld = LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        ranger_server_node
    ])

    return ld
