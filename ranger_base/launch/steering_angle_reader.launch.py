#!/usr/bin/env python3

"""
Launch file for steering angle reader node
Reads CAN frame 0x271 (motor angles) and publishes steering angle in radians and degrees
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('ranger_base')
    
    # Launch arguments
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='can0',
        description='CAN port name (e.g., can0, can1)'
    )
    
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='ranger_mini_v2',
        description='Robot model (ranger_mini_v1, ranger_mini_v2, ranger_mini_v3, ranger)'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='50',
        description='Update rate in Hz for publishing steering angle'
    )
    
    # Steering angle reader node
    steering_angle_reader_node = Node(
        package='ranger_base',
        executable='steering_angle_reader',
        name='steering_angle_reader',
        output='screen',
        parameters=[{
            'port_name': LaunchConfiguration('port_name'),
            'robot_model': LaunchConfiguration('robot_model'),
            'update_rate': LaunchConfiguration('update_rate')
        }],
        remappings=[
            # Add any topic remappings if needed
        ]
    )

    return LaunchDescription([
        port_name_arg,
        robot_model_arg,
        update_rate_arg,
        steering_angle_reader_node
    ])
