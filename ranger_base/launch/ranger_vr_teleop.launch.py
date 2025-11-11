"""
Launch file for Ranger VR Teleoperation
Starts the ranger_vr_teleop node with parameters from config file
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package name
    package_name = 'ranger_base'
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name),
            'config',
            'ranger_vr_teleop_params.yaml'
        ]),
        description='Path to the config file for VR teleop parameters'
    )
    
    # Create the VR teleop node
    ranger_vr_teleop_node = Node(
        package=package_name,
        executable='ranger_vr_teleop',
        name='ranger_vr_teleop',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        ranger_vr_teleop_node,
    ])