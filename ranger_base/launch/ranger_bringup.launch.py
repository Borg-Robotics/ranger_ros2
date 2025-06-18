#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare launch configurations
    port_name_arg   = DeclareLaunchArgument('port_name', default_value='can0', description='CAN port name')
    robot_model_arg = DeclareLaunchArgument('robot_model', default_value='ranger_mini_v2', description='Robot model')
    odom_frame_arg  = DeclareLaunchArgument('odom_frame', default_value='odom', description='Odometry frame')
    base_frame_arg  = DeclareLaunchArgument('base_frame', default_value='robot_footprint', description='Base frame')
    update_rate_arg = DeclareLaunchArgument('update_rate', default_value='50', description='Update rate')
    odom_topic_arg  = DeclareLaunchArgument('odom_topic_name', default_value='odom', description='Odometry topic name')
    publish_tf_arg  = DeclareLaunchArgument('publish_odom_tf', default_value='true', description='Publish odom TF')

    # ranger_base_node
    ranger_node = Node(
        package='ranger_base',
        executable='ranger_base_node',
        name='ranger_base_node',
        output='both',
        parameters=[{
            'port_name':       LaunchConfiguration('port_name'),
            'robot_model':     LaunchConfiguration('robot_model'),
            'odom_frame':      LaunchConfiguration('odom_frame'),
            'base_frame':      LaunchConfiguration('base_frame'),
            'update_rate':     LaunchConfiguration('update_rate'),
            'odom_topic_name': LaunchConfiguration('odom_topic_name'),
            'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
        }]
    )

    # Create launch description and populate
    ld = LaunchDescription([
        port_name_arg,
        robot_model_arg,
        odom_frame_arg,
        base_frame_arg,
        update_rate_arg,
        odom_topic_arg,
        publish_tf_arg,
        ranger_node
    ])

    return ld