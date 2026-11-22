"""MID-360 driver only. No MAVROS, flight controller, or motion commands."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('simple_flight')
    return LaunchDescription([
        DeclareLaunchArgument('xfer_format', default_value='1'),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[os.path.join(share, 'config', 'lidar.yaml'), {
                'user_config_path': os.path.join(share, 'config', 'MID360_config.json'),
                'xfer_format': ParameterValue(LaunchConfiguration('xfer_format'), value_type=int),
            }],
        ),
    ])
