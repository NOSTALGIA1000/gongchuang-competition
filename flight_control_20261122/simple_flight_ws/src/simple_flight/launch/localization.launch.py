"""FAST-LIO bench estimator only; radar must already publish CustomMsg."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('simple_flight')
    return LaunchDescription([Node(
        package='fast_lio', executable='fastlio_mapping', name='simple_fastlio',
        parameters=[os.path.join(share, 'config', 'mid360s_fastlio.yaml')],
        output='screen',
    )])
