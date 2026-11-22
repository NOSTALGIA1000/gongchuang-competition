"""Connect USB FCU with only telemetry, params, time and external-pose plugins."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    own = Path(get_package_share_directory('simple_flight'))
    mavros = Path(get_package_share_directory('mavros'))
    return LaunchDescription([IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(mavros/'launch/node.launch')),
        launch_arguments={
            'fcu_url': '/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00:57600',
            'gcs_url': '', 'tgt_system': '1', 'tgt_component': '1',
            'pluginlists_yaml': str(own/'config/mavros_bench_plugins.yaml'),
            'config_yaml': str(own/'config/mavros_bench.yaml'),
            'fcu_protocol': 'v2.0', 'respawn_mavros': 'false',
        }.items())])
