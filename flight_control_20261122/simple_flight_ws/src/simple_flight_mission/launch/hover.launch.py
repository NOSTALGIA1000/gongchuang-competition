"""Independent hover launch. Default mode is check, never flight."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("check", "mode-test", "execute"):
        raise ValueError("mode must be check, mode-test or execute")
    config = str(Path(get_package_share_directory("simple_flight_mission"))/"config/hover.yaml")
    arguments = ["--config", config]
    if mode != "check":
        arguments.append("--" + mode)
    report = LaunchConfiguration("report_dir").perform(context)
    if report:
        arguments.extend(["--report-dir", report])
    # use_global_arguments=False in the executable means no arbitrary ROS
    # remapping can silently turn a check into a different operating mode.
    return [Node(package="simple_flight_mission", executable="hover_control",
                 arguments=arguments, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="check"),
        DeclareLaunchArgument("report_dir", default_value=""),
        OpaqueFunction(function=_start),
    ])
