"""Independent route launch with an explicit editable YAML path; default check."""
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
    config = LaunchConfiguration("config_file").perform(context)
    arguments = ["--config", config]
    if mode != "check":
        arguments.append("--"+mode)
    report = LaunchConfiguration("report_dir").perform(context)
    if report:
        arguments.extend(["--report-dir", report])
    return [Node(package="simple_flight_mission", executable="route_control",
                 arguments=arguments, output="screen")]


def generate_launch_description():
    config = str(Path(get_package_share_directory("simple_flight_mission"))/"config/route.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="check"),
        DeclareLaunchArgument("config_file", default_value=config),
        DeclareLaunchArgument("report_dir", default_value=""),
        OpaqueFunction(function=_start),
    ])
