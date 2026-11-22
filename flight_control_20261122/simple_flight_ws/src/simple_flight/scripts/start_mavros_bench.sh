#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jetson/simple_flight_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
exec 9>/home/jetson/simple_flight_ws/log/mavros_bench.lock
if ! flock -n 9; then
  echo 'The MAVROS bench connection is already running.' >&2
  exit 3
fi
if pgrep -f '[m]avros_node' >/dev/null; then
  echo 'A MAVROS node already exists.' >&2
  exit 3
fi
port=/dev/serial/by-id/usb-Auterion_PX4_FMU_v6C.x_0-if00
if [[ ! -e "$port" ]] || fuser "$port" >/dev/null 2>&1; then
  echo 'Expected PX4 USB port is absent or already in use.' >&2
  exit 2
fi
echo 'BENCH ONLY: telemetry/parameters/external pose; no automatic arm, mode or setpoints.'
exec ros2 launch simple_flight mavros_bench.launch.py
