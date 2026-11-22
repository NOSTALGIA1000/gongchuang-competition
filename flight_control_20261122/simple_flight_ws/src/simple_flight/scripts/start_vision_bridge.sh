#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jetson/simple_flight_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
operation_mode="${1:-bench}"
if [[ "$#" -gt 1 || ( "$operation_mode" != bench && "$operation_mode" != manual_position ) ]]; then
  echo 'Usage: start_vision_bridge.sh [bench|manual_position]' >&2
  exit 2
fi
exec 9>/home/jetson/simple_flight_ws/log/vision_bridge.lock
if ! flock -n 9; then
  echo 'A vision bridge is already running.' >&2
  exit 3
fi
if pgrep -f '[s]imple_flight/vision_bridge' >/dev/null; then
  echo 'A vision bridge process already exists.' >&2
  exit 3
fi
exec ros2 run simple_flight vision_bridge --ros-args --params-file \
  /home/jetson/simple_flight_ws/install/simple_flight/share/simple_flight/config/vision_bridge.yaml \
  -p "operation_mode:=$operation_mode"
