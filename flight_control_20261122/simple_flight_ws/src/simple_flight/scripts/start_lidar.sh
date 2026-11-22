#!/usr/bin/env bash
set -eo pipefail
if ! ip -4 addr show dev enP8p1s0 | grep -q 'inet 192.168.1.5/24'; then
  echo 'MID-360S host address 192.168.1.5/24 is missing on enP8p1s0.' >&2
  echo 'This entry does not change network settings automatically.' >&2
  exit 2
fi
if ! ip route get 255.255.255.255 2>/dev/null | grep -q 'dev enP8p1s0'; then
  echo 'Livox discovery broadcast route must use enP8p1s0; see the setup guide.' >&2
  exit 2
fi
if pgrep -f '[l]ivox_ros_driver2_node' >/dev/null; then
  echo 'A Livox driver is already running; do not start a second instance.' >&2
  exit 3
fi
source /opt/ros/humble/setup.bash
source /home/jetson/livox_ws/install/setup.bash
source /home/jetson/simple_flight_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
exec ros2 launch simple_flight lidar.launch.py "$@"
