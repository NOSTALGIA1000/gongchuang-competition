#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/jetson/livox_ws/install/setup.bash
source /home/jetson/fastlio_ws/install/setup.bash
source /home/jetson/simple_flight_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if pgrep -f '[f]astlio_mapping' >/dev/null; then
  echo 'FAST-LIO is already running; do not start a second instance.' >&2
  exit 3
fi
if ! timeout 20 python3 -m simple_flight.lidar_input_wait --timeout 10; then
  echo 'Localization input wait failed: /livox/lidar must publish fresh nonempty CustomMsg data.' >&2
  exit 2
fi
echo 'BENCH ONLY: extrinsics are initial/unmeasured. No flight interfaces are started.'
exec ros2 launch simple_flight localization.launch.py
