#!/usr/bin/env bash
# Standalone ground survey; no mission/controller invocation or configuration writes.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/simple_flight_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export PYTHONUNBUFFERED=1
exec python3 "$HOME/simple_flight_ws/src/simple_flight_mission/scripts/coordinate_calibration.py" "$@"
