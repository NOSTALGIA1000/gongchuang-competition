#!/usr/bin/env bash
set -eo pipefail

mkdir -p /home/jetson/simple_flight_ws/log
exec 9>/home/jetson/simple_flight_ws/log/boot_supervisor.lock
if ! flock -n 9; then
  echo 'The boot localization supervisor is already running.' >&2
  exit 3
fi

source /opt/ros/humble/setup.bash
source /home/jetson/livox_ws/install/setup.bash
source /home/jetson/fastlio_ws/install/setup.bash
source /home/jetson/simple_flight_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONUNBUFFERED=1

exec ros2 run simple_flight boot_supervisor
