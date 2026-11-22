#!/usr/bin/env bash
set -eo pipefail
cd "$HOME/simple_flight_ws"
source /opt/ros/humble/setup.bash
colcon build --base-paths src/simple_flight_payload src/simple_flight_mission \
  --packages-select simple_flight_payload simple_flight_mission --executor sequential
python3 src/simple_flight_payload/tools/verify_install.py --repair
