#!/usr/bin/env bash
set -eo pipefail
# Pose aiding only. Start disarmed and stationary; never restart in flight.
exec bash /home/jetson/simple_flight_ws/install/simple_flight/share/simple_flight/scripts/start_vision_bridge.sh manual_position
