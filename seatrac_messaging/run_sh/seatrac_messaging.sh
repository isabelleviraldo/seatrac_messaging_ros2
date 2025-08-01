#!/usr/bin/env bash

# Strict mode for safety
set -euo pipefail

# Prompt for SeaTrac IDs (optional for auto-run setup)
read -rp "Local SeaTrac ID? " self_beacon_id
read -rp "Remote SeaTrac ID? " destination_id

# Source ROS 2 + workspace setup files
set +u  # allow unset variables during sourcing
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -u

# Launch the ROS 2 launch file with parameters
exec ros2 launch seatrac_messaging seatrac_messaging.launch.py \
  self_beacon_id:="${self_beacon_idS}" \
  beacon_destination_id:="${destination_id}"
