#!/usr/bin/env bash

# Strict mode for safety
set -euo pipefail

# Prompt for SeaTrac IDs (optional for auto-run setup)
read -rp "Local SeaTrac ID? " self_beacon_id
read -rp "Remote SeaTrac ID? " destination_id

#setup so that it is viewable from wherever
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast><Interfaces><NetworkInterface>lo</NetworkInterface></Interfaces></General></Domain></CycloneDDS>"

# Source ROS 2 + workspace setup files
set +u  # allow unset variables during sourcing
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/setup.bash
set -u

#for some reason ros topics arent exposed unless we do this
ros2 daemon stop
ros2 daemon start

# Launch the ROS 2 launch file with parameters
exec ros2 launch seatrac_messaging seatrac_messaging.launch.py \
  self_beacon_id:="${self_beacon_id}" \
  beacon_destination_id:="${destination_id}"
