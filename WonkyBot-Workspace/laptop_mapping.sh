#!/bin/bash
# ─────────────────────────────────────────────────────────────
# LAPTOP MAPPING
# Runs RTABMAP on the laptop while Pi runs the drivers.
#
# PREREQUISITES:
#   On the Pi (via SSH):
#     ros2 launch solid_octo pi_driver.launch.py
#
# USAGE:
#   bash laptop_mapping.sh
#
# WHEN DONE:
#   In a separate terminal, save the map:
#     ros2 run nav2_map_server map_saver_cli -f ~/room_map -t /rtabmap/map
#
#   Then Ctrl+C this script to stop RTABMAP.
# ─────────────────────────────────────────────────────────────

WORKSPACE=~/seniordesign_ws

source /opt/ros/jazzy/setup.bash
source $WORKSPACE/install/setup.bash

echo "════════════════════════════════════════"
echo "   LAPTOP MAPPING"
echo "════════════════════════════════════════"
echo ""
echo "Make sure Pi is running:"
echo "  ros2 launch solid_octo pi_driver.launch.py"
echo ""
echo "Checking for Pi topics..."

# Quick check that Pi topics are visible
timeout 5 ros2 topic list | grep -q "/camera/camera"
if [ $? -ne 0 ]; then
  echo "  WARNING: Camera topics not found. Is pi_driver running?"
  echo "  Continuing anyway..."
else
  echo "  Camera topics found."
fi

timeout 5 ros2 topic list | grep -q "/odom"
if [ $? -ne 0 ]; then
  echo "  WARNING: /odom not found. Is octo_pilot running on the Pi?"
  echo "  Continuing anyway..."
else
  echo "  /odom found."
fi

timeout 5 ros2 topic list | grep -q "/scan"
if [ $? -ne 0 ]; then
  echo "  WARNING: /scan not found. Is depthimage_to_laserscan running?"
  echo "  Continuing anyway..."
else
  echo "  /scan found."
fi

echo ""
echo "Starting RTABMAP..."
echo "Drive the robot slowly around the arena with the joystick."
echo "Press Ctrl+C when done mapping."
echo ""

ros2 launch solid_octo mapping.launch.py

