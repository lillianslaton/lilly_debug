#!/bin/bash
# ─────────────────────────────────────────────────────────────
# LAPTOP NAVIGATION
# Runs Nav2 + detection + sorting on the laptop while Pi runs drivers.
#
# PREREQUISITES:
#   On the Pi (via SSH):
#     ros2 launch solid_octo pi_driver.launch.py
#
#   A saved map (from laptop_mapping.sh):
#     ~/room_map.yaml and ~/room_map.pgm
#
# USAGE:
#   bash laptop_navigation.sh
#   bash laptop_navigation.sh /path/to/custom_map.yaml
# ─────────────────────────────────────────────────────────────

WORKSPACE=~/seniordesign_ws
MAP_PATH=${1:-~/room_map.yaml}

source /opt/ros/jazzy/setup.bash
source $WORKSPACE/install/setup.bash

echo "════════════════════════════════════════"
echo "   LAPTOP NAVIGATION"
echo "════════════════════════════════════════"
echo ""

# Expand ~ in MAP_PATH
MAP_PATH=$(eval echo "$MAP_PATH")

if [ ! -f "$MAP_PATH" ]; then
  echo "  ERROR: Map not found at $MAP_PATH"
  echo "  Run laptop_mapping.sh first and save your map."
  echo "  Or pass a custom path: bash laptop_navigation.sh /path/to/map.yaml"
  exit 1
fi

echo "  Map: $MAP_PATH"
echo ""
echo "Make sure Pi is running:"
echo "  ros2 launch solid_octo pi_driver.launch.py"
echo ""
echo "Checking for Pi topics..."

timeout 5 ros2 topic list | grep -q "/camera/camera"
if [ $? -ne 0 ]; then
  echo "  WARNING: Camera topics not found. Is pi_driver running?"
else
  echo "  Camera topics found."
fi

timeout 5 ros2 topic list | grep -q "/odom"
if [ $? -ne 0 ]; then
  echo "  WARNING: /odom not found."
else
  echo "  /odom found."
fi

timeout 5 ros2 topic list | grep -q "/scan"
if [ $? -ne 0 ]; then
  echo "  WARNING: /scan not found."
else
  echo "  /scan found."
fi

echo ""
echo "Starting Nav2 + detection pipeline..."
echo "Waypoint navigator will auto-start after 30 seconds."
echo "Press Ctrl+C to stop everything."
echo ""

ros2 launch solid_octo navigation.launch.py map:=$MAP_PATH

