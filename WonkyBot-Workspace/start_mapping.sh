#!/bin/bash
# ─────────────────────────────────────────────────────────────
# MASTER MAPPING LAUNCH
# Starts all nodes needed to build a map with RTABMAP
#
# USAGE:
#   ./start_mapping.sh
#
# WHEN DONE MAPPING:
#   1. In Terminal 5 (RTABMAP), the map saves automatically to ~/.ros/rtabmap.db
#   2. To export to a Nav2-compatible map, run in a NEW terminal:
#      ros2 run nav2_map_server map_saver_cli -f ~/room_map -t /rtabmap/map
#   3. This creates room_map.yaml and room_map.pgm in your home folder
#   4. Kill this script with Ctrl+C when done
#
# CONTROLS DURING MAPPING:
#   Use your joystick (joy node) or teleop_twist_keyboard to drive
#   Drive slowly and cover the entire arena at least once
#   Overlap your path slightly so RTABMAP gets loop closures
# ─────────────────────────────────────────────────────────────

WORKSPACE=~/seniordesign_ws
MAP_SAVE_PATH=~/room_map

# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source $WORKSPACE/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "════════════════════════════════════════"
echo "   MAPPING LAUNCH SEQUENCE"
echo "════════════════════════════════════════"
echo ""
echo "Starting all nodes in background..."
echo "Check each terminal tab for errors."
echo ""

# ── Terminal 1: Motors ───────────────────────────────────────
echo "[1/5] Starting motors (octo_launch)..."
gnome-terminal --tab --title="1. Motors" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- MOTORS ---'
  ros2 launch solid_octo octo_launch.py
  exec bash"

sleep 2

# ── Terminal 2: TF Transform ─────────────────────────────────
echo "[2/5] Starting TF transform (camera → base_link)..."
gnome-terminal --tab --title="2. TF" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- TF TRANSFORM ---'
  ros2 run tf2_ros static_transform_publisher 0.43 0 0.10 0 0 0 base_link camera_link
  exec bash"

sleep 1

# ── Terminal 3: Camera ───────────────────────────────────────
echo "[3/5] Starting RealSense camera..."
gnome-terminal --tab --title="3. Camera" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- REALSENSE CAMERA ---'
  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
  exec bash"

sleep 3

# ── Terminal 4: Depth → LaserScan ────────────────────────────
echo "[4/5] Starting depth to laserscan..."
gnome-terminal --tab --title="4. LaserScan" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- DEPTH TO LASERSCAN ---'
  ros2 run depthimage_to_laserscan depthimage_to_laserscan_node \
    --ros-args \
    -p output_frame:=camera_link \
    -r depth:=/camera/camera/depth/image_rect_raw \
    -r depth_camera_info:=/camera/camera/depth/camera_info
  exec bash"

sleep 2

# ── Terminal 5: RTABMAP ──────────────────────────────────────
echo "[5/5] Starting RTABMAP..."
gnome-terminal --tab --title="5. RTABMAP" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- RTABMAP MAPPING ---'
  ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:='--delete_db_on_start' \
    visual_odometry:=false \
    frame_id:=base_link \
    odom_topic:=/odom \
    subscribe_scan:=true \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    rgb_topic:=/camera/camera/color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    approx_sync:=true \
    queue_size:=20
  exec bash"

sleep 2

echo ""
echo "════════════════════════════════════════"
echo "   ALL NODES LAUNCHED"
echo "════════════════════════════════════════"
echo ""
echo "Drive the robot around the full arena slowly."
echo "Cover all areas with slight path overlap."
echo ""
echo "WHEN DONE — save the map by running in a new terminal:"
echo "  ros2 run nav2_map_server map_saver_cli -f ~/room_map -t /rtabmap/map"
echo ""
echo "This script stays open. Press Ctrl+C to kill everything."
echo ""

# Keep script alive so Ctrl+C kills terminal session cleanly
wait

