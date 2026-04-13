#!/bin/bash
# ─────────────────────────────────────────────────────────────
# MASTER NAVIGATION LAUNCH — WonkyBot v2
# Starts all nodes needed for autonomous ball sorting.
#
# USAGE:
#   bash ~/start_nav.sh
#
# AFTER LAUNCH:
#   Wait for Tab 6 to print "ALL LIFECYCLE NODES ACTIVE"
#   Then set 2D Pose Estimate in RViz.
#   Autonomy starts automatically after Nav2 is ready.
# ─────────────────────────────────────────────────────────────

WORKSPACE=~/seniordesign_ws
MAP_PATH=~/room_map.yaml
PARAMS_PATH=$WORKSPACE/src/solid_octo_config_nav/config/nav2_params.yaml

source /opt/ros/jazzy/setup.bash
source $WORKSPACE/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "════════════════════════════════════════"
echo "   NAVIGATION LAUNCH PREFLIGHT CHECK"
echo "════════════════════════════════════════"

if [ ! -f "$MAP_PATH" ]; then
  echo ""
  echo "  ✗ ERROR: Map not found at $MAP_PATH"
  echo "    Run start_mapping.sh first and save your map."
  echo ""
  exit 1
fi

if [ ! -f "$PARAMS_PATH" ]; then
  echo ""
  echo "  ✗ ERROR: nav2_params.yaml not found at $PARAMS_PATH"
  echo ""
  exit 1
fi

echo "  ✓ Map found:    $MAP_PATH"
echo "  ✓ Params found: $PARAMS_PATH"
echo ""
echo "Starting all nodes..."
echo ""

# ── Tab 1: Motors ────────────────────────────────────────────
echo "[1/10] Starting motors..."
gnome-terminal --tab --title="1. Motors" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- MOTORS ---'
  ros2 launch solid_octo octo_launch.py
  exec bash"

sleep 2

# ── Tab 2: TF Transform ──────────────────────────────────────
echo "[2/10] Starting TF transform..."
gnome-terminal --tab --title="2. TF" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- TF TRANSFORM ---'
  ros2 run tf2_ros static_transform_publisher 0.43 0 0.10 0 0 0 base_link camera_link
  exec bash"

sleep 1

# ── Tab 3: Camera ────────────────────────────────────────────
echo "[3/10] Starting RealSense camera..."
gnome-terminal --tab --title="3. Camera" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- REALSENSE CAMERA ---'
  ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    unite_imu_method:=2 \
    enable_gyro:=true \
    enable_accel:=true
  exec bash"

sleep 3

# ── Tab 4: Depth → LaserScan ─────────────────────────────────
echo "[4/10] Starting depth to laserscan..."
gnome-terminal --tab --title="4. LaserScan" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- DEPTH TO LASERSCAN ---'
  ros2 run depthimage_to_laserscan depthimage_to_laserscan_node \
    --ros-args \
    -p output_frame:=camera_link \
    -p range_min:=0.35 \
    -p range_max:=4.0 \
    -r depth:=/camera/camera/depth/image_rect_raw \
    -r depth_camera_info:=/camera/camera/depth/camera_info
  exec bash"

sleep 2

# ── Tab 5: Nav2 ──────────────────────────────────────────────
echo "[5/10] Starting Nav2..."
gnome-terminal --tab --title="5. Nav2" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- NAV2 ---'
  ros2 launch nav2_bringup bringup_launch.py \
    map:=$MAP_PATH \
    params_file:=$PARAMS_PATH \
    use_sim_time:=False \
    autostart:=False
  exec bash"

sleep 3

# ── Tab 6: Lifecycle Activation ──────────────────────────────
echo "[6/10] Starting lifecycle activation..."
gnome-terminal --tab --title="6. Lifecycle" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- LIFECYCLE ACTIVATION ---'
  echo 'Waiting for Nav2 to finish starting...'
  sleep 8

  echo 'Step 1/3: Localization...'
  ros2 lifecycle set /map_server configure
  ros2 lifecycle set /amcl configure
  ros2 lifecycle set /map_server activate
  ros2 lifecycle set /amcl activate

  echo 'Setting initial pose at origin...'
  ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    \"{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}\"

  echo 'Step 2/3: Configuring servers...'
  ros2 lifecycle set /controller_server configure
  ros2 lifecycle set /planner_server configure
  ros2 lifecycle set /behavior_server configure
  ros2 lifecycle set /smoother_server configure
  ros2 lifecycle set /waypoint_follower configure

  echo 'Step 3/3: Activating servers...'
  ros2 lifecycle set /controller_server activate
  ros2 lifecycle set /planner_server activate
  ros2 lifecycle set /behavior_server activate
  ros2 lifecycle set /smoother_server activate
  ros2 lifecycle set /waypoint_follower activate

  echo 'Activating navigator...'
  ros2 lifecycle set /bt_navigator configure
  ros2 lifecycle set /bt_navigator activate

  echo ''
  echo '══════════════════════════════════════'
  echo '  ✓ ALL LIFECYCLE NODES ACTIVE'
  echo '  Set 2D Pose Estimate in RViz now.'
  echo '══════════════════════════════════════'
  exec bash"

sleep 2

# ── Tab 7: twist_mux — cmd_vel arbitration ───────────────────
echo "[7/10] Starting twist_mux..."
gnome-terminal --tab --title="7. twist_mux" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- TWIST_MUX ---'
  ros2 run twist_mux twist_mux \
    --ros-args \
    --params-file $WORKSPACE/src/solid_octo/configs/twist_mux.yaml \
    -r /cmd_vel_out:=/cmd_vel
  exec bash"

sleep 1

# ── Tab 8: cmd_vel relay (Nav2 → twist_mux) ──────────────────
echo "[8/10] Starting cmd_vel relay..."
gnome-terminal --tab --title="8. Relay" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- CMD_VEL RELAY ---'
  python3 -c \"
import rclpy
from geometry_msgs.msg import Twist
rclpy.init()
node = rclpy.create_node('cmd_vel_relay')
pub  = node.create_publisher(Twist, '/cmd_vel_nav', 10)
sub  = node.create_subscription(Twist, '/cmd_vel_nav_in', lambda m: pub.publish(m), 10)
print('Relay running: /cmd_vel_nav_in -> /cmd_vel_nav')
rclpy.spin(node)
\"
  exec bash"

sleep 1

# ── Tab 9: Detector node ─────────────────────────────────────
echo "[9/10] Starting detector node..."
gnome-terminal --tab --title="9. Detector" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- DETECTOR NODE ---'
  echo 'Waiting for camera to be ready...'
  sleep 6
  ros2 run solid_octo detector_node
  exec bash"

sleep 1

# ── Tab 10: Sorting master ────────────────────────────────────
echo "[10/10] Starting sorting master (autonomy)..."
gnome-terminal --tab --title="10. Autonomy" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source $WORKSPACE/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo '--- SORTING MASTER ---'
  echo 'Waiting for Nav2 lifecycle to complete...'

  # Active wait — polls until Nav2 is confirmed active
  until ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q 'active'; do
    echo '  ... waiting for Nav2 ...'
    sleep 2
  done

  echo '✓ Nav2 active — starting sorting master'
  ros2 run solid_octo sorting_master
  exec bash"

echo ""
echo "════════════════════════════════════════"
echo "   ALL 10 NODES LAUNCHED"
echo "════════════════════════════════════════"
echo ""
echo "Next steps:"
echo "  1. Wait for Tab 6: 'ALL LIFECYCLE NODES ACTIVE'"
echo "  2. Set 2D Pose Estimate in RViz"
echo "  3. Robot will begin autonomy automatically"
echo ""
echo "Troubleshooting:"
echo "  Not moving       → check Tab 1 (motors)"
echo "  No map           → check Tab 5 (Nav2)"
echo "  No laser scan    → check Tab 4"
echo "  TF errors        → check Tab 2"
echo "  No detections    → check Tab 9 (detector)"
echo ""

wait
