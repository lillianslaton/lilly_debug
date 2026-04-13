# WonkyBot v2

Autonomous ball-sorting robot using ROS 2 Jazzy, Nav2, RealSense D455, and a Raspberry Pi 5 + Pico 2.

## Quick start
- Pi drivers:   `ros2 launch solid_octo pi_driver.launch.py`
- Navigation:   `bash laptop_navigation.sh`
- Mapping:      `bash laptop_mapping.sh`

## State machine
Driven by vertical stepper position:
- `steps_ver=0, hor=0`      → find ball (drive forward)
- `steps_ver=92500, hor=0`  → grab (lower arm, close gripper, raise)
- `steps_ver=0, hor=35000`  → find bucket (deliver)

## Key tuning values
- `BALL_GRAB_DISTANCE_M` in sorting_master.py
- `WHEEL_SEP = 0.477` in diff_drive_controller.py
- HSV ranges in colors.json
