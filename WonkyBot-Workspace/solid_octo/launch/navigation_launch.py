"""
navigation.launch.py
Runs on the LAPTOP over the same ROS2 network as the Pi.
Starts Nav2, twist_mux, detector, detection_3d, sorting_master,
blind_navigator for full autonomous ball sorting.

Prerequisites:
  - Pi is running: ros2 launch solid_octo pi_driver.launch.py
  - A saved map exists (from mapping.launch.py)
  - Same ROS_DOMAIN_ID on both machines

Usage:
  ros2 launch solid_octo navigation.launch.py map:=/path/to/room_map.yaml

cmd_vel priority (handled by twist_mux):
  255 - cmd_vel_emergency  (manual override, highest)
  100 - cmd_vel_joy        (joystick teleop)
   10 - cmd_vel_nav        (Nav2 path following, lowest)

sorting_master and blind_navigator publish to /cmd_vel directly and are
managed by twist_mux priority. During vision approach, Nav2 is paused
via the /pause_navigation lock topic.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_share_path


def generate_launch_description():

    solid_octo_path = get_package_share_path("solid_octo")
    twist_mux_config = str(solid_octo_path / "configs" / "twist_mux.yaml")

    # ── Arguments ────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        "map",
        description="Full path to the map YAML file",
    )

    nav2_params_default = os.path.join(
        get_package_share_directory("solid_octo_config_nav"),
        "config", "nav2_params.yaml",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=nav2_params_default,
        description="Full path to nav2_params.yaml",
    )

    # ── twist_mux — cmd_vel arbitration ──────────────────────
    # Resolves the conflict between Nav2, sorting_master, and blind_navigator.
    # Nav2 publishes to /cmd_vel_nav (priority 10).
    # Joystick publishes to /cmd_vel_joy (priority 100).
    # Whoever wins outputs to /cmd_vel which octo_pilot reads.
    # /pause_navigation lock pauses Nav2 during vision approach.
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_config],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )

    # ── Nav2 bringup ─────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch", "bringup_launch.py",
            )
        ]),
        launch_arguments={
            "map":         LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": "False",
            "autostart":   "True",
        }.items(),
    )

    # ── Detection pipeline ────────────────────────────────────
    detector_node = Node(
        package="solid_octo",
        executable="detector_node",
        output="screen",
    )

    detection_3d_node = Node(
        package="solid_octo",
        executable="detection_3d",
        output="screen",
    )

    # ── Sorting master ────────────────────────────────────────
    # Stays idle until /vision_activate fires. Publishes to /cmd_vel.
    # twist_mux is NOT involved here — sorting_master publishes at highest
    # effective priority by being the only active /cmd_vel publisher during
    # the vision phase (Nav2 is paused via /pause_navigation).
    sorting_master_node = Node(
        package="solid_octo",
        executable="sorting_master",
        output="screen",
    )

    # ── Blind navigator ───────────────────────────────────────
    # Primary autonomy coordinator. Uses /detected_objects_3d to find balls,
    # blind-navigates to them using /odometry/filtered, hands off to
    # sorting_master, then uses Nav2 for bucket delivery.
    # NOTE: waypoint_navigator is NOT launched — blind_navigator handles
    # the full ball-find → grab → bucket → release cycle.
    blind_navigator_node = Node(
        package="solid_octo",
        executable="blind_navigator",
        output="screen",
    )

    return LaunchDescription([
        map_arg,
        params_arg,

        # twist_mux first — must be up before any velocity commands
        twist_mux_node,

        # Nav2 — allow 3s for twist_mux to be ready
        TimerAction(period=3.0, actions=[nav2_launch]),

        # Detection pipeline — camera must be up (Pi side) before these matter
        TimerAction(period=5.0, actions=[detector_node]),
        TimerAction(period=5.0, actions=[detection_3d_node]),

        # Autonomy nodes — wait for Nav2 to be active
        # blind_navigator uses nav2.waitUntilNav2Active() internally so the
        # timer here is just a safety buffer; 15s is conservative.
        TimerAction(period=8.0,  actions=[sorting_master_node]),
        TimerAction(period=15.0, actions=[blind_navigator_node]),
    ])
