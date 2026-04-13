#!/usr/bin/env python3
"""
blind_navigator.py

Primary autonomy coordinator for WonkyBot.

Full cycle per ball:
  1. Listen to /detected_objects_3d for ball positions
  2. Drive to a point APPROACH_OFFSET metres in front of the ball
     using proportional odometry control (/odometry/filtered)
  3. Pause Nav2 via /pause_navigation (twist_mux lock)
  4. Publish /vision_activate → sorting_master does fine approach + GRAB
  5. Wait for /grab_complete
  6. Unpause Nav2; use Nav2 to drive to the known bucket waypoint
  7. Publish /release_activate → sorting_master sends RELEASE
  8. Return to IDLE and repeat

NOTE: waypoint_navigator.py is NOT used — this node is the sequencer.
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool

from tf_transformations import euler_from_quaternion
from math import sin, cos, atan2, hypot

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ── Proportional controller ───────────────────────────────────
KP_V = 0.5
KP_W = 1.0
MAX_V = 0.3
MAX_W = 0.5
DIST_TOLERANCE = 0.08   # metres — goal reached threshold

# ── Ball approach ─────────────────────────────────────────────
# Stop this far from the ball so sorting_master can fine-approach
APPROACH_OFFSET = 0.45  # metres

# ── Grab timeout ─────────────────────────────────────────────
GRAB_TIMEOUT_SEC = 45.0

# ── Bucket waypoints (known fixed positions) ──────────────────
# Format: { "COLOR": (x_map, y_map, yaw_rad) }
# Fill these in after measuring your arena with AMCL pose echo.
BUCKET_WAYPOINTS = {
    "RED":    (3.387, -3.133, -2.138),
    "YELLOW": (0.000,  0.000,  0.000),  # TODO: measure
    "GREEN":  (0.000,  0.000,  0.000),  # TODO: measure
    "BLUE":   (0.000,  0.000,  0.000),  # TODO: measure
}

HOME = (0.000, 0.000, 0.000)


def make_pose(nav: BasicNavigator, x, y, yaw) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id    = "map"
    pose.header.stamp       = nav.get_clock().now().to_msg()
    pose.pose.position.x    = x
    pose.pose.position.y    = y
    pose.pose.position.z    = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class BlindNavigator(Node):

    def __init__(self):
        super().__init__("blind_navigation")

        # ── Publishers ────────────────────────────────────────
        self.vel_pub     = self.create_publisher(Twist,  "/cmd_vel",           1)
        self.vision_pub  = self.create_publisher(Bool,   "/vision_activate",  10)
        self.release_pub = self.create_publisher(Bool,   "/release_activate", 10)
        # twist_mux lock: publishing True pauses Nav2 (/cmd_vel_nav priority 10)
        self.pause_pub   = self.create_publisher(Bool,   "/pause_navigation", 10)

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_cb, 1)
        self.create_subscription(
            String, "/detected_objects_3d", self._detections_3d_cb, 10)
        self.create_subscription(
            Bool, "/grab_complete", self._grab_complete_cb, 10)

        # ── Robot state ───────────────────────────────────────
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.robot_theta = 0.0

        # ── Goal state ────────────────────────────────────────
        self.goal_x          = 0.0
        self.goal_y          = 0.0
        self.has_goal        = False
        self.is_goal_reached = True

        # ── Autonomy state machine ────────────────────────────
        # States: IDLE → APPROACH_BALL → WAIT_GRAB → NAV_BUCKET → IDLE
        self.state        = "IDLE"
        self.target_color = None
        self.grab_done    = False

        # ── Nav2 ─────────────────────────────────────────────
        # Initialised lazily in run() after Nav2 is active
        self.nav = None

        # ── 20 Hz control loop ────────────────────────────────
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            "BlindNavigator ready.\n"
            "Waiting for detections on /detected_objects_3d..."
        )

    # ══════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def _detections_3d_cb(self, msg: String):
        # Only update goal when we're idle and looking for a ball
        if self.state != "IDLE":
            return
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        balls = [d for d in detections if d["type"] == "BALL"]
        if not balls:
            return

        # Pick closest ball by depth
        best = min(balls, key=lambda d: d["depth_m"])
        bx = best["odom_xyz"][0]
        by = best["odom_xyz"][1]

        # Compute approach point APPROACH_OFFSET metres before the ball
        dx = bx - self.robot_x
        dy = by - self.robot_y
        dist = hypot(dx, dy)

        if dist < DIST_TOLERANCE:
            # Already close enough — hand off to vision immediately
            self._start_vision_approach(best["color"])
            return

        if dist > APPROACH_OFFSET:
            ratio = (dist - APPROACH_OFFSET) / dist
            gx = self.robot_x + dx * ratio
            gy = self.robot_y + dy * ratio
        else:
            gx, gy = bx, by

        self._set_goal(gx, gy)
        self.target_color = best["color"]
        self.state = "APPROACH_BALL"
        self.get_logger().info(
            f"[APPROACH_BALL] {best['color']} ball at odom "
            f"({bx:.2f},{by:.2f}), driving to ({gx:.2f},{gy:.2f})"
        )

    def _grab_complete_cb(self, msg: Bool):
        if msg.data and self.state == "WAIT_GRAB":
            self.grab_done = True
            self.get_logger().info("Grab confirmed — heading to bucket")

    # ══════════════════════════════════════════════════════════
    # State machine tick (20 Hz)
    # ══════════════════════════════════════════════════════════

    def _tick(self):
        if self.state == "APPROACH_BALL":
            self._run_proportional_drive()

        elif self.state == "WAIT_GRAB":
            # sorting_master owns /cmd_vel; we just wait
            if self.grab_done:
                self._end_vision_approach()

        # NAV_BUCKET is managed by Nav2 via run() not the tick

    # ══════════════════════════════════════════════════════════
    # Proportional drive toward self.goal_x / self.goal_y
    # ══════════════════════════════════════════════════════════

    def _run_proportional_drive(self):
        if not self.has_goal or self.is_goal_reached:
            return

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist = hypot(dx, dy)

        if dist < DIST_TOLERANCE:
            self._stop()
            self.is_goal_reached = True
            self.get_logger().info(
                f"Approach waypoint reached ({self.goal_x:.2f},{self.goal_y:.2f})"
            )
            # Hand off to sorting_master
            self._start_vision_approach(self.target_color)
            return

        target_heading = atan2(dy, dx)
        heading_error  = atan2(
            sin(target_heading - self.robot_theta),
            cos(target_heading - self.robot_theta),
        )
        cmd_w = KP_W * heading_error
        cmd_v = KP_V * dist * max(0.0, cos(heading_error))

        msg = Twist()
        msg.linear.x  = max(min(cmd_v, MAX_V), -MAX_V)
        msg.angular.z = max(min(cmd_w, MAX_W), -MAX_W)
        self.vel_pub.publish(msg)

    # ══════════════════════════════════════════════════════════
    # Vision handoff helpers
    # ══════════════════════════════════════════════════════════

    def _start_vision_approach(self, color: str):
        """Pause Nav2 and activate sorting_master."""
        self.target_color = color
        self.grab_done    = False
        self.state        = "WAIT_GRAB"

        # Lock twist_mux so Nav2 can't interfere
        lock = Bool(); lock.data = True
        self.pause_pub.publish(lock)

        # Activate sorting_master
        activate = Bool(); activate.data = True
        self.vision_pub.publish(activate)

        self.get_logger().info(
            f"[WAIT_GRAB] Vision activated for {color} ball. "
            f"Nav2 paused via /pause_navigation."
        )

    def _end_vision_approach(self):
        """Unlock Nav2 and drive to bucket."""
        # Unlock twist_mux
        lock = Bool(); lock.data = False
        self.pause_pub.publish(lock)

        self.state = "NAV_BUCKET"
        self.get_logger().info(
            f"[NAV_BUCKET] Driving to {self.target_color} bucket via Nav2"
        )

        # Nav2 bucket delivery (blocking call — runs in spin thread via run())
        self._nav_to_bucket()

    # ══════════════════════════════════════════════════════════
    # Nav2 bucket delivery
    # ══════════════════════════════════════════════════════════

    def _nav_to_bucket(self):
        if self.nav is None:
            self.get_logger().warn("Nav2 not initialised — cannot deliver to bucket")
            self.state = "IDLE"
            return

        waypoint = BUCKET_WAYPOINTS.get(self.target_color)
        if waypoint is None:
            self.get_logger().warn(
                f"No bucket waypoint for color '{self.target_color}' — returning home"
            )
            waypoint = HOME

        bx, by, byaw = waypoint
        self.get_logger().info(f"Nav2 → bucket ({bx:.2f},{by:.2f})")

        self.nav.goToPose(make_pose(self.nav, bx, by, byaw))
        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.05)
            fb = self.nav.getFeedback()
            if fb:
                self.get_logger().info(
                    f"  Bucket approach — {fb.distance_remaining:.2f}m remaining",
                    throttle_duration_sec=2.0,
                )

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Reached bucket — releasing ball")
        else:
            self.get_logger().warn(f"Bucket nav failed: {result} — releasing anyway")

        # Release
        release = Bool(); release.data = True
        self.release_pub.publish(release)

        # Small pause for arm
        import time; time.sleep(1.5)

        self.state = "IDLE"
        self.target_color = None
        self.get_logger().info("[IDLE] Cycle complete. Waiting for next ball.")

    # ══════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════

    def _set_goal(self, x: float, y: float):
        self.goal_x          = x
        self.goal_y          = y
        self.has_goal        = True
        self.is_goal_reached = False

    def _stop(self):
        self.vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = BlindNavigator()

    # Initialise Nav2 and wait for it to be active before spinning
    node.nav = BasicNavigator()
    node.get_logger().info("Waiting for Nav2 to become active...")
    node.nav.waitUntilNav2Active()
    node.get_logger().info("Nav2 active — autonomy loop starting.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
