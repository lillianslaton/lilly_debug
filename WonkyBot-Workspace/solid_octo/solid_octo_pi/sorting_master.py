#!/usr/bin/env python3
"""
sorting_master.py

State machine driven entirely by vertical stepper position.
The arm position IS the state — no separate state variable needed.

  steps_ver == 0  (HOME)
    → FIND_BALL phase
    → Drive forward slowly while scanning for a ball
    → When ball detected close enough: stop, lower arm (steps_ver → 92500)

  steps_ver == 92500  (DOWN)
    → GRAB phase
    → Close gripper (steps_hor → 35000)
    → Raise arm back up (steps_ver → 0, steps_hor stays 35000)

  steps_ver == 0 + steps_hor == 35000  (UP WITH BALL)
    → FIND_BUCKET phase
    → Spin/steer toward matching-color bucket
    → When bucket close enough: open gripper (steps_hor → 0) → DONE

  steps_hor == 0 + steps_ver == 0  (HOME AGAIN)
    → Repeat from FIND_BALL
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

# ── Stepper physical limits ───────────────────────────────────
VER_HOME   =     0    # arm raised, home position
VER_DOWN   = 92500    # arm fully lowered to ball level
HOR_HOME   =     0    # gripper open
HOR_CLOSED = 35000    # gripper closed

# Position tolerance — within this many counts = "at target"
VER_TOLERANCE = 500
HOR_TOLERANCE = 300

# ── Drive behaviour ───────────────────────────────────────────
SEARCH_FORWARD_SPEED = 0.10   # m/s — slow creep forward while searching for ball
SEARCH_SPIN_SPEED    = 0.25   # rad/s — spin when no ball seen at all
APPROACH_SPEED       = 0.10   # m/s — creep during fine approach to ball/bucket

# ── Vision tuning ─────────────────────────────────────────────
IMAGE_WIDTH      = 480
IMAGE_CENTRE_X   = IMAGE_WIDTH // 2
CENTRE_DEADBAND  = 20         # pixels — ignore small heading errors
KP_ANGULAR       = 0.004      # pixel error → rad/s

# ── Grab / release triggers ───────────────────────────────────
BALL_GRAB_DISTANCE_M   = 0.28  # stop and lower arm when ball this close
BUCKET_DROP_DISTANCE_M = 0.32  # stop and release when bucket this close
CLOSE_FRAMES_REQUIRED  = 5     # consecutive close frames before triggering

# ── Color priority ────────────────────────────────────────────
TARGET_PRIORITY = ["RED", "YELLOW", "GREEN", "BLUE"]


class SortingMaster(Node):

    def __init__(self):
        super().__init__("sorting_master")

        # ── Publishers ────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist,  "/cmd_vel",       10)
        self.arm_pub     = self.create_publisher(String, "/arm_command",   10)
        self.done_pub    = self.create_publisher(Bool,   "/grab_complete", 10)

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(
            String, "/detected_objects", self._objects_cb, 10)
        self.create_subscription(
            String, "/arm_state",        self._arm_state_cb, 10)

        # ── State ─────────────────────────────────────────────
        self.latest_objects   = []
        self.steps_ver        = VER_HOME
        self.steps_hor        = HOR_HOME
        self.arm_moving       = False
        self.target_color     = None   # color of ball we picked up
        self.close_count      = 0
        self.last_valid_depth = 999.0

        # Whether we have commanded the arm yet this cycle
        # (prevents sending the same arm command on every tick)
        self._grab_commanded    = False
        self._release_commanded = False

        # 10 Hz control loop
        self.create_timer(0.1, self._loop)

        self.get_logger().info(
            "sorting_master ready\n"
            f"  steps_ver={VER_HOME} → FIND_BALL (drive forward)\n"
            f"  steps_ver={VER_DOWN} → GRAB\n"
            f"  steps_ver={VER_HOME} + steps_hor={HOR_CLOSED} → FIND_BUCKET"
        )

    # ══════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════

    def _objects_cb(self, msg: String):
        try:
            self.latest_objects = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_objects = []

    def _arm_state_cb(self, msg: String):
        try:
            s = json.loads(msg.data)
            self.steps_ver  = int(s.get("steps_ver", self.steps_ver))
            self.steps_hor  = int(s.get("steps_hor", self.steps_hor))
            self.arm_moving = bool(s.get("moving",   self.arm_moving))
        except (json.JSONDecodeError, ValueError):
            pass

    # ══════════════════════════════════════════════════════════
    # Main loop — state routed by stepper position
    # ══════════════════════════════════════════════════════════

    def _loop(self):
        # While arm is mid-travel, hold still and wait
        if self.arm_moving:
            self._stop()
            return

        at_ver_home = abs(self.steps_ver - VER_HOME) <= VER_TOLERANCE
        at_ver_down = abs(self.steps_ver - VER_DOWN) <= VER_TOLERANCE
        hor_open    = abs(self.steps_hor - HOR_HOME)   <= HOR_TOLERANCE
        hor_closed  = abs(self.steps_hor - HOR_CLOSED) <= HOR_TOLERANCE

        if at_ver_home and hor_open:
            # ── FIND_BALL: drive forward, steer toward ball ───
            self._grab_commanded = False
            self._phase_find_ball()

        elif at_ver_down and hor_open:
            # ── GRAB: arm is down, close gripper then raise ───
            if not self._grab_commanded:
                self._grab_commanded = True
                self._stop()
                self._send_arm(ver=VER_HOME, hor=HOR_CLOSED)
                self.get_logger().info(
                    f"Arm down — closing gripper ({HOR_CLOSED}) "
                    f"and raising arm ({VER_HOME})"
                )

        elif at_ver_home and hor_closed:
            # ── FIND_BUCKET: arm up, ball gripped ────────────
            self._release_commanded = False
            self._phase_find_bucket()

        else:
            # Unexpected position — return safely to home
            self.get_logger().warn(
                f"Unexpected arm state ver={self.steps_ver} "
                f"hor={self.steps_hor} — homing"
            )
            self._stop()
            self._send_arm(ver=VER_HOME, hor=HOR_HOME)

    # ══════════════════════════════════════════════════════════
    # Phase: FIND_BALL
    # Arm at home, gripper open.
    # Drive forward slowly. Steer toward any visible ball.
    # When close enough, stop and lower arm.
    # ══════════════════════════════════════════════════════════

    def _phase_find_ball(self):
        balls = [o for o in self.latest_objects if o.get("type") == "BALL"]

        if not balls:
            # No ball in sight — drive forward slowly to keep searching
            self._drive(linear=SEARCH_FORWARD_SPEED, angular=0.0)
            self.close_count = 0
            return

        target  = self._pick_priority(balls)
        depth_m = target.get("depth_m", 0.0)
        cx      = target["cx"]

        if depth_m > 0.0:
            self.last_valid_depth = depth_m

        # Check if we're close enough to grab
        close = (
            (depth_m > 0.0 and depth_m <= BALL_GRAB_DISTANCE_M) or
            (depth_m == 0.0 and self.last_valid_depth <= 0.35)
        )
        self.close_count = self.close_count + 1 if close else 0

        if self.close_count >= CLOSE_FRAMES_REQUIRED:
            # Lock in the color, reset counter, lower the arm
            self.target_color     = target["color"]
            self.close_count      = 0
            self.last_valid_depth = 999.0
            self._stop()
            self._send_arm(ver=VER_DOWN, hor=HOR_HOME)
            self.get_logger().info(
                f"Ball in range ({depth_m:.2f}m) — "
                f"lowering arm. Color: {self.target_color}"
            )
            return

        # Steer toward ball while driving forward
        angular = self._heading_correction(cx)
        self._drive(linear=APPROACH_SPEED, angular=angular)

        self.get_logger().debug(
            f"[FIND_BALL] {target['color']} "
            f"cx={cx} depth={depth_m:.2f}m"
        )

    # ══════════════════════════════════════════════════════════
    # Phase: FIND_BUCKET
    # Arm raised, gripper closed (ball held).
    # Steer toward matching-color bucket.
    # When close enough, open gripper and lower arm to home.
    # ══════════════════════════════════════════════════════════

    def _phase_find_bucket(self):
        buckets = [o for o in self.latest_objects if o.get("type") == "BUCKET"]

        # Prefer color-matching bucket; fall back to any bucket visible
        matching   = [b for b in buckets if b.get("color") == self.target_color]
        candidates = matching if matching else buckets

        if not candidates:
            # No bucket visible — spin to search
            self._drive(linear=0.0, angular=SEARCH_SPIN_SPEED)
            self.close_count = 0
            return

        target  = self._pick_priority(candidates)
        depth_m = target.get("depth_m", 0.0)
        cx      = target["cx"]

        if depth_m > 0.0:
            self.last_valid_depth = depth_m

        close = (
            (depth_m > 0.0 and depth_m <= BUCKET_DROP_DISTANCE_M) or
            (depth_m == 0.0 and self.last_valid_depth <= 0.40)
        )
        self.close_count = self.close_count + 1 if close else 0

        if self.close_count >= CLOSE_FRAMES_REQUIRED:
            if not self._release_commanded:
                self._release_commanded = True
                self.close_count      = 0
                self.last_valid_depth = 999.0
                self._stop()
                # Open gripper — arm stays at home
                self._send_arm(ver=VER_HOME, hor=HOR_HOME)
                # Signal coordinator that cycle is complete
                done      = Bool()
                done.data = True
                self.done_pub.publish(done)
                self.get_logger().info(
                    f"Delivered {self.target_color} ball — "
                    f"opening gripper. Returning to FIND_BALL."
                )
                self.target_color = None
            return

        # Steer toward bucket
        angular = self._heading_correction(cx)
        self._drive(linear=APPROACH_SPEED, angular=angular)

        self.get_logger().debug(
            f"[FIND_BUCKET] {target['color']} bucket "
            f"cx={cx} depth={depth_m:.2f}m"
        )

    # ══════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════

    def _pick_priority(self, objects: list) -> dict:
        """Return highest-priority, largest-area object."""
        def key(o):
            try:    pri = TARGET_PRIORITY.index(o["color"])
            except: pri = 99
            return (pri, -o.get("area", 0))
        return sorted(objects, key=key)[0]

    def _heading_correction(self, cx: int) -> float:
        """Proportional angular correction toward pixel centre x."""
        error = cx - IMAGE_CENTRE_X
        if abs(error) < CENTRE_DEADBAND:
            return 0.0
        return -KP_ANGULAR * error

    def _drive(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self.cmd_vel_pub.publish(t)

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    def _send_arm(self, ver: int, hor: int):
        msg      = String()
        msg.data = json.dumps({"steps_ver": ver, "steps_hor": hor})
        self.arm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SortingMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
