import json
import rclpy
from rclpy.node import Node

from tf_transformations import quaternion_about_axis

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import serial
from math import sin, cos, atan2

# ── Pico serial format ────────────────────────────────────────
# Send:    "lin_vel,ang_vel,step_dir,step2_dir\n"
# Receive: "meas_lin_vel,meas_ang_vel\n"
#
# step_dir  → stepper s (vertical / raise-lower):  -1=raise  0=stop  1=lower
# step2_dir → stepper a (horizontal / open-close): -1=open   0=stop  1=close

# ── Stepper physical limits (pulse counts from Pico) ─────────
VER_HOME =      0   # arm raised, resting position
VER_DOWN =  92500   # arm fully lowered to ball level

HOR_HOME =      0   # gripper fully open
HOR_CLOSED = 35000  # gripper fully closed around ball

# Safety clamps — never command outside these
VER_MIN =      0
VER_MAX =  92500
HOR_MIN =      0
HOR_MAX =  35000

# ── Motion: how many Pico ticks per pulse count unit ─────────
# The Pico stepper timer fires at 1,000,000 / (pulse_width * 2) Hz.
# Each tick here = one 0.0133s serial cycle.
# COUNTS_PER_TICK controls how fast the arm moves:
#   higher = faster arm travel per serial tick.
# Start at 500 and tune until arm speed feels right.
COUNTS_PER_TICK = 500

# Tolerance: how close to target counts as "arrived"
VER_TOLERANCE = 200
HOR_TOLERANCE = 200

# ── Grab sequence ─────────────────────────────────────────────
# Vertical moves first (lower), then horizontal (close gripper).
# Set SEQUENTIAL = False to move both simultaneously.
SEQUENTIAL_MOTION = True

# ── /arm_state publish rate ───────────────────────────────────
ARM_STATE_EVERY = 10   # publish every N ticks (~7 Hz)


class OctoPilot(Node):
    def __init__(self):
        super().__init__("octo_pilot")

        # ── Serial to Pico ────────────────────────────────────
        self.pico_msngr      = serial.Serial("/dev/ttyACM0", 115200, timeout=0.01)
        self.pico_comm_timer = self.create_timer(0.0133, self._tick)

        # ── Subscribers ───────────────────────────────────────
        self.create_subscription(Twist,  "/cmd_vel",     self._vel_cb,     1)
        self.create_subscription(String, "/arm_command", self._arm_cmd_cb, 10)

        # ── Publishers ────────────────────────────────────────
        self.odom_pub      = self.create_publisher(Odometry, "/odom",      1)
        self.arm_state_pub = self.create_publisher(String,   "/arm_state", 10)

        # ── Drive state ───────────────────────────────────────
        self.motion_data  = {"meas_lin_vel": 0.0, "meas_ang_vel": 0.0}
        self.targ_lin_vel = 0.0
        self.targ_ang_vel = 0.0
        self.pose         = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.prev_ts      = self.get_clock().now()
        self.curr_ts      = self.get_clock().now()
        self.set_vel_ts   = self.get_clock().now()
        self.GROUND_CLEARANCE = 0.0375

        # ── Arm state ─────────────────────────────────────────
        # Tracked in actual pulse counts (same units as Pico firmware)
        self.steps_ver  = VER_HOME    # current vertical pulse count
        self.steps_hor  = HOR_HOME    # current horizontal pulse count
        self.target_ver = VER_HOME    # commanded target
        self.target_hor = HOR_HOME

        self.step_dir   = 0   # sent to Pico each tick: -1/0/1
        self.step2_dir  = 0

        self._state_tick = 0

        self.get_logger().info(
            "---\nOcto pilot up.\n"
            f"Vertical stepper:   home={VER_HOME}  down={VER_DOWN}\n"
            f"Horizontal stepper: home={HOR_HOME}  closed={HOR_CLOSED}\n"
            "Accepts: JSON {\"steps_ver\": N, \"steps_hor\": N} "
            "or legacy GRAB/RELEASE\n---"
        )

    # ══════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════

    def _vel_cb(self, msg: Twist):
        self.set_vel_ts   = self.get_clock().now()
        self.targ_lin_vel = msg.linear.x
        self.targ_ang_vel = msg.angular.z

    def _arm_cmd_cb(self, msg: String):
        raw = msg.data.strip()

        # ── JSON format (from sorting_master) ─────────────────
        try:
            data = json.loads(raw)
            ver = int(data.get("steps_ver", self.target_ver))
            hor = int(data.get("steps_hor", self.target_hor))
            self.target_ver = max(VER_MIN, min(VER_MAX, ver))
            self.target_hor = max(HOR_MIN, min(HOR_MAX, hor))
            self.get_logger().info(
                f"ARM cmd: ver→{self.target_ver}  hor→{self.target_hor}"
            )
            return
        except (json.JSONDecodeError, ValueError, KeyError):
            pass

        # ── Legacy text fallback ──────────────────────────────
        cmd = raw.upper()
        if cmd == "GRAB":
            self.target_ver = VER_DOWN
            self.target_hor = HOR_CLOSED
            self.get_logger().info(
                f"ARM GRAB → ver={VER_DOWN}, hor={HOR_CLOSED}"
            )
        elif cmd == "RELEASE":
            self.target_ver = VER_HOME
            self.target_hor = HOR_HOME
            self.get_logger().info(
                f"ARM RELEASE → ver={VER_HOME}, hor={HOR_HOME}"
            )
        else:
            self.get_logger().warn(f"ARM: unknown command '{raw}'")

    # ══════════════════════════════════════════════════════════
    # Arm motion: advance toward targets by COUNTS_PER_TICK
    # ══════════════════════════════════════════════════════════

    def _at_ver_target(self) -> bool:
        return abs(self.steps_ver - self.target_ver) <= VER_TOLERANCE

    def _at_hor_target(self) -> bool:
        return abs(self.steps_hor - self.target_hor) <= HOR_TOLERANCE

    def _update_arm(self):
        # ── Vertical stepper ──────────────────────────────────
        if not self._at_ver_target():
            remaining = self.target_ver - self.steps_ver
            move = min(COUNTS_PER_TICK, abs(remaining))
            if remaining > 0:
                self.step_dir   =  1        # lower
                self.steps_ver += move
            else:
                self.step_dir   = -1        # raise
                self.steps_ver -= move
        else:
            self.step_dir = 0

        # ── Horizontal stepper ────────────────────────────────
        # In sequential mode: wait for vertical to finish first.
        # This ensures the arm is fully lowered before gripping,
        # and fully raised before opening at the bucket.
        if SEQUENTIAL_MOTION and not self._at_ver_target():
            self.step2_dir = 0
            return

        if not self._at_hor_target():
            remaining = self.target_hor - self.steps_hor
            move = min(COUNTS_PER_TICK, abs(remaining))
            if remaining > 0:
                self.step2_dir   =  1       # close
                self.steps_hor  += move
            else:
                self.step2_dir   = -1       # open
                self.steps_hor  -= move
        else:
            self.step2_dir = 0

    # ══════════════════════════════════════════════════════════
    # Main Pico tick
    # ══════════════════════════════════════════════════════════

    def _tick(self):
        self._update_arm()

        # Send to Pico
        self.pico_msngr.write((
            f"{self.targ_lin_vel:.3f},"
            f"{self.targ_ang_vel:.3f},"
            f"{self.step_dir},"
            f"{self.step2_dir}\n"
        ).encode("utf-8"))

        # Receive odometry
        if self.pico_msngr.inWaiting() > 0:
            raw = self.pico_msngr.readline().decode("utf-8", "ignore").strip()
            if raw:
                parts = raw.split(",")
                if len(parts) == 2:
                    try:
                        self.motion_data["meas_lin_vel"] = float(parts[0])
                        self.motion_data["meas_ang_vel"] = float(parts[1])
                    except ValueError:
                        pass

        # Integrate pose
        self.curr_ts = self.get_clock().now()
        dt = (self.curr_ts - self.prev_ts).nanoseconds * 1e-9
        self.pose["x"]    += self.motion_data["meas_lin_vel"] * cos(self.pose["theta"]) * dt
        self.pose["y"]    += self.motion_data["meas_lin_vel"] * sin(self.pose["theta"]) * dt
        self.pose["theta"] = atan2(
            sin(self.pose["theta"] + self.motion_data["meas_ang_vel"] * dt),
            cos(self.pose["theta"] + self.motion_data["meas_ang_vel"] * dt),
        )
        quat = quaternion_about_axis(self.pose["theta"], (0, 0, 1))
        self.prev_ts = self.curr_ts

        # Publish /odom
        odom = Odometry()
        odom.header.stamp            = self.curr_ts.to_msg()
        odom.header.frame_id         = "odom"
        odom.child_frame_id          = "base_link"
        odom.pose.pose.position.x    = self.pose["x"]
        odom.pose.pose.position.y    = self.pose["y"]
        odom.pose.pose.position.z    = self.GROUND_CLEARANCE
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x    = self.motion_data["meas_lin_vel"]
        odom.twist.twist.angular.z   = self.motion_data["meas_ang_vel"]
        self.odom_pub.publish(odom)

        # Publish /arm_state at ~7 Hz
        self._state_tick += 1
        if self._state_tick >= ARM_STATE_EVERY:
            self._state_tick = 0
            s      = String()
            s.data = json.dumps({
                "steps_ver":  self.steps_ver,
                "steps_hor":  self.steps_hor,
                "target_ver": self.target_ver,
                "target_hor": self.target_hor,
                "moving": (
                    not self._at_ver_target() or
                    not self._at_hor_target()
                ),
            })
            self.arm_state_pub.publish(s)

        # Safety: clear drive commands after 0.5s silence
        if (self.curr_ts - self.set_vel_ts).nanoseconds > 500_000_000:
            self.targ_lin_vel = 0.0
            self.targ_ang_vel = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = OctoPilot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
