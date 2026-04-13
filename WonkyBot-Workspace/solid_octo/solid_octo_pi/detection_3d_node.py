#!/usr/bin/env python3
"""
detection_3d_node.py
Subscribes to /detected_objects (from detector_node) and the depth image.
Deprojects each detection's (cx, cy) pixel to a 3D point in camera frame,
then transforms to odom frame using the robot's pose.
Publishes enriched detections on /detected_objects_3d.

CAMERA MOUNT (must match your physical robot):
  base_link → camera_link: x=0.43m forward, z=0.10m up, no rotation
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from nav_msgs.msg import Odometry


# ── Camera mount: base_link → camera_link ────────────────────
# Update these if you physically move the camera.
CAM_X = 0.43   # meters forward from base_link origin
CAM_Y = 0.00
CAM_Z = 0.10   # meters up from base_link origin
# Camera faces forward, no rotation — identity rotation matrix
R_CAM_TO_BASE = np.eye(3)
T_CAM_TO_BASE = np.array([CAM_X, CAM_Y, CAM_Z])

# ── Detector node output resolution ──────────────────────────
DETECTOR_W = 480
DETECTOR_H = 360


def transform_cam_to_odom(point_cam: np.ndarray, robot_pose: tuple) -> np.ndarray:
    """
    Transform a 3D point from camera frame to odom frame.
    robot_pose: (x, y, theta) from /odometry/filtered
    """
    # Camera → base_link (static)
    point_base = R_CAM_TO_BASE @ point_cam + T_CAM_TO_BASE

    # base_link → odom (dynamic, from odometry)
    x, y, theta = robot_pose
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    R_base_odom = np.array([
        [cos_t, -sin_t, 0.0],
        [sin_t,  cos_t, 0.0],
        [0.0,    0.0,   1.0],
    ])
    t_base_odom = np.array([x, y, 0.0])
    return R_base_odom @ point_base + t_base_odom


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def deproject_pixel(cx: float, cy: float, depth_m: float, fx: float, fy: float,
                    ppx: float, ppy: float) -> np.ndarray:
    """Backproject a 2D pixel + depth to a 3D ray in camera frame."""
    x = (cx - ppx) / fx * depth_m
    y = (cy - ppy) / fy * depth_m
    z = depth_m
    return np.array([x, y, z])


class Detection3DNode(Node):

    def __init__(self):
        super().__init__("detection_3d_node")

        self.bridge       = CvBridge()
        self.depth_frame  = None
        self.intrinsics   = None          # set on first CameraInfo message
        self.robot_pose   = (0.0, 0.0, 0.0)   # (x, y, theta) in odom

        # Publishers
        self.pub_3d = self.create_publisher(String, "/detected_objects_3d", 10)

        # Subscribers
        self.create_subscription(
            String, "/detected_objects",
            self._detections_cb, 10)
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info",
            self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/odometry/filtered",
            self._odom_cb, 10)

        self.get_logger().info(
            f"detection_3d_node ready — "
            f"camera mount: x={CAM_X}m forward, z={CAM_Z}m up"
        )

    # ── Store latest depth frame ──────────────────────────────
    def _depth_cb(self, msg: Image):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
        except Exception:
            pass

    # ── Latch camera intrinsics (only need once) ──────────────
    def _camera_info_cb(self, msg: CameraInfo):
        if self.intrinsics is not None:
            return   # already have them
        self.intrinsics = {
            "fx":  msg.k[0],
            "fy":  msg.k[4],
            "ppx": msg.k[2],
            "ppy": msg.k[5],
            "w":   msg.width,
            "h":   msg.height,
        }
        self.get_logger().info(
            f"Intrinsics: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} "
            f"ppx={msg.k[2]:.1f} ppy={msg.k[5]:.1f} "
            f"res={msg.width}x{msg.height}"
        )

    # ── Track robot pose ──────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.robot_pose = (pos.x, pos.y, yaw)

    # ── Main: enrich 2D detections with 3D positions ──────────
    def _detections_cb(self, msg: String):
        if self.depth_frame is None or self.intrinsics is None:
            return

        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if not detections:
            return

        depth_h, depth_w = self.depth_frame.shape[:2]
        fx  = self.intrinsics["fx"]
        fy  = self.intrinsics["fy"]
        ppx = self.intrinsics["ppx"]
        ppy = self.intrinsics["ppy"]

        # Scale pixel coords from detector resolution to full depth resolution
        scale_x = depth_w / DETECTOR_W
        scale_y = depth_h / DETECTOR_H

        enriched = []
        for det in detections:
            cx_full = int(det["cx"] * scale_x)
            cy_full = int(det["cy"] * scale_y)

            # Median depth over 13x13 patch to reduce noise
            r = 6
            patch = self.depth_frame[
                max(cy_full - r, 0): cy_full + r + 1,
                max(cx_full - r, 0): cx_full + r + 1,
            ]
            valid = patch[patch > 0].astype(float)
            if len(valid) == 0:
                continue
            depth_m = float(np.median(valid)) / 1000.0

            # Filter bogus readings
            if not (0.10 < depth_m < 2.5):
                continue

            # Deproject pixel → 3D camera frame
            point_cam = deproject_pixel(cx_full, cy_full, depth_m, fx, fy, ppx, ppy)

            # Transform camera → odom
            point_odom = transform_cam_to_odom(point_cam, self.robot_pose)

            enriched.append({
                "color":    det["color"],
                "type":     det["type"],
                "depth_m":  round(depth_m, 3),
                "cam_xyz":  [round(float(v), 3) for v in point_cam],
                "odom_xyz": [round(float(v), 3) for v in point_odom],
                # Pass through 2D fields so downstream nodes can still use them
                "cx":       det["cx"],
                "cy":       det["cy"],
                "area":     det.get("area", 0),
            })

        if enriched:
            out      = String()
            out.data = json.dumps(enriched)
            self.pub_3d.publish(out)
            summary = [
                (d["color"], d["type"],
                 f"odom=({d['odom_xyz'][0]:.2f},{d['odom_xyz'][1]:.2f})",
                 f"d={d['depth_m']:.2f}m")
                for d in enriched
            ]
            self.get_logger().info(str(summary), throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = Detection3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
