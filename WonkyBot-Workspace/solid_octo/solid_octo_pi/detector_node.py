#!/usr/bin/env python3
"""
detector_node.py
Subscribes to the RealSense D455 color + depth images.
Publishes detected balls/buckets as a JSON string on /detected_objects.
Each detection includes depth_m so sorting_master doesn't need /scan.

Shape classification uses a 4-feature voting system:
  1. Circularity   (weight 2) — balls score > 0.60
  2. Aspect ratio  (weight 1) — balls are roughly square (w ≈ h)
  3. Extent        (weight 1) — balls fill their bounding box well
  4. Straight lines (weight 2) — buckets have strong rectangular edges

Score > 0 → BALL.  Score <= 0 → BUCKET.
Default when evidence is weak is always BALL.
"""

import json
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from collections import deque

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ── Config ────────────────────────────────────────────────────
COLORS_JSON = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "colors.json"
)
FRAME_W   = 480
FRAME_H   = 360
MIN_AREA  = 800       # minimum contour area in pixels
MAX_DEPTH = 2.0       # ignore detections farther than this (meters)
KERNEL    = np.ones((5, 5), np.uint8)

# ── Ball thresholds ───────────────────────────────────────────
BALL_MIN_CIRCULARITY  = 0.60   # balls are very round
BALL_MAX_ASPECT_DELTA = 0.35   # w/h must be within 0.35 of 1.0
BALL_MIN_EXTENT       = 0.60   # contour fills ≥ 60% of bounding box

# ── Bucket thresholds ─────────────────────────────────────────
BUCKET_MAX_CIRCULARITY = 0.75  # buckets are less circular than balls
BUCKET_ASPECT_HI       = 1.30  # clearly wider than tall
BUCKET_ASPECT_LO       = 0.70  # clearly taller than wide
BUCKET_MIN_LINES       = 2     # must have ≥ 2 strong straight edges

# ── Voting weights ────────────────────────────────────────────
WEIGHT_CIRCULARITY = 2
WEIGHT_ASPECT      = 1
WEIGHT_EXTENT      = 1
WEIGHT_LINES       = 2


# ══════════════════════════════════════════════════════════════
# Color helpers  (unchanged from original)
# ══════════════════════════════════════════════════════════════

def load_colors(path: str) -> dict:
    with open(path) as f:
        raw = json.load(f)
    out = {}
    for name, v in raw.items():
        out[name] = {
            "lower":  np.array(v["lower"],  dtype=np.uint8),
            "upper":  np.array(v["upper"],  dtype=np.uint8),
            "bgr":    tuple(v["bgr"]),
            "lower2": np.array(v["lower2"], dtype=np.uint8) if "lower2" in v else None,
            "upper2": np.array(v["upper2"], dtype=np.uint8) if "upper2" in v else None,
        }
    return out


def build_mask(hsv: np.ndarray, color: dict) -> np.ndarray:
    mask = cv2.inRange(hsv, color["lower"], color["upper"])
    if color["lower2"] is not None:
        mask = cv2.bitwise_or(
            mask, cv2.inRange(hsv, color["lower2"], color["upper2"])
        )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    return mask


# ══════════════════════════════════════════════════════════════
# Shape feature helpers
# ══════════════════════════════════════════════════════════════

def _circularity(cnt) -> float:
    area = cv2.contourArea(cnt)
    perim = cv2.arcLength(cnt, True)
    if perim == 0:
        return 0.0
    return (4 * np.pi * area) / (perim ** 2)


def _aspect_ratio(bbox) -> float:
    """w / h — 1.0 = square, <1 = taller, >1 = wider."""
    x, y, w, h = bbox
    return float(w) / h if h > 0 else 1.0


def _extent(cnt, bbox) -> float:
    """Fraction of bounding-box area covered by the contour."""
    x, y, w, h = bbox
    bb_area = w * h
    if bb_area == 0:
        return 0.0
    return cv2.contourArea(cnt) / bb_area


def _count_lines(cnt, gray) -> int:
    """Count Hough straight lines inside the contour's bounding region."""
    x, y, w, h = cv2.boundingRect(cnt)
    pad = 6
    x1 = max(x - pad, 0);  y1 = max(y - pad, 0)
    x2 = min(x + w + pad, gray.shape[1])
    y2 = min(y + h + pad, gray.shape[0])
    roi = gray[y1:y2, x1:x2]
    if roi.size == 0:
        return 0
    edges = cv2.Canny(roi, 50, 150)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180,
        threshold=25,
        minLineLength=max(8, int(min(w, h) * 0.20)),
        maxLineGap=10,
    )
    return len(lines) if lines is not None else 0


# ══════════════════════════════════════════════════════════════
# Classifier  (replaces the old single aspect-ratio check)
# ══════════════════════════════════════════════════════════════

def classify_contour(cnt, gray):
    """
    Returns (obj_type, bbox, features) or (None, None, None) if too small.

    Each feature votes +1 (ball) or -1 (bucket) with its weight.
    Final score > 0 → BALL, <= 0 → BUCKET.
    Weak evidence defaults to BALL as requested.
    """
    area = cv2.contourArea(cnt)
    perim = cv2.arcLength(cnt, True)
    if area < MIN_AREA or perim == 0:
        return None, None, None

    bbox         = cv2.boundingRect(cnt)
    circularity  = _circularity(cnt)
    aspect_ratio = _aspect_ratio(bbox)
    extent       = _extent(cnt, bbox)
    n_lines      = _count_lines(cnt, gray)

    score = 0

    # ── Feature 1: circularity ────────────────────────────────
    if circularity >= BALL_MIN_CIRCULARITY:
        score += WEIGHT_CIRCULARITY          # strongly ball-like
    elif circularity <= BUCKET_MAX_CIRCULARITY * 0.85:
        score -= WEIGHT_CIRCULARITY          # strongly bucket-like

    # ── Feature 2: aspect ratio ───────────────────────────────
    if abs(aspect_ratio - 1.0) <= BALL_MAX_ASPECT_DELTA:
        score += WEIGHT_ASPECT               # roughly square → ball
    elif aspect_ratio >= BUCKET_ASPECT_HI or aspect_ratio <= BUCKET_ASPECT_LO:
        score -= WEIGHT_ASPECT               # clearly not square → bucket

    # ── Feature 3: extent ─────────────────────────────────────
    if extent >= BALL_MIN_EXTENT:
        score += WEIGHT_EXTENT
    elif extent < 0.50:
        score -= WEIGHT_EXTENT

    # ── Feature 4: straight lines ─────────────────────────────
    if n_lines >= BUCKET_MIN_LINES:
        score -= WEIGHT_LINES                # rectangular edges → bucket
    else:
        score += WEIGHT_LINES                # no straight edges → ball

    obj_type = "BALL" if score > 0 else "BUCKET"

    features = {
        "circularity":  round(circularity, 3),
        "aspect_ratio": round(aspect_ratio, 3),
        "extent":       round(extent, 3),
        "n_lines":      n_lines,
        "score":        score,
    }

    return obj_type, bbox, features


# ══════════════════════════════════════════════════════════════
# ROS2 Node  (structure identical to original)
# ══════════════════════════════════════════════════════════════

class DetectorNode(Node):

    def __init__(self):
        super().__init__("detector_node")

        self.bridge          = CvBridge()
        self.depth_frame     = None
        self.color_frame_np  = None
        self.colors          = load_colors(COLORS_JSON)

        # Publishers
        self.pub_objects = self.create_publisher(String, "/detected_objects", 10)
        self.pub_debug   = self.create_publisher(Image,  "/detector/debug_image", 10)

        # QoS: keep only latest frame
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # RealSense subscriptions
        self.create_subscription(
            Image, "/camera/camera/color/image_raw",
            self._store_color, sensor_qos)
        self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw",
            self._store_depth, sensor_qos)

        self.frame_count = 0
        self.create_timer(0.15, self.process_frame)   # ~7 Hz

        self.get_logger().info(
            f"detector_node ready — colors: {list(self.colors.keys())}"
        )

    # ── Frame storage callbacks ───────────────────────────────
    def _store_color(self, msg: Image):
        try:
            self.color_frame_np = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _store_depth(self, msg: Image):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
        except Exception:
            pass

    # ── Timer-driven processing ───────────────────────────────
    def process_frame(self):
        if self.color_frame_np is None:
            return
        raw = self.color_frame_np
        self.color_frame_np = None   # consume

        self.frame_count += 1

        try:
            frame       = cv2.resize(raw, (FRAME_W, FRAME_H))
            depth_small = (
                cv2.resize(self.depth_frame, (FRAME_W, FRAME_H),
                           interpolation=cv2.INTER_NEAREST)
                if self.depth_frame is not None else None
            )

            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            detections = []

            for color_name, color_data in self.colors.items():
                mask        = build_mask(hsv, color_data)
                contours, _ = cv2.findContours(
                    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                for cnt in contours:
                    obj_type, bbox, features = classify_contour(cnt, gray)
                    if obj_type is None:
                        continue

                    x, y, w, h = bbox
                    M  = cv2.moments(cnt)
                    cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else x + w // 2
                    cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else y + h // 2

                    # Depth: 9x9 median patch around centre
                    if depth_small is not None:
                        r     = 4
                        patch = depth_small[
                            max(cy - r, 0):cy + r + 1,
                            max(cx - r, 0):cx + r + 1
                        ]
                        valid   = patch[patch > 0].astype(float)
                        depth_m = float(np.median(valid)) / 1000.0 if len(valid) > 0 else 0.0
                    else:
                        depth_m = 0.0

                    if depth_m > MAX_DEPTH:
                        continue

                    detections.append({
                        "color":        color_name,
                        "type":         obj_type,
                        "cx":           cx,
                        "cy":           cy,
                        "area":         int(cv2.contourArea(cnt)),
                        "circularity":  features["circularity"],
                        "aspect_ratio": features["aspect_ratio"],
                        "extent":       features["extent"],
                        "n_lines":      features["n_lines"],
                        "score":        features["score"],
                        "depth_m":      round(depth_m, 3),
                    })

                    # Debug overlay — circle for balls, rectangle for buckets
                    color_bgr = color_data["bgr"]
                    if obj_type == "BALL":
                        radius = (w + h) // 4
                        cv2.circle(frame, (cx, cy), radius, color_bgr, 2)
                    else:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)

                    cv2.putText(
                        frame,
                        f"{color_name} {obj_type} {depth_m:.2f}m s={features['score']:+d}",
                        (x, max(y - 8, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, color_bgr, 1, cv2.LINE_AA,
                    )

            msg_out      = String()
            msg_out.data = json.dumps(detections)
            self.pub_objects.publish(msg_out)

            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            if detections:
                summary = [
                    (d["color"], d["type"], f"{d['depth_m']}m", f"s={d['score']:+d}")
                    for d in detections
                ]
                self.get_logger().info(str(summary), throttle_duration_sec=1.0)

        except Exception as e:
            import traceback
            self.get_logger().error(
                f"Detection error: {e}\n{traceback.format_exc()}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
