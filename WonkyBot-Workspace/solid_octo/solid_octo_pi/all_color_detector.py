"""
all_color_detector.py
Extends professor's green_detector.py to track all 4 colors.
Minimal changes — same QoS, same callback pattern, same deprojection.
Publishes closest large blob of any color to /goal_in_odom.

Shape classification uses a 4-feature voting system instead of
aspect ratio alone, so buckets are no longer misclassified as balls.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
from tf2_geometry_msgs import PointStamped

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

import cv2 as cv
import numpy as np

# ── HSV color ranges ──────────────────────────────────────────
COLORS = {
    "RED":    {"lower": [0,   120,  70], "upper": [10,  255, 255],
               "lower2":[170, 120,  70], "upper2":[179, 255, 255], "draw": (0, 0, 255)},
    "GREEN":  {"lower": [35,  40,   40], "upper": [85,  255, 255], "draw": (0, 255, 0)},
    "BLUE":   {"lower": [100, 72,   94], "upper": [110, 255, 255], "draw": (255, 0, 0)},
    "YELLOW": {"lower": [18,  38,   72], "upper": [31,  255, 255], "draw": (0, 255, 255)},
}

MIN_AREA  = 4000   # minimum contour area at 1280x720
MAX_DEPTH = 2.0    # ignore anything further than 2 meters

# ── Shape classifier thresholds ───────────────────────────────
# Balls
BALL_MIN_CIRCULARITY  = 0.60   # balls are very round
BALL_MAX_ASPECT_DELTA = 0.35   # w/h must be within 0.35 of 1.0
BALL_MIN_EXTENT       = 0.60   # contour fills ≥ 60 % of bounding box

# Buckets
BUCKET_MAX_CIRCULARITY = 0.75  # buckets are less circular
BUCKET_ASPECT_HI       = 1.30  # clearly wider than tall
BUCKET_ASPECT_LO       = 0.70  # clearly taller than wide
BUCKET_MIN_LINES       = 2     # must have ≥ 2 strong straight edges

# Voting weights — positive score = BALL, zero or negative = BUCKET
WEIGHT_CIRCULARITY = 2
WEIGHT_ASPECT      = 1
WEIGHT_EXTENT      = 1
WEIGHT_LINES       = 2


# ══════════════════════════════════════════════════════════════
# Shape feature helpers
# ══════════════════════════════════════════════════════════════

def _circularity(cnt) -> float:
    area  = cv.contourArea(cnt)
    perim = cv.arcLength(cnt, True)
    if perim == 0:
        return 0.0
    return (4 * np.pi * area) / (perim ** 2)


def _aspect_ratio(bbox) -> float:
    x, y, w, h = bbox
    return float(w) / h if h > 0 else 1.0


def _extent(cnt, bbox) -> float:
    x, y, w, h = bbox
    bb = w * h
    return cv.contourArea(cnt) / bb if bb > 0 else 0.0


def _count_lines(cnt, gray) -> int:
    x, y, w, h = cv.boundingRect(cnt)
    pad = 6
    x1 = max(x - pad, 0);  y1 = max(y - pad, 0)
    x2 = min(x + w + pad, gray.shape[1])
    y2 = min(y + h + pad, gray.shape[0])
    roi = gray[y1:y2, x1:x2]
    if roi.size == 0:
        return 0
    edges = cv.Canny(roi, 50, 150)
    lines = cv.HoughLinesP(
        edges, 1, np.pi / 180,
        threshold=25,
        minLineLength=max(8, int(min(w, h) * 0.20)),
        maxLineGap=10,
    )
    return len(lines) if lines is not None else 0


def classify_shape(cnt, gray) -> tuple:
    """
    Returns (label, score, features) where label is "BALL" or "BUCKET".

    Four features vote independently:
      circularity  (weight 2) — balls are rounder
      aspect ratio (weight 1) — balls are roughly square
      extent       (weight 1) — balls fill their bbox more
      straight lines (weight 2) — buckets have rectangular edges

    score > 0 → BALL.  score <= 0 → BUCKET.
    Default when evidence is weak: BALL.
    """
    bbox         = cv.boundingRect(cnt)
    circ         = _circularity(cnt)
    aspect       = _aspect_ratio(bbox)
    extent       = _extent(cnt, bbox)
    n_lines      = _count_lines(cnt, gray)

    score = 0

    # Circularity
    if circ >= BALL_MIN_CIRCULARITY:
        score += WEIGHT_CIRCULARITY
    elif circ <= BUCKET_MAX_CIRCULARITY * 0.85:
        score -= WEIGHT_CIRCULARITY

    # Aspect ratio
    if abs(aspect - 1.0) <= BALL_MAX_ASPECT_DELTA:
        score += WEIGHT_ASPECT
    elif aspect >= BUCKET_ASPECT_HI or aspect <= BUCKET_ASPECT_LO:
        score -= WEIGHT_ASPECT

    # Extent
    if extent >= BALL_MIN_EXTENT:
        score += WEIGHT_EXTENT
    elif extent < 0.50:
        score -= WEIGHT_EXTENT

    # Straight lines
    if n_lines >= BUCKET_MIN_LINES:
        score -= WEIGHT_LINES      # rectangular edges → bucket
    else:
        score += WEIGHT_LINES      # no straight edges → ball

    label = "BALL" if score > 0 else "BUCKET"
    return label, score, {"circ": round(circ, 2), "aspect": round(aspect, 2),
                          "extent": round(extent, 2), "lines": n_lines}


# ══════════════════════════════════════════════════════════════
# Node
# ══════════════════════════════════════════════════════════════

class AllColorDetector(Node):

    def __init__(self):
        super().__init__("all_color_detector")

        reliable_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/aligned_depth_to_color/camera_info",
            self.get_info,
            reliable_qos,
        )
        self.color_subscriber = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.detect_colors,
            reliable_qos,
        )
        # Professor uses integer 10 for depth — keep that exactly
        self.depth_subscriber = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.get_depth_image,
            10,
        )

        self.create_subscription(String, "/detect_mode", self._mode_cb, 10)

        self.detection_publisher = self.create_publisher(
            Image, "/camera/detection", reliable_qos,
        )
        self.goal_publisher = self.create_publisher(
            PointStamped, "/goal_in_odom", reliable_qos,
        )
        self.color_publisher = self.create_publisher(
            String, "/detected_color", reliable_qos,
        )
        self.dist_publisher = self.create_publisher(
            Float32, "/ball_distance", reliable_qos,
        )

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_model        = PinholeCameraModel()
        self.depth_image         = None
        self.x_img               = None
        self.y_img               = None
        self.detected_color_name = None
        self.detect_mode         = "BALL"

        self.get_logger().info("All-color detector started. Mode: BALL")

    # ── Mode switch ───────────────────────────────────────────
    def _mode_cb(self, msg: String):
        new_mode = msg.data.strip().upper()
        if new_mode != self.detect_mode:
            self.detect_mode = new_mode
            self.x_img = None
            self.y_img = None
            self.get_logger().info(f"Detect mode -> {self.detect_mode}")

    # ── Camera info ───────────────────────────────────────────
    def get_info(self, msg):
        self.camera_model.from_camera_info(msg)

    # ── Depth + deprojection (unchanged from professor) ───────
    def get_depth_image(self, msg):
        self.depth_image = CvBridge().imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

        if self.x_img is None or self.y_img is None:
            return

        dh, dw = self.depth_image.shape[:2]
        cx, cy = self.x_img, self.y_img
        if not (0 <= cx < dw and 0 <= cy < dh):
            return

        r     = 15
        patch = self.depth_image[max(0, cy - r):min(dh, cy + r),
                                  max(0, cx - r):min(dw, cx + r)]
        valid = patch[patch > 0]
        if valid.size == 0:
            return
        z_cam = float(np.median(valid)) * 0.001

        if z_cam <= 0.05 or z_cam > MAX_DEPTH:
            return

        dist_msg      = Float32()
        dist_msg.data = float(z_cam)
        self.dist_publisher.publish(dist_msg)

        ray = self.camera_model.project_pixel_to_3d_ray((cx, cy))
        if ray[2] == 0:
            return
        scale = z_cam / ray[2]

        point_camera              = PointStamped()
        point_camera.header.frame_id = "camera_link"
        point_camera.point.x      = ray[0] * scale
        point_camera.point.y      = ray[1] * scale
        point_camera.point.z      = z_cam

        try:
            point_odom = self.tf_buffer.transform(
                point_camera, "odom", timeout=Duration(seconds=0.1)
            )
            self.goal_publisher.publish(point_odom)
            out      = String()
            out.data = self.detected_color_name or ""
            self.color_publisher.publish(out)
            self.get_logger().info(
                f"{self.detected_color_name} -> odom "
                f"({point_odom.point.x:.2f}, {point_odom.point.y:.2f}) "
                f"d={z_cam:.2f}m",
                throttle_duration_sec=0.5,
            )
        except Exception as e:
            self.get_logger().debug(f"TF2 failed: {e}")

    # ── Color + shape detection ───────────────────────────────
    def detect_colors(self, msg):
        """
        Find the best target based on current detect_mode.

        Modes:
          "BALL"       — largest ball, any color
          "BALL:RED"   — largest red ball only
          "BUCKET:RED" — largest red bucket only
          "OFF"        — publish nothing
        """
        if self.detect_mode == "OFF":
            return

        cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv    = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)
        gray   = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)

        # Parse mode
        seeking_ball  = (self.detect_mode == "BALL" or
                         self.detect_mode.startswith("BALL:"))
        ball_filter   = (self.detect_mode.split(":", 1)[1].lower()
                         if self.detect_mode.startswith("BALL:") else None)
        bucket_target = (self.detect_mode.split(":", 1)[1].upper()
                         if self.detect_mode.startswith("BUCKET:") else None)

        best_area  = 0
        best_x     = None
        best_y     = None
        best_color = None

        for name, ranges in COLORS.items():
            # Color filtering
            if ball_filter   and name.upper() != ball_filter.upper():   continue
            if bucket_target and name.upper() != bucket_target.upper(): continue

            # Build HSV mask
            mask = cv.inRange(
                hsv, np.array(ranges["lower"]), np.array(ranges["upper"])
            )
            if "lower2" in ranges:
                mask |= cv.inRange(
                    hsv, np.array(ranges["lower2"]), np.array(ranges["upper2"])
                )
            mask = cv.erode(mask,  None, iterations=2)
            mask = cv.dilate(mask, None, iterations=2)

            contours, _ = cv.findContours(
                mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
            )

            for c in contours:
                area = cv.contourArea(c)
                if area < MIN_AREA:
                    continue

                # ── Voting classifier ─────────────────────────
                label, score, feats = classify_shape(c, gray)

                x, y, w, h = cv.boundingRect(c)
                cx_c = int(x + w / 2)
                cy_c = int(y + h / 2)
                draw_col = ranges["draw"]

                # Draw: circle for balls, rectangle for buckets
                if label == "BALL":
                    radius = (w + h) // 4
                    cv.circle(cv_img, (cx_c, cy_c), radius, draw_col, 2)
                else:
                    cv.rectangle(cv_img, (x, y), (x + w, y + h), draw_col, 2)

                cv.putText(
                    cv_img,
                    f"{name} {label} s={score:+d} c={feats['circ']:.2f}",
                    (x, max(y - 8, 10)),
                    cv.FONT_HERSHEY_SIMPLEX, 0.45, draw_col, 1, cv.LINE_AA,
                )

                # Only track the shape type we're currently seeking
                want_this = (
                    (seeking_ball  and label == "BALL") or
                    (bucket_target and label == "BUCKET")
                )
                if want_this and area > best_area:
                    best_area  = area
                    best_x     = cx_c
                    best_y     = cy_c
                    best_color = name

        self.x_img               = best_x
        self.y_img               = best_y
        self.detected_color_name = best_color

        self.detection_publisher.publish(
            CvBridge().cv2_to_imgmsg(cv_img, encoding="bgr8")
        )


def main(args=None):
    rclpy.init(args=args)
    node = AllColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
