#!/usr/bin/env python3
"""Optional low-rate 2x2 visualizer for lane and cone driving.

This node is deliberately read-only.  Sensor callbacks retain only the latest
ROS message, while JPEG decoding, rectification, projection, and GUI rendering
run from a separate low-rate timer.  Starting or stopping this process cannot
change the motor command path.
"""

from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Iterable, Sequence

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path as NavPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Float32MultiArray, String

from my_rule.perception.camera_input import (
    CameraRectifier,
    decode_compressed_bgr,
)
from my_rule.perception.lidar_camera_association import project_laser_xy
from my_rule_msgs.msg import Centerline, ObjectDetectionArray


Color = tuple[int, int, int]
Point2 = tuple[float, float]

BLACK: Color = (0, 0, 0)
WHITE: Color = (255, 255, 255)
GRAY: Color = (100, 100, 100)
RED: Color = (0, 0, 255)
YELLOW: Color = (0, 255, 255)
GREEN: Color = (0, 220, 80)
CYAN: Color = (255, 220, 0)
BLUE: Color = (255, 100, 0)
MAGENTA: Color = (255, 0, 255)


def message_stamp_ns(message) -> int:
    """Return a ROS header stamp when available."""
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return 0
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def infer_drive_mode(state: str, requested_mode: str = "auto") -> str:
    """Resolve lane/cone visualization without affecting control."""
    requested = str(requested_mode).strip().lower()
    if requested in ("lane", "cone"):
        return requested
    normalized = str(state).upper()
    if (
        normalized.startswith("CONE_")
        or "CONE_SLALOM" in normalized
        or "CONE_RECOVERY" in normalized
        or "CONE_MODE=1" in normalized
    ):
        return "cone"
    return "lane"


def roi_points(
    width: int,
    height: int,
    ratios: Sequence[float],
) -> np.ndarray:
    """Return TL, TR, BR, BL source-ROI pixels."""
    if len(ratios) != 8:
        raise ValueError("ROI requires eight x/y ratios")
    values = np.asarray(ratios, dtype=np.float64).reshape(4, 2)
    scale = np.asarray((float(width), float(height)), dtype=np.float64)
    return np.rint(values * scale).astype(np.int32)


def scan_points(
    message: LaserScan | None,
    minimum_range_m: float,
    maximum_range_m: float,
) -> np.ndarray:
    """Convert the latest planar scan to finite laser-frame XY points."""
    if message is None:
        return np.empty((0, 2), dtype=np.float32)
    ranges = np.asarray(message.ranges, dtype=np.float32)
    if ranges.size == 0 or float(message.angle_increment) == 0.0:
        return np.empty((0, 2), dtype=np.float32)
    angles = (
        float(message.angle_min)
        + np.arange(ranges.size, dtype=np.float32)
        * float(message.angle_increment)
    )
    valid = (
        np.isfinite(ranges)
        & (ranges >= float(minimum_range_m))
        & (ranges <= float(maximum_range_m))
    )
    if not np.any(valid):
        return np.empty((0, 2), dtype=np.float32)
    return np.column_stack(
        (ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid]))
    ).astype(np.float32)


def rear_axle_to_laser(
    points: Iterable[Point2],
    lidar_to_rear_axle_m: float,
) -> np.ndarray:
    """Express a rear-axle path in the laser frame."""
    array = np.asarray(list(points), dtype=np.float64).reshape(-1, 2)
    if array.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    result = array.copy()
    result[:, 0] -= float(lidar_to_rear_axle_m)
    return result


def metric_to_topdown_pixels(
    points: Iterable[Point2],
    width: int,
    height: int,
    forward_range_m: float,
    lateral_range_m: float,
) -> np.ndarray:
    """Map vehicle coordinates to a forward-up top-down image."""
    array = np.asarray(list(points), dtype=np.float64).reshape(-1, 2)
    if array.size == 0:
        return np.empty((0, 2), dtype=np.int32)
    columns = width * 0.5 - array[:, 1] * width / float(lateral_range_m)
    rows = height - 1 - array[:, 0] * (height - 1) / float(forward_range_m)
    return np.rint(np.column_stack((columns, rows))).astype(np.int32)


def canonical_pixels(
    points: Iterable[Point2],
    width: int,
    height: int,
    forward_range_m: float,
    lateral_range_m: float,
) -> np.ndarray:
    """Map canonical forward/lateral coordinates to canonical pixels."""
    return metric_to_topdown_pixels(
        points,
        width,
        height,
        forward_range_m,
        lateral_range_m,
    )


def polyline_in_bounds(
    image: np.ndarray,
    pixels: np.ndarray,
    color: Color,
    thickness: int,
) -> None:
    """Draw only finite in-frame runs to avoid long clipping artifacts."""
    if pixels.shape[0] < 2:
        return
    height, width = image.shape[:2]
    valid = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < height)
    )
    start = None
    for index, usable in enumerate(valid):
        if usable and start is None:
            start = index
        at_end = index == len(valid) - 1
        if start is not None and ((not usable) or at_end):
            stop = index + 1 if usable and at_end else index
            segment = pixels[start:stop]
            if segment.shape[0] >= 2:
                cv2.polylines(
                    image,
                    [segment.astype(np.int32)],
                    False,
                    color,
                    int(thickness),
                    cv2.LINE_AA,
                )
            start = None


def resize_contain(image: np.ndarray, width: int, height: int) -> np.ndarray:
    """Resize an image without cropping and center it in one panel body."""
    if image is None or image.size == 0:
        return np.zeros((height, width, 3), dtype=np.uint8)
    source_height, source_width = image.shape[:2]
    scale = min(width / float(source_width), height / float(source_height))
    resized = cv2.resize(
        image,
        (
            max(1, int(round(source_width * scale))),
            max(1, int(round(source_height * scale))),
        ),
        interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR,
    )
    output = np.zeros((height, width, 3), dtype=np.uint8)
    offset_x = max(0, (width - resized.shape[1]) // 2)
    offset_y = max(0, (height - resized.shape[0]) // 2)
    output[
        offset_y : offset_y + resized.shape[0],
        offset_x : offset_x + resized.shape[1],
    ] = resized
    return output


def make_panel(
    body: np.ndarray,
    title: str,
    panel_width: int,
    panel_height: int,
    status: str = "",
) -> np.ndarray:
    """Build one labeled panel."""
    title_height = 30
    body_height = panel_height - title_height
    canvas = np.full((panel_height, panel_width, 3), 16, dtype=np.uint8)
    canvas[title_height:] = resize_contain(body, panel_width, body_height)
    cv2.putText(
        canvas,
        title,
        (10, 21),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.56,
        WHITE,
        1,
        cv2.LINE_AA,
    )
    if status:
        text_size = cv2.getTextSize(
            status, cv2.FONT_HERSHEY_SIMPLEX, 0.43, 1
        )[0]
        cv2.putText(
            canvas,
            status,
            (max(10, panel_width - text_size[0] - 10), 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            CYAN,
            1,
            cv2.LINE_AA,
        )
    return canvas


class PerceptionViewNode(Node):
    """Render lane and cone perception in an optional external GUI process."""

    def __init__(self) -> None:
        super().__init__("my_rule_perception_view")
        perception_share = Path(get_package_share_directory("xycar_perception"))

        defaults = {
            "mode": "auto",
            "view_rate_hz": 5.0,
            "panel_width": 640,
            "panel_height": 360,
            "window_name": "my_rule perception | lane + cone",
            "show_window": True,
            "camera_topic": "/wide_camera_mjpeg/image_raw/compressed",
            "scan_topic": "/scan",
            "canonical_topic": "/perception/canonical_road_image",
            "canonical_white_topic": "/perception/canonical_white_mask",
            "canonical_yellow_topic": "/perception/canonical_yellow_mask",
            "lane_path_topic": "/my_rule/connected_lane_path",
            "cone_path_topic": "/my_rule/cone_path",
            "cone_cluster_topic": "/my_rule/cone_clusters",
            "state_topic": "/my_rule/state",
            "motor_topic": "/xycar_motor",
            "yolo_detection_topic": "/my_rule/object_detections",
            "show_yolo_boxes": True,
            "camera_timeout_sec": 1.0,
            "data_timeout_sec": 1.0,
            "camera_yaml": str(
                perception_share / "config" / "wide_camera_fisheye_1280x1024.yaml"
            ),
            "lidar_camera_extrinsic_yaml": str(
                perception_share
                / "config"
                / "lidar_camera_extrinsic_measured.yaml"
            ),
            "rect_balance": 0.3,
            "src_tl_x_ratio": 0.36875,
            "src_tl_y_ratio": 0.482421875,
            "src_tr_x_ratio": 0.7078125,
            "src_tr_y_ratio": 0.474609375,
            "src_br_x_ratio": 0.99375,
            "src_br_y_ratio": 0.59765625,
            "src_bl_x_ratio": 0.0359375,
            "src_bl_y_ratio": 0.607421875,
            "canonical_lateral_range_m": 1.4,
            "canonical_forward_range_m": 1.5,
            "lidar_to_rear_axle_m": 0.42,
            "lidar_projection_min_range_m": 0.18,
            "lidar_projection_max_range_m": 3.0,
            "topdown_forward_range_m": 2.5,
            "topdown_lateral_range_m": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.mode_request = str(self.get_parameter("mode").value)
        self.panel_width = max(320, int(self.get_parameter("panel_width").value))
        self.panel_height = max(220, int(self.get_parameter("panel_height").value))
        self.window_name = str(self.get_parameter("window_name").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        graphical_session = os.environ.get("DISPLAY") or os.environ.get(
            "WAYLAND_DISPLAY"
        )
        if self.show_window and not graphical_session:
            self.get_logger().warning(
                "no graphical display detected; viewer subscriptions remain active "
                "but the window is disabled"
            )
            self.show_window = False

        self.bridge = CvBridge()
        self.rectifier = CameraRectifier(
            str(self.get_parameter("camera_yaml").value),
            float(self.get_parameter("rect_balance").value),
        )
        self.rotation_camera_laser, self.translation_camera_laser = (
            self.load_extrinsic(
                str(self.get_parameter("lidar_camera_extrinsic_yaml").value)
            )
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("camera_topic").value),
            self.on_camera,
            sensor_qos,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.on_scan,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("canonical_topic").value),
            self.on_canonical,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("canonical_white_topic").value),
            self.on_canonical_white,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("canonical_yellow_topic").value),
            self.on_canonical_yellow,
            sensor_qos,
        )
        self.create_subscription(
            Centerline,
            str(self.get_parameter("lane_path_topic").value),
            self.on_lane_path,
            10,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter("cone_path_topic").value),
            self.on_cone_path,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("cone_cluster_topic").value),
            self.on_cone_clusters,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("state_topic").value),
            self.on_state,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("motor_topic").value),
            self.on_motor,
            10,
        )
        self.yolo_subscription = None
        if bool(self.get_parameter("show_yolo_boxes").value):
            self.yolo_subscription = self.create_subscription(
                ObjectDetectionArray,
                str(self.get_parameter("yolo_detection_topic").value),
                self.on_yolo,
                sensor_qos,
            )

        self.latest_camera = None
        self.latest_scan = None
        self.latest_canonical = None
        self.latest_white = None
        self.latest_yellow = None
        self.latest_lane_path = None
        self.latest_cone_path = None
        self.latest_cone_clusters = None
        self.latest_yolo = None
        self.latest_state = ""
        self.latest_motor = (0.0, 0.0)
        self.arrival_times: dict[str, float] = {}
        self.cached_camera_stamp = None
        self.cached_camera = None
        self.cached_canonical_stamp = None
        self.cached_canonical = None
        self.window_created = False

        rate = max(0.5, float(self.get_parameter("view_rate_hz").value))
        self.create_timer(1.0 / rate, self.render)
        yolo_text = "enabled" if self.yolo_subscription is not None else "standby"
        self.get_logger().info(
            f"optional 2x2 viewer ready: rate={rate:.1f}Hz, mode={self.mode_request}, "
            f"YOLO boxes={yolo_text}; this node publishes no control topics"
        )

    @staticmethod
    def load_extrinsic(path_text: str) -> tuple[np.ndarray, np.ndarray]:
        path = Path(path_text).expanduser().resolve()
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        transform = data["T_camera_lidar"]
        rotation = np.asarray(
            transform["R_row_major"], dtype=np.float64
        ).reshape(3, 3)
        translation = np.asarray(transform["t_xyz"], dtype=np.float64).reshape(3)
        return rotation, translation

    def remember(self, key: str, message) -> None:
        setattr(self, f"latest_{key}", message)
        self.arrival_times[key] = time.monotonic()

    def on_camera(self, message: CompressedImage) -> None:
        self.remember("camera", message)

    def on_scan(self, message: LaserScan) -> None:
        self.remember("scan", message)

    def on_canonical(self, message: Image) -> None:
        self.remember("canonical", message)

    def on_canonical_white(self, message: Image) -> None:
        self.remember("white", message)

    def on_canonical_yellow(self, message: Image) -> None:
        self.remember("yellow", message)

    def on_lane_path(self, message: Centerline) -> None:
        self.remember("lane_path", message)

    def on_cone_path(self, message: NavPath) -> None:
        self.remember("cone_path", message)

    def on_cone_clusters(self, message: PoseArray) -> None:
        self.remember("cone_clusters", message)

    def on_yolo(self, message) -> None:
        self.remember("yolo", message)

    def on_state(self, message: String) -> None:
        self.latest_state = str(message.data)
        self.arrival_times["state"] = time.monotonic()

    def on_motor(self, message: Float32MultiArray) -> None:
        if len(message.data) >= 2:
            self.latest_motor = (float(message.data[0]), float(message.data[1]))
            self.arrival_times["motor"] = time.monotonic()

    def fresh(self, key: str, timeout_name: str = "data_timeout_sec") -> bool:
        received = self.arrival_times.get(key, 0.0)
        timeout = float(self.get_parameter(timeout_name).value)
        return received > 0.0 and time.monotonic() - received <= timeout

    def current_camera(self) -> np.ndarray | None:
        message = self.latest_camera
        if message is None or not self.fresh("camera", "camera_timeout_sec"):
            return None
        stamp = message_stamp_ns(message)
        cache_key = stamp if stamp > 0 else id(message)
        if self.cached_camera_stamp == cache_key:
            return self.cached_camera
        decoded = decode_compressed_bgr(message.data)
        if decoded is None:
            return None
        self.cached_camera = self.rectifier.rectify(decoded)
        self.cached_camera_stamp = cache_key
        return self.cached_camera

    def current_canonical(self) -> np.ndarray | None:
        message = self.latest_canonical
        if message is None or not self.fresh("canonical"):
            return None
        stamp = message_stamp_ns(message)
        cache_key = stamp if stamp > 0 else id(message)
        if self.cached_canonical_stamp == cache_key:
            return self.cached_canonical
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"canonical conversion failed: {exc}")
            return None
        self.cached_canonical = np.asarray(image).copy()
        self.cached_canonical_stamp = cache_key
        return self.cached_canonical

    def mask(self, key: str, message: Image | None) -> np.ndarray | None:
        if message is None or not self.fresh(key):
            return None
        try:
            return np.asarray(
                self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
            )
        except Exception:
            return None

    def roi_ratios(self) -> list[float]:
        return [
            float(self.get_parameter(name).value)
            for name in (
                "src_tl_x_ratio",
                "src_tl_y_ratio",
                "src_tr_x_ratio",
                "src_tr_y_ratio",
                "src_br_x_ratio",
                "src_br_y_ratio",
                "src_bl_x_ratio",
                "src_bl_y_ratio",
            )
        ]

    def laser_points(self) -> np.ndarray:
        if not self.fresh("scan"):
            return np.empty((0, 2), dtype=np.float32)
        return scan_points(
            self.latest_scan,
            float(self.get_parameter("lidar_projection_min_range_m").value),
            float(self.get_parameter("lidar_projection_max_range_m").value),
        )

    def lane_path(self) -> list[Point2]:
        if self.latest_lane_path is None or not self.fresh("lane_path"):
            return []
        return [
            (float(point.x), float(point.y))
            for point in self.latest_lane_path.points
        ]

    def cone_path(self) -> list[Point2]:
        if self.latest_cone_path is None or not self.fresh("cone_path"):
            return []
        return [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in self.latest_cone_path.poses
        ]

    def cone_clusters(self) -> list[Point2]:
        if self.latest_cone_clusters is None or not self.fresh("cone_clusters"):
            return []
        offset = float(self.get_parameter("lidar_to_rear_axle_m").value)
        return [
            (float(pose.position.x) + offset, float(pose.position.y))
            for pose in self.latest_cone_clusters.poses
        ]

    def overlay_projected_points(
        self,
        image: np.ndarray,
        points_laser: np.ndarray,
    ) -> np.ndarray:
        output = image.copy()
        if (
            points_laser.shape[0] == 0
            or self.rectifier.rectified_matrix is None
        ):
            return output
        pixels, valid = project_laser_xy(
            points_laser,
            self.rotation_camera_laser,
            self.translation_camera_laser,
            self.rectifier.rectified_matrix,
        )
        distances = np.linalg.norm(points_laser, axis=1)
        height, width = output.shape[:2]
        for index in np.flatnonzero(valid):
            u, v = pixels[index]
            if not (0 <= u < width and 0 <= v < height):
                continue
            ratio = float(
                np.clip(
                    distances[index]
                    / max(
                        0.1,
                        float(
                            self.get_parameter(
                                "lidar_projection_max_range_m"
                            ).value
                        ),
                    ),
                    0.0,
                    1.0,
                )
            )
            color = (
                int(255 * ratio),
                int(255 * (1.0 - abs(ratio - 0.5) * 2.0)),
                int(255 * (1.0 - ratio)),
            )
            cv2.circle(
                output,
                (int(round(u)), int(round(v))),
                3,
                color,
                -1,
                cv2.LINE_AA,
            )
        return output

    def overlay_yolo(self, image: np.ndarray) -> np.ndarray:
        output = image.copy()
        if self.latest_yolo is None or not self.fresh("yolo"):
            return output
        height, width = output.shape[:2]
        source_width = max(1, int(self.latest_yolo.image_width))
        source_height = max(1, int(self.latest_yolo.image_height))
        scale_x = width / float(source_width)
        scale_y = height / float(source_height)
        for detection in self.latest_yolo.detections:
            x1 = int(round(float(detection.xmin) * scale_x))
            y1 = int(round(float(detection.ymin) * scale_y))
            x2 = int(round(float(detection.xmax) * scale_x))
            y2 = int(round(float(detection.ymax) * scale_y))
            x1, x2 = sorted((max(0, x1), min(width - 1, x2)))
            y1, y2 = sorted((max(0, y1), min(height - 1, y2)))
            if x2 <= x1 or y2 <= y1:
                continue
            cv2.rectangle(output, (x1, y1), (x2, y2), MAGENTA, 2)
            label = (
                f"{detection.class_name} "
                f"{float(detection.confidence):.2f}"
            )
            cv2.putText(
                output,
                label,
                (x1, max(16, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                MAGENTA,
                1,
                cv2.LINE_AA,
            )
        return output

    def camera_path_panel(
        self,
        camera: np.ndarray | None,
        path: list[Point2],
        mode: str,
    ) -> np.ndarray:
        if camera is None:
            return self.topdown_scene(mode, show_scan=True, show_path=True)
        output = camera.copy()
        laser_path = rear_axle_to_laser(
            path,
            float(self.get_parameter("lidar_to_rear_axle_m").value),
        )
        if (
            laser_path.shape[0] >= 2
            and self.rectifier.rectified_matrix is not None
        ):
            pixels, valid = project_laser_xy(
                laser_path,
                self.rotation_camera_laser,
                self.translation_camera_laser,
                self.rectifier.rectified_matrix,
            )
            integer = np.full((pixels.shape[0], 2), -10000, dtype=np.int32)
            integer[valid] = np.rint(pixels[valid]).astype(np.int32)
            polyline_in_bounds(output, integer, CYAN, 7)
            polyline_in_bounds(output, integer, BLACK, 3)
        angle, speed = self.latest_motor
        origin = (output.shape[1] // 2, output.shape[0] - 35)
        length = 115
        radians = math.radians(float(angle))
        endpoint = (
            int(round(origin[0] - math.sin(radians) * length)),
            int(round(origin[1] - math.cos(radians) * length)),
        )
        cv2.arrowedLine(output, origin, endpoint, MAGENTA, 5, cv2.LINE_AA)
        cv2.putText(
            output,
            f"steer={angle:+.1f} speed={speed:.1f}",
            (15, output.shape[0] - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            WHITE,
            2,
            cv2.LINE_AA,
        )
        return output

    def canonical_scene(self) -> np.ndarray:
        canonical = self.current_canonical()
        if canonical is None:
            return self.blank_scene("WAITING FOR CANONICAL LANE")
        output = canonical.copy()
        white = self.mask("white", self.latest_white)
        yellow = self.mask("yellow", self.latest_yellow)
        if white is not None and white.shape == output.shape[:2]:
            output[white > 0] = RED
        if yellow is not None and yellow.shape == output.shape[:2]:
            output[yellow > 0] = YELLOW
        path = self.lane_path()
        pixels = canonical_pixels(
            path,
            output.shape[1],
            output.shape[0],
            float(self.get_parameter("canonical_forward_range_m").value),
            float(self.get_parameter("canonical_lateral_range_m").value),
        )
        polyline_in_bounds(output, pixels, CYAN, 7)
        polyline_in_bounds(output, pixels, BLACK, 3)
        return output

    def topdown_scene(
        self,
        mode: str,
        *,
        show_scan: bool,
        show_path: bool,
    ) -> np.ndarray:
        width = self.panel_width
        height = self.panel_height - 30
        output = np.full((height, width, 3), 24, dtype=np.uint8)
        forward_range = float(
            self.get_parameter("topdown_forward_range_m").value
        )
        lateral_range = float(
            self.get_parameter("topdown_lateral_range_m").value
        )

        for distance in np.arange(0.5, forward_range + 0.01, 0.5):
            row = metric_to_topdown_pixels(
                [(float(distance), 0.0)],
                width,
                height,
                forward_range,
                lateral_range,
            )[0, 1]
            cv2.line(output, (0, int(row)), (width - 1, int(row)), (48, 48, 48), 1)
            cv2.putText(
                output,
                f"{distance:.1f}m",
                (5, max(15, int(row) - 3)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                GRAY,
                1,
                cv2.LINE_AA,
            )
        cv2.line(
            output,
            (width // 2, 0),
            (width // 2, height - 1),
            (60, 60, 60),
            1,
        )

        offset = float(self.get_parameter("lidar_to_rear_axle_m").value)
        if show_scan:
            laser = self.laser_points()
            rear_scan = (
                np.column_stack((laser[:, 0] + offset, laser[:, 1]))
                if laser.shape[0]
                else laser
            )
            scan_pixels = metric_to_topdown_pixels(
                rear_scan,
                width,
                height,
                forward_range,
                lateral_range,
            )
            for x, y in scan_pixels:
                if 0 <= x < width and 0 <= y < height:
                    cv2.circle(output, (int(x), int(y)), 1, GRAY, -1)

        clusters = self.cone_clusters()
        cluster_pixels = metric_to_topdown_pixels(
            clusters,
            width,
            height,
            forward_range,
            lateral_range,
        )
        for (forward, lateral), (x, y) in zip(clusters, cluster_pixels):
            if 0 <= x < width and 0 <= y < height:
                color = BLUE if lateral >= 0.0 else RED
                cv2.circle(output, (int(x), int(y)), 7, color, -1, cv2.LINE_AA)

        if show_path:
            path = self.cone_path() if mode == "cone" else self.lane_path()
            path_pixels = metric_to_topdown_pixels(
                path,
                width,
                height,
                forward_range,
                lateral_range,
            )
            polyline_in_bounds(output, path_pixels, CYAN, 7)
            polyline_in_bounds(output, path_pixels, BLACK, 3)

        vehicle = np.asarray(
            [
                [width // 2, height - 8],
                [width // 2 - 10, height - 28],
                [width // 2 + 10, height - 28],
            ],
            dtype=np.int32,
        )
        cv2.fillConvexPoly(output, vehicle, GREEN)
        return output

    @staticmethod
    def blank_scene(text: str) -> np.ndarray:
        output = np.full((330, 640, 3), 24, dtype=np.uint8)
        cv2.putText(
            output,
            text,
            (35, output.shape[0] // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            GRAY,
            2,
            cv2.LINE_AA,
        )
        return output

    def render(self) -> None:
        if not self.show_window:
            return
        camera = self.current_camera()
        mode = infer_drive_mode(self.latest_state, self.mode_request)
        laser = self.laser_points()

        if camera is not None:
            camera_lidar = self.overlay_projected_points(camera, laser)
            roi = camera.copy()
            cv2.polylines(
                roi,
                [roi_points(roi.shape[1], roi.shape[0], self.roi_ratios())],
                True,
                GREEN,
                4,
                cv2.LINE_AA,
            )
            roi = self.overlay_yolo(roi)
        else:
            camera_lidar = self.topdown_scene(
                mode, show_scan=True, show_path=False
            )
            roi = self.topdown_scene(mode, show_scan=False, show_path=True)

        if mode == "cone":
            perception = self.topdown_scene(
                mode, show_scan=True, show_path=True
            )
            active_path = self.cone_path()
            perception_title = "2 CONE BEV | clusters + center path"
        else:
            perception = self.canonical_scene()
            active_path = self.lane_path()
            perception_title = "2 LANE BEV | outer:red center:yellow target:black"

        path_camera = self.camera_path_panel(camera, active_path, mode)
        angle, speed = self.latest_motor
        mode_status = f"mode={mode.upper()}"
        motor_status = f"steer={angle:+.1f} speed={speed:.1f}"
        panels = [
            make_panel(
                camera_lidar,
                "1 CAMERA + LiDAR projection",
                self.panel_width,
                self.panel_height,
                f"points={laser.shape[0]}",
            ),
            make_panel(
                perception,
                perception_title,
                self.panel_width,
                self.panel_height,
                mode_status,
            ),
            make_panel(
                roi,
                "3 CAMERA ROI | YOLO boxes when available",
                self.panel_width,
                self.panel_height,
                "camera=ON" if camera is not None else "camera=OFF",
            ),
            make_panel(
                path_camera,
                "4 ACTIVE PATH + final command",
                self.panel_width,
                self.panel_height,
                motor_status,
            ),
        ]
        canvas = np.vstack(
            (
                np.hstack((panels[0], panels[1])),
                np.hstack((panels[2], panels[3])),
            )
        )
        if not self.window_created:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                self.window_name,
                self.panel_width * 2,
                self.panel_height * 2,
            )
            self.window_created = True
        cv2.imshow(self.window_name, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            rclpy.shutdown()

    def destroy_node(self):
        if self.window_created:
            cv2.destroyWindow(self.window_name)
            cv2.waitKey(1)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionViewNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
