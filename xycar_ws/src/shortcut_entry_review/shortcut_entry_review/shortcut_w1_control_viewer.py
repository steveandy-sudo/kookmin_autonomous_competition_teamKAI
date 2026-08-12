#!/usr/bin/env python3
"""Composite rosbag viewer for deterministic W1 shortcut control review."""

from __future__ import annotations

from collections import OrderedDict
import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosbag2_interfaces.srv import Pause, PlayNext, Resume
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, String


WINDOW = "Shortcut W1 Control Review"
CANVAS_WIDTH = 1360
CANVAS_HEIGHT = 760
PHASE_NAMES = {
    0: "LEFT4_ARMED",
    1: "W1_LOCKED",
    2: "Y1_LOCKED",
    3: "PAIR_TRACK",
    4: "CRUISE_HANDOFF",
}


def _stamp_key(message: CompressedImage | Image) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


def _finite_at(values: list[float], index: int) -> bool:
    return len(values) > index and math.isfinite(float(values[index]))


def _letterbox(
    image: np.ndarray | None,
    width: int,
    height: int,
    *,
    empty_text: str,
) -> tuple[np.ndarray, tuple[int, int, int, int]]:
    panel = np.full((height, width, 3), (17, 16, 15), dtype=np.uint8)
    if image is None or image.size == 0:
        cv2.putText(
            panel,
            empty_text,
            (30, height // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.66,
            (150, 150, 150),
            1,
            cv2.LINE_AA,
        )
        return panel, (0, 0, width, height)
    source_height, source_width = image.shape[:2]
    scale = min(width / source_width, height / source_height)
    resized_width = max(1, int(round(source_width * scale)))
    resized_height = max(1, int(round(source_height * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(
        image, (resized_width, resized_height), interpolation=interpolation
    )
    x = (width - resized_width) // 2
    y = (height - resized_height) // 2
    panel[y : y + resized_height, x : x + resized_width] = resized
    return panel, (x, y, resized_width, resized_height)


def _put(
    image: np.ndarray,
    text: str,
    point: tuple[int, int],
    *,
    scale: float = 0.52,
    color: tuple[int, int, int] = (235, 235, 235),
    thickness: int = 1,
) -> None:
    cv2.putText(
        image,
        text,
        point,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


class ShortcutW1ControlViewer(Node):
    """Show the synchronized camera, selector BEV, and real controller output."""

    def __init__(self) -> None:
        super().__init__("shortcut_w1_control_viewer")
        self.declare_parameter(
            "camera_topic", "/wide_camera_mjpeg/image_raw/compressed"
        )
        self.declare_parameter("debug_topic", "/shortcut/entry/debug_image")
        self.declare_parameter(
            "selector_diagnostics_topic", "/shortcut/entry/diagnostics"
        )
        self.declare_parameter(
            "controller_candidate_topic",
            "/shortcut/entry/controller_candidate",
        )
        self.declare_parameter(
            "control_debug_topic", "/shortcut/entry/control_debug"
        )
        self.declare_parameter("status_topic", "/shortcut/entry/status")
        self.declare_parameter(
            "mux_status_topic", "/shortcut/entry/mux_status"
        )
        self.declare_parameter("player_controls_enabled", True)
        self.declare_parameter("initial_paused", True)
        self.declare_parameter("autoplay", False)
        self.declare_parameter("playback_rate", 0.5)
        self.declare_parameter("maximum_steering_command", 42.0)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=8,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.bridge = CvBridge()
        self.camera_frames: OrderedDict[
            tuple[int, int], np.ndarray
        ] = OrderedDict()
        self.camera_image: np.ndarray | None = None
        self.canonical_image: np.ndarray | None = None
        self.selector_diagnostics: list[float] = []
        self.controller_candidate: list[float] = [0.0, 0.0]
        self.raw_w1_candidate: list[float] = [0.0, 0.0]
        self.control_debug: list[float] = []
        self.reason = "waiting for semantic masks"
        self.control_reason = "RULE retained until spatial gate"
        self.display_stamp: tuple[int, int] | None = None
        self.first_stamp_ns: int | None = None
        self.semantic_frames = 0
        self.paused = bool(self.get_parameter("initial_paused").value)
        self.quit_requested = False
        self.fullscreen = False
        self.player_primed = False
        self.autoplay_started = False
        self.pending_player_call = None

        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("camera_topic").value),
            self.on_camera,
            image_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("debug_topic").value),
            self.on_debug,
            image_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("selector_diagnostics_topic").value),
            self.on_selector_diagnostics,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("controller_candidate_topic").value),
            self.on_controller_candidate,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("control_debug_topic").value),
            self.on_control_debug,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("status_topic").value),
            self.on_status,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mux_status_topic").value),
            self.on_mux_status,
            10,
        )

        self.pause_client = self.create_client(Pause, "/rosbag2_player/pause")
        self.resume_client = self.create_client(
            Resume, "/rosbag2_player/resume"
        )
        self.play_next_client = self.create_client(
            PlayNext, "/rosbag2_player/play_next"
        )
        self.create_timer(0.20, self.prime_first_frame)
        self.create_timer(0.20, self.start_autoplay)
        self.get_logger().info(
            "W1 control viewer ready: SPACE bag pause/resume, F fullscreen, Q quit"
        )

    def on_camera(self, message: CompressedImage) -> None:
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            return
        key = _stamp_key(message)
        self.camera_frames[key] = image
        self.camera_frames.move_to_end(key)
        while len(self.camera_frames) > 32:
            self.camera_frames.popitem(last=False)
        if self.camera_image is None:
            self.camera_image = image

    def on_debug(self, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            )
        except Exception as exc:
            self.get_logger().warning(f"debug image conversion failed: {exc}")
            return
        key = _stamp_key(message)
        self.canonical_image = image.copy()
        stamp_ns = key[0] * 1_000_000_000 + key[1]
        if self.display_stamp is not None and key < self.display_stamp:
            self.first_stamp_ns = stamp_ns
            self.semantic_frames = 0
            self.controller_candidate = [0.0, 0.0]
            self.raw_w1_candidate = [0.0, 0.0]
            self.control_debug = []
        self.display_stamp = key
        self.semantic_frames += 1
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp_ns
        matched = self.camera_frames.get(key)
        if matched is not None:
            self.camera_image = matched

    def on_selector_diagnostics(self, message: Float32MultiArray) -> None:
        self.selector_diagnostics = [float(value) for value in message.data]

    def on_controller_candidate(self, message: Float32MultiArray) -> None:
        if len(message.data) >= 2:
            self.raw_w1_candidate = [
                float(message.data[0]),
                float(message.data[1]),
            ]

    def on_control_debug(self, message: Float32MultiArray) -> None:
        self.control_debug = [float(value) for value in message.data]
        if len(self.control_debug) >= 2:
            self.controller_candidate = self.control_debug[:2]

    def on_status(self, message: String) -> None:
        self.reason = str(message.data)

    def on_mux_status(self, message: String) -> None:
        self.control_reason = str(message.data)

    def prime_first_frame(self) -> None:
        if (
            self.player_primed
            or self.camera_image is not None
            or not bool(self.get_parameter("player_controls_enabled").value)
            or not self.play_next_client.service_is_ready()
        ):
            return
        self.player_primed = True
        self.pending_player_call = self.play_next_client.call_async(
            PlayNext.Request()
        )

    def start_autoplay(self) -> None:
        """Resume once the paused player has supplied its first camera frame."""
        if (
            self.autoplay_started
            or not bool(self.get_parameter("autoplay").value)
            or not bool(self.get_parameter("player_controls_enabled").value)
            or self.camera_image is None
            or not self.resume_client.service_is_ready()
        ):
            return
        self.autoplay_started = True
        self.pending_player_call = self.resume_client.call_async(
            Resume.Request()
        )
        self.paused = False
        self.get_logger().info("First frame ready; rosbag autoplay started")

    def toggle_player(self) -> None:
        if not bool(self.get_parameter("player_controls_enabled").value):
            self.paused = not self.paused
            return
        if self.paused:
            if not self.resume_client.service_is_ready():
                self.get_logger().warning("rosbag resume service is not ready")
                return
            self.pending_player_call = self.resume_client.call_async(
                Resume.Request()
            )
            self.paused = False
        else:
            if not self.pause_client.service_is_ready():
                self.get_logger().warning("rosbag pause service is not ready")
                return
            self.pending_player_call = self.pause_client.call_async(
                Pause.Request()
            )
            self.paused = True

    def handle_key(self, key: int) -> None:
        key &= 0xFF
        if key in (ord("q"), ord("Q"), 27):
            self.quit_requested = True
        elif key == ord(" "):
            self.toggle_player()
        elif key in (ord("f"), ord("F")):
            self.fullscreen = not self.fullscreen
            cv2.setWindowProperty(
                WINDOW,
                cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN
                if self.fullscreen
                else cv2.WINDOW_NORMAL,
            )

    def _phase(self) -> str:
        if not self.selector_diagnostics:
            return "WAITING"
        return PHASE_NAMES.get(
            int(round(self.selector_diagnostics[0])), "UNKNOWN"
        )

    def _bag_offset(self) -> float:
        if self.display_stamp is None or self.first_stamp_ns is None:
            return 0.0
        stamp_ns = (
            self.display_stamp[0] * 1_000_000_000 + self.display_stamp[1]
        )
        return max(0.0, (stamp_ns - self.first_stamp_ns) / 1_000_000_000.0)

    def render(self) -> np.ndarray:
        canvas = np.full(
            (CANVAS_HEIGHT, CANVAS_WIDTH, 3), (28, 23, 21), dtype=np.uint8
        )
        margin = 18
        gap = 18
        panel_width = (CANVAS_WIDTH - 2 * margin - gap) // 2
        right_x = margin + panel_width + gap
        image_y = 58
        image_height = 505
        bottom_y = 575
        bottom_height = CANVAS_HEIGHT - bottom_y - 14

        _put(
            canvas,
            "RECORDED WIDE CAMERA",
            (margin + 8, 38),
            scale=0.72,
            thickness=2,
        )
        _put(
            canvas,
            "CANONICAL BIRD'S-EYE VIEW / W1 CONTROL",
            (right_x + 8, 38),
            scale=0.68,
            thickness=2,
        )

        camera_panel, _ = _letterbox(
            self.camera_image,
            panel_width,
            image_height,
            empty_text="waiting for recorded camera",
        )
        canonical_panel, content = _letterbox(
            self.canonical_image,
            panel_width,
            image_height,
            empty_text="waiting for W1 selector",
        )
        canvas[
            image_y : image_y + image_height,
            margin : margin + panel_width,
        ] = camera_panel
        canvas[
            image_y : image_y + image_height,
            right_x : right_x + panel_width,
        ] = canonical_panel

        if self.display_stamp is not None:
            stamp_text = (
                f"stamp {self.display_stamp[0]}.{self.display_stamp[1]:09d}"
            )
            cv2.rectangle(
                canvas,
                (margin + 8, image_y + 8),
                (margin + 340, image_y + 38),
                (12, 12, 12),
                -1,
            )
            _put(canvas, stamp_text, (margin + 16, image_y + 30), scale=0.48)

        content_x, content_y, content_w, content_h = content
        for fraction in (0.25, 0.50, 0.75):
            x = right_x + content_x + int(round(content_w * fraction))
            cv2.line(
                canvas,
                (x, image_y + content_y),
                (x, image_y + content_y + content_h),
                (58, 55, 53),
                1,
            )
        for fraction in (1.0 / 3.0, 2.0 / 3.0):
            y = image_y + content_y + int(round(content_h * fraction))
            cv2.line(
                canvas,
                (right_x + content_x, y),
                (right_x + content_x + content_w, y),
                (58, 55, 53),
                1,
            )

        angle = float(self.controller_candidate[0])
        speed = float(self.controller_candidate[1])
        blend = self.control_debug[2] if len(self.control_debug) > 2 else 0.0
        distance = (
            self.control_debug[3] if len(self.control_debug) > 3 else math.nan
        )
        start_distance = (
            self.control_debug[4] if len(self.control_debug) > 4 else math.nan
        )
        raw_w1_angle = (
            self.control_debug[5]
            if len(self.control_debug) > 5
            else self.raw_w1_candidate[0]
        )
        rule_angle = (
            self.control_debug[6] if len(self.control_debug) > 6 else 0.0
        )
        maximum = max(
            1.0, float(self.get_parameter("maximum_steering_command").value)
        )
        direction = (
            "RIGHT"
            if angle > 0.5
            else "LEFT" if angle < -0.5 else "STRAIGHT"
        )

        cv2.rectangle(
            canvas,
            (margin, bottom_y),
            (margin + panel_width, bottom_y + bottom_height),
            (37, 31, 28),
            -1,
        )
        cv2.rectangle(
            canvas,
            (right_x, bottom_y),
            (right_x + panel_width, bottom_y + bottom_height),
            (37, 31, 28),
            -1,
        )
        _put(
            canvas,
            "DETECTED LINES",
            (margin + 18, bottom_y + 28),
            scale=0.62,
            thickness=2,
        )

        path_valid = (
            len(self.selector_diagnostics) > 2
            and self.selector_diagnostics[2] > 0.5
        )
        line_items = [
            (
                "W1",
                "target white",
                _finite_at(self.selector_diagnostics, 5),
                (255, 0, 255),
            ),
            (
                "W2",
                "other white",
                _finite_at(self.selector_diagnostics, 11),
                (255, 150, 40),
            ),
            (
                "Y1",
                "target yellow",
                _finite_at(self.selector_diagnostics, 7),
                (0, 255, 80),
            ),
            ("Y2", "other yellow", False, (160, 150, 145)),
            ("PATH", "W1 centerline", path_valid, (255, 210, 40)),
        ]
        positions = [
            (margin + 24, bottom_y + 61),
            (margin + 340, bottom_y + 61),
            (margin + 24, bottom_y + 98),
            (margin + 340, bottom_y + 98),
            (margin + 24, bottom_y + 135),
        ]
        for (name, description, active, color), (x, y) in zip(line_items, positions):
            shown = color if active else (140, 132, 128)
            cv2.circle(canvas, (x, y - 5), 6, shown, -1, cv2.LINE_AA)
            _put(canvas, name, (x + 16, y), scale=0.47, color=shown)
            _put(canvas, description, (x + 66, y), scale=0.43, color=shown)

        state_color = (45, 245, 255) if self.paused else (80, 235, 100)
        _put(
            canvas,
            f"phase: {self._phase()}",
            (right_x + 18, bottom_y + 28),
            scale=0.55,
            thickness=2,
        )
        _put(
            canvas,
            "PAUSED" if self.paused else "PLAYING",
            (right_x + panel_width - 145, bottom_y + 28),
            scale=0.55,
            color=state_color,
            thickness=2,
        )

        # Keep the steering indication in the status card rather than over
        # the lane pixels.  The BEV panel above remains unobstructed.
        arrow_base = (
            right_x + panel_width - 72,
            bottom_y + 132,
        )
        arrow_tip = (
            arrow_base[0]
            + int(round(np.clip(angle / maximum, -1.0, 1.0) * 48)),
            bottom_y + 67,
        )
        cv2.arrowedLine(
            canvas,
            arrow_base,
            arrow_tip,
            (45, 245, 255),
            4,
            cv2.LINE_AA,
            tipLength=0.24,
        )
        _put(
            canvas,
            direction,
            (right_x + panel_width - 142, bottom_y + 154),
            scale=0.39,
            color=(45, 245, 255),
            thickness=1,
        )
        _put(
            canvas,
            f"output steering: {angle:+.2f}    RULE speed: {speed:+.2f}",
            (right_x + 18, bottom_y + 60),
            scale=0.51,
            color=(45, 245, 255),
            thickness=2,
        )
        rate = float(self.get_parameter("playback_rate").value)
        _put(
            canvas,
            f"bag: {self._bag_offset():.2f}s   rate: {rate:.1f}x   "
            f"semantic frames: {self.semantic_frames}",
            (right_x + 18, bottom_y + 91),
            scale=0.43,
            color=(190, 184, 180),
        )
        _put(
            canvas,
            f"blend={blend:.2f}  fork={distance:.2f}m  "
            f"start={start_distance:.2f}m",
            (right_x + 18, bottom_y + 119),
            scale=0.42,
            color=(190, 184, 180),
        )
        reason = (
            f"RULE {rule_angle:+.1f} -> W1 {raw_w1_angle:+.1f}; "
            f"{self.control_reason}"
        )[:84]
        _put(
            canvas,
            reason,
            (right_x + 18, bottom_y + 145),
            scale=0.37,
            color=(170, 166, 162),
        )
        _put(
            canvas,
            "SPACE pause/resume    F fullscreen    Q quit",
            (right_x + panel_width - 380, bottom_y + 164),
            scale=0.38,
            color=(225, 220, 216),
        )
        return canvas


def main() -> None:
    cv2.setNumThreads(1)
    rclpy.init()
    node = ShortcutW1ControlViewer()
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, CANVAS_WIDTH, CANVAS_HEIGHT)
    try:
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.01)
            cv2.imshow(WINDOW, node.render())
            node.handle_key(cv2.waitKey(1))
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
