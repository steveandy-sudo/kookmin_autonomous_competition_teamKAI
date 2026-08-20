#!/usr/bin/env python3
"""Rosbag shortcut review UI and opt-in steering-only hardware bridge."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from my_rule_msgs.msg import ObjectDetectionArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosbag2_interfaces.srv import PlayNext, TogglePaused
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32MultiArray, String


YELLOW = "\033[93m"
MAGENTA = "\033[95m"
GREEN = "\033[92m"
CYAN = "\033[96m"
RESET = "\033[0m"


@dataclass
class EventFrame:
    title: str
    color: tuple[int, int, int]
    image: np.ndarray
    stamp: float


class ShortcutBagWheelGate(Node):
    """Drive the restored shortcut stack from recorded camera/detections."""

    def __init__(self) -> None:
        super().__init__("shortcut_bag_wheel_gate")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("display_enabled", True)
        self.declare_parameter("left_confidence", 0.40)
        self.declare_parameter("rule_speed_command", 9.0)
        self.declare_parameter("save_directory", "/tmp/shortcut_bag_event_frames")
        self.declare_parameter("camera_topic", "/wide_camera_mjpeg/image_raw/compressed")
        self.declare_parameter("detections_topic", "/my_rule/object_detections")
        self.declare_parameter("candidate_topic", "/hybrid/shortcut_candidate")
        self.declare_parameter("motor_topic", "/xycar_motor")

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # The recorded camera and object detector both offered BEST_EFFORT.
        # A default integer-depth subscription is RELIABLE and therefore never
        # matches the detector publisher created by rosbag2.
        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.bridge = CvBridge()
        self.hardware_enabled = bool(self.get_parameter("hardware_enabled").value)
        self.display_enabled = bool(self.get_parameter("display_enabled").value)
        self.save_directory = Path(str(self.get_parameter("save_directory").value))
        self.save_directory.mkdir(parents=True, exist_ok=True)

        self.front: np.ndarray | None = None
        self.canonical: np.ndarray | None = None
        self.bev: np.ndarray | None = None
        self.front_stamp = math.nan
        self.left_detect_frames = 0
        self.left_confirmed = False
        self.left_absent_frames = 0
        self.detection_messages = 0
        self.last_left_seen = False
        self.processing_enabled = False
        self.ready = False
        self.handoff = False
        self.phase = 0.0
        self.candidate = (0.0, 0.0, 0.0, 0.0)
        self.mux_status = "waiting for bag"
        self.events: list[EventFrame] = []
        self.window_name = "MAIN W1/W2 SHORTCUT BAG REVIEW | SPACE pause/play | N next frame | Q quit"
        self.last_ui_time = 0.0

        self.processing_pub = self.create_publisher(
            Bool, "/hybrid/shortcut_processing_enabled", state_qos
        )
        self.rule_pub = self.create_publisher(
            Float32MultiArray, "/hybrid/rule_candidate", 10
        )
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("motor_topic").value),
            10,
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("camera_topic").value),
            self.on_camera,
            image_qos,
        )
        self.create_subscription(
            ObjectDetectionArray,
            str(self.get_parameter("detections_topic").value),
            self.on_detections,
            detection_qos,
        )
        self.create_subscription(
            Image,
            "/shortcut/entry/canonical_model_input",
            self.on_canonical,
            image_qos,
        )
        self.create_subscription(
            Image,
            "/shortcut/entry/debug_image",
            self.on_bev,
            image_qos,
        )
        self.create_subscription(
            Bool, "/shortcut/entry/ready", self.on_ready, state_qos
        )
        self.create_subscription(
            Bool, "/shortcut/entry/cruise_enabled", self.on_handoff, state_qos
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("candidate_topic").value),
            self.on_candidate,
            10,
        )
        self.create_subscription(
            String, "/shortcut/entry/mux_status", self.on_mux_status, 10
        )

        self.toggle_client = self.create_client(
            TogglePaused, "/rosbag2_player/toggle_paused"
        )
        self.next_client = self.create_client(
            PlayNext, "/rosbag2_player/play_next"
        )
        self.create_timer(0.05, self.publish_rule)
        if self.display_enabled:
            self.create_timer(0.03, self.update_ui)
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1800, 1000)
        mode = "STEERING-ONLY HARDWARE" if self.hardware_enabled else "SHADOW ONLY"
        print(f"{CYAN}[SHORTCUT BAG] {mode}; recorded /xycar_motor is never replayed{RESET}", flush=True)

    @staticmethod
    def stamp_sec(message) -> float:
        return float(message.header.stamp.sec) + 1e-9 * float(message.header.stamp.nanosec)

    @staticmethod
    def normalize_class_name(value: str) -> str:
        return str(value).strip().lower().replace("-", "_").replace(" ", "_")

    def on_camera(self, message: CompressedImage) -> None:
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            return
        self.front = frame
        self.front_stamp = self.stamp_sec(message)

    def on_canonical(self, message: Image) -> None:
        try:
            self.canonical = self.bridge.imgmsg_to_cv2(message, "bgr8")
        except Exception as exc:
            self.get_logger().warning(f"canonical conversion failed: {exc}")

    def on_bev(self, message: Image) -> None:
        try:
            self.bev = self.bridge.imgmsg_to_cv2(message, "bgr8")
        except Exception as exc:
            self.get_logger().warning(f"BEV conversion failed: {exc}")

    def on_detections(self, message: ObjectDetectionArray) -> None:
        self.detection_messages += 1
        threshold = float(self.get_parameter("left_confidence").value)
        left_seen = any(
            self.normalize_class_name(item.class_name) == "left_4"
            and float(item.confidence) >= threshold
            for item in message.detections
        )
        self.last_left_seen = left_seen
        if not self.left_confirmed:
            self.left_detect_frames = self.left_detect_frames + 1 if left_seen else 0
            if self.left_detect_frames >= 2:
                self.left_confirmed = True
                self.capture_event(
                    "1  RECORDED LEFT_4 DETECTED 2/2",
                    (0, 255, 255),
                    YELLOW,
                )
        elif not self.processing_enabled:
            if left_seen:
                self.left_absent_frames = 0
            else:
                self.left_absent_frames += 1
                if self.left_absent_frames >= 2:
                    self.capture_event(
                        "2  RECORDED LEFT_4 ABSENT 2/2",
                        (0, 255, 255),
                        YELLOW,
                    )
                    self.processing_enabled = True
                    self.processing_pub.publish(Bool(data=True))
                    print(
                        f"{YELLOW}[MISSION] shortcut perception enabled after left_4 2/2 absent{RESET}",
                        flush=True,
                    )

    def on_ready(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested and not self.ready:
            self.capture_event(
                "3  RED W1/W2 INTERSECTION - STEERING START",
                (0, 0, 255),
                MAGENTA,
            )
        self.ready = requested

    def on_handoff(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested and not self.handoff:
            self.capture_event(
                "4  YELLOW RULE MODE RESUMED",
                (0, 255, 0),
                GREEN,
            )
        self.handoff = requested

    def on_candidate(self, message: Float32MultiArray) -> None:
        values = list(message.data) + [0.0, 0.0, 0.0, 0.0]
        self.candidate = tuple(float(value) for value in values[:4])
        if self.hardware_enabled:
            # Steering is live, propulsion is deliberately held at zero.
            self.motor_pub.publish(
                Float32MultiArray(data=[float(self.candidate[0]), 0.0])
            )

    def on_mux_status(self, message: String) -> None:
        self.mux_status = str(message.data)

    def publish_rule(self) -> None:
        speed = max(0.0, float(self.get_parameter("rule_speed_command").value))
        self.rule_pub.publish(Float32MultiArray(data=[0.0, speed]))
        if self.processing_enabled:
            self.processing_pub.publish(Bool(data=True))

    def capture_event(
        self,
        title: str,
        color: tuple[int, int, int],
        terminal_color: str,
    ) -> None:
        if len(self.events) >= 4 or self.front is None:
            return
        image = self.front.copy()
        cv2.rectangle(image, (0, 0), (image.shape[1] - 1, image.shape[0] - 1), color, 12)
        cv2.rectangle(image, (0, 0), (image.shape[1], 54), (0, 0, 0), -1)
        cv2.putText(
            image,
            title,
            (14, 37),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.82,
            color,
            2,
            cv2.LINE_AA,
        )
        event = EventFrame(title, color, image, self.front_stamp)
        self.events.append(event)
        index = len(self.events)
        path = self.save_directory / f"event_{index}.png"
        cv2.imwrite(str(path), image)
        print(
            f"{terminal_color}[MISSION EVENT {index}/4] {title} | camera_stamp={self.front_stamp:.9f} | saved={path}{RESET}",
            flush=True,
        )
        if index == 4:
            self.save_four_event_board()

    @staticmethod
    def panel(image: np.ndarray | None, width: int, height: int, label: str) -> np.ndarray:
        canvas = np.full((height, width, 3), 18, dtype=np.uint8)
        if image is not None and image.size:
            scale = min(width / image.shape[1], (height - 34) / image.shape[0])
            resized = cv2.resize(
                image,
                (max(1, int(image.shape[1] * scale)), max(1, int(image.shape[0] * scale))),
            )
            x = (width - resized.shape[1]) // 2
            y = 34 + (height - 34 - resized.shape[0]) // 2
            canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
        cv2.putText(
            canvas,
            label,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (235, 235, 235),
            2,
            cv2.LINE_AA,
        )
        return canvas

    def steering_panel(self, width: int, height: int) -> np.ndarray:
        canvas = np.full((height, width, 3), 18, dtype=np.uint8)
        angle = float(self.candidate[0])
        start = (width // 2, height - 50)
        tip = (
            int(round(start[0] + np.clip(angle / 42.0, -1.0, 1.0) * width * 0.36)),
            70,
        )
        cv2.arrowedLine(canvas, start, tip, (0, 255, 255), 12, cv2.LINE_AA, tipLength=0.18)
        cv2.putText(
            canvas,
            f"COMPUTED STEERING {angle:+.2f}  SPEED {self.candidate[1]:.2f}",
            (14, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.66,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            (
                f"detections={self.detection_messages} "
                f"left_4={int(self.last_left_seen)} "
                f"enabled={int(self.processing_enabled)} "
                f"ready={int(self.ready)} handoff={int(self.handoff)}  "
                f"{self.mux_status[:35]}"
            ),
            (14, height - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (210, 210, 210),
            1,
            cv2.LINE_AA,
        )
        return canvas

    def compose(self) -> np.ndarray:
        panel_w, top_h, event_h = 600, 390, 270
        top = np.hstack(
            [
                self.panel(self.front, panel_w, top_h, "RECORDED FRONT CAMERA"),
                self.panel(self.canonical, panel_w, top_h, "CANONICAL MODEL INPUT"),
                self.panel(self.bev, panel_w, top_h, "BEV W1 / W2 / Y1 / RED X"),
            ]
        )
        event_panels = []
        for index in range(4):
            if index < len(self.events):
                event = self.events[index]
                panel = self.panel(event.image, 450, event_h, event.title)
                cv2.rectangle(panel, (0, 0), (449, event_h - 1), event.color, 5)
            else:
                panel = self.panel(None, 450, event_h, f"EVENT {index + 1} WAITING")
            event_panels.append(panel)
        event_row = np.hstack(event_panels)
        steering = self.steering_panel(1800, 210)
        board = np.vstack([top, steering, event_row])
        if self.handoff:
            # Green: shortcut entry is complete and yellow RULE owns steering.
            cv2.rectangle(
                board,
                (5, 5),
                (board.shape[1] - 6, board.shape[0] - 6),
                (0, 255, 0),
                12,
            )
        elif self.ready:
            # Red: the W1/W2 branch gate has opened; direct W1 steering owns it.
            cv2.rectangle(
                board,
                (5, 5),
                (board.shape[1] - 6, board.shape[0] - 6),
                (0, 0, 255),
                12,
            )
        return board

    def save_four_event_board(self) -> None:
        board = np.hstack(
            [self.panel(event.image, 480, 330, event.title) for event in self.events]
        )
        path = self.save_directory / "four_event_frames.png"
        cv2.imwrite(str(path), board)
        print(f"{GREEN}[MISSION] four-frame board saved: {path}{RESET}", flush=True)

    def update_ui(self) -> None:
        if time.monotonic() - self.last_ui_time < 0.02:
            return
        self.last_ui_time = time.monotonic()
        cv2.imshow(self.window_name, self.compose())
        key = cv2.waitKey(1) & 0xFF
        if key == 32:
            if self.toggle_client.service_is_ready():
                self.toggle_client.call_async(TogglePaused.Request())
                print(f"{CYAN}[BAG] pause/play toggled{RESET}", flush=True)
        elif key in (ord("n"), ord("N"), 83):
            if self.next_client.service_is_ready():
                self.next_client.call_async(PlayNext.Request())
                print(f"{CYAN}[BAG] next recorded message{RESET}", flush=True)
        elif key in (27, ord("q"), ord("Q")):
            rclpy.shutdown()

    def stop(self) -> None:
        if rclpy.ok():
            self.processing_pub.publish(Bool(data=False))
            if self.hardware_enabled:
                for _ in range(5):
                    self.motor_pub.publish(Float32MultiArray(data=[0.0, 0.0]))
        if self.display_enabled:
            cv2.destroyAllWindows()


def main() -> None:
    rclpy.init()
    node = ShortcutBagWheelGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
