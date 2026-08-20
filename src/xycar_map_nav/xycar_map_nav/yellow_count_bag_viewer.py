#!/usr/bin/env python3
"""Review left_4 and W1-input BEV yellow inference from a rosbag.

This tool is deliberately observation-only.  It does not publish a motor or
shortcut candidate command, and it does not implement the future turn.  Its
only job is to make the prospective yellow-segment trigger easy to count.
"""

from __future__ import annotations

import math
import time

import cv2
from cv_bridge import CvBridge
from my_rule_msgs.msg import ObjectDetectionArray
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosbag2_interfaces.srv import PlayNext, TogglePaused
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool

from xycar_map_nav.yellow_count_core import (
    count_band_bounds,
    W1BevYellowProjector,
    YellowBandPassCounter,
    yellow_mask_occupies_count_band,
)


YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"


class YellowCountBagViewer(Node):
    """Display freshly inferred lane classes around the left_4 trigger."""

    WAIT_LEFT = "WAIT LEFT_4 2/2"
    WAIT_ABSENCE = "LEFT_4 CONFIRMED - WAIT ABSENT 2/2"
    MODE_ON = "YELLOW_COUNT MODE ON - W1 BEV YELLOW"

    def __init__(self) -> None:
        super().__init__("yellow_count_bag_viewer")
        self.declare_parameter("display_enabled", True)
        self.declare_parameter("left_confidence", 0.40)
        self.declare_parameter("yellow_minimum_area_px", 20)
        self.declare_parameter("yellow_pass_component_minimum_area_px", 80)
        self.declare_parameter("yellow_pass_line_ratio", 0.68)
        self.declare_parameter("yellow_pass_band_half_height_px", 10)
        self.declare_parameter("yellow_visible_frames", 2)
        self.declare_parameter("yellow_absent_frames", 1)
        self.declare_parameter("yellow_pass_target", 1)
        self.declare_parameter(
            "camera_topic", "/wide_camera_mjpeg/image_raw/compressed"
        )
        self.declare_parameter(
            "detections_topic", "/my_rule/object_detections"
        )
        self.declare_parameter(
            "processing_enabled_topic", "/yellow_count/processing_enabled"
        )
        self.declare_parameter(
            "yellow_mask_topic", "/yellow_count/lraspp/yellow_mask"
        )

        best_effort = QoSProfile(
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
        self.display_enabled = bool(
            self.get_parameter("display_enabled").value
        )
        self.front: np.ndarray | None = None
        self.yellow_mask: np.ndarray | None = None
        self.yellow_raw_area = 0
        self.bev_projector = W1BevYellowProjector()
        self.front_stamp = math.nan
        self.mask_stamp = math.nan
        self.model_frames = 0
        self.left_boxes: list[tuple[int, int, int, int, float]] = []
        self.detection_messages = 0
        self.left_frames = 0
        self.absent_frames = 0
        self.phase = self.WAIT_LEFT
        self.mode_on_stamp = math.nan
        self.yellow_area = 0
        self.yellow_visible = False
        self.yellow_at_pass_line = False
        self.yellow_visible_frames = 0
        self.yellow_absent_frames = 0
        self.yellow_episode_active = False
        self.passed_yellow_segments = 0
        self.pass_counter = YellowBandPassCounter(
            target=int(self.get_parameter("yellow_pass_target").value),
            visible_frames=int(
                self.get_parameter("yellow_visible_frames").value
            ),
            absent_frames=int(
                self.get_parameter("yellow_absent_frames").value
            ),
        )
        self.steering_triggered = False
        self.steering_trigger_stamp = math.nan
        self.window_name = (
            "YELLOW_COUNT TRIGGER REVIEW | SPACE pause/play | "
            "M manual arm/reset | N next message | Q quit"
        )
        self.last_ui_time = 0.0

        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("camera_topic").value),
            self.on_camera,
            best_effort,
        )
        self.create_subscription(
            ObjectDetectionArray,
            str(self.get_parameter("detections_topic").value),
            self.on_detections,
            best_effort,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("yellow_mask_topic").value),
            self.on_yellow_mask,
            best_effort,
        )
        self.processing_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("processing_enabled_topic").value),
            state_qos,
        )
        self.toggle_client = self.create_client(
            TogglePaused, "/rosbag2_player/toggle_paused"
        )
        self.next_client = self.create_client(
            PlayNext, "/rosbag2_player/play_next"
        )
        self.create_timer(0.05, self.publish_processing_state)
        if self.display_enabled:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1800, 920)
            self.create_timer(0.03, self.update_ui)
        print(
            f"{CYAN}[YELLOW_COUNT] observation only: no steering and no "
            f"/xycar_motor output; W1-model yellow BEV inference starts "
            f"after left_4 absence{RESET}",
            flush=True,
        )

    @staticmethod
    def stamp_sec(message) -> float:
        return float(message.header.stamp.sec) + 1e-9 * float(
            message.header.stamp.nanosec
        )

    @staticmethod
    def normalized_class_name(value: str) -> str:
        return str(value).strip().lower().replace("-", "_").replace(" ", "_")

    def on_camera(self, message: CompressedImage) -> None:
        data = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            return
        self.front = image
        self.front_stamp = self.stamp_sec(message)

    def on_yellow_mask(self, message: Image) -> None:
        try:
            raw_yellow = self.bridge.imgmsg_to_cv2(message, "mono8")
            self.yellow_raw_area = int(np.count_nonzero(raw_yellow))
            self.yellow_mask = self.bev_projector.project(raw_yellow)
            self.mask_stamp = self.stamp_sec(message)
            self.model_frames += 1
        except Exception as exc:
            self.get_logger().warning(f"yellow-mask conversion failed: {exc}")
            return
        self.update_yellow_episode()

    def publish_processing_state(self) -> None:
        self.processing_publisher.publish(
            Bool(data=self.phase == self.MODE_ON)
        )

    def arm_yellow_count(self, source: str) -> None:
        """Start or reset observation without ever publishing motor output."""
        self.phase = self.MODE_ON
        self.mode_on_stamp = self.front_stamp
        self.yellow_visible_frames = 0
        self.yellow_absent_frames = 0
        self.yellow_episode_active = False
        self.passed_yellow_segments = 0
        self.pass_counter.reset()
        self.steering_triggered = False
        self.steering_trigger_stamp = math.nan
        print(
            f"{YELLOW}[YELLOW_COUNT {self.pass_counter.target}/"
            f"{self.pass_counter.target}] YELLOW_COUNT MODE ON "
            f"source={source} camera_stamp={self.front_stamp:.9f}{RESET}",
            flush=True,
        )
        self.processing_publisher.publish(Bool(data=True))

    def update_yellow_episode(self) -> None:
        """Count each yellow dash after it reaches and leaves a BEV pass line."""
        if self.yellow_mask is None:
            return
        mask = (self.yellow_mask > 0).astype(np.uint8)
        self.yellow_area = int(np.count_nonzero(mask))
        minimum_area = max(
            1, int(self.get_parameter("yellow_minimum_area_px").value)
        )
        self.yellow_visible = self.yellow_area >= minimum_area
        self.yellow_at_pass_line = yellow_mask_occupies_count_band(
            mask,
            line_ratio=float(
                self.get_parameter("yellow_pass_line_ratio").value
            ),
            half_height_px=int(
                self.get_parameter(
                    "yellow_pass_band_half_height_px"
                ).value
            ),
            component_minimum_area_px=int(
                self.get_parameter(
                    "yellow_pass_component_minimum_area_px"
                ).value
            ),
        )
        if self.phase != self.MODE_ON or self.steering_triggered:
            return
        entered, passed, triggered_now = self.pass_counter.update(
            self.yellow_at_pass_line
        )
        self.yellow_visible_frames = self.pass_counter.visible_frames
        self.yellow_absent_frames = self.pass_counter.absent_frames
        self.yellow_episode_active = self.pass_counter.episode_active
        self.passed_yellow_segments = self.pass_counter.passed
        if entered:
            print(
                f"{CYAN}[YELLOW_COUNT] yellow dash "
                f"{self.passed_yellow_segments + 1} entered the BEV "
                f"count band; waiting for it to leave{RESET}",
                flush=True,
            )
        if not passed:
            return

        target = max(1, int(self.get_parameter("yellow_pass_target").value))
        print(
            f"{YELLOW}[YELLOW_COUNT] YELLOW LINE DISAPPEARED "
            f"{self.passed_yellow_segments}/{target} "
            f"mask_stamp={self.mask_stamp:.9f}{RESET}",
            flush=True,
        )
        if not triggered_now:
            return

        self.steering_triggered = True
        self.steering_trigger_stamp = self.mask_stamp
        print(
            "[YELLOW_COUNT STEERING TRIGGER] YELLOW DASH "
            f"{target}/{target} DISAPPEARED -> START LEFT ENTRY STEERING NOW | "
            f"mask_stamp={self.mask_stamp:.9f} "
            f"camera_stamp={self.front_stamp:.9f}",
            flush=True,
        )

    def on_detections(self, message: ObjectDetectionArray) -> None:
        self.detection_messages += 1
        threshold = float(self.get_parameter("left_confidence").value)
        boxes = []
        for detection in message.detections:
            if self.normalized_class_name(detection.class_name) != "left_4":
                continue
            confidence = float(detection.confidence)
            if confidence < threshold:
                continue
            boxes.append(
                (
                    int(detection.xmin),
                    int(detection.ymin),
                    int(detection.xmax),
                    int(detection.ymax),
                    confidence,
                )
            )
        self.left_boxes = boxes
        left_seen = bool(boxes)

        if self.phase == self.WAIT_LEFT:
            self.left_frames = self.left_frames + 1 if left_seen else 0
            if self.left_frames >= 2:
                self.phase = self.WAIT_ABSENCE
                self.absent_frames = 0
                print(
                    f"{YELLOW}[YELLOW_COUNT 1/2] LEFT_4 DETECTED 2/2 "
                    f"camera_stamp={self.front_stamp:.9f}{RESET}",
                    flush=True,
                )
            return

        if self.phase == self.WAIT_ABSENCE:
            self.absent_frames = 0 if left_seen else self.absent_frames + 1
            if self.absent_frames >= 2:
                self.arm_yellow_count("LEFT_4_ABSENT_2/2")

    def front_overlay(self) -> np.ndarray | None:
        if self.front is None:
            return None
        image = self.front.copy()
        for xmin, ymin, xmax, ymax, confidence in self.left_boxes:
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 255), 4)
            cv2.putText(
                image,
                f"left_4 {confidence:.2f}",
                (xmin, max(28, ymin - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.85,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return image

    def yellow_overlay(self) -> tuple[np.ndarray | None, int]:
        if self.yellow_mask is None:
            return None, 0
        mask = (self.yellow_mask > 0).astype(np.uint8)
        canvas = np.full((*mask.shape, 3), 18, dtype=np.uint8)
        canvas[mask > 0] = (0, 235, 255)
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask, 8
        )
        components = []
        for index in range(1, count):
            x, y, width, height, area = stats[index]
            if int(area) < 12:
                continue
            components.append(
                (
                    float(centroids[index][1]),
                    int(x),
                    int(y),
                    int(width),
                    int(height),
                )
            )
        band_top, pass_line_y, band_bottom = count_band_bounds(
            mask.shape[0],
            line_ratio=float(
                self.get_parameter("yellow_pass_line_ratio").value
            ),
            half_height_px=int(
                self.get_parameter(
                    "yellow_pass_band_half_height_px"
                ).value
            ),
        )
        band_overlay = canvas.copy()
        cv2.rectangle(
            band_overlay,
            (0, band_top),
            (mask.shape[1] - 1, band_bottom),
            (90, 70, 0),
            -1,
        )
        canvas = cv2.addWeighted(canvas, 0.72, band_overlay, 0.28, 0.0)
        cv2.line(
            canvas,
            (0, band_top),
            (mask.shape[1] - 1, band_top),
            (255, 255, 0),
            1,
        )
        cv2.line(
            canvas,
            (0, band_bottom),
            (mask.shape[1] - 1, band_bottom),
            (255, 255, 0),
            1,
        )
        cv2.line(
            canvas,
            (0, pass_line_y),
            (mask.shape[1] - 1, pass_line_y),
            (255, 255, 0),
            2,
        )
        cv2.putText(
            canvas,
            "COUNT BAND: ENTER -> LEAVE",
            (8, max(22, band_top - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
        return canvas, len(components)

    @staticmethod
    def panel(
        image: np.ndarray | None,
        width: int,
        height: int,
        label: str,
    ) -> np.ndarray:
        canvas = np.full((height, width, 3), 18, dtype=np.uint8)
        if image is not None and image.size:
            available_height = height - 42
            scale = min(width / image.shape[1], available_height / image.shape[0])
            resized = cv2.resize(
                image,
                (
                    max(1, int(round(image.shape[1] * scale))),
                    max(1, int(round(image.shape[0] * scale))),
                ),
                interpolation=cv2.INTER_NEAREST,
            )
            x = (width - resized.shape[1]) // 2
            y = 42 + (available_height - resized.shape[0]) // 2
            canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
        cv2.putText(
            canvas,
            label,
            (12, 29),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (240, 240, 240),
            2,
            cv2.LINE_AA,
        )
        return canvas

    def compose(self) -> np.ndarray:
        yellow, visible_components = self.yellow_overlay()
        top = np.hstack(
            [
                self.panel(
                    self.front_overlay(), 900, 650, "RECORDED FRONT + LEFT_4"
                ),
                self.panel(
                    yellow,
                    900,
                    650,
                    "W1 INPUT BEV: YELLOW ONLY",
                ),
            ]
        )
        status = np.full((250, 1800, 3), 18, dtype=np.uint8)
        phase_color = (0, 255, 255) if self.phase != self.MODE_ON else (255, 255, 0)
        cv2.putText(
            status,
            self.phase,
            (30, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.05,
            phase_color,
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            status,
            (
                f"left frames {self.left_frames}/2   absent frames "
                f"{self.absent_frames}/2   detector messages "
                f"{self.detection_messages}   visible blobs "
                f"{visible_components} (not the pass count)   model frames "
                f"{self.model_frames}"
            ),
            (30, 112),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (230, 230, 230),
            2,
            cv2.LINE_AA,
        )
        elapsed = (
            max(0.0, self.front_stamp - self.mode_on_stamp)
            if self.phase == self.MODE_ON
            and math.isfinite(self.front_stamp)
            and math.isfinite(self.mode_on_stamp)
            else 0.0
        )
        cv2.putText(
            status,
            (
                f"W1 MODEL BEV YELLOW TEMPORAL PASSES "
                f"{self.passed_yellow_segments}/"
                f"{max(1, int(self.get_parameter('yellow_pass_target').value))}"
                f"   raw_area={self.yellow_raw_area}px "
                f"bev_area={self.yellow_area}px visible="
                f"{int(self.yellow_visible)} at_pass_line="
                f"{int(self.yellow_at_pass_line)} episode="
                f"{int(self.yellow_episode_active)}"
            ),
            (30, 165),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.67,
            (0, 220, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            status,
            (
                f"camera {self.front_stamp:.3f}   mode elapsed {elapsed:.3f}s   "
                "SPACE pause/play | M manual arm/reset | "
                "N next recorded message | Q quit"
            ),
            (30, 218),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (180, 220, 255),
            2,
            cv2.LINE_AA,
        )
        board = np.vstack([top, status])
        if self.steering_triggered:
            tinted = np.full_like(board, (150, 0, 150))
            board = cv2.addWeighted(board, 0.72, tinted, 0.28, 0.0)
            cv2.rectangle(
                board,
                (12, 12),
                (board.shape[1] - 13, board.shape[0] - 13),
                (255, 0, 255),
                32,
            )
        return board

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
        elif key in (ord("m"), ord("M")):
            self.arm_yellow_count("MANUAL_M_KEY")
        elif key in (ord("n"), ord("N"), 83):
            if self.next_client.service_is_ready():
                self.next_client.call_async(PlayNext.Request())
                print(f"{CYAN}[BAG] next recorded message{RESET}", flush=True)
        elif key in (27, ord("q"), ord("Q")):
            rclpy.shutdown()

    def stop(self) -> None:
        if self.display_enabled:
            cv2.destroyAllWindows()


def main() -> None:
    rclpy.init()
    node = YellowCountBagViewer()
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
