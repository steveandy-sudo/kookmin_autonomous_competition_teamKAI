"""Publish a synchronized-looking 2x2 image mosaic for rosbag diagnosis."""

from __future__ import annotations

from dataclasses import dataclass
import re

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


@dataclass
class Pane:
    label: str
    topic: str
    message: Image | None = None


def _stamp_sec(message: Image | None) -> float | None:
    if message is None:
        return None
    return float(message.header.stamp.sec) + 1.0e-9 * float(
        message.header.stamp.nanosec
    )


def _stamp_ns(message: Image | None) -> int | None:
    if message is None:
        return None
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


def _letterbox(image: np.ndarray, width: int, height: int) -> np.ndarray:
    canvas = np.full((height, width, 3), 24, dtype=np.uint8)
    if image.size == 0:
        return canvas
    scale = min(float(width) / image.shape[1], float(height) / image.shape[0])
    resized_width = max(1, int(round(image.shape[1] * scale)))
    resized_height = max(1, int(round(image.shape[0] * scale)))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_AREA,
    )
    left = (width - resized_width) // 2
    top = (height - resized_height) // 2
    canvas[top : top + resized_height, left : left + resized_width] = resized
    return canvas


class ReplayDiagnosticMosaic(Node):
    def __init__(self) -> None:
        super().__init__("replay_diagnostic_mosaic")
        self.declare_parameter("pane_width", 640)
        self.declare_parameter("pane_height", 360)
        self.declare_parameter("output_rate_hz", 10.0)
        self.declare_parameter("output_topic", "/replay/diagnostic_mosaic")
        self.declare_parameter("start_offset_sec", 0.0)
        self.declare_parameter("bag_start_time_ns", 0)
        self.bridge = CvBridge()
        self.panes = [
            Pane("RECTIFIED CAMERA", "/replay/camera"),
            Pane("YOLO OBJECTS", "/replay/object_detection"),
            Pane("CANONICAL", "/replay/canonical"),
            Pane("RULE DEBUG", "/replay/rule_debug"),
        ]
        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self._subscriptions = [
            self.create_subscription(
                Image,
                pane.topic,
                lambda message, index=index: self._on_image(index, message),
                input_qos,
            )
            for index, pane in enumerate(self.panes)
        ]
        self._subscriptions.extend(
            (
                self.create_subscription(
                    String,
                    "/hybrid_gate/status",
                    self._on_status,
                    input_qos,
                ),
                self.create_subscription(
                    Bool,
                    "/hybrid_gate/drive_armed",
                    self._on_drive_armed,
                    input_qos,
                ),
            )
        )
        self.publisher = self.create_publisher(
            Image,
            str(self.get_parameter("output_topic").value),
            output_qos,
        )
        self.last_camera_stamp: float | None = None
        self.first_camera_stamp: float | None = None
        self.status_text = "NO STATUS"
        self.drive_armed = False
        rate_hz = max(1.0, float(self.get_parameter("output_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)

    def _on_image(self, index: int, message: Image) -> None:
        self.panes[index].message = message

    def _on_status(self, message: String) -> None:
        self.status_text = str(message.data)

    def _on_drive_armed(self, message: Bool) -> None:
        self.drive_armed = bool(message.data)

    def _mode_banner(self, width: int) -> np.ndarray:
        source_match = re.search(r"\bsource=(\S+)", self.status_text)
        target_match = re.search(r"\btarget_waypoint=(\S+)", self.status_text)
        command_match = re.search(
            r"\bcmd=\[([+-]?[\d.]+),([+-]?[\d.]+)\]",
            self.status_text,
        )
        source = source_match.group(1) if source_match else "UNKNOWN"
        labels = {
            "RL": "MODEL",
            "RULE": "RULE",
            "CONE_RULE": "CONE DRIVING",
            "YOLO_LIDAR_AVOIDANCE": "AVOIDANCE",
        }
        selected_mode = labels.get(source, source)
        mode = selected_mode if self.drive_armed else "STOPPED"
        target = target_match.group(1) if target_match else "?"
        command = (
            f"steer={command_match.group(1)} speed={command_match.group(2)}"
            if command_match
            else "steer=? speed=?"
        )
        colors = {
            "MODEL": (210, 130, 40),
            "RULE": (50, 210, 80),
            "CONE DRIVING": (0, 170, 255),
            "AVOIDANCE": (40, 60, 245),
            "STOPPED": (180, 180, 180),
        }
        color = colors.get(mode, (180, 180, 180))
        banner = np.full((52, width, 3), 18, dtype=np.uint8)
        cv2.putText(
            banner,
            f"MODE: {mode} | TARGET: {target} | {command}",
            (14, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.88,
            color,
            2,
            cv2.LINE_AA,
        )
        return banner

    def _render_pane(
        self,
        pane: Pane,
        reference_stamp: float,
        bag_time_sec: float,
        width: int,
        height: int,
    ) -> np.ndarray:
        if pane.message is None:
            image = np.empty((0, 0, 3), dtype=np.uint8)
        else:
            try:
                image = self.bridge.imgmsg_to_cv2(
                    pane.message,
                    desired_encoding="bgr8",
                )
            except Exception as error:  # cv_bridge reports encoding failures here.
                self.get_logger().warning(str(error), throttle_duration_sec=2.0)
                image = np.empty((0, 0, 3), dtype=np.uint8)
        output = _letterbox(image, width, height)
        stamp = _stamp_sec(pane.message)
        suffix = "NO DATA" if stamp is None else f"dt={stamp-reference_stamp:+.3f}s"
        cv2.rectangle(output, (0, 0), (width, 34), (18, 18, 18), -1)
        cv2.putText(
            output,
            f"{pane.label} | bag_t={bag_time_sec:.2f}s | {suffix}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (40, 235, 255),
            1,
            cv2.LINE_AA,
        )
        return output

    def _publish(self) -> None:
        camera = self.panes[0].message
        camera_stamp = _stamp_sec(camera)
        if camera is None or camera_stamp is None:
            return
        if self.last_camera_stamp == camera_stamp:
            return
        self.last_camera_stamp = camera_stamp
        if self.first_camera_stamp is None:
            self.first_camera_stamp = camera_stamp
        bag_start_time_ns = int(
            self.get_parameter("bag_start_time_ns").value
        )
        camera_stamp_ns = _stamp_ns(camera)
        if bag_start_time_ns > 0 and camera_stamp_ns is not None:
            bag_time_sec = (
                camera_stamp_ns - bag_start_time_ns
            ) / 1.0e9
        else:
            bag_time_sec = float(
                self.get_parameter("start_offset_sec").value
            ) + (camera_stamp - self.first_camera_stamp)
        width = max(240, int(self.get_parameter("pane_width").value))
        height = max(160, int(self.get_parameter("pane_height").value))
        rendered = [
            self._render_pane(
                pane,
                camera_stamp,
                bag_time_sec,
                width,
                height,
            )
            for pane in self.panes
        ]
        mosaic = np.vstack(
            (np.hstack(rendered[:2]), np.hstack(rendered[2:]))
        )
        mosaic = np.vstack((self._mode_banner(mosaic.shape[1]), mosaic))
        output = self.bridge.cv2_to_imgmsg(mosaic, encoding="bgr8")
        output.header = camera.header
        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = ReplayDiagnosticMosaic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
