#!/usr/bin/env python3
"""ROS 2 ONNX lane-segmentation inference without motor outputs."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


EXPECTED_LABELS = {
    "whiteboundary": "white",
    "yellowcenterline": "yellow",
}


def normalized_label(name: str) -> str:
    return "".join(character for character in name.lower() if character.isalnum())


def class_roles(names: Any) -> dict[int, str]:
    items = names.items() if isinstance(names, dict) else enumerate(names)
    roles = {
        int(class_id): EXPECTED_LABELS.get(normalized_label(str(name)), "")
        for class_id, name in items
    }
    roles = {class_id: role for class_id, role in roles.items() if role}
    if set(roles.values()) != {"white", "yellow"}:
        raise RuntimeError(
            "lane model must contain white-boundary and yellow_centerline "
            f"classes; model names={names!r}"
        )
    return roles


def merge_instance_masks(
    masks: Any,
    classes: Any,
    confidences: Any,
    output_shape: tuple[int, int],
    roles: dict[int, str],
    *,
    confidence_threshold: float,
    role_confidence_thresholds: dict[str, float] | None = None,
    mask_threshold: float = 0.5,
) -> tuple[np.ndarray, np.ndarray, int, int]:
    height, width = output_shape
    white = np.zeros((height, width), dtype=np.uint8)
    yellow = np.zeros_like(white)
    counts = {"white": 0, "yellow": 0}

    mask_array = np.asarray(masks)
    class_array = np.asarray(classes).reshape(-1)
    confidence_array = np.asarray(confidences).reshape(-1)
    if mask_array.size == 0:
        return white, yellow, 0, 0
    if mask_array.ndim == 2:
        mask_array = mask_array[np.newaxis, ...]

    for mask, class_id, confidence in zip(
        mask_array, class_array, confidence_array
    ):
        role = roles.get(int(round(float(class_id))))
        role_threshold = (
            role_confidence_thresholds.get(role, confidence_threshold)
            if role is not None and role_confidence_thresholds is not None
            else confidence_threshold
        )
        if role is None or float(confidence) < role_threshold:
            continue
        if mask.shape != output_shape:
            mask = cv2.resize(
                mask.astype(np.float32),
                (width, height),
                interpolation=cv2.INTER_LINEAR,
            )
        binary = mask >= float(mask_threshold)
        if not np.any(binary):
            continue
        target = white if role == "white" else yellow
        target[binary] = 255
        counts[role] += 1

    return white, yellow, counts["white"], counts["yellow"]


class LaneSegInferenceNode(Node):
    """Publish semantic lane masks from the packaged fixed-size ONNX model."""

    def __init__(self) -> None:
        super().__init__("lane_seg_inference")
        package_share = Path(get_package_share_directory("lane_seg_control"))
        self.declare_parameter(
            "model_path", str(package_share / "models" / "best_512.onnx")
        )
        self.declare_parameter("image_topic", "/wide_camera/rect/image_raw")
        self.declare_parameter("processed_image_topic", "/lane_seg/source_image")
        self.declare_parameter(
            "white_mask_topic", "/lane_seg/white_boundary_mask"
        )
        self.declare_parameter(
            "yellow_mask_topic", "/lane_seg/yellow_centerline_mask"
        )
        self.declare_parameter("debug_topic", "/lane_seg/debug_image")
        self.declare_parameter(
            "perception_debug_topic", "/perception/yolo_debug_image"
        )
        self.declare_parameter("diagnostics_topic", "/lane_seg/diagnostics")
        self.declare_parameter("image_size", 512)
        self.declare_parameter("confidence", 0.20)
        self.declare_parameter("yellow_confidence", 0.40)
        self.declare_parameter("iou", 0.60)
        self.declare_parameter("max_detections", 30)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("cpu_threads", 4)
        self.declare_parameter("mask_threshold", 0.5)
        self.declare_parameter("debug_rate_hz", 1.0)

        self.bridge = CvBridge()
        self.image_size = int(self.get_parameter("image_size").value)
        self.confidence = float(self.get_parameter("confidence").value)
        self.yellow_confidence = float(
            self.get_parameter("yellow_confidence").value
        )
        self.role_confidence_thresholds = {
            "white": self.confidence,
            "yellow": self.yellow_confidence,
        }
        self.inference_confidence = min(
            self.role_confidence_thresholds.values()
        )
        self.iou = float(self.get_parameter("iou").value)
        self.max_detections = int(
            self.get_parameter("max_detections").value
        )
        self.device = str(self.get_parameter("device").value)
        self.mask_threshold = float(
            self.get_parameter("mask_threshold").value
        )
        self.debug_rate_hz = float(self.get_parameter("debug_rate_hz").value)

        import torch
        from ultralytics import YOLO

        cpu_threads = max(1, int(self.get_parameter("cpu_threads").value))
        if self.device.lower() == "cpu":
            torch.set_num_threads(cpu_threads)

        model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"lane segmentation model not found: {model_path}")
        self.model = YOLO(str(model_path), task="segment")
        self.roles = class_roles(self.model.names)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.white_pub = self.create_publisher(
            Image, str(self.get_parameter("white_mask_topic").value), sensor_qos
        )
        self.processed_image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("processed_image_topic").value),
            sensor_qos,
        )
        self.yellow_pub = self.create_publisher(
            Image, str(self.get_parameter("yellow_mask_topic").value), sensor_qos
        )
        self.debug_pub = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), sensor_qos
        )
        self.perception_debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter("perception_debug_topic").value),
            sensor_qos,
        )
        self.diagnostics_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            sensor_qos,
        )

        self.last_log_time = time.monotonic()
        self.last_debug_bucket: int | None = None
        self.processed_frames = 0
        self.get_logger().info(
            f"lane segmentation ready: model={model_path}, input={self.image_size}, "
            f"device={self.device}, roles={self.roles}, "
            f"thresholds={self.role_confidence_thresholds}"
        )

    def on_image(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"camera conversion failed: {exc}")
            return

        started = time.perf_counter()
        try:
            result = self.model.predict(
                source=frame,
                task="segment",
                imgsz=self.image_size,
                conf=self.inference_confidence,
                iou=self.iou,
                max_det=self.max_detections,
                device=self.device,
                retina_masks=True,
                verbose=False,
            )[0]
        except Exception as exc:
            self.get_logger().error(f"lane segmentation inference failed: {exc}")
            return

        if result.masks is None or result.boxes is None:
            masks = np.empty((0, *frame.shape[:2]), dtype=np.float32)
            classes = np.empty((0,), dtype=np.float32)
            confidences = np.empty((0,), dtype=np.float32)
        else:
            masks = result.masks.data.detach().cpu().numpy()
            classes = result.boxes.cls.detach().cpu().numpy()
            confidences = result.boxes.conf.detach().cpu().numpy()

        white, yellow, white_count, yellow_count = merge_instance_masks(
            masks,
            classes,
            confidences,
            frame.shape[:2],
            self.roles,
            confidence_threshold=self.confidence,
            role_confidence_thresholds=self.role_confidence_thresholds,
            mask_threshold=self.mask_threshold,
        )
        elapsed_ms = (time.perf_counter() - started) * 1000.0

        white_message = self.bridge.cv2_to_imgmsg(white, encoding="mono8")
        yellow_message = self.bridge.cv2_to_imgmsg(yellow, encoding="mono8")
        white_message.header = message.header
        yellow_message.header = message.header
        self.processed_image_pub.publish(message)
        self.white_pub.publish(white_message)
        self.yellow_pub.publish(yellow_message)

        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        debug_bucket = (
            int(stamp_ns * self.debug_rate_hz / 1_000_000_000)
            if stamp_ns > 0 and self.debug_rate_hz > 0.0
            else None
        )
        if (
            self.debug_pub.get_subscription_count() > 0
            or self.perception_debug_pub.get_subscription_count() > 0
        ) and debug_bucket is not None and debug_bucket != self.last_debug_bucket:
            accepted_indices = [
                index
                for index, (class_id, confidence) in enumerate(
                    zip(classes, confidences)
                )
                if (
                    self.roles.get(int(round(float(class_id)))) is not None
                    and float(confidence)
                    >= self.role_confidence_thresholds[
                        self.roles[int(round(float(class_id)))]
                    ]
                )
            ]
            debug = (
                result[accepted_indices].plot()
                if accepted_indices
                else frame.copy()
            )
            debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_message.header = message.header
            self.debug_pub.publish(debug_message)
            self.perception_debug_pub.publish(debug_message)
            self.last_debug_bucket = debug_bucket

        image_area = float(max(1, white.size))
        diagnostics = Float32MultiArray()
        diagnostics.data = [
            float(elapsed_ms),
            float(1000.0 / elapsed_ms if elapsed_ms > 0.0 else 0.0),
            float(white_count),
            float(yellow_count),
            float(np.count_nonzero(white) / image_area),
            float(np.count_nonzero(yellow) / image_area),
        ]
        self.diagnostics_pub.publish(diagnostics)
        self.processed_frames += 1

        now = time.monotonic()
        if now - self.last_log_time >= 5.0:
            self.get_logger().info(
                f"lane segmentation: {elapsed_ms:.1f}ms, "
                f"white={white_count}, yellow={yellow_count}"
            )
            self.last_log_time = now


def main() -> None:
    rclpy.init()
    node = LaneSegInferenceNode()
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
