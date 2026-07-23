#!/usr/bin/env python3
"""ROS 2 TorchScript LR-ASPP lane segmentation without motor outputs."""

from __future__ import annotations

import time
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


IMAGENET_MEAN = np.asarray((0.485, 0.456, 0.406), dtype=np.float32)
IMAGENET_STD = np.asarray((0.229, 0.224, 0.225), dtype=np.float32)


def prepare_model_input(
    frame: np.ndarray, width: int, height: int
) -> np.ndarray:
    """Return a contiguous ImageNet-normalized RGB NCHW tensor array."""
    resized = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    normalized = (rgb - IMAGENET_MEAN) / IMAGENET_STD
    return np.ascontiguousarray(normalized.transpose(2, 0, 1)[np.newaxis, ...])


def masks_from_probabilities(
    probabilities: np.ndarray,
    *,
    white_class_id: int = 1,
    yellow_class_id: int = 2,
    white_confidence: float = 0.5,
    yellow_confidence: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert CxHxW semantic probabilities into disjoint lane masks."""
    if probabilities.ndim != 3:
        raise ValueError(
            f"semantic probabilities must be CxHxW, got {probabilities.shape}"
        )
    class_count = probabilities.shape[0]
    if not 0 <= white_class_id < class_count:
        raise ValueError(
            f"white class {white_class_id} is outside {class_count} classes"
        )
    if not 0 <= yellow_class_id < class_count:
        raise ValueError(
            f"yellow class {yellow_class_id} is outside {class_count} classes"
        )

    labels = np.argmax(probabilities, axis=0)
    confidence = np.max(probabilities, axis=0)
    white = (
        (labels == int(white_class_id))
        & (confidence >= float(white_confidence))
    ).astype(np.uint8) * 255
    yellow = (
        (labels == int(yellow_class_id))
        & (confidence >= float(yellow_confidence))
    ).astype(np.uint8) * 255
    return white, yellow


class LrasppInferenceNode(Node):
    """Publish semantic lane masks from a TorchScript LR-ASPP model."""

    def __init__(self) -> None:
        super().__init__("lane_seg_lraspp_inference")
        package_share = Path(get_package_share_directory("lane_seg_control"))
        self.declare_parameter(
            "model_path",
            str(
                package_share
                / "models"
                / "kookmin_lane_lraspp_mbv3s_256x144.pt"
            ),
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
        self.declare_parameter("input_width", 256)
        self.declare_parameter("input_height", 144)
        self.declare_parameter("white_class_id", 1)
        self.declare_parameter("yellow_class_id", 2)
        self.declare_parameter("white_confidence", 0.5)
        self.declare_parameter("yellow_confidence", 0.5)
        self.declare_parameter("cpu_threads", 4)
        self.declare_parameter("opencv_threads", 1)
        self.declare_parameter("output_qos_depth", 1)
        self.declare_parameter("debug_rate_hz", 1.0)
        self.declare_parameter("max_output_rate_hz", 0.0)
        self.declare_parameter("output_native_resolution", True)

        self.bridge = CvBridge()
        self.input_width = int(self.get_parameter("input_width").value)
        self.input_height = int(self.get_parameter("input_height").value)
        self.white_class_id = int(self.get_parameter("white_class_id").value)
        self.yellow_class_id = int(self.get_parameter("yellow_class_id").value)
        self.white_confidence = float(
            self.get_parameter("white_confidence").value
        )
        self.yellow_confidence = float(
            self.get_parameter("yellow_confidence").value
        )
        self.debug_rate_hz = float(self.get_parameter("debug_rate_hz").value)
        self.max_output_rate_hz = max(
            0.0, float(self.get_parameter("max_output_rate_hz").value)
        )
        self.output_native_resolution = bool(
            self.get_parameter("output_native_resolution").value
        )

        import torch

        self.torch = torch
        cpu_threads = max(1, int(self.get_parameter("cpu_threads").value))
        opencv_threads = max(1, int(self.get_parameter("opencv_threads").value))
        torch.set_num_threads(cpu_threads)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            pass
        cv2.setNumThreads(opencv_threads)

        model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(
                f"LR-ASPP TorchScript model not found: {model_path}"
            )
        model = torch.jit.load(str(model_path), map_location="cpu").eval()
        self.model = torch.jit.optimize_for_inference(model)
        warmup = torch.zeros(
            (1, 3, self.input_height, self.input_width), dtype=torch.float32
        )
        with torch.inference_mode():
            output = self.model(warmup)
        if not isinstance(output, torch.Tensor) or output.ndim != 4:
            raise RuntimeError(
                "LR-ASPP model must return an NxCxHxW tensor, "
                f"got {type(output)!r}"
            )
        if output.shape[1] <= max(self.white_class_id, self.yellow_class_id):
            raise RuntimeError(
                f"LR-ASPP model has {output.shape[1]} classes but lane class IDs "
                f"are {self.white_class_id}/{self.yellow_class_id}"
            )

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(self.get_parameter("output_qos_depth").value)),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(self.get_parameter("output_qos_depth").value)),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_callback_group = MutuallyExclusiveCallbackGroup()
        self.inference_callback_group = MutuallyExclusiveCallbackGroup()
        self.processed_image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("processed_image_topic").value),
            output_qos,
        )
        self.white_pub = self.create_publisher(
            Image,
            str(self.get_parameter("white_mask_topic").value),
            output_qos,
        )
        self.yellow_pub = self.create_publisher(
            Image,
            str(self.get_parameter("yellow_mask_topic").value),
            output_qos,
        )
        self.debug_pub = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), input_qos
        )
        self.perception_debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter("perception_debug_topic").value),
            input_qos,
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
            input_qos,
            callback_group=self.image_callback_group,
        )

        self.last_log_time = time.monotonic()
        self.last_debug_bucket: int | None = None
        self.latest_image: Image | None = None
        self.last_processed_stamp_ns: int | None = None
        self.received_image_count = 0
        self.scheduler_tick_count = 0
        self.scheduler_stop = threading.Event()
        self.scheduler_thread: threading.Thread | None = None
        if self.max_output_rate_hz > 0.0:
            self.scheduler_thread = threading.Thread(
                target=self.run_output_scheduler,
                name="lane_seg_7hz_scheduler",
                daemon=True,
            )
            self.scheduler_thread.start()
        self.get_logger().info(
            f"LR-ASPP lane segmentation ready: model={model_path}, "
            f"input={self.input_width}x{self.input_height}, classes="
            f"background/white/yellow=0/{self.white_class_id}/{self.yellow_class_id}, "
            f"thresholds={self.white_confidence:.2f}/{self.yellow_confidence:.2f}, "
            f"threads=torch:{cpu_threads},opencv:{opencv_threads}, "
            f"output={'native' if self.output_native_resolution else 'model'}, "
            f"scheduler={'latest-frame timer' if self.max_output_rate_hz > 0.0 else 'input'}, "
            f"rate_limit={self.max_output_rate_hz:.1f}Hz"
        )

    def on_image(self, message: Image) -> None:
        self.received_image_count += 1
        self.latest_image = message
        if self.max_output_rate_hz <= 0.0:
            self.process_image(message)

    def on_output_timer(self) -> None:
        message = self.latest_image
        if message is None:
            return
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if self.last_processed_stamp_ns == stamp_ns:
            return
        if (
            self.last_processed_stamp_ns is not None
            and stamp_ns < self.last_processed_stamp_ns
        ):
            self.last_debug_bucket = None
        self.process_image(message)

    def run_output_scheduler(self) -> None:
        period = 1.0 / self.max_output_rate_hz
        deadline = time.monotonic() + period
        while not self.scheduler_stop.is_set():
            wait_sec = max(0.0, deadline - time.monotonic())
            if self.scheduler_stop.wait(wait_sec):
                break
            self.scheduler_tick_count += 1
            self.on_output_timer()
            deadline += period
            if deadline < time.monotonic() - period:
                deadline = time.monotonic() + period

    def destroy_node(self):
        self.scheduler_stop.set()
        if self.scheduler_thread is not None:
            self.scheduler_thread.join(timeout=2.0)
        return super().destroy_node()

    def process_image(self, message: Image) -> None:
        callback_started = time.perf_counter()
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"camera conversion failed: {exc}")
            return

        started = time.perf_counter()
        model_input = prepare_model_input(
            frame, self.input_width, self.input_height
        )
        try:
            tensor = self.torch.from_numpy(model_input)
            with self.torch.inference_mode():
                logits = self.model(tensor)
                probabilities = self.torch.softmax(logits, dim=1)[0].cpu().numpy()
        except Exception as exc:
            self.get_logger().error(f"LR-ASPP inference failed: {exc}")
            return

        white_small, yellow_small = masks_from_probabilities(
            probabilities,
            white_class_id=self.white_class_id,
            yellow_class_id=self.yellow_class_id,
            white_confidence=self.white_confidence,
            yellow_confidence=self.yellow_confidence,
        )
        if self.output_native_resolution:
            output_frame = frame
            output_size = (frame.shape[1], frame.shape[0])
            white = cv2.resize(
                white_small, output_size, interpolation=cv2.INTER_NEAREST
            )
            yellow = cv2.resize(
                yellow_small, output_size, interpolation=cv2.INTER_NEAREST
            )
        else:
            output_frame = cv2.resize(
                frame,
                (self.input_width, self.input_height),
                interpolation=cv2.INTER_AREA,
            )
            white = white_small
            yellow = yellow_small
        elapsed_ms = (time.perf_counter() - started) * 1000.0

        processed_message = self.bridge.cv2_to_imgmsg(
            output_frame, encoding="bgr8"
        )
        white_message = self.bridge.cv2_to_imgmsg(white, encoding="mono8")
        yellow_message = self.bridge.cv2_to_imgmsg(yellow, encoding="mono8")
        processed_message.header = message.header
        white_message.header = message.header
        yellow_message.header = message.header
        self.processed_image_pub.publish(processed_message)
        self.white_pub.publish(white_message)
        self.yellow_pub.publish(yellow_message)
        self.last_processed_stamp_ns = stamp_ns

        debug_bucket = (
            int(stamp_ns * self.debug_rate_hz / 1_000_000_000)
            if stamp_ns > 0 and self.debug_rate_hz > 0.0
            else None
        )
        if (
            self.debug_pub.get_subscription_count() > 0
            or self.perception_debug_pub.get_subscription_count() > 0
        ) and debug_bucket is not None and debug_bucket != self.last_debug_bucket:
            overlay = np.zeros_like(output_frame)
            overlay[white > 0] = (255, 255, 255)
            overlay[yellow > 0] = (0, 220, 255)
            selected = (white > 0) | (yellow > 0)
            debug = output_frame.copy()
            blended = cv2.addWeighted(
                output_frame, 0.45, overlay, 0.55, 0.0
            )
            debug[selected] = blended[selected]
            cv2.putText(
                debug,
                f"LR-ASPP {elapsed_ms:.1f}ms",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (40, 40, 255),
                2,
                cv2.LINE_AA,
            )
            debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_message.header = message.header
            self.debug_pub.publish(debug_message)
            self.perception_debug_pub.publish(debug_message)
            self.last_debug_bucket = debug_bucket

        white_components = max(0, cv2.connectedComponents(white_small)[0] - 1)
        yellow_components = max(0, cv2.connectedComponents(yellow_small)[0] - 1)
        callback_elapsed_ms = (
            time.perf_counter() - callback_started
        ) * 1000.0
        small_area = float(max(1, white_small.size))
        diagnostics = Float32MultiArray()
        diagnostics.data = [
            float(elapsed_ms),
            float(1000.0 / elapsed_ms if elapsed_ms > 0.0 else 0.0),
            float(white_components),
            float(yellow_components),
            float(np.count_nonzero(white_small) / small_area),
            float(np.count_nonzero(yellow_small) / small_area),
            float(callback_elapsed_ms),
            float(self.received_image_count),
            float(self.scheduler_tick_count),
        ]
        self.diagnostics_pub.publish(diagnostics)

        now = time.monotonic()
        if now - self.last_log_time >= 5.0:
            self.get_logger().info(
                f"LR-ASPP lane segmentation: {elapsed_ms:.1f}ms, "
                f"white_px={np.count_nonzero(white_small)}, "
                f"yellow_px={np.count_nonzero(yellow_small)}"
            )
            self.last_log_time = now


def main() -> None:
    rclpy.init()
    node = LrasppInferenceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
