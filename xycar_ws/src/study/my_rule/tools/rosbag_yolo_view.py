#!/usr/bin/env python3
"""Display Ultralytics detections from a ROS 2 CompressedImage topic."""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from ultralytics import YOLO


class RosbagYoloView(Node):
    """Keep only the newest camera frame and draw every model class."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rosbag_yolo_view")
        self.args = args
        self.model = YOLO(str(args.model))
        self.lock = threading.Lock()
        self.latest_frame: tuple[bytes, int, int, str] | None = None
        self.received = 0
        self.processed = 0
        self.last_log_time = time.monotonic()

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.subscription_group = MutuallyExclusiveCallbackGroup()
        self.inference_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            CompressedImage,
            args.topic,
            self.on_image,
            input_qos,
            callback_group=self.subscription_group,
        )
        self.image_publisher = self.create_publisher(
            Image,
            args.output_topic,
            output_qos,
        )
        self.create_timer(
            1.0 / args.rate,
            self.on_inference,
            callback_group=self.inference_group,
        )
        self.get_logger().info(
            f"ready: model={args.model}, topic={args.topic}, "
            f"output={args.output_topic}, device={args.device}, "
            f"rate={args.rate:.1f}Hz"
        )

    def on_image(self, message: CompressedImage) -> None:
        with self.lock:
            self.latest_frame = (
                bytes(message.data),
                int(message.header.stamp.sec),
                int(message.header.stamp.nanosec),
                str(message.header.frame_id),
            )
            self.received += 1

    def on_inference(self) -> None:
        with self.lock:
            latest_frame = self.latest_frame
            self.latest_frame = None
        if latest_frame is None:
            if not self.args.no_window:
                cv2.waitKey(1)
            return
        jpeg, stamp_sec, stamp_nanosec, frame_id = latest_frame

        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("received an invalid JPEG frame")
            return

        started = time.perf_counter()
        results = self.model.predict(
            source=frame,
            imgsz=self.args.imgsz,
            conf=self.args.confidence,
            iou=self.args.iou,
            device=self.args.device,
            verbose=False,
        )
        result = results[0]
        annotated = np.ascontiguousarray(result.plot())
        detection_count = len(result.boxes) if result.boxes is not None else 0
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.processed += 1

        cv2.putText(
            annotated,
            (
                f"received={self.received} processed={self.processed} "
                f"detections={detection_count} inference={elapsed_ms:.1f}ms"
            ),
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        output = Image()
        output.header.stamp.sec = stamp_sec
        output.header.stamp.nanosec = stamp_nanosec
        output.header.frame_id = frame_id
        output.height = annotated.shape[0]
        output.width = annotated.shape[1]
        output.encoding = "bgr8"
        output.is_bigendian = False
        output.step = annotated.shape[1] * 3
        output.data = annotated.tobytes()
        self.image_publisher.publish(output)

        if not self.args.no_window:
            cv2.imshow("ROS bag YOLO model check - Q or ESC to quit", annotated)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                if rclpy.ok():
                    rclpy.shutdown()

        now = time.monotonic()
        if now - self.last_log_time >= 5.0:
            self.get_logger().info(
                f"received={self.received}, processed={self.processed}, "
                f"detections={detection_count}, inference={elapsed_ms:.1f}ms"
            )
            self.last_log_time = now

    def close(self) -> None:
        if not self.args.no_window:
            cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="View YOLO detections while a ROS 2 camera bag is playing."
    )
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument(
        "--topic",
        default="/wide_camera_mjpeg/image_raw/compressed",
    )
    parser.add_argument(
        "--output-topic",
        default="/model_check/annotated_image",
    )
    parser.add_argument("--device", default="0")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.70)
    parser.add_argument("--no-window", action="store_true")
    args = parser.parse_args()
    args.model = args.model.expanduser().resolve()
    if not args.model.is_file():
        parser.error(f"model does not exist: {args.model}")
    if args.rate <= 0:
        parser.error("--rate must be greater than zero")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = RosbagYoloView(args)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
