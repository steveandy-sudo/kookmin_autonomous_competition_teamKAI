#!/usr/bin/env python3
"""Low-rate, latest-frame Ultralytics YOLO perception for mission semantics."""

from __future__ import annotations

import threading
import time
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from my_rule_msgs.msg import ObjectDetection, ObjectDetectionArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool

from my_rule.perception.camera_input import (
    CameraRectifier,
    decode_compressed_bgr,
)
from my_rule.perception.object_perception import (
    DetectionRecord,
    apply_class_aliases,
    filter_detections,
    green_hsv_evidence_in_box,
    normalize_class_name,
    parse_class_aliases,
)


class ObjectDetectionNode(Node):
    """Run object YOLO independently of the latency-critical lane loop."""

    def __init__(self) -> None:
        super().__init__("my_rule_object_detection_node")
        package_share = Path(get_package_share_directory("my_rule"))
        defaults = {
            "model_path": str(
                package_share
                / "models"
                / "kookmin_objects_best_20260804.pt"
            ),
            "image_topic": "/wide_camera_mjpeg/image_raw/compressed",
            "use_compressed_image": True,
            "detections_topic": "/my_rule/object_detections",
            "debug_topic": "/my_rule/object_detection/debug_image",
            "startup_green_topic": "/my_rule/start_signal_green",
            "camera_yaml": "",
            "enable_rectify": True,
            "rect_balance": 0.3,
            "inference_rate_hz": 3.0,
            "max_input_age_sec": 0.40,
            "input_size": 640,
            "device": "cpu",
            "half": False,
            "iou_threshold": 0.50,
            "max_detections": 50,
            "cpu_threads": 2,
            "opencv_threads": 1,
            "car_confidence": 0.45,
            "obstacle_vehicle_confidence": 0.45,
            "cone_confidence": 0.50,
            "red_confidence": 0.50,
            "yellow_confidence": 0.50,
            "green_confidence": 0.50,
            "red_4_confidence": 0.50,
            "yellow_4_confidence": 0.50,
            "green_4_confidence": 0.50,
            "yellow_centerline_confidence": 0.45,
            "left_4_confidence": 0.50,
            # A non-empty identity alias makes rclpy infer STRING_ARRAY;
            # launch YAML can then replace it with model-specific aliases.
            "class_aliases": ["car=car"],
            "startup_signal_hsv_enabled": True,
            "startup_signal_hsv_rate_hz": 20.0,
            "startup_signal_box_timeout_sec": 1.0,
            "startup_signal_min_confidence": 0.50,
            "startup_signal_max_center_y_ratio": 0.55,
            "startup_signal_min_area_ratio": 0.00005,
            "startup_green_hsv_lower": [70, 100, 120],
            "startup_green_hsv_upper": [90, 255, 255],
            "startup_green_min_pixels": 20,
            "startup_green_min_pixel_ratio": 0.015,
            "startup_green_required_frames": 2,
            "required_classes": [
                "car",
                "cone",
                "red",
                "yellow",
                "green",
                "red_4",
                "yellow_4",
                "green_4",
                "yellow_centerline",
                "left_4",
            ],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image: CompressedImage | Image | None = None
        self.last_processed_stamp_ns: int | None = None
        self.received_count = 0
        self.processed_count = 0
        self.dropped_stale_count = 0
        self.last_log_time = time.monotonic()
        self.rectify_lock = threading.Lock()
        self.startup_lock = threading.Lock()
        self.startup_signal_box: tuple[int, int, int, int] | None = None
        self.startup_signal_box_time = 0.0
        self.startup_green_frames = 0
        self.startup_green_latched = False
        self.last_startup_hsv_time = 0.0

        self.thresholds = {
            "car": self.parameter_float("car_confidence"),
            "obstacle_vehicle": self.parameter_float(
                "obstacle_vehicle_confidence"
            ),
            "cone": self.parameter_float("cone_confidence"),
            "red": self.parameter_float("red_confidence"),
            "yellow": self.parameter_float("yellow_confidence"),
            "green": self.parameter_float("green_confidence"),
            "red_4": self.parameter_float("red_4_confidence"),
            "yellow_4": self.parameter_float("yellow_4_confidence"),
            "green_4": self.parameter_float("green_4_confidence"),
            "yellow_centerline": self.parameter_float(
                "yellow_centerline_confidence"
            ),
            "left_4": self.parameter_float("left_4_confidence"),
        }
        self.class_aliases = parse_class_aliases(
            self.get_parameter("class_aliases").value
        )
        self.rectifier: CameraRectifier | None = None
        if bool(self.get_parameter("enable_rectify").value):
            camera_yaml = str(self.get_parameter("camera_yaml").value)
            self.rectifier = CameraRectifier(
                camera_yaml,
                self.parameter_float("rect_balance"),
            )

        import torch

        torch.set_num_threads(
            max(1, int(self.get_parameter("cpu_threads").value))
        )
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            pass
        cv2.setNumThreads(
            max(1, int(self.get_parameter("opencv_threads").value))
        )

        model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(
                f"object YOLO model not found: {model_path}"
            )
        try:
            from ultralytics import YOLO

            self.model = YOLO(str(model_path))
        except Exception as exc:
            raise RuntimeError(
                "failed to load the YOLO11 object model. Install an "
                "Ultralytics release that supports YOLO11/C3k2: "
                f"{exc}"
            ) from exc

        model_names = getattr(self.model, "names", {})
        if isinstance(model_names, dict):
            names = {
                normalize_class_name(value)
                for value in model_names.values()
            }
        else:
            names = {
                normalize_class_name(value)
                for value in list(model_names)
            }
        canonical_names = {
            self.class_aliases.get(name, name) for name in names
        }
        required = {
            normalize_class_name(value)
            for value in self.get_parameter("required_classes").value
        }
        missing = sorted(required - canonical_names)
        if missing:
            raise RuntimeError(
                f"object model is missing required classes {missing}; "
                f"raw={sorted(names)}, canonical={sorted(canonical_names)}"
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.detection_pub = self.create_publisher(
            ObjectDetectionArray,
            str(self.get_parameter("detections_topic").value),
            qos,
        )
        self.debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter("debug_topic").value),
            qos,
        )
        startup_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.startup_green_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("startup_green_topic").value),
            startup_qos,
        )
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.inference_group = MutuallyExclusiveCallbackGroup()
        image_type = (
            CompressedImage
            if bool(self.get_parameter("use_compressed_image").value)
            else Image
        )
        self.create_subscription(
            image_type,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            qos,
            callback_group=self.image_group,
        )
        rate_hz = max(
            0.2, self.parameter_float("inference_rate_hz")
        )
        self.create_timer(
            1.0 / rate_hz,
            self.on_inference_timer,
            callback_group=self.inference_group,
        )
        self.get_logger().info(
            f"object YOLO ready: model={model_path}, rate={rate_hz:.1f}Hz, "
            f"device={self.get_parameter('device').value}, "
            f"raw_classes={sorted(names)}, "
            f"canonical_classes={sorted(canonical_names)}"
        )

    def parameter_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    @staticmethod
    def message_stamp_ns(message: CompressedImage | Image) -> int:
        return (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    def input_age_sec(self, stamp_ns: int) -> float:
        if stamp_ns <= 0:
            return 0.0
        delta_ns = self.get_clock().now().nanoseconds - stamp_ns
        if delta_ns < 0 or delta_ns >= 60_000_000_000:
            return 0.0
        return delta_ns / 1.0e9

    def on_image(self, message: CompressedImage | Image) -> None:
        self.received_count += 1
        with self.lock:
            self.latest_image = message
        self.process_startup_signal_hsv(message)

    def decode_and_rectify(self, message: CompressedImage | Image):
        if isinstance(message, CompressedImage):
            frame = decode_compressed_bgr(message.data)
        else:
            try:
                frame = self.bridge.imgmsg_to_cv2(
                    message, desired_encoding="bgr8"
                )
            except Exception:
                return None
        if frame is None:
            return None
        if self.rectifier is not None:
            with self.rectify_lock:
                frame = self.rectifier.rectify(frame)
        return frame

    def update_startup_signal_box(
        self,
        records: list[DetectionRecord],
        image_width: int,
        image_height: int,
    ) -> None:
        """Cache the exact full-housing YOLO box used by startup HSV."""
        if (
            not bool(
                self.get_parameter("startup_signal_hsv_enabled").value
            )
            or image_width <= 0
            or image_height <= 0
        ):
            return
        minimum_confidence = self.parameter_float(
            "startup_signal_min_confidence"
        )
        maximum_center_y = (
            self.parameter_float("startup_signal_max_center_y_ratio")
            * image_height
        )
        minimum_area = (
            self.parameter_float("startup_signal_min_area_ratio")
            * image_width
            * image_height
        )
        candidates = [
            record
            for record in records
            if record.class_name in {"red", "yellow", "green"}
            and record.confidence >= minimum_confidence
            and record.center_y <= maximum_center_y
            and record.area >= minimum_area
        ]
        if not candidates:
            return
        selected = max(candidates, key=lambda record: record.confidence)
        with self.startup_lock:
            if self.startup_green_latched:
                return
            self.startup_signal_box = (
                selected.xmin,
                selected.ymin,
                selected.xmax,
                selected.ymax,
            )
            self.startup_signal_box_time = time.monotonic()

    def process_startup_signal_hsv(
        self,
        message: CompressedImage | Image,
    ) -> None:
        """Confirm startup green at camera rate inside the cached YOLO box."""
        if not bool(
            self.get_parameter("startup_signal_hsv_enabled").value
        ):
            return
        now = time.monotonic()
        rate_hz = max(
            1.0, self.parameter_float("startup_signal_hsv_rate_hz")
        )
        with self.startup_lock:
            if self.startup_green_latched:
                return
            if now - self.last_startup_hsv_time < 1.0 / rate_hz:
                return
            self.last_startup_hsv_time = now
            box = self.startup_signal_box
            box_age = (
                now - self.startup_signal_box_time
                if self.startup_signal_box_time > 0.0
                else float("inf")
            )
        timeout = max(
            0.0, self.parameter_float("startup_signal_box_timeout_sec")
        )
        if box is None or (timeout > 0.0 and box_age > timeout):
            with self.startup_lock:
                self.startup_green_frames = 0
            return

        frame = self.decode_and_rectify(message)
        if frame is None:
            return
        evidence = green_hsv_evidence_in_box(
            frame,
            box,
            lower_hsv=list(
                self.get_parameter("startup_green_hsv_lower").value
            ),
            upper_hsv=list(
                self.get_parameter("startup_green_hsv_upper").value
            ),
        )
        green_seen = (
            evidence.valid
            and evidence.green_pixels
            >= int(self.get_parameter("startup_green_min_pixels").value)
            and evidence.pixel_ratio
            >= self.parameter_float("startup_green_min_pixel_ratio")
        )
        required = max(
            1,
            int(self.get_parameter("startup_green_required_frames").value),
        )
        publish_green = False
        with self.startup_lock:
            if self.startup_green_latched:
                return
            self.startup_green_frames = (
                self.startup_green_frames + 1 if green_seen else 0
            )
            if self.startup_green_frames >= required:
                self.startup_green_latched = True
                publish_green = True
        if publish_green:
            output = Bool()
            output.data = True
            self.startup_green_pub.publish(output)
            self.get_logger().info(
                "startup green confirmed by HSV inside the YOLO box: "
                f"pixels={evidence.green_pixels}, "
                f"ratio={evidence.pixel_ratio:.3f}, frames={required}"
            )

    def on_inference_timer(self) -> None:
        with self.lock:
            message = self.latest_image
        if message is None:
            return
        stamp_ns = self.message_stamp_ns(message)
        if stamp_ns == self.last_processed_stamp_ns:
            return
        self.last_processed_stamp_ns = stamp_ns
        max_age = max(
            0.0, self.parameter_float("max_input_age_sec")
        )
        if max_age > 0.0 and self.input_age_sec(stamp_ns) > max_age:
            self.dropped_stale_count += 1
            return

        started = time.perf_counter()
        frame = self.decode_and_rectify(message)
        if frame is None:
            self.get_logger().error("object YOLO received an invalid JPEG")
            return
        height, width = frame.shape[:2]

        try:
            results = self.model.predict(
                source=frame,
                imgsz=int(self.get_parameter("input_size").value),
                conf=min(self.thresholds.values()),
                iou=self.parameter_float("iou_threshold"),
                max_det=int(self.get_parameter("max_detections").value),
                device=str(self.get_parameter("device").value),
                half=bool(self.get_parameter("half").value),
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(f"object YOLO inference failed: {exc}")
            return

        records: list[DetectionRecord] = []
        if results:
            result = results[0]
            boxes = getattr(result, "boxes", None)
            if boxes is not None and len(boxes) > 0:
                xyxy = boxes.xyxy.detach().cpu().numpy()
                confidences = boxes.conf.detach().cpu().numpy()
                class_ids = boxes.cls.detach().cpu().numpy().astype(int)
                names = getattr(
                    result,
                    "names",
                    getattr(self.model, "names", {}),
                )
                for coordinates, score, class_id in zip(
                    xyxy, confidences, class_ids
                ):
                    if isinstance(names, dict):
                        class_name = str(names.get(int(class_id), class_id))
                    else:
                        class_name = str(names[int(class_id)])
                    xmin, ymin, xmax, ymax = coordinates.tolist()
                    records.append(
                        DetectionRecord(
                            class_name=class_name,
                            class_id=int(class_id),
                            confidence=float(score),
                            xmin=int(max(0, min(width - 1, round(xmin)))),
                            ymin=int(max(0, min(height - 1, round(ymin)))),
                            xmax=int(max(0, min(width, round(xmax)))),
                            ymax=int(max(0, min(height, round(ymax)))),
                        )
                    )
        accepted = filter_detections(
            apply_class_aliases(records, self.class_aliases),
            self.thresholds,
        )
        self.update_startup_signal_box(accepted, width, height)
        output = ObjectDetectionArray()
        output.header = message.header
        output.image_width = width
        output.image_height = height
        for record in accepted:
            detection = ObjectDetection()
            detection.class_name = record.class_name
            detection.class_id = record.class_id
            detection.confidence = record.confidence
            detection.xmin = record.xmin
            detection.ymin = record.ymin
            detection.xmax = record.xmax
            detection.ymax = record.ymax
            output.detections.append(detection)
        self.detection_pub.publish(output)
        self.processed_count += 1

        if self.debug_pub.get_subscription_count() > 0:
            debug = frame.copy()
            for record in accepted:
                cv2.rectangle(
                    debug,
                    (record.xmin, record.ymin),
                    (record.xmax, record.ymax),
                    (0, 220, 255),
                    2,
                )
                cv2.putText(
                    debug,
                    f"{record.class_name} {record.confidence:.2f}",
                    (record.xmin, max(16, record.ymin - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 220, 255),
                    2,
                    cv2.LINE_AA,
                )
            debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_message.header = message.header
            self.debug_pub.publish(debug_message)

        now = time.monotonic()
        if now - self.last_log_time >= 5.0:
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self.get_logger().info(
                f"object YOLO: detections={len(accepted)}, "
                f"last={elapsed_ms:.1f}ms, received={self.received_count}, "
                f"processed={self.processed_count}, "
                f"stale={self.dropped_stale_count}"
            )
            self.last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectDetectionNode()
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
