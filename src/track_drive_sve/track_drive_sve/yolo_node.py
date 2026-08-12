#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO 차량 탐지 노드.

front/left/right/behind 4개 카메라를 받아 best.pt로 차량을 탐지하고,
각 방향의 안정 추적(persistent) 개수를 /yolo_obstacle/stable_counts 로 발행한다.
회피 노드(obstacle_avoidance_node)가 이 카운트를 받아 회피 offset을 만든다.

원본(YoloObstacleQuadViewer) 알고리즘 그대로이며, best.pt 경로만 패키지 기준으로 수정했다.
show_window 파라미터로 4분할 창 표시를 끌 수 있다(false면 합성/imshow를 건너뛰어 부하 절감).
"""

import cv2
import numpy as np
import rclpy

from pathlib import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from ultralytics import YOLO


# best.pt는 이 Python 패키지 폴더에 함께 둔다.
_PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL = str(_PACKAGE_DIR / "best.pt")
DEFAULT_TOPICS = {
    "front": "/image_raw",
    "left": "/usb_cam/image_raw/left",
    "right": "/usb_cam/image_raw/right",
    "behind": "/usb_cam/image_raw/behind",
}


def resolve_model_path(model_value):
    """ROS 파라미터로 받은 모델 경로를 현재 패키지/실행 위치 기준으로 찾는다."""
    raw_path = Path(str(model_value)).expanduser()
    candidates = [raw_path] if raw_path.is_absolute() else [
        _PACKAGE_DIR / raw_path,
        Path.cwd() / raw_path,
    ]

    for candidate in candidates:
        if candidate.is_file():
            return str(candidate.resolve())

    searched = ", ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(
        f"YOLO model file not found: {model_value!r}. Searched: {searched}"
    )


class YoloObstacleQuadViewer(Node):
    def __init__(self):
        super().__init__("yolo_obstacle_quad_viewer")

        self.declare_parameter("model", DEFAULT_MODEL)
        self.declare_parameter("front_topic", DEFAULT_TOPICS["front"])
        self.declare_parameter("left_topic", DEFAULT_TOPICS["left"])
        self.declare_parameter("right_topic", DEFAULT_TOPICS["right"])
        self.declare_parameter("behind_topic", DEFAULT_TOPICS["behind"])
        self.declare_parameter("conf", 0.80)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("device", "0")
        self.declare_parameter("tile_width", 480)
        self.declare_parameter("tile_height", 360)
        self.declare_parameter("show_window", True)
        self.declare_parameter("log_every_n", 30)
        self.declare_parameter("min_area_ratio", 0.003)
        self.declare_parameter("front_roi", "0.05,0.22,0.95,0.95")
        self.declare_parameter("left_roi", "0.00,0.18,0.95,0.90")
        self.declare_parameter("right_roi", "0.05,0.18,1.00,0.90")
        self.declare_parameter("behind_roi", "0.00,0.18,1.00,0.90")
        self.declare_parameter("min_persistent_frames", 6)
        self.declare_parameter("max_missed_frames", 3)
        self.declare_parameter("track_iou", 0.25)

        model_path = resolve_model_path(self.get_parameter("model").value)
        self.conf = float(self.get_parameter("conf").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.tile_width = int(self.get_parameter("tile_width").value)
        self.tile_height = int(self.get_parameter("tile_height").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)
        self.min_area_ratio = float(self.get_parameter("min_area_ratio").value)
        self.min_persistent_frames = int(
            self.get_parameter("min_persistent_frames").value
        )
        self.max_missed_frames = int(self.get_parameter("max_missed_frames").value)
        self.track_iou = float(self.get_parameter("track_iou").value)

        self.model = YOLO(model_path)
        self.frames = {}
        self.detection_counts = {}
        self.tracks = {}
        self.frame_count = 0

        self.count_pub = self.create_publisher(
            Int32MultiArray,
            "/yolo_obstacle/stable_counts",
            10,
        )

        self.topics = {
            "front": self.get_parameter("front_topic").value,
            "left": self.get_parameter("left_topic").value,
            "right": self.get_parameter("right_topic").value,
            "behind": self.get_parameter("behind_topic").value,
        }
        self.rois = {
            "front": self.parse_roi("front_roi"),
            "left": self.parse_roi("left_roi"),
            "right": self.parse_roi("right_roi"),
            "behind": self.parse_roi("behind_roi"),
        }

        self.image_subscriptions = []
        for name, topic in self.topics.items():
            subscription = self.create_subscription(
                Image,
                topic,
                lambda msg, camera_name=name: self.image_callback(camera_name, msg),
                qos_profile_sensor_data,
            )
            self.image_subscriptions.append(subscription)
            self.frames[name] = self.make_placeholder(name, topic)
            self.detection_counts[name] = 0
            self.tracks[name] = []
            self.get_logger().info(f"Subscribing {name}: {topic}")
            self.get_logger().info(f"{name} ROI: {self.rois[name]}")

        self.timer = self.create_timer(1.0 / 15.0, self.show_quad_view)
        self.get_logger().info(f"Loaded YOLO model: {model_path}")
        self.get_logger().info(f"show_window={self.show_window}")
        self.get_logger().info("Press q in the OpenCV window to close it.")

    def image_callback(self, camera_name, msg):
        try:
            frame = self.image_msg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(f"{camera_name}: {exc}")
            return

        roi_frame, offset_x, offset_y = self.crop_roi(camera_name, frame)
        results = self.model.predict(
            source=roi_frame,
            conf=self.conf,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )

        result = results[0]
        # 창을 띄울 때만 시각화용 annotated 이미지를 만든다(부하 절감).
        annotated = frame.copy() if self.show_window else None
        detections = []

        if self.show_window:
            self.draw_roi(annotated, camera_name)
        if result.boxes is not None:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                confidence = float(box.conf[0])
                x1, y1, x2, y2 = [float(value) for value in box.xyxy[0]]
                x1 += offset_x
                x2 += offset_x
                y1 += offset_y
                y2 += offset_y

                if not self.accept_box(frame, x1, y1, x2, y2):
                    continue

                detections.append(
                    {
                        "class_name": class_name,
                        "confidence": confidence,
                        "bbox": (x1, y1, x2, y2),
                    }
                )

        stable_tracks = self.update_tracks(camera_name, detections)
        if self.show_window:
            for track in stable_tracks:
                x1, y1, x2, y2 = track["bbox"]
                self.draw_box(
                    annotated,
                    track["class_name"],
                    track["confidence"],
                    track["hits"],
                    int(x1),
                    int(y1),
                    int(x2),
                    int(y2),
                )
            self.frames[camera_name] = annotated

        self.detection_counts[camera_name] = len(stable_tracks)
        self.publish_counts()

    def show_quad_view(self):
        # 창 표시가 켜져 있을 때만 타일을 합성하고 imshow 한다.
        # 끄면 매 프레임 4타일 합성+imshow 부하가 사라져 처리가 가벼워진다.
        if self.show_window:
            front = self.make_tile("front")
            left = self.make_tile("left")
            right = self.make_tile("right")
            behind = self.make_tile("behind")

            top = np.hstack((front, left))
            bottom = np.hstack((right, behind))
            quad = np.vstack((top, bottom))

            cv2.imshow("YOLO obstacle vehicles - front left right behind", quad)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.get_logger().info("q pressed, shutting down.")
                rclpy.shutdown()

        self.frame_count += 1
        if self.frame_count % self.log_every_n == 0:
            summary = ", ".join(
                f"{name}={self.detection_counts[name]}" for name in self.topics
            )
            self.get_logger().info(f"detections: {summary}")

    def publish_counts(self):
        msg = Int32MultiArray()
        msg.data = [
            int(self.detection_counts["front"]),
            int(self.detection_counts["left"]),
            int(self.detection_counts["right"]),
            int(self.detection_counts["behind"]),
        ]
        self.count_pub.publish(msg)

    def make_tile(self, camera_name):
        frame = self.frames.get(camera_name)
        if frame is None:
            frame = self.make_placeholder(camera_name, self.topics[camera_name])

        tile = cv2.resize(frame, (self.tile_width, self.tile_height))
        label = f"{camera_name.upper()}  detections: {self.detection_counts[camera_name]}"

        cv2.rectangle(tile, (0, 0), (self.tile_width, 34), (0, 0, 0), -1)
        cv2.putText(
            tile,
            label,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return tile

    def make_placeholder(self, camera_name, topic):
        image = np.zeros((self.tile_height, self.tile_width, 3), dtype=np.uint8)
        cv2.putText(
            image,
            camera_name.upper(),
            (20, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "waiting for image topic",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (180, 180, 180),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            topic,
            (20, 125),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (150, 150, 150),
            1,
            cv2.LINE_AA,
        )
        return image

    def parse_roi(self, parameter_name):
        raw_value = self.get_parameter(parameter_name).value
        values = [float(value.strip()) for value in raw_value.split(",")]
        if len(values) != 4:
            raise ValueError(f"{parameter_name} must have 4 comma-separated values")
        x1, y1, x2, y2 = values
        x1 = min(max(x1, 0.0), 1.0)
        y1 = min(max(y1, 0.0), 1.0)
        x2 = min(max(x2, 0.0), 1.0)
        y2 = min(max(y2, 0.0), 1.0)
        if x2 <= x1 or y2 <= y1:
            raise ValueError(f"{parameter_name} must satisfy x2>x1 and y2>y1")
        return x1, y1, x2, y2

    def crop_roi(self, camera_name, frame):
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = self.rois[camera_name]
        px1 = int(width * x1)
        py1 = int(height * y1)
        px2 = int(width * x2)
        py2 = int(height * y2)
        return frame[py1:py2, px1:px2], px1, py1

    def draw_roi(self, frame, camera_name):
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = self.rois[camera_name]
        pt1 = (int(width * x1), int(height * y1))
        pt2 = (int(width * x2), int(height * y2))
        cv2.rectangle(frame, pt1, pt2, (0, 255, 255), 2)

    def accept_box(self, frame, x1, y1, x2, y2):
        height, width = frame.shape[:2]
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        image_area = float(width * height)
        return area / image_area >= self.min_area_ratio

    def update_tracks(self, camera_name, detections):
        tracks = self.tracks[camera_name]
        matched_detection_indexes = set()

        for track in tracks:
            best_index = None
            best_iou = 0.0

            for index, detection in enumerate(detections):
                if index in matched_detection_indexes:
                    continue

                iou = self.box_iou(track["bbox"], detection["bbox"])
                if iou > best_iou:
                    best_iou = iou
                    best_index = index

            if best_index is not None and best_iou >= self.track_iou:
                detection = detections[best_index]
                track["bbox"] = detection["bbox"]
                track["class_name"] = detection["class_name"]
                track["confidence"] = detection["confidence"]
                track["hits"] += 1
                track["misses"] = 0
                matched_detection_indexes.add(best_index)
            else:
                track["misses"] += 1

        for index, detection in enumerate(detections):
            if index in matched_detection_indexes:
                continue

            tracks.append(
                {
                    "bbox": detection["bbox"],
                    "class_name": detection["class_name"],
                    "confidence": detection["confidence"],
                    "hits": 1,
                    "misses": 0,
                }
            )

        self.tracks[camera_name] = [
            track for track in tracks if track["misses"] <= self.max_missed_frames
        ]

        return [
            track
            for track in self.tracks[camera_name]
            if track["hits"] >= self.min_persistent_frames and track["misses"] == 0
        ]

    def box_iou(self, box_a, box_b):
        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_width = max(0.0, inter_x2 - inter_x1)
        inter_height = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_width * inter_height

        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union_area = area_a + area_b - inter_area

        if union_area <= 0.0:
            return 0.0

        return inter_area / union_area

    def draw_box(self, frame, class_name, confidence, hits, x1, y1, x2, y2):
        color = (255, 0, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{class_name} {confidence:.2f} stable:{hits}"
        text_y = max(20, y1 - 6)
        cv2.rectangle(frame, (x1, text_y - 18), (x1 + 190, text_y + 4), color, -1)
        cv2.putText(
            frame,
            label,
            (x1 + 4, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def image_msg_to_bgr(self, msg):
        height = msg.height
        width = msg.width
        encoding = msg.encoding.lower()

        if encoding in ("bgr8", "rgb8"):
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            if encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image.copy()

        if encoding == "mono8":
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        if encoding in ("bgra8", "rgba8"):
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
            if encoding == "rgba8":
                return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloObstacleQuadViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("YOLO obstacle quad viewer terminated")
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
