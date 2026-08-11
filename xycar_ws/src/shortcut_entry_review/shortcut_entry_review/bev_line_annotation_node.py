#!/usr/bin/env python3
"""Interactive RViz Publish Point annotation for one frozen semantic BEV."""

from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
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
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from lane_seg_control.canonical_adapter_node import (
    build_bev_geometry,
    warp_semantic_masks_only,
)

from .bev_annotation_core import BevAnnotationSession


BEV_WIDTH = 640
BEV_HEIGHT = 660
LATERAL_RANGE_M = 1.4
FORWARD_RANGE_M = 1.5


def message_stamp_ns(message: Image) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


class BevLineAnnotationNode(Node):
    """Freeze one BEV and collect W1/W2/Y1/Y2 as ordered polylines."""

    def __init__(self) -> None:
        super().__init__("shortcut_bev_line_annotation")
        self.declare_parameter(
            "white_mask_topic", "/shortcut_review/lane_seg/white_boundary_mask"
        )
        self.declare_parameter(
            "yellow_mask_topic",
            "/shortcut_review/lane_seg/yellow_centerline_mask",
        )
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("command_topic", "/shortcut_annotation/key")
        self.declare_parameter("status_topic", "/shortcut_annotation/status")
        self.declare_parameter(
            "cloud_topic", "/shortcut_annotation/semantic_bev_cloud"
        )
        self.declare_parameter(
            "image_topic", "/shortcut_annotation/semantic_bev_image"
        )
        self.declare_parameter(
            "markers_topic", "/shortcut_annotation/line_markers"
        )
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter(
            "output_dir",
            "/home/kai/kookmin_autonomous_competition_teamKAI/analysis/shortcut_line_annotations",
        )
        self.declare_parameter(
            "bag_start_timestamp_ns", 1786345371165916137
        )

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.bridge = CvBridge()
        self.session = BevAnnotationSession(lines_per_color=2)
        self.white_message: Image | None = None
        self.yellow_message: Image | None = None
        self.bev_white: np.ndarray | None = None
        self.bev_yellow: np.ndarray | None = None
        self.current_stamp_ns = 0
        self.frozen = False
        self.geometry = None
        self.geometry_input_size: tuple[int, int] | None = None

        self.cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            latched_qos,
        )
        self.image_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("image_topic").value),
            latched_qos,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("markers_topic").value),
            latched_qos,
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), latched_qos
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("white_mask_topic").value),
            self.on_white,
            input_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("yellow_mask_topic").value),
            self.on_yellow,
            input_qos,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("clicked_point_topic").value),
            self.on_clicked_point,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("command_topic").value),
            self.on_command,
            10,
        )
        self.publish_status(
            "rosbag을 멈춘 뒤 키보드 창에서 w를 눌러 W1을 시작하세요"
        )

    @property
    def frame_id(self) -> str:
        return str(self.get_parameter("frame_id").value)

    def on_white(self, message: Image) -> None:
        if self.frozen:
            return
        self.white_message = message
        self.try_update_bev()

    def on_yellow(self, message: Image) -> None:
        if self.frozen:
            return
        self.yellow_message = message
        self.try_update_bev()

    def ensure_geometry(self, width: int, height: int):
        size = (int(width), int(height))
        if self.geometry is not None and size == self.geometry_input_size:
            return self.geometry
        self.geometry = build_bev_geometry(
            width,
            height,
            source_ratios=(
                0.442578,
                0.480781,
                0.688281,
                0.480781,
                0.919141,
                0.614189,
                0.190625,
                0.614189,
            ),
            destination_ratios=(0.205714, 0.794286, 0.0, 0.666666667),
            bev_width=BEV_WIDTH,
            bev_height=BEV_HEIGHT,
        )
        self.geometry_input_size = size
        return self.geometry

    def try_update_bev(self) -> None:
        if self.white_message is None or self.yellow_message is None:
            return
        if message_stamp_ns(self.white_message) != message_stamp_ns(
            self.yellow_message
        ):
            return
        white = self.bridge.imgmsg_to_cv2(
            self.white_message, desired_encoding="mono8"
        )
        yellow = self.bridge.imgmsg_to_cv2(
            self.yellow_message, desired_encoding="mono8"
        )
        if white.shape != yellow.shape:
            return
        geometry = self.ensure_geometry(white.shape[1], white.shape[0])
        self.bev_white, self.bev_yellow, _ = warp_semantic_masks_only(
            white,
            yellow,
            geometry,
            valid_lateral_margin_px=0,
            valid_erode_px=0,
            clip_to_source_polygon=False,
        )
        self.current_stamp_ns = message_stamp_ns(self.white_message)
        self.publish_cloud()

    def pixel_to_world(self, x_px: float, y_px: float) -> tuple[float, float]:
        forward = (BEV_HEIGHT - float(y_px)) * FORWARD_RANGE_M / BEV_HEIGHT
        left = -(float(x_px) - BEV_WIDTH * 0.5) * LATERAL_RANGE_M / BEV_WIDTH
        return forward, left

    def world_to_pixel(self, forward: float, left: float) -> tuple[float, float]:
        x_px = BEV_WIDTH * 0.5 - float(left) * BEV_WIDTH / LATERAL_RANGE_M
        y_px = BEV_HEIGHT - float(forward) * BEV_HEIGHT / FORWARD_RANGE_M
        return x_px, y_px

    def header(self) -> Header:
        header = Header()
        header.frame_id = self.frame_id
        header.stamp.sec = self.current_stamp_ns // 1_000_000_000
        header.stamp.nanosec = self.current_stamp_ns % 1_000_000_000
        return header

    def publish_cloud(self) -> None:
        if self.bev_white is None or self.bev_yellow is None:
            return
        points = []
        for mask, rgb in (
            (self.bev_white, 0xFFFFFF),
            (self.bev_yellow, 0xFFD000),
        ):
            ys, xs = np.nonzero(mask)
            for x_px, y_px in zip(xs[::2], ys[::2]):
                forward, left = self.pixel_to_world(x_px, y_px)
                points.append((forward, left, 0.0, rgb))
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="rgb", offset=12, datatype=PointField.UINT32, count=1
            ),
        ]
        self.cloud_publisher.publish(
            point_cloud2.create_cloud(self.header(), fields, points)
        )
        self.publish_bev_image()

    def publish_bev_image(self) -> None:
        if self.bev_white is None or self.bev_yellow is None:
            return
        output = np.full((BEV_HEIGHT, BEV_WIDTH, 3), 24, dtype=np.uint8)
        output[self.bev_white > 0] = (255, 255, 255)
        output[self.bev_yellow > 0] = (0, 208, 255)
        for line in self.session.all_lines():
            if not line.points_px:
                continue
            points = np.rint(np.asarray(line.points_px)).astype(np.int32)
            color = (255, 255, 255) if line.color == "white" else (0, 208, 255)
            if len(points) >= 2:
                cv2.polylines(output, [points], False, color, 4, cv2.LINE_AA)
            for point in points:
                cv2.circle(output, tuple(point), 5, (255, 0, 255), -1)
            cv2.putText(
                output,
                line.label,
                tuple(points[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
        bag_start_ns = int(
            self.get_parameter("bag_start_timestamp_ns").value
        )
        offset = (
            (self.current_stamp_ns - bag_start_ns) / 1_000_000_000.0
            if bag_start_ns > 0
            else math.nan
        )
        cv2.rectangle(output, (0, 0), (BEV_WIDTH, 38), (12, 12, 12), -1)
        cv2.putText(
            output,
            f"SEMANTIC BEV | BAG OFFSET {offset:.3f} s | FAR / FORWARD",
            (10, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (240, 240, 240),
            2,
            cv2.LINE_AA,
        )
        cv2.arrowedLine(
            output,
            (BEV_WIDTH - 45, BEV_HEIGHT - 25),
            (BEV_WIDTH - 45, BEV_HEIGHT - 115),
            (70, 240, 70),
            4,
            tipLength=0.25,
        )
        cv2.putText(
            output,
            "CAR",
            (BEV_WIDTH // 2 - 24, BEV_HEIGHT - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (80, 180, 255),
            2,
            cv2.LINE_AA,
        )
        message = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
        message.header = self.header()
        self.image_publisher.publish(message)

    def on_clicked_point(self, message: PointStamped) -> None:
        if not self.frozen:
            self.publish_status("먼저 w 또는 y를 눌러 현재 BEV 프레임을 고정하세요")
            return
        x_px, y_px = self.world_to_pixel(message.point.x, message.point.y)
        if not (0.0 <= x_px < BEV_WIDTH and 0.0 <= y_px < BEV_HEIGHT):
            self.publish_status("클릭이 BEV 범위 밖입니다")
            return
        try:
            label = self.session.add_point(x_px, y_px)
        except ValueError as exc:
            self.publish_status(str(exc))
            return
        self.publish_markers()
        self.publish_bev_image()
        self.publish_status(
            f"{label}: point {len(self.session.current_points)} 추가"
        )

    def freeze(self) -> bool:
        if self.bev_white is None or self.bev_yellow is None:
            self.publish_status("아직 LR-ASPP BEV 프레임을 받지 못했습니다")
            return False
        self.frozen = True
        self.publish_cloud()
        return True

    def on_command(self, message: String) -> None:
        command = str(message.data).strip().lower()
        if command in ("w", "y"):
            if not self.freeze():
                return
            color = "white" if command == "w" else "yellow"
            try:
                label = self.session.begin(color)
            except ValueError as exc:
                self.publish_status(str(exc))
                return
            self.publish_markers()
            self.publish_bev_image()
            self.publish_status(f"{label} 시작: Publish Point를 여러 개 찍으세요")
            return
        if command in ("f", "enter"):
            try:
                line = self.session.finish(allow_empty=True)
            except ValueError as exc:
                self.publish_status(str(exc))
                return
            self.publish_markers()
            self.publish_bev_image()
            suffix = " (선 없음)" if not line.points_px else ""
            self.publish_status(f"{line.label} 완료{suffix}")
            return
        if command == "z":
            changed = self.session.undo()
            self.publish_markers()
            self.publish_bev_image()
            self.publish_status("마지막 점 취소" if changed else "취소할 점이 없습니다")
            return
        if command == "r":
            self.session.reset()
            self.publish_markers()
            self.publish_bev_image()
            self.publish_status("현재 고정 프레임의 모든 점을 초기화했습니다")
            return
        if command == "n":
            self.session.reset()
            self.frozen = False
            self.white_message = None
            self.yellow_message = None
            self.publish_markers()
            self.publish_status("다음 수신 BEV 프레임으로 전환합니다")
            return
        if command == "s":
            self.save()
            return
        self.publish_status(f"알 수 없는 명령: {command}")

    def save(self) -> None:
        if self.session.current_color is not None:
            try:
                self.session.finish(allow_empty=True)
            except ValueError as exc:
                self.publish_status(str(exc))
                return
        if self.bev_white is None or self.bev_yellow is None:
            self.publish_status("저장할 BEV 프레임이 없습니다")
            return
        bag_start_ns = int(
            self.get_parameter("bag_start_timestamp_ns").value
        )
        bag_offset = (
            (self.current_stamp_ns - bag_start_ns) / 1_000_000_000.0
            if bag_start_ns > 0
            else None
        )
        try:
            document = self.session.to_document(
                image_width=BEV_WIDTH,
                image_height=BEV_HEIGHT,
                bag_offset_sec=bag_offset,
                timestamp_ns=self.current_stamp_ns,
            )
        except ValueError as exc:
            self.publish_status(str(exc))
            return
        output_dir = Path(
            str(self.get_parameter("output_dir").value)
        ).expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        suffix = f"{bag_offset:08.3f}s" if bag_offset is not None else str(
            self.current_stamp_ns
        )
        stem = f"shortcut_lines_{suffix}"
        (output_dir / f"{stem}.json").write_text(
            json.dumps(document, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        cv2.imwrite(str(output_dir / f"{stem}_white.png"), self.bev_white)
        cv2.imwrite(str(output_dir / f"{stem}_yellow.png"), self.bev_yellow)
        combined = np.full((BEV_HEIGHT, BEV_WIDTH, 3), 24, dtype=np.uint8)
        combined[self.bev_white > 0] = (255, 255, 255)
        combined[self.bev_yellow > 0] = (0, 208, 255)
        for line in self.session.all_lines(include_current=False):
            if not line.points_px:
                continue
            points = np.rint(np.asarray(line.points_px)).astype(np.int32)
            color = (255, 255, 255) if line.color == "white" else (0, 208, 255)
            cv2.polylines(combined, [points], False, color, 4, cv2.LINE_AA)
            cv2.putText(
                combined,
                line.label,
                tuple(points[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
        cv2.imwrite(str(output_dir / f"{stem}_annotated.png"), combined)
        self.publish_markers()
        self.publish_bev_image()
        saved_path = output_dir / (stem + ".json")
        self.session.reset()
        self.frozen = False
        self.white_message = None
        self.yellow_message = None
        self.publish_markers()
        self.publish_status(
            f"저장 완료: {saved_path} | 프레임 고정 자동 해제"
        )

    def publish_markers(self) -> None:
        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)
        header = self.header()
        marker_id = 0
        for line in self.session.all_lines():
            strip = Marker()
            strip.header = header
            strip.ns = "shortcut_line_annotations"
            strip.id = marker_id
            marker_id += 1
            strip.type = Marker.LINE_STRIP
            strip.action = Marker.ADD
            strip.pose.orientation.w = 1.0
            strip.scale.x = 0.018
            if line.color == "white":
                strip.color.r = strip.color.g = strip.color.b = 1.0
            else:
                strip.color.r = 1.0
                strip.color.g = 0.82
                strip.color.b = 0.0
            strip.color.a = 1.0
            for x_px, y_px in line.points_px:
                forward, left = self.pixel_to_world(x_px, y_px)
                strip.points.append(Point(x=forward, y=left, z=0.025))
            markers.markers.append(strip)
            if strip.points:
                label = Marker()
                label.header = header
                label.ns = "shortcut_line_labels"
                label.id = marker_id
                marker_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position = strip.points[0]
                label.pose.position.z = 0.09
                label.pose.orientation.w = 1.0
                label.scale.z = 0.10
                label.color.r = 1.0
                label.color.g = 0.0
                label.color.b = 1.0
                label.color.a = 1.0
                label.text = line.label
                markers.markers.append(label)
        arrow = Marker()
        arrow.header = header
        arrow.ns = "shortcut_vehicle_axis"
        arrow.id = marker_id
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.scale.x = 0.04
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color.g = 1.0
        arrow.color.a = 1.0
        arrow.points = [Point(x=0.0, y=0.0, z=0.04), Point(x=0.8, y=0.0, z=0.04)]
        markers.markers.append(arrow)
        self.marker_publisher.publish(markers)

    def publish_status(self, message: str) -> None:
        self.status_publisher.publish(String(data=str(message)))
        self.get_logger().info(str(message))


def main() -> None:
    cv2.setNumThreads(1)
    rclpy.init()
    node = BevLineAnnotationNode()
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
