#!/usr/bin/env python3
"""Project old/new metric target paths back onto the rectified camera."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from kaiev26_msgs.msg import Centerline
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def camera_to_bev_matrix(
    *,
    image_width: int,
    image_height: int,
    bev_height: int,
    dst_top_y_ratio: float,
    dst_bottom_y_ratio: float,
) -> np.ndarray:
    source = np.float32(
        [
            [0.442578 * image_width, 0.480781 * image_height],
            [0.688281 * image_width, 0.480781 * image_height],
            [0.919141 * image_width, 0.614189 * image_height],
            [0.190625 * image_width, 0.614189 * image_height],
        ]
    )
    destination = np.float32(
        [
            [0.205714 * 640, dst_top_y_ratio * bev_height],
            [0.794286 * 640, dst_top_y_ratio * bev_height],
            [0.794286 * 640, dst_bottom_y_ratio * bev_height],
            [0.205714 * 640, dst_bottom_y_ratio * bev_height],
        ]
    )
    return cv2.getPerspectiveTransform(source, destination)


class CameraPathCompareViewer(Node):
    def __init__(self) -> None:
        super().__init__("camera_path_compare_viewer")
        self.declare_parameter(
            "image_topic", "/comparison/new/source_image"
        )
        self.declare_parameter(
            "seg_path_topic", "/comparison/seg/connected_path"
        )
        self.declare_parameter(
            "row_path_topic", "/comparison/row/connected_path"
        )
        self.declare_parameter(
            "new_path_topic", "/comparison/new/connected_path"
        )
        self.declare_parameter(
            "output_topic", "/comparison/camera_path_overlay"
        )
        self.bridge = CvBridge()
        self.images = {}
        self.seg_paths = {}
        self.row_paths = {}
        self.new_paths = {}

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.output_pub = self.create_publisher(
            Image, str(self.get_parameter("output_topic").value), sensor_qos
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            sensor_qos,
        )
        self.create_subscription(
            Centerline,
            str(self.get_parameter("seg_path_topic").value),
            self.on_seg_path,
            10,
        )
        self.create_subscription(
            Centerline,
            str(self.get_parameter("row_path_topic").value),
            self.on_row_path,
            10,
        )
        self.create_subscription(
            Centerline,
            str(self.get_parameter("new_path_topic").value),
            self.on_new_path,
            10,
        )
        self.old_inverse = np.linalg.inv(
            camera_to_bev_matrix(
                image_width=256,
                image_height=144,
                bev_height=660,
                dst_top_y_ratio=0.0,
                dst_bottom_y_ratio=2.0 / 3.0,
            )
        )
        self.new_inverse = np.linalg.inv(
            camera_to_bev_matrix(
                image_width=512,
                image_height=288,
                bev_height=1100,
                dst_top_y_ratio=0.4,
                dst_bottom_y_ratio=0.8,
            )
        )
        self.get_logger().info(
            "camera path overlay ready: blue=seg, orange=row, green=new"
        )

    @staticmethod
    def message_path(message: Centerline) -> np.ndarray | None:
        if len(message.points) < 2:
            return None
        return np.asarray(
            [(float(point.x), float(point.y)) for point in message.points],
            dtype=np.float32,
        )

    @staticmethod
    def stamp_ns(message) -> int:
        return (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    @staticmethod
    def trim_cache(cache, maximum=12):
        while len(cache) > maximum:
            cache.pop(min(cache))

    def on_seg_path(self, message: Centerline) -> None:
        stamp = self.stamp_ns(message)
        self.seg_paths[stamp] = (
            self.message_path(message),
            str(message.source),
        )
        self.trim_cache(self.seg_paths)
        self.try_render(stamp)

    def on_row_path(self, message: Centerline) -> None:
        stamp = self.stamp_ns(message)
        self.row_paths[stamp] = (
            self.message_path(message),
            str(message.source),
        )
        self.trim_cache(self.row_paths)
        self.try_render(stamp)

    def on_new_path(self, message: Centerline) -> None:
        stamp = self.stamp_ns(message)
        self.new_paths[stamp] = (
            self.message_path(message),
            str(message.source),
        )
        self.trim_cache(self.new_paths)
        self.try_render(stamp)

    @staticmethod
    def project_path(
        path: np.ndarray | None,
        inverse: np.ndarray,
        *,
        bev_height: int,
        camera_width: int,
        camera_height: int,
        output_width: int,
        output_height: int,
    ) -> np.ndarray:
        if path is None or len(path) < 2:
            return np.empty((0, 2), dtype=np.int32)
        lateral_m_per_px = 1.4 / 640.0
        forward_m_per_px = 1.5 / 660.0
        bev = np.empty((len(path), 1, 2), dtype=np.float32)
        bev[:, 0, 0] = 320.0 - path[:, 1] / lateral_m_per_px
        bev[:, 0, 1] = float(bev_height) - path[:, 0] / forward_m_per_px
        camera = cv2.perspectiveTransform(bev, inverse).reshape(-1, 2)
        camera[:, 0] *= float(output_width) / float(camera_width)
        camera[:, 1] *= float(output_height) / float(camera_height)
        valid = (
            np.all(np.isfinite(camera), axis=1)
            & (camera[:, 0] >= -0.1 * output_width)
            & (camera[:, 0] <= 1.1 * output_width)
            & (camera[:, 1] >= -0.1 * output_height)
            & (camera[:, 1] <= 1.1 * output_height)
        )
        return np.rint(camera[valid]).astype(np.int32)

    @staticmethod
    def draw_path(image, points, color, label):
        if len(points) < 2:
            return
        cv2.polylines(image, [points], False, (15, 15, 15), 7, cv2.LINE_AA)
        cv2.polylines(image, [points], False, color, 4, cv2.LINE_AA)
        cv2.circle(image, tuple(points[-1]), 6, (15, 15, 15), -1, cv2.LINE_AA)
        cv2.circle(image, tuple(points[-1]), 4, color, -1, cv2.LINE_AA)
        midpoint = points[len(points) // 2]
        cv2.putText(
            image,
            label,
            (int(midpoint[0]) + 7, int(midpoint[1]) - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            2,
            cv2.LINE_AA,
        )

    def on_image(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        stamp = self.stamp_ns(message)
        self.images[stamp] = (frame, message.header)
        self.trim_cache(self.images)
        self.try_render(stamp)

    def try_render(self, stamp: int) -> None:
        if not (
            stamp in self.images
            and stamp in self.seg_paths
            and stamp in self.row_paths
            and stamp in self.new_paths
        ):
            return
        frame, header = self.images.pop(stamp)
        seg_path, seg_source = self.seg_paths.pop(stamp)
        row_path, row_source = self.row_paths.pop(stamp)
        new_path, new_source = self.new_paths.pop(stamp)
        height, width = frame.shape[:2]
        seg_points = self.project_path(
            seg_path,
            self.old_inverse,
            bev_height=660,
            camera_width=256,
            camera_height=144,
            output_width=width,
            output_height=height,
        )
        row_points = self.project_path(
            row_path,
            self.old_inverse,
            bev_height=660,
            camera_width=256,
            camera_height=144,
            output_width=width,
            output_height=height,
        )
        new_points = self.project_path(
            new_path,
            self.new_inverse,
            bev_height=1100,
            camera_width=512,
            camera_height=288,
            output_width=width,
            output_height=height,
        )
        overlay = frame.copy()
        self.draw_path(overlay, seg_points, (255, 190, 40), "SEG")
        self.draw_path(overlay, row_points, (0, 160, 255), "ROW")
        self.draw_path(overlay, new_points, (70, 245, 70), "NEW")
        cv2.rectangle(overlay, (0, 0), (width, 60), (20, 20, 20), -1)
        cv2.putText(
            overlay,
            f"SEG 1.5m {seg_source}",
            (10, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 190, 40),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            f"ROW 1.5m {row_source}",
            (10, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 160, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            f"NEW 2.5m {new_source}",
            (10, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (70, 245, 70),
            1,
            cv2.LINE_AA,
        )
        output = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        output.header = header
        self.output_pub.publish(output)


def main():
    rclpy.init()
    node = CameraPathCompareViewer()
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
