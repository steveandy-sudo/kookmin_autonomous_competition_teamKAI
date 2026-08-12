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
from xycar_rule_drive.canonical_stanley_pursuit_driver import (
    path_heading_change_per_m,
)


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
        self.declare_parameter("match_tolerance_sec", 0.15)
        self.bridge = CvBridge()
        self.match_tolerance_ns = int(
            max(
                0.0,
                float(self.get_parameter("match_tolerance_sec").value),
            )
            * 1_000_000_000
        )
        self.images = {}
        self.seg_paths = {}
        self.row_paths = {}
        self.new_paths = {}
        self.latest_stamps = {}

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.output_pub = self.create_publisher(
            Image,
            str(self.get_parameter("output_topic").value),
            output_qos,
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

    def reset_on_stamp_rewind(self, stream: str, stamp: int) -> None:
        previous = self.latest_stamps.get(stream)
        # A looping rosbag jumps back by many seconds. Clear every synchronized
        # cache so the first frames of the next loop are not discarded behind
        # timestamps retained from the end of the previous loop.
        if previous is not None and stamp + 1_000_000_000 < previous:
            self.images.clear()
            self.seg_paths.clear()
            self.row_paths.clear()
            self.new_paths.clear()
            self.latest_stamps.clear()
            self.get_logger().info("rosbag timestamp rewind: comparison cache reset")
        self.latest_stamps[stream] = max(
            stamp,
            self.latest_stamps.get(stream, stamp),
        )

    def closest_entry(self, cache, stamp):
        if not cache:
            return None
        closest_stamp = min(cache, key=lambda value: abs(value - stamp))
        if abs(closest_stamp - stamp) > self.match_tolerance_ns:
            return None
        return cache[closest_stamp]

    def on_seg_path(self, message: Centerline) -> None:
        stamp = self.stamp_ns(message)
        self.reset_on_stamp_rewind("seg", stamp)
        self.seg_paths[stamp] = (
            self.message_path(message),
            str(message.source),
        )
        self.trim_cache(self.seg_paths)
        self.try_render(stamp)

    def on_row_path(self, message: Centerline) -> None:
        stamp = self.stamp_ns(message)
        self.reset_on_stamp_rewind("row", stamp)
        self.row_paths[stamp] = (
            self.message_path(message),
            str(message.source),
        )
        self.trim_cache(self.row_paths)
        self.try_render(stamp)

    def on_new_path(self, message: Centerline) -> None:
        stamp = self.stamp_ns(message)
        self.reset_on_stamp_rewind("new", stamp)
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

    @staticmethod
    def curve_status(path: np.ndarray | None) -> tuple[str, float, float]:
        if path is None or len(path) < 3:
            return "WAIT", float("nan"), 0.0
        reach_m = float(np.max(path[:, 0]))
        curvature = path_heading_change_per_m(
            path,
            near_x_m=0.20,
            far_x_m=1.20,
            segment_count=1,
        )
        if not np.isfinite(curvature):
            return "SHORT", curvature, reach_m
        label = "CURVE" if curvature > 0.16 else "STRAIGHT"
        if label == "CURVE" and reach_m >= 1.0:
            label = "CURVE/PREVIEW"
        return label, curvature, reach_m

    def on_image(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        stamp = self.stamp_ns(message)
        self.reset_on_stamp_rewind("image", stamp)
        self.images[stamp] = (frame, message.header)
        self.trim_cache(self.images)
        self.try_render(stamp)

    def try_render(self, stamp: int) -> None:
        image_entry = self.closest_entry(self.images, stamp)
        if image_entry is None:
            return
        frame, header = image_entry
        seg_path, seg_source = self.closest_entry(
            self.seg_paths, stamp
        ) or (None, "WAIT")
        row_path, row_source = self.closest_entry(
            self.row_paths, stamp
        ) or (None, "WAIT")
        new_path, new_source = self.closest_entry(
            self.new_paths, stamp
        ) or (None, "WAIT")
        seg_curve, seg_curvature, seg_reach = self.curve_status(seg_path)
        row_curve, row_curvature, row_reach = self.curve_status(row_path)
        new_curve, new_curvature, new_reach = self.curve_status(new_path)
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
        # Keep the curve classification readable in RViz even when the image
        # panel is scaled down alongside the other comparison views.
        cv2.rectangle(overlay, (0, 0), (width, 126), (20, 20, 20), -1)
        cv2.putText(
            overlay,
            f"SEG {seg_curve} k={seg_curvature:.2f} reach={seg_reach:.2f}m {seg_source}",
            (14, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.80,
            (255, 190, 40),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            f"ROW {row_curve} k={row_curvature:.2f} reach={row_reach:.2f}m {row_source}",
            (14, 74),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.80,
            (0, 160, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            f"XBIN {new_curve} k={new_curvature:.2f} reach={new_reach:.2f}m {new_source}",
            (14, 114),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.80,
            (70, 245, 70),
            2,
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
