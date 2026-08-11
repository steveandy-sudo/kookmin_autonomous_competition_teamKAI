#!/usr/bin/env python3
"""Publish a motor-free preview of the white-left/yellow-right entry path."""

from __future__ import annotations

import math

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, String

from lane_seg_control.canonical_adapter_node import (
    build_bev_geometry,
    warp_semantic_masks_only,
)

from .white_yellow_entry_core import (
    WhiteYellowEntrySelector,
    render_entry_debug,
)


def stamp_key(message: Image) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


class WhiteYellowEntryReviewNode(Node):
    """Synchronize semantic masks and expose only a shadow path/status."""

    def __init__(self) -> None:
        super().__init__("white_yellow_entry_review")
        self.declare_parameter(
            "white_mask_topic", "/shortcut/lraspp/white_mask"
        )
        self.declare_parameter(
            "yellow_mask_topic", "/shortcut/lraspp/yellow_mask"
        )
        self.declare_parameter(
            "processing_enabled_topic", "/hybrid/shortcut_processing_enabled"
        )
        self.declare_parameter("default_enabled", False)
        self.declare_parameter("debug_topic", "/shortcut/entry/debug_image")
        self.declare_parameter("status_topic", "/shortcut/entry/status")
        self.declare_parameter(
            "diagnostics_topic", "/shortcut/entry/diagnostics"
        )
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("bag_start_timestamp_ns", 0)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        gate_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.bridge = CvBridge()
        self.selector = WhiteYellowEntrySelector()
        self.enabled = bool(self.get_parameter("default_enabled").value)
        self.white_message: Image | None = None
        self.yellow_message: Image | None = None
        self.last_processed_stamp: tuple[int, int] | None = None
        self.geometry = None
        self.geometry_input_size: tuple[int, int] | None = None

        self.debug_publisher = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), debug_qos
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.diagnostics_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("white_mask_topic").value),
            self.on_white,
            image_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("yellow_mask_topic").value),
            self.on_yellow,
            image_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("processing_enabled_topic").value),
            self.on_gate,
            gate_qos,
        )
        self.get_logger().info(
            "white-left/yellow-right shortcut shadow selector ready; no motor "
            "or hybrid candidate topic is published"
        )

    def on_gate(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested == self.enabled:
            return
        self.enabled = requested
        self.white_message = None
        self.yellow_message = None
        self.last_processed_stamp = None
        self.selector.reset()

    def on_white(self, message: Image) -> None:
        if not self.enabled:
            return
        self.white_message = message
        self.try_process()

    def on_yellow(self, message: Image) -> None:
        if not self.enabled:
            return
        self.yellow_message = message
        self.try_process()

    def ensure_geometry(self, width: int, height: int):
        input_size = (int(width), int(height))
        if self.geometry is not None and input_size == self.geometry_input_size:
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
            bev_width=640,
            bev_height=660,
        )
        self.geometry_input_size = input_size
        return self.geometry

    def try_process(self) -> None:
        if self.white_message is None or self.yellow_message is None:
            return
        white_stamp = stamp_key(self.white_message)
        yellow_stamp = stamp_key(self.yellow_message)
        if white_stamp != yellow_stamp or white_stamp == self.last_processed_stamp:
            return
        white = self.bridge.imgmsg_to_cv2(
            self.white_message, desired_encoding="mono8"
        )
        yellow = self.bridge.imgmsg_to_cv2(
            self.yellow_message, desired_encoding="mono8"
        )
        if white.shape != yellow.shape:
            self.get_logger().warning(
                f"semantic mask size mismatch: {white.shape} != {yellow.shape}"
            )
            return
        geometry = self.ensure_geometry(white.shape[1], white.shape[0])
        bev_white, bev_yellow, _ = warp_semantic_masks_only(
            white,
            yellow,
            geometry,
            valid_lateral_margin_px=0,
            valid_erode_px=0,
            clip_to_source_polygon=False,
        )
        result = self.selector.process(bev_white, bev_yellow)
        debug = render_entry_debug(bev_white, bev_yellow, result)
        bag_start_ns = int(
            self.get_parameter("bag_start_timestamp_ns").value
        )
        if bag_start_ns > 0:
            stamp_ns = white_stamp[0] * 1_000_000_000 + white_stamp[1]
            bag_offset_sec = (stamp_ns - bag_start_ns) / 1_000_000_000.0
            cv2.rectangle(
                debug,
                (0, debug.shape[0] - 42),
                (debug.shape[1], debug.shape[0]),
                (16, 16, 16),
                -1,
            )
            cv2.putText(
                debug,
                f"BAG OFFSET {bag_offset_sec:.3f} s  |  SPACE: pause/resume",
                (12, debug.shape[0] - 13),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.70,
                (240, 240, 240),
                2,
                cv2.LINE_AA,
            )
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_message.header = self.white_message.header
        debug_message.header.frame_id = str(
            self.get_parameter("base_frame_id").value
        )
        self.debug_publisher.publish(debug_message)
        self.status_publisher.publish(String(data=result.reason))
        self.diagnostics_publisher.publish(
            Float32MultiArray(
                data=[
                    1.0 if result.valid else 0.0,
                    float(result.target_lateral_px),
                    float(result.median_separation_px),
                    float(
                        result.white.base_x
                        if result.white.base_x is not None
                        else math.nan
                    ),
                    float(
                        result.yellow.base_x
                        if result.yellow.base_x is not None
                        else math.nan
                    ),
                    float(len(result.white.centers)),
                    float(len(result.yellow.centers)),
                ]
            )
        )
        self.last_processed_stamp = white_stamp


def main() -> None:
    cv2.setNumThreads(1)
    rclpy.init()
    node = WhiteYellowEntryReviewNode()
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
