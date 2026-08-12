#!/usr/bin/env python3
"""Publish a W1/Y1 entry path and readiness gates from semantic masks."""

from __future__ import annotations

import math

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from kaiev26_msgs.msg import Centerline
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
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, Header, String

from lane_seg_control.canonical_adapter_node import (
    build_bev_geometry,
    warp_semantic_masks_only,
)

from .sequence_entry_core import (
    SequenceEntryConfig,
    SequenceAwareEntrySelector,
    pixels_to_vehicle_path,
    render_sequence_debug,
)


def stamp_key(message: Image) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


class SequenceEntryNode(Node):
    """Synchronize masks and expose vehicle-frame W1/Y1 state and path."""

    def __init__(self) -> None:
        super().__init__("shortcut_sequence_entry")
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
        self.declare_parameter("input_is_bev", False)
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("forward_range_m", 1.5)
        self.declare_parameter("lateral_range_m", 1.4)
        self.declare_parameter("w1_path_weight", 0.60)
        self.declare_parameter("show_opencv_windows", False)
        self.declare_parameter(
            "path_topic", "/shortcut/entry/selected_centerline"
        )
        self.declare_parameter("ready_topic", "/shortcut/entry/ready")
        self.declare_parameter(
            "entry_distance_topic", "/shortcut/entry/entry_distance_m"
        )
        self.declare_parameter(
            "path_valid_topic", "/shortcut/entry/path_valid"
        )
        self.declare_parameter(
            "cruise_handoff_topic", "/shortcut/entry/cruise_enabled"
        )
        self.declare_parameter("phase_topic", "/shortcut/entry/phase")
        self.declare_parameter("status_topic", "/shortcut/entry/status")
        self.declare_parameter(
            "diagnostics_topic", "/shortcut/entry/diagnostics"
        )
        self.declare_parameter("debug_topic", "/shortcut/entry/debug_image")
        self.declare_parameter(
            "canonical_input_debug_topic",
            "/shortcut/entry/canonical_model_input",
        )

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.bridge = CvBridge()
        self.selector = SequenceAwareEntrySelector(
            SequenceEntryConfig(
                w1_path_weight=float(
                    self.get_parameter("w1_path_weight").value
                )
            )
        )
        self.enabled = bool(self.get_parameter("default_enabled").value)
        self.white_message: Image | None = None
        self.yellow_message: Image | None = None
        self.last_stamp: tuple[int, int] | None = None
        self.semantic_frame_index = 0
        self.handoff_logged = False
        self.w1_logged = False
        self.geometry = None
        self.geometry_input_size: tuple[int, int] | None = None

        self.path_publisher = self.create_publisher(
            Centerline, str(self.get_parameter("path_topic").value), 10
        )
        self.ready_publisher = self.create_publisher(
            Bool, str(self.get_parameter("ready_topic").value), state_qos
        )
        self.entry_distance_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("entry_distance_topic").value),
            state_qos,
        )
        self.path_valid_publisher = self.create_publisher(
            Bool, str(self.get_parameter("path_valid_topic").value), state_qos
        )
        self.cruise_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("cruise_handoff_topic").value),
            state_qos,
        )
        self.phase_publisher = self.create_publisher(
            Float32, str(self.get_parameter("phase_topic").value), state_qos
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.diagnostics_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.debug_publisher = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), image_qos
        )
        self.canonical_input_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("canonical_input_debug_topic").value),
            image_qos,
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
            self.on_enabled,
            state_qos,
        )
        self.publish_state(False, False, False, 0.0)
        self.entry_distance_publisher.publish(Float32(data=math.nan))
        self.get_logger().info(
            "sequence W1/Y1 selector ready: timestamps are used only for mask "
            "synchronization; phase transitions use observation order"
        )

    def publish_state(
        self,
        ready: bool,
        path_valid: bool,
        cruise_handoff: bool,
        phase: float,
    ) -> None:
        self.ready_publisher.publish(Bool(data=bool(ready)))
        self.path_valid_publisher.publish(Bool(data=bool(path_valid)))
        self.cruise_publisher.publish(Bool(data=bool(cruise_handoff)))
        self.phase_publisher.publish(Float32(data=float(phase)))

    def on_enabled(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested == self.enabled:
            return
        self.enabled = requested
        self.white_message = None
        self.yellow_message = None
        self.last_stamp = None
        self.semantic_frame_index = 0
        self.handoff_logged = False
        self.w1_logged = False
        self.selector.reset()
        self.publish_state(False, False, False, 0.0)
        self.entry_distance_publisher.publish(Float32(data=math.nan))

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
        size = (int(width), int(height))
        if self.geometry is not None and self.geometry_input_size == size:
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
        self.geometry_input_size = size
        return self.geometry

    def bev_masks(self, white, yellow):
        if bool(self.get_parameter("input_is_bev").value):
            return white, yellow
        geometry = self.ensure_geometry(white.shape[1], white.shape[0])
        bev_white, bev_yellow, _ = warp_semantic_masks_only(
            white,
            yellow,
            geometry,
            valid_lateral_margin_px=0,
            valid_erode_px=0,
            clip_to_source_polygon=False,
        )
        return bev_white, bev_yellow

    def try_process(self) -> None:
        if self.white_message is None or self.yellow_message is None:
            return
        white_stamp = stamp_key(self.white_message)
        yellow_stamp = stamp_key(self.yellow_message)
        if white_stamp != yellow_stamp or white_stamp == self.last_stamp:
            return
        if self.last_stamp is not None and white_stamp < self.last_stamp:
            # A looping/re-seeked review bag is a new observation sequence.
            # This does not participate in W1 identity selection; it only
            # prevents state from the end of the previous playback lap from
            # leaking into the first frame of the next lap.
            self.selector.reset()
            self.semantic_frame_index = 0
            self.handoff_logged = False
            self.w1_logged = False
            self.publish_state(False, False, False, 0.0)
            self.entry_distance_publisher.publish(Float32(data=math.nan))
        try:
            white = self.bridge.imgmsg_to_cv2(
                self.white_message, desired_encoding="mono8"
            )
            yellow = self.bridge.imgmsg_to_cv2(
                self.yellow_message, desired_encoding="mono8"
            )
        except Exception as exc:
            self.get_logger().error(f"semantic mask conversion failed: {exc}")
            return
        if white.shape != yellow.shape:
            self.get_logger().warning(
                f"mask size mismatch: {white.shape} != {yellow.shape}"
            )
            return
        bev_white, bev_yellow = self.bev_masks(white, yellow)
        result = self.selector.process(bev_white, bev_yellow)
        self.semantic_frame_index += 1
        if result.ready and not self.w1_logged and result.w1 is not None:
            self.get_logger().warning(
                "[MISSION] W1 DETECTED/LOCKED: identity acquired; "
                "RULE control remains until spatial entry gate; "
                f"W1_slope={result.w1.direction_dx_dy:+.3f} "
                f"span={result.w1.vertical_span_ratio:.3f}"
            )
            self.w1_logged = True
        output_header = Header(
            stamp=self.white_message.header.stamp,
            frame_id=str(self.get_parameter("base_frame_id").value),
        )
        if result.path_valid:
            vehicle_path = pixels_to_vehicle_path(
                result.path_pixels,
                width=bev_white.shape[1],
                height=bev_white.shape[0],
                forward_range_m=float(
                    self.get_parameter("forward_range_m").value
                ),
                lateral_range_m=float(
                    self.get_parameter("lateral_range_m").value
                ),
            )
            path_message = Centerline()
            path_message.header = output_header
            path_message.detection_id = 0
            path_message.track_id = 0
            path_message.points = [
                Point(x=float(forward), y=float(lateral), z=0.0)
                for forward, lateral in vehicle_path
            ]
            path_message.confidence = 1.0
            path_message.source = "shortcut_W1_fixed_lane_offset"
            self.path_publisher.publish(path_message)

        # Publish the path before readiness.  This preserves the requested
        # first-W1 handoff while giving the existing 20 Hz controller the
        # earliest possible chance to create a candidate before RULE releases.
        self.publish_state(
            result.ready,
            result.path_valid,
            result.cruise_handoff,
            float(result.phase),
        )
        self.status_publisher.publish(String(data=result.reason))

        if result.cruise_handoff and not self.handoff_logged:
            stamp = self.white_message.header.stamp
            w1_slope = (
                result.w1.direction_dx_dy
                if result.w1 is not None
                else math.nan
            )
            y1_slope = (
                result.y1.direction_dx_dy
                if result.y1 is not None
                else math.nan
            )
            self.get_logger().warning(
                "SHORTCUT HANDOFF REQUEST: semantic W1/Y1 entry -> existing "
                "ShortcutCore cruise; "
                f"ros_stamp={int(stamp.sec)}.{int(stamp.nanosec):09d} "
                f"semantic_frame={self.semantic_frame_index} "
                "condition=forward_alignment_3_frames "
                f"W1_slope={w1_slope:+.3f} Y1_slope={y1_slope:+.3f}"
            )
            self.handoff_logged = True

        w1 = result.w1
        w2 = result.w2
        y1 = result.y1
        entry_distance_m = math.nan
        if w1 is not None and w2 is not None:
            intersection_y = self.selector._branch_intersection_y_ratio(
                w1, w2
            )
            if intersection_y is not None and math.isfinite(intersection_y):
                forward_range_m = float(
                    self.get_parameter("forward_range_m").value
                )
                entry_distance_m = max(
                    0.0, (1.0 - float(intersection_y)) * forward_range_m
                )
        self.entry_distance_publisher.publish(
            Float32(data=float(entry_distance_m))
        )
        self.diagnostics_publisher.publish(
            Float32MultiArray(
                data=[
                    float(result.phase),
                    1.0 if result.ready else 0.0,
                    1.0 if result.path_valid else 0.0,
                    1.0 if result.used_synthetic_y1 else 0.0,
                    float(result.pair_separation_ratio),
                    float(w1.mean_x_ratio if w1 is not None else math.nan),
                    float(
                        w1.direction_dx_dy if w1 is not None else math.nan
                    ),
                    float(y1.mean_x_ratio if y1 is not None else math.nan),
                    float(
                        y1.direction_dx_dy if y1 is not None else math.nan
                    ),
                    float(len(result.white_candidates)),
                    float(len(result.yellow_candidates)),
                    # Appended fields keep the original diagnostics indices
                    # stable while allowing the control viewer to report W2.
                    float(w2.mean_x_ratio if w2 is not None else math.nan),
                    float(
                        w2.direction_dx_dy if w2 is not None else math.nan
                    ),
                    # Current W1/W2 fork distance in the vehicle-forward BEV
                    # frame.  This is a spatial control gate, never a bag
                    # timestamp, frame number, or fixed pixel coordinate.
                    float(entry_distance_m),
                ]
            )
        )

        # The composite viewer owns phase/candidate text.  Keep this image a
        # clean BEV layer so labels never cover the 256x144 lane geometry.
        debug = render_sequence_debug(
            bev_white,
            bev_yellow,
            result,
            show_candidates=False,
            show_status=False,
        )
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_message.header = output_header
        self.debug_publisher.publish(debug_message)

        canonical = np.full(
            (bev_white.shape[0], bev_white.shape[1], 3), 28, dtype=np.uint8
        )
        canonical[bev_white > 0] = (255, 255, 255)
        canonical[bev_yellow > 0] = (0, 220, 255)
        cv2.putText(
            canonical,
            "CANONICAL MODEL INPUT (before W1/Y1 selection)",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (80, 80, 255),
            2,
            cv2.LINE_AA,
        )
        canonical_message = self.bridge.cv2_to_imgmsg(
            canonical, encoding="bgr8"
        )
        canonical_message.header = output_header
        self.canonical_input_publisher.publish(canonical_message)
        if bool(self.get_parameter("show_opencv_windows").value):
            cv2.imshow("Shortcut Canonical Model Input", canonical)
            cv2.imshow("Shortcut W1-Y1 Selection and Path", debug)
            cv2.waitKey(1)
        self.last_stamp = white_stamp


def main() -> None:
    cv2.setNumThreads(1)
    rclpy.init()
    node = SequenceEntryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
