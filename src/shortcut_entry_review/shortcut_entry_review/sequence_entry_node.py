#!/usr/bin/env python3
"""Publish a W1/Y1 entry path and readiness gates from semantic masks."""

from __future__ import annotations

import math
import time

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
    branch_point_distance_m,
    pixels_to_vehicle_path,
    render_sequence_debug,
    select_entry_handoff_condition,
    spatial_steering_gate,
    update_w1_steering_delay_counts,
    w1_steering_delay_ready,
)


def stamp_key(message: Image) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


class SequenceEntryNode(Node):
    """Synchronize masks and expose vehicle-frame W1/Y1 state and path."""

    def __init__(self) -> None:
        super().__init__("shortcut_sequence_entry")
        self.declare_parameter("white_mask_topic", "/shortcut/lraspp/white_mask")
        self.declare_parameter("yellow_mask_topic", "/shortcut/lraspp/yellow_mask")
        self.declare_parameter(
            "processing_enabled_topic", "/hybrid/shortcut_processing_enabled"
        )
        self.declare_parameter("default_enabled", False)
        self.declare_parameter("rule_command_topic", "/hybrid/rule_candidate")
        self.declare_parameter("rule_command_timeout_sec", 0.35)
        self.declare_parameter("speed_command_to_mps", 0.04)
        self.declare_parameter("entry_speed_command", 9.0)
        self.declare_parameter("spatial_gate_response_time_sec", 0.35)
        self.declare_parameter("spatial_gate_minimum_distance_m", 0.25)
        self.declare_parameter("spatial_gate_blend_distance_m", 0.25)
        self.declare_parameter("w1_steering_start_delay_frames", 4)
        self.declare_parameter(
            "w1_steering_delay_missing_tolerance_frames", 2
        )
        self.declare_parameter("minimum_entry_progress_m", 0.50)
        self.declare_parameter("pair_track_handoff_required_frames", 2)
        self.declare_parameter("w1_loss_handoff_enabled", True)
        self.declare_parameter("maximum_entry_steering_sec", 1.5)
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
            "steering_blend_topic", "/shortcut/entry/steering_blend"
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
        self.w1_locked_logged = False
        self.w1_steering_delay_frames = 0
        self.w1_steering_delay_missing_frames = 0
        self.w1_spatial_gate_latched = False
        self.steering_started = False
        self.steering_blend = 0.0
        self.rule_command = (0.0, 0.0)
        self.rule_command_time = float("-inf")
        self.entry_progress_m = 0.0
        self.entry_steering_active_sec = 0.0
        self.entry_progress_update_time = float("-inf")
        self.pair_track_frames = 0
        self.pair_track_confirmed = False
        self.y1_confirmed_once = False
        self.geometry = None
        self.geometry_input_size: tuple[int, int] | None = None

        self.path_publisher = self.create_publisher(
            Centerline, str(self.get_parameter("path_topic").value), 10
        )
        self.ready_publisher = self.create_publisher(
            Bool, str(self.get_parameter("ready_topic").value), state_qos
        )
        self.steering_blend_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("steering_blend_topic").value),
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
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("rule_command_topic").value),
            self.on_rule_command,
            10,
        )
        self.publish_state(False, False, False, 0.0)
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
        self.path_valid_publisher.publish(Bool(data=bool(path_valid)))
        self.cruise_publisher.publish(Bool(data=bool(cruise_handoff)))
        self.phase_publisher.publish(Float32(data=float(phase)))
        self.steering_blend_publisher.publish(
            Float32(data=float(self.steering_blend))
        )
        # Publish authority-ready last so the mux can cache the phase, blend
        # and path/controller state before the hybrid driver sees the gate.
        self.ready_publisher.publish(Bool(data=bool(ready)))

    def on_rule_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            return
        self.rule_command = (float(message.data[0]), float(message.data[1]))
        self.rule_command_time = time.monotonic()

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
        self.w1_locked_logged = False
        self.w1_steering_delay_frames = 0
        self.w1_steering_delay_missing_frames = 0
        self.w1_spatial_gate_latched = False
        self.steering_started = False
        self.steering_blend = 0.0
        self.entry_progress_m = 0.0
        self.entry_steering_active_sec = 0.0
        self.entry_progress_update_time = float("-inf")
        self.pair_track_frames = 0
        self.pair_track_confirmed = False
        self.y1_confirmed_once = False
        self.selector.reset()
        self.publish_state(False, False, False, 0.0)

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
        forward_range_m = float(self.get_parameter("forward_range_m").value)
        branch_distance = branch_point_distance_m(
            result.w1,
            result.white_candidates,
            forward_range_m=forward_range_m,
        )
        now = time.monotonic()
        rule_fresh = (
            now - self.rule_command_time
            <= float(self.get_parameter("rule_command_timeout_sec").value)
        )
        rule_speed_command = self.rule_command[1] if rule_fresh else 0.0
        entry_speed_command = min(
            max(0.0, float(rule_speed_command)),
            max(0.0, float(self.get_parameter("entry_speed_command").value)),
        )
        rule_driving = bool(rule_fresh and entry_speed_command > 0.0)
        gate = spatial_steering_gate(
            branch_distance_m=branch_distance,
            rule_speed_command=entry_speed_command,
            speed_command_to_mps=float(
                self.get_parameter("speed_command_to_mps").value
            ),
            response_time_sec=float(
                self.get_parameter("spatial_gate_response_time_sec").value
            ),
            minimum_trigger_distance_m=float(
                self.get_parameter("spatial_gate_minimum_distance_m").value
            ),
            blend_distance_m=float(
                self.get_parameter("spatial_gate_blend_distance_m").value
            ),
        )
        if result.w1 is not None and not self.w1_locked_logged:
            self.get_logger().info(
                "[MISSION] W1 DETECTED/LOCKED: steering remains RULE until "
                "the spatial branch gate opens"
            )
            self.w1_locked_logged = True
        w1_observed = bool(
            result.w1 is not None and self.selector.w1_missing_frames == 0
        )
        w1_delay_required_frames = max(
            0,
            int(
                self.get_parameter(
                    "w1_steering_start_delay_frames"
                ).value
            ),
        )
        w1_delay_missing_tolerance_frames = max(
            0,
            int(
                self.get_parameter(
                    "w1_steering_delay_missing_tolerance_frames"
                ).value
            ),
        )
        if not self.steering_started:
            if gate.ready and rule_driving and w1_observed:
                self.w1_spatial_gate_latched = True
            if self.w1_spatial_gate_latched and rule_driving:
                (
                    self.w1_steering_delay_frames,
                    self.w1_steering_delay_missing_frames,
                ) = update_w1_steering_delay_counts(
                    observed_frames=self.w1_steering_delay_frames,
                    missing_frames=self.w1_steering_delay_missing_frames,
                    w1_observed=w1_observed,
                    missing_tolerance_frames=(
                        w1_delay_missing_tolerance_frames
                    ),
                )
                if self.w1_steering_delay_frames == 0:
                    self.w1_spatial_gate_latched = False
        w1_delay_ready = w1_steering_delay_ready(
            observed_frames=self.w1_steering_delay_frames,
            required_frames=w1_delay_required_frames,
        )
        if self.steering_started and math.isfinite(
            self.entry_progress_update_time
        ):
            elapsed_sec = max(0.0, now - self.entry_progress_update_time)
            dt_sec = min(
                0.25,
                elapsed_sec,
            )
            self.entry_progress_m += (
                entry_speed_command
                * float(self.get_parameter("speed_command_to_mps").value)
                * dt_sec
            )
            if entry_speed_command > 0.0:
                self.entry_steering_active_sec += elapsed_sec
            self.entry_progress_update_time = now
        if self.w1_spatial_gate_latched and rule_driving and w1_delay_ready:
            if not self.steering_started:
                self.get_logger().warning(
                    "[MISSION] SHORTCUT ENTRY STEERING START: "
                    f"branch={branch_distance:.3f}m "
                    f"trigger={gate.trigger_distance_m:.3f}m "
                    f"W1_delay={self.w1_steering_delay_frames}/"
                    f"{w1_delay_required_frames} "
                    f"gap={self.w1_steering_delay_missing_frames}/"
                    f"{w1_delay_missing_tolerance_frames} "
                    f"entry_speed={entry_speed_command:.2f}"
                )
                self.entry_progress_m = 0.0
                self.entry_steering_active_sec = 0.0
                self.entry_progress_update_time = now
            self.steering_started = True
            self.steering_blend = max(
                self.steering_blend, max(0.05, float(gate.blend))
            )
        elif self.w1_spatial_gate_latched and rule_driving:
            self.get_logger().info(
                "[MISSION] W1 STEERING DELAY: retaining yellow Xbin RULE "
                f"valid={self.w1_steering_delay_frames}/"
                f"{w1_delay_required_frames} "
                f"gap={self.w1_steering_delay_missing_frames}/"
                f"{w1_delay_missing_tolerance_frames}"
            )
        output_header = Header(
            stamp=self.white_message.header.stamp,
            frame_id=str(self.get_parameter("base_frame_id").value),
        )
        if result.path_valid:
            vehicle_path = pixels_to_vehicle_path(
                result.path_pixels,
                width=bev_white.shape[1],
                height=bev_white.shape[0],
                forward_range_m=forward_range_m,
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
            path_message.confidence = (
                0.55 if result.used_synthetic_y1 else 1.0
            )
            path_message.source = (
                "shortcut_W1_synthetic_Y1"
                if result.used_synthetic_y1
                else "shortcut_W1_Y1"
            )
            self.path_publisher.publish(path_message)

        # W1 can be locked and published early, but the hybrid driver keeps
        # RULE authority until the physical W1/W2 branch gate opens.  The
        # aligned visual geometry must then persist through a minimum estimated
        # travel distance so camera alignment alone cannot end the turn early.
        minimum_entry_progress_m = max(
            0.0,
            float(self.get_parameter("minimum_entry_progress_m").value),
        )
        w1_visible = result.w1 is not None
        y1_visible = result.y1 is not None
        if y1_visible:
            self.y1_confirmed_once = True
        if w1_visible and y1_visible:
            self.pair_track_frames += 1
        else:
            self.pair_track_frames = 0
        pair_required_frames = max(
            1,
            int(
                self.get_parameter(
                    "pair_track_handoff_required_frames"
                ).value
            ),
        )
        if self.pair_track_frames >= pair_required_frames:
            self.pair_track_confirmed = True

        handoff_condition = select_entry_handoff_condition(
            geometric_handoff=bool(result.cruise_handoff),
            steering_started=self.steering_started,
            progress_m=self.entry_progress_m,
            minimum_progress_m=minimum_entry_progress_m,
            pair_track_confirmed=self.pair_track_confirmed,
            y1_confirmed=self.y1_confirmed_once,
            w1_visible=w1_visible,
            w1_loss_handoff_enabled=bool(
                self.get_parameter("w1_loss_handoff_enabled").value
            ),
            steering_active_sec=self.entry_steering_active_sec,
            maximum_steering_sec=max(
                0.0,
                float(
                    self.get_parameter("maximum_entry_steering_sec").value
                ),
            ),
        )
        cruise_handoff = handoff_condition is not None
        published_phase = (
            4.0 if cruise_handoff else min(float(result.phase), 3.0)
        )
        self.publish_state(
            self.steering_started,
            result.path_valid,
            cruise_handoff,
            published_phase,
        )
        self.status_publisher.publish(String(data=result.reason))

        if y1_visible and result.y1 is not None:
            w1_slope_text = (
                f"{result.w1.direction_dx_dy:+.3f}"
                if result.w1 is not None
                else "missing"
            )
            self.get_logger().info(
                "\033[92m[MISSION] Y1 DETECTED: "
                f"slope={result.y1.direction_dx_dy:+.3f} "
                f"W1_slope={w1_slope_text} "
                f"phase={int(result.phase)} "
                f"pair={self.pair_track_frames}/{pair_required_frames} "
                f"alignment={self.selector.alignment_frames}/"
                f"{self.selector.config.alignment_required_frames} "
                f"progress={self.entry_progress_m:.3f}/"
                f"{minimum_entry_progress_m:.3f}m "
                f"active={self.entry_steering_active_sec:.2f}s\033[0m"
            )

        if cruise_handoff and not self.handoff_logged:
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
                "SHORTCUT HANDOFF REQUEST: semantic W1 entry -> yellow "
                "Xbin RULE; "
                f"ros_stamp={int(stamp.sec)}.{int(stamp.nanosec):09d} "
                f"semantic_frame={self.semantic_frame_index} "
                f"condition={handoff_condition} "
                f"progress={self.entry_progress_m:.3f}m "
                f"minimum={minimum_entry_progress_m:.3f}m "
                f"W1_slope={w1_slope:+.3f} Y1_slope={y1_slope:+.3f}"
            )
            self.handoff_logged = True

        w1 = result.w1
        y1 = result.y1
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
                    float(branch_distance),
                    float(gate.trigger_distance_m),
                    float(self.steering_blend),
                    float(self.rule_command[1] if rule_fresh else math.nan),
                    float(self.entry_progress_m),
                    float(minimum_entry_progress_m),
                    float(self.pair_track_frames),
                    float(pair_required_frames),
                    1.0 if self.pair_track_confirmed else 0.0,
                    1.0 if self.y1_confirmed_once else 0.0,
                    1.0 if cruise_handoff else 0.0,
                    float(self.entry_steering_active_sec),
                    float(
                        self.get_parameter("maximum_entry_steering_sec").value
                    ),
                    float(self.w1_steering_delay_frames),
                    float(w1_delay_required_frames),
                    float(self.w1_steering_delay_missing_frames),
                    float(w1_delay_missing_tolerance_frames),
                    1.0 if self.w1_spatial_gate_latched else 0.0,
                ]
            )
        )

        debug = render_sequence_debug(bev_white, bev_yellow, result)
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
