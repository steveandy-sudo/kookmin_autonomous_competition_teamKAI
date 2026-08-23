#!/usr/bin/env python3
"""Select the team lane controller or the vehicle-tested cone controller."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import time
from typing import Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, TwistStamped
from my_rule_msgs.msg import ObjectDetectionArray
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray, String

from my_rule.control.lane_controller import (
    CanonicalStanleyPursuitDriver,
    offset_path_left,
    offset_path_right,
)
from my_rule.perception.object_perception import (
    DetectionRecord,
    cone_modalities_match,
    detection_side_counts,
    normalize_class_name,
)


def interpolate_command(
    target_angle_deg: float,
    actual_angles_deg: Sequence[float],
    commands: Sequence[float],
) -> float:
    """Convert a physical wheel angle to the existing Xycar servo command."""
    actual = np.asarray(actual_angles_deg, dtype=np.float64)
    command = np.asarray(commands, dtype=np.float64)
    if actual.size < 2 or actual.size != command.size:
        raise ValueError("steering conversion tables must have equal length >= 2")
    if np.any(np.diff(actual) <= 0.0):
        raise ValueError("physical steering angles must be strictly increasing")
    sign = -1.0 if float(target_angle_deg) < 0.0 else 1.0
    magnitude = abs(float(target_angle_deg))
    mapped = float(np.interp(magnitude, actual, command))
    return sign * mapped


def sector_min_distance(
    scan: LaserScan,
    minimum_angle_deg: float,
    maximum_angle_deg: float,
) -> float:
    """Return the finite positive minimum range inside a scan sector."""
    values = sector_distances(scan, minimum_angle_deg, maximum_angle_deg)
    return min(values) if values else float("inf")


def sector_distances(
    scan: LaserScan,
    minimum_angle_deg: float,
    maximum_angle_deg: float,
) -> list[float]:
    """Return all finite positive ranges inside a scan sector."""
    if not scan.ranges or scan.angle_increment == 0.0:
        return []
    values: list[float] = []
    minimum = math.radians(float(minimum_angle_deg))
    maximum = math.radians(float(maximum_angle_deg))
    for index, distance in enumerate(scan.ranges):
        angle = float(scan.angle_min) + index * float(scan.angle_increment)
        if minimum <= angle <= maximum and math.isfinite(distance) and distance > 0.0:
            values.append(float(distance))
    return values


def initial_traffic_go(
    traffic_control_enabled: bool,
    wait_for_green_at_start: bool,
) -> bool:
    """Choose whether the one-shot startup signal gate begins released."""
    return (
        not bool(traffic_control_enabled)
        or not bool(wait_for_green_at_start)
    )


def signal_approach_speed(
    requested_speed: float,
    area_ratio: float,
    slowdown_area_ratio: float,
    stop_area_ratio: float,
) -> tuple[float, float, bool]:
    """Reduce speed continuously as a calibrated signal box grows."""
    speed = max(0.0, float(requested_speed))
    area = max(0.0, float(area_ratio))
    slowdown = max(0.0, float(slowdown_area_ratio))
    stop = max(0.0, float(stop_area_ratio))
    if stop <= slowdown or stop <= 0.0:
        return speed, 0.0, False
    progress = float(
        np.clip((area - slowdown) / (stop - slowdown), 0.0, 1.0)
    )
    return speed * (1.0 - progress), progress, area >= stop


@dataclass(frozen=True)
class StaticObstacleEstimate:
    """Current YOLO+LiDAR estimate used by the static-obstacle state machine."""

    detected: bool = False
    in_path: bool = False
    close: bool = False
    sensor_confirmed: bool = False
    position: str = "unknown"
    distance_m: float = float("inf")
    position_ratio: float = 0.0
    target_speed_mps: float = float("inf")
    reason: str = "none"


@dataclass(frozen=True)
class DynamicObstacleEstimate:
    """Tracked moving-vehicle state used by the disabled-by-default pass logic."""

    detected: bool = False
    sensor_confirmed: bool = False
    distance_m: float = float("inf")
    second_distance_m: float = float("inf")
    position: str = "unknown"
    vehicle_count: int = 0
    lateral_ratio: float = 0.0
    lateral_velocity_ratio_s: float = 0.0
    lane_stable_frames: int = 0
    closing_speed_mps: float = 0.0
    estimated_speed_command: float = 0.0
    ttc_sec: float = float("inf")
    reason: str = "none"


def inferred_target_speed_mps(
    *,
    previous_distance_m: float,
    current_distance_m: float,
    dt_sec: float,
    ego_speed_command: float,
    speed_gain_mps_per_command: float,
) -> float:
    """Infer longitudinal target speed from ego command and range closure."""
    if (
        not math.isfinite(previous_distance_m)
        or not math.isfinite(current_distance_m)
        or dt_sec <= 0.0
    ):
        return float("inf")
    ego_speed_mps = max(
        0.0,
        float(ego_speed_command) * float(speed_gain_mps_per_command),
    )
    closing_speed_mps = (
        float(previous_distance_m) - float(current_distance_m)
    ) / float(dt_sec)
    return max(0.0, ego_speed_mps - closing_speed_mps)


def obstacle_ttc(
    distance_m: float,
    closing_speed_mps: float,
    minimum_closing_speed_mps: float = 0.05,
) -> float:
    """Return time to collision for an approaching target."""
    if (
        not math.isfinite(distance_m)
        or not math.isfinite(closing_speed_mps)
        or closing_speed_mps
        < max(1.0e-3, float(minimum_closing_speed_mps))
    ):
        return float("inf")
    return max(0.0, float(distance_m)) / float(closing_speed_mps)


def dynamic_follow_speed(
    *,
    obstacle_distance_m: float,
    closing_speed_mps: float,
    estimated_vehicle_speed_command: float,
    current_speed_command: float,
    speed_to_mps_scale: float,
    standstill_gap_m: float,
    target_time_gap_sec: float,
    follow_kp: float,
    closing_gain: float,
    minimum_speed_command: float,
    maximum_speed_command: float,
) -> tuple[float, float]:
    """Return a following command and its speed-dependent target gap."""
    current_speed_mps = max(
        0.1,
        float(current_speed_command) * float(speed_to_mps_scale),
    )
    target_gap = (
        float(standstill_gap_m)
        + float(target_time_gap_sec) * current_speed_mps
    )
    closing = max(0.0, float(closing_speed_mps))
    if closing > 0.0:
        target_gap += closing * closing / 1.6
    gap_error = float(obstacle_distance_m) - target_gap
    requested = (
        float(estimated_vehicle_speed_command)
        + float(follow_kp) * gap_error
        - float(closing_gain) * closing
    )
    return (
        float(
            np.clip(
                requested,
                float(minimum_speed_command),
                float(maximum_speed_command),
            )
        ),
        target_gap,
    )


def choose_static_avoidance_direction(
    *,
    obstacle_position: str,
    left_clear: bool,
    right_clear: bool,
    left_distance_m: float,
    right_distance_m: float,
) -> str | None:
    """Choose the opposite lane first, otherwise the wider verified side."""
    preferred = (
        "right"
        if obstacle_position == "left"
        else "left"
        if obstacle_position == "right"
        else ""
    )
    clear = {"left": bool(left_clear), "right": bool(right_clear)}
    if preferred and clear[preferred]:
        return preferred
    candidates = [side for side in ("left", "right") if clear[side]]
    if not candidates:
        return None
    distances = {
        "left": float(left_distance_m),
        "right": float(right_distance_m),
    }
    return max(candidates, key=lambda side: distances[side])


class DriveManagerNode(CanonicalStanleyPursuitDriver):
    """Run lane control directly and switch the final motor source for cones."""

    def __init__(self) -> None:
        super().__init__(node_name="my_rule_drive_manager")

        defaults = {
            "cone_cmd_topic": "/my_rule/cone_cmd",
            "cone_cluster_topic": "/my_rule/cone_clusters",
            "scan_topic": "/scan",
            "ultra_topic": "/xycar_ultrasonic",
            "object_detections_topic": "/my_rule/object_detections",
            "startup_green_topic": "/my_rule/start_signal_green",
            "course_signal_enable_topic": "/my_rule/course_signal_enable",
            "state_topic": "/my_rule/state",
            "force_cone_mode": False,
            "cone_mode_entry_required_frames": 3,
            "cone_mode_entry_confidence": 0.35,
            "require_yolo_cone_for_entry": True,
            "yolo_cone_class_name": "cone",
            "yolo_cone_min_confidence": 0.50,
            "yolo_cone_min_count": 2,
            "yolo_cone_required_frames": 2,
            "yolo_cone_timeout_sec": 0.75,
            "yolo_cone_min_center_y_ratio": 0.25,
            "cone_sensor_side_deadband_ratio": 0.08,
            "cone_mode_min_duration_sec": 1.0,
            "cone_mode_exit_timeout_sec": 0.8,
            "external_cone_cmd_timeout_sec": 0.5,
            "cone_cluster_timeout_sec": 0.5,
            "cone_mode_presence_min_clusters": 2,
            "cone_mode_lane_recovery_required_frames": 6,
            "cone_mode_lane_recovery_hold_sec": 0.35,
            "cone_mode_exit_creep_timeout_sec": 1.8,
            "cone_exit_creep_speed": 8.0,
            "cone_exit_creep_steer_retention": 0.70,
            # A fresh zero/invalid command from cone_node is an intentional
            # stop, not permission to replay an older steering command.  The
            # legacy hold/creep behaviour remains available for integrated
            # course experiments, but is disabled for the safe default.
            "cone_manager_recovery_enabled": False,
            "cone_fresh_stop_is_authoritative": True,
            "cone_lane_handoff_duration_sec": 0.65,
            "cone_lane_handoff_max_speed": 8.0,
            "minimum_drive_speed": 4.0,
            "cone_emergency_stop_distance_m": 0.35,
            "cone_emergency_clear_distance_m": 0.50,
            "cone_emergency_clear_hold_sec": 0.30,
            "emergency_front_min_angle_deg": -12.0,
            "emergency_front_max_angle_deg": 12.0,
            "motor_publish_rate_hz": 100.0,
            "cone_max_target_angle_deg": 26.0,
            "cone_steering_actual_deg": [0.0, 4.0, 10.0, 16.0, 26.0],
            "cone_steering_command": [0.0, 10.0, 20.0, 30.0, 42.0],
            "enable_traffic_light_control": True,
            "wait_for_green_at_start": True,
            "traffic_startup_only": True,
            "traffic_required_frames": 2,
            "traffic_min_confidence": 0.50,
            "traffic_signal_max_center_y_ratio": 0.55,
            "traffic_signal_min_area_ratio": 0.00005,
            "course_signal_control_enabled": False,
            "course_signal_activation_mode": "mission_state",
            "course_signal_yellow_action": "ignore",
            "course_signal_red_confirm_frames": 2,
            "course_signal_green_confirm_frames": 2,
            "course_signal_yellow_confirm_frames": 2,
            "course_signal_observation_timeout_sec": 0.80,
            "course_signal_slowdown_area_ratio": 0.0,
            "course_signal_stop_area_ratio": 0.0,
            "course_signal_yellow_commit_area_ratio": 0.0,
            "course_signal_lost_action": "hold",
            "enable_static_obstacle_handling": True,
            "static_obstacle_class_name": "car",
            "static_obstacle_min_confidence": 0.50,
            "static_obstacle_required_frames": 3,
            "static_obstacle_timeout_sec": 0.75,
            "static_obstacle_detection_range_m": 2.20,
            "static_obstacle_min_area_ratio": 0.008,
            "static_obstacle_min_center_y_ratio": 0.20,
            "static_obstacle_center_half_width_ratio": 0.24,
            "static_obstacle_stop_distance_m": 1.20,
            "static_obstacle_max_target_speed_mps": 0.14,
            "static_obstacle_motion_alpha": 0.55,
            "static_obstacle_max_range_jump_m": 0.80,
            "static_obstacle_speed": 3.0,
            "static_obstacle_lane_offset_m": 0.30,
            "static_obstacle_change_min_sec": 0.55,
            "static_obstacle_min_avoid_sec": 0.80,
            "static_obstacle_no_side_fallback_sec": 2.0,
            "static_obstacle_clear_required_frames": 4,
            "static_obstacle_side_clear_hold_sec": 0.25,
            "static_obstacle_direction_clear_hold_sec": 0.15,
            "static_obstacle_pass_side_detect_cm": 30.0,
            "static_obstacle_pass_side_clear_cm": 40.0,
            "static_obstacle_rear_clear_cm": 30.0,
            "static_obstacle_front_clear_distance_m": 0.75,
            "static_obstacle_rejoin_sec": 0.80,
            "static_obstacle_rejoin_max_sec": 1.80,
            "static_obstacle_cooldown_sec": 0.80,
            "static_obstacle_bias_command": 16.0,
            "static_obstacle_bias_sec": 0.50,
            "static_obstacle_lidar_hfov_deg": 100.0,
            "static_obstacle_lidar_padding_deg": 3.0,
            "static_obstacle_lidar_min_points": 2,
            "static_obstacle_side_clearance_m": 0.65,
            "static_obstacle_scan_timeout_sec": 0.50,
            "static_obstacle_ultra_timeout_sec": 0.50,
            "static_obstacle_require_fresh_ultra": True,
            "static_obstacle_scan_left_min_deg": 20.0,
            "static_obstacle_scan_left_max_deg": 75.0,
            "static_obstacle_scan_right_min_deg": -75.0,
            "static_obstacle_scan_right_max_deg": -20.0,
            "static_obstacle_ultra_side_clearance_cm": 35.0,
            "static_obstacle_ultra_left_index": 0,
            "static_obstacle_ultra_right_index": 4,
            "static_obstacle_ultra_right_back_index": 5,
            "static_obstacle_ultra_rear_center_index": 6,
            "static_obstacle_ultra_left_back_index": 7,
            "enable_dynamic_obstacle_handling": False,
            "dynamic_obstacle_class_name": "obstacle_vehicle",
            "dynamic_obstacle_min_confidence": 0.45,
            "dynamic_obstacle_min_area_ratio": 0.006,
            "dynamic_obstacle_min_center_y_ratio": 0.20,
            "dynamic_obstacle_center_half_width_ratio": 0.30,
            "dynamic_obstacle_detection_range_m": 3.0,
            "dynamic_obstacle_required_frames": 4,
            "dynamic_obstacle_detection_grace_sec": 0.35,
            "dynamic_obstacle_lidar_hfov_deg": 60.0,
            "dynamic_obstacle_lidar_padding_deg": 3.0,
            "dynamic_obstacle_lidar_min_points": 2,
            "dynamic_obstacle_speed_to_mps_scale": 0.080612,
            "dynamic_obstacle_relative_speed_alpha": 0.35,
            "dynamic_obstacle_max_range_jump_m": 0.80,
            "dynamic_obstacle_max_closing_speed_mps": 1.50,
            "dynamic_obstacle_lateral_deadband_ratio": 0.05,
            "dynamic_obstacle_lateral_velocity_alpha": 0.35,
            "dynamic_obstacle_lateral_motion_threshold_ratio_s": 0.08,
            "dynamic_obstacle_target_time_gap_sec": 2.0,
            "dynamic_obstacle_standstill_gap_m": 0.65,
            "dynamic_obstacle_follow_kp": 0.80,
            "dynamic_obstacle_follow_closing_gain": 4.0,
            "dynamic_obstacle_min_follow_speed": 3.0,
            "dynamic_obstacle_expected_speed_command": 3.0,
            "dynamic_obstacle_prepare_distance_m": 2.40,
            "dynamic_obstacle_min_commit_distance_m": 1.10,
            "dynamic_obstacle_prepare_clear_hold_sec": 0.25,
            "dynamic_obstacle_min_ttc_sec": 3.0,
            "dynamic_obstacle_hard_brake_ttc_sec": 1.60,
            "dynamic_obstacle_emergency_ttc_sec": 0.90,
            "dynamic_obstacle_curve_steer_threshold_command": 8.0,
            "dynamic_obstacle_allow_curve_start": False,
            "dynamic_obstacle_allow_multi_vehicle": False,
            "dynamic_obstacle_multi_vehicle_gap_m": 0.80,
            "dynamic_obstacle_require_lane_path": True,
            "dynamic_obstacle_min_lane_span_m": 0.45,
            "dynamic_obstacle_lane_offset_m": 0.30,
            "dynamic_obstacle_change_speed": 5.0,
            "dynamic_obstacle_change_min_sec": 0.65,
            "dynamic_obstacle_pass_min_speed": 6.5,
            "dynamic_obstacle_pass_max_speed": 8.0,
            "dynamic_obstacle_pass_speed_margin_command": 3.5,
            "dynamic_obstacle_rejoin_speed": 5.0,
            "dynamic_obstacle_rejoin_min_sec": 0.65,
            "dynamic_obstacle_rejoin_max_sec": 1.50,
            "dynamic_obstacle_timeout_sec": 8.0,
            "dynamic_obstacle_timeout_hold_speed": 3.0,
            "dynamic_obstacle_no_side_fallback_sec": 3.0,
            "dynamic_obstacle_clear_required_frames": 4,
            "dynamic_obstacle_front_clear_distance_m": 1.0,
            "dynamic_obstacle_pass_clear_hold_sec": 0.30,
            "dynamic_obstacle_side_emergency_cm": 18.0,
            "dynamic_obstacle_cut_in_hold_sec": 0.60,
            "dynamic_obstacle_abort_hold_sec": 0.80,
            "dynamic_obstacle_cooldown_sec": 1.0,
            "dynamic_obstacle_bias_command": 4.0,
            "dynamic_obstacle_bias_sec": 0.40,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.force_cone_mode = bool(self.get_parameter("force_cone_mode").value)
        self.state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), 10
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("cone_cmd_topic").value),
            self.on_cone_command,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("cone_cluster_topic").value),
            self.on_cone_clusters,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.on_scan,
            sensor_qos,
        )
        self.create_subscription(
            Int32MultiArray,
            str(self.get_parameter("ultra_topic").value),
            self.on_ultrasonic,
            sensor_qos,
        )
        self.create_subscription(
            ObjectDetectionArray,
            str(self.get_parameter("object_detections_topic").value),
            self.on_object_detections,
            sensor_qos,
        )
        startup_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("startup_green_topic").value),
            self.on_startup_green,
            startup_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("course_signal_enable_topic").value),
            self.on_course_signal_enable,
            10,
        )

        self.lane_angle = 0.0
        self.lane_speed = 0.0
        self.lane_command_time = 0.0
        self.last_lane_frame_time: float | None = None
        self.cone_angle = 0.0
        self.cone_speed = 0.0
        self.cone_confidence = 0.0
        self.cone_command_time = 0.0
        self.last_valid_cone_angle = 0.0
        self.last_valid_cone_speed = 0.0
        self.last_valid_cone_time = 0.0
        self.cone_emergency_latched = False
        self.cone_emergency_clear_started_at = 0.0
        self.cone_cluster_count = 0
        self.cone_cluster_left_count = 0
        self.cone_cluster_right_count = 0
        self.cone_cluster_time = 0.0
        self.yolo_cone_count = 0
        self.yolo_cone_left_count = 0
        self.yolo_cone_right_count = 0
        self.yolo_cone_frames = 0
        self.yolo_cone_time = 0.0
        self.yolo_cone_max_confidence = 0.0
        self.last_cone_gate_reason = "idle"
        self.front_distance = float("inf")
        self.left_distance = float("inf")
        self.right_distance = float("inf")
        self.latest_scan: LaserScan | None = None
        self.last_scan_time = 0.0
        self.latest_ultra: list[float] = []
        self.last_ultra_time = 0.0
        self.cone_entry_frames = 0
        self.cone_mode_active = self.force_cone_mode
        self.cone_mode_started_at = time.monotonic() if self.force_cone_mode else 0.0
        self.lane_recovery_frames = 0
        self.lane_recovery_started_at = 0.0
        self.last_counted_canonical_time: float | None = None
        self.handoff_started_at = 0.0
        self.handoff_from_angle = 0.0
        self.handoff_from_speed = 0.0
        self.last_final_angle = 0.0
        self.last_final_speed = 0.0
        self.last_state = ""
        self.last_state_log_time = 0.0
        self.traffic_control_enabled = bool(
            self.get_parameter("enable_traffic_light_control").value
        )
        self.wait_for_green_at_start = bool(
            self.get_parameter("wait_for_green_at_start").value
        )
        self.traffic_startup_only = bool(
            self.get_parameter("traffic_startup_only").value
        )
        self.traffic_go = initial_traffic_go(
            self.traffic_control_enabled,
            self.wait_for_green_at_start,
        )
        self.traffic_red_frames = 0
        self.traffic_green_frames = 0
        self.course_signal_control_enabled = bool(
            self.get_parameter("course_signal_control_enabled").value
        )
        activation_mode = str(
            self.get_parameter("course_signal_activation_mode").value
        ).strip().lower()
        self.course_signal_armed = (
            self.course_signal_control_enabled
            and activation_mode == "always_after_start"
            and self.traffic_go
        )
        self.course_signal_state = "SEARCHING"
        self.course_signal_color = "unknown"
        self.course_signal_area_ratio = 0.0
        self.course_signal_last_seen_time = 0.0
        self.course_signal_red_frames = 0
        self.course_signal_green_frames = 0
        self.course_signal_yellow_frames = 0
        self.static_obstacle_frames = 0
        self.static_obstacle_time = 0.0
        self.static_obstacle_confirmed = False
        self.static_obstacle_estimate = StaticObstacleEstimate()
        self.static_obstacle_state = "CLEAR"
        self.static_obstacle_detection_generation = 0
        self.static_obstacle_last_command_generation = -1
        self.static_obstacle_clear_frames = 0
        self.static_obstacle_previous_distance_m = float("inf")
        self.static_obstacle_previous_distance_time = 0.0
        self.static_obstacle_target_speed_mps = float("inf")
        self.static_obstacle_direction = "center"
        self.static_obstacle_direction_candidate = ""
        self.static_obstacle_direction_clear_since = 0.0
        self.static_obstacle_started_at = 0.0
        self.static_obstacle_phase_started_at = 0.0
        self.static_obstacle_rejoin_started_at = 0.0
        self.static_obstacle_pass_detected = False
        self.static_obstacle_clear_since = 0.0
        self.static_obstacle_cooldown_until = 0.0
        self.dynamic_obstacle_estimate = DynamicObstacleEstimate()
        self.dynamic_obstacle_last_estimate = DynamicObstacleEstimate()
        self.dynamic_obstacle_state = "CLEAR"
        self.dynamic_obstacle_time = 0.0
        self.dynamic_obstacle_last_seen_time = 0.0
        self.dynamic_obstacle_confirm_frames = 0
        self.dynamic_obstacle_clear_frames = 0
        self.dynamic_obstacle_previous_distance_m = float("inf")
        self.dynamic_obstacle_previous_distance_time = 0.0
        self.dynamic_obstacle_closing_speed_mps = 0.0
        self.dynamic_obstacle_track_ratio = 0.0
        self.dynamic_obstacle_track_velocity = 0.0
        self.dynamic_obstacle_track_time = 0.0
        self.dynamic_obstacle_lane_candidate = "unknown"
        self.dynamic_obstacle_lane_candidate_frames = 0
        self.dynamic_obstacle_prepare_clear_since = 0.0
        self.dynamic_obstacle_direction = "center"
        self.dynamic_obstacle_started_at = 0.0
        self.dynamic_obstacle_phase_started_at = 0.0
        self.dynamic_obstacle_pass_detected = False
        self.dynamic_obstacle_clear_since = 0.0
        self.dynamic_obstacle_rejoin_started_at = 0.0
        self.dynamic_obstacle_cut_in_hold_until = 0.0
        self.dynamic_obstacle_abort_hold_until = 0.0
        self.dynamic_obstacle_cooldown_until = 0.0

        rate_hz = max(
            1.0, float(self.get_parameter("motor_publish_rate_hz").value)
        )
        self.create_timer(1.0 / rate_hz, self.on_fast_motor_timer)
        entry_gate = (
            "YOLO+LiDAR"
            if bool(
                self.get_parameter(
                    "require_yolo_cone_for_entry"
                ).value
            )
            else "LiDAR override"
        )
        traffic_mode = (
            "enabled" if self.traffic_control_enabled else "disabled"
        )
        course_signal_mode = (
            "control"
            if self.course_signal_control_enabled
            else "observe"
        )
        self.get_logger().info(
            "drive manager ready: lane=direct canonical 7Hz, "
            f"cone=vehicle-tested controller/{rate_hz:.0f}Hz, "
            f"cone_entry={entry_gate}, traffic={traffic_mode}, "
            f"course_signal={course_signal_mode}"
        )

    def publish_motor(self, angle: float, speed: float) -> None:
        """Receive a lane candidate from the inherited 7 Hz controller."""
        now = time.monotonic()
        self.lane_angle = float(angle)
        self.lane_speed = float(speed)
        self.lane_command_time = now

        lane_message = Float32MultiArray()
        lane_message.data = [self.lane_angle, self.lane_speed]
        self.shadow_motor_pub.publish(lane_message)

        self.update_cone_mode(now)
        final_angle, final_speed, state, reason = self.select_final_command(now)
        self.publish_final(final_angle, final_speed, state, reason, now)

    def on_cone_command(self, message: Float32MultiArray) -> None:
        data = list(message.data)
        if len(data) < 3:
            return
        now = time.monotonic()
        self.cone_angle = float(data[0])
        self.cone_speed = float(data[1])
        self.cone_confidence = float(data[2])
        self.cone_command_time = now
        entry_confidence = float(
            self.get_parameter("cone_mode_entry_confidence").value
        )
        if self.cone_confidence > 0.2:
            self.last_valid_cone_angle = self.cone_angle
            self.last_valid_cone_speed = self.cone_speed
            self.last_valid_cone_time = now
        if self.cone_confidence >= entry_confidence:
            self.cone_entry_frames += 1
        elif not self.cone_mode_active:
            self.cone_entry_frames = 0
        self.update_cone_mode(now)
        if self.cone_mode_active:
            angle, speed, state, reason = self.select_final_command(now)
            self.publish_final(angle, speed, state, reason, now)

    def on_cone_clusters(self, message: PoseArray) -> None:
        self.cone_cluster_count = len(message.poses)
        deadband = 0.03
        self.cone_cluster_left_count = sum(
            1 for pose in message.poses if float(pose.position.y) > deadband
        )
        self.cone_cluster_right_count = sum(
            1 for pose in message.poses if float(pose.position.y) < -deadband
        )
        self.cone_cluster_time = time.monotonic()
        self.update_cone_mode(self.cone_cluster_time)

    def on_scan(self, message: LaserScan) -> None:
        self.latest_scan = message
        self.last_scan_time = time.monotonic()
        self.front_distance = sector_min_distance(
            message,
            float(self.get_parameter("emergency_front_min_angle_deg").value),
            float(self.get_parameter("emergency_front_max_angle_deg").value),
        )
        self.left_distance = sector_min_distance(
            message,
            float(
                self.get_parameter(
                    "static_obstacle_scan_left_min_deg"
                ).value
            ),
            float(
                self.get_parameter(
                    "static_obstacle_scan_left_max_deg"
                ).value
            ),
        )
        self.right_distance = sector_min_distance(
            message,
            float(
                self.get_parameter(
                    "static_obstacle_scan_right_min_deg"
                ).value
            ),
            float(
                self.get_parameter(
                    "static_obstacle_scan_right_max_deg"
                ).value
            ),
        )

    def on_ultrasonic(self, message: Int32MultiArray) -> None:
        self.latest_ultra = [float(value) for value in message.data]
        self.last_ultra_time = time.monotonic()

    def on_startup_green(self, message: Bool) -> None:
        """Permanently release the one-shot startup gate after HSV confirms."""
        if (
            not message.data
            or not self.traffic_control_enabled
            or not self.wait_for_green_at_start
            or self.traffic_go
        ):
            return
        self.traffic_go = True
        self.traffic_red_frames = 0
        self.traffic_green_frames = 0
        activation_mode = str(
            self.get_parameter("course_signal_activation_mode").value
        ).strip().lower()
        if (
            self.course_signal_control_enabled
            and activation_mode == "always_after_start"
        ):
            self.course_signal_armed = True
            self.course_signal_state = "SEARCHING"
        self.get_logger().info(
            "startup green accepted; the start signal gate is permanently "
            "released for this launch"
        )

    def on_course_signal_enable(self, message: Bool) -> None:
        """Arm course-signal control only inside an expected mission zone."""
        if not self.course_signal_control_enabled:
            self.course_signal_armed = False
            return
        self.course_signal_armed = bool(message.data)
        self.course_signal_state = "SEARCHING"
        self.course_signal_color = "unknown"
        self.course_signal_area_ratio = 0.0
        self.course_signal_last_seen_time = 0.0
        self.course_signal_red_frames = 0
        self.course_signal_green_frames = 0
        self.course_signal_yellow_frames = 0
        self.get_logger().info(
            "course signal zone "
            + ("armed" if self.course_signal_armed else "disarmed")
        )

    def on_object_detections(
        self, message: ObjectDetectionArray
    ) -> None:
        now = time.monotonic()
        width = int(message.image_width)
        height = int(message.image_height)
        records = [
            DetectionRecord(
                class_name=normalize_class_name(detection.class_name),
                class_id=int(detection.class_id),
                confidence=float(detection.confidence),
                xmin=int(detection.xmin),
                ymin=int(detection.ymin),
                xmax=int(detection.xmax),
                ymax=int(detection.ymax),
            )
            for detection in message.detections
        ]
        cone_class = normalize_class_name(
            str(self.get_parameter("yolo_cone_class_name").value)
        )
        cone_confidence = float(
            self.get_parameter("yolo_cone_min_confidence").value
        )
        minimum_cone_y = (
            float(
                self.get_parameter(
                    "yolo_cone_min_center_y_ratio"
                ).value
            )
            * height
        )
        cones = [
            record
            for record in records
            if record.class_name == cone_class
            and record.confidence >= cone_confidence
            and record.center_y >= minimum_cone_y
        ]
        (
            self.yolo_cone_count,
            self.yolo_cone_left_count,
            self.yolo_cone_right_count,
        ) = detection_side_counts(
            cones,
            image_width=width,
            class_name=cone_class,
            center_deadband_ratio=float(
                self.get_parameter(
                    "cone_sensor_side_deadband_ratio"
                ).value
            ),
        )
        self.yolo_cone_time = now
        self.yolo_cone_max_confidence = max(
            (record.confidence for record in cones),
            default=0.0,
        )
        if self.yolo_cone_count >= int(
            self.get_parameter("yolo_cone_min_count").value
        ):
            self.yolo_cone_frames += 1
        else:
            self.yolo_cone_frames = 0

        self.update_traffic_state(records, width, height)
        self.update_course_signal_state(records, width, height, now)
        self.update_static_obstacle_state(records, width, height, now)
        self.update_dynamic_obstacle_state(records, width, height, now)
        self.update_cone_mode(now)

    def update_traffic_state(
        self,
        records: Sequence[DetectionRecord],
        image_width: int,
        image_height: int,
    ) -> None:
        if not self.traffic_control_enabled or image_width <= 0 or image_height <= 0:
            return
        if self.traffic_startup_only:
            # The startup gate is released only by high-rate HSV evidence
            # measured inside the detector-owned full traffic-light box.
            # Once released, later signal detections cannot re-latch it.
            return
        threshold = float(
            self.get_parameter("traffic_min_confidence").value
        )
        maximum_y = (
            float(
                self.get_parameter(
                    "traffic_signal_max_center_y_ratio"
                ).value
            )
            * image_height
        )
        minimum_area = (
            float(
                self.get_parameter(
                    "traffic_signal_min_area_ratio"
                ).value
            )
            * image_width
            * image_height
        )
        signals = [
            record
            for record in records
            if record.class_name in {"red", "green"}
            and record.confidence >= threshold
            and record.center_y <= maximum_y
            and record.area >= minimum_area
        ]
        red_seen = any(record.class_name == "red" for record in signals)
        green_seen = any(record.class_name == "green" for record in signals)
        required = max(
            1, int(self.get_parameter("traffic_required_frames").value)
        )
        if red_seen:
            self.traffic_red_frames += 1
            self.traffic_green_frames = 0
            if self.traffic_red_frames >= required:
                self.traffic_go = False
        elif green_seen:
            self.traffic_green_frames += 1
            self.traffic_red_frames = 0
            if self.traffic_green_frames >= required:
                self.traffic_go = True
        else:
            self.traffic_red_frames = 0
            self.traffic_green_frames = 0

    def update_course_signal_state(
        self,
        records: Sequence[DetectionRecord],
        image_width: int,
        image_height: int,
        now: float,
    ) -> None:
        """Track later signals separately from the one-shot start gate."""
        if (
            not self.traffic_go
            or image_width <= 0
            or image_height <= 0
        ):
            return
        threshold = float(
            self.get_parameter("traffic_min_confidence").value
        )
        maximum_y = (
            float(
                self.get_parameter(
                    "traffic_signal_max_center_y_ratio"
                ).value
            )
            * image_height
        )
        minimum_area = (
            float(
                self.get_parameter(
                    "traffic_signal_min_area_ratio"
                ).value
            )
            * image_width
            * image_height
        )
        signals = [
            record
            for record in records
            if record.class_name in {"red", "yellow", "green"}
            and record.confidence >= threshold
            and record.center_y <= maximum_y
            and record.area >= minimum_area
        ]
        # Safety priority when model outputs overlap: red, yellow, then green.
        selected = next(
            (
                max(
                    (
                        record
                        for record in signals
                        if record.class_name == color
                    ),
                    key=lambda record: record.confidence,
                )
                for color in ("red", "yellow", "green")
                if any(record.class_name == color for record in signals)
            ),
            None,
        )
        if selected is None:
            self.course_signal_red_frames = 0
            self.course_signal_green_frames = 0
            self.course_signal_yellow_frames = 0
            return

        color = selected.class_name
        self.course_signal_color = color
        self.course_signal_area_ratio = selected.area / float(
            image_width * image_height
        )
        self.course_signal_last_seen_time = now
        if color == "red":
            self.course_signal_red_frames += 1
            self.course_signal_green_frames = 0
            self.course_signal_yellow_frames = 0
            required = max(
                1,
                int(
                    self.get_parameter(
                        "course_signal_red_confirm_frames"
                    ).value
                ),
            )
            if self.course_signal_red_frames >= required:
                self.course_signal_state = "RED_APPROACH"
        elif color == "green":
            self.course_signal_green_frames += 1
            self.course_signal_red_frames = 0
            self.course_signal_yellow_frames = 0
            required = max(
                1,
                int(
                    self.get_parameter(
                        "course_signal_green_confirm_frames"
                    ).value
                ),
            )
            if self.course_signal_green_frames >= required:
                self.course_signal_state = "GREEN_PASS"
        else:
            self.course_signal_yellow_frames += 1
            self.course_signal_red_frames = 0
            self.course_signal_green_frames = 0
            required = max(
                1,
                int(
                    self.get_parameter(
                        "course_signal_yellow_confirm_frames"
                    ).value
                ),
            )
            if self.course_signal_yellow_frames >= required:
                action = str(
                    self.get_parameter(
                        "course_signal_yellow_action"
                    ).value
                ).strip().lower()
                if action == "prepare_stop":
                    self.course_signal_state = "YELLOW_APPROACH"
                elif action == "dilemma_zone":
                    commit_area = float(
                        self.get_parameter(
                            "course_signal_yellow_commit_area_ratio"
                        ).value
                    )
                    self.course_signal_state = (
                        "YELLOW_PASS"
                        if commit_area > 0.0
                        and self.course_signal_area_ratio >= commit_area
                        else "YELLOW_APPROACH"
                    )
                # "ignore" deliberately preserves the previous control state.

    def update_static_obstacle_state(
        self,
        records: Sequence[DetectionRecord],
        image_width: int,
        image_height: int,
        now: float,
    ) -> None:
        self.static_obstacle_detection_generation += 1
        if not bool(
            self.get_parameter("enable_static_obstacle_handling").value
        ):
            self.reset_static_obstacle_state()
            return
        target = normalize_class_name(
            str(self.get_parameter("static_obstacle_class_name").value)
        )
        minimum_confidence = float(
            self.get_parameter("static_obstacle_min_confidence").value
        )
        minimum_y = (
            float(
                self.get_parameter(
                    "static_obstacle_min_center_y_ratio"
                ).value
            )
            * image_height
        )
        half_width = (
            float(
                self.get_parameter(
                    "static_obstacle_center_half_width_ratio"
                ).value
            )
            * image_width
        )
        image_center = 0.5 * image_width
        minimum_area = (
            float(
                self.get_parameter(
                    "static_obstacle_min_area_ratio"
                ).value
            )
            * image_width
            * image_height
        )
        candidates = [
            record
            for record in records
            if record.class_name == target
            and record.confidence >= minimum_confidence
            and record.center_y >= minimum_y
            and record.area >= minimum_area
            and record.xmax >= image_center - half_width
            and record.xmin <= image_center + half_width
        ]
        if not candidates:
            self.static_obstacle_estimate = StaticObstacleEstimate()
            self.static_obstacle_time = now
            self.static_obstacle_confirmed = False
            self.static_obstacle_frames = 0
            if self.static_obstacle_state == "PASS":
                self.static_obstacle_clear_frames += 1
            return

        record = max(candidates, key=lambda item: item.area)
        position_ratio = (
            (record.center_x - image_center) / max(1.0, half_width)
        )
        position = (
            "left"
            if position_ratio < -0.20
            else "right"
            if position_ratio > 0.20
            else "center"
        )
        in_path = (
            record.xmax >= image_center - half_width
            and record.xmin <= image_center + half_width
        )
        distance = self.static_obstacle_lidar_distance(
            record,
            image_width,
            now,
        )
        distance_source = "bbox_lidar"
        if not math.isfinite(distance) and in_path and math.isfinite(
            self.front_distance
        ):
            distance = self.front_distance
            distance_source = "front_lidar"
        sensor_confirmed = math.isfinite(distance)
        detection_range = float(
            self.get_parameter("static_obstacle_detection_range_m").value
        )
        close = sensor_confirmed and distance <= detection_range

        target_speed = self.update_static_obstacle_motion(distance, now)
        stationary = (
            math.isfinite(target_speed)
            and target_speed
            <= float(
                self.get_parameter(
                    "static_obstacle_max_target_speed_mps"
                ).value
            )
        )
        if close and in_path and sensor_confirmed and stationary:
            self.static_obstacle_frames += 1
        else:
            self.static_obstacle_frames = 0
        required = max(
            1,
            int(
                self.get_parameter(
                    "static_obstacle_required_frames"
                ).value
            ),
        )
        self.static_obstacle_confirmed = (
            self.static_obstacle_frames >= required
        )
        self.static_obstacle_time = now
        self.static_obstacle_estimate = StaticObstacleEstimate(
            detected=True,
            in_path=in_path,
            close=close,
            sensor_confirmed=sensor_confirmed,
            position=position,
            distance_m=distance,
            position_ratio=position_ratio,
            target_speed_mps=target_speed,
            reason=(
                f"{target}:pos={position_ratio:+.2f}:"
                f"dist={distance:.2f}m:{distance_source}:"
                f"target_v={target_speed:.2f}mps:"
                f"static={self.static_obstacle_frames}/{required}"
            ),
        )
        if self.static_obstacle_state == "PASS":
            self.static_obstacle_clear_frames = 0

    def static_obstacle_lidar_distance(
        self,
        record: DetectionRecord,
        image_width: int,
        now: float,
    ) -> float:
        if (
            self.latest_scan is None
            or self.last_scan_time <= 0.0
            or now - self.last_scan_time
            > float(
                self.get_parameter(
                    "static_obstacle_scan_timeout_sec"
                ).value
            )
            or image_width <= 0
        ):
            return float("inf")
        half_fov = 0.5 * float(
            self.get_parameter("static_obstacle_lidar_hfov_deg").value
        )
        center_ratio = (
            record.center_x - 0.5 * image_width
        ) / max(1.0, 0.5 * image_width)
        center_angle = -center_ratio * half_fov
        bbox_half_angle = (
            0.5 * record.width / max(1.0, float(image_width))
        ) * 2.0 * half_fov
        padding = float(
            self.get_parameter(
                "static_obstacle_lidar_padding_deg"
            ).value
        )
        values = sector_distances(
            self.latest_scan,
            center_angle - bbox_half_angle - padding,
            center_angle + bbox_half_angle + padding,
        )
        minimum_points = max(
            1,
            int(
                self.get_parameter(
                    "static_obstacle_lidar_min_points"
                ).value
            ),
        )
        if len(values) < minimum_points:
            return float("inf")
        return float(np.percentile(np.asarray(values), 20.0))

    def update_static_obstacle_motion(
        self,
        distance_m: float,
        now: float,
    ) -> float:
        if not math.isfinite(distance_m):
            self.static_obstacle_previous_distance_m = float("inf")
            self.static_obstacle_previous_distance_time = 0.0
            self.static_obstacle_target_speed_mps = float("inf")
            return float("inf")
        target_speed = float("inf")
        if (
            self.static_obstacle_previous_distance_time > 0.0
            and math.isfinite(self.static_obstacle_previous_distance_m)
        ):
            dt = now - self.static_obstacle_previous_distance_time
            jump = (
                self.static_obstacle_previous_distance_m - distance_m
            )
            if (
                0.02 <= dt <= 1.5
                and abs(jump)
                <= float(
                    self.get_parameter(
                        "static_obstacle_max_range_jump_m"
                    ).value
                )
            ):
                target_speed = inferred_target_speed_mps(
                    previous_distance_m=(
                        self.static_obstacle_previous_distance_m
                    ),
                    current_distance_m=distance_m,
                    dt_sec=dt,
                    ego_speed_command=self.last_final_speed,
                    speed_gain_mps_per_command=float(
                        self.get_parameter(
                            "speed_gain_mps_per_cmd"
                        ).value
                    ),
                )
                if math.isfinite(self.static_obstacle_target_speed_mps):
                    alpha = float(
                        np.clip(
                            self.get_parameter(
                                "static_obstacle_motion_alpha"
                            ).value,
                            0.0,
                            1.0,
                        )
                    )
                    target_speed = (
                        alpha * target_speed
                        + (1.0 - alpha)
                        * self.static_obstacle_target_speed_mps
                    )
        self.static_obstacle_previous_distance_m = float(distance_m)
        self.static_obstacle_previous_distance_time = now
        self.static_obstacle_target_speed_mps = target_speed
        return target_speed

    def static_obstacle_scan_is_fresh(self, now: float) -> bool:
        return (
            self.last_scan_time > 0.0
            and now - self.last_scan_time
            <= float(
                self.get_parameter(
                    "static_obstacle_scan_timeout_sec"
                ).value
            )
        )

    def static_obstacle_ultra_is_fresh(self, now: float) -> bool:
        return (
            self.last_ultra_time > 0.0
            and now - self.last_ultra_time
            <= float(
                self.get_parameter(
                    "static_obstacle_ultra_timeout_sec"
                ).value
            )
        )

    def static_obstacle_ultra_values_for_side(
        self,
        side: str,
    ) -> list[float]:
        if side == "left":
            indices = (
                int(
                    self.get_parameter(
                        "static_obstacle_ultra_left_index"
                    ).value
                ),
                int(
                    self.get_parameter(
                        "static_obstacle_ultra_left_back_index"
                    ).value
                ),
            )
        else:
            indices = (
                int(
                    self.get_parameter(
                        "static_obstacle_ultra_right_index"
                    ).value
                ),
                int(
                    self.get_parameter(
                        "static_obstacle_ultra_right_back_index"
                    ).value
                ),
            )
        values: list[float] = []
        for index in indices:
            if 0 <= index < len(self.latest_ultra):
                value = float(self.latest_ultra[index])
                if value > 0.0:
                    values.append(value)
        return values

    def static_obstacle_rear_center_clear(self, now: float) -> bool:
        if bool(
            self.get_parameter(
                "static_obstacle_require_fresh_ultra"
            ).value
        ) and not self.static_obstacle_ultra_is_fresh(now):
            return False
        index = int(
            self.get_parameter(
                "static_obstacle_ultra_rear_center_index"
            ).value
        )
        if not (0 <= index < len(self.latest_ultra)):
            return not bool(
                self.get_parameter(
                    "static_obstacle_require_fresh_ultra"
                ).value
            )
        value = float(self.latest_ultra[index])
        if value <= 0.0:
            return not bool(
                self.get_parameter(
                    "static_obstacle_require_fresh_ultra"
                ).value
            )
        return value > float(
            self.get_parameter(
                "static_obstacle_rear_clear_cm"
            ).value
        )

    def static_obstacle_side_is_clear(
        self,
        side: str,
        now: float,
    ) -> bool:
        if not self.static_obstacle_scan_is_fresh(now):
            return False
        scan_distance = (
            self.left_distance if side == "left" else self.right_distance
        )
        scan_clear = (
            math.isfinite(scan_distance)
            and scan_distance
            > float(
                self.get_parameter(
                    "static_obstacle_side_clearance_m"
                ).value
            )
        )
        if not scan_clear:
            return False

        require_ultra = bool(
            self.get_parameter(
                "static_obstacle_require_fresh_ultra"
            ).value
        )
        if require_ultra and not self.static_obstacle_ultra_is_fresh(now):
            return False
        values = self.static_obstacle_ultra_values_for_side(side)
        if not values:
            return not require_ultra
        return min(values) > float(
            self.get_parameter(
                "static_obstacle_ultra_side_clearance_cm"
            ).value
        )

    def static_obstacle_side_detected(self, side: str) -> bool:
        values = self.static_obstacle_ultra_values_for_side(side)
        ultra_detected = (
            bool(values)
            and min(values)
            < float(
                self.get_parameter(
                    "static_obstacle_pass_side_detect_cm"
                ).value
            )
        )
        scan_distance = (
            self.left_distance if side == "left" else self.right_distance
        )
        scan_detected = (
            math.isfinite(scan_distance)
            and scan_distance
            < float(
                self.get_parameter(
                    "static_obstacle_side_clearance_m"
                ).value
            )
        )
        return ultra_detected or scan_detected

    def static_obstacle_side_cleared(
        self,
        side: str,
        now: float,
    ) -> bool:
        if not self.static_obstacle_scan_is_fresh(now):
            return False
        scan_distance = (
            self.left_distance if side == "left" else self.right_distance
        )
        scan_cleared = (
            math.isfinite(scan_distance)
            and scan_distance
            > float(
                self.get_parameter(
                    "static_obstacle_side_clearance_m"
                ).value
            )
        )
        if not scan_cleared:
            return False
        require_ultra = bool(
            self.get_parameter(
                "static_obstacle_require_fresh_ultra"
            ).value
        )
        if require_ultra and not self.static_obstacle_ultra_is_fresh(now):
            return False
        values = self.static_obstacle_ultra_values_for_side(side)
        ultra_cleared = (
            bool(values)
            and min(values)
            > float(
                self.get_parameter(
                    "static_obstacle_pass_side_clear_cm"
                ).value
            )
        )
        if not values:
            ultra_cleared = not require_ultra
        return (
            scan_cleared
            and ultra_cleared
            and self.static_obstacle_rear_center_clear(now)
        )

    def static_obstacle_avoid_direction(
        self,
        estimate: StaticObstacleEstimate,
        now: float,
    ) -> str | None:
        return choose_static_avoidance_direction(
            obstacle_position=estimate.position,
            left_clear=self.static_obstacle_side_is_clear("left", now),
            right_clear=self.static_obstacle_side_is_clear("right", now),
            left_distance_m=self.left_distance,
            right_distance_m=self.right_distance,
        )

    def static_obstacle_steering(
        self,
        direction: str,
        offset_fraction: float = 1.0,
    ) -> float:
        fraction = float(np.clip(offset_fraction, 0.0, 1.0))
        offset = (
            float(
                self.get_parameter(
                    "static_obstacle_lane_offset_m"
                ).value
            )
            * fraction
        )
        if self.path_valid and self.latest_path is not None:
            shifted = (
                offset_path_left(self.latest_path, offset)
                if direction == "left"
                else offset_path_right(self.latest_path, offset)
            )
            steering, _ = self.steering_command_for_path(shifted)
            return float(
                np.clip(
                    steering,
                    self.angle_command_min,
                    self.angle_command_max,
                )
            )
        sign = -1.0 if direction == "left" else 1.0
        return float(
            np.clip(
                self.lane_angle
                + sign
                * float(
                    self.get_parameter(
                        "static_obstacle_bias_command"
                    ).value
                )
                * fraction,
                self.angle_command_min,
                self.angle_command_max,
            )
        )

    def reset_static_obstacle_state(self) -> None:
        self.static_obstacle_frames = 0
        self.static_obstacle_confirmed = False
        self.static_obstacle_state = "CLEAR"
        self.static_obstacle_clear_frames = 0
        self.static_obstacle_direction = "center"
        self.static_obstacle_direction_candidate = ""
        self.static_obstacle_direction_clear_since = 0.0
        self.static_obstacle_started_at = 0.0
        self.static_obstacle_phase_started_at = 0.0
        self.static_obstacle_rejoin_started_at = 0.0
        self.static_obstacle_pass_detected = False
        self.static_obstacle_clear_since = 0.0
        self.static_obstacle_cooldown_until = 0.0

    def static_obstacle_command(
        self,
        now: float,
    ) -> tuple[float, float, str, str] | None:
        if not bool(
            self.get_parameter("enable_static_obstacle_handling").value
        ):
            self.reset_static_obstacle_state()
            return None

        estimate = self.static_obstacle_estimate
        fresh = (
            self.static_obstacle_time > 0.0
            and now - self.static_obstacle_time
            <= float(
                self.get_parameter(
                    "static_obstacle_timeout_sec"
                ).value
            )
        )

        if self.static_obstacle_state == "COOLDOWN":
            if now < self.static_obstacle_cooldown_until:
                return None
            self.reset_static_obstacle_state()

        candidate = (
            fresh
            and estimate.detected
            and estimate.in_path
            and estimate.close
            and estimate.sensor_confirmed
        )
        if self.static_obstacle_state == "CLEAR":
            if candidate and self.static_obstacle_confirmed:
                if self.cone_mode_active:
                    return (
                        0.0,
                        0.0,
                        "STATIC_OBSTACLE_HOLD",
                        "confirmed_car_in_cone_mode",
                    )
                self.static_obstacle_state = "WAIT_CLEAR_SIDE"
                self.static_obstacle_direction_candidate = ""
                self.static_obstacle_direction_clear_since = 0.0
            elif (
                candidate
                and estimate.distance_m
                <= float(
                    self.get_parameter(
                        "static_obstacle_stop_distance_m"
                    ).value
                )
            ):
                return (
                    self.lane_angle,
                    0.0,
                    "STATIC_OBSTACLE_HOLD",
                    f"classifying_car:{estimate.reason}",
                )
            else:
                return None

        if self.cone_mode_active:
            return (
                0.0,
                0.0,
                "STATIC_OBSTACLE_HOLD",
                "avoidance_deferred_during_cone_mode",
            )

        if self.static_obstacle_state == "WAIT_CLEAR_SIDE":
            direction = self.static_obstacle_avoid_direction(estimate, now)
            if direction is None:
                self.static_obstacle_direction_candidate = ""
                self.static_obstacle_direction_clear_since = 0.0
                return (
                    self.lane_angle,
                    0.0,
                    "STATIC_OBSTACLE_WAIT",
                    "both_sides_blocked_or_sensor_stale",
                )
            if direction != self.static_obstacle_direction_candidate:
                self.static_obstacle_direction_candidate = direction
                self.static_obstacle_direction_clear_since = now
            elapsed = now - self.static_obstacle_direction_clear_since
            if elapsed < float(
                self.get_parameter(
                    "static_obstacle_direction_clear_hold_sec"
                ).value
            ):
                return (
                    self.lane_angle,
                    0.0,
                    "STATIC_OBSTACLE_WAIT",
                    f"verify_{direction}={elapsed:.2f}s",
                )
            self.static_obstacle_direction = direction
            self.static_obstacle_state = "CHANGE_OUT"
            self.static_obstacle_started_at = now
            self.static_obstacle_phase_started_at = now
            self.static_obstacle_clear_frames = 0
            self.static_obstacle_pass_detected = False
            self.static_obstacle_clear_since = 0.0

        speed = float(
            self.get_parameter("static_obstacle_speed").value
        )
        if self.static_obstacle_state == "CHANGE_OUT":
            elapsed = now - self.static_obstacle_started_at
            phase_elapsed = now - self.static_obstacle_phase_started_at
            obstacle_side = (
                "right"
                if self.static_obstacle_direction == "left"
                else "left"
            )
            if (
                self.static_obstacle_side_detected(obstacle_side)
                or phase_elapsed
                >= float(
                    self.get_parameter(
                        "static_obstacle_change_min_sec"
                    ).value
                )
            ):
                self.static_obstacle_pass_detected = (
                    self.static_obstacle_pass_detected
                    or self.static_obstacle_side_detected(obstacle_side)
                )
                self.static_obstacle_state = "PASS"
                self.static_obstacle_phase_started_at = now
            bias_sec = max(
                0.01,
                float(
                    self.get_parameter(
                        "static_obstacle_bias_sec"
                    ).value
                ),
            )
            bias_fraction = float(
                np.clip(1.0 - elapsed / bias_sec, 0.0, 1.0)
            )
            steering = self.static_obstacle_steering(
                self.static_obstacle_direction
            )
            sign = (
                -1.0
                if self.static_obstacle_direction == "left"
                else 1.0
            )
            steering = float(
                np.clip(
                    steering
                    + sign
                    * float(
                        self.get_parameter(
                            "static_obstacle_bias_command"
                        ).value
                    )
                    * bias_fraction,
                    self.angle_command_min,
                    self.angle_command_max,
                )
            )
            return (
                steering,
                speed,
                "STATIC_OBSTACLE_AVOID",
                f"change_out:{self.static_obstacle_direction}:"
                f"{estimate.reason}",
            )

        if self.static_obstacle_state == "PASS":
            elapsed = now - self.static_obstacle_started_at
            obstacle_side = (
                "right"
                if self.static_obstacle_direction == "left"
                else "left"
            )
            side_detected = self.static_obstacle_side_detected(
                obstacle_side
            )
            side_cleared = self.static_obstacle_side_cleared(
                obstacle_side,
                now,
            )
            if side_detected:
                self.static_obstacle_pass_detected = True
                self.static_obstacle_clear_since = 0.0
            no_longer_in_path = not (
                fresh and estimate.detected and estimate.in_path
            )
            if (
                self.static_obstacle_pass_detected
                and side_cleared
                and no_longer_in_path
            ):
                if self.static_obstacle_clear_since <= 0.0:
                    self.static_obstacle_clear_since = now
            elif side_detected:
                self.static_obstacle_clear_since = 0.0
            side_clear_held = (
                self.static_obstacle_clear_since > 0.0
                and now - self.static_obstacle_clear_since
                >= float(
                    self.get_parameter(
                        "static_obstacle_side_clear_hold_sec"
                    ).value
                )
            )
            primary_complete = (
                elapsed
                >= float(
                    self.get_parameter(
                        "static_obstacle_min_avoid_sec"
                    ).value
                )
                and self.static_obstacle_pass_detected
                and side_clear_held
            )
            fallback_complete = (
                elapsed
                >= float(
                    self.get_parameter(
                        "static_obstacle_no_side_fallback_sec"
                    ).value
                )
                and self.static_obstacle_clear_frames
                >= int(
                    self.get_parameter(
                        "dynamic_obstacle_clear_required_frames"
                    ).value
                )
                and math.isfinite(self.front_distance)
                and self.front_distance
                > float(
                    self.get_parameter(
                        "static_obstacle_front_clear_distance_m"
                    ).value
                )
            )
            if primary_complete or fallback_complete:
                self.static_obstacle_state = "REJOIN"
                self.static_obstacle_rejoin_started_at = now
            else:
                return (
                    self.static_obstacle_steering(
                        self.static_obstacle_direction
                    ),
                    speed,
                    "STATIC_OBSTACLE_AVOID",
                    f"pass:{self.static_obstacle_direction}:"
                    f"side_seen={int(self.static_obstacle_pass_detected)}:"
                    f"side_clear={int(side_clear_held)}:"
                    f"visual_clear={self.static_obstacle_clear_frames}",
                )

        if self.static_obstacle_state == "REJOIN":
            elapsed = now - self.static_obstacle_rejoin_started_at
            duration = max(
                0.05,
                float(
                    self.get_parameter(
                        "static_obstacle_rejoin_sec"
                    ).value
                ),
            )
            progress = float(np.clip(elapsed / duration, 0.0, 1.0))
            offset_fraction = 1.0 - progress * progress * (
                3.0 - 2.0 * progress
            )
            steering = self.static_obstacle_steering(
                self.static_obstacle_direction,
                offset_fraction,
            )
            ready = elapsed >= duration and self.path_valid
            timed_out = elapsed >= float(
                self.get_parameter(
                    "static_obstacle_rejoin_max_sec"
                ).value
            )
            if ready or timed_out:
                self.reset_static_obstacle_state()
                self.static_obstacle_state = "COOLDOWN"
                self.static_obstacle_cooldown_until = (
                    now
                    + float(
                        self.get_parameter(
                            "static_obstacle_cooldown_sec"
                        ).value
                    )
                )
            return (
                steering,
                speed,
                "STATIC_OBSTACLE_REJOIN",
                f"progress={progress:.2f}:"
                f"path={int(self.path_valid)}:"
                f"timeout={int(timed_out)}",
            )
        return None

    def reset_dynamic_obstacle_state(
        self,
        *,
        clear_tracking: bool = True,
    ) -> None:
        self.dynamic_obstacle_state = "CLEAR"
        self.dynamic_obstacle_confirm_frames = 0
        self.dynamic_obstacle_clear_frames = 0
        self.dynamic_obstacle_prepare_clear_since = 0.0
        self.dynamic_obstacle_direction = "center"
        self.dynamic_obstacle_started_at = 0.0
        self.dynamic_obstacle_phase_started_at = 0.0
        self.dynamic_obstacle_pass_detected = False
        self.dynamic_obstacle_clear_since = 0.0
        self.dynamic_obstacle_rejoin_started_at = 0.0
        self.dynamic_obstacle_cut_in_hold_until = 0.0
        self.dynamic_obstacle_cooldown_until = 0.0
        if clear_tracking:
            self.dynamic_obstacle_previous_distance_m = float("inf")
            self.dynamic_obstacle_previous_distance_time = 0.0
            self.dynamic_obstacle_closing_speed_mps = 0.0
            self.dynamic_obstacle_track_ratio = 0.0
            self.dynamic_obstacle_track_velocity = 0.0
            self.dynamic_obstacle_track_time = 0.0
            self.dynamic_obstacle_lane_candidate = "unknown"
            self.dynamic_obstacle_lane_candidate_frames = 0

    def update_dynamic_obstacle_state(
        self,
        records: Sequence[DetectionRecord],
        image_width: int,
        image_height: int,
        now: float,
    ) -> None:
        if not bool(
            self.get_parameter("enable_dynamic_obstacle_handling").value
        ):
            self.dynamic_obstacle_estimate = DynamicObstacleEstimate()
            return
        target = normalize_class_name(
            str(
                self.get_parameter(
                    "dynamic_obstacle_class_name"
                ).value
            )
        )
        confidence = float(
            self.get_parameter(
                "dynamic_obstacle_min_confidence"
            ).value
        )
        minimum_area = (
            float(
                self.get_parameter(
                    "dynamic_obstacle_min_area_ratio"
                ).value
            )
            * image_width
            * image_height
        )
        minimum_y = (
            float(
                self.get_parameter(
                    "dynamic_obstacle_min_center_y_ratio"
                ).value
            )
            * image_height
        )
        candidates: list[
            tuple[float, DetectionRecord, bool, str]
        ] = []
        image_center = 0.5 * image_width
        corridor_half = (
            float(
                self.get_parameter(
                    "dynamic_obstacle_center_half_width_ratio"
                ).value
            )
            * image_width
        )
        for record in records:
            if (
                record.class_name != target
                or record.confidence < confidence
                or record.area < minimum_area
                or record.center_y < minimum_y
            ):
                continue
            distance = self.dynamic_obstacle_lidar_distance(
                record,
                image_width,
                now,
            )
            source = "bbox_lidar"
            overlaps_corridor = (
                record.xmax >= image_center - corridor_half
                and record.xmin <= image_center + corridor_half
            )
            if (
                not math.isfinite(distance)
                and overlaps_corridor
                and math.isfinite(self.front_distance)
                and self.static_obstacle_scan_is_fresh(now)
            ):
                distance = self.front_distance
                source = "front_lidar"
            candidates.append(
                (distance, record, math.isfinite(distance), source)
            )

        self.dynamic_obstacle_time = now
        if not candidates:
            self.dynamic_obstacle_estimate = DynamicObstacleEstimate()
            if self.dynamic_obstacle_state == "PASS":
                self.dynamic_obstacle_clear_frames += 1
            return

        candidates.sort(
            key=lambda item: (
                item[0] if math.isfinite(item[0]) else float("inf")
            )
        )
        distance, record, sensor_confirmed, source = candidates[0]
        second_distance = (
            candidates[1][0] if len(candidates) > 1 else float("inf")
        )
        (
            position,
            lateral_ratio,
            lateral_velocity,
            stable_frames,
        ) = self.update_dynamic_lateral_track(
            record,
            image_width,
            now,
        )
        (
            closing_speed,
            estimated_speed,
            ttc,
        ) = self.update_dynamic_longitudinal_track(
            distance,
            now,
            sensor_confirmed,
        )
        detected = (
            sensor_confirmed
            and distance
            <= float(
                self.get_parameter(
                    "dynamic_obstacle_detection_range_m"
                ).value
            )
        )
        estimate = DynamicObstacleEstimate(
            detected=detected,
            sensor_confirmed=sensor_confirmed,
            distance_m=distance,
            second_distance_m=second_distance,
            position=position,
            vehicle_count=len(candidates),
            lateral_ratio=lateral_ratio,
            lateral_velocity_ratio_s=lateral_velocity,
            lane_stable_frames=stable_frames,
            closing_speed_mps=closing_speed,
            estimated_speed_command=estimated_speed,
            ttc_sec=ttc,
            reason=(
                f"{target}:{source}:pos={position}:"
                f"d={distance:.2f}m:v={estimated_speed:.2f}:"
                f"closing={closing_speed:+.2f}mps:"
                f"ttc={ttc:.2f}s"
            ),
        )
        self.dynamic_obstacle_estimate = estimate
        if detected:
            self.dynamic_obstacle_last_estimate = estimate
            self.dynamic_obstacle_last_seen_time = now
        if self.dynamic_obstacle_state == "PASS":
            self.dynamic_obstacle_clear_frames = 0

    def dynamic_obstacle_lidar_distance(
        self,
        record: DetectionRecord,
        image_width: int,
        now: float,
    ) -> float:
        if (
            self.latest_scan is None
            or not self.static_obstacle_scan_is_fresh(now)
            or image_width <= 0
        ):
            return float("inf")
        half_fov = 0.5 * float(
            self.get_parameter(
                "dynamic_obstacle_lidar_hfov_deg"
            ).value
        )
        center_ratio = (
            record.center_x - 0.5 * image_width
        ) / max(1.0, 0.5 * image_width)
        center_angle = -center_ratio * half_fov
        bbox_half_angle = (
            record.width / max(1.0, float(image_width))
        ) * half_fov
        padding = float(
            self.get_parameter(
                "dynamic_obstacle_lidar_padding_deg"
            ).value
        )
        values = sector_distances(
            self.latest_scan,
            center_angle - bbox_half_angle - padding,
            center_angle + bbox_half_angle + padding,
        )
        minimum_points = max(
            1,
            int(
                self.get_parameter(
                    "dynamic_obstacle_lidar_min_points"
                ).value
            ),
        )
        if len(values) < minimum_points:
            return float("inf")
        return float(np.percentile(np.asarray(values), 20.0))

    def update_dynamic_lateral_track(
        self,
        record: DetectionRecord,
        image_width: int,
        now: float,
    ) -> tuple[str, float, float, int]:
        ratio = (
            record.center_x - 0.5 * image_width
        ) / max(1.0, float(image_width))
        velocity = 0.0
        if self.dynamic_obstacle_track_time > 0.0:
            dt = max(0.02, now - self.dynamic_obstacle_track_time)
            measured = (
                ratio - self.dynamic_obstacle_track_ratio
            ) / dt
            alpha = float(
                np.clip(
                    self.get_parameter(
                        "dynamic_obstacle_lateral_velocity_alpha"
                    ).value,
                    0.0,
                    1.0,
                )
            )
            velocity = (
                alpha * measured
                + (1.0 - alpha)
                * self.dynamic_obstacle_track_velocity
            )
        self.dynamic_obstacle_track_time = now
        self.dynamic_obstacle_track_ratio = ratio
        self.dynamic_obstacle_track_velocity = velocity
        deadband = float(
            self.get_parameter(
                "dynamic_obstacle_lateral_deadband_ratio"
            ).value
        )
        position = (
            "left"
            if ratio < -deadband
            else "right"
            if ratio > deadband
            else "center"
        )
        moving = abs(velocity) >= float(
            self.get_parameter(
                "dynamic_obstacle_lateral_motion_threshold_ratio_s"
            ).value
        )
        if moving:
            self.dynamic_obstacle_lane_candidate = "transitioning"
            self.dynamic_obstacle_lane_candidate_frames = 0
            return "transitioning", ratio, velocity, 0
        if position == self.dynamic_obstacle_lane_candidate:
            self.dynamic_obstacle_lane_candidate_frames += 1
        else:
            self.dynamic_obstacle_lane_candidate = position
            self.dynamic_obstacle_lane_candidate_frames = 1
        return (
            position,
            ratio,
            velocity,
            self.dynamic_obstacle_lane_candidate_frames,
        )

    def update_dynamic_longitudinal_track(
        self,
        distance_m: float,
        now: float,
        new_measurement: bool,
    ) -> tuple[float, float, float]:
        expected = float(
            self.get_parameter(
                "dynamic_obstacle_expected_speed_command"
            ).value
        )
        scale = max(
            1.0e-4,
            float(
                self.get_parameter(
                    "dynamic_obstacle_speed_to_mps_scale"
                ).value
            ),
        )
        if not new_measurement or not math.isfinite(distance_m):
            closing = self.dynamic_obstacle_closing_speed_mps
            estimated = expected
            if self.dynamic_obstacle_previous_distance_time > 0.0:
                ego_mps = max(0.0, self.last_final_speed * scale)
                estimated = max(0.0, (ego_mps - closing) / scale)
            return closing, estimated, obstacle_ttc(distance_m, closing)

        closing = 0.0
        if (
            self.dynamic_obstacle_previous_distance_time > 0.0
            and math.isfinite(
                self.dynamic_obstacle_previous_distance_m
            )
        ):
            dt = max(
                0.02,
                now - self.dynamic_obstacle_previous_distance_time,
            )
            range_change = (
                self.dynamic_obstacle_previous_distance_m - distance_m
            )
            if abs(range_change) <= float(
                self.get_parameter(
                    "dynamic_obstacle_max_range_jump_m"
                ).value
            ):
                maximum = float(
                    self.get_parameter(
                        "dynamic_obstacle_max_closing_speed_mps"
                    ).value
                )
                measured = float(
                    np.clip(range_change / dt, -maximum, maximum)
                )
                alpha = float(
                    np.clip(
                        self.get_parameter(
                            "dynamic_obstacle_relative_speed_alpha"
                        ).value,
                        0.0,
                        1.0,
                    )
                )
                closing = (
                    alpha * measured
                    + (1.0 - alpha)
                    * self.dynamic_obstacle_closing_speed_mps
                )
        self.dynamic_obstacle_previous_distance_m = distance_m
        self.dynamic_obstacle_previous_distance_time = now
        self.dynamic_obstacle_closing_speed_mps = closing
        ego_mps = max(0.0, self.last_final_speed * scale)
        estimated = max(0.0, (ego_mps - closing) / scale)
        estimated = float(
            np.clip(
                estimated,
                0.0,
                float(
                    self.get_parameter(
                        "dynamic_obstacle_pass_max_speed"
                    ).value
                ),
            )
        )
        return closing, estimated, obstacle_ttc(distance_m, closing)

    def held_dynamic_obstacle_estimate(
        self,
        now: float,
    ) -> DynamicObstacleEstimate:
        age = now - self.dynamic_obstacle_last_seen_time
        if (
            age < 0.0
            or age
            > float(
                self.get_parameter(
                    "dynamic_obstacle_detection_grace_sec"
                ).value
            )
            or not self.dynamic_obstacle_last_estimate.detected
        ):
            return DynamicObstacleEstimate()
        previous = self.dynamic_obstacle_last_estimate
        predicted = max(
            0.0,
            previous.distance_m
            - max(0.0, previous.closing_speed_mps) * age,
        )
        return replace(
            previous,
            distance_m=predicted,
            ttc_sec=obstacle_ttc(
                predicted,
                previous.closing_speed_mps,
            ),
            reason=f"{previous.reason}:held={age:.2f}s",
        )

    def dynamic_lane_path_ready(self) -> bool:
        if not bool(
            self.get_parameter(
                "dynamic_obstacle_require_lane_path"
            ).value
        ):
            return True
        return (
            self.path_valid
            and self.latest_path_info is not None
            and self.latest_path_info.forward_span_m
            >= float(
                self.get_parameter(
                    "dynamic_obstacle_min_lane_span_m"
                ).value
            )
        )

    def dynamic_overtake_candidate_ready(
        self,
        estimate: DynamicObstacleEstimate,
        now: float,
    ) -> bool:
        if (
            not estimate.detected
            or not estimate.sensor_confirmed
            or estimate.position == "transitioning"
            or estimate.lane_stable_frames
            < int(
                self.get_parameter(
                    "dynamic_obstacle_required_frames"
                ).value
            )
        ):
            return False
        if now < self.dynamic_obstacle_abort_hold_until:
            return False
        if estimate.vehicle_count >= 2 and not bool(
            self.get_parameter(
                "dynamic_obstacle_allow_multi_vehicle"
            ).value
        ):
            return False
        if estimate.vehicle_count >= 2:
            gap = estimate.second_distance_m - estimate.distance_m
            if (
                not math.isfinite(gap)
                or gap
                < float(
                    self.get_parameter(
                        "dynamic_obstacle_multi_vehicle_gap_m"
                    ).value
                )
            ):
                return False
        if (
            abs(self.lane_angle)
            > float(
                self.get_parameter(
                    "dynamic_obstacle_curve_steer_threshold_command"
                ).value
            )
            and not bool(
                self.get_parameter(
                    "dynamic_obstacle_allow_curve_start"
                ).value
            )
        ):
            return False
        if not (
            float(
                self.get_parameter(
                    "dynamic_obstacle_min_commit_distance_m"
                ).value
            )
            <= estimate.distance_m
            <= float(
                self.get_parameter(
                    "dynamic_obstacle_prepare_distance_m"
                ).value
            )
        ):
            return False
        if estimate.ttc_sec < float(
            self.get_parameter(
                "dynamic_obstacle_min_ttc_sec"
            ).value
        ):
            return False
        return self.dynamic_lane_path_ready()

    def dynamic_overtake_direction(
        self,
        estimate: DynamicObstacleEstimate,
        now: float,
    ) -> str | None:
        return choose_static_avoidance_direction(
            obstacle_position=estimate.position,
            left_clear=self.static_obstacle_side_is_clear("left", now),
            right_clear=self.static_obstacle_side_is_clear("right", now),
            left_distance_m=self.left_distance,
            right_distance_m=self.right_distance,
        )

    def dynamic_obstacle_moves_into_pass_lane(
        self,
        estimate: DynamicObstacleEstimate,
        direction: str,
    ) -> bool:
        if not estimate.detected:
            return False
        threshold = 0.5 * float(
            self.get_parameter(
                "dynamic_obstacle_lateral_motion_threshold_ratio_s"
            ).value
        )
        if direction == "left":
            return estimate.lateral_velocity_ratio_s < -threshold
        if direction == "right":
            return estimate.lateral_velocity_ratio_s > threshold
        return False

    def dynamic_obstacle_steering(
        self,
        direction: str,
        offset_fraction: float = 1.0,
    ) -> float:
        fraction = float(np.clip(offset_fraction, 0.0, 1.0))
        offset = (
            float(
                self.get_parameter(
                    "dynamic_obstacle_lane_offset_m"
                ).value
            )
            * fraction
        )
        if self.path_valid and self.latest_path is not None:
            shifted = (
                offset_path_left(self.latest_path, offset)
                if direction == "left"
                else offset_path_right(self.latest_path, offset)
            )
            steering, _ = self.steering_command_for_path(shifted)
            return float(
                np.clip(
                    steering,
                    self.angle_command_min,
                    self.angle_command_max,
                )
            )
        return self.lane_angle

    def dynamic_obstacle_pass_speed(
        self,
        estimate: DynamicObstacleEstimate,
    ) -> float:
        vehicle_speed = estimate.estimated_speed_command
        if vehicle_speed <= 0.0 or not math.isfinite(vehicle_speed):
            vehicle_speed = float(
                self.get_parameter(
                    "dynamic_obstacle_expected_speed_command"
                ).value
            )
        return float(
            np.clip(
                vehicle_speed
                + float(
                    self.get_parameter(
                        "dynamic_obstacle_pass_speed_margin_command"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "dynamic_obstacle_pass_min_speed"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "dynamic_obstacle_pass_max_speed"
                    ).value
                ),
            )
        )

    def dynamic_follow_command(
        self,
        estimate: DynamicObstacleEstimate,
    ) -> tuple[float, float, str, str]:
        vehicle_speed = estimate.estimated_speed_command
        if vehicle_speed <= 0.0 or not math.isfinite(vehicle_speed):
            vehicle_speed = float(
                self.get_parameter(
                    "dynamic_obstacle_expected_speed_command"
                ).value
            )
        maximum = max(
            float(
                self.get_parameter(
                    "dynamic_obstacle_min_follow_speed"
                ).value
            ),
            self.lane_speed,
        )
        speed, target_gap = dynamic_follow_speed(
            obstacle_distance_m=estimate.distance_m,
            closing_speed_mps=estimate.closing_speed_mps,
            estimated_vehicle_speed_command=vehicle_speed,
            current_speed_command=max(
                self.last_final_speed,
                self.lane_speed,
            ),
            speed_to_mps_scale=float(
                self.get_parameter(
                    "dynamic_obstacle_speed_to_mps_scale"
                ).value
            ),
            standstill_gap_m=float(
                self.get_parameter(
                    "dynamic_obstacle_standstill_gap_m"
                ).value
            ),
            target_time_gap_sec=float(
                self.get_parameter(
                    "dynamic_obstacle_target_time_gap_sec"
                ).value
            ),
            follow_kp=float(
                self.get_parameter(
                    "dynamic_obstacle_follow_kp"
                ).value
            ),
            closing_gain=float(
                self.get_parameter(
                    "dynamic_obstacle_follow_closing_gain"
                ).value
            ),
            minimum_speed_command=float(
                self.get_parameter(
                    "dynamic_obstacle_min_follow_speed"
                ).value
            ),
            maximum_speed_command=maximum,
        )
        if estimate.ttc_sec <= float(
            self.get_parameter(
                "dynamic_obstacle_hard_brake_ttc_sec"
            ).value
        ):
            speed = min(
                speed,
                float(
                    self.get_parameter(
                        "dynamic_obstacle_min_follow_speed"
                    ).value
                ),
            )
        return (
            self.lane_angle,
            speed,
            "DYNAMIC_FOLLOW",
            f"gap={estimate.distance_m:.2f}/{target_gap:.2f}m:"
            f"v={estimate.estimated_speed_command:.2f}:"
            f"ttc={estimate.ttc_sec:.2f}s",
        )

    def dynamic_obstacle_command(
        self,
        now: float,
    ) -> tuple[float, float, str, str] | None:
        if not bool(
            self.get_parameter(
                "enable_dynamic_obstacle_handling"
            ).value
        ):
            if self.dynamic_obstacle_state != "CLEAR":
                self.reset_dynamic_obstacle_state()
            return None

        estimate = self.dynamic_obstacle_estimate
        if (
            not estimate.detected
            and self.dynamic_obstacle_state
            in {"TRACK_CONFIRM", "FOLLOW_GAP", "PREPARE_PASS"}
        ):
            estimate = self.held_dynamic_obstacle_estimate(now)

        if self.dynamic_obstacle_state == "COOLDOWN":
            if now < self.dynamic_obstacle_cooldown_until:
                return None
            self.reset_dynamic_obstacle_state(clear_tracking=False)

        if self.cone_mode_active:
            if (
                estimate.detected
                and (
                    estimate.distance_m
                    < float(
                        self.get_parameter(
                            "dynamic_obstacle_min_commit_distance_m"
                        ).value
                    )
                    or estimate.ttc_sec
                    <= float(
                        self.get_parameter(
                            "dynamic_obstacle_emergency_ttc_sec"
                        ).value
                    )
                )
            ):
                return (
                    self.last_final_angle,
                    0.0,
                    "DYNAMIC_OBSTACLE_STOP",
                    "vehicle_danger_during_cone_mode",
                )
            return None

        if not estimate.detected:
            if self.dynamic_obstacle_state in {
                "CHANGE_OUT",
                "PASS",
                "REJOIN",
            }:
                return self.dynamic_overtake_command(now, estimate)
            self.reset_dynamic_obstacle_state()
            return None

        if (
            estimate.distance_m
            < float(
                self.get_parameter(
                    "dynamic_obstacle_min_commit_distance_m"
                ).value
            )
            or estimate.ttc_sec
            <= float(
                self.get_parameter(
                    "dynamic_obstacle_emergency_ttc_sec"
                ).value
            )
        ) and self.dynamic_obstacle_state not in {
            "PASS",
            "REJOIN",
        }:
            self.dynamic_obstacle_state = "FOLLOW_GAP"
            return (
                self.lane_angle,
                0.0,
                "DYNAMIC_OBSTACLE_STOP",
                estimate.reason,
            )

        if self.dynamic_obstacle_state == "CLEAR":
            self.dynamic_obstacle_state = "TRACK_CONFIRM"

        if self.dynamic_obstacle_state == "TRACK_CONFIRM":
            if estimate.lane_stable_frames >= int(
                self.get_parameter(
                    "dynamic_obstacle_required_frames"
                ).value
            ):
                self.dynamic_obstacle_state = "FOLLOW_GAP"
            return self.dynamic_follow_command(estimate)

        if self.dynamic_obstacle_state == "FOLLOW_GAP":
            if self.dynamic_overtake_candidate_ready(estimate, now):
                self.dynamic_obstacle_state = "PREPARE_PASS"
                self.dynamic_obstacle_prepare_clear_since = 0.0
            return self.dynamic_follow_command(estimate)

        if self.dynamic_obstacle_state == "PREPARE_PASS":
            if not self.dynamic_overtake_candidate_ready(estimate, now):
                self.dynamic_obstacle_state = "FOLLOW_GAP"
                self.dynamic_obstacle_prepare_clear_since = 0.0
                return self.dynamic_follow_command(estimate)
            direction = self.dynamic_overtake_direction(estimate, now)
            if direction is None:
                self.dynamic_obstacle_prepare_clear_since = 0.0
                return self.dynamic_follow_command(estimate)
            if self.dynamic_obstacle_prepare_clear_since <= 0.0:
                self.dynamic_obstacle_prepare_clear_since = now
            if (
                now - self.dynamic_obstacle_prepare_clear_since
                < float(
                    self.get_parameter(
                        "dynamic_obstacle_prepare_clear_hold_sec"
                    ).value
                )
            ):
                return self.dynamic_follow_command(estimate)
            self.dynamic_obstacle_direction = direction
            self.dynamic_obstacle_state = "CHANGE_OUT"
            self.dynamic_obstacle_started_at = now
            self.dynamic_obstacle_phase_started_at = now
            self.dynamic_obstacle_pass_detected = False
            self.dynamic_obstacle_clear_since = 0.0

        return self.dynamic_overtake_command(now, estimate)

    def dynamic_overtake_command(
        self,
        now: float,
        estimate: DynamicObstacleEstimate,
    ) -> tuple[float, float, str, str]:
        direction = self.dynamic_obstacle_direction
        obstacle_side = "right" if direction == "left" else "left"
        if (
            self.dynamic_obstacle_state == "CHANGE_OUT"
            and self.dynamic_obstacle_moves_into_pass_lane(
                estimate,
                direction,
            )
        ):
            self.dynamic_obstacle_state = "FOLLOW_GAP"
            self.dynamic_obstacle_abort_hold_until = (
                now
                + float(
                    self.get_parameter(
                        "dynamic_obstacle_abort_hold_sec"
                    ).value
                )
            )
            return self.dynamic_follow_command(estimate)

        if (
            self.dynamic_obstacle_state == "PASS"
            and self.dynamic_obstacle_moves_into_pass_lane(
                estimate,
                direction,
            )
        ):
            self.dynamic_obstacle_cut_in_hold_until = max(
                self.dynamic_obstacle_cut_in_hold_until,
                now
                + float(
                    self.get_parameter(
                        "dynamic_obstacle_cut_in_hold_sec"
                    ).value
                ),
            )
        if (
            self.dynamic_obstacle_state == "PASS"
            and now < self.dynamic_obstacle_cut_in_hold_until
        ):
            values = self.static_obstacle_ultra_values_for_side(
                obstacle_side
            )
            side_danger = (
                self.static_obstacle_ultra_is_fresh(now)
                and bool(values)
                and min(values)
                < float(
                    self.get_parameter(
                        "dynamic_obstacle_side_emergency_cm"
                    ).value
                )
            )
            longitudinal_danger = (
                estimate.detected
                and (
                    estimate.distance_m
                    < float(
                        self.get_parameter(
                            "dynamic_obstacle_min_commit_distance_m"
                        ).value
                    )
                    or estimate.ttc_sec
                    <= float(
                        self.get_parameter(
                            "dynamic_obstacle_hard_brake_ttc_sec"
                        ).value
                    )
                )
            )
            return (
                self.dynamic_obstacle_steering(direction),
                (
                    0.0
                    if side_danger or longitudinal_danger
                    else float(
                        self.get_parameter(
                            "dynamic_obstacle_timeout_hold_speed"
                        ).value
                    )
                ),
                "DYNAMIC_OVERTAKE_YIELD",
                f"cut_in:side={int(side_danger)}:"
                f"front={int(longitudinal_danger)}",
            )

        if self.dynamic_obstacle_state == "CHANGE_OUT":
            phase_elapsed = now - self.dynamic_obstacle_phase_started_at
            side_seen = self.static_obstacle_side_detected(obstacle_side)
            if (
                side_seen
                or phase_elapsed
                >= float(
                    self.get_parameter(
                        "dynamic_obstacle_change_min_sec"
                    ).value
                )
            ):
                self.dynamic_obstacle_pass_detected = side_seen
                self.dynamic_obstacle_state = "PASS"
                self.dynamic_obstacle_phase_started_at = now
        elif self.dynamic_obstacle_state == "PASS":
            if self.static_obstacle_side_detected(obstacle_side):
                self.dynamic_obstacle_pass_detected = True
                self.dynamic_obstacle_clear_since = 0.0
            elif (
                self.dynamic_obstacle_pass_detected
                and self.static_obstacle_side_cleared(
                    obstacle_side,
                    now,
                )
            ):
                if self.dynamic_obstacle_clear_since <= 0.0:
                    self.dynamic_obstacle_clear_since = now
                elif (
                    now - self.dynamic_obstacle_clear_since
                    >= float(
                        self.get_parameter(
                            "dynamic_obstacle_pass_clear_hold_sec"
                        ).value
                    )
                ):
                    self.dynamic_obstacle_state = "REJOIN"
                    self.dynamic_obstacle_rejoin_started_at = now

        elapsed_total = now - self.dynamic_obstacle_started_at
        fallback = (
            self.dynamic_obstacle_state in {"CHANGE_OUT", "PASS"}
            and not estimate.detected
            and elapsed_total
            >= float(
                self.get_parameter(
                    "dynamic_obstacle_no_side_fallback_sec"
                ).value
            )
            and self.dynamic_obstacle_clear_frames
            >= int(
                self.get_parameter(
                    "static_obstacle_clear_required_frames"
                ).value
            )
            and math.isfinite(self.front_distance)
            and self.front_distance
            > float(
                self.get_parameter(
                    "dynamic_obstacle_front_clear_distance_m"
                ).value
            )
            and self.static_obstacle_side_cleared(
                obstacle_side,
                now,
            )
        )
        if fallback:
            self.dynamic_obstacle_state = "REJOIN"
            self.dynamic_obstacle_rejoin_started_at = now

        if self.dynamic_obstacle_state == "REJOIN":
            elapsed = now - self.dynamic_obstacle_rejoin_started_at
            minimum = max(
                0.05,
                float(
                    self.get_parameter(
                        "dynamic_obstacle_rejoin_min_sec"
                    ).value
                ),
            )
            progress = float(np.clip(elapsed / minimum, 0.0, 1.0))
            offset_fraction = 1.0 - progress * progress * (
                3.0 - 2.0 * progress
            )
            steering = self.dynamic_obstacle_steering(
                direction,
                offset_fraction,
            )
            complete = elapsed >= minimum and self.path_valid
            timed_out = elapsed >= float(
                self.get_parameter(
                    "dynamic_obstacle_rejoin_max_sec"
                ).value
            )
            if complete or timed_out:
                self.reset_dynamic_obstacle_state(
                    clear_tracking=False
                )
                self.dynamic_obstacle_state = "COOLDOWN"
                self.dynamic_obstacle_cooldown_until = (
                    now
                    + float(
                        self.get_parameter(
                            "dynamic_obstacle_cooldown_sec"
                        ).value
                    )
                )
            return (
                steering,
                float(
                    self.get_parameter(
                        "dynamic_obstacle_rejoin_speed"
                    ).value
                ),
                "DYNAMIC_OVERTAKE_REJOIN",
                f"progress={progress:.2f}:"
                f"path={int(self.path_valid)}",
            )

        steering = self.dynamic_obstacle_steering(direction)
        if self.dynamic_obstacle_state == "CHANGE_OUT":
            bias_duration = max(
                0.01,
                float(
                    self.get_parameter(
                        "dynamic_obstacle_bias_sec"
                    ).value
                ),
            )
            ratio = float(
                np.clip(
                    1.0 - elapsed_total / bias_duration,
                    0.0,
                    1.0,
                )
            )
            sign = -1.0 if direction == "left" else 1.0
            steering = float(
                np.clip(
                    steering
                    + sign
                    * float(
                        self.get_parameter(
                            "dynamic_obstacle_bias_command"
                        ).value
                    )
                    * ratio,
                    self.angle_command_min,
                    self.angle_command_max,
                )
            )
            speed = float(
                self.get_parameter(
                    "dynamic_obstacle_change_speed"
                ).value
            )
        else:
            speed = self.dynamic_obstacle_pass_speed(estimate)
        timed_out = elapsed_total > float(
            self.get_parameter(
                "dynamic_obstacle_timeout_sec"
            ).value
        )
        if timed_out:
            speed = min(
                speed,
                float(
                    self.get_parameter(
                        "dynamic_obstacle_timeout_hold_speed"
                    ).value
                ),
            )
        return (
            steering,
            speed,
            "DYNAMIC_OVERTAKE",
            f"phase={self.dynamic_obstacle_state}:"
            f"lane={direction}:d={estimate.distance_m:.2f}m:"
            f"ttc={estimate.ttc_sec:.2f}s:"
            f"timeout={int(timed_out)}",
        )

    def on_fast_motor_timer(self) -> None:
        """Keep the validated 100 Hz motor stream only while cones own control."""
        now = time.monotonic()
        self.update_cone_mode(now)
        traffic_stop = self.traffic_control_enabled and not self.traffic_go
        obstacle_stop = (
            bool(
                self.get_parameter(
                    "enable_static_obstacle_handling"
                ).value
            )
            and (
                self.static_obstacle_confirmed
                or self.static_obstacle_state
                not in {"CLEAR", "COOLDOWN"}
                or (
                    self.static_obstacle_estimate.detected
                    and self.static_obstacle_estimate.in_path
                    and self.static_obstacle_estimate.sensor_confirmed
                    and self.static_obstacle_estimate.distance_m
                    <= float(
                        self.get_parameter(
                            "static_obstacle_stop_distance_m"
                        ).value
                    )
                )
            )
        )
        dynamic_override = (
            bool(
                self.get_parameter(
                    "enable_dynamic_obstacle_handling"
                ).value
            )
            and (
                self.dynamic_obstacle_estimate.detected
                or self.dynamic_obstacle_state
                not in {"CLEAR", "COOLDOWN"}
            )
        )
        course_signal_override = (
            self.course_signal_control_enabled
            and self.course_signal_armed
            and self.course_signal_state
            in {
                "RED_APPROACH",
                "RED_HOLD",
                "YELLOW_APPROACH",
                "YELLOW_HOLD",
            }
        )
        if (
            not self.cone_mode_active
            and self.handoff_started_at <= 0.0
            and not traffic_stop
            and not obstacle_stop
            and not dynamic_override
            and not course_signal_override
        ):
            return
        angle, speed, state, reason = self.select_final_command(now)
        self.publish_final(angle, speed, state, reason, now)

    def update_cone_mode(self, now: float) -> None:
        if self.force_cone_mode:
            if not self.cone_mode_active:
                self.enter_cone_mode(now, "forced")
            return
        if not self.cone_mode_active:
            required = max(
                1,
                int(
                    self.get_parameter(
                        "cone_mode_entry_required_frames"
                    ).value
                ),
            )
            gate_ready, reason = self.cone_entry_gate(now, required)
            self.last_cone_gate_reason = reason
            if gate_ready:
                self.enter_cone_mode(now, reason)
            return

        minimum_duration = float(
            self.get_parameter("cone_mode_min_duration_sec").value
        )
        if now - self.cone_mode_started_at < minimum_duration:
            return
        path_recent = (
            self.last_valid_cone_time > 0.0
            and now - self.last_valid_cone_time
            <= float(self.get_parameter("cone_mode_exit_timeout_sec").value)
        )
        clusters_recent = (
            self.cone_cluster_count
            >= int(self.get_parameter("cone_mode_presence_min_clusters").value)
            and self.cone_cluster_time > 0.0
            and now - self.cone_cluster_time
            <= float(self.get_parameter("cone_cluster_timeout_sec").value)
        )
        if path_recent or clusters_recent:
            self.reset_lane_recovery()
            return

        canonical_time = self.last_canonical_command_time
        lane_ready = (
            canonical_time is not None
            and self.path_valid
            and self.lane_visible
            and self.lane_speed > 0.0
        )
        if not lane_ready:
            self.reset_lane_recovery()
            return
        if canonical_time != self.last_counted_canonical_time:
            self.last_counted_canonical_time = canonical_time
            self.lane_recovery_frames += 1
            if self.lane_recovery_started_at <= 0.0:
                self.lane_recovery_started_at = now
        required_frames = int(
            self.get_parameter("cone_mode_lane_recovery_required_frames").value
        )
        required_hold = float(
            self.get_parameter("cone_mode_lane_recovery_hold_sec").value
        )
        if (
            self.lane_recovery_frames >= required_frames
            and now - self.lane_recovery_started_at >= required_hold
        ):
            self.begin_lane_handoff(now)

    def cone_entry_gate(
        self, now: float, required_path_frames: int
    ) -> tuple[bool, str]:
        path_fresh = (
            self.cone_command_time > 0.0
            and now - self.cone_command_time
            <= float(
                self.get_parameter("external_cone_cmd_timeout_sec").value
            )
        )
        if (
            not path_fresh
            or self.cone_entry_frames < required_path_frames
        ):
            return False, "waiting_lidar_path"
        cluster_fresh = (
            self.cone_cluster_time > 0.0
            and now - self.cone_cluster_time
            <= float(
                self.get_parameter("cone_cluster_timeout_sec").value
            )
        )
        minimum_clusters = max(
            1,
            int(
                self.get_parameter(
                    "cone_mode_presence_min_clusters"
                ).value
            ),
        )
        if not cluster_fresh or self.cone_cluster_count < minimum_clusters:
            return False, "waiting_lidar_clusters"
        if not bool(
            self.get_parameter("require_yolo_cone_for_entry").value
        ):
            return True, "lidar_only_override"
        yolo_fresh = (
            self.yolo_cone_time > 0.0
            and now - self.yolo_cone_time
            <= float(self.get_parameter("yolo_cone_timeout_sec").value)
        )
        required_yolo_frames = max(
            1,
            int(
                self.get_parameter("yolo_cone_required_frames").value
            ),
        )
        if not yolo_fresh or self.yolo_cone_frames < required_yolo_frames:
            return False, "waiting_yolo_cones"
        matched = cone_modalities_match(
            yolo_total=self.yolo_cone_count,
            yolo_left=self.yolo_cone_left_count,
            yolo_right=self.yolo_cone_right_count,
            lidar_total=self.cone_cluster_count,
            lidar_left=self.cone_cluster_left_count,
            lidar_right=self.cone_cluster_right_count,
            minimum_yolo_count=max(
                1,
                int(self.get_parameter("yolo_cone_min_count").value),
            ),
            minimum_lidar_count=minimum_clusters,
        )
        if not matched:
            return False, "waiting_sensor_side_match"
        return True, "yolo+lidar_confirmed"

    def enter_cone_mode(self, now: float, reason: str) -> None:
        self.cone_mode_active = True
        self.cone_mode_started_at = now
        self.handoff_started_at = 0.0
        self.reset_lane_recovery()
        self.get_logger().info(f"cone mode enabled: {reason}")

    def reset_lane_recovery(self) -> None:
        self.lane_recovery_frames = 0
        self.lane_recovery_started_at = 0.0
        self.last_counted_canonical_time = self.last_canonical_command_time

    def begin_lane_handoff(self, now: float) -> None:
        self.cone_mode_active = False
        self.cone_entry_frames = 0
        self.yolo_cone_frames = 0
        self.last_cone_gate_reason = "handoff"
        self.handoff_started_at = now
        self.handoff_from_angle = self.cone_target_to_command(
            self.last_valid_cone_angle
        )
        self.handoff_from_speed = (
            self.last_valid_cone_speed
            if self.last_valid_cone_speed > 0.0
            else float(self.get_parameter("cone_exit_creep_speed").value)
        )
        self.reset_lane_recovery()
        self.get_logger().info("lane recovered: blending from cone to lane")

    def apply_course_signal_overlay(
        self,
        angle: float,
        speed: float,
        state: str,
        reason: str,
        now: float,
    ) -> tuple[float, float, str, str]:
        """Keep path steering while a later red/yellow signal controls speed."""
        if (
            not self.course_signal_control_enabled
            or not self.course_signal_armed
            or not self.traffic_go
        ):
            return angle, speed, state, reason
        signal_state = self.course_signal_state
        if signal_state in {"RED_HOLD", "YELLOW_HOLD"}:
            return (
                angle,
                0.0,
                "COURSE_" + signal_state,
                f"bbox_area={self.course_signal_area_ratio:.5f}",
            )
        if signal_state not in {"RED_APPROACH", "YELLOW_APPROACH"}:
            return angle, speed, state, reason

        timeout = max(
            0.0,
            float(
                self.get_parameter(
                    "course_signal_observation_timeout_sec"
                ).value
            ),
        )
        age = (
            now - self.course_signal_last_seen_time
            if self.course_signal_last_seen_time > 0.0
            else float("inf")
        )
        if timeout > 0.0 and age > timeout:
            lost_action = str(
                self.get_parameter("course_signal_lost_action").value
            ).strip().lower()
            if lost_action == "hold":
                return (
                    angle,
                    0.0,
                    "COURSE_SIGNAL_LOST_HOLD",
                    f"last={signal_state} age={age:.2f}s",
                )
            return angle, speed, state, reason

        target_speed, progress, should_stop = signal_approach_speed(
            speed,
            self.course_signal_area_ratio,
            float(
                self.get_parameter(
                    "course_signal_slowdown_area_ratio"
                ).value
            ),
            float(
                self.get_parameter(
                    "course_signal_stop_area_ratio"
                ).value
            ),
        )
        if should_stop:
            hold_state = (
                "COURSE_RED_HOLD"
                if signal_state == "RED_APPROACH"
                else "COURSE_YELLOW_HOLD"
            )
            self.course_signal_state = hold_state.replace("COURSE_", "")
            return (
                angle,
                0.0,
                hold_state,
                f"bbox_area={self.course_signal_area_ratio:.5f}",
            )
        if progress <= 0.0:
            return angle, speed, state, reason
        approach_state = (
            "COURSE_RED_APPROACH"
            if signal_state == "RED_APPROACH"
            else "COURSE_YELLOW_APPROACH"
        )
        return (
            angle,
            min(max(0.0, speed), target_speed),
            approach_state,
            f"bbox_area={self.course_signal_area_ratio:.5f} "
            f"brake={progress:.2f}",
        )

    def select_final_command(
        self, now: float
    ) -> tuple[float, float, str, str]:
        if self.traffic_control_enabled and not self.traffic_go:
            state = (
                "TRAFFIC_STOP"
                if self.traffic_red_frames > 0
                else "WAIT_GREEN"
            )
            reason = (
                "startup_yolo_box_hsv_gate"
                if self.traffic_startup_only
                else "yolo_signal_gate"
            )
            return 0.0, 0.0, state, reason

        static_command = self.static_obstacle_command(now)
        if static_command is not None:
            return static_command
        dynamic_command = self.dynamic_obstacle_command(now)
        if dynamic_command is not None:
            return dynamic_command

        emergency = float(
            self.get_parameter("cone_emergency_stop_distance_m").value
        )
        if self.cone_mode_active and self.front_distance < emergency:
            self.cone_emergency_latched = True
            self.cone_emergency_clear_started_at = 0.0
        if self.cone_mode_active and self.cone_emergency_latched:
            clear_distance = max(
                emergency,
                float(
                    self.get_parameter(
                        "cone_emergency_clear_distance_m"
                    ).value
                ),
            )
            cone_fresh = (
                self.cone_command_time > 0.0
                and now - self.cone_command_time
                <= float(
                    self.get_parameter(
                        "external_cone_cmd_timeout_sec"
                    ).value
                )
                and self.cone_confidence > 0.2
                and self.cone_speed > 0.0
            )
            if self.front_distance >= clear_distance and cone_fresh:
                if self.cone_emergency_clear_started_at <= 0.0:
                    self.cone_emergency_clear_started_at = now
                clear_hold = max(
                    0.0,
                    float(
                        self.get_parameter(
                            "cone_emergency_clear_hold_sec"
                        ).value
                    ),
                )
                if now - self.cone_emergency_clear_started_at >= clear_hold:
                    self.cone_emergency_latched = False
                    self.cone_emergency_clear_started_at = 0.0
            else:
                self.cone_emergency_clear_started_at = 0.0
        if self.cone_mode_active and self.cone_emergency_latched:
            return 0.0, 0.0, "EMERGENCY_STOP", (
                f"front={self.front_distance:.2f}m"
            )

        if self.cone_mode_active:
            fresh_timeout = float(
                self.get_parameter("external_cone_cmd_timeout_sec").value
            )
            command_age = (
                now - self.cone_command_time
                if self.cone_command_time > 0.0
                else float("inf")
            )
            if command_age <= fresh_timeout:
                if (
                    bool(
                        self.get_parameter(
                            "cone_fresh_stop_is_authoritative"
                        ).value
                    )
                    and (
                        self.cone_confidence <= 0.2
                        or self.cone_speed <= 0.0
                    )
                ):
                    return 0.0, 0.0, "CONE_STOP", "fresh_cone_stop"
                if self.cone_confidence > 0.2 and self.cone_speed > 0.0:
                    return self.apply_course_signal_overlay(
                        self.cone_target_to_command(self.cone_angle),
                        self.cone_speed,
                        "CONE_SLALOM",
                        f"confidence={self.cone_confidence:.2f}",
                        now,
                    )
            if not bool(
                self.get_parameter("cone_manager_recovery_enabled").value
            ):
                return 0.0, 0.0, "CONE_STOP", "waiting_fresh_cone_cmd"

            age = (
                now - self.last_valid_cone_time
                if self.last_valid_cone_time > 0.0
                else float("inf")
            )
            if age <= fresh_timeout and self.cone_confidence > 0.2:
                return self.apply_course_signal_overlay(
                    self.cone_target_to_command(self.cone_angle),
                    self.cone_speed,
                    "CONE_SLALOM",
                    f"confidence={self.cone_confidence:.2f}",
                    now,
                )
            hold_timeout = float(
                self.get_parameter("cone_mode_exit_timeout_sec").value
            )
            if age <= hold_timeout:
                return self.apply_course_signal_overlay(
                    self.cone_target_to_command(self.last_valid_cone_angle),
                    self.last_valid_cone_speed,
                    "CONE_RECOVERY",
                    "hold_last_cone",
                    now,
                )
            creep_timeout = max(
                hold_timeout,
                float(
                    self.get_parameter(
                        "cone_mode_exit_creep_timeout_sec"
                    ).value
                ),
            )
            if age <= creep_timeout:
                progress = (age - hold_timeout) / max(
                    1.0e-6, creep_timeout - hold_timeout
                )
                retention = 1.0 - progress * (
                    1.0
                    - float(
                        self.get_parameter(
                            "cone_exit_creep_steer_retention"
                        ).value
                    )
                )
                return self.apply_course_signal_overlay(
                    self.cone_target_to_command(
                        self.last_valid_cone_angle * retention
                    ),
                    max(
                        float(
                            self.get_parameter(
                                "minimum_drive_speed"
                            ).value
                        ),
                        float(
                            self.get_parameter(
                                "cone_exit_creep_speed"
                            ).value
                        ),
                    ),
                    "CONE_RECOVERY",
                    "cone_exit_creep",
                    now,
                )
            return 0.0, 0.0, "CONE_RECOVERY", "waiting_cone_path"

        self.cone_emergency_latched = False
        self.cone_emergency_clear_started_at = 0.0

        if self.handoff_started_at > 0.0:
            duration = max(
                0.05,
                float(
                    self.get_parameter(
                        "cone_lane_handoff_duration_sec"
                    ).value
                ),
            )
            progress = float(
                np.clip((now - self.handoff_started_at) / duration, 0.0, 1.0)
            )
            blend = progress * progress * (3.0 - 2.0 * progress)
            lane_speed = min(
                self.lane_speed,
                float(
                    self.get_parameter("cone_lane_handoff_max_speed").value
                ),
            )
            lane_speed = max(
                float(self.get_parameter("minimum_drive_speed").value),
                lane_speed,
            )
            angle = (1.0 - blend) * self.handoff_from_angle + blend * self.lane_angle
            speed = (1.0 - blend) * self.handoff_from_speed + blend * lane_speed
            if progress >= 1.0:
                self.handoff_started_at = 0.0
            return self.apply_course_signal_overlay(
                angle,
                speed,
                "CONE_HANDOFF",
                f"blend={progress:.2f}",
                now,
            )

        return self.apply_course_signal_overlay(
            self.lane_angle,
            self.lane_speed,
            "LANE_FOLLOW",
            self.latest_path_source,
            now,
        )

    def cone_target_to_command(self, angle_deg: float) -> float:
        limit = float(self.get_parameter("cone_max_target_angle_deg").value)
        target = float(np.clip(angle_deg, -limit, limit))
        return interpolate_command(
            target,
            self.get_parameter("cone_steering_actual_deg").value,
            self.get_parameter("cone_steering_command").value,
        )

    def publish_final(
        self,
        angle: float,
        speed: float,
        state: str,
        reason: str,
        now: float,
    ) -> None:
        self.last_final_angle = float(angle)
        self.last_final_speed = float(speed)
        message = Float32MultiArray()
        message.data = [self.last_final_angle, self.last_final_speed]
        if self.motor_pub is not None:
            self.motor_pub.publish(message)

        if self.latest_header is not None:
            trace = TwistStamped()
            trace.header = self.latest_header
            trace.twist.angular.z = self.last_final_angle
            trace.twist.linear.x = self.last_final_speed
            self.action_trace_pub.publish(trace)

        state_text = (
            f"{state} / reason={reason} / angle={self.last_final_angle:.1f} "
            f"/ speed={self.last_final_speed:.1f} "
            f"/ front={self.front_distance:.2f} / cone={int(self.cone_mode_active)} "
            f"/ cone_gate={self.last_cone_gate_reason}:"
            f"y{self.yolo_cone_count}:l{self.cone_cluster_count} "
            f"/ green={int(self.traffic_go)} "
            f"/ signal={self.course_signal_state}:"
            f"{self.course_signal_color}:"
            f"{self.course_signal_area_ratio:.5f}:"
            f"{int(self.course_signal_armed)} "
            f"/ static={self.static_obstacle_state}:"
            f"{self.static_obstacle_estimate.position}:"
            f"{self.static_obstacle_estimate.distance_m:.2f}:"
            f"{self.static_obstacle_estimate.target_speed_mps:.2f} "
            f"/ dynamic={self.dynamic_obstacle_state}:"
            f"{self.dynamic_obstacle_estimate.position}:"
            f"{self.dynamic_obstacle_estimate.distance_m:.2f}:"
            f"{self.dynamic_obstacle_estimate.ttc_sec:.2f} "
            f"/ side={self.left_distance:.2f},{self.right_distance:.2f}"
        )
        state_message = String()
        state_message.data = state_text
        self.state_pub.publish(state_message)
        if state != self.last_state or now - self.last_state_log_time >= 1.0:
            self.get_logger().info(state_text)
            self.last_state = state
            self.last_state_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveManagerNode()
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
