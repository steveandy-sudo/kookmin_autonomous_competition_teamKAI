#!/usr/bin/env python3
import math
import os
import time
from dataclasses import dataclass, replace
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String

try:
    import cv2
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - handled at runtime on the car
    cv2 = None
    CvBridge = None

try:
    import yaml
except Exception:  # pragma: no cover - optional unless projection is enabled
    yaml = None

try:
    from yolo_msgs.msg import DetectionArray
except Exception:  # pragma: no cover - YOLO is optional for this node
    DetectionArray = None

try:
    from custom_interfaces.msg import Detections as LegacyDetections
except Exception:  # pragma: no cover - 2025 reference stack is optional
    LegacyDetections = None

try:
    from xycar_msgs.msg import XycarMotor
except Exception:  # pragma: no cover - Float32MultiArray wrapper is the default
    XycarMotor = None

from my_rule.yolo_contract import resolve_detection_class_name


STEERING_ACTUAL_TO_COMMAND = (
    (0.0, 0.0),
    (4.0, 10.0),
    (10.0, 20.0),
    (16.0, 30.0),
    (26.0, 42.0),
)


@dataclass
class LaneEstimate:
    valid: bool = False
    target_x: float = 0.0
    image_center_x: float = 0.0
    confidence: float = 0.0
    source: str = "none"


@dataclass
class ConeEstimate:
    active: bool = False
    steer_hint: float = 0.0
    confidence: float = 0.0
    source: str = "none"


@dataclass
class ObjectDetection:
    class_name: str
    score: float
    cx: float
    cy: float
    width: float
    height: float

    @property
    def area(self) -> float:
        return max(0.0, self.width) * max(0.0, self.height)


@dataclass
class ObstacleEstimate:
    detected: bool = False
    distance_m: float = float("inf")
    second_distance_m: float = float("inf")
    position: str = "unknown"
    vehicle_count: int = 0
    source: str = "none"
    lane_state: str = "unknown"
    lateral_ratio: float = 0.0
    lateral_velocity_ratio_s: float = 0.0
    lane_stable_frames: int = 0
    sensor_confirmed: bool = False
    closing_speed_mps: float = 0.0
    estimated_speed_cmd: float = 0.0
    ttc_sec: float = float("inf")


def obstacle_ttc(distance_m: float, closing_speed_mps: float, minimum_closing_speed_mps: float = 0.05) -> float:
    """Return time-to-collision for a closing target, otherwise infinity."""
    if not math.isfinite(distance_m) or not math.isfinite(closing_speed_mps):
        return float("inf")
    if closing_speed_mps < max(1e-3, float(minimum_closing_speed_mps)):
        return float("inf")
    return max(0.0, float(distance_m)) / float(closing_speed_mps)


@dataclass
class FixedObstacleEstimate:
    detected: bool = False
    in_path: bool = False
    close: bool = False
    sensor_confirmed: bool = False
    direction: str = "unknown"
    position: str = "unknown"
    distance_m: float = float("inf")
    position_ratio: float = 0.0
    distance_source: str = "none"
    reason: str = "none"


# Keep the old import name usable for bags and helper scripts made before the
# 2026-07-15 rule revision. Runtime behavior now treats the object as fixed.
PedestrianEstimate = FixedObstacleEstimate


class RuleDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("my_rule_driver")

        self._declare_parameters()
        self._read_parameters()

        self.bridge = CvBridge() if CvBridge is not None else None
        self.camera_matrix: Optional[np.ndarray] = None
        self.lidar_camera_rotation: Optional[np.ndarray] = None
        self.lidar_camera_translation: Optional[np.ndarray] = None
        self.load_lidar_camera_extrinsic()

        self.motor_pub = self.create_motor_publisher()
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.lane_override_pub = self.create_publisher(String, self.lane_override_topic, 10)

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, self.image_topic, self.image_callback, camera_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, sensor_qos)
        if bool(self.use_lidar_camera_projection):
            self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, sensor_qos)
        self.create_subscription(Int32MultiArray, self.ultra_topic, self.ultra_callback, 10)
        self.create_subscription(Float32MultiArray, self.lane_cmd_topic, self.lane_cmd_callback, 10)
        self.create_subscription(Float32MultiArray, self.cone_cmd_topic, self.cone_cmd_callback, 10)
        self.create_subscription(PoseArray, self.cone_cluster_topic, self.cone_cluster_callback, 10)
        self.create_subscription(String, self.traffic_light_topic, self.traffic_light_callback, 10)
        self.create_subscription(Path, self.center_curve_topic, self.center_curve_callback, 10)

        if self.enable_yolo:
            self.setup_detection_subscriptions()

        self.latest_lane = LaneEstimate()
        self.latest_cone_image = ConeEstimate()
        self.latest_traffic_color = "unknown"
        self.latest_detections: List[ObjectDetection] = []
        self.latest_scan_points = np.empty((0, 3), dtype=np.float32)
        self.latest_scan_clusters = np.empty((0, 3), dtype=np.float32)
        self.latest_front_distance = float("inf")
        self.latest_left_distance = float("inf")
        self.latest_right_distance = float("inf")
        self.latest_ultra: List[int] = []
        self.last_ultra_time = 0.0
        self.latest_obstacle = ObstacleEstimate()
        self.latest_fixed_obstacle = FixedObstacleEstimate()
        self.latest_pedestrian = self.latest_fixed_obstacle
        self.center_curve_points: Optional[np.ndarray] = None
        self.center_curve_time = 0.0

        self.image_width = 0
        self.image_height = 0
        self.last_image_time = 0.0
        self.last_scan_time = 0.0
        self.last_detection_time = 0.0
        self.detection_generation = 0
        self.external_lane_angle = 0.0
        self.external_lane_speed = 0.0
        self.external_lane_confidence = 0.0
        self.external_lane_time = 0.0
        self.external_lane_generation = 0
        self.external_lane_observed = (False, False, False)
        self.external_cone_angle = 0.0
        self.external_cone_speed = 0.0
        self.external_cone_confidence = 0.0
        self.external_cone_time = 0.0
        self.last_valid_cone_cmd_time = 0.0
        self.last_valid_cone_angle = 0.0
        self.last_valid_cone_speed = 0.0
        self.external_cone_cluster_count = 0
        self.external_cone_cluster_time = 0.0
        self.cone_path_entry_frames = 0
        self.external_traffic_color = "unknown"
        self.external_traffic_time = 0.0

        self.started = not self.wait_for_green
        self.start_time = time.monotonic() if self.started else 0.0
        self.green_frames = 0
        self.start_marker_frames = 0
        self.start_marker_last_seen_time = 0.0
        self.state = "WAIT_TRAFFIC_LIGHT" if self.wait_for_green else "LANE_FOLLOW"
        self.overtake_until = 0.0
        self.overtake_started_at = 0.0
        self.overtake_direction = "left"
        self.obstacle_drive_state = "CENTER_DRIVING"
        self.overtake_passing_obstacle = False
        self.overtake_clear_since = 0.0
        self.overtake_phase = "IDLE"
        self.overtake_rejoin_started_at = 0.0
        self.overtake_rejoin_distance_m = 0.0
        self.overtake_rejoin_last_time = 0.0
        self.obstacle_track_time = 0.0
        self.obstacle_track_ratio = 0.0
        self.obstacle_track_velocity = 0.0
        self.obstacle_distance_track_time = 0.0
        self.obstacle_distance_track_m = float("inf")
        self.obstacle_closing_speed_mps = 0.0
        self.obstacle_distance_track_samples = 0
        self.obstacle_lane_candidate = "unknown"
        self.obstacle_lane_candidate_frames = 0
        self.obstacle_lane_track_generation = -1
        self.obstacle_lane_track_state = "unknown"
        self.obstacle_lane_track_ratio = 0.0
        self.obstacle_lane_track_velocity = 0.0
        self.obstacle_lane_track_frames = 0
        self.obstacle_confirm_frames = 0
        self.obstacle_last_confirm_generation = -1
        self.obstacle_last_seen_time = 0.0
        self.obstacle_last_estimate = ObstacleEstimate()
        self.obstacle_prepare_clear_since = 0.0
        self.obstacle_state_started_at = 0.0
        self.smoothed_obstacle_position = 0.0
        self.last_obstacle_position = "unknown"
        self.overtake_phase_started_at = 0.0
        self.overtake_cooldown_until = 0.0
        self.overtake_abort_hold_until = 0.0
        self.overtake_cut_in_hold_until = 0.0
        self.latest_shortcut_signal = False
        self.latest_intersection_signal = "unknown"
        self.intersection_stop_frames = 0
        self.intersection_go_frames = 0
        self.intersection_waiting_for_go = False
        self.fixed_obstacle_state = "CLEAR"
        self.fixed_obstacle_frames = 0
        self.fixed_obstacle_clear_frames = 0
        self.fixed_obstacle_started_at = 0.0
        self.fixed_obstacle_phase_started_at = 0.0
        self.fixed_obstacle_rejoin_started_at = 0.0
        self.fixed_obstacle_direction = "center"
        self.fixed_obstacle_last_detection_generation = 0
        self.fixed_obstacle_pass_detected = False
        self.fixed_obstacle_clear_since = 0.0
        self.fixed_obstacle_direction_candidate = ""
        self.fixed_obstacle_direction_clear_since = 0.0
        self.fixed_obstacle_cooldown_until = 0.0
        self.shortcut_signal_frames = 0
        self.shortcut_until = 0.0
        self.shortcut_taken = False
        self.shortcut_in_progress = False
        self.current_lane_target = self.target_lane_side
        self.last_lane_override_published = ""
        self.cone_mode_active = False
        self.cone_mode_started_at = 0.0
        self.cone_entry_frames = 0
        self.last_cone_visual_time = 0.0
        self.cone_lane_recovery_frames = 0
        self.cone_lane_last_generation = -1
        self.cone_lane_recovery_started_at = 0.0
        self.cone_lane_handoff_started_at = 0.0
        self.cone_lane_handoff_from_angle = 0.0
        self.cone_lane_handoff_from_speed = 0.0
        self.cone_lane_handoff_target_angle = 0.0
        self.cone_lane_handoff_target_speed = 0.0
        self.latest_finish_seen = False
        self.finish_frames = 0
        self.finish_clear_frames = 0
        self.finish_marker_armed = False
        self.lap_count = 0
        self.last_lap_time = self.start_time
        self.course_checkpoint_pending = ""
        self.route_gate_frames = 0
        self.route_gate_clear_frames = 0
        self.route_gate_armed = False
        self.route_gate_recent_until = 0.0
        self.normal_cone_pass_count = 0
        self.shortcut_pass_count = 0
        self.finished = False
        self.last_angle = 0.0
        self.last_speed = 0.0
        self.launch_hold_until = 0.0
        self.last_control_time = time.monotonic()
        self.last_status_log = 0.0
        self.last_cv_error_log = 0.0
        self.smoothed_lane_x: Optional[float] = None

        period = 1.0 / max(1.0, self.control_rate_hz)
        self.create_timer(period, self.control_callback)

        self.get_logger().info(
            "my_rule ready: image=%s scan=%s ultra=%s motor=%s type=%s"
            % (self.image_topic, self.scan_topic, self.ultra_topic, self.motor_topic, self.motor_message_type)
        )

    def create_motor_publisher(self):
        if self.motor_message_type == "xycar_motor" and XycarMotor is not None:
            return self.create_publisher(XycarMotor, self.motor_topic, 10)
        return self.create_publisher(Float32MultiArray, self.motor_topic, 10)

    def setup_detection_subscriptions(self) -> None:
        subscribed = False
        if DetectionArray is not None:
            self.create_subscription(DetectionArray, self.detection_topic, self.detection_callback, 10)
            self.get_logger().info(f"YOLO detections enabled on {self.detection_topic}")
            subscribed = True

        alt_topic = str(self.detection_alt_topic).strip()
        if bool(self.use_detection_alt_topic) and alt_topic and alt_topic != self.detection_topic:
            if LegacyDetections is not None:
                self.create_subscription(LegacyDetections, alt_topic, self.detection_callback, 10)
                self.get_logger().info(f"Legacy detections enabled on {alt_topic}")
                subscribed = True
            elif DetectionArray is not None:
                self.create_subscription(DetectionArray, alt_topic, self.detection_callback, 10)
                self.get_logger().info(f"YOLO-compatible alternate detections enabled on {alt_topic}")
                subscribed = True

        if not subscribed:
            self.get_logger().warn("No YOLO detection message package is available. Running without detections.")

    def _declare_parameters(self) -> None:
        params = {
            "image_topic": "/wide_camera/rect/image_raw",
            "camera_info_topic": "/wide_camera/rect/camera_info",
            "scan_topic": "/scan",
            "ultra_topic": "/xycar_ultrasonic",
            "motor_topic": "/xycar_motor",
            "motor_message_type": "float32_multi_array",
            "state_topic": "my_rule/state",
            "lane_cmd_topic": "my_rule/lane_cmd",
            "lane_override_topic": "my_rule/lane_override",
            "cone_cmd_topic": "my_rule/cone_cmd",
            "cone_cluster_topic": "my_rule/cone_clusters",
            "traffic_light_topic": "my_rule/traffic_light",
            "center_curve_topic": "/center_curve",
            "center_curve_timeout_sec": 0.5,
            "use_external_lane_cmd": True,
            "external_lane_only_fast_path": False,
            "use_external_cone_cmd": True,
            "use_internal_cone_fallback": False,
            "use_external_traffic_light": True,
            "external_cmd_timeout_sec": 0.5,
            "enable_yolo": True,
            "detection_topic": "/yolo/detections",
            "use_detection_alt_topic": True,
            "detection_alt_topic": "/yolo_detections",
            "wait_for_green": True,
            "green_required_frames": 4,
            "start_require_checkerboard": False,
            "start_checkerboard_required_frames": 2,
            "start_checkerboard_hold_sec": 1.0,
            "start_checkerboard_min_area_ratio": 0.018,
            "start_checkerboard_use_image_checker": False,
            "enable_intersection_signal_stop": True,
            "intersection_signal_required_frames": 2,
            "intersection_signal_release_frames": 2,
            "intersection_signal_min_start_elapsed_sec": 1.0,
            "intersection_signal_min_area_ratio": 0.0003,
            "intersection_signal_max_y_ratio": 0.65,
            "intersection_signal_use_image_color": False,
            "intersection_stop_on_unknown_when_waiting": True,
            "intersection_left_signal_assume_green": True,
            "control_rate_hz": 30.0,
            "base_speed": 8.0,
            "slow_speed": 4.0,
            "cone_speed": 17.0,
            "cone_launch_speed": 8.0,
            "cone_speed_accel_per_sec": 10.0,
            "cone_speed_decel_per_sec": 32.0,
            "overtake_speed": 2.5,
            "stop_speed": 0.0,
            "minimum_drive_speed": 4.0,
            "launch_speed": 3.0,
            "launch_hold_sec": 0.20,
            "lane_moderate_speed": 6.5,
            "lane_tight_speed": 5.0,
            "lane_moderate_threshold_deg": 5.0,
            "lane_tight_threshold_deg": 12.0,
            "lane_full_turn_threshold_deg": 22.0,
            "lane_confidence_stop": 0.12,
            "lane_confidence_low": 0.35,
            "lane_confidence_medium": 0.60,
            "lane_confidence_high": 0.75,
            "lane_low_confidence_speed": 4.0,
            "lane_medium_confidence_speed": 6.5,
            "lane_high_confidence_speed": 8.0,
            "max_steer_cmd": 26.0,
            "steer_rate_cmd_per_sec": 260.0,
            "speed_accel_per_sec": 16.0,
            "speed_decel_per_sec": 40.0,
            "pedestrian_resume_accel_per_sec": 16.0,
            "use_asymmetric_steering_map": False,
            "steering_negative_actual_deg": [0.0, 4.0, 10.0, 16.0, 26.0],
            "steering_negative_command": [0.0, 10.0, 20.0, 30.0, 42.0],
            "steering_positive_actual_deg": [0.0, 4.0, 10.0, 16.0, 26.0],
            "steering_positive_command": [0.0, 4.0, 10.0, 18.0, 32.0],
            "lane_steer_gain": 0.16,
            "lane_half_width_px": 115.0,
            "target_lane_side": "center",
            "lane_roi_top_ratio": 0.52,
            "lane_roi_bottom_ratio": 0.98,
            "lane_min_pixels": 120,
            "yellow_h_min": 14,
            "yellow_h_max": 45,
            "yellow_sat_min": 60,
            "yellow_val_min": 80,
            "white_sat_max": 120,
            "white_val_min": 145,
            "traffic_roi_top_ratio": 0.0,
            "traffic_roi_bottom_ratio": 0.45,
            "traffic_green_min_pixels": 45,
            "emergency_stop_distance_m": 0.45,
            "obstacle_distance_m": 1.15,
            "pedestrian_stop_distance_m": 1.8,
            "enable_pedestrian_handling": False,
            "pedestrian_required_frames": 1,
            "pedestrian_clear_required_frames": 2,
            "pedestrian_missing_clear_required_frames": 5,
            "pedestrian_stop_min_area_ratio": 0.008,
            "pedestrian_corridor_half_width_ratio": 0.24,
            "pedestrian_min_y_ratio": 0.20,
            "enable_fixed_obstacle_handling": True,
            "fixed_obstacle_detection_range_m": 2.2,
            "fixed_obstacle_required_frames": 3,
            "fixed_obstacle_clear_required_frames": 4,
            "fixed_obstacle_min_area_ratio": 0.008,
            "fixed_obstacle_corridor_half_width_ratio": 0.24,
            "fixed_obstacle_min_y_ratio": 0.20,
            "fixed_obstacle_require_lidar_confirmation": True,
            "fixed_obstacle_speed": 3.0,
            "fixed_obstacle_min_avoid_sec": 0.8,
            "fixed_obstacle_change_min_sec": 0.55,
            "fixed_obstacle_no_side_fallback_sec": 2.0,
            "fixed_obstacle_side_clear_hold_sec": 0.25,
            "fixed_obstacle_direction_clear_hold_sec": 0.15,
            "fixed_obstacle_pass_side_detect_cm": 30.0,
            "fixed_obstacle_pass_side_clear_cm": 40.0,
            "fixed_obstacle_front_clear_distance_m": 0.75,
            "fixed_obstacle_rejoin_sec": 0.8,
            "fixed_obstacle_rejoin_max_sec": 1.8,
            "fixed_obstacle_rejoin_lane_confidence": 0.35,
            "fixed_obstacle_cooldown_sec": 0.8,
            "fixed_obstacle_bias_cmd": 16.0,
            "fixed_obstacle_bias_sec": 0.5,
            "side_clearance_m": 0.65,
            "scan_angle_offset_deg": 0.0,
            "scan_min_range_m": 0.1,
            "scan_max_range_m": 8.0,
            "scan_front_min_deg": -12.0,
            "scan_front_max_deg": 12.0,
            "scan_left_min_deg": 20.0,
            "scan_left_max_deg": 75.0,
            "scan_right_min_deg": -75.0,
            "scan_right_max_deg": -20.0,
            "ultra_side_clearance_cm": 35.0,
            "ultra_left_index": 0,
            "ultra_right_index": 4,
            "ultra_left_back_index": 7,
            "ultra_right_back_index": 5,
            "cone_enable": True,
            "cone_range_m": 1.55,
            "cone_cluster_gap_m": 0.22,
            "cone_min_cluster_points": 3,
            "cone_max_cluster_diameter_m": 0.38,
            "cone_vfh_max_angle_deg": 62.0,
            "cone_steer_gain": 0.72,
            "cone_image_min_area": 260.0,
            "cone_yolo_keywords": "cone,traffic_cone",
            "cone_yolo_min_count": 2,
            "cone_yolo_min_area_ratio": 0.002,
            "use_yolo_cone_approach": True,
            "cone_yolo_approach_speed": 1.8,
            "cone_mode_require_yolo": False,
            "cone_mode_entry_required_frames": 3,
            "cone_mode_entry_confidence": 0.35,
            "cone_mode_min_duration_sec": 1.0,
            "cone_mode_exit_timeout_sec": 0.8,
            "cone_cluster_timeout_sec": 0.5,
            "cone_mode_presence_min_clusters": 2,
            "cone_mode_lane_recovery_required_frames": 6,
            "cone_mode_lane_recovery_confidence": 0.45,
            "cone_mode_lane_recovery_aligned_confidence": 0.25,
            "cone_mode_lane_recovery_max_angle_delta_deg": 12.0,
            "cone_mode_lane_recovery_hold_sec": 0.35,
            "cone_mode_exit_creep_timeout_sec": 1.4,
            "cone_exit_creep_speed": 8.0,
            "cone_exit_creep_steer_retention": 0.70,
            "cone_lane_handoff_duration_sec": 0.65,
            "cone_lane_handoff_max_speed": 3.5,
            "cone_emergency_stop_distance_m": 0.35,
            "overtake_duration_sec": 8.0,
            "overtake_bias_cmd": 4.0,
            "overtake_lane_change_bias_sec": 0.4,
            "overtake_detection_range_m": 3.0,
            "overtake_curve_range_m": 1.0,
            "overtake_multi_vehicle_gap_m": 0.8,
            "overtake_target_time_gap_sec": 2.0,
            "overtake_speed_to_mps_scale": 0.08,
            "overtake_require_vehicle_detection": True,
            "overtake_follow_kp": 0.80,
            "overtake_follow_closing_gain": 4.0,
            "overtake_follow_standstill_gap_m": 0.65,
            "overtake_min_follow_speed": 3.0,
            "overtake_curve_steer_threshold_cmd": 8.0,
            "overtake_min_area_ratio": 0.006,
            "overtake_vehicle_min_score": 0.45,
            "overtake_confirm_frames": 4,
            "overtake_detection_grace_sec": 0.35,
            "overtake_prepare_distance_m": 2.4,
            "overtake_min_commit_distance_m": 1.1,
            "overtake_prepare_clear_hold_sec": 0.25,
            "overtake_allow_curve_start": False,
            "overtake_allow_multi_vehicle": False,
            "overtake_min_lane_confidence": 0.45,
            "overtake_require_target_lane_markings": True,
            "overtake_min_ttc_sec": 3.0,
            "overtake_hard_brake_ttc_sec": 1.6,
            "overtake_emergency_ttc_sec": 0.9,
            "overtake_expected_vehicle_speed_cmd": 3.0,
            "overtake_change_speed": 5.0,
            "overtake_change_min_sec": 0.65,
            "overtake_pass_min_speed": 6.5,
            "overtake_pass_max_speed": 8.0,
            "overtake_pass_speed_margin_cmd": 3.5,
            "overtake_rejoin_speed": 5.0,
            "overtake_timeout_hold_speed": 3.0,
            "overtake_no_side_fallback_sec": 3.0,
            "overtake_front_clear_distance_m": 1.0,
            "overtake_cooldown_sec": 1.0,
            "overtake_abort_hold_sec": 0.8,
            "overtake_cut_in_hold_sec": 0.6,
            "overtake_require_fresh_ultra": True,
            "ultra_timeout_sec": 0.5,
            "overtake_relative_speed_alpha": 0.35,
            "overtake_max_range_jump_m": 0.8,
            "overtake_max_closing_speed_mps": 1.5,
            "overtake_lidar_match_hfov_deg": 60.0,
            "overtake_lidar_match_padding_deg": 3.0,
            "overtake_lidar_match_min_points": 2,
            "use_lidar_camera_projection": False,
            "lidar_camera_extrinsic_yaml": "",
            "lidar_projection_bbox_padding_px": 3.0,
            "lidar_projection_cluster_gap_m": 0.30,
            "overtake_pass_side_detect_cm": 30.0,
            "overtake_pass_side_clear_cm": 40.0,
            "overtake_pass_clear_hold_sec": 0.30,
            "overtake_side_emergency_cm": 18.0,
            "overtake_position_smooth_alpha": 0.30,
            "overtake_position_threshold": 0.15,
            "overtake_lane_stable_frames": 4,
            "overtake_lane_deadband_ratio": 0.05,
            "overtake_lateral_motion_threshold_ratio_per_sec": 0.08,
            "overtake_lateral_velocity_alpha": 0.35,
            "overtake_rejoin_min_sec": 0.65,
            "overtake_rejoin_max_sec": 1.50,
            "overtake_rejoin_deadline_m": 0.75,
            "overtake_rejoin_lane_confidence": 0.35,
            "overtake_rejoin_alignment_cmd": 5.0,
            "overtake_rejoin_hard_timeout_sec": 2.5,
            "enable_shortcut": True,
            "shortcut_green_required_frames": 3,
            "shortcut_duration_sec": 1.8,
            "shortcut_bias_cmd": -18.0,
            "shortcut_target_lap": 2,
            "enable_lap_count": True,
            "total_laps": 3,
            "lap_count_source": "route_signal",
            "route_gate_required_frames": 2,
            "route_gate_clear_required_frames": 3,
            "route_gate_recent_sec": 1.0,
            "route_gate_min_interval_sec": 8.0,
            "finish_required_frames": 2,
            "finish_clear_required_frames": 3,
            "finish_min_interval_sec": 12.0,
            "finish_stop_after_laps": False,
            "finish_min_area_ratio": 0.025,
            "finish_use_image_checker": True,
            "image_timeout_sec": 0.4,
            "scan_timeout_sec": 0.8,
            "detection_timeout_sec": 0.7,
            "detection_min_score": 0.35,
            "debug_view": False,
        }
        self._param_names = tuple(params.keys())
        for name, default in params.items():
            self.declare_parameter(name, default)

    def _read_parameters(self) -> None:
        for param in self._param_names:
            setattr(self, param, self.get_parameter(param).value)
        self.motor_message_type = str(self.motor_message_type).lower().strip()
        if self.motor_message_type in ("xycar", "xycar_msgs", "xycar_motor_msg"):
            self.motor_message_type = "xycar_motor"
        if self.motor_message_type not in ("float32_multi_array", "xycar_motor"):
            self.get_logger().warn("motor_message_type must be float32_multi_array or xycar_motor. Using float32_multi_array.")
            self.motor_message_type = "float32_multi_array"
        if self.motor_message_type == "xycar_motor" and XycarMotor is None:
            self.get_logger().warn("xycar_msgs/XycarMotor is unavailable. Falling back to Float32MultiArray motor output.")
            self.motor_message_type = "float32_multi_array"
        self.target_lane_side = str(self.target_lane_side).lower().strip()
        if self.target_lane_side not in ("left", "right", "center", "center_line", "middle"):
            self.get_logger().warn("target_lane_side must be left, right, or center. Using center.")
            self.target_lane_side = "center"
        if self.target_lane_side in ("center_line", "middle"):
            self.target_lane_side = "center"

    def image_callback(self, msg: Image) -> None:
        if bool(self.external_lane_only_fast_path):
            self.image_height = int(msg.height)
            self.image_width = int(msg.width)
            self.last_image_time = time.monotonic()
            return

        if cv2 is None or self.bridge is None:
            self._log_cv_unavailable()
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_cv_error_log > 2.0:
                self.get_logger().warn(f"Could not convert camera frame: {exc}")
                self.last_cv_error_log = now
            return

        self.image_height, self.image_width = frame.shape[:2]
        self.last_image_time = time.monotonic()
        self.latest_lane = self.detect_lane(frame)
        self.latest_cone_image = self.detect_cones_from_image(frame)
        self.latest_traffic_color = self.detect_traffic_light(frame)
        self.latest_shortcut_signal = self.detect_shortcut_signal(frame)
        self.update_start_marker(frame, self.last_image_time)
        self.update_intersection_signal(frame, self.last_image_time)
        self.latest_finish_seen = self.detect_finish_marker(frame)

        if self.debug_view:
            self.show_debug_view(frame)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        matrix = np.asarray(msg.k, dtype=np.float64)
        if matrix.size == 9 and np.all(np.isfinite(matrix)):
            self.camera_matrix = matrix.reshape(3, 3)

    def load_lidar_camera_extrinsic(self) -> None:
        if not bool(self.use_lidar_camera_projection):
            return
        path = os.path.expanduser(str(self.lidar_camera_extrinsic_yaml).strip())
        if yaml is None or not path or not os.path.isfile(path):
            self.get_logger().warn(
                f"LiDAR-camera projection disabled: invalid extrinsic YAML {path!r}."
            )
            return
        try:
            with open(path, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream)
            transform = data["T_camera_lidar"]
            self.lidar_camera_rotation = np.asarray(
                transform["R_row_major"], dtype=np.float64
            ).reshape(3, 3)
            self.lidar_camera_translation = np.asarray(
                transform["t_xyz"], dtype=np.float64
            ).reshape(3)
            self.get_logger().info(f"Loaded LiDAR-camera extrinsic: {path}")
        except Exception as exc:
            self.lidar_camera_rotation = None
            self.lidar_camera_translation = None
            self.get_logger().warn(f"LiDAR-camera projection disabled: {exc}")

    def scan_callback(self, msg: LaserScan) -> None:
        points = self.scan_to_points(msg)
        self.latest_scan_points = points
        self.latest_scan_clusters = self.extract_small_clusters(points)
        self.latest_front_distance = self.sector_min_distance(
            points, float(self.scan_front_min_deg), float(self.scan_front_max_deg)
        )
        self.latest_left_distance = self.sector_min_distance(
            points, float(self.scan_left_min_deg), float(self.scan_left_max_deg)
        )
        self.latest_right_distance = self.sector_min_distance(
            points, float(self.scan_right_min_deg), float(self.scan_right_max_deg)
        )
        self.last_scan_time = time.monotonic()

    def ultra_callback(self, msg: Int32MultiArray) -> None:
        self.latest_ultra = list(msg.data)
        self.last_ultra_time = time.monotonic()

    def lane_cmd_callback(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 3:
            return
        self.external_lane_angle = float(data[0])
        self.external_lane_speed = float(data[1])
        self.external_lane_confidence = float(data[2])
        self.external_lane_time = time.monotonic()
        self.external_lane_generation += 1
        self.external_lane_observed = (
            tuple(float(value) >= 0.5 for value in data[3:6])
            if len(data) >= 6
            else (False, False, False)
        )

    def cone_cmd_callback(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 3:
            return
        now = time.monotonic()
        self.external_cone_angle = float(data[0])
        self.external_cone_speed = float(data[1])
        self.external_cone_confidence = float(data[2])
        self.external_cone_time = now
        active_recovery = self.cone_mode_active and self.external_cone_confidence > 0.2
        entry_candidate = self.external_cone_confidence >= float(self.cone_mode_entry_confidence)
        if active_recovery or entry_candidate:
            self.last_valid_cone_cmd_time = now
            self.last_valid_cone_angle = self.external_cone_angle
            self.last_valid_cone_speed = self.external_cone_speed
        if entry_candidate:
            if not self.cone_mode_active:
                self.cone_path_entry_frames += 1
        elif not self.cone_mode_active:
            self.cone_path_entry_frames = 0

    def cone_cluster_callback(self, msg: PoseArray) -> None:
        self.external_cone_cluster_count = len(msg.poses)
        self.external_cone_cluster_time = time.monotonic()

    def traffic_light_callback(self, msg: String) -> None:
        if not bool(self.use_external_traffic_light):
            return
        self.external_traffic_color = msg.data.strip().lower()
        self.latest_traffic_color = self.external_traffic_color
        self.external_traffic_time = time.monotonic()

    def center_curve_callback(self, msg: Path) -> None:
        points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        if len(points) < 2:
            self.center_curve_points = None
            return
        self.center_curve_points = np.asarray(points, dtype=np.float32)
        self.center_curve_time = time.monotonic()

    def detection_callback(self, msg) -> None:
        detections = []
        for det in getattr(msg, "detections", []):
            parsed = self.parse_detection(det)
            if parsed is not None and parsed.score >= float(self.detection_min_score):
                detections.append(parsed)
        self.latest_detections = detections
        now = time.monotonic()
        self.last_detection_time = now
        self.detection_generation += 1
        if self.yolo_cone_entry_detected():
            self.cone_entry_frames += 1
            self.last_cone_visual_time = now
        else:
            self.cone_entry_frames = 0

    def control_callback(self) -> None:
        now = time.monotonic()
        dt = max(0.001, now - self.last_control_time)
        self.last_control_time = now
        previous_state = self.state

        raw_angle, raw_speed, state, reason = self.choose_command(now)
        raw_speed = self.normalize_drive_speed(raw_speed)
        angle = self.rate_limit(self.last_angle, raw_angle, float(self.steer_rate_cmd_per_sec), dt)
        launch_speed = self.launch_speed_for_state(state)
        accel_rate, decel_rate = self.speed_rates_for_state(state)
        if raw_speed <= float(self.stop_speed) + 1e-6:
            # Safety decisions must reach the ROS1 motor bridge in this cycle.
            speed = float(self.stop_speed)
            self.launch_hold_until = 0.0
        elif self.last_speed <= float(self.stop_speed) + 1e-6:
            speed = min(
                raw_speed,
                max(float(self.minimum_drive_speed), launch_speed),
            )
            self.launch_hold_until = now + max(0.0, float(self.launch_hold_sec))
        elif now < self.launch_hold_until:
            speed = min(
                raw_speed,
                max(float(self.minimum_drive_speed), launch_speed),
            )
        elif raw_speed >= self.last_speed:
            accel = accel_rate
            if previous_state == "FIXED_OBSTACLE_WAIT":
                accel = max(accel, float(self.pedestrian_resume_accel_per_sec))
            speed = self.rate_limit(self.last_speed, raw_speed, accel, dt)
        else:
            speed = self.rate_limit(self.last_speed, raw_speed, decel_rate, dt)

        self.last_angle = angle
        self.last_speed = speed
        self.state = state

        self.publish_motor(angle, speed)
        self.publish_state(angle, speed, reason)

    def choose_command(self, now: float) -> Tuple[float, float, str, str]:
        image_age = now - self.last_image_time if self.last_image_time > 0.0 else float("inf")
        scan_age = now - self.last_scan_time if self.last_scan_time > 0.0 else float("inf")

        front_distance = self.latest_front_distance if scan_age < float(self.scan_timeout_sec) else float("inf")

        emergency_distance = (
            float(self.cone_emergency_stop_distance_m)
            if self.cone_mode_active
            else float(self.emergency_stop_distance_m)
        )
        if front_distance < emergency_distance:
            return 0.0, float(self.stop_speed), "EMERGENCY_STOP", f"front={front_distance:.2f}m"

        if self.wait_for_green and not self.started:
            self.set_lane_override("center")
            start_marker_ready = self.start_marker_ready(now)
            if self.is_green_light() and start_marker_ready:
                self.green_frames += 1
            else:
                self.green_frames = 0
            if self.green_frames >= int(self.green_required_frames):
                self.mark_started(now)
                self.get_logger().info("Green light confirmed. Starting rule drive.")
            else:
                if not start_marker_ready:
                    marker_frames = self.start_marker_frames
                    required_frames = int(self.start_checkerboard_required_frames)
                    reason = f"marker={marker_frames}/{required_frames}, light={self.latest_traffic_color}"
                    return 0.0, float(self.stop_speed), "WAIT_START_MARKER", reason
                return 0.0, float(self.stop_speed), "WAIT_TRAFFIC_LIGHT", self.latest_traffic_color

        if self.finished:
            return 0.0, float(self.stop_speed), "FINISH", f"lap={self.lap_count}"

        if image_age > float(self.image_timeout_sec) and not self.cone_mode_active:
            return 0.0, float(self.stop_speed), "WAIT_CAMERA", f"image_age={image_age:.2f}s"

        self.finish_shortcut_if_needed(now)
        self.update_lap_counter(now)
        if self.finished:
            return 0.0, float(self.stop_speed), "FINISH", f"lap={self.lap_count}"

        intersection_stop_reason = self.intersection_signal_stop_reason(now)
        if intersection_stop_reason is not None:
            return 0.0, float(self.stop_speed), "WAIT_LEFT_TRAFFIC_LIGHT", intersection_stop_reason

        fixed_obstacle_command = self.fixed_obstacle_command(now, front_distance)
        if fixed_obstacle_command is not None:
            return fixed_obstacle_command

        if self.should_take_shortcut(now):
            self.shortcut_until = now + float(self.shortcut_duration_sec)
            self.shortcut_taken = True
            self.shortcut_in_progress = True
            self.reset_cone_mode()
            self.set_lane_override("center")

        if now < self.shortcut_until:
            steer = self.clamp_angle(float(self.shortcut_bias_cmd) + self.lane_steer() * 0.35)
            return steer, float(self.slow_speed), "SHORTCUT_LEFT", "shortcut"

        self.update_cone_mode(now)
        if self.cone_mode_active:
            self.set_lane_override("center")
            cone_command = self.cone_command(now)
            if cone_command is not None:
                return cone_command[0], cone_command[1], "CONE_SLALOM", cone_command[2]

            angle, speed, reason = self.cone_dropout_recovery(now)
            return angle, speed, "CONE_RECOVERY", reason

        handoff_command = self.cone_lane_handoff_command(now)
        if handoff_command is not None:
            angle, speed, reason = handoff_command
            return angle, speed, "CONE_HANDOFF", reason

        obstacle_command = self.obstacle_command(now, front_distance)
        if obstacle_command is not None:
            return obstacle_command

        cone_approach_command = self.cone_approach_command()
        if cone_approach_command is not None:
            return cone_approach_command

        self.set_lane_override("center")
        lane, speed, lane_source = self.lane_command(now)
        if abs(lane) > float(self.max_steer_cmd) * 0.65:
            speed = min(speed, float(self.slow_speed))
        return lane, speed, "LANE_FOLLOW", lane_source

    def detect_lane(self, frame: np.ndarray) -> LaneEstimate:
        h, w = frame.shape[:2]
        roi_top = int(h * float(self.lane_roi_top_ratio))
        roi_bottom = int(h * float(self.lane_roi_bottom_ratio))
        roi = frame[roi_top:roi_bottom, :]
        if roi.size == 0:
            return LaneEstimate()

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]

        white_mask = (
            (sat <= float(self.white_sat_max))
            & (val >= float(self.white_val_min))
        ).astype(np.uint8) * 255
        yellow_mask = (
            (hue >= float(self.yellow_h_min))
            & (hue <= float(self.yellow_h_max))
            & (sat >= float(self.yellow_sat_min))
            & (val >= float(self.yellow_val_min))
        ).astype(np.uint8) * 255

        kernel = np.ones((3, 3), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        band_top = int(roi.shape[0] * 0.45)
        band = slice(band_top, roi.shape[0])
        image_mid = w * 0.5

        white_y, white_x = np.where(white_mask[band, :] > 0)
        yellow_y, yellow_x = np.where(yellow_mask[band, :] > 0)

        min_pixels = int(self.lane_min_pixels)
        left_x = self.median_or_none(white_x[white_x < image_mid - 10])
        right_x = self.median_or_none(white_x[white_x > image_mid + 10])
        yellow_center = self.median_or_none(yellow_x)

        target_x = None
        source = "none"
        confidence = 0.0

        if yellow_center is not None and len(yellow_x) >= max(40, min_pixels // 3):
            offset = float(self.lane_half_width_px)
            if self.current_lane_target == "center":
                target_x = yellow_center
            else:
                target_x = yellow_center + offset if self.current_lane_target == "right" else yellow_center - offset
            source = "yellow_center"
            confidence = 0.82
        elif left_x is not None and right_x is not None and len(white_x) >= min_pixels:
            target_x = (left_x + right_x) * 0.5
            source = "white_pair"
            confidence = 0.90
        elif right_x is not None and np.count_nonzero(white_x > image_mid) >= max(40, min_pixels // 3):
            target_x = right_x - float(self.lane_half_width_px)
            source = "right_white"
            confidence = 0.55
        elif left_x is not None and np.count_nonzero(white_x < image_mid) >= max(40, min_pixels // 3):
            target_x = left_x + float(self.lane_half_width_px)
            source = "left_white"
            confidence = 0.55

        if target_x is None:
            if self.smoothed_lane_x is not None:
                return LaneEstimate(True, self.smoothed_lane_x, image_mid, 0.25, "predicted")
            return LaneEstimate(False, image_mid, image_mid, 0.0, "none")

        target_x = float(np.clip(target_x, 0.0, float(w - 1)))
        if self.smoothed_lane_x is None:
            self.smoothed_lane_x = target_x
        else:
            self.smoothed_lane_x = 0.72 * self.smoothed_lane_x + 0.28 * target_x

        return LaneEstimate(True, self.smoothed_lane_x, image_mid, confidence, source)

    def detect_cones_from_image(self, frame: np.ndarray) -> ConeEstimate:
        if not bool(self.cone_enable):
            return ConeEstimate()

        yolo_cones = self.detect_cones_from_yolo(frame.shape[:2])
        if yolo_cones.active:
            return yolo_cones

        h, w = frame.shape[:2]
        roi = frame[int(h * 0.38): h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower = np.array([4, 80, 80], dtype=np.uint8)
        upper = np.array([24, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = []
        total_area = 0.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < float(self.cone_image_min_area):
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0.0:
                continue
            cx = moments["m10"] / moments["m00"]
            centers.append(cx)
            total_area += area

        if not centers:
            return ConeEstimate()

        cone_center = float(np.mean(centers))
        error = (cone_center - (w * 0.5)) / max(1.0, w * 0.5)
        steer_hint = self.clamp_angle(error * float(self.max_steer_cmd) * 0.65)
        confidence = float(np.clip(total_area / 5000.0, 0.25, 1.0))
        return ConeEstimate(True, steer_hint, confidence, "orange_image")

    def detect_cones_from_yolo(self, image_shape: Tuple[int, int]) -> ConeEstimate:
        h, w = image_shape
        cones = self.valid_yolo_cones(h)
        if not cones:
            return ConeEstimate()

        centers = [det.cx for det in cones]
        total_area = sum(det.area for det in cones)
        cone_center = float(np.mean(centers))
        error = (cone_center - (w * 0.5)) / max(1.0, w * 0.5)
        steer_hint = self.clamp_angle(error * float(self.max_steer_cmd) * 0.45)
        confidence = float(np.clip(0.25 + 0.12 * len(cones) + total_area / max(1.0, self.image_area()) * 2.0, 0.0, 1.0))
        return ConeEstimate(True, steer_hint, confidence, "yolo_cone")

    def valid_yolo_cones(self, image_height: Optional[int] = None) -> List[ObjectDetection]:
        height = int(image_height if image_height is not None else self.image_height)
        min_area = float(self.cone_yolo_min_area_ratio) * self.image_area()
        cones = []
        for det in self.filter_detections(self.cone_yolo_keywords):
            if det.area < min_area:
                continue
            if height > 0 and det.cy < height * 0.12:
                continue
            cones.append(det)
        if len(cones) < int(self.cone_yolo_min_count):
            return []
        return cones

    def yolo_cone_entry_detected(self) -> bool:
        cones = self.valid_yolo_cones()
        if not cones:
            return False
        total_area = sum(det.area for det in cones)
        confidence = float(
            np.clip(
                0.25 + 0.12 * len(cones) + total_area / max(1.0, self.image_area()) * 2.0,
                0.0,
                1.0,
            )
        )
        return confidence >= float(self.cone_mode_entry_confidence)

    def detect_traffic_light(self, frame: np.ndarray) -> str:
        roi = self.traffic_roi(frame)
        if roi.size == 0:
            return "unknown"

        color = self.classify_light_roi(roi)
        det_roi_color = self.classify_detection_light_roi(frame)
        if det_roi_color != "unknown":
            return det_roi_color
        return color

    def traffic_roi(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        top = int(h * float(self.traffic_roi_top_ratio))
        bottom = int(h * float(self.traffic_roi_bottom_ratio))
        left = int(w * 0.18)
        right = int(w * 0.82)
        return frame[top:bottom, left:right]

    def classify_detection_light_roi(self, frame: np.ndarray) -> str:
        traffic = self.filter_detections(["traffic", "signal", "light"])
        if not traffic:
            return "unknown"
        det = max(traffic, key=lambda item: item.score)
        color = self.signal_color_from_detection(frame, det)
        if color != "unknown":
            return color
        return self.classify_detection_roi(frame, det)

    def classify_detection_roi(self, frame: np.ndarray, det: ObjectDetection) -> str:
        h, w = frame.shape[:2]
        x1 = int(np.clip(det.cx - det.width * 0.5, 0, w - 1))
        y1 = int(np.clip(det.cy - det.height * 0.5, 0, h - 1))
        x2 = int(np.clip(det.cx + det.width * 0.5, x1 + 1, w))
        y2 = int(np.clip(det.cy + det.height * 0.5, y1 + 1, h))
        return self.classify_light_roi(frame[y1:y2, x1:x2])

    def classify_light_roi(self, roi: np.ndarray) -> str:
        if roi.size == 0:
            return "unknown"
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red1 = cv2.inRange(hsv, np.array([0, 95, 100]), np.array([10, 255, 255]))
        red2 = cv2.inRange(hsv, np.array([170, 95, 100]), np.array([179, 255, 255]))
        yellow = cv2.inRange(hsv, np.array([15, 85, 105]), np.array([38, 255, 255]))
        green = cv2.inRange(hsv, np.array([45, 75, 95]), np.array([95, 255, 255]))

        red_count = int(np.count_nonzero(red1) + np.count_nonzero(red2))
        yellow_count = int(np.count_nonzero(yellow))
        green_count = int(np.count_nonzero(green))
        min_pixels = int(self.traffic_green_min_pixels)

        if max(red_count, yellow_count, green_count) < min_pixels:
            return "unknown"
        if green_count >= red_count and green_count >= yellow_count:
            return "green"
        if yellow_count >= red_count:
            return "yellow"
        return "red"

    def update_intersection_signal(self, frame: np.ndarray, now: float) -> None:
        if not bool(self.enable_intersection_signal_stop) or not self.started:
            self.latest_intersection_signal = "unknown"
            self.intersection_stop_frames = 0
            self.intersection_go_frames = 0
            self.intersection_waiting_for_go = False
            return

        if self.start_time > 0.0 and now - self.start_time < float(self.intersection_signal_min_start_elapsed_sec):
            self.latest_intersection_signal = "unknown"
            return

        signal = self.detect_intersection_signal(frame)
        self.latest_intersection_signal = signal
        if signal in ("red", "yellow"):
            self.intersection_stop_frames += 1
            self.intersection_go_frames = 0
            if self.intersection_stop_frames >= int(self.intersection_signal_required_frames):
                self.intersection_waiting_for_go = True
        elif signal in ("green", "left_green"):
            self.intersection_go_frames += 1
            self.intersection_stop_frames = 0
            if self.intersection_go_frames >= int(self.intersection_signal_release_frames):
                self.intersection_waiting_for_go = False
        else:
            self.intersection_stop_frames = max(0, self.intersection_stop_frames - 1)
            self.intersection_go_frames = 0

    def detect_intersection_signal(self, frame: np.ndarray) -> str:
        candidates = self.intersection_signal_detections()
        left_green_seen = False
        green_seen = False
        yellow_seen = False
        red_seen = False

        for det in candidates:
            name = det.class_name.lower()
            color = self.signal_color_from_detection(frame, det)
            is_left_signal = (
                "left_signal" in name
                or "left_turn" in name
                or ("left" in name and ("signal" in name or "turn" in name or "traffic" in name or "light" in name))
            )

            if is_left_signal:
                if "red" in name:
                    red_seen = True
                    continue
                if "yellow" in name:
                    yellow_seen = True
                    continue
                if (
                    "green" in name
                    or "go" in name
                    or color == "green"
                    or bool(self.intersection_left_signal_assume_green)
                ):
                    left_green_seen = True
                    continue

            if "green" in name or color == "green":
                green_seen = True
            elif "yellow" in name or color == "yellow":
                yellow_seen = True
            elif "red" in name or color == "red":
                red_seen = True

        if left_green_seen:
            return "left_green"
        if green_seen:
            return "green"
        if yellow_seen:
            return "yellow"
        if red_seen:
            return "red"

        if bool(self.intersection_signal_use_image_color):
            return self.classify_light_roi(self.traffic_roi(frame))
        return "unknown"

    def intersection_signal_detections(self) -> List[ObjectDetection]:
        min_area = float(self.intersection_signal_min_area_ratio) * self.image_area()
        keywords = ("red", "yellow", "green", "traffic", "signal", "light", "left", "turn")
        detections = []
        for det in self.filter_detections(keywords):
            if det.area < min_area:
                continue
            if not self.detection_in_signal_region(det):
                continue
            detections.append(det)
        return detections

    def signal_color_from_detection(self, frame: np.ndarray, det: ObjectDetection) -> str:
        name = det.class_name.lower()
        if "green" in name:
            return "green"
        if "yellow" in name:
            return "yellow"
        if "red" in name:
            return "red"
        if "light" in name or "signal" in name or "traffic" in name or "left" in name or "turn" in name:
            return self.classify_detection_roi(frame, det)
        return "unknown"

    def scan_to_points(self, msg: LaserScan) -> np.ndarray:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
        angles = angles + math.radians(float(self.scan_angle_offset_deg))
        min_range = max(float(msg.range_min), float(self.scan_min_range_m), 0.03)
        max_range = min(float(msg.range_max), float(self.scan_max_range_m))
        valid = np.isfinite(ranges) & (ranges > min_range) & (ranges < max_range)
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)

        ranges = ranges[valid]
        angles = angles[valid]
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        angle_deg = np.degrees(angles)
        return np.column_stack((x, y, angle_deg)).astype(np.float32)

    def sector_min_distance(self, points: np.ndarray, min_deg: float, max_deg: float) -> float:
        if points.size == 0:
            return float("inf")
        mask = (points[:, 2] >= min_deg) & (points[:, 2] <= max_deg) & (points[:, 0] > 0.0)
        if not np.any(mask):
            return float("inf")
        ranges = np.linalg.norm(points[mask, 0:2], axis=1)
        return float(np.min(ranges))

    def extract_small_clusters(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        fov = float(self.cone_vfh_max_angle_deg) + 15.0
        dist = np.linalg.norm(points[:, 0:2], axis=1)
        mask = (
            (points[:, 0] > 0.05)
            & (np.abs(points[:, 2]) < fov)
            & (dist < float(self.cone_range_m))
        )
        pts = points[mask]
        if pts.shape[0] < int(self.cone_min_cluster_points):
            return np.empty((0, 3), dtype=np.float32)

        pts = pts[np.argsort(pts[:, 2])]
        clusters = []
        current = [pts[0]]
        gap = float(self.cone_cluster_gap_m)

        for point in pts[1:]:
            prev = current[-1]
            if np.linalg.norm(point[0:2] - prev[0:2]) <= gap:
                current.append(point)
            else:
                clusters.append(np.asarray(current, dtype=np.float32))
                current = [point]
        clusters.append(np.asarray(current, dtype=np.float32))

        centroids = []
        for cluster in clusters:
            if cluster.shape[0] < int(self.cone_min_cluster_points):
                continue
            xy = cluster[:, 0:2]
            diameter = float(np.max(np.linalg.norm(xy - xy.mean(axis=0), axis=1)) * 2.0)
            if diameter > float(self.cone_max_cluster_diameter_m):
                continue
            cx, cy = xy.mean(axis=0)
            centroids.append([float(cx), float(cy), diameter])

        if not centroids:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(centroids, dtype=np.float32)

    def update_cone_mode(self, now: float) -> None:
        if not bool(self.cone_enable) or not self.started:
            self.reset_cone_mode()
            return

        # Do not re-enter from the last lingering cone command while the
        # controller is deliberately blending back to lane following.
        if self.cone_lane_handoff_started_at > 0.0:
            return

        visual_recent = (
            self.last_cone_visual_time > 0.0
            and now - self.last_cone_visual_time <= float(self.detection_timeout_sec)
        )
        path_recent = (
            bool(self.use_external_cone_cmd)
            and self.last_valid_cone_cmd_time > 0.0
            and now - self.last_valid_cone_cmd_time <= float(self.external_cmd_timeout_sec)
            and self.external_cone_confidence >= float(self.cone_mode_entry_confidence)
        )
        if not path_recent and not self.cone_mode_active:
            self.cone_path_entry_frames = 0
        if not visual_recent and not self.cone_mode_active:
            self.cone_entry_frames = 0

        if not self.cone_mode_active:
            required_frames = int(self.cone_mode_entry_required_frames)
            lidar_confirmed = path_recent and self.cone_path_entry_frames >= required_frames
            visual_confirmed = visual_recent and self.cone_entry_frames >= required_frames
            if not lidar_confirmed:
                return
            if bool(self.cone_mode_require_yolo) and not visual_confirmed:
                return
            self.cone_mode_active = True
            self.cone_mode_started_at = now
            self.cone_lane_recovery_frames = 0
            self.cone_lane_last_generation = self.external_lane_generation
            self.cone_lane_recovery_started_at = 0.0
            self.obstacle_drive_state = "CENTER_DRIVING"
            source = "LiDAR+YOLO" if bool(self.cone_mode_require_yolo) else "LiDAR"
            self.get_logger().info(f"Cone entry confirmed by {source}. Cone-path mode enabled.")
            return

        if now - self.cone_mode_started_at < float(self.cone_mode_min_duration_sec):
            return

        path_recent = (
            self.last_valid_cone_cmd_time > 0.0
            and now - self.last_valid_cone_cmd_time <= float(self.cone_mode_exit_timeout_sec)
        )
        clusters_recent = (
            self.external_cone_cluster_count >= int(self.cone_mode_presence_min_clusters)
            and self.external_cone_cluster_time > 0.0
            and now - self.external_cone_cluster_time <= float(self.cone_cluster_timeout_sec)
        )
        if visual_recent or path_recent or clusters_recent:
            self.cone_lane_recovery_frames = 0
            self.cone_lane_last_generation = self.external_lane_generation
            self.cone_lane_recovery_started_at = 0.0
            return

        if not self.lane_ready_for_cone_handoff(now):
            self.cone_lane_recovery_frames = 0
            self.cone_lane_last_generation = self.external_lane_generation
            self.cone_lane_recovery_started_at = 0.0
            return

        # The control timer runs faster than some perception publishers. Count
        # distinct lane messages rather than repeatedly counting one stale
        # sample as several frames.
        if self.external_lane_generation != self.cone_lane_last_generation:
            self.cone_lane_last_generation = self.external_lane_generation
            self.cone_lane_recovery_frames += 1
            if self.cone_lane_recovery_started_at <= 0.0:
                self.cone_lane_recovery_started_at = now

        if self.cone_lane_recovery_frames < int(self.cone_mode_lane_recovery_required_frames):
            return
        if now - self.cone_lane_recovery_started_at < float(self.cone_mode_lane_recovery_hold_sec):
            return

        self.begin_cone_lane_handoff(now)

    def lane_ready_for_cone_handoff(self, now: float) -> bool:
        if not bool(self.use_external_lane_cmd):
            return False
        if now - self.external_lane_time > float(self.external_cmd_timeout_sec):
            return False
        if self.external_lane_speed <= float(self.stop_speed) + 1e-6:
            return False

        angle_delta = abs(float(self.external_lane_angle) - float(self.last_valid_cone_angle))
        if angle_delta <= float(self.cone_mode_lane_recovery_max_angle_delta_deg):
            required_confidence = float(self.cone_mode_lane_recovery_aligned_confidence)
        else:
            required_confidence = float(self.cone_mode_lane_recovery_confidence)
        return self.external_lane_confidence >= required_confidence

    def begin_cone_lane_handoff(self, now: float) -> None:
        from_angle = self.clamp_angle(self.last_valid_cone_angle)
        from_speed = (
            float(self.last_valid_cone_speed)
            if self.last_valid_cone_speed > float(self.stop_speed)
            else float(self.cone_exit_creep_speed)
        )
        target_angle = self.clamp_angle(self.external_lane_angle)
        target_speed = min(
            float(self.external_lane_speed),
            float(self.cone_lane_handoff_max_speed),
        )

        self.reset_cone_mode()
        self.cone_lane_handoff_started_at = now
        self.cone_lane_handoff_from_angle = from_angle
        self.cone_lane_handoff_from_speed = from_speed
        self.cone_lane_handoff_target_angle = target_angle
        self.cone_lane_handoff_target_speed = max(float(self.minimum_drive_speed), target_speed)
        self.mark_course_checkpoint("normal")
        self.get_logger().info("Lane recovered after cone section. Blending back to lane following.")

    def cone_lane_handoff_command(self, now: float) -> Optional[Tuple[float, float, str]]:
        if self.cone_lane_handoff_started_at <= 0.0:
            return None

        if self.lane_ready_for_cone_handoff(now):
            self.cone_lane_handoff_target_angle = self.clamp_angle(self.external_lane_angle)
            self.cone_lane_handoff_target_speed = max(
                float(self.minimum_drive_speed),
                min(float(self.external_lane_speed), float(self.cone_lane_handoff_max_speed)),
            )

        duration = max(0.05, float(self.cone_lane_handoff_duration_sec))
        progress = float(np.clip((now - self.cone_lane_handoff_started_at) / duration, 0.0, 1.0))
        blend = progress * progress * (3.0 - 2.0 * progress)
        angle = (
            (1.0 - blend) * self.cone_lane_handoff_from_angle
            + blend * self.cone_lane_handoff_target_angle
        )
        speed = (
            (1.0 - blend) * self.cone_lane_handoff_from_speed
            + blend * self.cone_lane_handoff_target_speed
        )

        if progress >= 1.0:
            self.cone_lane_handoff_started_at = 0.0
        return self.clamp_angle(angle), speed, f"lane_blend={progress:.2f}"

    def reset_cone_mode(self) -> None:
        self.cone_mode_active = False
        self.cone_mode_started_at = 0.0
        self.cone_entry_frames = 0
        self.cone_path_entry_frames = 0
        self.cone_lane_recovery_frames = 0
        self.cone_lane_last_generation = self.external_lane_generation
        self.cone_lane_recovery_started_at = 0.0
        self.cone_lane_handoff_started_at = 0.0

    def cone_steer(self) -> Optional[float]:
        lidar_active = self.latest_scan_clusters.shape[0] >= 2
        image_active = self.latest_cone_image.active and self.latest_cone_image.confidence > 0.35
        if not lidar_active and not image_active:
            return None

        if lidar_active:
            return self.cone_vfh_steer()
        return self.latest_cone_image.steer_hint

    def cone_command(self, now: float) -> Optional[Tuple[float, float, str]]:
        if (
            bool(self.use_external_cone_cmd)
            and now - self.external_cone_time <= float(self.external_cmd_timeout_sec)
            and self.external_cone_confidence > 0.2
        ):
            speed = self.external_cone_speed if self.external_cone_speed > 0.0 else float(self.cone_speed)
            return self.clamp_angle(self.external_cone_angle), speed, "external_cone"

        if not bool(self.use_internal_cone_fallback):
            return None
        steer = self.cone_steer()
        if steer is None:
            return None
        return steer, float(self.cone_speed), "internal_cone"

    def cone_dropout_recovery(self, now: float) -> Tuple[float, float, str]:
        age = (
            now - self.last_valid_cone_cmd_time
            if self.last_valid_cone_cmd_time > 0.0
            else float("inf")
        )
        if age <= float(self.cone_mode_exit_timeout_sec):
            speed = (
                self.last_valid_cone_speed
                if self.last_valid_cone_speed > 0.0
                else float(self.cone_speed)
            )
            return self.clamp_angle(self.last_valid_cone_angle), speed, "hold_last_cone"

        creep_timeout = max(
            float(self.cone_mode_exit_timeout_sec),
            float(self.cone_mode_exit_creep_timeout_sec),
        )
        if age <= creep_timeout:
            creep_span = max(1e-6, creep_timeout - float(self.cone_mode_exit_timeout_sec))
            progress = float(
                np.clip(
                    (age - float(self.cone_mode_exit_timeout_sec)) / creep_span,
                    0.0,
                    1.0,
                )
            )
            retention = 1.0 - progress * (
                1.0 - float(self.cone_exit_creep_steer_retention)
            )
            angle = self.clamp_angle(self.last_valid_cone_angle * retention)
            speed = max(float(self.minimum_drive_speed), float(self.cone_exit_creep_speed))
            return angle, speed, "cone_exit_creep"

        return 0.0, float(self.stop_speed), "waiting_cone_path"

    def cone_approach_command(self) -> Optional[Tuple[float, float, str, str]]:
        if not bool(self.use_yolo_cone_approach):
            return None
        if not self.latest_cone_image.active or self.latest_cone_image.confidence < 0.35:
            return None
        speed = min(float(self.cone_yolo_approach_speed), float(self.slow_speed), float(self.base_speed))
        return self.lane_steer(), speed, "CONE_APPROACH", self.latest_cone_image.source

    def cone_vfh_steer(self) -> float:
        clusters = self.latest_scan_clusters
        max_angle = float(self.cone_vfh_max_angle_deg)
        bins = np.linspace(-max_angle, max_angle, 51)
        risk = np.zeros_like(bins)

        for cx, cy, diameter in clusters:
            distance = max(0.05, math.hypot(float(cx), float(cy)))
            angle = math.degrees(math.atan2(float(cy), float(cx)))
            spread = max(8.0, math.degrees(math.atan2(float(diameter) * 0.65 + 0.18, distance)))
            weight = 1.0 / max(0.1, distance) ** 1.7
            risk += weight * np.exp(-0.5 * ((bins - angle) / spread) ** 2)

        risk += 0.015 * np.abs(bins)
        best_angle = float(bins[int(np.argmin(risk))])
        steer = -best_angle * float(self.cone_steer_gain)
        return self.clamp_angle(steer)

    def lane_steer(self) -> float:
        if not self.latest_lane.valid:
            return 0.0
        error_px = self.latest_lane.target_x - self.latest_lane.image_center_x
        return self.clamp_angle(error_px * float(self.lane_steer_gain))

    def lane_command(self, now: float) -> Tuple[float, float, str]:
        external_is_fresh = (
            bool(self.use_external_lane_cmd)
            and now - self.external_lane_time <= float(self.external_cmd_timeout_sec)
        )
        if external_is_fresh:
            if (
                self.external_lane_confidence < float(self.lane_confidence_stop)
                or self.external_lane_speed <= float(self.stop_speed) + 1e-6
            ):
                return 0.0, float(self.stop_speed), "external_lane_lost"
            angle = self.clamp_angle(self.external_lane_angle)
            profile_speed = self.lane_speed_for_state(angle, self.external_lane_confidence)
            return (
                angle,
                min(self.external_lane_speed, profile_speed),
                "external_lane",
            )

        if not self.latest_lane.valid:
            return 0.0, float(self.stop_speed), "lane_lost"
        angle = self.lane_steer()
        speed = self.lane_speed_for_state(angle, self.latest_lane.confidence)
        return angle, speed, self.latest_lane.source

    def lane_speed_for_state(self, angle: float, confidence: float) -> float:
        if confidence < float(self.lane_confidence_stop):
            return float(self.stop_speed)

        points = (
            (0.0, float(self.base_speed)),
            (float(self.lane_moderate_threshold_deg), float(self.lane_moderate_speed)),
            (float(self.lane_tight_threshold_deg), float(self.lane_tight_speed)),
            (float(self.lane_full_turn_threshold_deg), float(self.minimum_drive_speed)),
        )
        severity = abs(float(angle))
        speed = points[-1][1]
        for index in range(1, len(points)):
            low_angle, low_speed = points[index - 1]
            high_angle, high_speed = points[index]
            if severity <= high_angle:
                span = max(1e-6, high_angle - low_angle)
                ratio = (severity - low_angle) / span
                speed = low_speed + ratio * (high_speed - low_speed)
                break

        if confidence < float(self.lane_confidence_low):
            speed = min(speed, float(self.lane_low_confidence_speed))
        elif confidence < float(self.lane_confidence_medium):
            speed = min(speed, float(self.lane_medium_confidence_speed))
        elif confidence < float(self.lane_confidence_high):
            speed = min(speed, float(self.lane_high_confidence_speed))
        return max(float(self.minimum_drive_speed), speed)

    def current_lane_steer(self, now: float) -> float:
        if (
            bool(self.use_external_lane_cmd)
            and now - self.external_lane_time <= float(self.external_cmd_timeout_sec)
            and self.external_lane_confidence > 0.05
        ):
            return self.clamp_angle(self.external_lane_angle)
        return self.lane_steer()

    def set_lane_override(self, target: str) -> None:
        normalized = str(target).strip().lower()
        if normalized in ("reset", "center_line", "middle"):
            normalized = "center"
        if normalized not in ("left", "center", "right"):
            return
        self.current_lane_target = normalized
        if normalized == self.last_lane_override_published:
            return
        self.lane_override_pub.publish(String(data=normalized))
        self.last_lane_override_published = normalized

    def is_green_light(self) -> bool:
        now = time.monotonic()
        if (
            bool(self.use_external_traffic_light)
            and now - self.external_traffic_time <= float(self.external_cmd_timeout_sec)
            and self.external_traffic_color == "green"
        ):
            return True
        if self.latest_traffic_color == "green":
            return True
        for det in self.filter_detections(["green"]):
            if "light" in det.class_name or "signal" in det.class_name or "traffic" in det.class_name:
                return True
        return False

    def intersection_signal_stop_reason(self, now: float) -> Optional[str]:
        if not bool(self.enable_intersection_signal_stop) or not self.started:
            return None
        if self.start_time > 0.0 and now - self.start_time < float(self.intersection_signal_min_start_elapsed_sec):
            return None

        signal = self.latest_intersection_signal
        required_stop = int(self.intersection_signal_required_frames)
        required_go = int(self.intersection_signal_release_frames)

        if signal in ("red", "yellow") and self.intersection_stop_frames >= required_stop:
            return f"{signal}:stop_frames={self.intersection_stop_frames}/{required_stop}"

        if self.intersection_waiting_for_go:
            if signal in ("green", "left_green") and self.intersection_go_frames >= required_go:
                return None
            if signal == "unknown" and not bool(self.intersection_stop_on_unknown_when_waiting):
                return None
            return f"{signal}:waiting_go={self.intersection_go_frames}/{required_go}"

        return None

    def update_start_marker(self, frame: np.ndarray, now: float) -> None:
        if not bool(self.start_require_checkerboard):
            return

        seen = self.detect_start_marker(frame)
        if seen:
            self.start_marker_frames += 1
            self.start_marker_last_seen_time = now
        else:
            self.start_marker_frames = max(0, self.start_marker_frames - 1)

    def start_marker_ready(self, now: float) -> bool:
        if not bool(self.start_require_checkerboard):
            return True
        required = max(1, int(self.start_checkerboard_required_frames))
        if self.start_marker_frames >= required:
            return True
        hold_sec = float(self.start_checkerboard_hold_sec)
        if self.start_marker_last_seen_time <= 0.0 or self.start_marker_frames <= 0:
            return False
        return now - self.start_marker_last_seen_time <= hold_sec

    def detect_start_marker(self, frame: np.ndarray) -> bool:
        min_area = float(self.start_checkerboard_min_area_ratio) * self.image_area()
        for det in self.filter_detections(["checkerboard", "checker", "checkered", "start"]):
            if det.area >= min_area and self.detection_in_lower_image(det, min_y_ratio=0.20):
                return True

        if not bool(self.start_checkerboard_use_image_checker):
            return False
        return self.detect_checker_pattern(frame, 0.50, 0.92, 0.18, 0.82)

    def mark_started(self, now: float) -> None:
        if not self.started:
            self.started = True
            self.start_time = now
            self.last_lap_time = now

    def detect_shortcut_signal(self, frame: np.ndarray) -> bool:
        candidates = self.filter_detections(["left", "shortcut"])
        for det in candidates:
            name = det.class_name.lower()
            is_left_signal = (
                "shortcut" in name
                or "left_turn" in name
                or ("left" in name and ("signal" in name or "turn" in name or "traffic" in name or "light" in name))
            )
            if not is_left_signal:
                continue
            if "green" in name:
                return True
            if "red" in name or "yellow" in name:
                return False
            if self.classify_detection_roi(frame, det) == "green":
                return True
            return bool(self.intersection_left_signal_assume_green)
        return False

    def detect_finish_marker(self, frame: np.ndarray) -> bool:
        if not bool(self.enable_lap_count):
            return False

        min_area = float(self.finish_min_area_ratio) * self.image_area()
        for det in self.filter_detections(["finish", "checker", "checkered", "goal"]):
            if det.area >= min_area:
                return True

        if not bool(self.finish_use_image_checker):
            return False

        return self.detect_checker_pattern(frame, 0.58, 0.92, 0.18, 0.82)

    def detect_checker_pattern(
        self,
        frame: np.ndarray,
        top_ratio: float,
        bottom_ratio: float,
        left_ratio: float,
        right_ratio: float,
    ) -> bool:
        h, w = frame.shape[:2]
        roi = frame[int(h * top_ratio): int(h * bottom_ratio), int(w * left_ratio): int(w * right_ratio)]
        if roi.size == 0:
            return False
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        black = gray < 65
        white = gray > 185
        black_ratio = float(np.count_nonzero(black)) / float(gray.size)
        white_ratio = float(np.count_nonzero(white)) / float(gray.size)
        if black_ratio < 0.08 or white_ratio < 0.08:
            return False

        sampled = gray[:: max(1, gray.shape[0] // 16), :]
        binary = sampled > 128
        transitions = np.count_nonzero(binary[:, 1:] != binary[:, :-1], axis=1)
        mean_transitions = float(np.mean(transitions)) if transitions.size > 0 else 0.0
        return mean_transitions >= 8.0

    def mark_course_checkpoint(self, source: str) -> None:
        """Arm one lap count after completing either the normal or shortcut route."""
        if source not in ("normal", "shortcut") or self.course_checkpoint_pending:
            return
        self.course_checkpoint_pending = source
        if source == "normal":
            self.normal_cone_pass_count += 1
        else:
            self.shortcut_pass_count += 1
        self.get_logger().info(f"Course checkpoint armed by {source} route.")

    def finish_shortcut_if_needed(self, now: float) -> None:
        if not self.shortcut_in_progress or now < self.shortcut_until:
            return
        self.shortcut_in_progress = False
        self.set_lane_override("center")
        self.mark_course_checkpoint("shortcut")

    def route_gate_signal_visible(self, now: float) -> bool:
        external_recent = (
            self.external_traffic_time > 0.0
            and now - self.external_traffic_time <= float(self.detection_timeout_sec)
            and self.external_traffic_color in ("red", "yellow", "green", "left")
        )
        if external_recent or self.latest_shortcut_signal:
            return True
        if self.latest_intersection_signal in ("red", "yellow", "green", "left"):
            return True
        signal_names = (
            "red_light", "yellow_light", "green_light", "left_signal",
            "traffic_light", "traffic_signal",
        )
        return any(self.detection_in_signal_region(det) for det in self.filter_detections(signal_names))

    def update_lap_counter(self, now: float) -> None:
        if not bool(self.enable_lap_count) or not self.started or self.finished:
            return
        if str(self.lap_count_source).lower().strip() != "route_signal":
            return

        signal_visible = self.route_gate_signal_visible(now)
        if not signal_visible:
            self.route_gate_frames = 0
            self.route_gate_clear_frames += 1
            if self.route_gate_clear_frames >= int(self.route_gate_clear_required_frames):
                self.route_gate_armed = True
            return

        self.route_gate_clear_frames = 0
        self.route_gate_frames += 1
        if self.route_gate_frames < int(self.route_gate_required_frames):
            return

        self.route_gate_recent_until = now + float(self.route_gate_recent_sec)
        if not self.route_gate_armed or not self.course_checkpoint_pending:
            return
        if now - self.last_lap_time < float(self.route_gate_min_interval_sec):
            return

        source = self.course_checkpoint_pending
        self.course_checkpoint_pending = ""
        self.route_gate_armed = False
        self.route_gate_frames = 0
        total_laps = max(1, int(self.total_laps))
        if self.lap_count < total_laps:
            self.lap_count += 1
            self.last_lap_time = now
            self.get_logger().info(
                f"Route gate passed after {source} route. lap={self.lap_count}/{total_laps} "
                f"normal={self.normal_cone_pass_count} shortcut={self.shortcut_pass_count}"
            )
        if self.lap_count >= total_laps and bool(self.finish_stop_after_laps):
            self.finished = True

    def obstacle_command(self, now: float, front_distance: float) -> Optional[Tuple[float, float, str, str]]:
        obstacle = self.estimate_obstacle_state(front_distance, now)
        if obstacle.detected:
            self.obstacle_last_seen_time = now
            self.obstacle_last_estimate = obstacle
        elif self.obstacle_drive_state in ("TRACK_CONFIRM", "FOLLOW_GAP", "PREPARE_PASS"):
            obstacle = self.held_obstacle_estimate(now)
        self.latest_obstacle = obstacle

        if self.obstacle_drive_state == "COOLDOWN":
            self.set_lane_override("center")
            if now < self.overtake_cooldown_until:
                return None
            self.reset_dynamic_obstacle_state(clear_distance_track=False)

        if not obstacle.detected:
            if self.obstacle_drive_state == "OVERTAKING":
                return self.overtake_command(now, obstacle)
            self.reset_dynamic_obstacle_state(clear_distance_track=True)
            self.set_lane_override("center")
            return None

        new_detection_frame = self.detection_generation != self.obstacle_last_confirm_generation
        if new_detection_frame:
            self.obstacle_last_confirm_generation = self.detection_generation
            if obstacle.sensor_confirmed:
                self.obstacle_confirm_frames = obstacle.lane_stable_frames
            else:
                self.obstacle_confirm_frames = 0

        if self.obstacle_drive_state == "OVERTAKING":
            return self.overtake_command(now, obstacle)

        dangerous_ttc = obstacle.ttc_sec <= float(self.overtake_emergency_ttc_sec)
        if obstacle.distance_m < float(self.emergency_stop_distance_m) or dangerous_ttc:
            self.obstacle_drive_state = "FOLLOW_GAP"
            self.set_lane_override("center")
            return (
                0.0,
                float(self.stop_speed),
                "OBSTACLE_STOP",
                f"{obstacle.source}:{obstacle.distance_m:.2f}m:ttc={obstacle.ttc_sec:.2f}s",
            )

        is_curve = self.is_curve_driving()
        if self.obstacle_drive_state == "CENTER_DRIVING":
            self.obstacle_drive_state = "TRACK_CONFIRM"
            self.obstacle_state_started_at = now

        confirmed = (
            obstacle.sensor_confirmed
            and obstacle.lane_state in ("left", "right")
            and self.obstacle_confirm_frames >= int(self.overtake_confirm_frames)
        )
        if self.obstacle_drive_state == "TRACK_CONFIRM":
            if confirmed:
                self.obstacle_drive_state = "FOLLOW_GAP"
                self.obstacle_state_started_at = now
            return self.follow_obstacle_command(now, obstacle, is_curve)

        if self.obstacle_drive_state == "FOLLOW_GAP":
            if self.overtake_candidate_ready(obstacle, is_curve, now):
                self.obstacle_drive_state = "PREPARE_PASS"
                self.obstacle_state_started_at = now
                self.obstacle_prepare_clear_since = 0.0
            return self.follow_obstacle_command(now, obstacle, is_curve)

        if self.obstacle_drive_state == "PREPARE_PASS":
            if not self.overtake_candidate_ready(obstacle, is_curve, now):
                self.obstacle_drive_state = "FOLLOW_GAP"
                self.obstacle_prepare_clear_since = 0.0
                return self.follow_obstacle_command(now, obstacle, is_curve)
            direction = self.overtake_direction_for_obstacle(obstacle)
            if direction is None or not self.overtake_side_is_clear(direction, now):
                self.obstacle_prepare_clear_since = 0.0
                return self.follow_obstacle_command(now, obstacle, is_curve)
            if self.obstacle_prepare_clear_since <= 0.0:
                self.obstacle_prepare_clear_since = now
            if now - self.obstacle_prepare_clear_since >= float(self.overtake_prepare_clear_hold_sec):
                return self.start_overtake(now, obstacle)
            return self.follow_obstacle_command(now, obstacle, is_curve)

        self.reset_dynamic_obstacle_state(clear_distance_track=False)
        return None

    def held_obstacle_estimate(self, now: float) -> ObstacleEstimate:
        age = now - float(self.obstacle_last_seen_time)
        if age < 0.0 or age > float(self.overtake_detection_grace_sec):
            return ObstacleEstimate()
        previous = self.obstacle_last_estimate
        if not previous.detected:
            return ObstacleEstimate()
        predicted_distance = max(
            0.0,
            float(previous.distance_m) - max(0.0, float(previous.closing_speed_mps)) * age,
        )
        return replace(
            previous,
            distance_m=predicted_distance,
            source=f"{previous.source}+hold",
            ttc_sec=obstacle_ttc(predicted_distance, previous.closing_speed_mps),
        )

    def reset_dynamic_obstacle_state(self, clear_distance_track: bool = True) -> None:
        self.obstacle_drive_state = "CENTER_DRIVING"
        self.overtake_passing_obstacle = False
        self.overtake_clear_since = 0.0
        self.overtake_phase = "IDLE"
        self.overtake_phase_started_at = 0.0
        self.obstacle_confirm_frames = 0
        self.obstacle_prepare_clear_since = 0.0
        self.obstacle_lane_candidate = "unknown"
        self.obstacle_lane_candidate_frames = 0
        if clear_distance_track:
            self.obstacle_distance_track_time = 0.0
            self.obstacle_distance_track_m = float("inf")
            self.obstacle_closing_speed_mps = 0.0
            self.obstacle_distance_track_samples = 0

    def estimate_obstacle_state(self, front_distance: float, now: Optional[float] = None) -> ObstacleEstimate:
        now = time.monotonic() if now is None else float(now)
        detections = [
            det
            for det in self.filter_detections(("obstacle_vehicle",))
            if self.normalize_class_name(det.class_name) == "obstacle_vehicle"
            and det.score >= float(self.overtake_vehicle_min_score)
        ]
        min_area = float(self.overtake_min_area_ratio) * self.image_area()
        candidates = []

        scan_fresh = (
            float(getattr(self, "last_scan_time", 0.0)) > 0.0
            and now - float(getattr(self, "last_scan_time", 0.0))
            <= float(getattr(self, "scan_timeout_sec", 0.8))
        )

        for det in detections:
            if det.area < min_area:
                continue
            position = self.classify_obstacle_position(det)
            distance = self.estimate_detection_distance(det) if scan_fresh else float("inf")
            source = "yolo+lidar" if math.isfinite(distance) else "yolo"
            sensor_confirmed = math.isfinite(distance)
            if not math.isfinite(distance):
                if scan_fresh and self.detection_in_drive_corridor(det) and math.isfinite(front_distance):
                    distance = front_distance
                    source = "yolo+front_lidar"
                    sensor_confirmed = True
                else:
                    distance = self.estimate_distance_from_bbox(det)
                    source = "yolo_bbox"
            candidates.append((distance, position, source, sensor_confirmed, det))

        if candidates:
            candidates.sort(key=lambda item: item[0])
            distance, position, source, sensor_confirmed, detection = candidates[0]
            new_measurement = self.detection_generation != self.obstacle_lane_track_generation
            if new_measurement:
                final_position = self.smooth_obstacle_position(position)
                lane_state, lateral_ratio, lateral_velocity, stable_frames = self.update_obstacle_lane_track(
                    detection, now
                )
                self.obstacle_lane_track_generation = self.detection_generation
                self.obstacle_lane_track_state = lane_state
                self.obstacle_lane_track_ratio = lateral_ratio
                self.obstacle_lane_track_velocity = lateral_velocity
                self.obstacle_lane_track_frames = stable_frames
            else:
                final_position = self.last_obstacle_position
                lane_state = self.obstacle_lane_track_state
                lateral_ratio = self.obstacle_lane_track_ratio
                lateral_velocity = self.obstacle_lane_track_velocity
                stable_frames = self.obstacle_lane_track_frames
            second_distance = candidates[1][0] if len(candidates) > 1 else float("inf")
            closing_speed, estimated_speed_cmd, ttc = self.update_obstacle_longitudinal_track(
                distance,
                now,
                new_measurement and sensor_confirmed,
            )
            return ObstacleEstimate(
                detected=distance < float(self.overtake_detection_range_m),
                distance_m=distance,
                second_distance_m=second_distance,
                position=final_position,
                vehicle_count=len(candidates),
                source=source,
                lane_state=lane_state,
                lateral_ratio=lateral_ratio,
                lateral_velocity_ratio_s=lateral_velocity,
                lane_stable_frames=stable_frames,
                sensor_confirmed=sensor_confirmed,
                closing_speed_mps=closing_speed,
                estimated_speed_cmd=estimated_speed_cmd,
                ttc_sec=ttc,
            )

        if not bool(self.overtake_require_vehicle_detection) and front_distance < float(self.obstacle_distance_m):
            return ObstacleEstimate(
                detected=True,
                distance_m=front_distance,
                position="center",
                vehicle_count=1,
                source="front_lidar",
                sensor_confirmed=True,
            )
        return ObstacleEstimate()

    def update_obstacle_longitudinal_track(
        self,
        distance_m: float,
        now: float,
        new_measurement: bool,
    ) -> Tuple[float, float, float]:
        expected_speed = float(self.overtake_expected_vehicle_speed_cmd)
        scale = max(1e-4, float(self.overtake_speed_to_mps_scale))
        if not new_measurement or not math.isfinite(distance_m):
            closing = float(self.obstacle_closing_speed_mps)
            estimated = expected_speed
            if self.obstacle_distance_track_time > 0.0:
                ego_mps = max(0.0, float(getattr(self, "last_speed", 0.0)) * scale)
                estimated = max(0.0, (ego_mps - closing) / scale)
            return closing, estimated, obstacle_ttc(distance_m, closing)

        closing = 0.0
        had_previous = self.obstacle_distance_track_time > 0.0 and math.isfinite(self.obstacle_distance_track_m)
        if had_previous:
            dt = max(0.02, now - self.obstacle_distance_track_time)
            range_jump = self.obstacle_distance_track_m - float(distance_m)
            if abs(range_jump) <= float(self.overtake_max_range_jump_m):
                measured = range_jump / dt
                maximum = max(0.1, float(self.overtake_max_closing_speed_mps))
                measured = float(np.clip(measured, -maximum, maximum))
                alpha = float(np.clip(self.overtake_relative_speed_alpha, 0.0, 1.0))
                if int(getattr(self, "obstacle_distance_track_samples", 0)) <= 1:
                    closing = measured
                else:
                    closing = alpha * measured + (1.0 - alpha) * self.obstacle_closing_speed_mps
        self.obstacle_distance_track_time = now
        self.obstacle_distance_track_m = float(distance_m)
        self.obstacle_closing_speed_mps = closing
        self.obstacle_distance_track_samples = int(
            getattr(self, "obstacle_distance_track_samples", 0)
        ) + 1

        if had_previous:
            ego_mps = max(0.0, float(getattr(self, "last_speed", 0.0)) * scale)
            estimated_speed = max(0.0, (ego_mps - closing) / scale)
        else:
            estimated_speed = expected_speed
        estimated_speed = float(np.clip(estimated_speed, 0.0, float(self.overtake_pass_max_speed)))
        return closing, estimated_speed, obstacle_ttc(distance_m, closing)

    def estimate_detection_distance(self, det: ObjectDetection) -> float:
        if self.latest_scan_points.size == 0 or self.image_width <= 0:
            return float("inf")

        if bool(self.use_lidar_camera_projection):
            projected_distance = self.estimate_projected_detection_distance(det)
            if math.isfinite(projected_distance):
                return projected_distance

        half_hfov = float(self.overtake_lidar_match_hfov_deg) * 0.5
        center_offset = (det.cx - self.image_width * 0.5) / max(1.0, self.image_width * 0.5)
        width_ratio = det.width / max(1.0, float(self.image_width))
        center_angle = -center_offset * half_hfov
        half_width_angle = max(2.0, width_ratio * float(self.overtake_lidar_match_hfov_deg) * 0.5)
        padding = float(self.overtake_lidar_match_padding_deg)

        min_angle = center_angle - half_width_angle - padding
        max_angle = center_angle + half_width_angle + padding
        points = self.latest_scan_points
        distance = np.linalg.norm(points[:, 0:2], axis=1)
        mask = (
            (points[:, 0] > 0.0)
            & (points[:, 2] >= min_angle)
            & (points[:, 2] <= max_angle)
            & (distance < float(self.overtake_detection_range_m) + 0.7)
        )
        if np.count_nonzero(mask) < int(self.overtake_lidar_match_min_points):
            return float("inf")
        return float(np.percentile(distance[mask], 20.0))

    def estimate_projected_detection_distance(self, det: ObjectDetection) -> float:
        if (
            self.camera_matrix is None
            or self.lidar_camera_rotation is None
            or self.lidar_camera_translation is None
            or self.latest_scan_points.size == 0
        ):
            return float("inf")

        points = self.latest_scan_points
        points_lidar = np.column_stack(
            (points[:, 0], points[:, 1], np.zeros(points.shape[0], dtype=np.float32))
        ).astype(np.float64)
        points_camera = (
            self.lidar_camera_rotation @ points_lidar.T
        ).T + self.lidar_camera_translation
        depth = points_camera[:, 2]
        valid_depth = depth > 0.05
        if not np.any(valid_depth):
            return float("inf")

        points_camera = points_camera[valid_depth]
        lidar_ranges = np.linalg.norm(points_lidar[valid_depth, :2], axis=1)
        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])
        u = fx * points_camera[:, 0] / points_camera[:, 2] + cx
        v = fy * points_camera[:, 1] / points_camera[:, 2] + cy

        padding = float(self.lidar_projection_bbox_padding_px)
        x1 = det.cx - det.width * 0.5 - padding
        x2 = det.cx + det.width * 0.5 + padding
        y1 = det.cy - det.height * 0.5 - padding
        y2 = det.cy + det.height * 0.5 + padding
        in_box = (
            (u >= x1)
            & (u <= x2)
            & (v >= y1)
            & (v <= y2)
            & (lidar_ranges <= float(self.overtake_detection_range_m) + 0.7)
        )
        ranges = np.sort(lidar_ranges[in_box])
        min_points = int(self.overtake_lidar_match_min_points)
        if ranges.size < min_points:
            return float("inf")

        gap = float(self.lidar_projection_cluster_gap_m)
        split_indices = np.where(np.diff(ranges) > gap)[0] + 1
        clusters = [cluster for cluster in np.split(ranges, split_indices) if cluster.size >= min_points]
        if not clusters:
            return float("inf")
        closest = min(clusters, key=lambda cluster: float(np.mean(cluster)))
        return float(np.mean(closest))

    def estimate_distance_from_bbox(self, det: ObjectDetection) -> float:
        area_ratio = det.area / max(1.0, self.image_area())
        if area_ratio <= 0.0:
            return float("inf")
        reference_area = max(float(self.overtake_min_area_ratio), 0.01)
        distance = float(self.overtake_detection_range_m) * math.sqrt(reference_area / area_ratio)
        return float(np.clip(distance, 0.55, float(self.overtake_detection_range_m)))

    def classify_obstacle_position(self, det: ObjectDetection) -> str:
        if self.image_width <= 0:
            return "unknown"
        reference_x = self.path_reference_x_at_y(det.cy)
        deadband = self.image_width * 0.04
        if det.cx < reference_x - deadband:
            return "left"
        if det.cx > reference_x + deadband:
            return "right"
        return "center"

    def update_obstacle_lane_track(self, det: ObjectDetection, now: float) -> Tuple[str, float, float, int]:
        if self.image_width <= 0:
            return "unknown", 0.0, 0.0, 0
        reference_x = self.path_reference_x_at_y(det.cy)
        ratio = (det.cx - reference_x) / max(1.0, float(self.image_width))
        velocity = 0.0
        if self.obstacle_track_time > 0.0:
            dt = max(0.02, now - self.obstacle_track_time)
            measured_velocity = (ratio - self.obstacle_track_ratio) / dt
            alpha = float(self.overtake_lateral_velocity_alpha)
            velocity = alpha * measured_velocity + (1.0 - alpha) * self.obstacle_track_velocity
        self.obstacle_track_time = now
        self.obstacle_track_ratio = ratio
        self.obstacle_track_velocity = velocity

        deadband = float(self.overtake_lane_deadband_ratio)
        raw_lane = "left" if ratio < -deadband else "right" if ratio > deadband else "center"
        moving = abs(velocity) >= float(self.overtake_lateral_motion_threshold_ratio_per_sec)
        if moving or raw_lane == "center":
            self.obstacle_lane_candidate = "unknown"
            self.obstacle_lane_candidate_frames = 0
            return "transitioning", float(ratio), float(velocity), 0

        if raw_lane == self.obstacle_lane_candidate:
            self.obstacle_lane_candidate_frames += 1
        else:
            self.obstacle_lane_candidate = raw_lane
            self.obstacle_lane_candidate_frames = 1
        stable_frames = self.obstacle_lane_candidate_frames
        if stable_frames < int(self.overtake_lane_stable_frames):
            return "unstable", float(ratio), float(velocity), stable_frames
        return raw_lane, float(ratio), float(velocity), stable_frames

    def path_reference_x_at_y(self, image_y: float) -> float:
        if (
            self.center_curve_points is not None
            and time.monotonic() - self.center_curve_time <= float(self.center_curve_timeout_sec)
        ):
            points = self.center_curve_points
            order = np.argsort(points[:, 1])
            ys = points[order, 1]
            xs = points[order, 0]
            unique_ys, unique_indices = np.unique(ys, return_index=True)
            unique_xs = xs[unique_indices]
            if unique_ys.size >= 2:
                return float(np.interp(float(image_y), unique_ys, unique_xs))
        if self.latest_lane.valid:
            return float(self.latest_lane.target_x)
        return float(self.image_width) * 0.5

    def smooth_obstacle_position(self, position: str) -> str:
        if position == "unknown":
            return self.last_obstacle_position
        numeric = {"left": -1.0, "center": 0.0, "right": 1.0}.get(position, 0.0)
        alpha = float(self.overtake_position_smooth_alpha)
        self.smoothed_obstacle_position = alpha * numeric + (1.0 - alpha) * self.smoothed_obstacle_position
        threshold = float(self.overtake_position_threshold)
        if self.smoothed_obstacle_position < -threshold:
            self.last_obstacle_position = "left"
        elif self.smoothed_obstacle_position > threshold:
            self.last_obstacle_position = "right"
        else:
            self.last_obstacle_position = "center"
        return self.last_obstacle_position

    def should_follow_obstacle(self, obstacle: ObstacleEstimate, is_curve: bool) -> bool:
        del is_curve
        return obstacle.detected

    def dynamic_lane_confidence(self, now: float) -> float:
        confidence = float(getattr(getattr(self, "latest_lane", None), "confidence", 0.0))
        if (
            now - float(getattr(self, "external_lane_time", 0.0))
            <= float(getattr(self, "external_cmd_timeout_sec", 0.5))
        ):
            confidence = max(confidence, float(getattr(self, "external_lane_confidence", 0.0)))
        return confidence

    def overtake_candidate_ready(self, obstacle: ObstacleEstimate, is_curve: bool, now: float) -> bool:
        if not obstacle.sensor_confirmed or obstacle.source.endswith("+hold"):
            return False
        last_detection_time = float(getattr(self, "last_detection_time", now))
        if now - last_detection_time > float(self.overtake_detection_grace_sec):
            return False
        if obstacle.lane_state not in ("left", "right"):
            return False
        confirmed_frames = max(
            int(obstacle.lane_stable_frames),
            int(getattr(self, "obstacle_confirm_frames", 0)),
        )
        if confirmed_frames < int(self.overtake_confirm_frames):
            return False
        if obstacle.vehicle_count >= 2 and not bool(self.overtake_allow_multi_vehicle):
            return False
        if obstacle.vehicle_count >= 2:
            gap = obstacle.second_distance_m - obstacle.distance_m
            if not math.isfinite(gap) or gap < float(self.overtake_multi_vehicle_gap_m):
                return False
        if is_curve and not bool(self.overtake_allow_curve_start):
            return False
        if now < float(self.overtake_abort_hold_until):
            return False
        if obstacle.distance_m > float(self.overtake_prepare_distance_m):
            return False
        if obstacle.distance_m < float(self.overtake_min_commit_distance_m):
            return False
        if obstacle.ttc_sec < float(self.overtake_min_ttc_sec):
            return False
        if self.dynamic_lane_confidence(now) < float(self.overtake_min_lane_confidence):
            return False
        direction = self.overtake_direction_for_obstacle(obstacle)
        if direction is None or not self.target_lane_markings_confirmed(direction, now):
            return False
        return True

    def target_lane_markings_confirmed(self, direction: str, now: float) -> bool:
        if not bool(self.overtake_require_target_lane_markings):
            return True
        if now - float(self.external_lane_time) > float(self.external_cmd_timeout_sec):
            return False
        observed = tuple(bool(value) for value in self.external_lane_observed)
        if len(observed) != 3:
            return False
        if direction == "left":
            return observed[0] and observed[1]
        if direction == "right":
            return observed[1] and observed[2]
        return False

    def can_start_overtake(self, obstacle: ObstacleEstimate, is_curve: bool) -> bool:
        now = time.monotonic()
        if not self.overtake_candidate_ready(obstacle, is_curve, now):
            return False
        direction = self.overtake_direction_for_obstacle(obstacle)
        return direction is not None and self.overtake_side_is_clear(direction, now)

    def overtake_direction_for_obstacle(self, obstacle: ObstacleEstimate) -> Optional[str]:
        occupied_lane = obstacle.lane_state
        if occupied_lane == "right":
            return "left"
        if occupied_lane == "left":
            return "right"
        return None

    def start_overtake(self, now: float, obstacle: ObstacleEstimate) -> Tuple[float, float, str, str]:
        direction = self.overtake_direction_for_obstacle(obstacle)
        if direction is None:
            self.obstacle_drive_state = "FOLLOW_GAP"
            return self.follow_obstacle_command(now, obstacle, self.is_curve_driving())
        self.obstacle_drive_state = "OVERTAKING"
        self.overtake_direction = direction
        self.overtake_started_at = now
        self.overtake_until = now + float(self.overtake_duration_sec)
        self.overtake_passing_obstacle = False
        self.overtake_clear_since = 0.0
        self.overtake_phase = "CHANGE_OUT"
        self.overtake_phase_started_at = now
        self.overtake_rejoin_started_at = 0.0
        self.overtake_rejoin_distance_m = 0.0
        self.overtake_rejoin_last_time = 0.0
        self.overtake_cut_in_hold_until = 0.0
        self.obstacle_prepare_clear_since = 0.0
        self.set_lane_override(direction)
        return self.overtake_command(now, obstacle)

    def overtake_command(self, now: float, obstacle: ObstacleEstimate) -> Tuple[float, float, str, str]:
        if self.overtake_phase == "CHANGE_OUT" and self.obstacle_moves_into_overtake_lane(obstacle):
            self.obstacle_drive_state = "FOLLOW_GAP"
            self.overtake_phase = "IDLE"
            self.overtake_abort_hold_until = now + float(self.overtake_abort_hold_sec)
            self.set_lane_override("center")
            return self.follow_obstacle_command(now, obstacle, self.is_curve_driving())

        obstacle_side = "right" if self.overtake_direction == "left" else "left"
        if self.overtake_phase == "PASS" and self.obstacle_moves_into_overtake_lane(obstacle):
            self.overtake_cut_in_hold_until = max(
                float(self.overtake_cut_in_hold_until),
                now + float(self.overtake_cut_in_hold_sec),
            )
        if self.overtake_phase == "PASS" and now < float(self.overtake_cut_in_hold_until):
            self.set_lane_override(self.overtake_direction)
            side_values = self.ultra_values_for_side(obstacle_side)
            side_danger = (
                self.ultra_is_fresh(now)
                and bool(side_values)
                and min(side_values) < float(self.overtake_side_emergency_cm)
            )
            longitudinal_danger = (
                obstacle.distance_m < float(self.overtake_min_commit_distance_m)
                or obstacle.ttc_sec <= float(self.overtake_hard_brake_ttc_sec)
            )
            speed = (
                float(self.stop_speed)
                if side_danger or longitudinal_danger
                else float(self.overtake_timeout_hold_speed)
            )
            reason = (
                f"vehicle_cut_in:lane={obstacle.lane_state}:vy={obstacle.lateral_velocity_ratio_s:+.2f}:"
                f"side_danger={int(side_danger)}:front_danger={int(longitudinal_danger)}"
            )
            return self.current_lane_steer(now), speed, "OVERTAKE_YIELD", reason

        if self.overtake_phase == "CHANGE_OUT":
            self.set_lane_override(self.overtake_direction)
            if self.side_obstacle_detected(obstacle_side, now):
                self.overtake_passing_obstacle = True
                self.overtake_phase = "PASS"
                self.overtake_phase_started_at = now
                self.overtake_clear_since = 0.0
        elif self.overtake_phase == "PASS" and self.side_obstacle_cleared(obstacle_side, now):
            if self.overtake_clear_since <= 0.0:
                self.overtake_clear_since = now
            elif now - self.overtake_clear_since >= float(self.overtake_pass_clear_hold_sec):
                self.begin_overtake_rejoin(now)
        elif self.overtake_phase == "PASS":
            self.overtake_clear_since = 0.0

        elapsed_total = max(0.0, now - self.overtake_started_at)
        no_side_fallback = (
            self.overtake_phase == "CHANGE_OUT"
            and not self.overtake_passing_obstacle
            and not obstacle.detected
            and elapsed_total >= float(self.overtake_no_side_fallback_sec)
            and self.scan_is_fresh(now)
            and self.latest_front_distance > float(self.overtake_front_clear_distance_m)
            and self.side_obstacle_cleared(obstacle_side, now)
        )
        if no_side_fallback:
            self.begin_overtake_rejoin(now)

        if self.overtake_phase == "REJOIN":
            self.set_lane_override("center")
            elapsed = max(0.0, now - self.overtake_rejoin_started_at)
            dt = max(0.0, now - float(self.overtake_rejoin_last_time))
            self.overtake_rejoin_last_time = now
            speed_mps = max(0.0, self.last_speed * float(self.overtake_speed_to_mps_scale))
            self.overtake_rejoin_distance_m += dt * speed_mps
            steer = self.current_lane_steer(now)
            lane_ready = self.dynamic_lane_confidence(now) >= float(
                self.overtake_rejoin_lane_confidence
            )
            complete = elapsed >= float(self.overtake_rejoin_min_sec) and lane_ready
            if complete:
                self.obstacle_drive_state = "COOLDOWN"
                self.overtake_cooldown_until = now + float(self.overtake_cooldown_sec)
                self.overtake_phase = "IDLE"
                self.overtake_passing_obstacle = False
                self.overtake_clear_since = 0.0
                return steer, float(self.overtake_rejoin_speed), "OVERTAKE_REJOIN", "complete"
            deadline = self.overtake_rejoin_distance_m >= float(self.overtake_rejoin_deadline_m)
            lane_confidence = self.dynamic_lane_confidence(now)
            if elapsed >= float(self.overtake_rejoin_hard_timeout_sec) and lane_confidence < float(
                self.overtake_rejoin_lane_confidence
            ):
                return steer, float(self.stop_speed), "OVERTAKE_REJOIN_WAIT", "lane_not_confirmed"
            speed = float(self.minimum_drive_speed) if deadline else float(self.overtake_rejoin_speed)
            reason = (
                f"distance={self.overtake_rejoin_distance_m:.2f}m:conf={lane_confidence:.2f}:"
                f"elapsed={elapsed:.2f}s"
            )
            return steer, speed, "OVERTAKE_REJOIN", reason

        self.set_lane_override(self.overtake_direction)
        direction_sign = -1.0 if self.overtake_direction == "left" else 1.0
        bias_duration = max(0.01, float(self.overtake_lane_change_bias_sec))
        transition_ratio = float(np.clip(1.0 - (now - self.overtake_started_at) / bias_duration, 0.0, 1.0))
        steer = self.current_lane_steer(now) + direction_sign * float(self.overtake_bias_cmd) * transition_ratio
        phase_elapsed = max(0.0, now - self.overtake_phase_started_at)
        if self.overtake_phase == "CHANGE_OUT" and phase_elapsed < float(self.overtake_change_min_sec):
            speed = float(self.overtake_change_speed)
        else:
            speed = self.overtake_pass_speed(obstacle)
        timed_out = now > self.overtake_until
        if timed_out:
            # A timeout may reduce speed, but can never authorize cutting back
            # in front of a vehicle whose rear has not been observed clear.
            speed = min(speed, float(self.overtake_timeout_hold_speed))
        reason = (
            f"phase={self.overtake_phase}:lane={self.overtake_direction}:"
            f"vehicle={obstacle.lane_state}:vy={obstacle.lateral_velocity_ratio_s:+.2f}:"
            f"distance={obstacle.distance_m:.2f}m:ttc={obstacle.ttc_sec:.2f}:"
            f"timeout={int(timed_out)}"
        )
        return self.clamp_angle(steer), speed, "OVERTAKE", reason

    def overtake_pass_speed(self, obstacle: ObstacleEstimate) -> float:
        vehicle_speed = float(obstacle.estimated_speed_cmd)
        if vehicle_speed <= 0.0 or not math.isfinite(vehicle_speed):
            vehicle_speed = float(self.overtake_expected_vehicle_speed_cmd)
        requested = vehicle_speed + float(self.overtake_pass_speed_margin_cmd)
        return float(
            np.clip(
                requested,
                float(self.overtake_pass_min_speed),
                float(self.overtake_pass_max_speed),
            )
        )

    def obstacle_moves_into_overtake_lane(self, obstacle: ObstacleEstimate) -> bool:
        if not obstacle.detected:
            return False
        if obstacle.lane_state == self.overtake_direction:
            return True
        threshold = 0.5 * float(self.overtake_lateral_motion_threshold_ratio_per_sec)
        if obstacle.lane_state != "transitioning":
            return False
        if self.overtake_direction == "left":
            return obstacle.lateral_velocity_ratio_s < -threshold
        return obstacle.lateral_velocity_ratio_s > threshold

    def begin_overtake_rejoin(self, now: float) -> None:
        self.overtake_phase = "REJOIN"
        self.overtake_phase_started_at = now
        self.overtake_rejoin_started_at = now
        self.overtake_rejoin_distance_m = 0.0
        self.overtake_rejoin_last_time = now
        self.overtake_clear_since = 0.0
        self.set_lane_override("center")

    def follow_obstacle_command(self, now: float, obstacle: ObstacleEstimate, is_curve: bool) -> Tuple[float, float, str, str]:
        self.set_lane_override("center")
        current_speed_mps = max(0.1, self.last_speed * float(self.overtake_speed_to_mps_scale))
        target_gap = (
            float(self.overtake_follow_standstill_gap_m)
            + float(self.overtake_target_time_gap_sec) * current_speed_mps
        )
        if obstacle.closing_speed_mps > 0.0:
            target_gap += obstacle.closing_speed_mps ** 2 / 1.6
        gap_error = obstacle.distance_m - target_gap
        vehicle_speed = float(obstacle.estimated_speed_cmd)
        if vehicle_speed <= 0.0 or not math.isfinite(vehicle_speed):
            vehicle_speed = float(self.overtake_expected_vehicle_speed_cmd)
        target_speed = (
            vehicle_speed
            + float(self.overtake_follow_kp) * gap_error
            - float(self.overtake_follow_closing_gain) * max(0.0, obstacle.closing_speed_mps)
        )
        speed_limit = float(self.slow_speed) if is_curve else float(self.base_speed)
        if obstacle.distance_m > float(self.overtake_prepare_distance_m) and not is_curve:
            target_speed = speed_limit
        if obstacle.ttc_sec <= float(self.overtake_hard_brake_ttc_sec):
            target_speed = min(target_speed, float(self.overtake_min_follow_speed))
        speed = float(np.clip(target_speed, float(self.overtake_min_follow_speed), speed_limit))
        steer = self.current_lane_steer(now)
        reason = (
            f"{obstacle.source}:{obstacle.distance_m:.2f}m:{obstacle.vehicle_count}v:"
            f"closing={obstacle.closing_speed_mps:+.2f}mps:ttc={obstacle.ttc_sec:.2f}s"
        )
        return steer, speed, "FOLLOW_GAP", reason

    def is_curve_driving(self) -> bool:
        lane_angle = self.external_lane_angle if self.external_lane_confidence > 0.05 else self.lane_steer()
        return abs(lane_angle) > float(self.overtake_curve_steer_threshold_cmd)

    def scan_is_fresh(self, now: float) -> bool:
        return (
            float(getattr(self, "last_scan_time", 0.0)) > 0.0
            and now - float(self.last_scan_time) <= float(self.scan_timeout_sec)
        )

    def ultra_is_fresh(self, now: float) -> bool:
        return (
            float(getattr(self, "last_ultra_time", 0.0)) > 0.0
            and now - float(self.last_ultra_time) <= float(self.ultra_timeout_sec)
        )

    def overtake_side_is_clear(self, side: str, now: float) -> bool:
        if not self.scan_is_fresh(now):
            return False
        scan_dist = self.latest_left_distance if side == "left" else self.latest_right_distance
        if not math.isfinite(scan_dist) or scan_dist <= float(self.side_clearance_m):
            return False
        values = self.ultra_values_for_side(side)
        if bool(self.overtake_require_fresh_ultra):
            if not self.ultra_is_fresh(now) or not values:
                return False
        return not values or min(values) > float(self.ultra_side_clearance_cm)

    def side_obstacle_detected(self, side: str, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else float(now)
        values = self.ultra_values_for_side(side)
        ultra_detected = (
            self.ultra_is_fresh(now)
            and bool(values)
            and min(values) < float(self.overtake_pass_side_detect_cm)
        )
        scan_dist = self.latest_left_distance if side == "left" else self.latest_right_distance
        scan_detected = (
            self.scan_is_fresh(now)
            and math.isfinite(scan_dist)
            and scan_dist < float(self.side_clearance_m)
        )
        return ultra_detected or scan_detected

    def side_obstacle_cleared(self, side: str, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else float(now)
        if not self.scan_is_fresh(now):
            return False
        scan_dist = self.latest_left_distance if side == "left" else self.latest_right_distance
        scan_cleared = math.isfinite(scan_dist) and scan_dist > float(self.side_clearance_m)
        if not scan_cleared:
            return False
        values = self.ultra_values_for_side(side)
        if bool(self.overtake_require_fresh_ultra):
            return (
                self.ultra_is_fresh(now)
                and bool(values)
                and min(values) > float(self.overtake_pass_side_clear_cm)
            )
        return not values or min(values) > float(self.overtake_pass_side_clear_cm)

    def ultra_values_for_side(self, side: str) -> List[float]:
        index = int(self.ultra_left_index if side == "left" else self.ultra_right_index)
        back_index = int(self.ultra_left_back_index if side == "left" else self.ultra_right_back_index)
        values = []
        for idx in (index, back_index):
            if 0 <= idx < len(self.latest_ultra):
                value = self.latest_ultra[idx]
                if value > 0:
                    values.append(float(value))
        return values

    def fixed_obstacle_command(self, now: float, front_distance: float) -> Optional[Tuple[float, float, str, str]]:
        if not bool(self.enable_fixed_obstacle_handling):
            self.reset_fixed_obstacle_state()
            return None

        obstacle = self.estimate_fixed_obstacle(front_distance)
        self.latest_fixed_obstacle = obstacle
        self.latest_pedestrian = obstacle
        new_frame = self.detection_generation != self.fixed_obstacle_last_detection_generation
        if new_frame:
            self.fixed_obstacle_last_detection_generation = self.detection_generation

        if self.fixed_obstacle_state == "COOLDOWN":
            self.set_lane_override("center")
            if now < self.fixed_obstacle_cooldown_until:
                return None
            self.reset_fixed_obstacle_state()

        confirmed = (
            obstacle.detected
            and obstacle.in_path
            and obstacle.close
            and (
                obstacle.sensor_confirmed
                or not bool(self.fixed_obstacle_require_lidar_confirmation)
            )
        )

        if self.fixed_obstacle_state == "CLEAR":
            if confirmed and new_frame:
                self.fixed_obstacle_frames += 1
            elif new_frame:
                self.fixed_obstacle_frames = 0
            if self.fixed_obstacle_frames < int(self.fixed_obstacle_required_frames):
                return None
            self.fixed_obstacle_state = "WAIT_CLEAR_SIDE"
            self.fixed_obstacle_direction_candidate = ""
            self.fixed_obstacle_direction_clear_since = 0.0

        if self.fixed_obstacle_state == "WAIT_CLEAR_SIDE":
            self.set_lane_override("center")
            direction = self.fixed_obstacle_avoid_direction(obstacle)
            if direction is None:
                self.fixed_obstacle_direction_candidate = ""
                self.fixed_obstacle_direction_clear_since = 0.0
                return 0.0, float(self.stop_speed), "FIXED_OBSTACLE_WAIT", "both_sides_blocked"

            if direction != self.fixed_obstacle_direction_candidate:
                self.fixed_obstacle_direction_candidate = direction
                self.fixed_obstacle_direction_clear_since = now
            side_clear_elapsed = now - self.fixed_obstacle_direction_clear_since
            if side_clear_elapsed < float(self.fixed_obstacle_direction_clear_hold_sec):
                reason = f"verify_{direction}={side_clear_elapsed:.2f}s:{obstacle.reason}"
                return 0.0, float(self.stop_speed), "FIXED_OBSTACLE_WAIT", reason

            self.fixed_obstacle_direction = direction
            self.fixed_obstacle_state = "CHANGE_OUT"
            self.fixed_obstacle_started_at = now
            self.fixed_obstacle_phase_started_at = now
            self.fixed_obstacle_clear_frames = 0
            self.fixed_obstacle_pass_detected = False
            self.fixed_obstacle_clear_since = 0.0

        if self.fixed_obstacle_state == "CHANGE_OUT":
            self.set_lane_override(self.fixed_obstacle_direction)
            elapsed = now - self.fixed_obstacle_started_at
            phase_elapsed = now - self.fixed_obstacle_phase_started_at
            obstacle_side = "right" if self.fixed_obstacle_direction == "left" else "left"
            if self.fixed_obstacle_side_detected(obstacle_side):
                self.fixed_obstacle_pass_detected = True
                self.fixed_obstacle_state = "PASS"
                self.fixed_obstacle_phase_started_at = now
            elif phase_elapsed >= float(self.fixed_obstacle_change_min_sec):
                self.fixed_obstacle_state = "PASS"
                self.fixed_obstacle_phase_started_at = now

            sign = -1.0 if self.fixed_obstacle_direction == "left" else 1.0
            bias_sec = max(0.01, float(self.fixed_obstacle_bias_sec))
            bias_ratio = float(np.clip(1.0 - elapsed / bias_sec, 0.0, 1.0))
            steer = self.current_lane_steer(now) + sign * float(self.fixed_obstacle_bias_cmd) * bias_ratio
            reason = f"phase=change_out:lane={self.fixed_obstacle_direction}:{obstacle.reason}"
            return self.clamp_angle(steer), float(self.fixed_obstacle_speed), "FIXED_OBSTACLE_AVOID", reason

        if self.fixed_obstacle_state == "PASS":
            self.set_lane_override(self.fixed_obstacle_direction)
            elapsed = now - self.fixed_obstacle_started_at
            obstacle_side = "right" if self.fixed_obstacle_direction == "left" else "left"
            side_detected = self.fixed_obstacle_side_detected(obstacle_side)
            side_cleared = self.fixed_obstacle_side_cleared(obstacle_side)
            if side_detected:
                self.fixed_obstacle_pass_detected = True
                self.fixed_obstacle_clear_since = 0.0

            if new_frame:
                if obstacle.detected:
                    self.fixed_obstacle_clear_frames = 0
                else:
                    self.fixed_obstacle_clear_frames += 1

            no_longer_in_path = not (obstacle.detected and obstacle.in_path)
            if self.fixed_obstacle_pass_detected and side_cleared and no_longer_in_path:
                if self.fixed_obstacle_clear_since <= 0.0:
                    self.fixed_obstacle_clear_since = now
            elif not side_detected:
                self.fixed_obstacle_clear_since = 0.0

            side_clear_held = (
                self.fixed_obstacle_clear_since > 0.0
                and now - self.fixed_obstacle_clear_since
                >= float(self.fixed_obstacle_side_clear_hold_sec)
            )
            primary_pass_complete = (
                elapsed >= float(self.fixed_obstacle_min_avoid_sec)
                and self.fixed_obstacle_pass_detected
                and side_clear_held
            )
            visual_clear = (
                self.fixed_obstacle_clear_frames
                >= int(self.fixed_obstacle_clear_required_frames)
            )
            # The degraded visual fallback is allowed only with a fresh,
            # finite front-LiDAR clearance. Sensor timeout must never be
            # interpreted as an empty road.
            front_clear = (
                math.isfinite(front_distance)
                and front_distance > float(self.fixed_obstacle_front_clear_distance_m)
            )
            fallback_pass_complete = (
                elapsed >= float(self.fixed_obstacle_no_side_fallback_sec)
                and visual_clear
                and front_clear
            )

            if primary_pass_complete or fallback_pass_complete:
                self.fixed_obstacle_state = "REJOIN"
                self.fixed_obstacle_rejoin_started_at = now
                self.set_lane_override("center")
            else:
                steer = self.current_lane_steer(now)
                reason = (
                    f"phase=pass:lane={self.fixed_obstacle_direction}:"
                    f"side_seen={int(self.fixed_obstacle_pass_detected)}:"
                    f"side_clear={int(side_clear_held)}:visual_clear={self.fixed_obstacle_clear_frames}:"
                    f"{obstacle.reason}"
                )
                return self.clamp_angle(steer), float(self.fixed_obstacle_speed), "FIXED_OBSTACLE_AVOID", reason

        if self.fixed_obstacle_state == "REJOIN":
            self.set_lane_override("center")
            elapsed = now - self.fixed_obstacle_rejoin_started_at
            lane_confidence = self.fixed_obstacle_lane_confidence(now)
            rejoin_ready = (
                elapsed >= float(self.fixed_obstacle_rejoin_sec)
                and lane_confidence >= float(self.fixed_obstacle_rejoin_lane_confidence)
            )
            rejoin_timed_out = elapsed >= float(self.fixed_obstacle_rejoin_max_sec)
            if rejoin_ready or rejoin_timed_out:
                self.reset_fixed_obstacle_state()
                self.fixed_obstacle_state = "COOLDOWN"
                self.fixed_obstacle_cooldown_until = now + float(self.fixed_obstacle_cooldown_sec)
                reason = f"complete:conf={lane_confidence:.2f}:timeout={int(rejoin_timed_out)}"
                return self.current_lane_steer(now), float(self.fixed_obstacle_speed), "FIXED_OBSTACLE_REJOIN", reason
            reason = f"t={elapsed:.2f}s:conf={lane_confidence:.2f}"
            return self.current_lane_steer(now), float(self.fixed_obstacle_speed), "FIXED_OBSTACLE_REJOIN", reason

        return None

    def estimate_fixed_obstacle(self, front_distance: float) -> FixedObstacleEstimate:
        # Dataset contract (ID 6): only the competition mission class may
        # activate this maneuver. Generic people or vehicles outside the
        # course must remain hard negatives and must not trigger avoidance.
        detections = [
            det
            for det in self.filter_detections(("fixed_obstacle",))
            if self.normalize_class_name(det.class_name) == "fixed_obstacle"
        ]
        min_area = float(self.fixed_obstacle_min_area_ratio) * self.image_area()
        detections = [det for det in detections if det.area >= min_area]
        if not detections:
            return FixedObstacleEstimate()

        det = max(detections, key=lambda item: item.area)
        reference_x = self.path_reference_x_at_y(det.cy)
        corridor_half = max(1.0, self.image_width * float(self.fixed_obstacle_corridor_half_width_ratio))
        x1 = det.cx - det.width * 0.5
        x2 = det.cx + det.width * 0.5
        overlaps_path = x2 >= reference_x - corridor_half and x1 <= reference_x + corridor_half
        low_enough = self.image_height <= 0 or det.cy >= self.image_height * float(self.fixed_obstacle_min_y_ratio)
        position_ratio = (det.cx - reference_x) / corridor_half
        if position_ratio < -0.20:
            position = "left"
        elif position_ratio > 0.20:
            position = "right"
        else:
            position = "center"

        scan_fresh = True
        if hasattr(self, "last_scan_time") and hasattr(self, "scan_timeout_sec"):
            scan_fresh = (
                time.monotonic() - float(self.last_scan_time)
                <= float(self.scan_timeout_sec)
            )
        distance = self.estimate_detection_distance(det) if scan_fresh else float("inf")
        sensor_confirmed = math.isfinite(distance)
        distance_source = "bbox_lidar" if sensor_confirmed else "none"
        if not sensor_confirmed and overlaps_path and math.isfinite(front_distance):
            distance = front_distance
            sensor_confirmed = True
            distance_source = "front_lidar"
        if not math.isfinite(distance):
            area_ratio = det.area / max(1.0, self.image_area())
            reference = max(float(self.fixed_obstacle_min_area_ratio), 0.01)
            distance = float(self.fixed_obstacle_detection_range_m) * math.sqrt(reference / max(area_ratio, 1e-6))
            distance_source = "bbox_only"
        close = distance <= float(self.fixed_obstacle_detection_range_m)
        reason = (
            f"{det.class_name}:pos={position_ratio:.2f}:dist={distance:.2f}m:"
            f"source={distance_source}"
        )
        return FixedObstacleEstimate(
            detected=True,
            in_path=overlaps_path and low_enough,
            close=close,
            sensor_confirmed=sensor_confirmed,
            direction="fixed",
            position=position,
            distance_m=float(distance),
            position_ratio=float(position_ratio),
            distance_source=distance_source,
            reason=reason,
        )

    def fixed_obstacle_avoid_direction(self, obstacle: FixedObstacleEstimate) -> Optional[str]:
        preferred = "right" if obstacle.position == "left" else "left" if obstacle.position == "right" else ""
        if preferred and self.side_is_clear(preferred):
            return preferred
        candidates = [side for side in ("left", "right") if self.side_is_clear(side)]
        if not candidates:
            return None
        return max(
            candidates,
            key=lambda side: self.latest_left_distance if side == "left" else self.latest_right_distance,
        )

    def fixed_obstacle_side_detected(self, side: str) -> bool:
        values = self.ultra_values_for_side(side)
        ultra_detected = bool(values) and min(values) < float(self.fixed_obstacle_pass_side_detect_cm)
        scan_distance = self.latest_left_distance if side == "left" else self.latest_right_distance
        scan_detected = math.isfinite(scan_distance) and scan_distance < float(self.side_clearance_m)
        return ultra_detected or scan_detected

    def fixed_obstacle_side_cleared(self, side: str) -> bool:
        values = self.ultra_values_for_side(side)
        ultra_cleared = not values or min(values) > float(self.fixed_obstacle_pass_side_clear_cm)
        scan_distance = self.latest_left_distance if side == "left" else self.latest_right_distance
        scan_cleared = not math.isfinite(scan_distance) or scan_distance > float(self.side_clearance_m)
        return ultra_cleared and scan_cleared

    def fixed_obstacle_lane_confidence(self, now: float) -> float:
        confidence = float(getattr(getattr(self, "latest_lane", None), "confidence", 0.0))
        external_time = float(getattr(self, "external_lane_time", 0.0))
        timeout = float(getattr(self, "external_cmd_timeout_sec", 0.5))
        if now - external_time <= timeout:
            confidence = max(confidence, float(getattr(self, "external_lane_confidence", 0.0)))
        return confidence

    def reset_fixed_obstacle_state(self) -> None:
        self.fixed_obstacle_state = "CLEAR"
        self.fixed_obstacle_frames = 0
        self.fixed_obstacle_clear_frames = 0
        self.fixed_obstacle_started_at = 0.0
        self.fixed_obstacle_phase_started_at = 0.0
        self.fixed_obstacle_rejoin_started_at = 0.0
        self.fixed_obstacle_direction = "center"
        self.fixed_obstacle_pass_detected = False
        self.fixed_obstacle_clear_since = 0.0
        self.fixed_obstacle_direction_candidate = ""
        self.fixed_obstacle_direction_clear_since = 0.0
        self.fixed_obstacle_cooldown_until = 0.0

    # Compatibility wrappers for scripts written against the former pedestrian API.
    def pedestrian_command(self, now: float, front_distance: float) -> Optional[Tuple[float, float, str, str]]:
        return self.fixed_obstacle_command(now, front_distance)

    def estimate_pedestrian(self, now: float, front_distance: float) -> FixedObstacleEstimate:
        del now
        return self.estimate_fixed_obstacle(front_distance)

    def reset_pedestrian_state(self) -> None:
        self.reset_fixed_obstacle_state()

    def should_take_shortcut(self, now: float) -> bool:
        if not bool(self.enable_shortcut) or self.shortcut_taken or self.shortcut_in_progress:
            return False
        if now < self.shortcut_until:
            return False
        target_lap = max(1, int(self.shortcut_target_lap))
        total_laps = max(1, int(self.total_laps))
        if self.lap_count < target_lap - 1 or self.lap_count >= total_laps:
            return False
        if now > self.route_gate_recent_until:
            return False

        left_signal_seen = self.latest_shortcut_signal or bool(
            self.filter_detections(("left_signal", "left_arrow", "shortcut_signal"))
        )
        if left_signal_seen:
            self.shortcut_signal_frames += 1
        else:
            self.shortcut_signal_frames = max(0, self.shortcut_signal_frames - 1)
        return self.shortcut_signal_frames >= int(self.shortcut_green_required_frames)

    def side_is_clear(self, side: str) -> bool:
        scan_dist = self.latest_left_distance if side == "left" else self.latest_right_distance
        scan_clear = scan_dist > float(self.side_clearance_m)

        index = int(self.ultra_left_index if side == "left" else self.ultra_right_index)
        back_index = int(self.ultra_left_back_index if side == "left" else self.ultra_right_back_index)
        ultra_clear = True
        values = []
        for idx in (index, back_index):
            if 0 <= idx < len(self.latest_ultra):
                value = self.latest_ultra[idx]
                if value > 0:
                    values.append(float(value))
        if values:
            ultra_clear = min(values) > float(self.ultra_side_clearance_cm)

        return scan_clear and ultra_clear

    def detection_in_drive_corridor(self, det: ObjectDetection) -> bool:
        if self.image_width <= 0 or self.image_height <= 0:
            return True
        center_margin = self.image_width * 0.26
        center_ok = abs(det.cx - self.image_width * 0.5) < center_margin
        lower_ok = det.cy > self.image_height * 0.25
        return center_ok and lower_ok

    def detection_in_lower_image(self, det: ObjectDetection, min_y_ratio: float) -> bool:
        if self.image_height <= 0:
            return True
        return det.cy > self.image_height * min_y_ratio

    def detection_in_signal_region(self, det: ObjectDetection) -> bool:
        if self.image_width <= 0 or self.image_height <= 0:
            return True
        if det.cy > self.image_height * float(self.intersection_signal_max_y_ratio):
            return False
        return 0.04 * self.image_width <= det.cx <= 0.96 * self.image_width

    def image_area(self) -> float:
        if self.image_width <= 0 or self.image_height <= 0:
            return 640.0 * 480.0
        return float(self.image_width * self.image_height)

    def filter_detections(self, keywords: Sequence[str]) -> List[ObjectDetection]:
        now = time.monotonic()
        if now - self.last_detection_time > float(self.detection_timeout_sec):
            return []
        lowered = self.normalize_keywords(keywords)
        return [
            det
            for det in self.latest_detections
            if any(keyword in self.normalize_class_name(det.class_name) for keyword in lowered)
        ]

    @staticmethod
    def normalize_keywords(keywords: Sequence[str]) -> List[str]:
        if isinstance(keywords, str):
            items = keywords.split(",")
        else:
            items = keywords
        return [
            str(keyword).strip().lower().replace("-", "_").replace(" ", "_")
            for keyword in items
            if str(keyword).strip()
        ]

    @staticmethod
    def normalize_class_name(class_name: str) -> str:
        return str(class_name).lower().replace("-", "_").replace(" ", "_")

    def parse_detection(self, det) -> Optional[ObjectDetection]:
        class_name = resolve_detection_class_name(det)
        score = float(getattr(det, "score", getattr(det, "confidence", 1.0)))

        if hasattr(det, "bbox"):
            bbox = det.bbox
            center = getattr(bbox, "center", None)
            size = getattr(bbox, "size", None)
            if center is not None and size is not None:
                position = getattr(center, "position", center)
                cx = float(getattr(position, "x", getattr(center, "x", 0.0)))
                cy = float(getattr(position, "y", getattr(center, "y", 0.0)))
                width = float(getattr(size, "x", getattr(size, "width", 0.0)))
                height = float(getattr(size, "y", getattr(size, "height", 0.0)))
                return ObjectDetection(class_name, score, cx, cy, width, height)

        if all(hasattr(det, attr) for attr in ("xmin", "ymin", "xmax", "ymax")):
            xmin = float(det.xmin)
            ymin = float(det.ymin)
            xmax = float(det.xmax)
            ymax = float(det.ymax)
            return ObjectDetection(
                class_name,
                score,
                (xmin + xmax) * 0.5,
                (ymin + ymax) * 0.5,
                xmax - xmin,
                ymax - ymin,
            )

        return None

    def median_or_none(self, values: np.ndarray) -> Optional[float]:
        if values.size == 0:
            return None
        return float(np.median(values))

    def clamp_angle(self, angle: float) -> float:
        limit = float(self.max_steer_cmd)
        return float(np.clip(angle, -limit, limit))

    def rate_limit(self, previous: float, target: float, rate: float, dt: float) -> float:
        step = max(0.0, rate) * dt
        if target > previous + step:
            return previous + step
        if target < previous - step:
            return previous - step
        return target

    @staticmethod
    def is_cone_motion_state(state: str) -> bool:
        return str(state).startswith("CONE_")

    def launch_speed_for_state(self, state: str) -> float:
        if RuleDriverNode.is_cone_motion_state(state):
            return float(self.cone_launch_speed)
        return float(self.launch_speed)

    def speed_rates_for_state(self, state: str) -> Tuple[float, float]:
        if RuleDriverNode.is_cone_motion_state(state):
            return (
                float(self.cone_speed_accel_per_sec),
                float(self.cone_speed_decel_per_sec),
            )
        return float(self.speed_accel_per_sec), float(self.speed_decel_per_sec)

    def normalize_drive_speed(self, speed: float) -> float:
        speed = float(speed)
        if speed <= float(self.stop_speed) + 1e-6:
            return float(self.stop_speed)
        return max(float(self.minimum_drive_speed), speed)

    def publish_motor(self, angle: float, speed: float) -> None:
        command_angle = self.steering_command_from_target_angle(angle)
        if self.motor_message_type == "xycar_motor" and XycarMotor is not None:
            msg = XycarMotor()
            msg.angle = float(command_angle)
            msg.speed = float(speed)
        else:
            msg = Float32MultiArray()
            msg.data = [float(command_angle), float(speed)]
        self.motor_pub.publish(msg)

    def steering_command_from_target_angle(self, angle: float) -> float:
        sign = -1.0 if angle < 0.0 else 1.0
        target_angle = abs(angle)
        steering_map = self.steering_map_for_sign(sign)

        if target_angle <= steering_map[0][0]:
            return 0.0

        for index in range(1, len(steering_map)):
            low_actual, low_command = steering_map[index - 1]
            high_actual, high_command = steering_map[index]
            if target_angle <= high_actual:
                ratio = (target_angle - low_actual) / (high_actual - low_actual)
                command = low_command + ratio * (high_command - low_command)
                return sign * command

        return sign * steering_map[-1][1]

    def steering_map_for_sign(self, sign: float) -> Tuple[Tuple[float, float], ...]:
        if not bool(self.use_asymmetric_steering_map):
            return STEERING_ACTUAL_TO_COMMAND
        if sign < 0.0:
            actual = list(self.steering_negative_actual_deg)
            command = list(self.steering_negative_command)
        else:
            actual = list(self.steering_positive_actual_deg)
            command = list(self.steering_positive_command)
        if len(actual) < 2 or len(actual) != len(command):
            return STEERING_ACTUAL_TO_COMMAND
        table = tuple((float(a), float(c)) for a, c in zip(actual, command))
        if any(table[index][0] <= table[index - 1][0] for index in range(1, len(table))):
            return STEERING_ACTUAL_TO_COMMAND
        return table

    def publish_state(self, angle: float, speed: float, reason: str) -> None:
        front = self.latest_front_distance
        lane_conf = self.latest_lane.confidence
        obstacle = self.latest_obstacle
        fixed_obstacle = self.latest_fixed_obstacle
        motor_command = self.steering_command_from_target_angle(angle)
        steering_saturated = abs(angle) >= float(self.max_steer_cmd) - 1e-3
        external_age = (
            time.monotonic() - float(self.external_lane_time)
            if self.external_lane_time > 0.0
            else float("inf")
        )
        msg = String()
        msg.data = (
            f"{self.state} / reason={reason} / angle={angle:.1f} "
            f"/ motor_cmd={motor_command:.1f} / steer_sat={int(steering_saturated)} / speed={speed:.1f} "
            f"/ front={front:.2f} / light={self.latest_traffic_color} / int_sig={self.latest_intersection_signal} "
            f"/ lane={lane_conf:.2f} / ext_lane={self.external_lane_confidence:.2f}:"
            f"{external_age:.2f}s:{self.external_lane_generation} "
            f"/ cone_mode={int(self.cone_mode_active)} "
            f"/ fixed={self.fixed_obstacle_state}:{fixed_obstacle.position}:"
            f"{fixed_obstacle.position_ratio:.2f}:{fixed_obstacle.distance_m:.2f} "
            f"/ obs={self.obstacle_drive_state}:{self.overtake_phase}:{obstacle.distance_m:.2f}:"
            f"{obstacle.lane_state}:{obstacle.lateral_velocity_ratio_s:+.2f}:{obstacle.vehicle_count} "
            f"/ lap={self.lap_count}/{int(self.total_laps)}:pending={self.course_checkpoint_pending or 'none'}:"
            f"normal={self.normal_cone_pass_count}:shortcut={self.shortcut_pass_count}"
        )
        self.state_pub.publish(msg)

        now = time.monotonic()
        if now - self.last_status_log > 0.5:
            self.get_logger().info(msg.data)
            self.last_status_log = now

    def _log_cv_unavailable(self) -> None:
        now = time.monotonic()
        if now - self.last_cv_error_log > 2.0:
            self.get_logger().warn("OpenCV/cv_bridge is not available. Camera perception is disabled.")
            self.last_cv_error_log = now

    def show_debug_view(self, frame: np.ndarray) -> None:
        if cv2 is None:
            return
        vis = frame.copy()
        if self.latest_lane.valid:
            y = int(vis.shape[0] * 0.82)
            cv2.circle(vis, (int(self.latest_lane.target_x), y), 7, (0, 255, 255), -1)
            cv2.line(vis, (int(self.latest_lane.image_center_x), y - 25), (int(self.latest_lane.image_center_x), y + 25), (255, 255, 255), 2)
        cv2.putText(vis, self.state, (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        cv2.imshow("my_rule", vis)
        cv2.waitKey(1)

    def destroy_node(self) -> bool:
        if rclpy.ok():
            for _ in range(3):
                self.publish_motor(0.0, 0.0)
                time.sleep(0.03)
        if cv2 is not None and self.debug_view:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RuleDriverNode()
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
