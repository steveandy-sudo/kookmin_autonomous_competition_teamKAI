#!/usr/bin/env python3
import math
import time
from collections import deque
from pathlib import Path as FilePath
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from my_rule_msgs.msg import ObjectDetectionArray
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from my_rule.perception.camera_input import CameraRectifier
from my_rule.perception.lidar_camera_association import (
    ImageBox,
    associate_lidar_clusters_with_boxes,
    load_lidar_camera_extrinsic,
)
from my_rule.perception.object_perception import normalize_class_name
from my_rule.ego_cone_tracker import (
    EgoConeTracker,
    PlanarPose,
    TrackerSnapshot,
    merge_points,
    normalize_angle,
    reproject_points,
)

try:
    from scipy.interpolate import CubicSpline
except Exception:  # pragma: no cover - scipy is optional on the vehicle PC
    CubicSpline = None


Point2 = Tuple[float, float]


class ConeNode(Node):
    def __init__(self) -> None:
        super().__init__("my_rule_cone_node")
        perception_share = FilePath(
            get_package_share_directory("xycar_perception")
        )
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cone_cmd_topic", "my_rule/cone_cmd")
        self.declare_parameter("cone_cluster_topic", "my_rule/cone_clusters")
        self.declare_parameter(
            "cone_lidar_cluster_topic", "/my_rule/cone_lidar_clusters"
        )
        self.declare_parameter(
            "cone_fused_cluster_topic", "/my_rule/cone_fused_clusters"
        )
        self.declare_parameter("cone_path_topic", "my_rule/cone_path")
        self.declare_parameter(
            "cone_diagnostics_topic", "/my_rule/cone_diagnostics"
        )
        self.declare_parameter("cone_status_topic", "/my_rule/cone_status")
        self.declare_parameter("processing_gate_enabled", False)
        self.declare_parameter(
            "processing_enabled_topic",
            "/my_rule/cone_processing_enabled",
        )
        self.declare_parameter("cone_yolo_association_enabled", False)
        self.declare_parameter(
            "object_detections_topic", "/my_rule/object_detections"
        )
        self.declare_parameter(
            "camera_yaml",
            str(
                perception_share
                / "config"
                / "wide_camera_fisheye_1280x1024_20260708.yaml"
            ),
        )
        self.declare_parameter(
            "lidar_camera_extrinsic_yaml",
            str(
                perception_share
                / "config"
                / "lidar_camera_extrinsic_measured.yaml"
            ),
        )
        self.declare_parameter("camera_rect_balance", 0.3)
        self.declare_parameter("cone_yolo_box_timeout_sec", 0.75)
        self.declare_parameter("cone_yolo_min_confidence", 0.50)
        self.declare_parameter("cone_yolo_box_padding_ratio", 0.20)
        self.declare_parameter("cone_yolo_box_min_padding_px", 8.0)
        self.declare_parameter("cone_yolo_match_vertical", False)
        self.declare_parameter("cone_yolo_recover_corridor_partner", True)
        self.declare_parameter(
            "cone_yolo_recovered_centerline_max_deviation_m", 0.25
        )
        # YOLO confirms that the vehicle has entered the cone course. Once a
        # camera-confirmed LiDAR cluster has unlocked planning, geometry near
        # the accepted corridor may continue on native-rate 2-D LiDAR data.
        # This prevents a slow camera inference from erasing the steering path
        # without treating every pole-like LiDAR cluster as a cone.
        self.declare_parameter("lidar_geometry_planning_enabled", True)
        self.declare_parameter("lidar_geometry_path_band_margin_m", 0.16)
        self.declare_parameter(
            "lidar_geometry_reference_timeout_sec", 0.35
        )
        # Store camera-confirmed cone landmarks in odom and reproject them into
        # the current LiDAR frame. This replaces raw reuse of old vehicle-frame
        # coordinates during a short camera/LiDAR dropout.
        self.declare_parameter("ego_cone_tracking_enabled", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("ego_pose_timeout_sec", 0.12)
        self.declare_parameter("ego_cone_track_ttl_sec", 0.30)
        self.declare_parameter(
            "ego_cone_lidar_association_distance_m", 0.22
        )
        self.declare_parameter(
            "ego_cone_semantic_association_distance_m", 0.25
        )
        self.declare_parameter("ego_cone_lidar_smoothing_alpha", 0.30)
        self.declare_parameter("ego_cone_semantic_smoothing_alpha", 0.40)
        self.declare_parameter("ego_cone_merge_distance_m", 0.08)
        self.declare_parameter("ego_cone_min_forward_m", -0.20)
        self.declare_parameter("ego_cone_max_abs_lateral_m", 1.40)
        self.declare_parameter("ego_cone_max_pose_jump_m", 0.30)
        self.declare_parameter("ego_cone_max_pose_jump_yaw_deg", 45.0)
        self.declare_parameter("ego_cone_predicted_speed", 4.0)
        self.declare_parameter("ego_cone_predicted_confidence_cap", 0.55)
        self.declare_parameter("max_range_m", 3.0)
        self.declare_parameter("min_range_m", 0.18)
        self.declare_parameter("scan_angle_offset_deg", 0.0)
        self.declare_parameter("scan_front_min_deg", -70.0)
        self.declare_parameter("scan_front_max_deg", 70.0)
        self.declare_parameter("dbscan_eps_m", 0.15)
        self.declare_parameter("dbscan_min_samples", 3)
        self.declare_parameter("sparse_dbscan_min_samples", 2)
        self.declare_parameter("sparse_cluster_min_range_m", 0.8)
        self.declare_parameter("sparse_cluster_min_intensity", 80.0)
        self.declare_parameter("sparse_cluster_match_distance_m", 0.18)
        self.declare_parameter("sparse_cluster_required_frames", 2)
        self.declare_parameter("sparse_cluster_history_frames", 3)
        self.declare_parameter("max_cone_diameter_m", 0.3)
        self.declare_parameter("angle_bin_deg", 12.0)
        self.declare_parameter("group_grow_distance_m", 0.5)
        # Cones on one physical boundary were measured 0.25~0.35 m apart.
        # Direct neighbours may use a little tolerance, while a longer edge is
        # accepted only as one missing cone and only with tangent support.
        self.declare_parameter("boundary_expected_spacing_m", 0.30)
        self.declare_parameter("boundary_direct_gap_max_m", 0.50)
        self.declare_parameter("boundary_missing_gap_min_m", 0.50)
        self.declare_parameter("boundary_gap_max_m", 0.75)
        self.declare_parameter("boundary_gap_max_turn_deg", 55.0)
        # The 2026-07-21 venue bags contain plausible same-boundary gaps up to
        # about 1.0 m.  Keep the ordinary graph gate at 0.75 m and expose a
        # stricter, final-left-only continuation gate for those sparse spans.
        self.declare_parameter("boundary_extended_gap_max_m", 1.05)
        self.declare_parameter("boundary_extended_gap_max_turn_deg", 30.0)
        self.declare_parameter("boundary_extended_path_tube_m", 0.15)
        self.declare_parameter("boundary_extended_required_frames", 2)
        self.declare_parameter("boundary_extended_match_distance_m", 0.25)
        self.declare_parameter("boundary_extended_min_track_cones", 2)
        self.declare_parameter("boundary_identity_memory_frames", 6)
        self.declare_parameter("boundary_identity_match_distance_m", 0.35)
        self.declare_parameter("boundary_identity_swap_margin_m", 0.08)
        self.declare_parameter("seed_min_angle_deg", 20.0)
        self.declare_parameter("seed_max_angle_deg", 100.0)
        self.declare_parameter("seed_max_range_m", 3.0)
        self.declare_parameter("allow_single_boundary_fallback", True)
        self.declare_parameter("single_boundary_min_cones", 2)
        self.declare_parameter("single_boundary_min_span_m", 0.20)
        self.declare_parameter("single_boundary_switch_frames", 3)
        # A short paired observation should preserve the established physical
        # boundary identity, but several consecutive paired scans are enough
        # to re-anchor both sides before the next one-sided section.
        self.declare_parameter("paired_boundary_reanchor_frames", 3)
        self.declare_parameter(
            "single_boundary_reacquire_min_fused_clusters", 3
        )
        self.declare_parameter(
            "single_boundary_reacquire_require_opposite_support", True
        )
        self.declare_parameter("single_boundary_confidence_cap", 0.60)
        self.declare_parameter("nearest_gate_confidence_cap", 0.40)
        self.declare_parameter("single_boundary_max_speed", 9.5)
        self.declare_parameter("lidar_to_rear_axle_m", 0.42)
        self.declare_parameter("wheelbase_m", 0.33)
        self.declare_parameter("lookahead_min_m", 0.7)
        self.declare_parameter("lookahead_max_m", 1.45)
        self.declare_parameter("lookahead_scale", 0.12)
        self.declare_parameter("far_preview_distance_m", 0.75)
        self.declare_parameter("far_preview_weight", 0.65)
        self.declare_parameter("steering_gain", 1.05)
        self.declare_parameter("max_steer_cmd", 42.0)
        # Keep the established speed reduction profile independent from the
        # wider steering range.  Otherwise raising max_steer_cmd would make a
        # given bend run faster even though only its steering headroom changed.
        self.declare_parameter("cone_speed_full_steer_deg", 26.0)
        self.declare_parameter("cone_speed", 17.0)
        self.declare_parameter("cone_min_drive_speed", 9.0)
        self.declare_parameter("cone_speed_steer_exponent", 1.0)
        self.declare_parameter("cone_speed_confidence_floor_ratio", 0.35)
        # Keep the vehicle-tested 9.0~17 profile. A fresh, confident and
        # straight bilateral path may reach 17, but no longer exceeds it.
        self.declare_parameter("cone_preview_speed_control", True)
        self.declare_parameter("cone_straight_boost_speed", 17.0)
        self.declare_parameter("cone_straight_boost_min_confidence", 0.75)
        self.declare_parameter("cone_straight_boost_min_path_distance_m", 1.35)
        self.declare_parameter("cone_straight_boost_full_angle_deg", 1.0)
        self.declare_parameter("cone_straight_boost_max_angle_deg", 3.0)
        self.declare_parameter("cone_speed_preview_near_m", 0.80)
        self.declare_parameter("cone_speed_preview_far_m", 1.45)
        self.declare_parameter("cone_speed_preview_samples", 4)
        self.declare_parameter("min_confidence", 0.3)
        self.declare_parameter("path_interpolation_method", "linear")
        self.declare_parameter("path_sample_count", 100)
        self.declare_parameter("path_hold_frames", 5)
        self.declare_parameter("path_hold_sec", 0.35)
        self.declare_parameter("pair_max_forward_delta_m", 0.30)
        # LiDAR cluster centres can measure a few centimetres inside the physical
        # cone spacing, so keep a small tolerance below the 0.78 m course minimum.
        self.declare_parameter("min_corridor_width_m", 0.68)
        self.declare_parameter("max_corridor_width_m", 0.98)
        self.declare_parameter("expected_corridor_width_m", 0.85)
        # Recent indoor bags contain path-consistent bilateral observations up
        # to 1.05 m during the final left, while the venue bags stay below
        # 0.95 m.  Wider observations are temporary and never update the
        # nominal corridor-width learner.
        self.declare_parameter("final_left_pair_max_width_m", 1.10)
        self.declare_parameter("final_left_pair_max_forward_delta_m", 0.45)
        self.declare_parameter("final_left_wide_pair_path_deviation_m", 0.20)
        self.declare_parameter("final_left_wide_pair_required_frames", 2)
        self.declare_parameter("corridor_width_learning_enabled", True)
        self.declare_parameter("corridor_width_learning_alpha", 0.20)
        self.declare_parameter("corridor_width_max_update_m", 0.04)
        self.declare_parameter("corridor_width_learning_min_pairs", 2)
        self.declare_parameter("min_path_midpoints", 2)
        self.declare_parameter("min_path_span_m", 0.15)
        self.declare_parameter("path_gap_fill_start_m", 0.35)
        self.declare_parameter("path_gap_fill_max_m", 0.95)
        self.declare_parameter("path_gap_sample_spacing_m", 0.12)
        # A geometrically validated bilateral path may legitimately move this
        # far when it replaces a less reliable single-boundary estimate.
        self.declare_parameter("max_path_target_jump_m", 0.45)
        self.declare_parameter("inferred_max_path_target_jump_m", 0.25)
        self.declare_parameter("inferred_path_target_rate_mps", 0.80)
        # A measured bilateral corridor is normally the best source, but its
        # first frames after a one-sided fallback can be paired differently.
        # Keep the one-sided slew limit briefly while the pair stabilises.
        self.declare_parameter("paired_reacquire_frames", 3)
        self.declare_parameter("paired_reacquire_min_target_jump_m", 0.08)
        self.declare_parameter("paired_reacquire_path_target_rate_mps", 0.65)
        self.declare_parameter("path_target_rate_max_dt_sec", 0.12)
        self.declare_parameter("path_max_heading_range_deg", 120.0)
        # Control paths are expressed at the rear axle.  A path ordered around
        # the LiDAR origin can end closer to the rear axle after a tight turn;
        # Pure Pursuit would then start from the last sample and lose all
        # forward arc.  These guards keep ordering and continuity in the actual
        # controller frame without assuming that x is monotonic.
        self.declare_parameter("path_min_control_arc_m", 0.20)
        # A one-sided offset is less observable than a paired centre path.
        # Do not let a very short fragment replace an established turn: at the
        # 2026-08-19 162.17 s failure a 0.37 m left-offset fragment replaced a
        # 1.47 m right-turn path and briefly commanded the opposite sign.
        self.declare_parameter("single_boundary_min_control_arc_m", 0.50)
        # The four 2026-08-20 fixed-speed failures accepted one-sided paths
        # whose first point was 1.0~1.9 m ahead.  The successful pass never
        # started a one-sided path farther than 0.73 m in the same final bend.
        self.declare_parameter("final_left_single_path_max_start_m", 0.85)
        self.declare_parameter("final_left_reversal_target_margin_m", 0.01)
        self.declare_parameter("path_orientation_start_margin_m", 0.04)
        # Lateral slew limiting translates an inferred path. That translation
        # can make a late sample much closer to the rear axle than sample zero,
        # invalidating the path ordering assumed by Pure Pursuit.
        self.declare_parameter("path_start_nearest_max_fraction", 0.20)
        self.declare_parameter("path_start_nearest_distance_margin_m", 0.05)
        self.declare_parameter("path_heading_jump_threshold_deg", 55.0)
        self.declare_parameter("path_heading_jump_required_frames", 2)
        self.declare_parameter("path_heading_confirmation_tolerance_deg", 20.0)
        self.declare_parameter("path_heading_probe_distance_m", 0.35)
        self.declare_parameter("path_reversal_min_lateral_m", 0.12)
        self.declare_parameter("path_reversal_required_frames", 2)
        self.declare_parameter("steering_median_window", 1)
        self.declare_parameter("steering_max_delta_deg", 14.0)
        self.declare_parameter("inferred_steering_max_delta_deg", 10.0)
        self.declare_parameter("steering_max_rate_deg_per_sec", 180.0)
        self.declare_parameter(
            "inferred_steering_max_rate_deg_per_sec", 100.0
        )
        self.declare_parameter("steering_rate_max_dt_sec", 0.12)
        self.declare_parameter("sharp_curve_preview_enabled", True)
        self.declare_parameter("sharp_curve_heading_threshold_deg", 22.0)
        self.declare_parameter("sharp_curve_steering_gain", 1.18)
        # The standalone cone course has a measured left -> right -> large-left
        # sequence.  A pending phase catches the first measured turn-back even
        # when bilateral geometry is sparse; once committed, a one-frame path
        # failure must not release or reverse the known final turn.
        # Course phase remains available in diagnostics, but must not impose a
        # fixed left-right-left steering sequence on a displaced entry.
        self.declare_parameter("cone_turn_sequence_guard_enabled", False)
        self.declare_parameter("cone_first_left_enter_steer_deg", 5.0)
        self.declare_parameter("cone_right_enter_steer_deg", 8.0)
        self.declare_parameter("cone_right_minimum_hold_sec", 0.75)
        self.declare_parameter("cone_final_left_pending_steer_deg", 3.0)
        self.declare_parameter("cone_final_left_enter_steer_deg", 5.0)
        self.declare_parameter("cone_final_left_pending_frames", 2)
        self.declare_parameter("cone_final_left_pending_hold_steer_deg", 5.0)
        # Venue command medians are -10.7~-13.3 deg, their tight peaks are
        # -16.2~-17.1 deg, and the successful fixed-8 indoor pass reached
        # -21.9 deg.  Ramp through that measured envelope only while geometry
        # is missing, held, or trying to unwind the final left.
        self.declare_parameter("cone_final_left_min_hold_steer_deg", 12.0)
        self.declare_parameter("cone_final_left_fallback_target_deg", 16.5)
        self.declare_parameter("cone_final_left_fallback_max_steer_deg", 22.0)
        self.declare_parameter("cone_final_left_fallback_ramp_deg_per_sec", 10.0)
        self.declare_parameter("cone_final_left_recovery_speed", 8.0)
        self.declare_parameter("cone_final_left_recovery_frames", 20)
        self.declare_parameter("cone_final_left_recovery_max_sec", 2.00)
        # Straightening is geometry-triggered, not time-triggered. The three
        # 2026-07-21 venue runs peaked at 16.2~17.1 deg and only then unwound.
        self.declare_parameter("cone_final_left_release_min_peak_deg", 15.0)
        # The 2026-08-20 03:11 failure peaked at only 14.0 deg in raw geometry
        # even though the guarded output had already delivered -22 deg for
        # more than one second.  In that case accept a lower measured peak
        # only after a sufficiently strong guarded turn was actually held.
        self.declare_parameter(
            "cone_final_left_release_guarded_min_peak_deg", 12.0
        )
        self.declare_parameter(
            "cone_final_left_release_guarded_output_deg", 21.0
        )
        self.declare_parameter(
            "cone_final_left_release_guarded_hold_sec", 0.60
        )
        self.declare_parameter("cone_final_left_release_margin_deg", 3.0)
        self.declare_parameter("cone_final_left_release_required_frames", 2)
        self.declare_parameter(
            "cone_final_left_release_recovery_rate_deg_per_sec", 3.0
        )
        self.declare_parameter("cone_final_left_exit_steer_deg", 3.0)
        self.declare_parameter("cone_final_left_exit_frames", 3)
        self.declare_parameter("allow_nearest_gate_fallback", True)
        self.declare_parameter("fallback_pair_min_lateral_separation_m", 0.4)
        self.declare_parameter("fallback_pair_max_center_offset_m", 0.65)
        self.declare_parameter("blind_recovery_frames", 5)
        self.declare_parameter("blind_recovery_max_sec", 0.35)
        # A rejected malformed path still provides cone-presence evidence.
        # Preserve the established turn with decay long enough to move through
        # that brief occlusion instead of stopping in an unrecoverable view.
        self.declare_parameter("invalid_inferred_recovery_frames", 10)
        self.declare_parameter("invalid_inferred_recovery_max_sec", 1.00)
        self.declare_parameter("blind_recovery_min_clusters", 2)
        self.declare_parameter("blind_recovery_steer_decay", 0.92)
        # Recovery is a safety exception to a nominal fixed-speed profile.
        # Creep while geometry is invalid and do not unwind a confirmed final
        # left into the right-hand cone boundary.
        self.declare_parameter("blind_recovery_speed", 4.0)
        self.declare_parameter("invalid_inferred_recovery_speed", 3.0)
        self.declare_parameter("invalid_inferred_recovery_steer_decay", 1.0)
        self.declare_parameter("diagnostics_log_period_sec", 0.50)

        self.scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cmd_pub = self.create_publisher(Float32MultiArray, str(self.get_parameter("cone_cmd_topic").value), 10)
        self.cluster_pub = self.create_publisher(PoseArray, str(self.get_parameter("cone_cluster_topic").value), 10)
        self.lidar_cluster_pub = self.create_publisher(
            PoseArray,
            str(self.get_parameter("cone_lidar_cluster_topic").value),
            10,
        )
        self.fused_cluster_pub = self.create_publisher(
            PoseArray,
            str(self.get_parameter("cone_fused_cluster_topic").value),
            10,
        )
        self.path_pub = self.create_publisher(Path, str(self.get_parameter("cone_path_topic").value), 10)
        self.diagnostics_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("cone_diagnostics_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("cone_status_topic").value),
            10,
        )
        self.scan_subscription = None
        self.cone_yolo_boxes: List[ImageBox] = []
        self.cone_yolo_boxes_time = float("-inf")
        self.cone_yolo_boxes_stamp_ns = 0
        self.cone_yolo_image_size = (0, 0)
        self.cone_yolo_camera_matrix: Optional[np.ndarray] = None
        self.cone_yolo_subscription = None
        self.cone_yolo_association_enabled = bool(
            self.get_parameter("cone_yolo_association_enabled").value
        )
        self.camera_rectifier: Optional[CameraRectifier] = None
        self.rotation_camera_laser: Optional[np.ndarray] = None
        self.translation_camera_laser: Optional[np.ndarray] = None
        if self.cone_yolo_association_enabled:
            self.camera_rectifier = CameraRectifier(
                str(self.get_parameter("camera_yaml").value),
                float(self.get_parameter("camera_rect_balance").value),
            )
            (
                self.rotation_camera_laser,
                self.translation_camera_laser,
            ) = load_lidar_camera_extrinsic(
                str(
                    self.get_parameter(
                        "lidar_camera_extrinsic_yaml"
                    ).value
                )
            )
            self.cone_yolo_subscription = self.create_subscription(
                ObjectDetectionArray,
                str(self.get_parameter("object_detections_topic").value),
                self.object_detections_callback,
                self.scan_qos,
            )
        self.prev_path: Optional[List[Point2]] = None
        self.last_path_target_lateral: Optional[float] = None
        self.last_raw_path_target_lateral = 0.0
        self.last_output_path_target_lateral = 0.0
        self.path_target_limit_applied = False
        self.path_miss_count = 0
        self.path_is_held = False
        self.current_scan_time: Optional[float] = None
        self.last_scan_time: Optional[float] = None
        self.last_path_accept_time: Optional[float] = None
        self.last_path_update_time: Optional[float] = None
        self.geometry_reference_path: Optional[List[Point2]] = None
        self.geometry_reference_time: Optional[float] = None
        self.ego_cone_tracking_enabled = bool(
            self.get_parameter("ego_cone_tracking_enabled").value
        )
        self.latest_odom_pose: Optional[PlanarPose] = None
        self.latest_odom_pose_time = float("-inf")
        self.last_ego_projection_pose: Optional[PlanarPose] = None
        self.ego_tracker_pose_valid = False
        self.ego_tracker_observed_count = 0
        self.ego_tracker_predicted_count = 0
        self.ego_tracker_max_prediction_age_sec = 0.0
        self.ego_tracker_prediction_only = False
        self.ego_tracker_prediction_authoritative = False
        self.ego_tracker_pose_jump_reset = False
        self.ego_cone_tracker = EgoConeTracker(
            ttl_sec=float(
                self.get_parameter("ego_cone_track_ttl_sec").value
            ),
            lidar_association_distance_m=float(
                self.get_parameter(
                    "ego_cone_lidar_association_distance_m"
                ).value
            ),
            semantic_association_distance_m=float(
                self.get_parameter(
                    "ego_cone_semantic_association_distance_m"
                ).value
            ),
            lidar_smoothing_alpha=float(
                self.get_parameter("ego_cone_lidar_smoothing_alpha").value
            ),
            semantic_smoothing_alpha=float(
                self.get_parameter(
                    "ego_cone_semantic_smoothing_alpha"
                ).value
            ),
            sensor_x_offset_m=float(
                self.get_parameter("lidar_to_rear_axle_m").value
            ),
            minimum_forward_m=float(
                self.get_parameter("ego_cone_min_forward_m").value
            ),
            maximum_forward_m=float(
                self.get_parameter("max_range_m").value
            ),
            maximum_abs_lateral_m=float(
                self.get_parameter("ego_cone_max_abs_lateral_m").value
            ),
            maximum_pose_jump_m=float(
                self.get_parameter("ego_cone_max_pose_jump_m").value
            ),
            maximum_pose_jump_yaw_deg=float(
                self.get_parameter("ego_cone_max_pose_jump_yaw_deg").value
            ),
        )
        self.odom_subscription = None
        if self.ego_cone_tracking_enabled:
            self.odom_subscription = self.create_subscription(
                Odometry,
                str(self.get_parameter("odom_topic").value),
                self.odometry_callback,
                20,
            )
        self.last_steering_time: Optional[float] = None
        self.last_diagnostics_log_time = float("-inf")
        self.learned_corridor_width_m = float(
            self.get_parameter("expected_corridor_width_m").value
        )
        self.geometry_planning_unlocked = not self.cone_yolo_association_enabled
        history_frames = max(1, int(self.get_parameter("sparse_cluster_history_frames").value))
        self.cluster_candidate_history = deque(maxlen=history_frames)
        self.midpoints_inferred = False
        self.midpoint_source = "none"
        self.active_inferred_boundary: Optional[str] = None
        self.pending_inferred_boundary: Optional[str] = None
        self.pending_inferred_frames = 0
        self.paired_boundary_frames = 0
        self.wide_pair_candidate_frames = 0
        self.last_wide_pair_width_m = 0.0
        self.last_observed_pair_width_m = 0.0
        self.wide_pair_active = False
        self.extended_boundary_candidates = {0: None, 1: None}
        self.extended_boundary_candidate_frames = {0: 0, 1: 0}
        self.current_yolo_confirmed_clusters: List[Point2] = []
        self.previous_left_boundary: List[Point2] = []
        self.previous_right_boundary: List[Point2] = []
        self.left_boundary_memory_age = 0
        self.right_boundary_memory_age = 0
        self.pending_path_reversal_sign = 0
        self.pending_path_reversal_frames = 0
        self.pending_path_heading_deg: Optional[float] = None
        self.pending_path_heading_frames = 0
        self.last_path_heading_deg: Optional[float] = None
        self.last_path_nearest_index = 0
        self.last_path_remaining_arc_m = 0.0
        self.last_path_orientation_reversed = False
        self.last_path_rejection_reason = "none"
        self.invalid_inferred_recovery_active = False
        self.accepted_path_source = "none"
        self.paired_reacquire_frames = 0
        self.steering_history = deque(maxlen=max(1, int(self.get_parameter("steering_median_window").value)))
        self.stabilized_steering: Optional[float] = None
        self.last_valid_steering = 0.0
        self.cone_turn_phase = "approach"
        self.right_turn_started_at: Optional[float] = None
        self.final_left_pending_frames = 0
        self.final_left_pending_started_at: Optional[float] = None
        self.final_left_committed_at: Optional[float] = None
        self.final_left_fallback_started_at: Optional[float] = None
        self.final_left_strong_hold_started_at: Optional[float] = None
        self.final_left_peak_steering = 0.0
        self.final_left_measured_peak_steering = 0.0
        self.final_left_release_frames = 0
        self.final_left_release_reference_steering = 0.0
        self.final_left_release_reference_time: Optional[float] = None
        self.final_left_exit_frames = 0
        self.blind_recovery_count = 0
        self.blind_recovery_started_at: Optional[float] = None
        self.last_recovery_angle = 0.0
        self.last_recovery_speed = 0.0
        self.had_valid_path = False
        self.processing_gate_enabled = bool(
            self.get_parameter("processing_gate_enabled").value
        )
        self.processing_enabled = not self.processing_gate_enabled
        self.processing_gate_subscription = None
        if self.processing_gate_enabled:
            gate_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.processing_gate_subscription = self.create_subscription(
                Bool,
                str(
                    self.get_parameter(
                        "processing_enabled_topic"
                    ).value
                ),
                self.processing_enabled_callback,
                gate_qos,
            )
        else:
            self.start_scan_processing()
        offset = float(self.get_parameter("scan_angle_offset_deg").value)
        method = str(self.get_parameter("path_interpolation_method").value)
        state = (
            "camera-gated sleep"
            if self.processing_gate_enabled
            else "active"
        )
        self.get_logger().info(
            f"cone_node ready: scan offset {offset:.1f} deg, "
            f"path interpolation {method}, state={state}, "
            "YOLO-cluster association="
            f"{'enabled' if self.cone_yolo_association_enabled else 'disabled'}, "
            "steering delta="
            f"{float(self.get_parameter('steering_max_delta_deg').value):.1f}/"
            f"{float(self.get_parameter('inferred_steering_max_delta_deg').value):.1f}deg, "
            "path target delta="
            f"{float(self.get_parameter('max_path_target_jump_m').value):.2f}/"
            f"{float(self.get_parameter('inferred_max_path_target_jump_m').value):.2f}m, "
            "steering rate="
            f"{float(self.get_parameter('steering_max_rate_deg_per_sec').value):.0f}/"
            f"{float(self.get_parameter('inferred_steering_max_rate_deg_per_sec').value):.0f}deg/s"
        )

    def odometry_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        )
        cos_yaw = 1.0 - 2.0 * (
            float(orientation.y) * float(orientation.y)
            + float(orientation.z) * float(orientation.z)
        )
        self.latest_odom_pose = PlanarPose(
            x=float(position.x),
            y=float(position.y),
            yaw=math.atan2(sin_yaw, cos_yaw),
        )
        self.latest_odom_pose_time = time.monotonic()

    def fresh_ego_pose(self, now: float) -> Optional[PlanarPose]:
        if not self.ego_cone_tracking_enabled:
            return None
        timeout = max(
            0.0,
            float(self.get_parameter("ego_pose_timeout_sec").value),
        )
        if (
            self.latest_odom_pose is None
            or float(now) - float(self.latest_odom_pose_time) > timeout
        ):
            return None
        return self.latest_odom_pose

    def clear_cached_geometry(self) -> None:
        """Drop geometry that cannot be safely moved across an odom jump."""

        self.prev_path = None
        self.geometry_reference_path = None
        self.geometry_reference_time = None
        self.last_path_accept_time = None
        self.last_path_update_time = None
        self.last_path_target_lateral = None
        self.previous_left_boundary = []
        self.previous_right_boundary = []
        self.cluster_candidate_history.clear()
        self.extended_boundary_candidates = {0: None, 1: None}

    def prepare_cached_geometry_for_pose(
        self,
        pose: Optional[PlanarPose],
    ) -> None:
        """Reproject all cached vehicle-frame geometry into the current pose."""

        self.ego_tracker_pose_valid = pose is not None
        if pose is None:
            return
        previous = self.last_ego_projection_pose
        self.last_ego_projection_pose = pose
        if previous is None:
            return

        translation = math.hypot(pose.x - previous.x, pose.y - previous.y)
        yaw_delta = abs(normalize_angle(pose.yaw - previous.yaw))
        maximum_translation = max(
            0.0,
            float(self.get_parameter("ego_cone_max_pose_jump_m").value),
        )
        maximum_yaw = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        "ego_cone_max_pose_jump_yaw_deg"
                    ).value
                ),
            )
        )
        if (
            (maximum_translation > 0.0 and translation > maximum_translation)
            or (maximum_yaw > 0.0 and yaw_delta > maximum_yaw)
        ):
            self.clear_cached_geometry()
            self.ego_tracker_pose_jump_reset = True
            return

        rear_offset = float(
            self.get_parameter("lidar_to_rear_axle_m").value
        )
        if self.prev_path:
            self.prev_path = reproject_points(
                self.prev_path,
                previous,
                pose,
            )
            self.last_path_target_lateral = self.path_target_lateral(
                self.prev_path
            )
            self.last_output_path_target_lateral = float(
                self.last_path_target_lateral
            )
            self.last_path_heading_deg = self.path_heading_at_distance_deg(
                self.prev_path,
                float(
                    self.get_parameter(
                        "path_heading_probe_distance_m"
                    ).value
                ),
            )
            self.last_path_nearest_index = self.path_nearest_index(
                self.prev_path
            )
            self.last_path_remaining_arc_m = self.path_arc_length(
                self.prev_path
            )
        if self.geometry_reference_path:
            self.geometry_reference_path = reproject_points(
                self.geometry_reference_path,
                previous,
                pose,
            )
        if self.previous_left_boundary:
            self.previous_left_boundary = reproject_points(
                self.previous_left_boundary,
                previous,
                pose,
                origin_x_offset_m=rear_offset,
            )
        if self.previous_right_boundary:
            self.previous_right_boundary = reproject_points(
                self.previous_right_boundary,
                previous,
                pose,
                origin_x_offset_m=rear_offset,
            )
        if self.current_yolo_confirmed_clusters:
            self.current_yolo_confirmed_clusters = reproject_points(
                self.current_yolo_confirmed_clusters,
                previous,
                pose,
                origin_x_offset_m=rear_offset,
            )
        history_maxlen = self.cluster_candidate_history.maxlen
        self.cluster_candidate_history = deque(
            (
                reproject_points(
                    points,
                    previous,
                    pose,
                    origin_x_offset_m=rear_offset,
                )
                for points in self.cluster_candidate_history
            ),
            maxlen=history_maxlen,
        )
        for side, candidate in list(self.extended_boundary_candidates.items()):
            if candidate is None:
                continue
            self.extended_boundary_candidates[side] = reproject_points(
                [candidate],
                previous,
                pose,
                origin_x_offset_m=rear_offset,
            )[0]

    def augment_planning_clusters_with_ego_tracks(
        self,
        planning_clusters: Sequence[Point2],
        lidar_clusters: Sequence[Point2],
        fused_clusters: Sequence[Point2],
        pose: Optional[PlanarPose],
        now: float,
    ) -> List[Point2]:
        self.ego_tracker_observed_count = 0
        self.ego_tracker_predicted_count = 0
        self.ego_tracker_max_prediction_age_sec = 0.0
        self.ego_tracker_prediction_only = False
        self.ego_tracker_prediction_authoritative = False
        if pose is None or not self.ego_cone_tracking_enabled:
            return list(planning_clusters)

        semantic_points = (
            list(fused_clusters)
            if self.cone_yolo_association_enabled
            else list(lidar_clusters)
        )
        snapshot: TrackerSnapshot = self.ego_cone_tracker.update(
            pose=pose,
            semantic_points=semantic_points,
            lidar_points=lidar_clusters,
            now_sec=now,
        )
        self.ego_tracker_pose_jump_reset = bool(
            self.ego_tracker_pose_jump_reset or snapshot.pose_jump_reset
        )
        self.ego_tracker_observed_count = snapshot.observed_count
        self.ego_tracker_predicted_count = snapshot.predicted_count
        self.ego_tracker_prediction_only = bool(
            snapshot.predicted_count > 0 and snapshot.observed_count == 0
        )
        self.ego_tracker_max_prediction_age_sec = max(
            (
                cone.age_sec
                for cone in snapshot.cones
                if cone.predicted
            ),
            default=0.0,
        )
        merged = merge_points(
            planning_clusters,
            snapshot.points,
            minimum_separation_m=float(
                self.get_parameter("ego_cone_merge_distance_m").value
            ),
        )
        fresh_left = any(point[1] > 0.05 for point in planning_clusters)
        fresh_right = any(point[1] < -0.05 for point in planning_clusters)
        merged_left = any(point[1] > 0.05 for point in merged)
        merged_right = any(point[1] < -0.05 for point in merged)
        self.ego_tracker_prediction_authoritative = bool(
            snapshot.predicted_count > 0
            and (
                not planning_clusters
                or (
                    not (fresh_left and fresh_right)
                    and merged_left
                    and merged_right
                )
            )
        )
        return merged

    def object_detections_callback(
        self, message: ObjectDetectionArray
    ) -> None:
        width = int(message.image_width)
        height = int(message.image_height)
        if width <= 0 or height <= 0 or self.camera_rectifier is None:
            return
        minimum_confidence = float(
            self.get_parameter("cone_yolo_min_confidence").value
        )
        boxes: List[ImageBox] = [
            (
                float(item.xmin),
                float(item.ymin),
                float(item.xmax),
                float(item.ymax),
            )
            for item in message.detections
            if (
                normalize_class_name(item.class_name) == "cone"
                and float(item.confidence) >= minimum_confidence
                and int(item.xmax) > int(item.xmin)
                and int(item.ymax) > int(item.ymin)
            )
        ]
        image_size = (width, height)
        if (
            self.cone_yolo_camera_matrix is None
            or image_size != self.cone_yolo_image_size
        ):
            _, _, _, matrix = self.camera_rectifier.rectification_parameters(
                width, height
            )
            self.cone_yolo_camera_matrix = matrix
            self.cone_yolo_image_size = image_size
        self.cone_yolo_boxes = boxes
        self.cone_yolo_boxes_time = time.monotonic()
        self.cone_yolo_boxes_stamp_ns = self.message_stamp_ns(message)

    def yolo_box_age_sec(self, scan_stamp_ns: int = 0) -> float:
        age_sec = time.monotonic() - self.cone_yolo_boxes_time
        if scan_stamp_ns > 0 and self.cone_yolo_boxes_stamp_ns > 0:
            stamp_delta_sec = (
                scan_stamp_ns - self.cone_yolo_boxes_stamp_ns
            ) / 1.0e9
            # Some sensor drivers use a separate clock. Only compare stamps
            # that plausibly share the active ROS clock.
            if abs(stamp_delta_sec) < 60.0:
                age_sec = max(0.0, stamp_delta_sec)
        return float(age_sec)

    @staticmethod
    def message_stamp_ns(message) -> int:
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        if stamp is None:
            return 0
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def associate_clusters_with_yolo(
        self,
        clusters: Sequence[Point2],
        scan_stamp_ns: int = 0,
    ) -> List[Point2]:
        if not self.cone_yolo_association_enabled:
            return list(clusters)
        timeout = max(
            0.0,
            float(self.get_parameter("cone_yolo_box_timeout_sec").value),
        )
        age_sec = self.yolo_box_age_sec(scan_stamp_ns)
        if (
            age_sec > timeout
            or self.cone_yolo_camera_matrix is None
            or self.rotation_camera_laser is None
            or self.translation_camera_laser is None
        ):
            return []
        width, height = self.cone_yolo_image_size
        return associate_lidar_clusters_with_boxes(
            clusters,
            self.cone_yolo_boxes,
            rotation_camera_laser=self.rotation_camera_laser,
            translation_camera_laser=self.translation_camera_laser,
            camera_matrix=self.cone_yolo_camera_matrix,
            image_width=width,
            image_height=height,
            padding_ratio=float(
                self.get_parameter("cone_yolo_box_padding_ratio").value
            ),
            minimum_padding_px=float(
                self.get_parameter("cone_yolo_box_min_padding_px").value
            ),
            match_vertical=bool(
                self.get_parameter("cone_yolo_match_vertical").value
            ),
        )

    def effective_corridor_width(self) -> float:
        minimum = float(self.get_parameter("min_corridor_width_m").value)
        maximum = float(self.get_parameter("max_corridor_width_m").value)
        default = float(self.get_parameter("expected_corridor_width_m").value)
        learned = float(getattr(self, "learned_corridor_width_m", default))
        return float(np.clip(learned, minimum, maximum))

    def final_left_phase_active(self, *, include_pending: bool = False) -> bool:
        phase = str(getattr(self, "cone_turn_phase", "approach"))
        committed = phase in (
            "final_left",
            "final_left_committed",
            "final_left_releasing",
        )
        return committed or (include_pending and phase == "final_left_pending")

    def point_is_yolo_confirmed(self, point: Point2) -> bool:
        match_distance = max(
            0.03,
            float(
                self.get_parameter(
                    "sparse_cluster_match_distance_m"
                ).value
            ),
        )
        return any(
            math.hypot(point[0] - confirmed[0], point[1] - confirmed[1])
            <= match_distance
            for confirmed in getattr(
                self,
                "current_yolo_confirmed_clusters",
                [],
            )
        )

    def update_corridor_width(self, measured_widths: Sequence[float]) -> None:
        """Learn the RC-course width only from a reliable bilateral frame."""
        if not bool(
            self.get_parameter("corridor_width_learning_enabled").value
        ):
            return
        minimum_pairs = max(
            1,
            int(
                self.get_parameter(
                    "corridor_width_learning_min_pairs"
                ).value
            ),
        )
        finite = [float(value) for value in measured_widths if math.isfinite(value)]
        if len(finite) < minimum_pairs:
            return
        minimum = float(self.get_parameter("min_corridor_width_m").value)
        maximum = float(self.get_parameter("max_corridor_width_m").value)
        measurement = float(np.median(np.asarray(finite, dtype=np.float64)))
        if not minimum <= measurement <= maximum:
            return
        previous = self.effective_corridor_width()
        alpha = float(
            np.clip(
                self.get_parameter("corridor_width_learning_alpha").value,
                0.0,
                1.0,
            )
        )
        maximum_update = max(
            0.0,
            float(self.get_parameter("corridor_width_max_update_m").value),
        )
        update = alpha * (measurement - previous)
        if maximum_update > 0.0:
            update = float(np.clip(update, -maximum_update, maximum_update))
        self.learned_corridor_width_m = float(
            np.clip(previous + update, minimum, maximum)
        )

    def select_planning_clusters(
        self,
        lidar_clusters: Sequence[Point2],
        yolo_clusters: Sequence[Point2],
    ) -> List[Point2]:
        """Use YOLO to unlock, then retain LiDAR geometry around the corridor.

        The 2-D scanner cannot reject poles by height as the old 3-D system did.
        Raw candidates are therefore admitted only inside a band around the
        last accepted centre path. Direct camera matches remain valid anchors.
        """
        raw = list(lidar_clusters)
        confirmed = list(yolo_clusters)
        if not self.cone_yolo_association_enabled:
            return raw
        if confirmed:
            self.geometry_planning_unlocked = True
        if not bool(
            self.get_parameter("lidar_geometry_planning_enabled").value
        ) or not getattr(self, "geometry_planning_unlocked", False):
            return confirmed
        now_value = getattr(self, "current_scan_time", None)
        now = time.monotonic() if now_value is None else float(now_value)
        reference_path = self.prev_path
        reference_time = getattr(self, "geometry_reference_time", None)
        reference_timeout = max(
            0.0,
            float(
                self.get_parameter(
                    "lidar_geometry_reference_timeout_sec"
                ).value
            ),
        )
        if (
            not reference_path
            and getattr(self, "geometry_reference_path", None)
            and reference_time is not None
            and now - float(reference_time) <= reference_timeout
        ):
            reference_path = self.geometry_reference_path
        if not reference_path:
            return confirmed

        margin = max(
            0.0,
            float(
                self.get_parameter(
                    "lidar_geometry_path_band_margin_m"
                ).value
            ),
        )
        lower = max(
            0.0,
            0.5 * float(self.get_parameter("min_corridor_width_m").value)
            - margin,
        )
        upper = (
            0.5 * float(self.get_parameter("max_corridor_width_m").value)
            + margin
        )
        rear_offset = float(self.get_parameter("lidar_to_rear_axle_m").value)
        selected = list(confirmed)
        selected_set = set(selected)
        for point in raw:
            if point in selected_set:
                continue
            path_distance = self.point_to_path_distance(
                (point[0] + rear_offset, point[1]),
                reference_path,
            )
            if lower <= path_distance <= upper:
                selected.append(point)
                selected_set.add(point)
        return sorted(selected, key=lambda point: math.hypot(*point))

    @staticmethod
    def point_to_path_distance(
        point: Point2,
        path: Sequence[Point2],
    ) -> float:
        """Return the shortest Euclidean distance from a point to a polyline."""
        if not path:
            return float("inf")
        px, py = point
        best = min(math.hypot(px - x, py - y) for x, y in path)
        for first, second in zip(path, path[1:]):
            vx = second[0] - first[0]
            vy = second[1] - first[1]
            length_squared = vx * vx + vy * vy
            if length_squared <= 1.0e-12:
                continue
            ratio = (
                (px - first[0]) * vx + (py - first[1]) * vy
            ) / length_squared
            ratio = float(np.clip(ratio, 0.0, 1.0))
            nearest = (first[0] + ratio * vx, first[1] + ratio * vy)
            best = min(best, math.hypot(px - nearest[0], py - nearest[1]))
        return float(best)

    @staticmethod
    def lateral_path_distance_at_x(
        candidate_path: Sequence[Point2],
        reference_path: Sequence[Point2],
        candidate_x_offset: float = 0.0,
    ) -> float:
        """Compare paths laterally at corresponding forward positions.

        ``x`` is used only to look up the reference path's lateral position.
        It must not contribute to the distance itself: consecutive LiDAR scans
        naturally move a boundary forward/backward in vehicle coordinates, and
        treating that longitudinal displacement as an error can prefer the
        normal pointing to the outside of a bend.
        """
        if not candidate_path or not reference_path:
            return float("inf")

        reference = sorted(
            ((float(x), float(y)) for x, y in reference_path),
            key=lambda point: point[0],
        )
        reference_x = np.asarray(
            [point[0] for point in reference],
            dtype=np.float64,
        )
        reference_y = np.asarray(
            [point[1] for point in reference],
            dtype=np.float64,
        )
        unique_x, unique_indices = np.unique(
            reference_x,
            return_index=True,
        )
        unique_y = reference_y[unique_indices]

        candidate_x = np.asarray(
            [float(x) + float(candidate_x_offset) for x, _ in candidate_path],
            dtype=np.float64,
        )
        candidate_y = np.asarray(
            [float(y) for _, y in candidate_path],
            dtype=np.float64,
        )
        reference_y_at_x = np.interp(candidate_x, unique_x, unique_y)
        return float(np.mean(np.abs(candidate_y - reference_y_at_x)))

    def recover_corridor_partners(
        self,
        lidar_clusters: Sequence[Point2],
        yolo_clusters: Sequence[Point2],
    ) -> List[Point2]:
        """Recover a camera-hidden cone only when corridor geometry supports it.

        A bend can move one physical boundary outside the camera image while
        both boundaries remain visible to the LiDAR.  Re-enabling every raw
        LiDAR cluster would also restore chair and table legs.  Instead, use a
        YOLO-confirmed cluster as an anchor and add at most one opposite cluster
        whose corridor width, longitudinal alignment and midpoint all agree
        with the most recent accepted path.
        """
        confirmed = list(yolo_clusters)
        if (
            not bool(
                self.get_parameter(
                    "cone_yolo_recover_corridor_partner"
                ).value
            )
            or not confirmed
        ):
            return confirmed

        max_forward_delta = float(
            self.get_parameter("pair_max_forward_delta_m").value
        )
        min_width = float(self.get_parameter("min_corridor_width_m").value)
        max_width = float(self.get_parameter("max_corridor_width_m").value)
        expected_width = self.effective_corridor_width()
        min_lateral = float(
            self.get_parameter("fallback_pair_min_lateral_separation_m").value
        )
        max_path_deviation = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_yolo_recovered_centerline_max_deviation_m"
                ).value
            ),
        )
        rear_offset = float(self.get_parameter("lidar_to_rear_axle_m").value)
        confirmed_set = set(confirmed)
        candidates = []
        for anchor in confirmed:
            for candidate in lidar_clusters:
                if candidate == anchor:
                    continue
                forward_delta = abs(anchor[0] - candidate[0])
                lateral_separation = abs(anchor[1] - candidate[1])
                width = math.hypot(
                    anchor[0] - candidate[0],
                    anchor[1] - candidate[1],
                )
                if (
                    forward_delta > max_forward_delta
                    or lateral_separation < min_lateral
                    or not min_width <= width <= max_width
                ):
                    continue
                midpoint_rear = (
                    0.5 * (anchor[0] + candidate[0]) + rear_offset,
                    0.5 * (anchor[1] + candidate[1]),
                )
                if self.prev_path:
                    path_deviation = self.point_to_path_distance(
                        midpoint_rear,
                        self.prev_path,
                    )
                    if path_deviation > max_path_deviation:
                        continue
                else:
                    # The first usable path has no temporal reference yet.
                    # Width, opposite lateral support and a centre corridor
                    # bound provide a conservative one-frame bootstrap.
                    max_center_offset = float(
                        self.get_parameter(
                            "fallback_pair_max_center_offset_m"
                        ).value
                    )
                    if anchor[1] * candidate[1] > 0.0:
                        continue
                    path_deviation = abs(midpoint_rear[1])
                    if path_deviation > max_center_offset:
                        continue
                # Pair two directly confirmed cones first. A raw-only partner
                # is considered only for a still-unpaired confirmed anchor.
                direct_rank = 0 if candidate in confirmed_set else 1
                cost = (
                    path_deviation
                    + 2.0 * forward_delta
                    + abs(width - expected_width)
                )
                candidates.append(
                    (direct_rank, cost, anchor, candidate)
                )

        used = set()
        recovered = list(confirmed)
        for _direct_rank, _cost, anchor, candidate in sorted(candidates):
            if anchor in used or candidate in used:
                continue
            used.add(anchor)
            used.add(candidate)
            if candidate not in confirmed_set:
                recovered.append(candidate)
        return recovered

    def processing_enabled_callback(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if requested == self.processing_enabled:
            return
        if requested:
            self.processing_enabled = True
            self.start_scan_processing()
            self.get_logger().info(
                "cone camera evidence received; LiDAR planning enabled"
            )
            return

        self.processing_enabled = False
        if self.scan_subscription is not None:
            self.destroy_subscription(self.scan_subscription)
            self.scan_subscription = None
        self.reset_processing_state()
        self.publish_cluster_array(self.lidar_cluster_pub, [])
        self.publish_cluster_array(self.fused_cluster_pub, [])
        self.publish_clusters([])
        self.publish_path([])
        self.publish_cmd(0.0, 0.0, 0.0)
        self.get_logger().info(
            "cone evidence cleared; LiDAR planning sleeping"
        )

    def start_scan_processing(self) -> None:
        if self.scan_subscription is not None:
            return
        self.scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            self.scan_qos,
        )

    def reset_processing_state(self) -> None:
        self.prev_path = None
        self.last_path_target_lateral = None
        self.last_raw_path_target_lateral = 0.0
        self.last_output_path_target_lateral = 0.0
        self.path_target_limit_applied = False
        self.path_miss_count = 0
        self.path_is_held = False
        self.current_scan_time = None
        self.last_scan_time = None
        self.last_path_accept_time = None
        self.last_path_update_time = None
        self.geometry_reference_path = None
        self.geometry_reference_time = None
        self.ego_cone_tracker.reset()
        self.last_ego_projection_pose = None
        self.ego_tracker_pose_valid = False
        self.ego_tracker_observed_count = 0
        self.ego_tracker_predicted_count = 0
        self.ego_tracker_max_prediction_age_sec = 0.0
        self.ego_tracker_prediction_only = False
        self.ego_tracker_prediction_authoritative = False
        self.ego_tracker_pose_jump_reset = False
        self.last_steering_time = None
        self.learned_corridor_width_m = float(
            self.get_parameter("expected_corridor_width_m").value
        )
        self.geometry_planning_unlocked = not self.cone_yolo_association_enabled
        self.cluster_candidate_history.clear()
        self.midpoints_inferred = False
        self.midpoint_source = "none"
        self.active_inferred_boundary = None
        self.pending_inferred_boundary = None
        self.pending_inferred_frames = 0
        self.paired_boundary_frames = 0
        self.wide_pair_candidate_frames = 0
        self.last_wide_pair_width_m = 0.0
        self.last_observed_pair_width_m = 0.0
        self.wide_pair_active = False
        self.extended_boundary_candidates = {0: None, 1: None}
        self.extended_boundary_candidate_frames = {0: 0, 1: 0}
        self.current_yolo_confirmed_clusters = []
        self.previous_left_boundary = []
        self.previous_right_boundary = []
        self.left_boundary_memory_age = 0
        self.right_boundary_memory_age = 0
        self.pending_path_reversal_sign = 0
        self.pending_path_reversal_frames = 0
        self.pending_path_heading_deg = None
        self.pending_path_heading_frames = 0
        self.last_path_heading_deg = None
        self.last_path_nearest_index = 0
        self.last_path_remaining_arc_m = 0.0
        self.last_path_orientation_reversed = False
        self.last_path_rejection_reason = "none"
        self.invalid_inferred_recovery_active = False
        self.accepted_path_source = "none"
        self.paired_reacquire_frames = 0
        self.steering_history.clear()
        self.stabilized_steering = None
        self.last_valid_steering = 0.0
        self.cone_turn_phase = "approach"
        self.right_turn_started_at = None
        self.final_left_pending_frames = 0
        self.final_left_pending_started_at = None
        self.final_left_committed_at = None
        self.final_left_fallback_started_at = None
        self.final_left_strong_hold_started_at = None
        self.final_left_peak_steering = 0.0
        self.final_left_measured_peak_steering = 0.0
        self.final_left_release_frames = 0
        self.final_left_release_reference_steering = 0.0
        self.final_left_release_reference_time = None
        self.final_left_exit_frames = 0
        self.blind_recovery_count = 0
        self.blind_recovery_started_at = None
        self.last_recovery_angle = 0.0
        self.last_recovery_speed = 0.0
        self.had_valid_path = False

    def scan_callback(self, msg: LaserScan) -> None:
        if not self.processing_enabled:
            return
        now = time.monotonic()
        scan_dt = (
            0.0
            if self.last_scan_time is None
            else max(0.0, now - self.last_scan_time)
        )
        self.current_scan_time = now
        self.last_scan_time = now
        self.ego_tracker_pose_jump_reset = False
        ego_pose = self.fresh_ego_pose(now)
        self.prepare_cached_geometry_for_pose(ego_pose)
        scan_stamp_ns = self.message_stamp_ns(msg)
        points = self.scan_to_points(msg)
        clusters = self.cluster_cones(points)
        clusters = self.filter_front_clusters_by_angle(clusters)
        self.publish_cluster_array(self.lidar_cluster_pub, clusters)
        yolo_confirmed_clusters = self.associate_clusters_with_yolo(
            clusters,
            scan_stamp_ns=scan_stamp_ns,
        )
        self.current_yolo_confirmed_clusters = list(
            yolo_confirmed_clusters
        )
        fused_clusters = self.recover_corridor_partners(
            clusters,
            yolo_confirmed_clusters,
        )
        self.publish_cluster_array(self.fused_cluster_pub, fused_clusters)
        planning_clusters = self.select_planning_clusters(
            clusters,
            fused_clusters,
        )
        planning_clusters = self.augment_planning_clusters_with_ego_tracks(
            planning_clusters,
            clusters,
            fused_clusters,
            ego_pose,
            now,
        )
        # The historical topic now reflects the exact candidates consumed by
        # path planning. Camera-only safety/entry logic keeps using the explicit
        # /cone_fused_clusters topic.
        self.publish_clusters(planning_clusters)

        left_cones, right_cones = self.form_cone_groups(planning_clusters)
        left_cones, right_cones = self.stabilize_boundary_identity(
            left_cones,
            right_cones,
        )
        midpoints = self.calculate_midpoints(left_cones, right_cones)
        boundary_switch_pending = (
            self.pending_inferred_boundary is not None
            and self.pending_inferred_frames > 0
        )
        if not midpoints and not boundary_switch_pending:
            fallback_midpoint = self.nearest_gate_midpoint(planning_clusters)
            if fallback_midpoint is not None:
                self.midpoints_inferred = True
                self.midpoint_source = "nearest_gate"
                midpoints = [fallback_midpoint]
        midpoints = self.guard_single_boundary_reacquisition(
            midpoints,
            fused_clusters=planning_clusters,
            left_cones=left_cones,
            right_cones=right_cones,
        )
        evidence_midpoint_count = len(midpoints)
        midpoints = self.bridge_midpoint_gaps(midpoints)
        path = self.interpolate_path(midpoints)
        self.publish_path(path)

        if not path:
            if self.publish_blind_recovery(len(planning_clusters)):
                self.publish_diagnostics(
                    raw_angle=self.last_valid_steering,
                    output_angle=float(self.last_recovery_angle),
                    speed=float(self.last_recovery_speed),
                    confidence=max(
                        0.21,
                        float(self.get_parameter("min_confidence").value)
                        * 0.75,
                    ),
                    scan_dt=scan_dt,
                    scan_stamp_ns=scan_stamp_ns,
                    lidar_count=len(clusters),
                    fused_count=len(fused_clusters),
                    planning_count=len(planning_clusters),
                    left_count=len(left_cones),
                    right_count=len(right_cones),
                    path_count=0,
                )
                return
            self.steering_history.clear()
            self.stabilized_steering = None
            self.publish_cmd(0.0, 0.0, 0.0)
            self.publish_diagnostics(
                raw_angle=0.0,
                output_angle=0.0,
                speed=0.0,
                confidence=0.0,
                scan_dt=scan_dt,
                scan_stamp_ns=scan_stamp_ns,
                lidar_count=len(clusters),
                fused_count=len(fused_clusters),
                planning_count=len(planning_clusters),
                left_count=len(left_cones),
                right_count=len(right_cones),
                path_count=0,
            )
            return

        raw_angle = self.pure_pursuit(path)
        # Phase recognition consumes the fresh geometric demand before output
        # rate limiting.  This preserves the measured right-to-left transition
        # that was hidden by steering slew in guarded bag 022406.
        self.update_cone_turn_phase(raw_angle)
        angle = self.stabilize_steering(raw_angle)
        angle = self.apply_cone_turn_phase_guard(angle)
        confidence = self.path_confidence(evidence_midpoint_count)
        speed = self.compute_speed(angle, confidence, path)
        if self.ego_tracker_prediction_authoritative:
            confidence = min(
                confidence,
                max(
                    0.21,
                    float(
                        self.get_parameter(
                            "ego_cone_predicted_confidence_cap"
                        ).value
                    ),
                ),
            )
            speed = min(
                speed,
                max(
                    0.0,
                    float(
                        self.get_parameter(
                            "ego_cone_predicted_speed"
                        ).value
                    ),
                ),
            )
        self.last_valid_steering = angle
        self.had_valid_path = True
        if not self.path_is_held:
            self.blind_recovery_count = 0
            self.blind_recovery_started_at = None
        self.publish_cmd(angle, speed, confidence)
        self.publish_diagnostics(
            raw_angle=raw_angle,
            output_angle=angle,
            speed=speed,
            confidence=confidence,
            scan_dt=scan_dt,
            scan_stamp_ns=scan_stamp_ns,
            lidar_count=len(clusters),
            fused_count=len(fused_clusters),
            planning_count=len(planning_clusters),
            left_count=len(left_cones),
            right_count=len(right_cones),
            path_count=len(path),
        )

    def scan_to_points(self, msg: LaserScan) -> np.ndarray:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
        angles = angles + math.radians(float(self.get_parameter("scan_angle_offset_deg").value))
        valid = (
            np.isfinite(ranges)
            & (ranges <= float(self.get_parameter("max_range_m").value))
            & (ranges > float(self.get_parameter("min_range_m").value))
            & self.angles_in_sector(
                angles,
                math.radians(float(self.get_parameter("scan_front_min_deg").value)),
                math.radians(float(self.get_parameter("scan_front_max_deg").value)),
            )
        )
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        intensities = np.asarray(msg.intensities, dtype=np.float32)
        if intensities.size != ranges.size:
            intensities = np.zeros_like(ranges)
        return np.column_stack((x, y, intensities[valid])).astype(np.float32)

    @staticmethod
    def angles_in_sector(angles: np.ndarray, min_angle: float, max_angle: float) -> np.ndarray:
        wrapped = (angles + math.pi) % (2.0 * math.pi) - math.pi
        min_wrapped = (min_angle + math.pi) % (2.0 * math.pi) - math.pi
        max_wrapped = (max_angle + math.pi) % (2.0 * math.pi) - math.pi
        if min_wrapped <= max_wrapped:
            return (wrapped >= min_wrapped) & (wrapped <= max_wrapped)
        return (wrapped >= min_wrapped) | (wrapped <= max_wrapped)

    def cluster_cones(self, points: np.ndarray) -> List[Point2]:
        strong_min_samples = max(2, int(self.get_parameter("dbscan_min_samples").value))
        sparse_min_samples = max(2, int(self.get_parameter("sparse_dbscan_min_samples").value))
        if points.shape[0] < sparse_min_samples:
            self.cluster_candidate_history.append([])
            return []
        xy_points = points[:, :2]
        labels = self.simple_dbscan(
            xy_points,
            float(self.get_parameter("dbscan_eps_m").value),
            sparse_min_samples,
        )
        candidates = []
        for label in sorted(set(labels)):
            if label < 0:
                continue
            cluster = points[labels == label]
            if cluster.shape[0] == 0:
                continue
            cluster_xy = cluster[:, :2]
            diameter = float(np.linalg.norm(cluster_xy.max(axis=0) - cluster_xy.min(axis=0)))
            if diameter > float(self.get_parameter("max_cone_diameter_m").value):
                continue
            dists = np.linalg.norm(cluster_xy, axis=1)
            p = cluster_xy[int(np.argmin(dists))]
            max_intensity = float(np.max(cluster[:, 2])) if cluster.shape[1] >= 3 else 0.0
            candidates.append(((float(p[0]), float(p[1])), int(cluster.shape[0]), max_intensity))

        centers: List[Point2] = []
        raw_centers = [candidate[0] for candidate in candidates]
        for center, sample_count, max_intensity in candidates:
            if sample_count >= strong_min_samples:
                centers.append(center)
                continue
            if not self.sparse_cluster_is_confirmed(center, max_intensity):
                continue
            centers.append(self.temporally_smoothed_center(center))
        self.cluster_candidate_history.append(raw_centers)
        return centers

    def sparse_cluster_is_confirmed(self, center: Point2, max_intensity: float) -> bool:
        if math.hypot(*center) < float(self.get_parameter("sparse_cluster_min_range_m").value):
            return False
        if max_intensity < float(self.get_parameter("sparse_cluster_min_intensity").value):
            return False
        required = max(1, int(self.get_parameter("sparse_cluster_required_frames").value))
        return self.cluster_history_support(center) + 1 >= required

    def cluster_history_support(self, center: Point2) -> int:
        match_distance = float(self.get_parameter("sparse_cluster_match_distance_m").value)
        support = 0
        for previous_centers in self.cluster_candidate_history:
            if any(math.hypot(center[0] - p[0], center[1] - p[1]) <= match_distance for p in previous_centers):
                support += 1
        return support

    def temporally_smoothed_center(self, center: Point2) -> Point2:
        match_distance = float(self.get_parameter("sparse_cluster_match_distance_m").value)
        matches = [center]
        for previous_centers in self.cluster_candidate_history:
            nearby = [
                p
                for p in previous_centers
                if math.hypot(center[0] - p[0], center[1] - p[1]) <= match_distance
            ]
            if nearby:
                matches.append(min(nearby, key=lambda p: math.hypot(center[0] - p[0], center[1] - p[1])))
        values = np.asarray(matches, dtype=np.float32)
        return float(values[:, 0].mean()), float(values[:, 1].mean())

    def simple_dbscan(self, points: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
        n = points.shape[0]
        labels = np.full(n, -1, dtype=np.int32)
        visited = np.zeros(n, dtype=bool)
        cluster_id = 0
        dmat = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
        neighbors = [np.where(dmat[i] <= eps)[0].tolist() for i in range(n)]
        for i in range(n):
            if visited[i]:
                continue
            visited[i] = True
            if len(neighbors[i]) < min_samples:
                labels[i] = -1
                continue
            labels[i] = cluster_id
            queue = deque(neighbors[i])
            while queue:
                j = queue.popleft()
                if not visited[j]:
                    visited[j] = True
                    if len(neighbors[j]) >= min_samples:
                        queue.extend(neighbors[j])
                if labels[j] < 0:
                    labels[j] = cluster_id
            cluster_id += 1
        return labels

    def filter_front_clusters_by_angle(self, centers: Sequence[Point2]) -> List[Point2]:
        degree_bin = float(self.get_parameter("angle_bin_deg").value)
        used_bins = set()
        filtered: List[Point2] = []
        for x, y in sorted(centers, key=lambda p: math.hypot(p[0], p[1])):
            bin_idx = int(math.degrees(math.atan2(y, x)) // degree_bin)
            if bin_idx in used_bins:
                continue
            filtered.append((x, y))
            used_bins.add(bin_idx)
        return filtered

    def order_connected_points(
        self,
        points: Sequence[Point2],
        *,
        maximum_gap_m: Optional[float] = None,
    ) -> List[Point2]:
        """Order a 2-D cone/path chain without assuming that x is monotonic.

        Tight cone courses can turn close to 90 degrees, where sorting by
        forward x shortens, reverses or crosses the path.  Start at the point
        nearest the vehicle and walk the locally connected chain instead.  The
        measured 0.25~0.35 m cone spacing is used as a soft preference; local
        tangent continuity prevents a walk from doubling back.
        """
        unique: List[Point2] = []
        for raw_x, raw_y in points:
            point = (float(raw_x), float(raw_y))
            if not any(math.hypot(point[0] - x, point[1] - y) < 1.0e-5 for x, y in unique):
                unique.append(point)
        if len(unique) < 2:
            return unique

        expected = max(
            0.05,
            float(self.get_parameter("boundary_expected_spacing_m").value),
        )
        maximum_gap = (
            float("inf")
            if maximum_gap_m is None
            else max(0.01, float(maximum_gap_m))
        )
        maximum_turn = math.radians(
            max(
                90.0,
                float(self.get_parameter("path_max_heading_range_deg").value),
            )
        )
        start = min(unique, key=lambda point: math.hypot(*point))
        ordered = [start]
        remaining = [point for point in unique if point != start]

        while remaining:
            endpoint = ordered[-1]
            previous_tangent = None
            if len(ordered) >= 2:
                previous = ordered[-2]
                tangent = np.asarray(
                    [endpoint[0] - previous[0], endpoint[1] - previous[1]],
                    dtype=np.float64,
                )
                norm = float(np.linalg.norm(tangent))
                if norm > 1.0e-6:
                    previous_tangent = tangent / norm

            candidates = []
            for point in remaining:
                vector = np.asarray(
                    [point[0] - endpoint[0], point[1] - endpoint[1]],
                    dtype=np.float64,
                )
                distance = float(np.linalg.norm(vector))
                if distance <= 1.0e-6 or distance > maximum_gap:
                    continue
                turn = 0.0
                if previous_tangent is not None:
                    direction = vector / distance
                    turn = math.acos(
                        float(
                            np.clip(
                                np.dot(previous_tangent, direction),
                                -1.0,
                                1.0,
                            )
                        )
                    )
                    if turn > maximum_turn:
                        continue
                # Distance dominates. Spacing and turn are tie-breakers that
                # retain a physical cone boundary through a tight bend.
                cost = (
                    distance
                    + 0.35 * abs(distance - expected)
                    + 0.18 * expected * turn
                )
                candidates.append((cost, distance, point))
            if not candidates:
                break
            _cost, _distance, selected = min(candidates)
            ordered.append(selected)
            remaining.remove(selected)
        return ordered

    @staticmethod
    def boundary_match_score(
        boundary: Sequence[Point2],
        reference: Sequence[Point2],
    ) -> float:
        if not boundary or not reference:
            return float("inf")
        return float(
            np.mean(
                [
                    min(
                        math.hypot(point[0] - old[0], point[1] - old[1])
                        for old in reference
                    )
                    for point in boundary
                ]
            )
        )

    def stabilize_boundary_identity(
        self,
        left: Sequence[Point2],
        right: Sequence[Point2],
    ) -> Tuple[List[Point2], List[Point2]]:
        """Keep physical left/right boundary identity through partial views.

        The instantaneous seed bearing can change side during a large bend.
        Short memory prevents the visible boundary from being reinterpreted as
        the opposite side, while expiring quickly enough not to encode a fixed
        competition-course shape.
        """
        left_values = list(left)
        right_values = list(right)
        previous_left = list(getattr(self, "previous_left_boundary", []))
        previous_right = list(getattr(self, "previous_right_boundary", []))
        match_limit = max(
            0.0,
            float(
                self.get_parameter(
                    "boundary_identity_match_distance_m"
                ).value
            ),
        )
        swap_margin = max(
            0.0,
            float(self.get_parameter("boundary_identity_swap_margin_m").value),
        )

        if left_values and right_values and previous_left and previous_right:
            normal = self.boundary_match_score(left_values, previous_left) + self.boundary_match_score(right_values, previous_right)
            swapped = self.boundary_match_score(left_values, previous_right) + self.boundary_match_score(right_values, previous_left)
            if swapped + swap_margin < normal:
                left_values, right_values = right_values, left_values
        elif left_values and not right_values and previous_right:
            right_score = self.boundary_match_score(left_values, previous_right)
            left_score = self.boundary_match_score(left_values, previous_left)
            if right_score <= match_limit and right_score + swap_margin < left_score:
                right_values, left_values = left_values, []
        elif right_values and not left_values and previous_left:
            left_score = self.boundary_match_score(right_values, previous_left)
            right_score = self.boundary_match_score(right_values, previous_right)
            if left_score <= match_limit and left_score + swap_margin < right_score:
                left_values, right_values = right_values, []

        memory_frames = max(
            0,
            int(self.get_parameter("boundary_identity_memory_frames").value),
        )
        if left_values:
            self.previous_left_boundary = list(left_values)
            self.left_boundary_memory_age = 0
        else:
            self.left_boundary_memory_age = getattr(self, "left_boundary_memory_age", 0) + 1
            if self.left_boundary_memory_age > memory_frames:
                self.previous_left_boundary = []
        if right_values:
            self.previous_right_boundary = list(right_values)
            self.right_boundary_memory_age = 0
        else:
            self.right_boundary_memory_age = getattr(self, "right_boundary_memory_age", 0) + 1
            if self.right_boundary_memory_age > memory_frames:
                self.previous_right_boundary = []
        return left_values, right_values

    def form_cone_groups(self, centers: Sequence[Point2]) -> Tuple[List[Point2], List[Point2]]:
        min_angle = float(self.get_parameter("seed_min_angle_deg").value)
        max_angle = float(self.get_parameter("seed_max_angle_deg").value)
        max_range = float(self.get_parameter("seed_max_range_m").value)
        left_seed = self.find_seed(centers, min_angle, max_angle, max_range)
        right_seed = self.find_seed(centers, 360.0 - max_angle, 360.0 - min_angle, max_range)
        if left_seed is not None and right_seed is not None:
            left, right = self.grow_groups_competitively(
                left_seed, right_seed, centers
            )
            return self.bridge_boundary_groups(left, right, centers)
        used = set()
        left = self.grow_group(left_seed, centers, used) if left_seed is not None else []
        right = self.grow_group(right_seed, centers, used) if right_seed is not None else []
        return self.bridge_boundary_groups(left, right, centers)

    def bridge_boundary_groups(
        self,
        left: Sequence[Point2],
        right: Sequence[Point2],
        centers: Sequence[Point2],
    ) -> Tuple[List[Point2], List[Point2]]:
        """Extend a locally established boundary across one missing cone.

        A long jump is accepted only after two points define a tangent and the
        new point continues that tangent within the configured turn angle. This
        is the 2-D RC equivalent of the old graph walk, with a much smaller
        metric threshold appropriate to the present course.
        """
        direct_gap = max(
            0.01,
            float(self.get_parameter("boundary_direct_gap_max_m").value),
        )
        missing_gap_min = max(
            direct_gap,
            float(self.get_parameter("boundary_missing_gap_min_m").value),
        )
        groups = [
            self.order_connected_points(left, maximum_gap_m=direct_gap),
            self.order_connected_points(right, maximum_gap_m=direct_gap),
        ]
        used = set(groups[0]) | set(groups[1])
        normal_maximum_gap = max(
            float(self.get_parameter("group_grow_distance_m").value),
            float(self.get_parameter("boundary_gap_max_m").value),
        )
        normal_maximum_turn = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter("boundary_gap_max_turn_deg").value
                ),
            )
        )
        final_left = self.final_left_phase_active(include_pending=True)
        extended_maximum_gap = normal_maximum_gap
        if final_left:
            extended_maximum_gap = max(
                normal_maximum_gap,
                float(
                    self.get_parameter(
                        "boundary_extended_gap_max_m"
                    ).value
                ),
            )
        extended_maximum_turn = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        "boundary_extended_gap_max_turn_deg"
                    ).value
                ),
            )
        )
        extended_tube = max(
            0.0,
            float(
                self.get_parameter(
                    "boundary_extended_path_tube_m"
                ).value
            ),
        )
        extended_minimum_track = max(
            2,
            int(
                self.get_parameter(
                    "boundary_extended_min_track_cones"
                ).value
            ),
        )

        while True:
            normal_candidates = []
            extended_by_side = {}
            for side, group in enumerate(groups):
                if len(group) < 2:
                    continue
                before, endpoint = group[-2], group[-1]
                tangent = np.asarray(
                    [endpoint[0] - before[0], endpoint[1] - before[1]],
                    dtype=np.float64,
                )
                tangent_norm = float(np.linalg.norm(tangent))
                if tangent_norm <= 1.0e-6:
                    continue
                tangent /= tangent_norm
                for point in centers:
                    if point in used:
                        continue
                    vector = np.asarray(
                        [point[0] - endpoint[0], point[1] - endpoint[1]],
                        dtype=np.float64,
                    )
                    distance = float(np.linalg.norm(vector))
                    if (
                        distance < missing_gap_min
                        or distance > extended_maximum_gap
                    ):
                        continue
                    direction = vector / distance
                    turn = math.acos(
                        float(np.clip(np.dot(tangent, direction), -1.0, 1.0))
                    )
                    expected = float(
                        self.get_parameter("boundary_expected_spacing_m").value
                    )
                    # This branch represents one missing cone, so prefer a gap
                    # close to twice the measured adjacent-cone spacing.
                    candidate = (
                        abs(distance - 2.0 * expected) + 0.35 * turn,
                        side,
                        point,
                    )
                    if distance <= normal_maximum_gap:
                        if turn <= normal_maximum_turn:
                            normal_candidates.append(candidate)
                        continue

                    # A 0.75~1.05 m continuation is admitted only inside the
                    # committed course trend and inside a narrow extrapolated
                    # tangent tube.  This is deliberately stricter than simply
                    # enlarging the nearest-neighbour radius around furniture.
                    if (
                        not final_left
                        or len(group) < extended_minimum_track
                        or turn > extended_maximum_turn
                    ):
                        continue
                    cross_track = abs(
                        tangent[0] * vector[1]
                        - tangent[1] * vector[0]
                    )
                    if cross_track > extended_tube:
                        continue
                    current = extended_by_side.get(side)
                    if current is None or candidate < current:
                        extended_by_side[side] = candidate

            if normal_candidates:
                best = min(normal_candidates)
            else:
                confirmed_extended = []
                required = max(
                    1,
                    int(
                        self.get_parameter(
                            "boundary_extended_required_frames"
                        ).value
                    ),
                )
                match_distance = max(
                    0.0,
                    float(
                        self.get_parameter(
                            "boundary_extended_match_distance_m"
                        ).value
                    ),
                )
                for side in (0, 1):
                    if side not in extended_by_side:
                        self.extended_boundary_candidates[side] = None
                        self.extended_boundary_candidate_frames[side] = 0
                for side, candidate in extended_by_side.items():
                    _cost, _candidate_side, point = candidate
                    previous = getattr(
                        self,
                        "extended_boundary_candidates",
                        {0: None, 1: None},
                    ).get(side)
                    if self.point_is_yolo_confirmed(point):
                        frames = required
                    elif (
                        previous is not None
                        and math.hypot(
                            point[0] - previous[0],
                            point[1] - previous[1],
                        )
                        <= match_distance
                    ):
                        frames = int(
                            getattr(
                                self,
                                "extended_boundary_candidate_frames",
                                {0: 0, 1: 0},
                            ).get(side, 0)
                        ) + 1
                    else:
                        frames = 1
                    self.extended_boundary_candidates[side] = point
                    self.extended_boundary_candidate_frames[side] = frames
                    if frames >= required:
                        confirmed_extended.append(candidate)
                if not confirmed_extended:
                    break
                best = min(confirmed_extended)

            _cost, side, point = best
            groups[side].append(point)
            groups[side] = self.order_connected_points(
                groups[side],
                maximum_gap_m=extended_maximum_gap,
            )
            used.add(point)
            self.extended_boundary_candidates[side] = None
            self.extended_boundary_candidate_frames[side] = 0
        return groups[0], groups[1]

    def grow_groups_competitively(
        self,
        left_seed: Point2,
        right_seed: Point2,
        centers: Sequence[Point2],
    ) -> Tuple[List[Point2], List[Point2]]:
        """Grow both boundaries together so the first side cannot consume the other."""
        threshold = min(
            float(self.get_parameter("group_grow_distance_m").value),
            float(self.get_parameter("boundary_direct_gap_max_m").value),
        )
        groups = [[left_seed], [right_seed]]
        used = {left_seed, right_seed}

        while True:
            best = None
            for point in centers:
                if point in used:
                    continue
                for side, group in enumerate(groups):
                    distance = min(math.hypot(point[0] - p[0], point[1] - p[1]) for p in group)
                    candidate = (distance, side, point)
                    if distance <= threshold and (best is None or candidate < best):
                        best = candidate
            if best is None:
                break
            _distance, side, point = best
            groups[side].append(point)
            used.add(point)

        left = self.order_connected_points(groups[0], maximum_gap_m=threshold)
        right = self.order_connected_points(groups[1], maximum_gap_m=threshold)
        return left, right

    def find_seed(
        self,
        centers: Sequence[Point2],
        start_deg: float,
        end_deg: float,
        max_dist: float,
    ) -> Optional[Point2]:
        best = None
        best_r = float("inf")
        for p in centers:
            r = math.hypot(p[0], p[1])
            deg = math.degrees(math.atan2(p[1], p[0]))
            if deg < 0.0:
                deg += 360.0
            if start_deg <= deg <= end_deg and r <= max_dist and r < best_r:
                best = p
                best_r = r
        return best

    def grow_group(self, seed: Point2, centers: Sequence[Point2], used: set) -> List[Point2]:
        threshold = min(
            float(self.get_parameter("group_grow_distance_m").value),
            float(self.get_parameter("boundary_direct_gap_max_m").value),
        )
        group: List[Point2] = []
        queue = deque([seed])
        used.add(seed)
        while queue:
            base = queue.popleft()
            group.append(base)
            for p in centers:
                if p in used:
                    continue
                if math.hypot(p[0] - base[0], p[1] - base[1]) <= threshold:
                    used.add(p)
                    queue.append(p)
        return self.order_connected_points(group, maximum_gap_m=threshold)

    def calculate_midpoints(self, left: Sequence[Point2], right: Sequence[Point2]) -> List[Point2]:
        self.midpoints_inferred = False
        self.midpoint_source = "none"
        if not left and not right:
            self.paired_boundary_frames = 0
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return []
        if not left or not right:
            self.paired_boundary_frames = 0
            return self.infer_midpoints_from_single_boundary(left, right)
        max_forward_delta = float(
            self.get_parameter("pair_max_forward_delta_m").value
        )
        min_width = float(self.get_parameter("min_corridor_width_m").value)
        max_width = float(self.get_parameter("max_corridor_width_m").value)
        expected_width = self.effective_corridor_width()
        final_left = self.final_left_phase_active(include_pending=True)
        allowed_max_width = max_width
        if final_left:
            allowed_max_width = max(
                max_width,
                float(
                    self.get_parameter(
                        "final_left_pair_max_width_m"
                    ).value
                ),
            )
            max_forward_delta = max(
                max_forward_delta,
                float(
                    self.get_parameter(
                        "final_left_pair_max_forward_delta_m"
                    ).value
                ),
            )

        ordinary_candidates = []
        wide_candidates = []
        observed_widths = []
        rear_offset = float(self.get_parameter("lidar_to_rear_axle_m").value)
        maximum_wide_path_deviation = max(
            0.0,
            float(
                self.get_parameter(
                    "final_left_wide_pair_path_deviation_m"
                ).value
            ),
        )
        for left_index, left_point in enumerate(left):
            for right_index, right_point in enumerate(right):
                forward_delta = abs(left_point[0] - right_point[0])
                width = math.hypot(left_point[0] - right_point[0], left_point[1] - right_point[1])
                if (
                    forward_delta > max_forward_delta
                    or not min_width <= width <= allowed_max_width
                ):
                    continue
                observed_widths.append(width)
                cost = 2.0 * forward_delta + abs(width - expected_width)
                candidate = (cost, left_index, right_index, width)
                if width <= max_width:
                    ordinary_candidates.append(candidate)
                    continue
                if not self.prev_path:
                    continue
                midpoint_rear = (
                    0.5 * (left_point[0] + right_point[0]) + rear_offset,
                    0.5 * (left_point[1] + right_point[1]),
                )
                if (
                    self.point_to_path_distance(midpoint_rear, self.prev_path)
                    > maximum_wide_path_deviation
                ):
                    continue
                wide_candidates.append(candidate)

        self.last_observed_pair_width_m = max(observed_widths, default=0.0)
        required_wide_frames = max(
            1,
            int(
                self.get_parameter(
                    "final_left_wide_pair_required_frames"
                ).value
            ),
        )
        yolo_anchored_wide = any(
            self.point_is_yolo_confirmed(left[left_index])
            or self.point_is_yolo_confirmed(right[right_index])
            for _cost, left_index, right_index, _width in wide_candidates
        )
        if wide_candidates:
            representative_width = float(
                np.median(
                    np.asarray(
                        [candidate[3] for candidate in wide_candidates],
                        dtype=np.float64,
                    )
                )
            )
            if (
                abs(
                    representative_width
                    - float(getattr(self, "last_wide_pair_width_m", 0.0))
                )
                <= 0.15
            ):
                self.wide_pair_candidate_frames = int(
                    getattr(self, "wide_pair_candidate_frames", 0)
                ) + 1
            else:
                self.wide_pair_candidate_frames = 1
            self.last_wide_pair_width_m = representative_width
        else:
            self.wide_pair_candidate_frames = 0
            self.last_wide_pair_width_m = 0.0
        allow_wide = bool(wide_candidates) and (
            yolo_anchored_wide
            or self.wide_pair_candidate_frames >= required_wide_frames
        )
        candidates = list(ordinary_candidates)
        if allow_wide:
            candidates.extend(wide_candidates)

        used_left = set()
        used_right = set()
        midpoints: List[Point2] = []
        paired_widths: List[float] = []
        for _cost, left_index, right_index, width in sorted(candidates):
            if left_index in used_left or right_index in used_right:
                continue
            left_point = left[left_index]
            right_point = right[right_index]
            midpoints.append(
                (
                    (left_point[0] + right_point[0]) * 0.5,
                    (left_point[1] + right_point[1]) * 0.5,
                )
            )
            used_left.add(left_index)
            used_right.add(right_index)
            # A temporary expanded corridor controls from the measured
            # midpoint, but never drags the nominal half-width used by a later
            # one-sided fallback toward furniture or a one-frame outlier.
            if width <= max_width:
                paired_widths.append(width)
        midpoints = self.order_connected_points(
            midpoints,
            maximum_gap_m=float(
                self.get_parameter("path_gap_fill_max_m").value
            ),
        )
        minimum_bilateral = max(2, int(self.get_parameter("min_path_midpoints").value))
        if len(midpoints) >= minimum_bilateral:
            self.wide_pair_active = any(
                width > max_width
                for _cost, left_index, right_index, width in sorted(candidates)
                if left_index in used_left and right_index in used_right
            )
            self.paired_boundary_frames = int(
                getattr(self, "paired_boundary_frames", 0)
            ) + 1
            reanchor_frames = max(
                1,
                int(
                    self.get_parameter(
                        "paired_boundary_reanchor_frames"
                    ).value
                ),
            )
            if self.paired_boundary_frames >= reanchor_frames:
                self.active_inferred_boundary = None
            self.update_corridor_width(paired_widths)
            # Two measured boundaries constrain the corridor directly. A denser
            # single boundary must never replace this path just because it has
            # more points; that caused left/right source flapping in S turns.
            self.midpoint_source = "paired"
            # Preserve the last reliable one-sided boundary identity. A brief
            # paired observation must not make the opposite boundary eligible
            # for immediate selection on the next sparse scan.
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return midpoints
        self.wide_pair_active = False
        self.paired_boundary_frames = 0
        inferred = self.infer_midpoints_from_richer_boundary(left, right)
        if inferred:
            return inferred
        if midpoints:
            self.midpoint_source = "paired_sparse"
            return midpoints
        return self.infer_midpoints_from_single_boundary(left, right)

    def guard_single_boundary_reacquisition(
        self,
        midpoints: Sequence[Point2],
        *,
        fused_clusters: Sequence[Point2],
        left_cones: Sequence[Point2],
        right_cones: Sequence[Point2],
    ) -> List[Point2]:
        """Reject an underconstrained one-sided path after path expiry.

        Initial single-boundary operation and updates while ``prev_path`` is
        alive retain their established behavior.  Only reacquisition after a
        previously valid path has expired is guarded: two isolated points from
        one side cannot define a trustworthy corridor tangent.  A third fused
        candidate and support from the opposite group are enough to restore
        the existing single-boundary fallback without requiring a valid pair.
        """
        values = list(midpoints)
        if (
            not values
            or self.prev_path is not None
            or not bool(getattr(self, "had_valid_path", False))
        ):
            return values

        source = str(getattr(self, "midpoint_source", "none"))
        if source == "left_offset":
            opposite_support = bool(right_cones)
        elif source == "right_offset":
            opposite_support = bool(left_cones)
        else:
            return values

        minimum_clusters = max(
            2,
            int(
                self.get_parameter(
                    "single_boundary_reacquire_min_fused_clusters"
                ).value
            ),
        )
        require_opposite = bool(
            self.get_parameter(
                "single_boundary_reacquire_require_opposite_support"
            ).value
        )
        if len(fused_clusters) < minimum_clusters or (
            require_opposite and not opposite_support
        ):
            self.midpoints_inferred = False
            self.midpoint_source = "reacquire_pending"
            return []
        return values

    def infer_midpoints_from_richer_boundary(
        self,
        left: Sequence[Point2],
        right: Sequence[Point2],
    ) -> List[Point2]:
        """Prefer a continuous boundary over a single, underconstrained cone pair."""
        minimum = max(2, int(self.get_parameter("single_boundary_min_cones").value))
        candidates = {}
        left_segment = self.continuous_boundary_segment(left)
        right_segment = self.continuous_boundary_segment(right)
        if self.boundary_is_sufficient(left_segment, minimum):
            centerline = self.offset_boundary_to_center(left_segment, is_left_boundary=True)
            candidates["left"] = (
                len(centerline),
                -float(np.mean(np.abs([p[1] for p in centerline]))),
                centerline,
            )
        if self.boundary_is_sufficient(right_segment, minimum):
            centerline = self.offset_boundary_to_center(right_segment, is_left_boundary=False)
            candidates["right"] = (
                len(centerline),
                -float(np.mean(np.abs([p[1] for p in centerline]))),
                centerline,
            )
        if not candidates:
            return []

        desired_side = max(candidates, key=lambda side: (candidates[side][0], candidates[side][1]))
        selected_side = self.select_inferred_boundary(desired_side, candidates)
        if selected_side is None:
            return []
        self.midpoints_inferred = True
        self.midpoint_source = f"{selected_side}_offset"
        return candidates[selected_side][2]

    def select_inferred_boundary(self, desired_side: str, candidates: dict) -> Optional[str]:
        """Debounce left/right single-boundary changes across sparse scan frames."""
        active_side = getattr(self, "active_inferred_boundary", None)
        if active_side is None:
            self.active_inferred_boundary = desired_side
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return desired_side

        if desired_side == active_side:
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return active_side

        # Once a usable single boundary has been selected, keep following its
        # inward normal until a measured bilateral corridor is available.  A
        # temporarily denser opposite boundary must not flip the inferred path.
        if active_side in candidates:
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return active_side

        if getattr(self, "pending_inferred_boundary", None) == desired_side:
            self.pending_inferred_frames = getattr(self, "pending_inferred_frames", 0) + 1
        else:
            self.pending_inferred_boundary = desired_side
            self.pending_inferred_frames = 1

        required = max(1, int(self.get_parameter("single_boundary_switch_frames").value))
        if self.pending_inferred_frames >= required:
            self.active_inferred_boundary = desired_side
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return desired_side

        # Keep the existing side for the confirmation frame. If it vanished,
        # return no candidate so interpolate_path holds the coherent old path.
        return active_side if active_side in candidates else None

    def continuous_boundary_segment(self, boundary: Sequence[Point2]) -> List[Point2]:
        """Return the longest locally connected segment and discard background outliers."""
        if not boundary:
            return []
        threshold = float(
            self.get_parameter("boundary_direct_gap_max_m").value
        )
        ordered = self.order_connected_points(
            boundary,
            maximum_gap_m=float(
                self.get_parameter("boundary_gap_max_m").value
            ),
        )
        segments: List[List[Point2]] = [[ordered[0]]]
        for point in ordered[1:]:
            previous = segments[-1][-1]
            if math.hypot(point[0] - previous[0], point[1] - previous[1]) <= threshold:
                segments[-1].append(point)
            else:
                segments.append([point])
        return max(
            segments,
            key=lambda segment: (
                len(segment),
                -float(np.mean([math.hypot(*point) for point in segment])),
            ),
        )

    def boundary_is_sufficient(self, boundary: Sequence[Point2], minimum: int) -> bool:
        if len(boundary) < minimum:
            return False
        span = sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(boundary, boundary[1:])
        )
        return span >= float(self.get_parameter("single_boundary_min_span_m").value)

    def infer_midpoints_from_single_boundary(
        self,
        left: Sequence[Point2],
        right: Sequence[Point2],
    ) -> List[Point2]:
        if not bool(self.get_parameter("allow_single_boundary_fallback").value):
            return []
        minimum = max(2, int(self.get_parameter("single_boundary_min_cones").value))
        left_segment = self.continuous_boundary_segment(left)
        right_segment = self.continuous_boundary_segment(right)
        if self.boundary_is_sufficient(left_segment, minimum) and not right:
            selected = self.select_inferred_boundary("left", {"left": (0, 0.0, [])})
            if selected != "left":
                return []
            self.midpoints_inferred = True
            self.midpoint_source = "left_offset"
            return self.offset_boundary_to_center(left_segment, is_left_boundary=True)
        if self.boundary_is_sufficient(right_segment, minimum) and not left:
            selected = self.select_inferred_boundary("right", {"right": (0, 0.0, [])})
            if selected != "right":
                return []
            self.midpoints_inferred = True
            self.midpoint_source = "right_offset"
            return self.offset_boundary_to_center(right_segment, is_left_boundary=False)
        return []

    def nearest_gate_midpoint(self, clusters: Sequence[Point2]) -> Optional[Point2]:
        if not bool(self.get_parameter("allow_nearest_gate_fallback").value):
            return None
        if len(clusters) < 2:
            return None

        max_forward_delta = float(self.get_parameter("pair_max_forward_delta_m").value)
        min_width = float(self.get_parameter("min_corridor_width_m").value)
        max_width = float(self.get_parameter("max_corridor_width_m").value)
        expected_width = self.effective_corridor_width()
        min_lateral = float(self.get_parameter("fallback_pair_min_lateral_separation_m").value)
        max_center_offset = float(self.get_parameter("fallback_pair_max_center_offset_m").value)
        candidates = []

        for first_index, first in enumerate(clusters):
            for second in clusters[first_index + 1 :]:
                forward_delta = abs(first[0] - second[0])
                lateral_separation = abs(first[1] - second[1])
                width = math.hypot(first[0] - second[0], first[1] - second[1])
                if forward_delta > max_forward_delta:
                    continue
                if lateral_separation < min_lateral or not min_width <= width <= max_width:
                    continue
                midpoint = ((first[0] + second[0]) * 0.5, (first[1] + second[1]) * 0.5)
                if abs(midpoint[1]) > max_center_offset:
                    continue
                distance = max(math.hypot(*first), math.hypot(*second))
                cost = distance + 2.0 * forward_delta + abs(width - expected_width) + 0.25 * abs(midpoint[1])
                candidates.append((cost, midpoint))

        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    def offset_boundary_to_center(self, boundary: Sequence[Point2], is_left_boundary: bool) -> List[Point2]:
        points = self.order_connected_points(
            boundary,
            maximum_gap_m=float(
                self.get_parameter("boundary_gap_max_m").value
            ),
        )
        expected_width = self.effective_corridor_width()
        half_width = expected_width * 0.5
        default_centerline: List[Point2] = []
        opposite_centerline: List[Point2] = []
        for index, (x, y) in enumerate(points):
            # A wider local baseline makes the boundary normal less sensitive
            # to one noisy cone while still following an S-shaped corridor.
            before = points[max(0, index - 2)]
            after = points[min(len(points) - 1, index + 2)]
            dx = after[0] - before[0]
            dy = after[1] - before[1]
            tangent_norm = math.hypot(dx, dy)
            if tangent_norm <= 1e-6:
                tangent_x, tangent_y = 1.0, 0.0
            else:
                tangent_x = dx / tangent_norm
                tangent_y = dy / tangent_norm
            if is_left_boundary:
                normal_x, normal_y = tangent_y, -tangent_x
            else:
                normal_x, normal_y = -tangent_y, tangent_x
            default_centerline.append(
                (
                    x + half_width * normal_x,
                    y + half_width * normal_y,
                )
            )
            opposite_centerline.append(
                (
                    x - half_width * normal_x,
                    y - half_width * normal_y,
                )
            )

        # On a tight bend, a physical boundary can cross the vehicle centre in
        # LiDAR coordinates and be seeded as the wrong side.  When a previously
        # accepted path exists, choose the normal direction that remains closest
        # to that path instead of trusting the instantaneous bearing alone.
        if self.prev_path:
            rear_offset = float(
                self.get_parameter("lidar_to_rear_axle_m").value
            )

            def path_score(candidate_path: Sequence[Point2]) -> float:
                return self.lateral_path_distance_at_x(
                    candidate_path,
                    self.prev_path,
                    candidate_x_offset=rear_offset,
                )

            if path_score(opposite_centerline) < path_score(default_centerline):
                default_centerline = opposite_centerline
        return default_centerline

    def bridge_midpoint_gaps(
        self,
        midpoints: Sequence[Point2],
    ) -> List[Point2]:
        """Densify a plausible centre-path gap without crossing a large void."""
        points = self.order_connected_points(
            [(float(x), float(y)) for x, y in midpoints],
            maximum_gap_m=float(
                self.get_parameter("path_gap_fill_max_m").value
            ),
        )
        if len(points) < 2:
            return points
        fill_start = max(
            0.0,
            float(self.get_parameter("path_gap_fill_start_m").value),
        )
        maximum_gap = max(
            fill_start,
            float(self.get_parameter("path_gap_fill_max_m").value),
        )
        spacing = max(
            0.03,
            float(self.get_parameter("path_gap_sample_spacing_m").value),
        )

        # Never make a single path jump across an implausibly large empty area.
        # Select a coherent segment; a previous accepted path can then be held
        # if that segment is too short for control.
        segments: List[List[Point2]] = [[points[0]]]
        for point in points[1:]:
            if math.hypot(
                point[0] - segments[-1][-1][0],
                point[1] - segments[-1][-1][1],
            ) > maximum_gap:
                segments.append([point])
            else:
                segments[-1].append(point)
        points = max(
            segments,
            key=lambda segment: (
                len(segment),
                sum(
                    math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(segment, segment[1:])
                ),
                -math.hypot(*segment[0]),
            ),
        )
        if len(points) < 2:
            return points

        bridged: List[Point2] = [points[0]]
        for index, (first, second) in enumerate(zip(points, points[1:])):
            delta = np.asarray(
                [second[0] - first[0], second[1] - first[1]],
                dtype=np.float64,
            )
            gap = float(np.linalg.norm(delta))
            insert_count = max(0, int(math.ceil(gap / spacing)) - 1)
            if gap <= fill_start or insert_count == 0:
                bridged.append(second)
                continue

            # With neighbours on both sides, cubic Hermite retains the local
            # entry/exit headings. At either end, the secant gives a monotonic
            # linear bridge and cannot overshoot a tight RC corridor.
            if index > 0:
                tangent_in = np.asarray(first) - np.asarray(points[index - 1])
            else:
                tangent_in = delta.copy()
            if index + 2 < len(points):
                tangent_out = np.asarray(points[index + 2]) - np.asarray(second)
            else:
                tangent_out = delta.copy()
            for tangent in (tangent_in, tangent_out):
                norm = float(np.linalg.norm(tangent))
                if norm > 1.0e-6:
                    tangent *= gap / norm
                else:
                    tangent[:] = delta

            p0 = np.asarray(first, dtype=np.float64)
            p1 = np.asarray(second, dtype=np.float64)
            for sample_index in range(1, insert_count + 1):
                ratio = sample_index / float(insert_count + 1)
                ratio2 = ratio * ratio
                ratio3 = ratio2 * ratio
                value = (
                    (2.0 * ratio3 - 3.0 * ratio2 + 1.0) * p0
                    + (ratio3 - 2.0 * ratio2 + ratio) * tangent_in
                    + (-2.0 * ratio3 + 3.0 * ratio2) * p1
                    + (ratio3 - ratio2) * tangent_out
                )
                bridged.append((float(value[0]), float(value[1])))
            bridged.append(second)
        return bridged

    def interpolate_path(self, midpoints: Sequence[Point2]) -> List[Point2]:
        min_midpoints = max(1, int(self.get_parameter("min_path_midpoints").value))
        if len(midpoints) < min_midpoints:
            return self.hold_previous_path()
        # Path following is performed around the rear axle, whereas cone
        # clusters are measured around the LiDAR.  Order the chain only after
        # moving it into the controller frame.  Ordering before this transform
        # can make the final point of a 90-degree bend closer to the rear axle
        # than the first point and leaves Pure Pursuit with no forward samples.
        rear_offset = float(self.get_parameter("lidar_to_rear_axle_m").value)
        controller_points = [
            (float(x) + rear_offset, float(y)) for x, y in midpoints
        ]
        pts = self.order_connected_points(
            controller_points,
            maximum_gap_m=float(
                self.get_parameter("path_gap_fill_max_m").value
            ),
        )
        if len(pts) == 1:
            return self.accept_new_path(pts)
        values = np.asarray(pts, dtype=np.float64)
        segment_lengths = np.linalg.norm(np.diff(values, axis=0), axis=1)
        keep = np.concatenate(([True], segment_lengths > 1.0e-5))
        values = values[keep]
        if len(values) < min_midpoints:
            return self.hold_previous_path()
        segment_lengths = np.linalg.norm(np.diff(values, axis=0), axis=1)
        arclength = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        if float(arclength[-1]) < float(
            self.get_parameter("min_path_span_m").value
        ):
            return self.hold_previous_path()
        sample_count = max(2, int(self.get_parameter("path_sample_count").value))
        interp_s = np.linspace(0.0, float(arclength[-1]), sample_count)
        method = str(self.get_parameter("path_interpolation_method").value).lower().strip()
        if method in ("cubic", "cubic_spline", "spline"):
            interp_x = self.cubic_interpolate(
                arclength,
                values[:, 0],
                interp_s,
            )
            interp_y = self.cubic_interpolate(
                arclength,
                values[:, 1],
                interp_s,
            )
        else:
            interp_x = np.interp(interp_s, arclength, values[:, 0])
            interp_y = np.interp(interp_s, arclength, values[:, 1])
        new_path = [(float(x), float(y)) for x, y in zip(interp_x, interp_y)]
        return self.accept_new_path(new_path)

    @staticmethod
    def path_max_heading_step_deg(path: Sequence[Point2]) -> float:
        if len(path) < 3:
            return 0.0
        points = np.asarray(path, dtype=np.float64)
        deltas = np.diff(points, axis=0)
        usable = np.linalg.norm(deltas, axis=1) > 1.0e-5
        deltas = deltas[usable]
        if len(deltas) < 2:
            return 0.0
        headings = np.unwrap(np.arctan2(deltas[:, 1], deltas[:, 0]))
        return float(np.max(np.abs(np.diff(headings))) * 180.0 / math.pi)

    @staticmethod
    def path_heading_demand_deg(
        path: Sequence[Point2],
        maximum_distance_m: float,
    ) -> float:
        if len(path) < 2:
            return 0.0
        headings = []
        for first, second in zip(path, path[1:]):
            if min(math.hypot(*first), math.hypot(*second)) > maximum_distance_m:
                continue
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            if math.hypot(dx, dy) > 1.0e-5:
                headings.append(abs(math.degrees(math.atan2(dy, dx))))
        return max(headings, default=0.0)

    @staticmethod
    def path_arc_length(
        path: Sequence[Point2],
        start_index: int = 0,
    ) -> float:
        points = list(path)
        if len(points) < 2:
            return 0.0
        start = int(np.clip(start_index, 0, len(points) - 1))
        return float(
            sum(
                math.hypot(
                    second[0] - first[0],
                    second[1] - first[1],
                )
                for first, second in zip(points[start:], points[start + 1 :])
            )
        )

    @staticmethod
    def path_nearest_index(path: Sequence[Point2]) -> int:
        if not path:
            return 0
        return min(
            range(len(path)),
            key=lambda index: math.hypot(*path[index]),
        )

    def path_start_is_consistent(self, path: Sequence[Point2]) -> bool:
        """Return false when a path loops materially closer to the rear axle.

        Monotonic x is intentionally not required, so legitimate 90-degree
        and S-shaped paths remain valid. A late sample is rejected only when it
        is materially closer to the controller origin than sample zero. Such a
        curve cannot safely be followed from sample zero and matches the 8/19
        opposite-steering failure (nearest index 84 of 100).
        """
        points = list(path)
        if len(points) < 2:
            return False
        nearest_index = self.path_nearest_index(points)
        max_fraction = float(
            np.clip(
                self.get_parameter(
                    "path_start_nearest_max_fraction"
                ).value,
                0.0,
                1.0,
            )
        )
        maximum_start_index = max(
            1,
            int(math.floor((len(points) - 1) * max_fraction)),
        )
        if nearest_index <= maximum_start_index:
            return True
        margin = max(
            0.0,
            float(
                self.get_parameter(
                    "path_start_nearest_distance_margin_m"
                ).value
            ),
        )
        start_distance = math.hypot(*points[0])
        nearest_distance = math.hypot(*points[nearest_index])
        return nearest_distance + margin >= start_distance

    @staticmethod
    def path_heading_at_distance_deg(
        path: Sequence[Point2],
        probe_distance_m: float,
    ) -> float:
        """Return a forward path tangent without assuming monotonic x."""
        points = list(path)
        if len(points) < 2:
            return 0.0
        target = max(0.0, float(probe_distance_m))
        travelled = 0.0
        selected = None
        for first, second in zip(points, points[1:]):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            segment = math.hypot(dx, dy)
            if segment <= 1.0e-6:
                continue
            selected = (dx, dy)
            if travelled + segment >= target:
                break
            travelled += segment
        if selected is None:
            return 0.0
        return float(math.degrees(math.atan2(selected[1], selected[0])))

    @staticmethod
    def heading_difference_deg(first: float, second: float) -> float:
        return float(abs((float(first) - float(second) + 180.0) % 360.0 - 180.0))

    def normalize_control_path(self, path: Sequence[Point2]) -> List[Point2]:
        """Orient an open path from the rear axle toward the visible course.

        Upstream geometry normally already provides this ordering.  The
        endpoint check is a final guard for sparse/reconstructed paths and does
        not sort by x, so a legitimate 90-degree or S-shaped path is retained.
        """
        values: List[Point2] = []
        for raw_x, raw_y in path:
            point = (float(raw_x), float(raw_y))
            if not values or math.hypot(
                point[0] - values[-1][0],
                point[1] - values[-1][1],
            ) > 1.0e-6:
                values.append(point)
        reversed_path = False
        if len(values) >= 2:
            margin = max(
                0.0,
                float(
                    self.get_parameter(
                        "path_orientation_start_margin_m"
                    ).value
                ),
            )
            first_distance = math.hypot(*values[0])
            last_distance = math.hypot(*values[-1])
            if last_distance + margin < first_distance:
                values.reverse()
                reversed_path = True
        self.last_path_orientation_reversed = reversed_path
        return values

    @staticmethod
    def point_at_path_distance(
        path: Sequence[Point2],
        target_distance_m: float,
    ) -> Point2:
        if not path:
            return (0.0, 0.0)
        points = list(path)
        # Accepted paths are explicitly ordered in the rear-axle controller
        # frame.  Searching the whole curve for the Euclidean nearest sample can
        # select the final sample of a hairpin and reduce the forward arc to
        # zero, producing alternating saturated steering commands.
        start_index = 0
        previous = (0.0, 0.0)
        distance = 0.0
        for point in points[start_index:]:
            segment = math.hypot(
                point[0] - previous[0],
                point[1] - previous[1],
            )
            if segment > 1.0e-6 and distance + segment >= target_distance_m:
                ratio = float(
                    np.clip(
                        (target_distance_m - distance) / segment,
                        0.0,
                        1.0,
                    )
                )
                return (
                    previous[0] + ratio * (point[0] - previous[0]),
                    previous[1] + ratio * (point[1] - previous[1]),
                )
            distance += segment
            previous = point
        return points[-1]

    def accept_new_path(self, new_path: List[Point2]) -> List[Point2]:
        new_path = self.normalize_control_path(new_path)
        if not new_path:
            self.last_path_rejection_reason = "empty"
            return self.hold_previous_path()
        if self.prev_path is not None and len(self.prev_path) > 1 and len(new_path) == 1:
            self.last_path_rejection_reason = "single_point"
            return self.hold_previous_path()
        source = str(getattr(self, "midpoint_source", "none"))
        single_boundary_sources = (
            "left_offset",
            "right_offset",
            "nearest_gate",
        )
        final_left_guard = (
            bool(
                self.get_parameter(
                    "cone_turn_sequence_guard_enabled"
                ).value
            )
            and self.final_left_phase_active(include_pending=True)
        )
        if final_left_guard and source in single_boundary_sources:
            maximum_start_distance = max(
                0.0,
                float(
                    self.get_parameter(
                        "final_left_single_path_max_start_m"
                    ).value
                ),
            )
            start_distance = math.hypot(*new_path[0])
            if (
                maximum_start_distance > 0.0
                and start_distance > maximum_start_distance
            ):
                self.last_path_rejection_reason = "distant_single_boundary"
                self.invalid_inferred_recovery_active = True
                return self.hold_previous_path()

        # Once the measured left->right sequence has started turning back, no
        # path source is allowed to reintroduce a right-hand target.  This also
        # covers paired_reacquire, which produced +15 deg in bag 022406.
        if final_left_guard:
            reversal_margin = max(
                0.0,
                float(
                    self.get_parameter(
                        "final_left_reversal_target_margin_m"
                    ).value
                ),
            )
            if self.path_target_lateral(new_path) < -reversal_margin:
                self.last_path_rejection_reason = "final_left_reversal"
                self.invalid_inferred_recovery_active = True
                return self.hold_previous_path()
        minimum_control_arc = max(
            0.0,
            float(self.get_parameter("path_min_control_arc_m").value),
        )
        remaining_arc = self.path_arc_length(new_path)
        if (
            self.prev_path is not None
            and len(new_path) > 1
            and remaining_arc < minimum_control_arc
        ):
            self.last_path_rejection_reason = "short_path"
            return self.hold_previous_path()
        single_boundary_minimum_arc = max(
            minimum_control_arc,
            float(
                self.get_parameter(
                    "single_boundary_min_control_arc_m"
                ).value
            ),
        )
        if (
            source in single_boundary_sources
            and remaining_arc < single_boundary_minimum_arc
        ):
            self.last_path_rejection_reason = "short_single_boundary"
            self.invalid_inferred_recovery_active = True
            return self.hold_previous_path()
        maximum_heading_step = max(
            0.0,
            float(self.get_parameter("path_max_heading_range_deg").value),
        )
        if (
            maximum_heading_step > 0.0
            and self.path_max_heading_step_deg(new_path) > maximum_heading_step
        ):
            self.last_path_rejection_reason = "heading_step"
            return self.hold_previous_path()
        inferred_sources = (
            "left_offset",
            "right_offset",
            "nearest_gate",
            "paired_sparse",
            "paired_reacquire",
        )
        previous_source = str(
            getattr(self, "accepted_path_source", "none")
        )
        probe_distance = max(
            0.05,
            float(
                self.get_parameter(
                    "path_heading_probe_distance_m"
                ).value
            ),
        )
        new_heading = self.path_heading_at_distance_deg(
            new_path,
            probe_distance,
        )
        previous_heading = getattr(self, "last_path_heading_deg", None)
        heading_threshold = max(
            0.0,
            float(
                self.get_parameter(
                    "path_heading_jump_threshold_deg"
                ).value
            ),
        )
        heading_required = max(
            1,
            int(
                self.get_parameter(
                    "path_heading_jump_required_frames"
                ).value
            ),
        )
        heading_jump = (
            previous_heading is not None
            and self.heading_difference_deg(new_heading, previous_heading)
            > heading_threshold
        )
        if heading_jump and heading_required > 1:
            tolerance = max(
                0.0,
                float(
                    self.get_parameter(
                        "path_heading_confirmation_tolerance_deg"
                    ).value
                ),
            )
            pending_heading = getattr(self, "pending_path_heading_deg", None)
            if (
                pending_heading is not None
                and self.heading_difference_deg(new_heading, pending_heading)
                <= tolerance
            ):
                self.pending_path_heading_frames = int(
                    getattr(self, "pending_path_heading_frames", 0)
                ) + 1
            else:
                self.pending_path_heading_deg = new_heading
                self.pending_path_heading_frames = 1
            if self.pending_path_heading_frames < heading_required:
                self.last_path_rejection_reason = "heading_pending"
                return self.hold_previous_path()
        else:
            self.pending_path_heading_deg = None
            self.pending_path_heading_frames = 0
        previous_target = (
            self.path_target_lateral(self.prev_path)
            if self.prev_path is not None
            else getattr(self, "last_path_target_lateral", None)
        )
        new_target = self.path_target_lateral(new_path)
        reacquire_active = int(
            getattr(self, "paired_reacquire_frames", 0)
        ) > 0
        reacquire_jump = (
            previous_target is not None
            and abs(float(new_target) - float(previous_target))
            >= max(
                0.0,
                float(
                    self.get_parameter(
                        "paired_reacquire_min_target_jump_m"
                    ).value
                ),
            )
        )
        paired_reacquire = source == "paired" and (
            reacquire_active
            or (previous_source in inferred_sources and reacquire_jump)
        )
        if paired_reacquire:
            self.paired_reacquire_frames = int(
                getattr(self, "paired_reacquire_frames", 0)
            ) + 1
            required = max(
                1,
                int(self.get_parameter("paired_reacquire_frames").value),
            )
            if self.paired_reacquire_frames <= required:
                source = "paired_reacquire"
                self.midpoint_source = source
            else:
                self.paired_reacquire_frames = 0
        elif source != "paired_reacquire":
            self.paired_reacquire_frames = 0
        inferred = source in inferred_sources
        self.last_raw_path_target_lateral = new_target
        self.path_target_limit_applied = False
        reversal_threshold = max(
            0.0,
            float(self.get_parameter("path_reversal_min_lateral_m").value),
        )
        reversal_required = max(
            1,
            int(self.get_parameter("path_reversal_required_frames").value),
        )
        abrupt_reversal = (
            previous_target is not None
            and abs(float(previous_target)) >= reversal_threshold
            and abs(float(new_target)) >= reversal_threshold
            and float(previous_target) * float(new_target) < 0.0
        )
        if abrupt_reversal and reversal_required > 1:
            new_sign = 1 if new_target > 0.0 else -1
            if getattr(self, "pending_path_reversal_sign", 0) == new_sign:
                self.pending_path_reversal_frames = getattr(
                    self,
                    "pending_path_reversal_frames",
                    0,
                ) + 1
            else:
                self.pending_path_reversal_sign = new_sign
                self.pending_path_reversal_frames = 1
            if self.pending_path_reversal_frames < reversal_required:
                self.last_path_rejection_reason = "reversal_pending"
                return self.hold_previous_path()
        else:
            self.pending_path_reversal_sign = 0
            self.pending_path_reversal_frames = 0
        if inferred and previous_target is not None:
            # A single boundary is geometrically underconstrained. Limit its
            # lateral output continuously instead of rejecting it until the
            # old path expires; expiry followed by an unrestricted replacement
            # was the source of the visible one-frame steering jump.
            now = float(
                getattr(self, "current_scan_time", None) or time.monotonic()
            )
            rate_parameter = (
                "paired_reacquire_path_target_rate_mps"
                if source == "paired_reacquire"
                else "inferred_path_target_rate_mps"
            )
            rate = max(
                0.0,
                float(
                    self.get_parameter(
                        rate_parameter
                    ).value
                ),
            )
            if rate > 0.0:
                maximum_dt = max(
                    0.0,
                    float(
                        self.get_parameter(
                            "path_target_rate_max_dt_sec"
                        ).value
                    ),
                )
                previous_time = getattr(self, "last_path_update_time", None)
                dt = maximum_dt if previous_time is None else float(
                    np.clip(now - previous_time, 0.0, maximum_dt)
                )
                max_step = rate * dt
            else:
                # Compatibility fallback for old configurations.
                max_step = max(
                    0.0,
                    float(
                        self.get_parameter(
                            "inferred_max_path_target_jump_m"
                        ).value
                    ),
                )
            delta = new_target - previous_target
            if abs(delta) > max_step:
                self.path_target_limit_applied = True
                limited_target = previous_target + math.copysign(
                    max_step,
                    delta,
                )
                # The path target is selected by arc distance, so translating
                # y also changes the origin-to-path arc slightly. Iterate the
                # translation to honour the requested lateral-rate bound.
                for _ in range(4):
                    current_target = self.path_target_lateral(new_path)
                    residual = current_target - limited_target
                    if abs(residual) <= 1.0e-4:
                        break
                    new_path = [
                        (float(x), float(y) - residual)
                        for x, y in new_path
                    ]
                new_target = self.path_target_lateral(new_path)
        elif self.prev_path is not None:
            max_jump = float(
                self.get_parameter("max_path_target_jump_m").value
            )
            if abs(new_target - previous_target) > max_jump:
                self.last_path_rejection_reason = "target_jump"
                return self.hold_previous_path()
        if (
            source in inferred_sources
            and not self.path_start_is_consistent(new_path)
        ):
            # Do not centre a malformed one-sided curve by translation and
            # then follow it from the wrong end. Keep the previous valid turn;
            # if it ages out, bounded blind recovery bridges the interval until
            # paired geometry returns.
            self.last_path_rejection_reason = "late_nearest_return"
            self.invalid_inferred_recovery_active = True
            return self.hold_previous_path()
        self.path_miss_count = 0
        self.path_is_held = False
        self.prev_path = new_path
        self.accepted_path_source = source
        self.last_path_target_lateral = new_target
        self.last_output_path_target_lateral = new_target
        self.last_path_heading_deg = self.path_heading_at_distance_deg(
            new_path,
            probe_distance,
        )
        self.last_path_nearest_index = self.path_nearest_index(new_path)
        self.last_path_remaining_arc_m = self.path_arc_length(new_path)
        updated_at = float(
            getattr(self, "current_scan_time", None) or time.monotonic()
        )
        previous_accept_time = getattr(self, "last_path_accept_time", None)
        accepted_at = (
            float(previous_accept_time)
            if (
                bool(getattr(self, "ego_tracker_prediction_only", False))
                and previous_accept_time is not None
            )
            else updated_at
        )
        self.last_path_accept_time = accepted_at
        self.last_path_update_time = updated_at
        self.geometry_reference_path = list(new_path)
        self.geometry_reference_time = accepted_at
        self.pending_path_reversal_sign = 0
        self.pending_path_reversal_frames = 0
        self.pending_path_heading_deg = None
        self.pending_path_heading_frames = 0
        self.last_path_rejection_reason = "none"
        self.invalid_inferred_recovery_active = False
        return self.prev_path

    def hold_previous_path(self) -> List[Point2]:
        self.path_miss_count += 1
        now = float(
            getattr(self, "current_scan_time", None) or time.monotonic()
        )
        hold_sec = max(
            0.0,
            float(self.get_parameter("path_hold_sec").value),
        )
        hold_frames = max(0, int(self.get_parameter("path_hold_frames").value))
        accepted_at = getattr(self, "last_path_accept_time", None)
        if hold_sec > 0.0 and accepted_at is not None:
            within_hold = now - float(accepted_at) <= hold_sec
        else:
            # Compatibility fallback for old configs and pure test harnesses.
            within_hold = self.path_miss_count <= hold_frames
        if self.prev_path is not None and within_hold:
            self.path_is_held = True
            return self.prev_path
        self.path_is_held = False
        self.prev_path = None
        return []

    def path_target_lateral(self, path: Sequence[Point2]) -> float:
        if not path:
            return 0.0
        lookahead = float(self.get_parameter("lookahead_min_m").value)
        return float(self.point_at_path_distance(path, lookahead)[1])

    def cubic_interpolate(self, xs: np.ndarray, ys: np.ndarray, interp_x: np.ndarray) -> np.ndarray:
        if xs.size < 3:
            return np.interp(interp_x, xs, ys)
        try:
            if CubicSpline is not None:
                spline_fn = CubicSpline(xs, ys)
                return np.asarray(spline_fn(interp_x), dtype=np.float32)
            return self.natural_cubic_interpolate(xs, ys, interp_x)
        except Exception as exc:
            self.get_logger().warn(f"Cubic spline failed, falling back to linear interpolation: {exc}")
            return np.interp(interp_x, xs, ys)

    @staticmethod
    def natural_cubic_interpolate(xs: np.ndarray, ys: np.ndarray, interp_x: np.ndarray) -> np.ndarray:
        n = xs.size
        if n < 3:
            return np.interp(interp_x, xs, ys)

        h = np.diff(xs.astype(np.float64))
        if np.any(h <= 1e-6):
            return np.interp(interp_x, xs, ys)

        alpha = np.zeros(n, dtype=np.float64)
        for i in range(1, n - 1):
            alpha[i] = 3.0 / h[i] * (ys[i + 1] - ys[i]) - 3.0 / h[i - 1] * (ys[i] - ys[i - 1])

        lower = np.ones(n, dtype=np.float64)
        mu = np.zeros(n, dtype=np.float64)
        z = np.zeros(n, dtype=np.float64)
        for i in range(1, n - 1):
            lower[i] = 2.0 * (xs[i + 1] - xs[i - 1]) - h[i - 1] * mu[i - 1]
            if abs(lower[i]) <= 1e-9:
                return np.interp(interp_x, xs, ys)
            mu[i] = h[i] / lower[i]
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / lower[i]

        b = np.zeros(n - 1, dtype=np.float64)
        c = np.zeros(n, dtype=np.float64)
        d = np.zeros(n - 1, dtype=np.float64)
        for j in range(n - 2, -1, -1):
            c[j] = z[j] - mu[j] * c[j + 1]
            b[j] = (ys[j + 1] - ys[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j])

        indices = np.searchsorted(xs, interp_x, side="right") - 1
        indices = np.clip(indices, 0, n - 2)
        dx = interp_x - xs[indices]
        interp_y = ys[indices] + b[indices] * dx + c[indices] * dx * dx + d[indices] * dx * dx * dx
        return interp_y.astype(np.float32)

    def pure_pursuit(self, path: Sequence[Point2]) -> float:
        if not path:
            return 0.0
        lookahead = self.dynamic_lookahead(path)
        near_angle = self.pure_pursuit_at_distance(path, lookahead)
        far_distance = min(
            float(self.get_parameter("lookahead_max_m").value),
            lookahead + float(self.get_parameter("far_preview_distance_m").value),
        )
        far_angle = self.pure_pursuit_at_distance(path, far_distance)
        far_weight = float(np.clip(self.get_parameter("far_preview_weight").value, 0.0, 1.0))
        gain = max(0.0, float(self.get_parameter("steering_gain").value))
        angle_cmd = gain * (
            (1.0 - far_weight) * near_angle + far_weight * far_angle
        )
        heading_demand = self.path_heading_demand_deg(
            path,
            float(self.get_parameter("lookahead_max_m").value),
        )
        if (
            bool(self.get_parameter("sharp_curve_preview_enabled").value)
            and heading_demand
            >= float(
                self.get_parameter(
                    "sharp_curve_heading_threshold_deg"
                ).value
            )
        ):
            close_angle = self.pure_pursuit_at_distance(
                path,
                max(
                    0.30,
                    min(lookahead, 0.72 * lookahead),
                ),
            )
            reference = near_angle if abs(near_angle) > 1.0e-3 else close_angle
            same_direction = [
                candidate
                for candidate in (close_angle, near_angle, far_angle)
                if reference == 0.0 or candidate * reference >= 0.0
            ]
            if same_direction:
                sharp_angle = max(same_direction, key=abs)
                angle_cmd = max(
                    0.0,
                    float(
                        self.get_parameter(
                            "sharp_curve_steering_gain"
                        ).value
                    ),
                ) * sharp_angle
        limit = float(self.get_parameter("max_steer_cmd").value)
        return float(np.clip(angle_cmd, -limit, limit))

    def pure_pursuit_at_distance(self, path: Sequence[Point2], lookahead: float) -> float:
        tx, ty = self.point_at_path_distance(path, lookahead)
        ld = max(1e-6, math.hypot(tx, ty))
        alpha = math.atan2(ty, tx)
        wheelbase = float(self.get_parameter("wheelbase_m").value)
        delta = math.atan2(2.0 * wheelbase * math.sin(alpha), ld)
        # Publish the physical target wheel angle. drive_manager performs the
        # existing Xycar steering-servo command conversion exactly once.
        return -math.degrees(delta)

    def dynamic_lookahead(self, path: Sequence[Point2]) -> float:
        if len(path) < 2:
            return float(self.get_parameter("lookahead_min_m").value)
        length = 0.0
        for a, b in zip(path[:-1], path[1:]):
            length += math.hypot(b[0] - a[0], b[1] - a[1])
        ld = float(self.get_parameter("lookahead_scale").value) * length
        return float(
            np.clip(
                ld,
                float(self.get_parameter("lookahead_min_m").value),
                float(self.get_parameter("lookahead_max_m").value),
            )
        )

    def update_cone_turn_phase(self, angle: float) -> None:
        """Latch the measured left -> right -> large-left cone sequence.

        The first raw turn-back observation arms a pending phase.  A second
        observation may come from the held copy of that same left path; this
        is deliberate because the next sparse scan is exactly where the
        2026-08-20 failures replaced the turn with a right/straight path.
        """
        if not bool(
            self.get_parameter("cone_turn_sequence_guard_enabled").value
        ):
            return

        phase = str(getattr(self, "cone_turn_phase", "approach"))
        source = str(getattr(self, "midpoint_source", "none"))
        value = float(angle)
        pending_armed_now = False
        fresh_trusted = (
            not self.path_is_held
            and source not in ("none", "reacquire_pending")
        )
        first_left = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_first_left_enter_steer_deg"
                ).value
            ),
        )
        right_turn = max(
            0.0,
            float(self.get_parameter("cone_right_enter_steer_deg").value),
        )
        pending_left = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_pending_steer_deg"
                ).value
            ),
        )
        commit_left = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_enter_steer_deg"
                ).value
            ),
        )

        if phase == "approach" and fresh_trusted and value <= -first_left:
            self.cone_turn_phase = "first_left"
            return
        if phase == "first_left" and fresh_trusted and value >= right_turn:
            self.cone_turn_phase = "right"
            now_value = getattr(self, "current_scan_time", None)
            self.right_turn_started_at = (
                time.monotonic() if now_value is None else float(now_value)
            )
            return
        if (
            phase == "right"
            and fresh_trusted
            and value <= -pending_left
        ):
            now_value = getattr(self, "current_scan_time", None)
            now = time.monotonic() if now_value is None else float(now_value)
            started_at = getattr(self, "right_turn_started_at", None)
            minimum_hold = max(
                0.0,
                float(
                    self.get_parameter(
                        "cone_right_minimum_hold_sec"
                    ).value
                ),
            )
            if started_at is not None and now - float(started_at) < minimum_hold:
                return
            self.cone_turn_phase = "final_left_pending"
            self.final_left_pending_frames = (
                1 if value <= -commit_left else 0
            )
            now_value = getattr(self, "current_scan_time", None)
            now = time.monotonic() if now_value is None else float(now_value)
            self.final_left_pending_started_at = now
            self.final_left_strong_hold_started_at = None
            self.final_left_peak_steering = min(0.0, value)
            self.final_left_measured_peak_steering = min(0.0, value)
            self.final_left_release_frames = 0
            self.final_left_release_reference_steering = 0.0
            self.final_left_release_reference_time = None
            self.final_left_exit_frames = 0
            phase = "final_left_pending"
            pending_armed_now = True

        if phase == "final_left_pending":
            if value <= -commit_left and not pending_armed_now:
                # Do not count an unrelated stale command before pending is
                # armed, but permit its own held left path to confirm entry.
                self.final_left_pending_frames = int(
                    getattr(self, "final_left_pending_frames", 0)
                ) + 1
            required = max(
                1,
                int(
                    self.get_parameter(
                        "cone_final_left_pending_frames"
                    ).value
                ),
            )
            if self.final_left_pending_frames >= required:
                self.cone_turn_phase = "final_left_committed"
                now_value = getattr(self, "current_scan_time", None)
                now = time.monotonic() if now_value is None else float(now_value)
                self.final_left_committed_at = now
                self.final_left_fallback_started_at = None
                self.final_left_strong_hold_started_at = None
                self.final_left_peak_steering = min(
                    float(getattr(self, "final_left_peak_steering", 0.0)),
                    value,
                )
                self.final_left_measured_peak_steering = min(
                    float(
                        getattr(
                            self,
                            "final_left_measured_peak_steering",
                            0.0,
                        )
                    ),
                    value,
                )
                self.final_left_release_frames = 0
                self.final_left_exit_frames = 0
            return

        exit_angle = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_exit_steer_deg"
                ).value
            ),
        )
        exit_required = max(
            1,
            int(
                self.get_parameter(
                    "cone_final_left_exit_frames"
                ).value
            ),
        )

        if phase == "final_left_releasing":
            if fresh_trusted and source == "paired":
                now_value = getattr(self, "current_scan_time", None)
                now = time.monotonic() if now_value is None else float(now_value)
                self.final_left_release_reference_steering = value
                self.final_left_release_reference_time = now
                if value >= -exit_angle:
                    self.final_left_exit_frames = int(
                        getattr(self, "final_left_exit_frames", 0)
                    ) + 1
                else:
                    self.final_left_exit_frames = 0
                if self.final_left_exit_frames >= exit_required:
                    self.cone_turn_phase = "complete"
            else:
                self.final_left_exit_frames = 0
            return

        if phase not in ("final_left", "final_left_committed"):
            return

        self.final_left_peak_steering = min(
            float(getattr(self, "final_left_peak_steering", 0.0)),
            value,
        )
        self.final_left_measured_peak_steering = min(
            float(
                getattr(
                    self,
                    "final_left_measured_peak_steering",
                    0.0,
                )
            ),
            value,
        )
        minimum_release_peak = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_release_min_peak_deg"
                ).value
            ),
        )
        guarded_release_peak = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_release_guarded_min_peak_deg"
                ).value
            ),
        )
        guarded_hold_sec = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_release_guarded_hold_sec"
                ).value
            ),
        )
        release_margin = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_release_margin_deg"
                ).value
            ),
        )
        now_value = getattr(self, "current_scan_time", None)
        now = time.monotonic() if now_value is None else float(now_value)
        strong_hold_started_at = getattr(
            self,
            "final_left_strong_hold_started_at",
            None,
        )
        guarded_turn_delivered = bool(
            self.final_left_measured_peak_steering
            <= -guarded_release_peak
            and strong_hold_started_at is not None
            and now - float(strong_hold_started_at) >= guarded_hold_sec
        )
        measured_peak_ready = bool(
            self.final_left_measured_peak_steering
            <= -minimum_release_peak
        )
        release_observed = (
            fresh_trusted
            and source == "paired"
            and (measured_peak_ready or guarded_turn_delivered)
            and value
            >= self.final_left_measured_peak_steering + release_margin
        )
        if release_observed:
            self.final_left_release_frames = int(
                getattr(self, "final_left_release_frames", 0)
            ) + 1
        elif fresh_trusted:
            self.final_left_release_frames = 0
        release_required = max(
            1,
            int(
                self.get_parameter(
                    "cone_final_left_release_required_frames"
                ).value
            ),
        )
        if (
            self.final_left_release_frames >= release_required
        ):
            self.cone_turn_phase = "final_left_releasing"
            self.final_left_release_reference_steering = value
            self.final_left_release_reference_time = now
            self.final_left_exit_frames = 0

    def final_left_fallback_angle(self, angle: float) -> float:
        """Ramp a committed blind/ambiguous final left through measured data."""
        minimum = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_min_hold_steer_deg"
                ).value
            ),
        )
        target = max(
            minimum,
            float(
                self.get_parameter(
                    "cone_final_left_fallback_target_deg"
                ).value
            ),
        )
        maximum = max(
            target,
            float(
                self.get_parameter(
                    "cone_final_left_fallback_max_steer_deg"
                ).value
            ),
        )
        rate = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_fallback_ramp_deg_per_sec"
                ).value
            ),
        )
        now_value = getattr(self, "current_scan_time", None)
        now = time.monotonic() if now_value is None else float(now_value)
        if getattr(self, "final_left_fallback_started_at", None) is None:
            self.final_left_fallback_started_at = now
        elapsed = max(0.0, now - float(self.final_left_fallback_started_at))
        magnitude = min(maximum, minimum + rate * elapsed)
        value = min(
            float(angle),
            float(getattr(self, "final_left_peak_steering", 0.0)),
            -magnitude,
        )
        strong_output = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_release_guarded_output_deg"
                ).value
            ),
        )
        if value <= -strong_output:
            if getattr(
                self,
                "final_left_strong_hold_started_at",
                None,
            ) is None:
                self.final_left_strong_hold_started_at = now
        self.final_left_peak_steering = min(
            float(getattr(self, "final_left_peak_steering", 0.0)),
            value,
        )
        return value

    def final_left_release_recovery_angle(self) -> float:
        """Continue an already observed unwind without restoring peak lock."""
        reference = float(
            getattr(self, "final_left_release_reference_steering", 0.0)
        )
        now_value = getattr(self, "current_scan_time", None)
        now = time.monotonic() if now_value is None else float(now_value)
        reference_time = getattr(
            self,
            "final_left_release_reference_time",
            None,
        )
        if reference_time is None:
            self.final_left_release_reference_time = now
            reference_time = now
        rate = max(
            0.0,
            float(
                self.get_parameter(
                    "cone_final_left_release_recovery_rate_deg_per_sec"
                ).value
            ),
        )
        elapsed = max(0.0, now - float(reference_time))
        return min(0.0, reference + rate * elapsed)

    def apply_cone_turn_phase_guard(self, angle: float) -> float:
        """Keep an ambiguous/held final-left path from unwinding the turn."""
        value = float(angle)
        if not bool(
            self.get_parameter(
                "cone_turn_sequence_guard_enabled"
            ).value
        ):
            return value

        phase = str(getattr(self, "cone_turn_phase", "approach"))
        if phase == "right":
            started_at = getattr(self, "right_turn_started_at", None)
            now_value = getattr(self, "current_scan_time", None)
            now = time.monotonic() if now_value is None else float(now_value)
            minimum_hold = max(
                0.0,
                float(
                    self.get_parameter(
                        "cone_right_minimum_hold_sec"
                    ).value
                ),
            )
            if started_at is not None and now - float(started_at) < minimum_hold:
                minimum_right = max(
                    0.0,
                    float(
                        self.get_parameter(
                            "cone_right_enter_steer_deg"
                        ).value
                    ),
                )
                return max(value, minimum_right)
        if phase == "final_left_pending":
            pending_hold = max(
                0.0,
                float(
                    self.get_parameter(
                        "cone_final_left_pending_hold_steer_deg"
                    ).value
                ),
            )
            return min(value, -pending_hold)

        source = str(getattr(self, "midpoint_source", "none"))
        if phase == "final_left_releasing":
            if not self.path_is_held and source == "paired":
                self.final_left_fallback_started_at = None
                return value
            return self.final_left_release_recovery_angle()
        if phase not in ("final_left", "final_left_committed"):
            return value

        # The successful indoor run unwound on fresh bilateral geometry from
        # -18.7 to -2.7 deg.  The failed run offered the same signal (-13.6 to
        # -8.6), but the old minimum-hold check overwrote it with -22 deg.
        if not self.path_is_held and source == "paired":
            minimum_hold = max(
                0.0,
                float(
                    self.get_parameter(
                        "cone_final_left_min_hold_steer_deg"
                    ).value
                ),
            )
            minimum_release_peak = max(
                0.0,
                float(
                    self.get_parameter(
                        "cone_final_left_release_min_peak_deg"
                    ).value
                ),
            )
            measured_peak = float(
                getattr(self, "final_left_measured_peak_steering", 0.0)
            )
            if value <= -minimum_hold or measured_peak <= -minimum_release_peak:
                self.final_left_fallback_started_at = None
                self.final_left_peak_steering = min(
                    float(getattr(self, "final_left_peak_steering", 0.0)),
                    value,
                )
                return value
            return self.final_left_fallback_angle(value)

        inferred = source in (
            "left_offset",
            "right_offset",
            "nearest_gate",
            "paired_sparse",
            "paired_reacquire",
            "reacquire_pending",
            "none",
        )
        if self.path_is_held or inferred:
            return self.final_left_fallback_angle(value)

        self.final_left_fallback_started_at = None
        self.final_left_peak_steering = min(
            float(getattr(self, "final_left_peak_steering", 0.0)),
            value,
        )
        return value

    def stabilize_steering(self, angle: float) -> float:
        self.steering_history.append(float(angle))
        if len(self.steering_history) < self.steering_history.maxlen:
            filtered = float(angle)
        else:
            filtered = float(np.median(np.asarray(self.steering_history, dtype=np.float32)))

        source = str(getattr(self, "midpoint_source", "none"))
        inferred = source in (
            "left_offset",
            "right_offset",
            "nearest_gate",
            "paired_sparse",
            "paired_reacquire",
        )
        rate_parameter = (
            "inferred_steering_max_rate_deg_per_sec"
            if inferred
            else "steering_max_rate_deg_per_sec"
        )
        rate = max(0.0, float(self.get_parameter(rate_parameter).value))
        now_value = getattr(self, "current_scan_time", None)
        now = time.monotonic() if now_value is None else float(now_value)
        previous_time = getattr(self, "last_steering_time", None)
        if rate > 0.0 and previous_time is not None:
            maximum_dt = max(
                0.0,
                float(
                    self.get_parameter("steering_rate_max_dt_sec").value
                ),
            )
            dt = float(np.clip(now - previous_time, 0.0, maximum_dt))
            max_delta = rate * dt
        else:
            # Preserve old YAML compatibility when a rate is not configured.
            parameter = (
                "inferred_steering_max_delta_deg"
                if inferred
                else "steering_max_delta_deg"
            )
            max_delta = max(0.0, float(self.get_parameter(parameter).value))
        if self.stabilized_steering is not None and max_delta > 0.0:
            filtered = float(
                np.clip(
                    filtered,
                    self.stabilized_steering - max_delta,
                    self.stabilized_steering + max_delta,
                )
            )
        self.stabilized_steering = filtered
        self.last_steering_time = now
        return filtered

    def held_path_progress(self) -> float:
        if not self.path_is_held:
            return 0.0
        hold_sec = max(
            0.0,
            float(self.get_parameter("path_hold_sec").value),
        )
        accepted_at = getattr(self, "last_path_accept_time", None)
        now_value = getattr(self, "current_scan_time", None)
        if hold_sec > 0.0 and accepted_at is not None:
            now = time.monotonic() if now_value is None else float(now_value)
            return float(
                np.clip((now - float(accepted_at)) / hold_sec, 0.0, 1.0)
            )
        hold_frames = max(1, int(self.get_parameter("path_hold_frames").value))
        return min(1.0, float(self.path_miss_count) / float(hold_frames))

    def path_confidence(self, midpoint_count: int) -> float:
        if self.path_is_held:
            remaining = 1.0 - self.held_path_progress()
            return max(0.21, float(self.get_parameter("min_confidence").value) * remaining)
        source = str(getattr(self, "midpoint_source", "none"))
        if source == "paired":
            return min(1.0, 0.5 + 0.2 * max(0, midpoint_count - 1))
        if source in ("left_offset", "right_offset", "paired_reacquire"):
            confidence = 0.38 + 0.06 * max(0, midpoint_count - 1)
            cap = float(self.get_parameter("single_boundary_confidence_cap").value)
            return float(np.clip(confidence, 0.0, cap))
        if source == "nearest_gate":
            cap = float(self.get_parameter("nearest_gate_confidence_cap").value)
            return float(np.clip(0.30 + 0.04 * max(0, midpoint_count - 1), 0.0, cap))
        return min(0.40, 0.30 + 0.05 * max(0, midpoint_count - 1))

    def preview_steering_demand(self, path: Sequence[Point2]) -> Tuple[float, float]:
        """Return maximum future steering demand and available path distance.

        Sampling several Pure Pursuit targets catches an S-bend even when its
        far endpoint has returned close to the vehicle centreline.  This value
        is used only for speed planning; the vehicle-tested steering controller
        and its 0.7~1.25 m lookahead remain unchanged.
        """
        if not path:
            return 0.0, 0.0
        distances = np.asarray([math.hypot(x, y) for x, y in path], dtype=np.float32)
        available = float(np.max(distances)) if distances.size else 0.0
        near = max(0.05, float(self.get_parameter("cone_speed_preview_near_m").value))
        far = max(near, float(self.get_parameter("cone_speed_preview_far_m").value))
        far = min(far, available)
        if far < near:
            return 0.0, available
        sample_count = max(2, int(self.get_parameter("cone_speed_preview_samples").value))
        samples = np.linspace(near, far, sample_count)
        demand = max(abs(self.pure_pursuit_at_distance(path, float(distance))) for distance in samples)
        return float(demand), available

    def straight_boost_ratio(
        self,
        angle: float,
        preview_angle: float,
        confidence: float,
        available_distance: float,
    ) -> float:
        if not bool(self.get_parameter("cone_preview_speed_control").value):
            return 0.0
        if self.path_is_held or str(getattr(self, "midpoint_source", "none")) != "paired":
            return 0.0
        if float(confidence) < float(self.get_parameter("cone_straight_boost_min_confidence").value):
            return 0.0
        if available_distance < float(
            self.get_parameter("cone_straight_boost_min_path_distance_m").value
        ):
            return 0.0

        demand = max(abs(float(angle)), abs(float(preview_angle)))
        full_angle = max(
            0.0,
            float(self.get_parameter("cone_straight_boost_full_angle_deg").value),
        )
        max_angle = max(
            full_angle + 1e-6,
            float(self.get_parameter("cone_straight_boost_max_angle_deg").value),
        )
        return float(np.clip((max_angle - demand) / (max_angle - full_angle), 0.0, 1.0))

    def compute_speed(
        self,
        angle: float,
        confidence: float = 1.0,
        path: Optional[Sequence[Point2]] = None,
    ) -> float:
        minimum = float(self.get_parameter("cone_min_drive_speed").value)
        base = max(minimum, float(self.get_parameter("cone_speed").value))
        limit = max(
            1.0,
            float(self.get_parameter("cone_speed_full_steer_deg").value),
        )
        preview_angle = 0.0
        available_distance = 0.0
        if bool(self.get_parameter("cone_preview_speed_control").value) and path:
            preview_angle, available_distance = self.preview_steering_demand(path)
        # Brake from the strongest steering demand visible ahead rather than
        # waiting until the current command becomes large inside the bend.
        steering_demand = max(abs(float(angle)), abs(float(preview_angle)))
        steer_ratio = min(1.0, steering_demand / limit)
        exponent = max(0.1, float(self.get_parameter("cone_speed_steer_exponent").value))
        speed = minimum + (base - minimum) * (1.0 - steer_ratio ** exponent)

        min_confidence = float(self.get_parameter("min_confidence").value)
        confidence_ratio = float(
            np.clip(
                (float(confidence) - min_confidence) / max(1e-6, 1.0 - min_confidence),
                0.0,
                1.0,
            )
        )
        confidence_floor = float(
            np.clip(self.get_parameter("cone_speed_confidence_floor_ratio").value, 0.0, 1.0)
        )
        confidence_speed = minimum + (base - minimum) * (
            confidence_floor + (1.0 - confidence_floor) * confidence_ratio
        )
        speed = min(speed, confidence_speed)

        if self.path_is_held:
            hold_ratio = self.held_path_progress()
            speed = minimum + (speed - minimum) * (1.0 - 0.55 * hold_ratio)
        source = str(getattr(self, "midpoint_source", "none"))
        if source in ("left_offset", "right_offset", "paired_reacquire"):
            speed = min(speed, float(self.get_parameter("single_boundary_max_speed").value))
        elif source == "nearest_gate":
            speed = minimum

        # Add speed above the proven curve profile only on a measured straight.
        # The boost fades continuously from full to zero between the configured
        # angle thresholds, while inferred and held paths never receive it.
        boost_ratio = self.straight_boost_ratio(
            angle,
            preview_angle,
            confidence,
            available_distance,
        )
        boost_speed = max(base, float(self.get_parameter("cone_straight_boost_speed").value))
        speed += boost_ratio * (boost_speed - base)
        return max(minimum, speed)

    def publish_blind_recovery(self, cluster_count: int) -> bool:
        if (
            bool(getattr(self, "ego_cone_tracking_enabled", False))
            and int(getattr(self, "ego_tracker_observed_count", 0))
            + int(getattr(self, "ego_tracker_predicted_count", 0))
            <= 0
        ):
            # Once the bounded odom landmark set expires, unrelated raw LiDAR
            # clusters must not restart a second vehicle-frame recovery window.
            return False
        max_frames = max(0, int(self.get_parameter("blind_recovery_frames").value))
        max_seconds = max(
            0.0,
            float(self.get_parameter("blind_recovery_max_sec").value),
        )
        invalid_inferred = bool(
            getattr(self, "invalid_inferred_recovery_active", False)
        )
        if invalid_inferred:
            max_frames = max(
                max_frames,
                int(
                    self.get_parameter(
                        "invalid_inferred_recovery_frames"
                    ).value
                ),
            )
            max_seconds = max(
                max_seconds,
                float(
                    self.get_parameter(
                        "invalid_inferred_recovery_max_sec"
                    ).value
                ),
            )
        if self.final_left_phase_active():
            max_frames = max(
                max_frames,
                int(
                    self.get_parameter(
                        "cone_final_left_recovery_frames"
                    ).value
                ),
            )
            max_seconds = max(
                max_seconds,
                float(
                    self.get_parameter(
                        "cone_final_left_recovery_max_sec"
                    ).value
                ),
            )
        min_clusters = max(1, int(self.get_parameter("blind_recovery_min_clusters").value))
        if not self.had_valid_path or cluster_count < min_clusters or self.blind_recovery_count >= max_frames:
            return False

        now_value = getattr(self, "current_scan_time", None)
        now = time.monotonic() if now_value is None else float(now_value)
        if getattr(self, "blind_recovery_started_at", None) is None:
            self.blind_recovery_started_at = now
        if (
            max_seconds > 0.0
            and now - float(self.blind_recovery_started_at) > max_seconds
        ):
            return False

        self.blind_recovery_count += 1
        decay_parameter = (
            "invalid_inferred_recovery_steer_decay"
            if invalid_inferred
            else "blind_recovery_steer_decay"
        )
        decay = float(
            np.clip(self.get_parameter(decay_parameter).value, 0.0, 1.0)
        )
        base_angle = float(self.last_valid_steering)
        phase = str(getattr(self, "cone_turn_phase", "approach"))
        if phase == "final_left_releasing":
            angle = self.final_left_release_recovery_angle()
            speed_parameter = "cone_final_left_recovery_speed"
        elif self.final_left_phase_active():
            angle = self.final_left_fallback_angle(base_angle)
            speed_parameter = "cone_final_left_recovery_speed"
        else:
            angle = base_angle * (decay ** self.blind_recovery_count)
            speed_parameter = (
                "invalid_inferred_recovery_speed"
                if invalid_inferred
                else "blind_recovery_speed"
            )
        speed = min(
            float(self.get_parameter("cone_min_drive_speed").value),
            max(0.0, float(self.get_parameter(speed_parameter).value)),
        )
        confidence = max(0.21, float(self.get_parameter("min_confidence").value) * 0.75)
        self.last_recovery_angle = angle
        self.last_recovery_speed = speed
        self.publish_cmd(angle, speed, confidence)
        return True

    def publish_diagnostics(
        self,
        *,
        raw_angle: float,
        output_angle: float,
        speed: float,
        confidence: float,
        scan_dt: float,
        scan_stamp_ns: int,
        lidar_count: int,
        fused_count: int,
        planning_count: int,
        left_count: int,
        right_count: int,
        path_count: int,
    ) -> None:
        """Publish every control stage needed to diagnose a delayed turn.

        Float array schema:
        [raw_steer, output_steer, speed, confidence, scan_dt, scan_hz,
         yolo_age, lidar_n, fused_n, planning_n, left_n, right_n, path_n,
         learned_width, source_code, held, raw_path_y, output_path_y,
         path_limit_applied, geometry_unlocked, path_nearest_index,
         path_remaining_arc, path_orientation_reversed, turn_phase_code,
         observed_pair_width, temporary_wide_pair, measured_final_left_peak,
         guarded_strong_hold_age, final_left_release_frames,
         ego_pose_valid, ego_observed_tracks, ego_predicted_tracks,
         ego_max_prediction_age, ego_prediction_authoritative,
         ego_pose_jump_reset]
        """
        source = str(getattr(self, "midpoint_source", "none"))
        source_codes = {
            "none": 0.0,
            "paired": 1.0,
            "left_offset": 2.0,
            "right_offset": 3.0,
            "nearest_gate": 4.0,
            "paired_sparse": 5.0,
            "reacquire_pending": 6.0,
            "paired_reacquire": 7.0,
        }
        phase_codes = {
            "approach": 0.0,
            "first_left": 1.0,
            "right": 2.0,
            "final_left_pending": 3.0,
            "final_left": 4.0,
            "final_left_committed": 4.0,
            "final_left_releasing": 5.0,
            "complete": 6.0,
        }
        phase = str(getattr(self, "cone_turn_phase", "approach"))
        yolo_age = self.yolo_box_age_sec(scan_stamp_ns)
        if not math.isfinite(yolo_age):
            yolo_age = -1.0
        strong_hold_started_at = getattr(
            self,
            "final_left_strong_hold_started_at",
            None,
        )
        now_value = getattr(self, "current_scan_time", None)
        now = time.monotonic() if now_value is None else float(now_value)
        strong_hold_age = (
            max(0.0, now - float(strong_hold_started_at))
            if strong_hold_started_at is not None
            else -1.0
        )
        scan_hz = 1.0 / scan_dt if scan_dt > 1.0e-6 else 0.0
        message = Float32MultiArray()
        message.data = [
            float(raw_angle),
            float(output_angle),
            float(speed),
            float(confidence),
            float(scan_dt),
            float(scan_hz),
            float(yolo_age),
            float(lidar_count),
            float(fused_count),
            float(planning_count),
            float(left_count),
            float(right_count),
            float(path_count),
            self.effective_corridor_width(),
            source_codes.get(source, -1.0),
            1.0 if self.path_is_held else 0.0,
            float(getattr(self, "last_raw_path_target_lateral", 0.0)),
            float(getattr(self, "last_output_path_target_lateral", 0.0)),
            1.0 if getattr(self, "path_target_limit_applied", False) else 0.0,
            1.0
            if getattr(self, "geometry_planning_unlocked", False)
            else 0.0,
            float(getattr(self, "last_path_nearest_index", 0)),
            float(getattr(self, "last_path_remaining_arc_m", 0.0)),
            1.0
            if getattr(self, "last_path_orientation_reversed", False)
            else 0.0,
            phase_codes.get(phase, -1.0),
            float(getattr(self, "last_observed_pair_width_m", 0.0)),
            1.0 if getattr(self, "wide_pair_active", False) else 0.0,
            float(
                getattr(self, "final_left_measured_peak_steering", 0.0)
            ),
            float(strong_hold_age),
            float(getattr(self, "final_left_release_frames", 0)),
            1.0 if getattr(self, "ego_tracker_pose_valid", False) else 0.0,
            float(getattr(self, "ego_tracker_observed_count", 0)),
            float(getattr(self, "ego_tracker_predicted_count", 0)),
            float(
                getattr(self, "ego_tracker_max_prediction_age_sec", 0.0)
            ),
            1.0
            if getattr(
                self,
                "ego_tracker_prediction_authoritative",
                False,
            )
            else 0.0,
            1.0
            if getattr(self, "ego_tracker_pose_jump_reset", False)
            else 0.0,
        ]
        self.diagnostics_pub.publish(message)

        status = (
            f"source={source} lidar={lidar_count} fused={fused_count} "
            f"plan={planning_count} LR={left_count}/{right_count} "
            f"path={path_count} width={self.effective_corridor_width():.3f}m "
            f"yolo_age={yolo_age:.3f}s scan={scan_hz:.1f}Hz "
            f"steer={raw_angle:+.1f}->{output_angle:+.1f}deg "
            f"path_y={getattr(self, 'last_raw_path_target_lateral', 0.0):+.3f}->"
            f"{getattr(self, 'last_output_path_target_lateral', 0.0):+.3f}m "
            f"held={int(self.path_is_held)} "
            "ego="
            f"{int(getattr(self, 'ego_tracker_pose_valid', False))}/"
            f"{getattr(self, 'ego_tracker_observed_count', 0)}/"
            f"{getattr(self, 'ego_tracker_predicted_count', 0)}/"
            f"{getattr(self, 'ego_tracker_max_prediction_age_sec', 0.0):.2f}s/"
            f"{int(getattr(self, 'ego_tracker_prediction_authoritative', False))} "
            f"phase={phase} "
            f"pair_width={getattr(self, 'last_observed_pair_width_m', 0.0):.3f}m "
            f"wide={int(getattr(self, 'wide_pair_active', False))} "
            "release="
            f"{getattr(self, 'final_left_measured_peak_steering', 0.0):+.1f}/"
            f"{strong_hold_age:.2f}s/"
            f"{getattr(self, 'final_left_release_frames', 0)} "
            f"reject={getattr(self, 'last_path_rejection_reason', 'none')}"
        )
        self.status_pub.publish(String(data=status))
        now_value = getattr(self, "current_scan_time", None)
        now = time.monotonic() if now_value is None else float(now_value)
        log_period = max(
            0.0,
            float(self.get_parameter("diagnostics_log_period_sec").value),
        )
        if now - self.last_diagnostics_log_time >= log_period:
            self.get_logger().info(f"[cone] {status}")
            self.last_diagnostics_log_time = now

    def publish_cmd(self, angle: float, speed: float, confidence: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(angle), float(speed), float(confidence)]
        self.cmd_pub.publish(msg)

    def publish_clusters(self, clusters: Sequence[Point2]) -> None:
        self.publish_cluster_array(self.cluster_pub, clusters)

    def publish_cluster_array(
        self,
        publisher,
        clusters: Sequence[Point2],
    ) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        for x, y in clusters:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            msg.poses.append(pose)
        publisher.publish(msg)

    def publish_path(self, path: Sequence[Point2]) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "rear_axle"
        for x, y in path:
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            msg.poses.append(pose)
        self.path_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConeNode()
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
