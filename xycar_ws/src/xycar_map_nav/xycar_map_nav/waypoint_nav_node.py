#!/usr/bin/env python3
"""Follow map waypoints and hand control to cone or vehicle rules."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import signal
import time

from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from my_rule_msgs.msg import ObjectDetectionArray
from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .control_core import (
    alignment_limited_speed_command,
    DynamicAvoidanceConfig,
    DynamicVehicleRule,
    filtered_steering_command,
    forward_backward_velocity_profile,
    minimum_effective_speed_command,
    minimum_profile_value_ahead,
    nearest_path_index,
    path_curvature_profile,
    pure_pursuit_command,
    rate_limited_speed_command,
    stanley_path_command,
    steering_command_for_curvature,
)
from .grid_planner import (
    PlannedRoute,
    load_map_grid,
    load_path_csv,
    plan_waypoint_route,
    smooth_path_points,
)
from .localization_guard import (
    compose_planar,
    GuardResult,
    LocalizationJumpGuard,
    PlanarTransform,
)
from .lidar_obstacle import (
    detect_path_obstacle,
    LidarBypassConfig,
    LidarBypassState,
    LidarObstacleBypassRule,
    LidarPathObstacle,
    LidarPathObstacleConfig,
)
from .mission_supervisor import (
    camera_box_lidar_sector,
    MissionMode,
    MissionSupervisor,
    MissionSupervisorConfig,
    scan_sector_distance,
)


VALID_CONTROLLERS = {
    "global_path",
    "cone_rule",
    "dynamic_vehicle_rule",
}


@dataclass
class RouteWaypoint:
    name: str
    x: float
    y: float
    controller_to_next: str = "global_path"


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class WaypointNavNode(Node):
    def __init__(self) -> None:
        super().__init__("xycar_waypoint_nav")
        self._declare_parameters()
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.drive_enabled = bool(self.get_parameter("drive_enabled").value)
        self.closed_route = bool(self.get_parameter("closed_route").value)

        self.map_yaml = Path(
            str(self.get_parameter("map_yaml").value)
        ).expanduser().resolve()
        self.waypoints_yaml = Path(
            str(self.get_parameter("waypoints_yaml").value)
        ).expanduser().resolve()
        path_csv_value = str(self.get_parameter("path_csv").value)
        self.path_csv = (
            Path(path_csv_value).expanduser().resolve()
            if path_csv_value.strip()
            else None
        )
        output_value = str(self.get_parameter("capture_output_yaml").value)
        self.capture_output_yaml = (
            Path(output_value).expanduser().resolve()
            if output_value.strip()
            else None
        )

        self.map_grid = load_map_grid(
            self.map_yaml,
            inflation_radius_m=float(
                self.get_parameter("inflation_radius_m").value
            ),
            unknown_is_occupied=bool(
                self.get_parameter("unknown_is_occupied").value
            ),
            ignore_occupancy=bool(
                self.get_parameter("ignore_map_occupancy").value
            ),
        )
        self.waypoints: list[RouteWaypoint] = []
        self.route: PlannedRoute | None = None
        self.speed_profile_mps: tuple[float, ...] = ()
        self.current_path_index: int | None = None
        self.route_complete = False

        self.command_inputs = [
            float(value)
            for value in self.get_parameter("steering_map_commands").value
        ]
        self.curvature_inputs = [
            float(value)
            for value in self.get_parameter("steering_map_curvatures").value
        ]
        self.dynamic_rule = DynamicVehicleRule(
            DynamicAvoidanceConfig(
                right_offset_m=float(
                    self.get_parameter("dynamic_right_offset_m").value
                ),
                left_offset_m=float(
                    self.get_parameter("dynamic_left_offset_m").value
                ),
                offset_rate_mps=float(
                    self.get_parameter("dynamic_offset_rate_mps").value
                ),
                speed_limit_command=float(
                    self.get_parameter(
                        "dynamic_speed_limit_command"
                    ).value
                ),
                front_trigger=int(
                    self.get_parameter("dynamic_front_trigger").value
                ),
                behind_passed_trigger=int(
                    self.get_parameter(
                        "dynamic_behind_passed_trigger"
                    ).value
                ),
                behind_return_trigger=int(
                    self.get_parameter(
                        "dynamic_behind_return_trigger"
                    ).value
                ),
                clear_reset_sec=float(
                    self.get_parameter("dynamic_clear_reset_sec").value
                ),
            )
        )
        self.lidar_obstacle_config = LidarPathObstacleConfig(
            detect_distance_m=float(
                self.get_parameter(
                    "lidar_obstacle_detect_distance_m"
                ).value
            ),
            minimum_distance_m=float(
                self.get_parameter(
                    "lidar_obstacle_minimum_distance_m"
                ).value
            ),
            path_corridor_half_width_m=float(
                self.get_parameter(
                    "lidar_obstacle_path_half_width_m"
                ).value
            ),
            minimum_cluster_points=int(
                self.get_parameter(
                    "lidar_obstacle_minimum_cluster_points"
                ).value
            ),
            maximum_scan_index_gap=int(
                self.get_parameter(
                    "lidar_obstacle_maximum_scan_index_gap"
                ).value
            ),
            maximum_cluster_gap_m=float(
                self.get_parameter(
                    "lidar_obstacle_maximum_cluster_gap_m"
                ).value
            ),
            minimum_cluster_width_m=float(
                self.get_parameter(
                    "lidar_obstacle_minimum_cluster_width_m"
                ).value
            ),
            maximum_cluster_width_m=float(
                self.get_parameter(
                    "lidar_obstacle_maximum_cluster_width_m"
                ).value
            ),
            side_probe_inner_m=float(
                self.get_parameter(
                    "lidar_obstacle_side_probe_inner_m"
                ).value
            ),
            side_probe_outer_m=float(
                self.get_parameter(
                    "lidar_obstacle_side_probe_outer_m"
                ).value
            ),
            lidar_x_m=float(
                self.get_parameter("lidar_obstacle_lidar_x_m").value
            ),
            lidar_y_m=float(
                self.get_parameter("lidar_obstacle_lidar_y_m").value
            ),
            lidar_yaw_rad=math.radians(
                float(
                    self.get_parameter(
                        "lidar_obstacle_lidar_yaw_deg"
                    ).value
                )
            ),
        )
        self.lidar_obstacle_rule = LidarObstacleBypassRule(
            LidarBypassConfig(
                required_frames=int(
                    self.get_parameter(
                        "lidar_obstacle_required_frames"
                    ).value
                ),
                left_offset_m=float(
                    self.get_parameter(
                        "lidar_obstacle_left_offset_m"
                    ).value
                ),
                right_offset_m=float(
                    self.get_parameter(
                        "lidar_obstacle_right_offset_m"
                    ).value
                ),
                offset_rate_mps=float(
                    self.get_parameter(
                        "lidar_obstacle_offset_rate_mps"
                    ).value
                ),
                speed_limit_command=float(
                    self.get_parameter(
                        "lidar_obstacle_speed_limit_command"
                    ).value
                ),
                estimated_obstacle_length_m=float(
                    self.get_parameter(
                        "lidar_obstacle_estimated_length_m"
                    ).value
                ),
                post_obstacle_margin_m=float(
                    self.get_parameter(
                        "lidar_obstacle_post_margin_m"
                    ).value
                ),
                clear_hold_sec=float(
                    self.get_parameter(
                        "lidar_obstacle_clear_hold_sec"
                    ).value
                ),
                return_deadband_m=float(
                    self.get_parameter(
                        "lidar_obstacle_return_deadband_m"
                    ).value
                ),
                centered_lateral_deadband_m=float(
                    self.get_parameter(
                        "lidar_obstacle_center_deadband_m"
                    ).value
                ),
            )
        )
        self.latest_lidar_obstacle: LidarPathObstacle | None = None
        self.dynamic_counts = (0, 0, 0, 0)
        self.dynamic_counts_time = 0.0
        self.semantic_vehicle_count = 0
        self.cone_mode_active = False
        self.cone_command = (0.0, 0.0)
        self.cone_command_confidence = 0.0
        self.cone_command_time = 0.0
        self.latest_scan: LaserScan | None = None
        self.emergency_front_range_m = float("inf")
        self.scan_time = 0.0
        self.mission_trigger_mode = str(
            self.get_parameter("mission_trigger_mode").value
        ).strip().lower()
        if self.mission_trigger_mode not in {"semantic", "route_segments"}:
            raise ValueError(
                "mission_trigger_mode must be semantic or route_segments"
            )
        self.mission_supervisor = MissionSupervisor(
            MissionSupervisorConfig(
                semantic_timeout_sec=float(
                    self.get_parameter("semantic_timeout_sec").value
                ),
                lidar_timeout_sec=float(
                    self.get_parameter("mission_lidar_timeout_sec").value
                ),
                cone_camera_required_frames=int(
                    self.get_parameter(
                        "cone_camera_required_frames"
                    ).value
                ),
                cone_camera_min_count=int(
                    self.get_parameter("cone_camera_min_count").value
                ),
                cone_camera_min_confidence=float(
                    self.get_parameter(
                        "cone_camera_min_confidence"
                    ).value
                ),
                cone_lidar_min_count=int(
                    self.get_parameter("cone_lidar_min_count").value
                ),
                cone_entry_distance_m=float(
                    self.get_parameter("cone_entry_distance_m").value
                ),
                cone_minimum_duration_sec=float(
                    self.get_parameter(
                        "cone_minimum_duration_sec"
                    ).value
                ),
                cone_clear_hold_sec=float(
                    self.get_parameter("cone_clear_hold_sec").value
                ),
                cone_processing_hold_sec=float(
                    self.get_parameter(
                        "cone_processing_hold_sec"
                    ).value
                ),
                vehicle_camera_required_frames=int(
                    self.get_parameter(
                        "vehicle_camera_required_frames"
                    ).value
                ),
                vehicle_camera_min_count=int(
                    self.get_parameter("vehicle_camera_min_count").value
                ),
                vehicle_camera_min_confidence=float(
                    self.get_parameter(
                        "vehicle_camera_min_confidence"
                    ).value
                ),
                vehicle_entry_distance_m=float(
                    self.get_parameter(
                        "vehicle_entry_distance_m"
                    ).value
                ),
                vehicle_minimum_duration_sec=float(
                    self.get_parameter(
                        "vehicle_minimum_duration_sec"
                    ).value
                ),
                vehicle_clear_hold_sec=float(
                    self.get_parameter(
                        "vehicle_clear_hold_sec"
                    ).value
                ),
                traffic_required_frames=int(
                    self.get_parameter(
                        "traffic_required_frames"
                    ).value
                ),
                traffic_control_enabled=bool(
                    self.get_parameter(
                        "traffic_control_enabled"
                    ).value
                ),
                lane_intervention_enabled=bool(
                    self.get_parameter(
                        "lane_intervention_enabled"
                    ).value
                ),
            )
        )
        self.last_published_mode = ""
        self.latest_speed_mps = 0.0
        self.latest_yaw_rate_radps = 0.0
        self.odom_time = 0.0
        self.localization_odom_frame_id = str(
            self.get_parameter("localization_odom_frame_id").value
        )
        self.localization_guard_enabled = bool(
            self.get_parameter("localization_guard_enabled").value
        )
        self.localization_guard = LocalizationJumpGuard(
            maximum_translation_jump_m=float(
                self.get_parameter(
                    "localization_maximum_translation_jump_m"
                ).value
            ),
            maximum_yaw_jump_rad=math.radians(
                float(
                    self.get_parameter(
                        "localization_maximum_yaw_jump_deg"
                    ).value
                )
            ),
            fault_after_sec=float(
                self.get_parameter(
                    "localization_jump_fault_after_sec"
                ).value
            ),
        )
        self.latest_guard_result: GuardResult | None = None
        self.last_guard_state = ""
        self.last_steering_command = 0.0
        self.last_speed_command = 0.0
        self.last_control_time = time.monotonic()
        self.drive_start_time = self.last_control_time + max(
            0.0,
            float(self.get_parameter("drive_start_delay_sec").value),
        )
        self.require_route_localization = bool(
            self.get_parameter("require_route_localization").value
        )
        self.route_localization_ready = (
            not self.require_route_localization
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(
            PathMessage,
            str(self.get_parameter("path_topic").value),
            transient_qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("waypoint_marker_topic").value),
            transient_qos,
        )
        self.mode_pub = self.create_publisher(
            String, str(self.get_parameter("control_mode_topic").value), 10
        )
        self.mission_reason_pub = self.create_publisher(
            String,
            str(self.get_parameter("mission_reason_topic").value),
            10,
        )
        self.cone_processing_enabled = False
        self.cone_processing_pub = self.create_publisher(
            Bool,
            str(
                self.get_parameter(
                    "cone_processing_enabled_topic"
                ).value
            ),
            transient_qos,
        )
        self.debug_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("debug_topic").value),
            10,
        )
        self.lidar_obstacle_debug_pub = self.create_publisher(
            Float32MultiArray,
            str(
                self.get_parameter(
                    "lidar_obstacle_debug_topic"
                ).value
            ),
            10,
        )
        self.localization_guard_status_pub = self.create_publisher(
            String,
            str(
                self.get_parameter(
                    "localization_guard_status_topic"
                ).value
            ),
            10,
        )
        self.localization_guard_debug_pub = self.create_publisher(
            Float32MultiArray,
            str(
                self.get_parameter(
                    "localization_guard_debug_topic"
                ).value
            ),
            10,
        )
        self.shadow_motor_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("shadow_motor_topic").value),
            10,
        )
        self.motor_pub = None
        if self.drive_enabled:
            self.motor_pub = self.create_publisher(
                Float32MultiArray,
                str(self.get_parameter("motor_topic").value),
                10,
            )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            sensor_qos,
        )
        self.create_subscription(
            Int32MultiArray,
            str(self.get_parameter("dynamic_counts_topic").value),
            self._on_dynamic_counts,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("cone_mode_topic").value),
            self._on_cone_mode,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("cone_command_topic").value),
            self._on_cone_command,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("cone_cluster_topic").value),
            self._on_cone_clusters,
            sensor_qos,
        )
        self.create_subscription(
            ObjectDetectionArray,
            str(self.get_parameter("object_detections_topic").value),
            self._on_object_detections,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            str(
                self.get_parameter(
                    "route_localization_ready_topic"
                ).value
            ),
            self._on_route_localization_ready,
            transient_qos,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("clicked_point_topic").value),
            self._on_clicked_point,
            10,
        )
        self.create_service(Trigger, "~/undo_waypoint", self._undo_waypoint)
        self.create_service(
            Trigger, "~/clear_waypoints", self._clear_waypoints
        )
        self.create_service(Trigger, "~/reload_route", self._reload_route)
        self.create_service(
            Trigger,
            "~/reset_localization_guard",
            self._reset_localization_guard,
        )

        self._load_route()
        rate = max(2.0, float(self.get_parameter("control_rate_hz").value))
        self.control_timer = self.create_timer(
            1.0 / rate,
            self._control_step,
        )
        self.cone_processing_timer = self.create_timer(
            0.10,
            self._update_cone_processing_gate,
        )
        self._publish_cone_processing_enabled(False, force=True)
        mode = "DRIVE" if self.drive_enabled else "SHADOW"
        self.get_logger().info(
            f"{mode}: {len(self.waypoints)} waypoints, "
            f"{len(self.route.points) if self.route else 0} path points; "
            "authority=global/cone-rule/dynamic-vehicle-rule/"
            "lidar-obstacle-rule, "
            f"mission_trigger={self.mission_trigger_mode}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("waypoints_yaml", "")
        self.declare_parameter("path_csv", "")
        self.declare_parameter("capture_output_yaml", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("closed_route", True)
        self.declare_parameter("drive_enabled", False)
        self.declare_parameter("drive_start_delay_sec", 0.0)
        self.declare_parameter("require_route_localization", False)
        self.declare_parameter(
            "route_localization_ready_topic",
            "/map_nav/route_localization/ready",
        )
        self.declare_parameter("inflation_radius_m", 0.23)
        self.declare_parameter("unknown_is_occupied", True)
        self.declare_parameter("ignore_map_occupancy", False)
        self.declare_parameter("waypoint_snap_radius_m", 0.45)
        self.declare_parameter("path_spacing_m", 0.10)
        self.declare_parameter("path_smoothing_enabled", True)
        self.declare_parameter("path_smoothing_data_weight", 0.0005)
        self.declare_parameter("path_smoothing_weight", 0.45)
        self.declare_parameter("path_smoothing_iterations", 2500)
        self.declare_parameter("path_smoothing_anchor_weight", 0.02)
        self.declare_parameter(
            "path_smoothing_maximum_deviation_m", -1.0
        )
        self.declare_parameter("csv_path_smoothing_enabled", True)
        self.declare_parameter("csv_path_smoothing_data_weight", 0.10)
        self.declare_parameter("csv_path_smoothing_weight", 0.40)
        self.declare_parameter("csv_path_smoothing_iterations", 100)
        self.declare_parameter("clearance_cost_weight", 3.0)
        self.declare_parameter("clearance_cost_decay_m", 0.35)
        self.declare_parameter("goal_tolerance_m", 0.25)
        self.declare_parameter("cruise_speed_command", 5.0)
        self.declare_parameter("minimum_speed_command", 3.0)
        self.declare_parameter("fixed_speed_command", -1.0)
        self.declare_parameter("speed_planner_mode", "local")
        self.declare_parameter("speed_profile_max_accel_mps2", 0.80)
        self.declare_parameter("speed_profile_max_decel_mps2", 1.40)
        self.declare_parameter(
            "speed_profile_curvature_window_m", 0.55
        )
        self.declare_parameter(
            "speed_profile_curvature_smoothing_points", 2
        )
        self.declare_parameter(
            "speed_profile_braking_preview_sec", 0.30
        )
        self.declare_parameter("curve_slowdown_curvature_per_m", 0.8)
        self.declare_parameter("speed_gain_mps_per_command", 0.080612)
        self.declare_parameter("maximum_lateral_accel_mps2", 0.90)
        self.declare_parameter("speed_alignment_cross_track_soft_m", 0.05)
        self.declare_parameter("speed_alignment_cross_track_hard_m", 0.20)
        self.declare_parameter("speed_alignment_heading_soft_rad", 0.08)
        self.declare_parameter("speed_alignment_heading_hard_rad", 0.35)
        self.declare_parameter(
            "speed_acceleration_rate_command_per_sec", 5.0
        )
        self.declare_parameter(
            "speed_deceleration_rate_command_per_sec", 30.0
        )
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("pose_timeout_sec", 0.25)
        self.declare_parameter("odom_topic", "/slam/odom")
        self.declare_parameter("odom_timeout_sec", 0.35)
        self.declare_parameter(
            "localization_odom_frame_id", "slam_odom"
        )
        self.declare_parameter("localization_guard_enabled", True)
        self.declare_parameter(
            "localization_maximum_translation_jump_m", 0.20
        )
        self.declare_parameter(
            "localization_maximum_yaw_jump_deg", 5.0
        )
        self.declare_parameter(
            "localization_jump_fault_after_sec", 0.30
        )
        self.declare_parameter(
            "localization_guard_status_topic",
            "/map_nav/localization_guard/status",
        )
        self.declare_parameter(
            "localization_guard_debug_topic",
            "/map_nav/localization_guard/debug",
        )
        self.declare_parameter("wheelbase_m", 0.32)
        self.declare_parameter("front_axle_offset_m", 0.16)
        self.declare_parameter("steering_delay_sec", 0.10)
        self.declare_parameter("stanley_gain", 1.20)
        self.declare_parameter("stanley_softening_mps", 0.45)
        self.declare_parameter("stanley_heading_gain", 1.0)
        self.declare_parameter("straight_stanley_gain", 0.45)
        self.declare_parameter("straight_stanley_softening_mps", 0.80)
        self.declare_parameter("straight_stanley_heading_gain", 0.55)
        self.declare_parameter("curvature_feedforward_gain", 0.50)
        self.declare_parameter("path_heading_window_m", 0.40)
        self.declare_parameter("path_heading_preview_m", 0.0)
        self.declare_parameter("path_curvature_window_m", 0.55)
        self.declare_parameter("path_curvature_preview_m", 0.10)
        self.declare_parameter("speed_curvature_preview_m", -1.0)
        self.declare_parameter("maximum_steering_angle_rad", 0.62)
        self.declare_parameter("straight_curvature_threshold_per_m", 0.16)
        self.declare_parameter(
            "straight_steering_rate_command_per_sec", 90.0
        )
        self.declare_parameter(
            "curve_steering_rate_command_per_sec", 300.0
        )
        self.declare_parameter("straight_steering_filter_sec", 0.16)
        self.declare_parameter("curve_steering_filter_sec", 0.04)
        self.declare_parameter("straight_yaw_rate_damping_sec", 0.45)
        self.declare_parameter("curve_yaw_rate_damping_sec", 0.12)
        self.declare_parameter("curve_controller", "stanley")
        self.declare_parameter("curve_pure_pursuit_lookahead_m", 0.30)
        self.declare_parameter(
            "curve_pure_pursuit_speed_preview_sec", 0.12
        )
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter(
            "shadow_motor_topic", "/map_nav/xycar_motor_shadow"
        )
        self.declare_parameter("path_topic", "/map_nav/global_path")
        self.declare_parameter(
            "waypoint_marker_topic", "/map_nav/waypoints"
        )
        self.declare_parameter(
            "control_mode_topic", "/map_nav/control_mode"
        )
        self.declare_parameter(
            "mission_reason_topic", "/map_nav/mission_reason"
        )
        self.declare_parameter("debug_topic", "/map_nav/debug")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_required_for_drive", True)
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("mission_trigger_mode", "semantic")
        self.declare_parameter("semantic_timeout_sec", 0.75)
        self.declare_parameter("mission_lidar_timeout_sec", 0.50)
        self.declare_parameter(
            "object_detections_topic", "/my_rule/object_detections"
        )
        self.declare_parameter(
            "cone_cluster_topic", "/my_rule/cone_clusters"
        )
        self.declare_parameter("cone_class_names", ["cone"])
        self.declare_parameter(
            "vehicle_class_names", ["obstacle_vehicle", "car"]
        )
        self.declare_parameter(
            "vehicle_camera_lidar_hfov_deg", 60.0
        )
        self.declare_parameter(
            "vehicle_camera_lidar_padding_deg", 3.0
        )
        self.declare_parameter("object_lidar_min_points", 2)
        self.declare_parameter("cone_camera_required_frames", 2)
        self.declare_parameter("cone_camera_min_count", 1)
        self.declare_parameter("cone_camera_min_confidence", 0.50)
        self.declare_parameter("cone_lidar_min_count", 1)
        self.declare_parameter("cone_entry_distance_m", 0.50)
        self.declare_parameter("cone_minimum_duration_sec", 1.0)
        self.declare_parameter("cone_clear_hold_sec", 0.70)
        self.declare_parameter("cone_processing_hold_sec", 1.50)
        self.declare_parameter(
            "cone_processing_enabled_topic",
            "/my_rule/cone_processing_enabled",
        )
        self.declare_parameter("cone_command_min_confidence", 0.30)
        self.declare_parameter("cone_input_is_physical_angle", True)
        self.declare_parameter(
            "cone_steering_actual_deg",
            [0.0, 4.0, 10.0, 16.0, 26.0],
        )
        self.declare_parameter(
            "cone_steering_commands",
            [0.0, 10.0, 20.0, 30.0, 42.0],
        )
        self.declare_parameter("vehicle_camera_required_frames", 2)
        self.declare_parameter("vehicle_camera_min_count", 1)
        self.declare_parameter("vehicle_camera_min_confidence", 0.45)
        self.declare_parameter("vehicle_entry_distance_m", 2.40)
        self.declare_parameter("vehicle_minimum_duration_sec", 0.50)
        self.declare_parameter("vehicle_clear_hold_sec", 0.50)
        self.declare_parameter("traffic_control_enabled", True)
        self.declare_parameter("traffic_required_frames", 2)
        self.declare_parameter("traffic_min_confidence", 0.50)
        self.declare_parameter(
            "traffic_signal_max_center_y_ratio", 0.55
        )
        self.declare_parameter("lane_intervention_enabled", False)
        self.declare_parameter(
            "dynamic_counts_topic", "/yolo_obstacle/stable_counts"
        )
        self.declare_parameter("dynamic_detector_timeout_sec", 0.6)
        self.declare_parameter("dynamic_detector_required", True)
        self.declare_parameter("dynamic_right_offset_m", 0.28)
        self.declare_parameter("dynamic_left_offset_m", 0.28)
        self.declare_parameter("dynamic_offset_rate_mps", 0.35)
        self.declare_parameter("dynamic_speed_limit_command", 4.0)
        self.declare_parameter("dynamic_front_trigger", 1)
        self.declare_parameter("dynamic_behind_passed_trigger", 1)
        self.declare_parameter("dynamic_behind_return_trigger", 2)
        self.declare_parameter("dynamic_clear_reset_sec", 0.5)
        self.declare_parameter("lidar_obstacle_fallback_enabled", True)
        self.declare_parameter(
            "lidar_obstacle_debug_topic",
            "/map_nav/lidar_obstacle_debug",
        )
        self.declare_parameter(
            "lidar_obstacle_detect_distance_m", 1.50
        )
        self.declare_parameter(
            "lidar_obstacle_minimum_distance_m", 0.18
        )
        self.declare_parameter(
            "lidar_obstacle_path_half_width_m", 0.18
        )
        self.declare_parameter(
            "lidar_obstacle_minimum_cluster_points", 3
        )
        self.declare_parameter(
            "lidar_obstacle_maximum_scan_index_gap", 2
        )
        self.declare_parameter(
            "lidar_obstacle_maximum_cluster_gap_m", 0.16
        )
        self.declare_parameter(
            "lidar_obstacle_minimum_cluster_width_m", 0.09
        )
        self.declare_parameter(
            "lidar_obstacle_maximum_cluster_width_m", 0.70
        )
        self.declare_parameter(
            "lidar_obstacle_side_probe_inner_m", 0.18
        )
        self.declare_parameter(
            "lidar_obstacle_side_probe_outer_m", 0.55
        )
        self.declare_parameter("lidar_obstacle_lidar_x_m", 0.065)
        self.declare_parameter("lidar_obstacle_lidar_y_m", 0.0)
        self.declare_parameter("lidar_obstacle_lidar_yaw_deg", 0.0)
        self.declare_parameter("lidar_obstacle_required_frames", 2)
        self.declare_parameter("lidar_obstacle_left_offset_m", 0.28)
        self.declare_parameter("lidar_obstacle_right_offset_m", 0.28)
        self.declare_parameter(
            "lidar_obstacle_offset_rate_mps", 0.45
        )
        self.declare_parameter(
            "lidar_obstacle_speed_limit_command", 4.0
        )
        self.declare_parameter(
            "lidar_obstacle_estimated_length_m", 0.35
        )
        self.declare_parameter(
            "lidar_obstacle_post_margin_m", 0.45
        )
        self.declare_parameter(
            "lidar_obstacle_clear_hold_sec", 0.25
        )
        self.declare_parameter(
            "lidar_obstacle_return_deadband_m", 0.02
        )
        self.declare_parameter(
            "lidar_obstacle_center_deadband_m", 0.04
        )
        self.declare_parameter("cone_mode_topic", "/hybrid/mode")
        self.declare_parameter(
            "cone_command_topic", "/my_rule/cone_cmd"
        )
        self.declare_parameter("cone_command_timeout_sec", 0.35)
        self.declare_parameter("cone_speed_cap_command", 9.5)
        self.declare_parameter("emergency_stop_distance_m", 0.38)
        self.declare_parameter("emergency_front_half_angle_deg", 15.0)
        self.declare_parameter("scan_timeout_sec", 0.5)
        self.declare_parameter(
            "steering_map_commands",
            [
                -42.0, -40.0, -35.0, -30.0, -20.0, -10.0, 0.0,
                10.0, 20.0, 30.0, 35.0, 40.0, 42.0,
            ],
        )
        self.declare_parameter(
            "steering_map_curvatures",
            [
                1.502435, 1.383494, 1.174860, 0.922781, 0.552809,
                0.194230, 0.0, -0.556883, -0.959829, -1.369323,
                -1.601706, -1.853397, -1.939236,
            ],
        )

    def _load_route(self, route_path: Path | None = None) -> None:
        selected_path = route_path or self.waypoints_yaml
        if not selected_path.is_file():
            raise ValueError(
                f"waypoint YAML does not exist: {selected_path}"
            )
        data = yaml.safe_load(
            selected_path.read_text(encoding="utf-8")
        )
        self.closed_route = bool(data.get("closed", self.closed_route))
        configured_frame = str(data.get("frame_id", self.frame_id))
        if configured_frame != self.frame_id:
            raise ValueError(
                f"waypoint frame {configured_frame} != {self.frame_id}"
            )
        waypoints = []
        for index, item in enumerate(data.get("waypoints", [])):
            controller = str(
                item.get("controller_to_next", "global_path")
            )
            if controller not in VALID_CONTROLLERS:
                raise ValueError(
                    f"waypoint {index} has invalid controller {controller}"
                )
            waypoints.append(
                RouteWaypoint(
                    name=str(item.get("name", f"wp_{index:02d}")),
                    x=float(item["x"]),
                    y=float(item["y"]),
                    controller_to_next=controller,
                )
            )
        self.waypoints = waypoints
        self._replan()

    def _replan(self) -> None:
        if len(self.waypoints) < 2:
            self.route = None
            self.speed_profile_mps = ()
            self.current_path_index = None
            self._publish_route()
            return
        if self.path_csv is not None:
            points = list(
                load_path_csv(
                    self.path_csv,
                    spacing_m=float(
                        self.get_parameter("path_spacing_m").value
                    ),
                    closed=self.closed_route,
                )
            )
            if bool(
                self.get_parameter("csv_path_smoothing_enabled").value
            ):
                points = smooth_path_points(
                    self.map_grid,
                    points,
                    closed=self.closed_route,
                    data_weight=float(
                        self.get_parameter(
                            "csv_path_smoothing_data_weight"
                        ).value
                    ),
                    smooth_weight=float(
                        self.get_parameter(
                            "csv_path_smoothing_weight"
                        ).value
                    ),
                    iterations=int(
                        self.get_parameter(
                            "csv_path_smoothing_iterations"
                        ).value
                    ),
                )
            self.route = PlannedRoute(
                points=tuple(points),
                segment_indices=tuple(0 for _ in points),
                snapped_waypoints=tuple(
                    (waypoint.x, waypoint.y)
                    for waypoint in self.waypoints
                ),
            )
            self.current_path_index = None
            self.route_complete = False
            self._rebuild_speed_profile()
            self._publish_route()
            return
        self.route = plan_waypoint_route(
            self.map_grid,
            [(waypoint.x, waypoint.y) for waypoint in self.waypoints],
            closed=self.closed_route,
            path_spacing_m=float(
                self.get_parameter("path_spacing_m").value
            ),
            waypoint_snap_radius_m=float(
                self.get_parameter("waypoint_snap_radius_m").value
            ),
            path_smoothing_enabled=bool(
                self.get_parameter("path_smoothing_enabled").value
            ),
            path_smoothing_data_weight=float(
                self.get_parameter("path_smoothing_data_weight").value
            ),
            path_smoothing_weight=float(
                self.get_parameter("path_smoothing_weight").value
            ),
            path_smoothing_iterations=int(
                self.get_parameter("path_smoothing_iterations").value
            ),
            path_smoothing_anchor_weight=float(
                self.get_parameter("path_smoothing_anchor_weight").value
            ),
            path_smoothing_maximum_deviation_m=float(
                self.get_parameter(
                    "path_smoothing_maximum_deviation_m"
                ).value
            ),
            clearance_cost_weight=float(
                self.get_parameter("clearance_cost_weight").value
            ),
            clearance_cost_decay_m=float(
                self.get_parameter("clearance_cost_decay_m").value
            ),
        )
        self.current_path_index = None
        self.route_complete = False
        self._rebuild_speed_profile()
        self._publish_route()

    def _rebuild_speed_profile(self) -> None:
        if self.route is None or not self.route.points:
            self.speed_profile_mps = ()
            return
        speed_gain = max(
            1.0e-3,
            float(self.get_parameter("speed_gain_mps_per_command").value),
        )
        curvatures = path_curvature_profile(
            self.route.points,
            closed=self.closed_route,
            curvature_window_m=float(
                self.get_parameter(
                    "speed_profile_curvature_window_m"
                ).value
            ),
            smoothing_points=int(
                self.get_parameter(
                    "speed_profile_curvature_smoothing_points"
                ).value
            ),
        )
        self.speed_profile_mps = forward_backward_velocity_profile(
            self.route.points,
            curvatures,
            closed=self.closed_route,
            maximum_speed_mps=max(
                0.0,
                float(
                    self.get_parameter("cruise_speed_command").value
                ),
            )
            * speed_gain,
            maximum_lateral_accel_mps2=float(
                self.get_parameter("maximum_lateral_accel_mps2").value
            ),
            maximum_accel_mps2=float(
                self.get_parameter("speed_profile_max_accel_mps2").value
            ),
            maximum_decel_mps2=float(
                self.get_parameter("speed_profile_max_decel_mps2").value
            ),
        )
        commands = [value / speed_gain for value in self.speed_profile_mps]
        self.get_logger().info(
            "forward/backward speed profile: "
            f"min={min(commands):.2f}, "
            f"mean={sum(commands) / len(commands):.2f}, "
            f"max={max(commands):.2f}"
        )

    def _publish_route(self) -> None:
        stamp = self.get_clock().now().to_msg()
        path = PathMessage()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp
        if self.route:
            for index, point in enumerate(self.route.points):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                following = (
                    (index + 1) % len(self.route.points)
                    if self.closed_route
                    else min(index + 1, len(self.route.points) - 1)
                )
                yaw = math.atan2(
                    self.route.points[following][1] - point[1],
                    self.route.points[following][0] - point[0],
                )
                pose.pose.orientation.z = math.sin(yaw * 0.5)
                pose.pose.orientation.w = math.cos(yaw * 0.5)
                path.poses.append(pose)
        self.path_pub.publish(path)

        markers = MarkerArray()
        for index, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header = path.header
            marker.ns = "map_waypoints"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.x
            marker.pose.position.y = waypoint.y
            marker.pose.position.z = 0.08
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.16
            if waypoint.controller_to_next == "cone_rule":
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.45, 0.0
            elif waypoint.controller_to_next == "dynamic_vehicle_rule":
                marker.color.r = 0.85
                marker.color.g = 0.15
                marker.color.b = 0.85
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.1, 0.8, 0.2
            marker.color.a = 1.0
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def _on_scan(self, message: LaserScan) -> None:
        self.latest_scan = message
        half_angle = math.radians(
            float(
                self.get_parameter("emergency_front_half_angle_deg").value
            )
        )
        minimum = float("inf")
        for index, value in enumerate(message.ranges):
            if not math.isfinite(value):
                continue
            angle = message.angle_min + index * message.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_angle and value >= message.range_min:
                minimum = min(minimum, float(value))
        self.emergency_front_range_m = minimum
        self.scan_time = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        self.latest_speed_mps = math.hypot(
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )
        self.latest_yaw_rate_radps = float(
            message.twist.twist.angular.z
        )
        self.odom_time = time.monotonic()

    def _on_dynamic_counts(self, message: Int32MultiArray) -> None:
        if len(message.data) < 4:
            return
        self.dynamic_counts = tuple(int(value) for value in message.data[:4])
        self.dynamic_counts_time = time.monotonic()

    def _on_cone_mode(self, message: String) -> None:
        self.cone_mode_active = message.data.startswith("CONE_RULE")

    def _on_cone_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            return
        angle = float(message.data[0])
        if bool(
            self.get_parameter("cone_input_is_physical_angle").value
        ):
            angle = self._cone_physical_to_motor_command(angle)
        self.cone_command = (angle, float(message.data[1]))
        self.cone_command_confidence = (
            float(message.data[2]) if len(message.data) >= 3 else 1.0
        )
        self.cone_command_time = time.monotonic()

    def _on_cone_clusters(self, message: PoseArray) -> None:
        distances = [
            math.hypot(
                float(pose.position.x),
                float(pose.position.y),
            )
            for pose in message.poses
            if (
                float(pose.position.x) > 0.0
                and math.isfinite(float(pose.position.x))
                and math.isfinite(float(pose.position.y))
            )
        ]
        self.mission_supervisor.observe_cone_lidar(
            now_sec=time.monotonic(),
            count=len(distances),
            nearest_distance_m=min(distances, default=float("inf")),
        )

    def _on_object_detections(
        self, message: ObjectDetectionArray
    ) -> None:
        now = time.monotonic()
        width = max(0, int(message.image_width))
        height = max(0, int(message.image_height))
        cone_names = {
            self._normalize_class_name(value)
            for value in self.get_parameter("cone_class_names").value
        }
        vehicle_names = {
            self._normalize_class_name(value)
            for value in self.get_parameter("vehicle_class_names").value
        }
        traffic_threshold = float(
            self.get_parameter("traffic_min_confidence").value
        )
        traffic_maximum_y = (
            float(
                self.get_parameter(
                    "traffic_signal_max_center_y_ratio"
                ).value
            )
            * height
        )
        cones = []
        vehicles = []
        signal_colors = set()
        for detection in message.detections:
            class_name = self._normalize_class_name(
                detection.class_name
            )
            confidence = float(detection.confidence)
            if class_name in cone_names:
                cones.append(detection)
            if class_name in vehicle_names:
                vehicles.append(detection)
            center_y = 0.5 * (
                float(detection.ymin) + float(detection.ymax)
            )
            if (
                class_name in {"red", "yellow", "green"}
                and confidence >= traffic_threshold
                and (height <= 0 or center_y <= traffic_maximum_y)
            ):
                signal_colors.add(class_name)

        vehicle_distances = [
            distance
            for distance in (
                self._scan_distance_for_detection(detection, width)
                for detection in vehicles
            )
            if math.isfinite(distance)
        ]
        self.semantic_vehicle_count = len(vehicles)
        if "red" in signal_colors:
            traffic_color = "red"
        elif "yellow" in signal_colors:
            traffic_color = "yellow"
        elif "green" in signal_colors:
            traffic_color = "green"
        else:
            traffic_color = "unknown"
        self.mission_supervisor.observe_objects(
            now_sec=now,
            cone_count=len(cones),
            cone_max_confidence=max(
                (float(item.confidence) for item in cones),
                default=0.0,
            ),
            vehicle_count=len(vehicles),
            vehicle_max_confidence=max(
                (float(item.confidence) for item in vehicles),
                default=0.0,
            ),
            vehicle_lidar_distance_m=min(
                vehicle_distances,
                default=float("inf"),
            ),
            traffic_color=traffic_color,
        )
        self._update_cone_processing_gate()

    def _update_cone_processing_gate(self) -> None:
        if self.mission_trigger_mode == "semantic":
            requested = self.mission_supervisor.cone_processing_requested(
                time.monotonic()
            )
        else:
            requested = self.cone_mode_active
            if self.current_path_index is not None:
                requested = requested or (
                    self._active_controller(self.current_path_index)
                    == "cone_rule"
                )
        self._publish_cone_processing_enabled(requested)

    def _publish_cone_processing_enabled(
        self, enabled: bool, *, force: bool = False
    ) -> None:
        enabled = bool(enabled)
        if not force and enabled == self.cone_processing_enabled:
            return
        self.cone_processing_enabled = enabled
        self.cone_processing_pub.publish(Bool(data=enabled))
        state = "enabled" if enabled else "sleeping"
        self.get_logger().info(f"cone processing {state}")

    @staticmethod
    def _normalize_class_name(value: str) -> str:
        return (
            str(value)
            .strip()
            .lower()
            .replace("-", "_")
            .replace(" ", "_")
        )

    def _scan_distance_for_detection(
        self, detection, image_width: int
    ) -> float:
        scan = self.latest_scan
        now = time.monotonic()
        if (
            scan is None
            or image_width <= 0
            or now - self.scan_time
            > float(
                self.get_parameter("mission_lidar_timeout_sec").value
            )
        ):
            return float("inf")
        minimum_angle, maximum_angle = camera_box_lidar_sector(
            xmin=float(detection.xmin),
            xmax=float(detection.xmax),
            image_width=image_width,
            horizontal_fov_deg=float(
                self.get_parameter(
                    "vehicle_camera_lidar_hfov_deg"
                ).value
            ),
            padding_deg=float(
                self.get_parameter(
                    "vehicle_camera_lidar_padding_deg"
                ).value
            ),
        )
        minimum_points = max(
            1, int(self.get_parameter("object_lidar_min_points").value)
        )
        return scan_sector_distance(
            ranges=scan.ranges,
            angle_min=float(scan.angle_min),
            angle_increment=float(scan.angle_increment),
            range_min=float(scan.range_min),
            range_max=float(scan.range_max),
            sector_min_angle=minimum_angle,
            sector_max_angle=maximum_angle,
            minimum_points=minimum_points,
        )

    def _detect_lidar_path_obstacle(
        self,
        *,
        vehicle_x: float,
        vehicle_y: float,
        vehicle_yaw: float,
    ) -> LidarPathObstacle | None:
        scan = self.latest_scan
        if (
            scan is None
            or self.route is None
            or self.current_path_index is None
        ):
            return None
        return detect_path_obstacle(
            ranges=scan.ranges,
            angle_min=float(scan.angle_min),
            angle_increment=float(scan.angle_increment),
            range_min=float(scan.range_min),
            range_max=float(scan.range_max),
            route_points=self.route.points,
            nearest_index=self.current_path_index,
            vehicle_x=vehicle_x,
            vehicle_y=vehicle_y,
            vehicle_yaw=vehicle_yaw,
            closed=self.closed_route,
            config=self.lidar_obstacle_config,
        )

    def _publish_lidar_obstacle_debug(
        self,
        obstacle: LidarPathObstacle | None,
        state: LidarBypassState,
    ) -> None:
        message = Float32MultiArray()
        if obstacle is None:
            values = [0.0] * 8
        else:
            values = [
                1.0,
                obstacle.distance_m,
                obstacle.lateral_m,
                obstacle.width_m,
                float(obstacle.point_count),
                obstacle.x_vehicle_m,
                obstacle.y_vehicle_m,
                obstacle.left_clearance_m
                - obstacle.right_clearance_m,
            ]
        mode_codes = {
            "NORMAL": 0.0,
            "BYPASS_LEFT": 1.0,
            "BYPASS_RIGHT": -1.0,
            "RETURN_CENTER": 2.0,
        }
        message.data = [
            *values,
            mode_codes.get(state.mode, 99.0),
            state.lateral_offset_m,
            state.remaining_clear_distance_m,
        ]
        self.lidar_obstacle_debug_pub.publish(message)

    def _cone_physical_to_motor_command(
        self, physical_angle_deg: float
    ) -> float:
        actual = [
            float(value)
            for value in self.get_parameter(
                "cone_steering_actual_deg"
            ).value
        ]
        commands = [
            float(value)
            for value in self.get_parameter(
                "cone_steering_commands"
            ).value
        ]
        if len(actual) < 2 or len(actual) != len(commands):
            raise ValueError(
                "cone steering maps must have equal length >= 2"
            )
        magnitude = abs(float(physical_angle_deg))
        if magnitude <= actual[0]:
            mapped = commands[0]
        elif magnitude >= actual[-1]:
            mapped = commands[-1]
        else:
            mapped = commands[-1]
            for index in range(1, len(actual)):
                if magnitude <= actual[index]:
                    ratio = (
                        (magnitude - actual[index - 1])
                        / (actual[index] - actual[index - 1])
                    )
                    mapped = (
                        commands[index - 1]
                        + ratio
                        * (commands[index] - commands[index - 1])
                    )
                    break
        return math.copysign(mapped, float(physical_angle_deg))

    def _on_route_localization_ready(self, message: Bool) -> None:
        was_ready = self.route_localization_ready
        self.route_localization_ready = bool(message.data)
        if self.route_localization_ready and not was_ready:
            self.drive_start_time = time.monotonic() + max(
                0.0,
                float(self.get_parameter("drive_start_delay_sec").value),
            )
            self.get_logger().info(
                "route localization ready; drive start delay begins"
            )

    def _on_clicked_point(self, message: PointStamped) -> None:
        if message.header.frame_id != self.frame_id:
            self.get_logger().warning(
                f"ignored clicked point in {message.header.frame_id}; "
                f"expected {self.frame_id}"
            )
            return
        if self.drive_enabled:
            self.get_logger().warning(
                "clicked point ignored while drive_enabled=true"
            )
            return
        self.waypoints.append(
            RouteWaypoint(
                name=f"clicked_{len(self.waypoints):02d}",
                x=float(message.point.x),
                y=float(message.point.y),
            )
        )
        try:
            self._replan()
            self._save_captured_route()
        except ValueError as exc:
            self.waypoints.pop()
            self.get_logger().error(f"clicked waypoint rejected: {exc}")

    def _save_captured_route(self) -> None:
        if self.capture_output_yaml is None:
            return
        data = {
            "frame_id": self.frame_id,
            "closed": self.closed_route,
            "waypoints": [
                {
                    "name": waypoint.name,
                    "x": round(waypoint.x, 4),
                    "y": round(waypoint.y, 4),
                    "controller_to_next": waypoint.controller_to_next,
                }
                for waypoint in self.waypoints
            ],
        }
        self.capture_output_yaml.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.capture_output_yaml.with_suffix(".yaml.tmp")
        temporary.write_text(
            yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        temporary.replace(self.capture_output_yaml)

    def _undo_waypoint(self, _, response):
        if not self.waypoints:
            response.success = False
            response.message = "no waypoint to undo"
            return response
        self.waypoints.pop()
        self._replan()
        self._save_captured_route()
        response.success = True
        response.message = "last waypoint removed"
        return response

    def _clear_waypoints(self, _, response):
        if self.drive_enabled:
            response.success = False
            response.message = "clear rejected while drive_enabled=true"
            return response
        self.waypoints.clear()
        self._replan()
        self._save_captured_route()
        response.success = True
        response.message = "waypoints cleared"
        return response

    def _reload_route(self, _, response):
        if self.drive_enabled:
            response.success = False
            response.message = "reload rejected while drive_enabled=true"
            return response
        try:
            selected_path = (
                self.capture_output_yaml
                if (
                    self.capture_output_yaml is not None
                    and self.capture_output_yaml.is_file()
                )
                else self.waypoints_yaml
            )
            self._load_route(selected_path)
        except (OSError, ValueError, KeyError) as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = "route reloaded"
        return response

    def _reset_localization_guard(self, _, response):
        if self.latest_speed_mps > 0.05:
            response.success = False
            response.message = (
                "stop the vehicle before resetting localization guard"
            )
            return response
        self.localization_guard.reset()
        self.latest_guard_result = None
        self.last_guard_state = ""
        self.drive_start_time = time.monotonic() + max(
            0.0,
            float(self.get_parameter("drive_start_delay_sec").value),
        )
        response.success = True
        response.message = (
            "localization guard reset; current transform will be accepted"
        )
        return response

    def _publish_localization_guard(
        self,
        result: GuardResult,
        raw: PlanarTransform,
    ) -> None:
        status = (
            f"{result.state} "
            f"translation={result.translation_residual_m:.3f}m "
            f"yaw={math.degrees(result.yaw_residual_rad):.2f}deg "
            f"age={result.outlier_age_sec:.3f}s"
        )
        self.localization_guard_status_pub.publish(String(data=status))
        state_codes = {
            "DISABLED": 0.0,
            "TRACKING": 1.0,
            "HOLDING": 2.0,
            "FAULT": 3.0,
        }
        debug = Float32MultiArray()
        debug.data = [
            state_codes[result.state],
            float(result.translation_residual_m),
            float(result.yaw_residual_rad),
            float(result.outlier_age_sec),
            float(raw.x),
            float(raw.y),
            float(raw.yaw),
            float(result.transform.x),
            float(result.transform.y),
            float(result.transform.yaw),
        ]
        self.localization_guard_debug_pub.publish(debug)
        if result.state != self.last_guard_state:
            if result.state == "FAULT":
                self.get_logger().error(
                    "localization jump persisted; motor output stopped: "
                    + status
                )
            elif result.state == "HOLDING":
                self.get_logger().warning(
                    "localization jump rejected temporarily: " + status
                )
            else:
                self.get_logger().info(
                    "localization guard -> " + status
                )
            self.last_guard_state = result.state

    def _vehicle_pose(self) -> tuple[float, float, float]:
        timeout = Duration(
            seconds=float(
                self.get_parameter("pose_timeout_sec").value
            )
        )
        if not self.localization_guard_enabled:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.base_frame_id,
                Time(),
                timeout=timeout,
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.latest_guard_result = None
            return (
                float(translation.x),
                float(translation.y),
                quaternion_yaw(
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                ),
            )
        map_to_odom = self.tf_buffer.lookup_transform(
            self.frame_id,
            self.localization_odom_frame_id,
            Time(),
            timeout=timeout,
        )
        odom_to_base = self.tf_buffer.lookup_transform(
            self.localization_odom_frame_id,
            self.base_frame_id,
            Time(),
            timeout=timeout,
        )
        map_translation = map_to_odom.transform.translation
        map_rotation = map_to_odom.transform.rotation
        raw = PlanarTransform(
            x=float(map_translation.x),
            y=float(map_translation.y),
            yaw=quaternion_yaw(
                map_rotation.x,
                map_rotation.y,
                map_rotation.z,
                map_rotation.w,
            ),
        )
        guard_enabled = (
            self.localization_guard_enabled
            and self.route_localization_ready
            and time.monotonic() >= self.drive_start_time
        )
        odom_translation = odom_to_base.transform.translation
        odom_rotation = odom_to_base.transform.rotation
        local_pose = PlanarTransform(
            x=float(odom_translation.x),
            y=float(odom_translation.y),
            yaw=quaternion_yaw(
                odom_rotation.x,
                odom_rotation.y,
                odom_rotation.z,
                odom_rotation.w,
            ),
        )
        result = self.localization_guard.update(
            raw,
            now_sec=time.monotonic(),
            enabled=guard_enabled,
            child_to_base=local_pose,
        )
        self.latest_guard_result = result
        self._publish_localization_guard(result, raw)

        pose = compose_planar(
            result.transform,
            local_pose,
        )
        return (
            pose.x,
            pose.y,
            pose.yaw,
        )

    def _active_controller(self, path_index: int) -> str:
        if self.route is None:
            return "global_path"
        segment = self.route.segment_indices[path_index]
        if segment >= len(self.waypoints):
            return "global_path"
        return self.waypoints[segment].controller_to_next

    def _control_step(self) -> None:
        if self.route is None:
            self._publish_command(0.0, 0.0, "NO_ROUTE")
            return
        if (
            self.require_route_localization
            and not self.route_localization_ready
        ):
            self._publish_command(
                0.0,
                0.0,
                "WAIT_ROUTE_LOCALIZATION",
            )
            return
        try:
            vehicle_x, vehicle_y, vehicle_yaw = self._vehicle_pose()
        except TransformException as exc:
            self._publish_command(0.0, 0.0, "WAIT_LOCALIZATION")
            self.get_logger().debug(f"TF unavailable: {exc}")
            return
        if (
            self.latest_guard_result is not None
            and self.latest_guard_result.faulted
        ):
            self._publish_command(
                0.0,
                0.0,
                "LOCALIZATION_JUMP_STOP",
            )
            return
        if (
            self.drive_enabled
            and time.monotonic() < self.drive_start_time
        ):
            self._publish_command(0.0, 0.0, "WAIT_DRIVE_START")
            return

        self.current_path_index = nearest_path_index(
            self.route.points,
            vehicle_x,
            vehicle_y,
            previous_index=self.current_path_index,
            closed=self.closed_route,
        )
        if not self.closed_route:
            goal = self.route.points[-1]
            at_end = self.current_path_index >= len(self.route.points) - 2
            near_goal = math.hypot(
                goal[0] - vehicle_x, goal[1] - vehicle_y
            ) <= float(self.get_parameter("goal_tolerance_m").value)
            if at_end and near_goal:
                self.route_complete = True
                self._publish_command(0.0, 0.0, "ROUTE_COMPLETE")
                return

        now = time.monotonic()
        scan_fresh = now - self.scan_time <= float(
            self.get_parameter("scan_timeout_sec").value
        )
        if (
            self.drive_enabled
            and bool(
                self.get_parameter("scan_required_for_drive").value
            )
            and not scan_fresh
        ):
            self._publish_command(0.0, 0.0, "WAIT_SCAN")
            return
        emergency = (
            scan_fresh
            and self.emergency_front_range_m
            <= float(
                self.get_parameter("emergency_stop_distance_m").value
            )
        )
        if emergency:
            self._publish_command(0.0, 0.0, "EMERGENCY_STOP")
            return

        lidar_obstacle_enabled = (
            self.mission_trigger_mode == "semantic"
            and bool(
                self.get_parameter(
                    "lidar_obstacle_fallback_enabled"
                ).value
            )
            and scan_fresh
            and (
                self.lidar_obstacle_rule.mode != "NORMAL"
                or not self.mission_supervisor.cone_processing_requested(
                    now
                )
            )
        )
        self.latest_lidar_obstacle = (
            self._detect_lidar_path_obstacle(
                vehicle_x=vehicle_x,
                vehicle_y=vehicle_y,
                vehicle_yaw=vehicle_yaw,
            )
            if lidar_obstacle_enabled
            else None
        )
        lidar_obstacle = self.lidar_obstacle_rule.update(
            now_sec=now,
            observation=self.latest_lidar_obstacle,
            path_index=self.current_path_index,
            route_points=self.route.points,
            closed=self.closed_route,
            enabled=lidar_obstacle_enabled,
        )
        self._publish_lidar_obstacle_debug(
            self.latest_lidar_obstacle,
            lidar_obstacle,
        )

        controller = self._active_controller(self.current_path_index)
        fresh_cone = (
            now - self.cone_command_time
            <= float(
                self.get_parameter("cone_command_timeout_sec").value
            )
            and self.cone_command_confidence
            >= float(
                self.get_parameter(
                    "cone_command_min_confidence"
                ).value
            )
        )
        if self.mission_trigger_mode == "semantic":
            decision = self.mission_supervisor.decide(
                now_sec=now,
                cone_command_ready=fresh_cone,
                dynamic_rule_mode=self.dynamic_rule.mode,
                lidar_obstacle_rule_mode=lidar_obstacle.mode,
            )
            self.mission_reason_pub.publish(
                String(data=decision.reason)
            )
            if decision.mode == MissionMode.TRAFFIC_STOP:
                self.dynamic_rule.reset()
                self._publish_command(
                    0.0,
                    0.0,
                    "TRAFFIC_STOP",
                )
                return
            if decision.mode == MissionMode.CONE_RULE:
                self.lidar_obstacle_rule.reset()
                if not fresh_cone:
                    self._publish_command(
                        0.0, 0.0, "WAIT_CONE_COMMAND"
                    )
                    return
                speed_cap = float(
                    self.get_parameter("cone_speed_cap_command").value
                )
                self._publish_command(
                    self.cone_command[0],
                    min(self.cone_command[1], speed_cap),
                    "CONE_RULE",
                )
                return
            if decision.mode == MissionMode.LIDAR_OBSTACLE_RULE:
                self.dynamic_rule.reset()
                dynamic_armed = False
                lidar_obstacle_armed = True
            else:
                lidar_obstacle_armed = False
            if decision.mode == MissionMode.LANE_INTERVENTION:
                self.dynamic_rule.reset()
                self._publish_command(
                    0.0,
                    0.0,
                    "WAIT_LANE_INTERVENTION",
                )
                return
            dynamic_armed = (
                decision.mode == MissionMode.DYNAMIC_VEHICLE_RULE
            )
            if dynamic_armed:
                self.lidar_obstacle_rule.reset()
        else:
            self.mission_reason_pub.publish(
                String(data=f"route_segment_{controller}")
            )
            if controller == "cone_rule":
                if self.cone_mode_active and fresh_cone:
                    speed_cap = float(
                        self.get_parameter(
                            "cone_speed_cap_command"
                        ).value
                    )
                    self._publish_command(
                        self.cone_command[0],
                        min(self.cone_command[1], speed_cap),
                        "CONE_RULE",
                    )
                else:
                    self._publish_command(
                        0.0, 0.0, "WAIT_CONE_RULE"
                    )
                return
            dynamic_armed = controller == "dynamic_vehicle_rule"
            lidar_obstacle_armed = False

        counts_fresh = (
            now - self.dynamic_counts_time
            <= float(
                self.get_parameter("dynamic_detector_timeout_sec").value
            )
        )
        if (
            self.mission_trigger_mode == "route_segments"
            and dynamic_armed
            and bool(
                self.get_parameter("dynamic_detector_required").value
            )
            and not counts_fresh
        ):
            self._publish_command(0.0, 0.0, "WAIT_DYNAMIC_DETECTOR")
            return
        counts = (
            self.dynamic_counts
            if counts_fresh
            else (self.semantic_vehicle_count, 0, 0, 0)
        )
        dynamic = self.dynamic_rule.update(
            now_sec=now,
            front_count=counts[0],
            behind_count=counts[3],
            armed=dynamic_armed,
        )
        active_lateral_offset = (
            lidar_obstacle.lateral_offset_m
            if lidar_obstacle_armed
            else dynamic.lateral_offset_m
        )
        active_speed_limit = (
            lidar_obstacle.speed_limit_command
            if lidar_obstacle_armed
            else dynamic.speed_limit_command
        )
        odom_fresh = now - self.odom_time <= float(
            self.get_parameter("odom_timeout_sec").value
        )
        speed_gain = max(
            0.001,
            float(self.get_parameter("speed_gain_mps_per_command").value),
        )
        measured_speed = (
            self.latest_speed_mps
            if odom_fresh
            else float(self.get_parameter("minimum_speed_command").value)
            * speed_gain
        )
        command_kwargs = {
            "points": self.route.points,
            "nearest_index": self.current_path_index,
            "vehicle_x": vehicle_x,
            "vehicle_y": vehicle_y,
            "vehicle_yaw": vehicle_yaw,
            "speed_mps": measured_speed,
            "lateral_offset_m": active_lateral_offset,
            "closed": self.closed_route,
            "wheelbase_m": float(self.get_parameter("wheelbase_m").value),
            "front_axle_offset_m": float(
                self.get_parameter("front_axle_offset_m").value
            ),
            "steering_delay_sec": float(
                self.get_parameter("steering_delay_sec").value
            ),
            "curvature_feedforward_gain": float(
                self.get_parameter("curvature_feedforward_gain").value
            ),
            "heading_window_m": float(
                self.get_parameter("path_heading_window_m").value
            ),
            "heading_preview_m": float(
                self.get_parameter("path_heading_preview_m").value
            ),
            "curvature_window_m": float(
                self.get_parameter("path_curvature_window_m").value
            ),
            "curvature_preview_m": float(
                self.get_parameter("path_curvature_preview_m").value
            ),
            "maximum_steering_angle_rad": float(
                self.get_parameter("maximum_steering_angle_rad").value
            ),
        }
        command = stanley_path_command(
            stanley_gain=float(self.get_parameter("stanley_gain").value),
            stanley_softening_mps=float(
                self.get_parameter("stanley_softening_mps").value
            ),
            heading_gain=float(
                self.get_parameter("stanley_heading_gain").value
            ),
            **command_kwargs,
        )
        straight = (
            abs(command.path_curvature_per_m)
            <= float(
                self.get_parameter(
                    "straight_curvature_threshold_per_m"
                ).value
            )
        )
        if straight:
            command = stanley_path_command(
                stanley_gain=float(
                    self.get_parameter("straight_stanley_gain").value
                ),
                stanley_softening_mps=float(
                    self.get_parameter(
                        "straight_stanley_softening_mps"
                    ).value
                ),
                heading_gain=float(
                    self.get_parameter(
                        "straight_stanley_heading_gain"
                    ).value
                ),
                **command_kwargs,
            )
        tracking_steering_angle = command.steering_angle_rad
        if (
            not straight
            and str(self.get_parameter("curve_controller").value)
            == "pure_pursuit"
        ):
            curve_lookahead = max(
                0.10,
                float(
                    self.get_parameter(
                        "curve_pure_pursuit_lookahead_m"
                    ).value
                )
                + measured_speed
                * max(
                    0.0,
                    float(
                        self.get_parameter(
                            "curve_pure_pursuit_speed_preview_sec"
                        ).value
                    ),
                ),
            )
            pursuit = pure_pursuit_command(
                self.route.points,
                self.current_path_index,
                vehicle_x=vehicle_x,
                vehicle_y=vehicle_y,
                vehicle_yaw=vehicle_yaw,
                lookahead_m=curve_lookahead,
                lateral_offset_m=active_lateral_offset,
                closed=self.closed_route,
            )
            tracking_steering_angle = math.atan(
                float(self.get_parameter("wheelbase_m").value)
                * pursuit.curvature_per_m
            )
        damping_parameter = (
            "straight_yaw_rate_damping_sec"
            if straight
            else "curve_yaw_rate_damping_sec"
        )
        desired_yaw_rate = (
            measured_speed * command.path_curvature_per_m
        )
        damped_steering_angle = tracking_steering_angle - float(
            self.get_parameter(damping_parameter).value
        ) * (self.latest_yaw_rate_radps - desired_yaw_rate)
        maximum_steering_angle = abs(
            float(
                self.get_parameter("maximum_steering_angle_rad").value
            )
        )
        damped_steering_angle = max(
            -maximum_steering_angle,
            min(maximum_steering_angle, damped_steering_angle),
        )
        damped_curvature = math.tan(damped_steering_angle) / max(
            0.05,
            float(self.get_parameter("wheelbase_m").value),
        )
        target_angle = steering_command_for_curvature(
            damped_curvature,
            self.command_inputs,
            self.curvature_inputs,
        )
        control_now = time.monotonic()
        dt = control_now - self.last_control_time
        self.last_control_time = control_now
        rate_parameter = (
            "straight_steering_rate_command_per_sec"
            if straight
            else "curve_steering_rate_command_per_sec"
        )
        filter_parameter = (
            "straight_steering_filter_sec"
            if straight
            else "curve_steering_filter_sec"
        )
        angle = filtered_steering_command(
            target_angle,
            self.last_steering_command,
            dt_sec=dt,
            rate_limit_command_per_sec=float(
                self.get_parameter(rate_parameter).value
            ),
            time_constant_sec=float(
                self.get_parameter(filter_parameter).value
            ),
        )
        self.last_steering_command = angle
        cruise = float(self.get_parameter("cruise_speed_command").value)
        minimum = float(self.get_parameter("minimum_speed_command").value)
        slowdown = max(
            0.01,
            float(
                self.get_parameter(
                    "curve_slowdown_curvature_per_m"
                ).value
            ),
        )
        speed_planner_mode = str(
            self.get_parameter("speed_planner_mode").value
        ).strip().lower()
        if (
            speed_planner_mode == "forward_backward"
            and self.speed_profile_mps
        ):
            speed = minimum_profile_value_ahead(
                self.route.points,
                self.speed_profile_mps,
                self.current_path_index,
                measured_speed
                * max(
                    0.0,
                    float(
                        self.get_parameter(
                            "speed_profile_braking_preview_sec"
                        ).value
                    ),
                ),
                closed=self.closed_route,
            ) / speed_gain
        else:
            speed_path_curvature = command.path_curvature_per_m
            speed_curvature_preview = float(
                self.get_parameter("speed_curvature_preview_m").value
            )
            if speed_curvature_preview >= 0.0:
                speed_command_kwargs = dict(command_kwargs)
                speed_command_kwargs["curvature_preview_m"] = (
                    speed_curvature_preview
                )
                speed_preview_command = stanley_path_command(
                    stanley_gain=0.0,
                    stanley_softening_mps=1.0,
                    heading_gain=0.0,
                    **speed_command_kwargs,
                )
                speed_path_curvature = (
                    speed_preview_command.path_curvature_per_m
                )
            path_curvature = abs(speed_path_curvature)
            curve_ratio = min(1.0, path_curvature / slowdown)
            speed = cruise + (minimum - cruise) * curve_ratio
            maximum_lateral_accel = max(
                0.05,
                float(
                    self.get_parameter(
                        "maximum_lateral_accel_mps2"
                    ).value
                ),
            )
            if path_curvature > 1.0e-4:
                lateral_speed_cap_mps = math.sqrt(
                    maximum_lateral_accel / path_curvature
                )
                speed = min(speed, lateral_speed_cap_mps / speed_gain)
        speed = alignment_limited_speed_command(
            speed,
            minimum,
            cross_track_error_m=command.cross_track_error_m,
            heading_error_rad=command.heading_error_rad,
            cross_track_soft_m=float(
                self.get_parameter(
                    "speed_alignment_cross_track_soft_m"
                ).value
            ),
            cross_track_hard_m=float(
                self.get_parameter(
                    "speed_alignment_cross_track_hard_m"
                ).value
            ),
            heading_soft_rad=float(
                self.get_parameter(
                    "speed_alignment_heading_soft_rad"
                ).value
            ),
            heading_hard_rad=float(
                self.get_parameter(
                    "speed_alignment_heading_hard_rad"
                ).value
            ),
        )
        if active_speed_limit is not None:
            speed = min(speed, active_speed_limit)
        target_speed = speed
        speed = rate_limited_speed_command(
            target_speed,
            self.last_speed_command,
            dt_sec=dt,
            acceleration_rate_command_per_sec=float(
                self.get_parameter(
                    "speed_acceleration_rate_command_per_sec"
                ).value
            ),
            deceleration_rate_command_per_sec=float(
                self.get_parameter(
                    "speed_deceleration_rate_command_per_sec"
                ).value
            ),
        )
        speed = minimum_effective_speed_command(
            speed,
            target_speed,
            minimum,
        )
        fixed_speed = float(
            self.get_parameter("fixed_speed_command").value
        )
        if fixed_speed >= 0.0:
            speed = fixed_speed
        if active_speed_limit is not None:
            speed = min(speed, active_speed_limit)
        self.last_speed_command = speed
        if lidar_obstacle_armed:
            mode = f"LIDAR_OBSTACLE_RULE_{lidar_obstacle.mode}"
        elif dynamic.mode != "NORMAL":
            mode = f"DYNAMIC_VEHICLE_RULE_{dynamic.mode}"
        else:
            mode = "GLOBAL_PATH"
        self._publish_command(
            angle,
            speed,
            mode,
            debug=[
                float(self.current_path_index),
                float(command.target_index),
                float(command.curvature_per_m),
                float(command.path_curvature_per_m),
                float(command.cross_track_error_m),
                float(command.heading_error_rad),
                float(measured_speed),
                1.0 if odom_fresh else 0.0,
                float(active_lateral_offset),
                float(self.emergency_front_range_m),
                float(counts[0]),
                float(counts[3]),
                float(damped_curvature),
                float(self.latest_yaw_rate_radps),
                (
                    float(self.latest_lidar_obstacle.distance_m)
                    if self.latest_lidar_obstacle is not None
                    else float("inf")
                ),
                float(lidar_obstacle.remaining_clear_distance_m),
            ],
        )

    def _publish_command(
        self,
        angle: float,
        speed: float,
        mode: str,
        debug: list[float] | None = None,
    ) -> None:
        if speed <= 0.0:
            self.last_steering_command = 0.0
            self.last_speed_command = 0.0
        command = Float32MultiArray()
        command.data = [float(angle), float(speed)]
        self.shadow_motor_pub.publish(command)
        if self.motor_pub is not None:
            self.motor_pub.publish(command)
        self.mode_pub.publish(String(data=mode))
        diagnostic = Float32MultiArray()
        diagnostic.data = [
            float(angle),
            float(speed),
            *(debug or []),
        ]
        self.debug_pub.publish(diagnostic)
        if mode != self.last_published_mode:
            self.get_logger().info(
                f"control mode -> {mode}: "
                f"angle={float(angle):.2f}, speed={float(speed):.2f}"
            )
            self.last_published_mode = mode

    def stop(self) -> None:
        self.control_timer.cancel()
        self._publish_command(0.0, 0.0, "SHUTDOWN_STOP")


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except ExternalShutdownException:
        pass
    except RuntimeError as exc:
        if "Unable to convert call argument to Python object" not in str(exc):
            raise
    finally:
        if rclpy.ok():
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
