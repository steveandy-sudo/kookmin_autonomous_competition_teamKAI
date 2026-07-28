#!/usr/bin/env python3
"""Follow map waypoints and hand control to cone or vehicle rules."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import signal
import time

from geometry_msgs.msg import PointStamped, PoseStamped
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
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
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
        self.dynamic_counts = (0, 0, 0, 0)
        self.dynamic_counts_time = 0.0
        self.cone_mode_active = False
        self.cone_command = (0.0, 0.0)
        self.cone_command_time = 0.0
        self.emergency_front_range_m = float("inf")
        self.scan_time = 0.0
        self.last_published_mode = ""
        self.latest_speed_mps = 0.0
        self.latest_yaw_rate_radps = 0.0
        self.odom_time = 0.0
        self.last_steering_command = 0.0
        self.last_speed_command = 0.0
        self.last_control_time = time.monotonic()
        self.drive_start_time = self.last_control_time + max(
            0.0,
            float(self.get_parameter("drive_start_delay_sec").value),
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
        self.debug_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("debug_topic").value),
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

        self._load_route()
        rate = max(2.0, float(self.get_parameter("control_rate_hz").value))
        self.create_timer(1.0 / rate, self._control_step)
        mode = "DRIVE" if self.drive_enabled else "SHADOW"
        self.get_logger().info(
            f"{mode}: {len(self.waypoints)} waypoints, "
            f"{len(self.route.points) if self.route else 0} path points; "
            "authority=global/cone-rule/dynamic-vehicle-rule"
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
        self.declare_parameter("debug_topic", "/map_nav/debug")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_required_for_drive", True)
        self.declare_parameter("clicked_point_topic", "/clicked_point")
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
        self.declare_parameter("cone_mode_topic", "/hybrid/mode")
        self.declare_parameter(
            "cone_command_topic", "/xycar_motor_shadow"
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
        self.cone_command = (float(message.data[0]), float(message.data[1]))
        self.cone_command_time = time.monotonic()

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

    def _vehicle_pose(self) -> tuple[float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            self.frame_id,
            self.base_frame_id,
            Time(),
            timeout=Duration(
                seconds=float(
                    self.get_parameter("pose_timeout_sec").value
                )
            ),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            quaternion_yaw(
                rotation.x, rotation.y, rotation.z, rotation.w
            ),
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
        try:
            vehicle_x, vehicle_y, vehicle_yaw = self._vehicle_pose()
        except TransformException as exc:
            self._publish_command(0.0, 0.0, "WAIT_LOCALIZATION")
            self.get_logger().debug(f"TF unavailable: {exc}")
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

        controller = self._active_controller(self.current_path_index)
        if controller == "cone_rule":
            fresh_cone = (
                now - self.cone_command_time
                <= float(
                    self.get_parameter("cone_command_timeout_sec").value
                )
            )
            if self.cone_mode_active and fresh_cone:
                speed_cap = float(
                    self.get_parameter("cone_speed_cap_command").value
                )
                self._publish_command(
                    self.cone_command[0],
                    min(self.cone_command[1], speed_cap),
                    "CONE_RULE",
                )
            else:
                self._publish_command(0.0, 0.0, "WAIT_CONE_RULE")
            return

        dynamic_armed = controller == "dynamic_vehicle_rule"
        counts_fresh = (
            now - self.dynamic_counts_time
            <= float(
                self.get_parameter("dynamic_detector_timeout_sec").value
            )
        )
        if (
            dynamic_armed
            and bool(
                self.get_parameter("dynamic_detector_required").value
            )
            and not counts_fresh
        ):
            self._publish_command(0.0, 0.0, "WAIT_DYNAMIC_DETECTOR")
            return
        counts = self.dynamic_counts if counts_fresh else (0, 0, 0, 0)
        dynamic = self.dynamic_rule.update(
            now_sec=now,
            front_count=counts[0],
            behind_count=counts[3],
            armed=dynamic_armed,
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
            "lateral_offset_m": dynamic.lateral_offset_m,
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
                lateral_offset_m=dynamic.lateral_offset_m,
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
        if dynamic.speed_limit_command is not None:
            speed = min(speed, dynamic.speed_limit_command)
        speed = rate_limited_speed_command(
            speed,
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
        fixed_speed = float(
            self.get_parameter("fixed_speed_command").value
        )
        if fixed_speed >= 0.0:
            speed = fixed_speed
        self.last_speed_command = speed
        mode = (
            f"DYNAMIC_VEHICLE_RULE_{dynamic.mode}"
            if dynamic.mode != "NORMAL"
            else "GLOBAL_PATH"
        )
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
                float(dynamic.lateral_offset_m),
                float(self.emergency_front_range_m),
                float(counts[0]),
                float(counts[3]),
                float(damped_curvature),
                float(self.latest_yaw_rate_radps),
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
    finally:
        if rclpy.ok():
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
