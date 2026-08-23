#!/usr/bin/env python3
"""Publish read-only RViz markers for the lane and LiDAR cone planners."""

import math
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseArray
from kaiev26_msgs.msg import Centerline
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray


Point2 = Tuple[float, float]

CONE_PATH_MARKERS = (
    ("planned_waypoint_line", 5),
    ("planned_waypoints", 6),
    ("planned_path_endpoints", 7),
    ("planned_path_endpoints", 8),
    ("pure_pursuit_target", 9),
)
LANE_PATH_MARKERS = (
    ("lane_planned_waypoint_line", 20),
    ("lane_planned_waypoints", 21),
    ("lane_control_target", 22),
)
MANUAL_PATH_MARKERS = (
    ("manual_driven_path", 40),
    ("manual_vehicle_position", 41),
)


def path_visibility_for_control_mode(mode: str) -> Tuple[bool, bool]:
    """Return (show_cone_path, show_lane_path) for the active controller."""
    normalized = str(mode).strip().upper()
    if normalized == "CONE_RULE":
        return True, False
    if normalized in {"RULE", "YOLO_LIDAR_AVOIDANCE"}:
        return False, True
    return False, False


def normalize_frame_id(frame_id: str) -> str:
    """Return a TF frame without the optional leading slash."""
    return str(frame_id).strip().lstrip("/")


def transform_planar_points(
    points: Sequence[Point2],
    source_frame: str,
    output_frame: str,
    lidar_to_rear_axle_m: float,
) -> Optional[List[Point2]]:
    """Transform points between the planner's rear-axle and LiDAR frames."""
    source = normalize_frame_id(source_frame)
    output = normalize_frame_id(output_frame)
    values = [(float(x), float(y)) for x, y in points]
    if not source or source == output:
        return values
    offset = float(lidar_to_rear_axle_m)
    rear_frames = {"rear_axle", "base_footprint"}
    if source in rear_frames and output == "laser_frame":
        return [(x - offset, y) for x, y in values]
    if source == "laser_frame" and output in rear_frames:
        return [(x + offset, y) for x, y in values]
    if source in rear_frames and output in rear_frames:
        return values
    return None


def select_lookahead_index(
    path: Sequence[Point2],
    minimum_m: float,
    maximum_m: float,
    length_scale: float,
) -> Tuple[Optional[int], float]:
    """Match the cone controller's dynamic lookahead and target selection."""
    if not path:
        return None, float(minimum_m)
    length = sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(path[:-1], path[1:])
    )
    lookahead = max(
        float(minimum_m),
        min(float(maximum_m), float(length_scale) * length),
    )
    for index, (x, y) in enumerate(path):
        if math.hypot(x, y) > lookahead:
            return index, lookahead
    return len(path) - 1, lookahead


def fov_outline_points(
    maximum_range_m: float,
    minimum_angle_deg: float,
    maximum_angle_deg: float,
    samples: int = 48,
) -> List[Point2]:
    """Build a closed line strip showing the accepted scan sector."""
    count = max(2, int(samples))
    start = math.radians(float(minimum_angle_deg))
    finish = math.radians(float(maximum_angle_deg))
    radius = max(0.0, float(maximum_range_m))
    arc = []
    for index in range(count):
        ratio = index / float(count - 1)
        angle = start + ratio * (finish - start)
        arc.append((radius * math.cos(angle), radius * math.sin(angle)))
    return [(0.0, 0.0), *arc, (0.0, 0.0)]


def sector_triangle_points(
    minimum_range_m: float,
    maximum_range_m: float,
    minimum_angle_deg: float,
    maximum_angle_deg: float,
    samples: int = 48,
) -> List[Point2]:
    """Build triangles filling an annular sector in the LiDAR XY plane."""
    count = max(2, int(samples))
    inner_radius = max(0.0, float(minimum_range_m))
    outer_radius = max(inner_radius, float(maximum_range_m))
    start = math.radians(float(minimum_angle_deg))
    finish = math.radians(float(maximum_angle_deg))
    triangles: List[Point2] = []
    for index in range(count - 1):
        first = start + index / float(count - 1) * (finish - start)
        second = start + (index + 1) / float(count - 1) * (finish - start)
        inner_first = (
            inner_radius * math.cos(first),
            inner_radius * math.sin(first),
        )
        inner_second = (
            inner_radius * math.cos(second),
            inner_radius * math.sin(second),
        )
        outer_first = (
            outer_radius * math.cos(first),
            outer_radius * math.sin(first),
        )
        outer_second = (
            outer_radius * math.cos(second),
            outer_radius * math.sin(second),
        )
        triangles.extend(
            [
                inner_first,
                outer_first,
                outer_second,
                inner_first,
                outer_second,
                inner_second,
            ]
        )
    return triangles


def make_point(x: float, y: float, z: float = 0.0) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def yaw_from_quaternion(z: float, w: float) -> float:
    """Return planar yaw for an odometry quaternion with zero roll/pitch."""
    return 2.0 * math.atan2(float(z), float(w))


def odom_trace_in_vehicle_frame(
    points: Sequence[Point2],
    current_position: Point2,
    current_yaw: float,
) -> List[Point2]:
    """Express an odom-frame trace relative to the current vehicle pose."""
    current_x, current_y = current_position
    cosine = math.cos(float(current_yaw))
    sine = math.sin(float(current_yaw))
    transformed: List[Point2] = []
    for world_x, world_y in points:
        delta_x = float(world_x) - float(current_x)
        delta_y = float(world_y) - float(current_y)
        transformed.append(
            (
                cosine * delta_x + sine * delta_y,
                -sine * delta_x + cosine * delta_y,
            )
        )
    return transformed


class ConePathVisualizer(Node):
    """Convert lane/cone planner outputs into one RViz-friendly MarkerArray.

    The historical class and executable names are retained so existing launch
    files keep working.  This node only subscribes to perception/control
    outputs and never publishes a motor command.
    """

    def __init__(self) -> None:
        super().__init__("my_rule_cone_path_visualizer")
        self.declare_parameter("path_topic", "/my_rule/cone_path")
        self.declare_parameter(
            "cluster_topic", "/my_rule/cone_fused_clusters"
        )
        self.declare_parameter("command_topic", "/my_rule/cone_cmd")
        self.declare_parameter(
            "lane_path_topic", "/rule_drive/connected_yellow_path"
        )
        self.declare_parameter(
            "lane_diagnostics_topic", "/rule_drive/diagnostics"
        )
        self.declare_parameter("control_mode_topic", "/hybrid_gate/mode")
        self.declare_parameter("manual_odometry_topic", "/odom")
        self.declare_parameter("marker_topic", "/my_rule/cone_path_markers")
        self.declare_parameter("output_frame", "laser_frame")
        self.declare_parameter("lidar_to_rear_axle_m", 0.42)
        self.declare_parameter("lookahead_min_m", 0.7)
        self.declare_parameter("lookahead_max_m", 1.45)
        self.declare_parameter("lookahead_scale", 0.12)
        self.declare_parameter("min_range_m", 0.18)
        self.declare_parameter("max_range_m", 2.2)
        self.declare_parameter("scan_front_min_deg", -94.0)
        self.declare_parameter("scan_front_max_deg", 94.0)
        self.declare_parameter("seed_min_angle_deg", 15.0)
        self.declare_parameter("seed_max_angle_deg", 90.0)
        self.declare_parameter("seed_max_range_m", 2.2)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("data_timeout_sec", 0.5)
        self.declare_parameter("control_mode_timeout_sec", 0.5)
        self.declare_parameter("gate_paths_by_control_mode", True)
        self.declare_parameter("path_point_stride", 5)
        self.declare_parameter("manual_path_enabled", True)
        self.declare_parameter("manual_path_min_spacing_m", 0.03)
        self.declare_parameter("manual_path_max_points", 10000)

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            marker_qos,
        )
        self.path_sub = self.create_subscription(
            Path,
            str(self.get_parameter("path_topic").value),
            self.on_path,
            input_qos,
        )
        self.cluster_sub = self.create_subscription(
            PoseArray,
            str(self.get_parameter("cluster_topic").value),
            self.on_clusters,
            input_qos,
        )
        self.command_sub = self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("command_topic").value),
            self.on_command,
            input_qos,
        )
        self.lane_path_sub = self.create_subscription(
            Centerline,
            str(self.get_parameter("lane_path_topic").value),
            self.on_lane_path,
            input_qos,
        )
        self.lane_diagnostics_sub = self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("lane_diagnostics_topic").value),
            self.on_lane_diagnostics,
            input_qos,
        )
        self.control_mode_sub = self.create_subscription(
            String,
            str(self.get_parameter("control_mode_topic").value),
            self.on_control_mode,
            input_qos,
        )
        self.manual_odometry_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("manual_odometry_topic").value),
            self.on_manual_odometry,
            input_qos,
        )

        self.latest_path: List[Point2] = []
        self.latest_path_frame = "rear_axle"
        self.latest_clusters: List[Point2] = []
        self.latest_cluster_frame = "laser_frame"
        self.clusters_received_at = 0.0
        self.latest_command = (0.0, 0.0, 0.0)
        self.latest_lane_path: List[Point2] = []
        self.latest_lane_frame = "laser_frame"
        self.latest_lane_source = "unknown"
        self.latest_lane_confidence = 0.0
        self.lane_path_received = False
        self.lane_path_received_at = 0.0
        self.latest_lane_diagnostics: List[float] = []
        self.lane_diagnostics_received_at = 0.0
        self.latest_control_mode = ""
        self.control_mode_received_at = 0.0
        self.path_received = False
        self.path_received_at = 0.0
        self.command_received_at = 0.0
        self.unsupported_path_frame = ""
        self.unsupported_lane_frame = ""
        self.manual_path: List[Point2] = []
        self.manual_current_position: Optional[Point2] = None
        self.manual_current_yaw = 0.0
        self.manual_last_stamp_ns: Optional[int] = None

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.publish_markers)
        self.get_logger().info(
            "lane/cone RViz visualizer ready: "
            f"lane={self.get_parameter('lane_path_topic').value}, "
            f"cone={self.get_parameter('path_topic').value}, "
            f"manual={self.get_parameter('manual_odometry_topic').value} -> "
            f"{self.get_parameter('marker_topic').value}"
        )

    def on_path(self, message: Path) -> None:
        self.latest_path = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        ]
        self.latest_path_frame = message.header.frame_id or "rear_axle"
        self.path_received = True
        self.path_received_at = time.monotonic()

    def on_clusters(self, message: PoseArray) -> None:
        self.latest_clusters = [
            (float(pose.position.x), float(pose.position.y))
            for pose in message.poses
        ]
        self.latest_cluster_frame = message.header.frame_id or "laser_frame"
        self.clusters_received_at = time.monotonic()

    def on_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 3:
            return
        self.latest_command = tuple(float(value) for value in message.data[:3])
        self.command_received_at = time.monotonic()

    def on_lane_path(self, message: Centerline) -> None:
        self.latest_lane_path = [
            (float(point.x), float(point.y)) for point in message.points
        ]
        self.latest_lane_frame = message.header.frame_id or "laser_frame"
        self.latest_lane_source = str(message.source) or "unknown"
        self.latest_lane_confidence = float(message.confidence)
        self.lane_path_received = True
        self.lane_path_received_at = time.monotonic()

    def on_lane_diagnostics(self, message: Float32MultiArray) -> None:
        self.latest_lane_diagnostics = [float(value) for value in message.data]
        self.lane_diagnostics_received_at = time.monotonic()

    def on_control_mode(self, message: String) -> None:
        control_mode = str(message.data).strip().upper()
        mode_changed = control_mode != self.latest_control_mode
        self.latest_control_mode = control_mode
        self.control_mode_received_at = time.monotonic()
        if mode_changed:
            # Do not wait for the next timer tick to remove the path owned by
            # the controller that just became inactive.
            self.publish_markers()

    def on_manual_odometry(self, message: Odometry) -> None:
        if not bool(self.get_parameter("manual_path_enabled").value):
            return
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if (
            self.manual_last_stamp_ns is not None
            and stamp_ns < self.manual_last_stamp_ns
        ):
            # A backwards timestamp means bag replay restarted or looped.
            self.manual_path.clear()
        self.manual_last_stamp_ns = stamp_ns

        position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        orientation = message.pose.pose.orientation
        self.manual_current_position = position
        self.manual_current_yaw = yaw_from_quaternion(
            orientation.z, orientation.w
        )

        spacing = max(
            0.0,
            float(self.get_parameter("manual_path_min_spacing_m").value),
        )
        if self.manual_path:
            distance = math.hypot(
                position[0] - self.manual_path[-1][0],
                position[1] - self.manual_path[-1][1],
            )
            if distance < spacing:
                return
        self.manual_path.append(position)
        maximum = max(
            2, int(self.get_parameter("manual_path_max_points").value)
        )
        if len(self.manual_path) > maximum:
            del self.manual_path[: len(self.manual_path) - maximum]

    def marker(self, namespace: str, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = normalize_frame_id(
            str(self.get_parameter("output_frame").value)
        )
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def transformed_path(self) -> Optional[List[Point2]]:
        return transform_planar_points(
            self.latest_path,
            self.latest_path_frame,
            str(self.get_parameter("output_frame").value),
            float(self.get_parameter("lidar_to_rear_axle_m").value),
        )

    def transformed_clusters(self) -> Optional[List[Point2]]:
        return transform_planar_points(
            self.latest_clusters,
            self.latest_cluster_frame,
            str(self.get_parameter("output_frame").value),
            float(self.get_parameter("lidar_to_rear_axle_m").value),
        )

    def transformed_lane_path(self) -> Optional[List[Point2]]:
        return transform_planar_points(
            self.latest_lane_path,
            self.latest_lane_frame,
            str(self.get_parameter("output_frame").value),
            float(self.get_parameter("lidar_to_rear_axle_m").value),
        )

    def publish_markers(self) -> None:
        markers = MarkerArray()
        delete_all = self.marker("cleanup", 0, Marker.ARROW)
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        self.append_fov(markers)
        self.append_vehicle_reference(markers)
        self.append_manual_path(markers)
        clusters = (
            self.transformed_clusters()
            if self.clusters_are_fresh()
            else []
        )
        if clusters:
            self.append_clusters(markers, clusters)

        show_cone_path, show_lane_path = self.visible_paths()
        self.append_inactive_path_deletes(
            markers,
            show_cone_path=show_cone_path,
            show_lane_path=show_lane_path,
        )

        if show_cone_path:
            cone_path = self.transformed_path()
            if cone_path is None:
                frame = normalize_frame_id(self.latest_path_frame)
                if frame != self.unsupported_path_frame:
                    self.unsupported_path_frame = frame
                    self.get_logger().warn(
                        "cannot visualize cone path frame "
                        f"'{frame}' without a TF transform"
                    )
            elif cone_path:
                self.unsupported_path_frame = ""
                self.append_path(markers, cone_path)
                self.append_lookahead(markers, cone_path)

        if show_lane_path:
            lane_path = self.transformed_lane_path()
            if lane_path is None:
                frame = normalize_frame_id(self.latest_lane_frame)
                if frame != self.unsupported_lane_frame:
                    self.unsupported_lane_frame = frame
                    self.get_logger().warn(
                        "cannot visualize lane path frame "
                        f"'{frame}' without a TF transform"
                    )
            elif lane_path:
                self.unsupported_lane_frame = ""
                self.append_lane_path(markers, lane_path)
                self.append_lane_target(markers)

        self.marker_pub.publish(markers)

    def append_manual_path(self, markers: MarkerArray) -> None:
        if (
            not bool(self.get_parameter("manual_path_enabled").value)
            or self.manual_current_position is None
            or not self.manual_path
        ):
            for namespace, marker_id in MANUAL_PATH_MARKERS:
                marker = self.marker(namespace, marker_id, Marker.ARROW)
                marker.action = Marker.DELETE
                markers.markers.append(marker)
            return

        relative_path = odom_trace_in_vehicle_frame(
            self.manual_path,
            self.manual_current_position,
            self.manual_current_yaw,
        )
        line = self.marker("manual_driven_path", 40, Marker.LINE_STRIP)
        line.scale.x = 0.055
        line.color.r = 0.10
        line.color.g = 1.0
        line.color.b = 0.20
        line.color.a = 1.0
        line.points = [make_point(x, y, 0.115) for x, y in relative_path]
        markers.markers.append(line)

        current = self.marker("manual_vehicle_position", 41, Marker.SPHERE)
        current.pose.position = make_point(0.0, 0.0, 0.13)
        current.scale.x = 0.11
        current.scale.y = 0.11
        current.scale.z = 0.08
        current.color.r = 0.05
        current.color.g = 1.0
        current.color.b = 0.15
        current.color.a = 1.0
        markers.markers.append(current)

    def append_inactive_path_deletes(
        self,
        markers: MarkerArray,
        *,
        show_cone_path: bool,
        show_lane_path: bool,
    ) -> None:
        marker_keys = []
        if not show_cone_path:
            marker_keys.extend(CONE_PATH_MARKERS)
        if not show_lane_path:
            marker_keys.extend(LANE_PATH_MARKERS)
        for namespace, marker_id in marker_keys:
            marker = self.marker(namespace, marker_id, Marker.ARROW)
            marker.action = Marker.DELETE
            markers.markers.append(marker)

    def visible_paths(self) -> Tuple[bool, bool]:
        if not bool(
            self.get_parameter("gate_paths_by_control_mode").value
        ):
            return True, True
        timeout = max(
            0.0,
            float(self.get_parameter("control_mode_timeout_sec").value),
        )
        if (
            not self.latest_control_mode
            or time.monotonic() - self.control_mode_received_at > timeout
        ):
            return False, False
        return path_visibility_for_control_mode(self.latest_control_mode)

    def append_fov(self, markers: MarkerArray) -> None:
        minimum_range = float(self.get_parameter("min_range_m").value)
        maximum_range = float(self.get_parameter("max_range_m").value)
        minimum_angle = float(
            self.get_parameter("scan_front_min_deg").value
        )
        maximum_angle = float(
            self.get_parameter("scan_front_max_deg").value
        )

        accepted_fill = self.marker(
            "accepted_scan_sector_fill", 30, Marker.TRIANGLE_LIST
        )
        accepted_fill.scale.x = 1.0
        accepted_fill.scale.y = 1.0
        accepted_fill.scale.z = 1.0
        accepted_fill.color.r = 0.95
        accepted_fill.color.g = 0.58
        accepted_fill.color.b = 0.10
        accepted_fill.color.a = 0.14
        accepted_fill.points = [
            make_point(x, y, 0.001)
            for x, y in sector_triangle_points(
                minimum_range,
                maximum_range,
                minimum_angle,
                maximum_angle,
            )
        ]
        markers.markers.append(accepted_fill)

        seed_minimum = float(
            self.get_parameter("seed_min_angle_deg").value
        )
        seed_maximum = float(
            self.get_parameter("seed_max_angle_deg").value
        )
        seed_range = float(self.get_parameter("seed_max_range_m").value)
        seed_fill = self.marker(
            "cone_seed_regions_fill", 31, Marker.TRIANGLE_LIST
        )
        seed_fill.scale.x = 1.0
        seed_fill.scale.y = 1.0
        seed_fill.scale.z = 1.0
        seed_fill.color.r = 1.0
        seed_fill.color.g = 0.42
        seed_fill.color.b = 0.02
        seed_fill.color.a = 0.28
        seed_points = [
            *sector_triangle_points(
                minimum_range,
                seed_range,
                seed_minimum,
                seed_maximum,
            ),
            *sector_triangle_points(
                minimum_range,
                seed_range,
                -seed_maximum,
                -seed_minimum,
            ),
        ]
        seed_fill.points = [
            make_point(x, y, 0.002) for x, y in seed_points
        ]
        markers.markers.append(seed_fill)

        outline = self.marker("accepted_scan_sector", 1, Marker.LINE_STRIP)
        outline.scale.x = 0.015
        outline.color.r = 0.35
        outline.color.g = 0.40
        outline.color.b = 0.48
        outline.color.a = 0.75
        outline.points = [
            make_point(x, y, 0.005)
            for x, y in fov_outline_points(
                maximum_range,
                minimum_angle,
                maximum_angle,
            )
        ]
        markers.markers.append(outline)

    def append_vehicle_reference(self, markers: MarkerArray) -> None:
        offset = float(self.get_parameter("lidar_to_rear_axle_m").value)
        axle = self.marker("vehicle_reference", 2, Marker.SPHERE)
        axle.pose.position = make_point(-offset, 0.0, 0.035)
        axle.scale.x = 0.09
        axle.scale.y = 0.09
        axle.scale.z = 0.07
        axle.color.r = 0.15
        axle.color.g = 0.55
        axle.color.b = 1.0
        axle.color.a = 1.0
        markers.markers.append(axle)

    def append_clusters(self, markers: MarkerArray, clusters: Sequence[Point2]) -> None:
        marker = self.marker(
            "yolo_lidar_fused_cone_clusters", 4, Marker.SPHERE_LIST
        )
        marker.scale.x = 0.09
        marker.scale.y = 0.09
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 0.42
        marker.color.b = 0.05
        marker.color.a = 1.0
        marker.points = [make_point(x, y, 0.06) for x, y in clusters]
        markers.markers.append(marker)

    def append_path(self, markers: MarkerArray, path: Sequence[Point2]) -> None:
        fresh = self.path_is_fresh()
        line = self.marker("planned_waypoint_line", 5, Marker.LINE_STRIP)
        line.scale.x = 0.055
        line.color.r = 1.0
        line.color.g = 0.42 if fresh else 0.25
        line.color.b = 0.05
        line.color.a = 1.0
        line.points = [make_point(x, y, 0.055) for x, y in path]
        markers.markers.append(line)

        samples = self.marker("planned_waypoints", 6, Marker.SPHERE_LIST)
        samples.scale.x = 0.045
        samples.scale.y = 0.045
        samples.scale.z = 0.045
        samples.color.r = 1.0
        samples.color.g = 0.65
        samples.color.b = 0.12
        samples.color.a = 0.9
        stride = max(1, int(self.get_parameter("path_point_stride").value))
        sampled = list(path[::stride])
        if sampled[-1] != path[-1]:
            sampled.append(path[-1])
        samples.points = [make_point(x, y, 0.065) for x, y in sampled]
        markers.markers.append(samples)

        start = self.marker("planned_path_endpoints", 7, Marker.SPHERE)
        start.pose.position = make_point(path[0][0], path[0][1], 0.08)
        start.scale.x = 0.10
        start.scale.y = 0.10
        start.scale.z = 0.10
        start.color.r = 1.0
        start.color.g = 0.55
        start.color.b = 0.08
        start.color.a = 1.0
        markers.markers.append(start)

        finish = self.marker("planned_path_endpoints", 8, Marker.SPHERE)
        finish.pose.position = make_point(path[-1][0], path[-1][1], 0.08)
        finish.scale.x = 0.11
        finish.scale.y = 0.11
        finish.scale.z = 0.11
        finish.color.r = 1.0
        finish.color.g = 0.30
        finish.color.b = 0.02
        finish.color.a = 1.0
        markers.markers.append(finish)

    def append_lookahead(self, markers: MarkerArray, path: Sequence[Point2]) -> None:
        index, _lookahead = select_lookahead_index(
            self.latest_path,
            float(self.get_parameter("lookahead_min_m").value),
            float(self.get_parameter("lookahead_max_m").value),
            float(self.get_parameter("lookahead_scale").value),
        )
        if index is None or index >= len(path):
            return
        target_x, target_y = path[index]

        target = self.marker("pure_pursuit_target", 9, Marker.SPHERE)
        target.pose.position = make_point(target_x, target_y, 0.09)
        target.scale.x = 0.13
        target.scale.y = 0.13
        target.scale.z = 0.13
        target.color.r = 1.0
        target.color.g = 0.95
        target.color.b = 0.0
        target.color.a = 1.0
        markers.markers.append(target)

    def append_lane_path(
        self, markers: MarkerArray, path: Sequence[Point2]
    ) -> None:
        fresh = self.lane_path_is_fresh()
        line = self.marker("lane_planned_waypoint_line", 20, Marker.LINE_STRIP)
        line.scale.x = 0.065
        line.color.r = 0.10 if fresh else 1.0
        line.color.g = 0.85 if fresh else 0.72
        line.color.b = 1.0 if fresh else 0.05
        line.color.a = 1.0
        line.points = [make_point(x, y, 0.085) for x, y in path]
        markers.markers.append(line)

        waypoints = self.marker("lane_planned_waypoints", 21, Marker.SPHERE_LIST)
        waypoints.scale.x = 0.055
        waypoints.scale.y = 0.055
        waypoints.scale.z = 0.055
        waypoints.color.r = 0.15
        waypoints.color.g = 0.65
        waypoints.color.b = 1.0
        waypoints.color.a = 0.95
        stride = max(1, int(self.get_parameter("path_point_stride").value))
        sampled = list(path[::stride])
        if sampled[-1] != path[-1]:
            sampled.append(path[-1])
        waypoints.points = [make_point(x, y, 0.095) for x, y in sampled]
        markers.markers.append(waypoints)

    def append_lane_target(self, markers: MarkerArray) -> None:
        timeout = max(0.0, float(self.get_parameter("data_timeout_sec").value))
        if (
            len(self.latest_lane_diagnostics) < 16
            or time.monotonic() - self.lane_diagnostics_received_at > timeout
        ):
            return
        target = transform_planar_points(
            [
                (
                    self.latest_lane_diagnostics[14],
                    self.latest_lane_diagnostics[15],
                )
            ],
            self.latest_lane_frame,
            str(self.get_parameter("output_frame").value),
            float(self.get_parameter("lidar_to_rear_axle_m").value),
        )
        if not target:
            return
        target_x, target_y = target[0]
        marker = self.marker("lane_control_target", 22, Marker.SPHERE)
        marker.pose.position = make_point(target_x, target_y, 0.13)
        marker.scale.x = 0.14
        marker.scale.y = 0.14
        marker.scale.z = 0.14
        marker.color.r = 0.10
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 1.0
        markers.markers.append(marker)

    def path_is_fresh(self) -> bool:
        timeout = max(0.0, float(self.get_parameter("data_timeout_sec").value))
        return (
            self.path_received
            and time.monotonic() - self.path_received_at <= timeout
        )

    def clusters_are_fresh(self) -> bool:
        timeout = max(0.0, float(self.get_parameter("data_timeout_sec").value))
        return (
            self.clusters_received_at > 0.0
            and time.monotonic() - self.clusters_received_at <= timeout
        )

    def lane_path_is_fresh(self) -> bool:
        timeout = max(0.0, float(self.get_parameter("data_timeout_sec").value))
        return (
            self.lane_path_received
            and time.monotonic() - self.lane_path_received_at <= timeout
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConePathVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
