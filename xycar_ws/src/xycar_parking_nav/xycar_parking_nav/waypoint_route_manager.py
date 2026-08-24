#!/usr/bin/env python3
"""Capture RViz waypoints, plan a map route, and write a Nav2 mission."""

from __future__ import annotations

import math
from pathlib import Path

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .grid_planner import PlannedRoute, load_map_grid, plan_waypoint_route
from .waypoint_route_core import (
    ReferenceLocation,
    RouteWaypoint,
    normalize_reverse_waypoint_ranges,
    parking_mission_document,
    parse_reverse_waypoint_ranges,
    reference_locations_from_mission,
    reverse_range_text,
    route_waypoints_from_items,
    waypoint_document,
    waypoint_headings,
)


def _atomic_yaml_write(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    temporary.replace(path)


class WaypointRouteManager(Node):
    def __init__(self) -> None:
        super().__init__("parking_waypoint_route")
        self._declare_parameters()
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.capture_enabled = bool(
            self.get_parameter("capture_enabled").value
        )
        self.closed_route = bool(self.get_parameter("closed_route").value)
        self.reverse_range_override = str(
            self.get_parameter("reverse_waypoint_ranges").value
        ).strip()
        self.reverse_ranges = parse_reverse_waypoint_ranges(
            self.reverse_range_override
        )
        self.map_yaml = Path(
            str(self.get_parameter("map_yaml").value)
        ).expanduser().resolve()
        self.waypoints_yaml = Path(
            str(self.get_parameter("waypoints_yaml").value)
        ).expanduser().resolve()
        self.mission_output_yaml = Path(
            str(self.get_parameter("mission_output_yaml").value)
        ).expanduser().resolve()
        reference_value = str(
            self.get_parameter("reference_mission_yaml").value
        ).strip()
        self.reference_locations = self._load_reference_locations(reference_value)
        self.reference_box_length = float(
            self.get_parameter("reference_box_length_m").value
        )
        self.reference_box_width = float(
            self.get_parameter("reference_box_width_m").value
        )
        self.parking_snap_radius = float(
            self.get_parameter("parking_snap_radius_m").value
        )
        self.grid = load_map_grid(
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

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(
            PathMessage,
            str(self.get_parameter("path_topic").value),
            transient_qos,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            transient_qos,
        )
        self.reference_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("reference_marker_topic").value),
            transient_qos,
        )
        if self.capture_enabled:
            self.create_subscription(
                PointStamped,
                str(self.get_parameter("clicked_point_topic").value),
                self._on_clicked_point,
                10,
            )
        self.create_service(Trigger, "~/undo_waypoint", self._undo_waypoint)
        self.create_service(Trigger, "~/clear_waypoints", self._clear_waypoints)
        self.create_service(Trigger, "~/reload_route", self._reload_route)

        if self.waypoints_yaml.is_file():
            self._load_route()
            # Rebuild the generated mission on every capture start so route
            # conversion improvements apply without deleting/re-clicking the
            # already captured waypoint coordinates.
            if self.capture_enabled:
                self._save()
        elif not self.capture_enabled:
            raise ValueError(
                f"waypoint YAML does not exist: {self.waypoints_yaml}"
            )
        else:
            self._publish_route()
        self.add_on_set_parameters_callback(self._on_parameter_change)
        mode = "캡처" if self.capture_enabled else "주행"
        self.get_logger().info(
            f"웨이포인트 {mode} 모드 준비: {len(self.waypoints)}개, "
            f"후진범위={reverse_range_text(self.reverse_ranges) or '없음'}, "
            f"저장={self.waypoints_yaml}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("waypoints_yaml", "")
        self.declare_parameter("mission_output_yaml", "")
        self.declare_parameter("reference_mission_yaml", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("closed_route", False)
        self.declare_parameter("reverse_waypoint_ranges", "")
        self.declare_parameter("capture_enabled", False)
        self.declare_parameter("path_topic", "/parking/waypoint_route")
        self.declare_parameter("marker_topic", "/parking/waypoint_markers")
        self.declare_parameter(
            "reference_marker_topic", "/parking/reference_locations"
        )
        self.declare_parameter("reference_box_length_m", 0.80)
        self.declare_parameter("reference_box_width_m", 0.50)
        self.declare_parameter("parking_snap_radius_m", 0.45)
        self.declare_parameter("clicked_point_topic", "/clicked_point")
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
        self.declare_parameter("path_smoothing_maximum_deviation_m", -1.0)
        self.declare_parameter("clearance_cost_weight", 3.0)
        self.declare_parameter("clearance_cost_decay_m", 0.35)

    @staticmethod
    def _load_reference_locations(value: str) -> list[ReferenceLocation]:
        if not value:
            return []
        path = Path(value).expanduser().resolve()
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        return reference_locations_from_mission(document)

    def _load_route(self) -> None:
        data = yaml.safe_load(
            self.waypoints_yaml.read_text(encoding="utf-8")
        ) or {}
        configured_frame = str(data.get("frame_id", self.frame_id))
        if configured_frame != self.frame_id:
            raise ValueError(
                f"waypoint frame {configured_frame} != {self.frame_id}"
            )
        self.closed_route = bool(data.get("closed", self.closed_route))
        self.waypoints = route_waypoints_from_items(data.get("waypoints", []))
        configured_ranges = (
            parse_reverse_waypoint_ranges(self.reverse_range_override)
            if self.reverse_range_override
            else parse_reverse_waypoint_ranges(data.get("reverse_ranges", []))
        )
        self.reverse_ranges = normalize_reverse_waypoint_ranges(
            configured_ranges, len(self.waypoints)
        )
        self._replan()

    def _replan(self) -> None:
        if len(self.waypoints) < 2:
            self.route = None
            self._publish_route()
            return
        self.route = plan_waypoint_route(
            self.grid,
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
                self.get_parameter("path_smoothing_maximum_deviation_m").value
            ),
            clearance_cost_weight=float(
                self.get_parameter("clearance_cost_weight").value
            ),
            clearance_cost_decay_m=float(
                self.get_parameter("clearance_cost_decay_m").value
            ),
        )
        self._publish_route()

    def _active_reverse_ranges(self) -> tuple[tuple[int, int], ...]:
        """Return configured ranges whose endpoints have already been clicked."""

        count = len(self.waypoints)
        return tuple(
            (start, end)
            for start, end in self.reverse_ranges
            if 0 <= start < end < count
        )

    def _on_parameter_change(self, parameters) -> SetParametersResult:
        requested = next(
            (
                parameter
                for parameter in parameters
                if parameter.name == "reverse_waypoint_ranges"
            ),
            None,
        )
        if requested is None:
            return SetParametersResult(successful=True)
        if not self.capture_enabled:
            return SetParametersResult(
                successful=False,
                reason="reverse ranges can only be changed in capture mode",
            )
        try:
            parsed = parse_reverse_waypoint_ranges(str(requested.value))
            normalized = normalize_reverse_waypoint_ranges(
                parsed, len(self.waypoints)
            )
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        previous = self.reverse_ranges
        self.reverse_ranges = normalized
        try:
            self._replan()
            self._save()
        except (OSError, ValueError, KeyError) as exc:
            self.reverse_ranges = previous
            self._replan()
            return SetParametersResult(successful=False, reason=str(exc))
        self.get_logger().info(
            "후진 웨이포인트 범위를 변경했습니다: "
            + (reverse_range_text(normalized) or "없음")
        )
        return SetParametersResult(successful=True)

    def _publish_route(self) -> None:
        stamp = self.get_clock().now().to_msg()
        path = PathMessage()
        path.header.frame_id = self.frame_id
        path.header.stamp = stamp
        points = self.route.points if self.route is not None else ()
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            if len(points) > 1:
                following = points[(index + 1) % len(points)]
                if not self.closed_route and index == len(points) - 1:
                    following = point
                    previous = points[index - 1]
                    yaw = math.atan2(
                        point[1] - previous[1], point[0] - previous[0]
                    )
                else:
                    yaw = math.atan2(
                        following[1] - point[1], following[0] - point[0]
                    )
                pose.pose.orientation.z = math.sin(yaw * 0.5)
                pose.pose.orientation.w = math.cos(yaw * 0.5)
            else:
                pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_publisher.publish(path)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        active_reverse_ranges = self._active_reverse_ranges()
        reverse_segment_origins = {
            origin
            for start, end in active_reverse_ranges
            for origin in range(start, end)
        }
        reverse_waypoint_indices = {
            index
            for start, end in active_reverse_ranges
            for index in range(start, end + 1)
        }
        headings = (
            waypoint_headings(
                self.waypoints,
                closed=self.closed_route,
                reverse_ranges=active_reverse_ranges,
            )
            if len(self.waypoints) >= 2
            else [self.waypoints[0].yaw or 0.0]
            if self.waypoints
            else []
        )
        for index, waypoint in enumerate(self.waypoints):
            is_reverse = index in reverse_waypoint_indices
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "parking_waypoints"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.x
            marker.pose.position.y = waypoint.y
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.14
            marker.scale.y = 0.14
            marker.scale.z = 0.14
            marker.color.r = 1.0 if is_reverse else 0.1
            marker.color.g = 0.45 if is_reverse else 0.95
            marker.color.b = 0.05 if is_reverse else 0.2
            marker.color.a = 1.0
            markers.markers.append(marker)

            heading = Marker()
            heading.header = marker.header
            heading.ns = "parking_waypoint_headings"
            heading.id = index
            heading.type = Marker.ARROW
            heading.action = Marker.ADD
            heading.pose.position.x = waypoint.x
            heading.pose.position.y = waypoint.y
            heading.pose.position.z = 0.10
            heading.pose.orientation.z = math.sin(headings[index] * 0.5)
            heading.pose.orientation.w = math.cos(headings[index] * 0.5)
            heading.scale.x = 0.30
            heading.scale.y = 0.065
            heading.scale.z = 0.065
            heading.color.r = 1.0 if is_reverse else 0.1
            heading.color.g = 0.45 if is_reverse else 0.95
            heading.color.b = 0.05 if is_reverse else 0.2
            heading.color.a = 1.0
            markers.markers.append(heading)

            label = Marker()
            label.header = marker.header
            label.ns = "parking_waypoint_numbers"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = waypoint.x
            label.pose.position.y = waypoint.y
            label.pose.position.z = 0.30
            label.pose.orientation.w = 1.0
            label.scale.z = 0.23
            label.text = f"{index}{' R' if is_reverse else ''}"
            label.color.r = 1.0
            label.color.g = 0.65 if is_reverse else 1.0
            label.color.b = 0.05 if is_reverse else 1.0
            label.color.a = 1.0
            markers.markers.append(label)

        if reverse_segment_origins:
            reverse_lines = Marker()
            reverse_lines.header.frame_id = self.frame_id
            reverse_lines.header.stamp = stamp
            reverse_lines.ns = "parking_reverse_segments"
            reverse_lines.id = 0
            reverse_lines.type = Marker.LINE_LIST
            reverse_lines.action = Marker.ADD
            reverse_lines.scale.x = 0.07
            reverse_lines.color.r = 1.0
            reverse_lines.color.g = 0.35
            reverse_lines.color.b = 0.02
            reverse_lines.color.a = 1.0
            for origin in sorted(reverse_segment_origins):
                start = self.waypoints[origin]
                end = self.waypoints[origin + 1]
                reverse_lines.points.extend(
                    [
                        Point(x=start.x, y=start.y, z=0.055),
                        Point(x=end.x, y=end.y, z=0.055),
                    ]
                )
            markers.markers.append(reverse_lines)
        self.marker_publisher.publish(markers)
        self._publish_reference_locations(stamp)

    @staticmethod
    def _set_color(
        marker: Marker, red: float, green: float, blue: float, alpha: float
    ) -> None:
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    def _publish_reference_locations(self, stamp) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        colors = {
            "PREVIOUS_START": (0.85, 0.20, 1.00),
            "A_PARK": (0.10, 0.70, 1.00),
            "B_PARK": (1.00, 0.45, 0.10),
        }
        half_length = 0.5 * self.reference_box_length
        half_width = 0.5 * self.reference_box_width
        local_corners = (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
        )
        for index, location in enumerate(self.reference_locations):
            red, green, blue = colors[location.name]
            cosine = math.cos(location.yaw)
            sine = math.sin(location.yaw)

            fill = Marker()
            fill.header.frame_id = self.frame_id
            fill.header.stamp = stamp
            fill.ns = "reference_location_fill"
            fill.id = index
            fill.type = Marker.CUBE
            fill.action = Marker.ADD
            fill.pose.position.x = location.x
            fill.pose.position.y = location.y
            fill.pose.position.z = 0.012
            fill.pose.orientation.z = math.sin(location.yaw * 0.5)
            fill.pose.orientation.w = math.cos(location.yaw * 0.5)
            fill.scale.x = self.reference_box_length
            fill.scale.y = self.reference_box_width
            fill.scale.z = 0.02
            self._set_color(fill, red, green, blue, 0.16)
            markers.markers.append(fill)

            outline = Marker()
            outline.header = fill.header
            outline.ns = "reference_location_outline"
            outline.id = index
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.scale.x = 0.045
            self._set_color(outline, red, green, blue, 0.95)
            for local_x, local_y in local_corners:
                outline.points.append(
                    Point(
                        x=location.x + cosine * local_x - sine * local_y,
                        y=location.y + sine * local_x + cosine * local_y,
                        z=0.035,
                    )
                )
            markers.markers.append(outline)

            arrow = Marker()
            arrow.header = fill.header
            arrow.ns = "reference_location_heading"
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = location.x
            arrow.pose.position.y = location.y
            arrow.pose.position.z = 0.07
            arrow.pose.orientation.z = math.sin(location.yaw * 0.5)
            arrow.pose.orientation.w = math.cos(location.yaw * 0.5)
            arrow.scale.x = 0.38
            arrow.scale.y = 0.09
            arrow.scale.z = 0.09
            self._set_color(arrow, red, green, blue, 1.0)
            markers.markers.append(arrow)

            label = Marker()
            label.header = fill.header
            label.ns = "reference_location_label"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = location.x
            label.pose.position.y = location.y
            label.pose.position.z = 0.22
            label.pose.orientation.w = 1.0
            label.scale.z = 0.15
            label.text = location.name
            self._set_color(label, red, green, blue, 1.0)
            markers.markers.append(label)
        self.reference_marker_publisher.publish(markers)

    def _save(self) -> None:
        active_reverse_ranges = self._active_reverse_ranges()
        _atomic_yaml_write(
            self.waypoints_yaml,
            waypoint_document(
                self.waypoints,
                frame_id=self.frame_id,
                closed=self.closed_route,
                reverse_ranges=active_reverse_ranges,
            ),
        )
        if len(self.waypoints) >= 2:
            _atomic_yaml_write(
                self.mission_output_yaml,
                parking_mission_document(
                    self.waypoints,
                    frame_id=self.frame_id,
                    closed=self.closed_route,
                    reference_locations=self.reference_locations,
                    parking_snap_radius_m=self.parking_snap_radius,
                    reverse_ranges=active_reverse_ranges,
                ),
            )
        elif self.mission_output_yaml.exists():
            self.mission_output_yaml.unlink()

    def _on_clicked_point(self, message: PointStamped) -> None:
        if message.header.frame_id != self.frame_id:
            self.get_logger().warning(
                f"클릭 좌표계가 {message.header.frame_id}입니다; "
                f"{self.frame_id} 좌표만 받습니다"
            )
            return
        waypoint = RouteWaypoint(
            name=f"WP_{len(self.waypoints):02d}",
            x=float(message.point.x),
            y=float(message.point.y),
        )
        self.waypoints.append(waypoint)
        try:
            self._replan()
            self._save()
        except (OSError, ValueError, KeyError) as exc:
            self.waypoints.pop()
            self._replan()
            self.get_logger().error(f"웨이포인트 거부: {exc}")
            return
        self.get_logger().info(
            f"웨이포인트 {len(self.waypoints)}개 저장: "
            f"{waypoint.x:.3f}, {waypoint.y:.3f}"
        )

    def _undo_waypoint(self, _request, response):
        if not self.capture_enabled:
            response.success = False
            response.message = "capture mode is disabled"
            return response
        if not self.waypoints:
            response.success = False
            response.message = "no waypoint to undo"
            return response
        self.waypoints.pop()
        self._replan()
        self._save()
        response.success = True
        response.message = "last waypoint removed"
        return response

    def _clear_waypoints(self, _request, response):
        if not self.capture_enabled:
            response.success = False
            response.message = "capture mode is disabled"
            return response
        self.waypoints.clear()
        self._replan()
        self._save()
        response.success = True
        response.message = "waypoints cleared"
        return response

    def _reload_route(self, _request, response):
        try:
            self._load_route()
        except (OSError, ValueError, KeyError) as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = "route reloaded"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointRouteManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
