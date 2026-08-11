#!/usr/bin/env python3
"""Publish RViz markers for the LiDAR cone planner without affecting control."""

import math
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseArray
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


Point2 = Tuple[float, float]


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
    if source == "rear_axle" and output == "laser_frame":
        return [(x - offset, y) for x, y in values]
    if source == "laser_frame" and output == "rear_axle":
        return [(x + offset, y) for x, y in values]
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


def make_point(x: float, y: float, z: float = 0.0) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


class ConePathVisualizer(Node):
    """Convert cone-planner outputs into an RViz-friendly MarkerArray."""

    def __init__(self) -> None:
        super().__init__("my_rule_cone_path_visualizer")
        self.declare_parameter("path_topic", "/my_rule/cone_path")
        self.declare_parameter("cluster_topic", "/my_rule/cone_clusters")
        self.declare_parameter("command_topic", "/my_rule/cone_cmd")
        self.declare_parameter("marker_topic", "/my_rule/cone_path_markers")
        self.declare_parameter("output_frame", "laser_frame")
        self.declare_parameter("lidar_to_rear_axle_m", 0.42)
        self.declare_parameter("lookahead_min_m", 0.7)
        self.declare_parameter("lookahead_max_m", 1.45)
        self.declare_parameter("lookahead_scale", 0.12)
        self.declare_parameter("max_range_m", 2.2)
        self.declare_parameter("scan_front_min_deg", -94.0)
        self.declare_parameter("scan_front_max_deg", 94.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("data_timeout_sec", 0.5)
        self.declare_parameter("path_point_stride", 5)

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

        self.latest_path: List[Point2] = []
        self.latest_path_frame = "rear_axle"
        self.latest_clusters: List[Point2] = []
        self.latest_cluster_frame = "laser_frame"
        self.latest_command = (0.0, 0.0, 0.0)
        self.path_received = False
        self.path_received_at = 0.0
        self.command_received_at = 0.0
        self.unsupported_path_frame = ""

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.publish_markers)
        self.get_logger().info(
            "cone path RViz visualizer ready: "
            f"{self.get_parameter('path_topic').value} -> "
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

    def on_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 3:
            return
        self.latest_command = tuple(float(value) for value in message.data[:3])
        self.command_received_at = time.monotonic()

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

    def publish_markers(self) -> None:
        markers = MarkerArray()
        delete_all = self.marker("cleanup", 0, Marker.ARROW)
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        self.append_fov(markers)
        self.append_vehicle_reference(markers)
        clusters = self.transformed_clusters()
        if clusters:
            self.append_clusters(markers, clusters)

        path = self.transformed_path()
        if path is None:
            frame = normalize_frame_id(self.latest_path_frame)
            if frame != self.unsupported_path_frame:
                self.unsupported_path_frame = frame
                self.get_logger().warn(
                    f"cannot visualize path frame '{frame}' without a TF transform"
                )
        elif path:
            self.unsupported_path_frame = ""
            self.append_path(markers, path)
            self.append_lookahead(markers, path)
        self.append_status(markers, path)
        self.marker_pub.publish(markers)

    def append_fov(self, markers: MarkerArray) -> None:
        outline = self.marker("accepted_scan_sector", 1, Marker.LINE_STRIP)
        outline.scale.x = 0.015
        outline.color.r = 0.35
        outline.color.g = 0.40
        outline.color.b = 0.48
        outline.color.a = 0.75
        outline.points = [
            make_point(x, y, 0.005)
            for x, y in fov_outline_points(
                float(self.get_parameter("max_range_m").value),
                float(self.get_parameter("scan_front_min_deg").value),
                float(self.get_parameter("scan_front_max_deg").value),
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
        marker = self.marker("cone_clusters", 4, Marker.SPHERE_LIST)
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
        line.color.r = 0.10 if fresh else 1.0
        line.color.g = 1.0 if fresh else 0.72
        line.color.b = 0.25 if fresh else 0.05
        line.color.a = 1.0
        line.points = [make_point(x, y, 0.055) for x, y in path]
        markers.markers.append(line)

        samples = self.marker("planned_waypoints", 6, Marker.SPHERE_LIST)
        samples.scale.x = 0.045
        samples.scale.y = 0.045
        samples.scale.z = 0.045
        samples.color.r = 0.20
        samples.color.g = 0.90
        samples.color.b = 1.0
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
        start.color.r = 0.15
        start.color.g = 0.65
        start.color.b = 1.0
        start.color.a = 1.0
        markers.markers.append(start)

        finish = self.marker("planned_path_endpoints", 8, Marker.SPHERE)
        finish.pose.position = make_point(path[-1][0], path[-1][1], 0.08)
        finish.scale.x = 0.11
        finish.scale.y = 0.11
        finish.scale.z = 0.11
        finish.color.r = 1.0
        finish.color.g = 0.20
        finish.color.b = 0.75
        finish.color.a = 1.0
        markers.markers.append(finish)

    def append_lookahead(self, markers: MarkerArray, path: Sequence[Point2]) -> None:
        index, lookahead = select_lookahead_index(
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

        label = self.marker("pure_pursuit_target", 10, Marker.TEXT_VIEW_FACING)
        label.pose.position = make_point(target_x, target_y, 0.23)
        label.scale.z = 0.11
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = f"lookahead {lookahead:.2f} m"
        markers.markers.append(label)

    def path_is_fresh(self) -> bool:
        timeout = max(0.0, float(self.get_parameter("data_timeout_sec").value))
        return self.path_received and time.monotonic() - self.path_received_at <= timeout

    def append_status(
        self,
        markers: MarkerArray,
        transformed_path: Optional[Sequence[Point2]],
    ) -> None:
        status = self.marker("planner_status", 11, Marker.TEXT_VIEW_FACING)
        maximum_range = float(self.get_parameter("max_range_m").value)
        status.pose.position = make_point(maximum_range * 0.78, -1.15, 0.18)
        status.scale.z = 0.13
        status.color.a = 1.0

        if transformed_path is None:
            headline = f"UNSUPPORTED FRAME: {self.latest_path_frame}"
            status.color.r, status.color.g, status.color.b = 1.0, 0.15, 0.15
        elif not self.path_received:
            headline = "WAITING FOR CONE PATH"
            status.color.r, status.color.g, status.color.b = 0.8, 0.8, 0.8
        elif not transformed_path:
            headline = "NO VALID PATH"
            status.color.r, status.color.g, status.color.b = 1.0, 0.15, 0.15
        elif not self.path_is_fresh():
            headline = f"PATH STALE ({len(transformed_path)} points)"
            status.color.r, status.color.g, status.color.b = 1.0, 0.72, 0.05
        else:
            headline = f"PATH ACTIVE ({len(transformed_path)} points)"
            status.color.r, status.color.g, status.color.b = 0.15, 1.0, 0.25

        steer, speed, confidence = self.latest_command
        status.text = (
            f"{headline}\n"
            f"steer {steer:+.1f} deg | speed {speed:.1f} | conf {confidence:.2f}"
        )
        markers.markers.append(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConePathVisualizer()
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
