from __future__ import annotations

import math
import time

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import Centerline, RoadSegment, RoadSegmentArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def make_point(x: float, y: float, z: float = 0.0) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


class LaneRuleDriver(Node):
    def __init__(self) -> None:
        super().__init__("xycar_lane_rule_driver")
        self.declare_parameter("road_segments_topic", "/perception/road_segments")
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter("target_path_topic", "/rule_drive/target_path")
        self.declare_parameter("debug_markers_topic", "/rule_drive/debug_markers")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("lane_width_m", 0.80)
        self.declare_parameter("min_lane_width_m", 0.35)
        self.declare_parameter("max_lane_width_m", 1.20)
        self.declare_parameter("min_segment_points", 4)
        self.declare_parameter("min_target_points", 3)
        self.declare_parameter("sample_count", 18)
        self.declare_parameter("wheel_base_m", 0.32)
        self.declare_parameter("steering_gain_rad_per_cmd", -0.0068)
        self.declare_parameter("angle_command_min", -42.0)
        self.declare_parameter("angle_command_max", 42.0)
        self.declare_parameter("lookahead_distance_m", 1.20)
        self.declare_parameter("min_lookahead_x_m", 0.45)
        self.declare_parameter("speed_command", 8.0)
        self.declare_parameter("min_speed_command", 5.0)
        self.declare_parameter("slow_down_angle_cmd", 18.0)
        self.declare_parameter("max_abs_angle_for_drive", 38.0)
        self.declare_parameter("perception_timeout_sec", 0.40)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("hold_last_path_sec", 0.25)

        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.lane_width_m = float(self.get_parameter("lane_width_m").value)
        self.min_lane_width_m = float(self.get_parameter("min_lane_width_m").value)
        self.max_lane_width_m = float(self.get_parameter("max_lane_width_m").value)
        self.min_segment_points = int(self.get_parameter("min_segment_points").value)
        self.min_target_points = int(self.get_parameter("min_target_points").value)
        self.sample_count = max(3, int(self.get_parameter("sample_count").value))
        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)
        self.steering_gain_rad_per_cmd = float(
            self.get_parameter("steering_gain_rad_per_cmd").value
        )
        self.angle_command_min = float(self.get_parameter("angle_command_min").value)
        self.angle_command_max = float(self.get_parameter("angle_command_max").value)
        self.lookahead_distance_m = float(self.get_parameter("lookahead_distance_m").value)
        self.min_lookahead_x_m = float(self.get_parameter("min_lookahead_x_m").value)
        self.speed_command = float(self.get_parameter("speed_command").value)
        self.min_speed_command = float(self.get_parameter("min_speed_command").value)
        self.slow_down_angle_cmd = float(self.get_parameter("slow_down_angle_cmd").value)
        self.max_abs_angle_for_drive = float(self.get_parameter("max_abs_angle_for_drive").value)
        self.perception_timeout_sec = float(self.get_parameter("perception_timeout_sec").value)
        self.hold_last_path_sec = float(self.get_parameter("hold_last_path_sec").value)

        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("motor_topic").value),
            10,
        )
        self.target_path_pub = self.create_publisher(
            Centerline,
            str(self.get_parameter("target_path_topic").value),
            10,
        )
        self.debug_markers_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("debug_markers_topic").value),
            10,
        )
        self.create_subscription(
            RoadSegmentArray,
            str(self.get_parameter("road_segments_topic").value),
            self.on_road_segments,
            10,
        )

        rate_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self.on_timer)

        self.last_segments_time = 0.0
        self.last_target_path: list[Point] = []
        self.last_header = None
        self.last_angle_command = 0.0
        self.get_logger().info(
            "lane rule driver ready: yellow centerline + outer white line -> /xycar_motor"
        )

    def on_road_segments(self, msg: RoadSegmentArray) -> None:
        target_path = self.build_target_path(msg)
        if len(target_path) < self.min_target_points:
            return
        self.last_target_path = target_path
        self.last_header = msg.header
        self.last_segments_time = time.monotonic()
        self.publish_target_path(msg.header, target_path)
        self.publish_debug_markers(msg.header, target_path)

    def build_target_path(self, msg: RoadSegmentArray) -> list[Point]:
        yellow_segments = [
            segment
            for segment in msg.segments
            if segment.type in {RoadSegment.TYPE_YESOL, RoadSegment.TYPE_YEDOT}
            and len(segment.points) >= self.min_segment_points
        ]
        white_segments = [
            segment
            for segment in msg.segments
            if segment.type in {RoadSegment.TYPE_WHSOL, RoadSegment.TYPE_WHDOT}
            and len(segment.points) >= self.min_segment_points
        ]
        if not yellow_segments or not white_segments:
            return []

        yellow = max(
            yellow_segments,
            key=lambda segment: float(segment.confidence) * max(1, len(segment.points)),
        )
        best_path: list[Point] = []
        best_score = -1.0
        for white in white_segments:
            path, score = self.mid_path_between_segments(yellow, white)
            if score > best_score:
                best_path = path
                best_score = score
        return best_path

    def mid_path_between_segments(
        self,
        yellow: RoadSegment,
        white: RoadSegment,
    ) -> tuple[list[Point], float]:
        bounds = self.overlap_x_bounds(yellow.points, white.points)
        if bounds is None:
            return [], -1.0
        start_x, end_x = bounds
        if end_x <= start_x:
            return [], -1.0

        path = []
        lane_width_errors = []
        for index in range(self.sample_count):
            ratio = index / float(self.sample_count - 1)
            x = start_x + (end_x - start_x) * ratio
            yellow_y = self.y_at_x(yellow.points, x)
            white_y = self.y_at_x(white.points, x)
            if yellow_y is None or white_y is None:
                continue
            width = abs(yellow_y - white_y)
            if width < self.min_lane_width_m or width > self.max_lane_width_m:
                continue
            path.append(make_point(x, (yellow_y + white_y) * 0.5, 0.0))
            lane_width_errors.append(abs(width - self.lane_width_m))

        if len(path) < self.min_target_points:
            return [], -1.0
        mean_error = sum(lane_width_errors) / max(1, len(lane_width_errors))
        score = len(path) - 3.0 * mean_error
        return path, score

    def on_timer(self) -> None:
        age = time.monotonic() - self.last_segments_time
        if not self.last_target_path or age > self.perception_timeout_sec + self.hold_last_path_sec:
            self.publish_motor(0.0, 0.0)
            return

        target = self.lookahead_point(self.last_target_path)
        if target is None:
            self.publish_motor(0.0, 0.0)
            return

        angle_command = self.compute_angle_command(target)
        speed_command = self.compute_speed_command(angle_command, age)
        self.last_angle_command = angle_command
        self.publish_motor(angle_command, speed_command)

    def lookahead_point(self, path: list[Point]) -> Point | None:
        forward = [point for point in path if point.x >= self.min_lookahead_x_m]
        if not forward:
            return None
        return min(forward, key=lambda point: abs(point.x - self.lookahead_distance_m))

    def compute_angle_command(self, target: Point) -> float:
        distance_sq = max(0.05, target.x * target.x + target.y * target.y)
        curvature = 2.0 * target.y / distance_sq
        steering_rad = math.atan(self.wheel_base_m * curvature)
        if abs(self.steering_gain_rad_per_cmd) < 1.0e-6:
            return 0.0
        angle_command = steering_rad / self.steering_gain_rad_per_cmd
        return clamp(angle_command, self.angle_command_min, self.angle_command_max)

    def compute_speed_command(self, angle_command: float, perception_age: float) -> float:
        if perception_age > self.perception_timeout_sec:
            return 0.0
        abs_angle = abs(angle_command)
        if abs_angle >= self.max_abs_angle_for_drive:
            return 0.0
        if abs_angle <= self.slow_down_angle_cmd:
            return self.speed_command
        ratio = (abs_angle - self.slow_down_angle_cmd) / max(
            1.0,
            self.max_abs_angle_for_drive - self.slow_down_angle_cmd,
        )
        return self.speed_command + ratio * (self.min_speed_command - self.speed_command)

    def publish_motor(self, angle: float, speed: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(msg)

    def publish_target_path(self, header, path: list[Point]) -> None:
        msg = Centerline()
        msg.header = header
        if not msg.header.frame_id:
            msg.header.frame_id = self.base_frame_id
        msg.detection_id = 0
        msg.track_id = 0
        msg.points = path
        msg.confidence = 1.0
        msg.source = "xycar_lane_rule_driver"
        self.target_path_pub.publish(msg)

    def publish_debug_markers(self, header, path: list[Point]) -> None:
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.header = header
        if not delete_all.header.frame_id:
            delete_all.header.frame_id = self.base_frame_id
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        path_marker = self.line_marker(delete_all.header, 1, "rule_target_path", path)
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.25
        path_marker.scale.x = 0.04
        markers.markers.append(path_marker)

        lookahead = self.lookahead_point(path)
        if lookahead is not None:
            point_marker = Marker()
            point_marker.header = delete_all.header
            point_marker.ns = "rule_lookahead"
            point_marker.id = 2
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position = lookahead
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = 0.10
            point_marker.scale.y = 0.10
            point_marker.scale.z = 0.10
            point_marker.color.a = 1.0
            point_marker.color.r = 0.1
            point_marker.color.g = 0.45
            point_marker.color.b = 1.0
            markers.markers.append(point_marker)
        self.debug_markers_pub.publish(markers)

    def line_marker(self, header, marker_id: int, namespace: str, points: list[Point]) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.color.a = 1.0
        marker.points = points
        return marker

    def overlap_x_bounds(self, first: list[Point], second: list[Point]) -> tuple[float, float] | None:
        if len(first) < 2 or len(second) < 2:
            return None
        start_x = max(min(point.x for point in first), min(point.x for point in second))
        end_x = min(max(point.x for point in first), max(point.x for point in second))
        return start_x, end_x

    def y_at_x(self, points: list[Point], target_x: float) -> float | None:
        if not points:
            return None
        sorted_points = sorted(points, key=lambda point: point.x)
        if target_x <= sorted_points[0].x:
            return float(sorted_points[0].y)
        if target_x >= sorted_points[-1].x:
            return float(sorted_points[-1].y)
        for start, end in zip(sorted_points, sorted_points[1:]):
            min_x = min(float(start.x), float(end.x))
            max_x = max(float(start.x), float(end.x))
            if target_x < min_x or target_x > max_x:
                continue
            dx = float(end.x) - float(start.x)
            if abs(dx) < 1.0e-6:
                return float(start.y)
            ratio = (target_x - float(start.x)) / dx
            return float(start.y) + ratio * (float(end.y) - float(start.y))
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneRuleDriver()
    try:
        rclpy.spin(node)
    finally:
        node.publish_motor(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
