from __future__ import annotations

from dataclasses import dataclass
import math
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from kaiev26_msgs.msg import (
    Centerline,
    PerceptionObjectArray,
    RoadSegment,
    RoadSegmentArray,
    TrafficLightObservationArray,
)
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class SegmentCandidate:
    segment_type: int
    points: list[Point]
    confidence: float


def make_point(x: float, y: float, z: float = 0.0) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def moving_average(values: list[float], window: int = 3) -> list[float]:
    if len(values) < window or window <= 1:
        return values
    half = window // 2
    smoothed = []
    for index in range(len(values)):
        start = max(0, index - half)
        end = min(len(values), index + half + 1)
        smoothed.append(float(np.mean(values[start:end])))
    return smoothed


class CameraPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("xycar_camera_perception")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("road_segments_topic", "/perception/road_segments")
        self.declare_parameter("centerline_topic", "/perception/centerline")
        self.declare_parameter("objects_topic", "/perception/objects")
        self.declare_parameter("traffic_lights_topic", "/perception/traffic_lights")
        self.declare_parameter("debug_image_topic", "/perception/debug_image")
        self.declare_parameter("debug_markers_topic", "/perception/debug_markers")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("source_name", "xycar_camera_perception")
        self.declare_parameter("publish_rate_limit_hz", 15.0)
        self.declare_parameter("publish_empty_optional_topics", True)
        self.declare_parameter("roi_top_row", 230)
        self.declare_parameter("roi_bottom_row", 470)
        self.declare_parameter("row_step_px", 6)
        self.declare_parameter("image_center_x_px", -1.0)
        self.declare_parameter("projection_mode", "vanishing_point")
        self.declare_parameter("horizon_row_px", 236.0)
        self.declare_parameter("vanishing_point_x_px", -1.0)
        self.declare_parameter("ipm_x_scale_m_px", 67.0)
        self.declare_parameter("ipm_y_scale_m_px", 0.18)
        self.declare_parameter("horizon_min_denom_px", 8.0)
        self.declare_parameter("min_projected_x_m", 0.20)
        self.declare_parameter("max_projected_x_m", 5.00)
        self.declare_parameter("near_x_m", 0.20)
        self.declare_parameter("far_x_m", 2.20)
        self.declare_parameter("near_m_per_px", 0.0022)
        self.declare_parameter("far_m_per_px", 0.0065)
        self.declare_parameter("lane_width_m", 0.80)
        self.declare_parameter("min_cluster_width_px", 3)
        self.declare_parameter("max_cluster_gap_px", 8)
        self.declare_parameter("min_segment_points", 4)
        self.declare_parameter("min_centerline_points", 3)
        self.declare_parameter("white_s_max", 80)
        self.declare_parameter("white_v_min", 145)
        self.declare_parameter("yellow_h_min", 15)
        self.declare_parameter("yellow_h_max", 42)
        self.declare_parameter("yellow_s_min", 60)
        self.declare_parameter("yellow_v_min", 110)
        self.declare_parameter("morphology_kernel_px", 3)
        self.declare_parameter("centerline_mode", "lane_midline")
        self.declare_parameter("use_yellow_as_centerline", True)

        self.bridge = CvBridge()
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.source_name = str(self.get_parameter("source_name").value)
        self.publish_empty_optional_topics = bool(
            self.get_parameter("publish_empty_optional_topics").value
        )
        self.roi_top_row = int(self.get_parameter("roi_top_row").value)
        self.roi_bottom_row = int(self.get_parameter("roi_bottom_row").value)
        self.row_step_px = max(1, int(self.get_parameter("row_step_px").value))
        self.image_center_x_px = float(self.get_parameter("image_center_x_px").value)
        self.projection_mode = str(self.get_parameter("projection_mode").value)
        self.horizon_row_px = float(self.get_parameter("horizon_row_px").value)
        self.vanishing_point_x_px = float(self.get_parameter("vanishing_point_x_px").value)
        self.ipm_x_scale_m_px = float(self.get_parameter("ipm_x_scale_m_px").value)
        self.ipm_y_scale_m_px = float(self.get_parameter("ipm_y_scale_m_px").value)
        self.horizon_min_denom_px = float(self.get_parameter("horizon_min_denom_px").value)
        self.min_projected_x_m = float(self.get_parameter("min_projected_x_m").value)
        self.max_projected_x_m = float(self.get_parameter("max_projected_x_m").value)
        self.near_x_m = float(self.get_parameter("near_x_m").value)
        self.far_x_m = float(self.get_parameter("far_x_m").value)
        self.near_m_per_px = float(self.get_parameter("near_m_per_px").value)
        self.far_m_per_px = float(self.get_parameter("far_m_per_px").value)
        self.lane_width_m = float(self.get_parameter("lane_width_m").value)
        self.min_cluster_width_px = int(self.get_parameter("min_cluster_width_px").value)
        self.max_cluster_gap_px = int(self.get_parameter("max_cluster_gap_px").value)
        self.min_segment_points = int(self.get_parameter("min_segment_points").value)
        self.min_centerline_points = int(self.get_parameter("min_centerline_points").value)
        self.white_s_max = int(self.get_parameter("white_s_max").value)
        self.white_v_min = int(self.get_parameter("white_v_min").value)
        self.yellow_h_min = int(self.get_parameter("yellow_h_min").value)
        self.yellow_h_max = int(self.get_parameter("yellow_h_max").value)
        self.yellow_s_min = int(self.get_parameter("yellow_s_min").value)
        self.yellow_v_min = int(self.get_parameter("yellow_v_min").value)
        self.morphology_kernel_px = int(self.get_parameter("morphology_kernel_px").value)
        self.centerline_mode = str(self.get_parameter("centerline_mode").value)
        self.use_yellow_as_centerline = bool(
            self.get_parameter("use_yellow_as_centerline").value
        )

        rate_limit_hz = float(self.get_parameter("publish_rate_limit_hz").value)
        self.min_publish_period = 0.0 if rate_limit_hz <= 0.0 else 1.0 / rate_limit_hz
        self.last_publish_wall_time = 0.0
        self.detection_id = 1

        self.road_segments_pub = self.create_publisher(
            RoadSegmentArray,
            str(self.get_parameter("road_segments_topic").value),
            10,
        )
        self.centerline_pub = self.create_publisher(
            Centerline,
            str(self.get_parameter("centerline_topic").value),
            10,
        )
        self.objects_pub = self.create_publisher(
            PerceptionObjectArray,
            str(self.get_parameter("objects_topic").value),
            10,
        )
        self.traffic_lights_pub = self.create_publisher(
            TrafficLightObservationArray,
            str(self.get_parameter("traffic_lights_topic").value),
            10,
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("debug_image_topic").value),
            10,
        )
        self.debug_markers_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("debug_markers_topic").value),
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            10,
        )
        self.get_logger().info(
            "camera perception ready: /image_raw -> /perception/road_segments, "
            "/perception/centerline"
        )

    def next_detection_id(self) -> int:
        value = self.detection_id
        self.detection_id += 1
        return value

    def on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_publish_wall_time < self.min_publish_period:
            return
        self.last_publish_wall_time = now

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"failed to convert image: {exc}", throttle_duration_sec=2.0)
            return

        header = RoadSegmentArray().header
        header.stamp = msg.header.stamp if msg.header.stamp.sec or msg.header.stamp.nanosec else self.get_clock().now().to_msg()
        header.frame_id = self.base_frame_id

        road_segments, centerline, debug = self.detect_lanes(image, header)
        self.road_segments_pub.publish(road_segments)
        self.centerline_pub.publish(centerline)

        if self.publish_empty_optional_topics:
            objects = PerceptionObjectArray()
            objects.header = header
            self.objects_pub.publish(objects)

            traffic_lights = TrafficLightObservationArray()
            traffic_lights.header = header
            self.traffic_lights_pub.publish(traffic_lights)

        if self.debug_image_pub.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)

        if self.debug_markers_pub.get_subscription_count() > 0:
            self.debug_markers_pub.publish(self.build_markers(header, road_segments, centerline))

    def detect_lanes(self, image: np.ndarray, header) -> tuple[RoadSegmentArray, Centerline, np.ndarray]:
        height, width = image.shape[:2]
        center_x = self.image_center_x_px if self.image_center_x_px >= 0.0 else width * 0.5
        roi_top = max(0, min(height - 1, self.roi_top_row))
        roi_bottom = max(roi_top + 1, min(height - 1, self.roi_bottom_row))

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, self.white_v_min], dtype=np.uint8),
            np.array([180, self.white_s_max, 255], dtype=np.uint8),
        )
        yellow_mask = cv2.inRange(
            hsv,
            np.array([self.yellow_h_min, self.yellow_s_min, self.yellow_v_min], dtype=np.uint8),
            np.array([self.yellow_h_max, 255, 255], dtype=np.uint8),
        )
        white_mask[:roi_top, :] = 0
        white_mask[roi_bottom + 1 :, :] = 0
        yellow_mask[:roi_top, :] = 0
        yellow_mask[roi_bottom + 1 :, :] = 0

        if self.morphology_kernel_px > 1:
            kernel = np.ones((self.morphology_kernel_px, self.morphology_kernel_px), np.uint8)
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
            yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
            yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        left_points: list[Point] = []
        right_points: list[Point] = []
        yellow_points: list[Point] = []
        debug = image.copy()
        cv2.rectangle(debug, (0, roi_top), (width - 1, roi_bottom), (80, 80, 80), 1)

        for row in range(roi_bottom, roi_top - 1, -self.row_step_px):
            white_clusters = self.row_clusters(white_mask[row])
            yellow_clusters = self.row_clusters(yellow_mask[row])

            left_cluster = self.closest_left_cluster(white_clusters, center_x)
            right_cluster = self.closest_right_cluster(white_clusters, center_x)
            yellow_cluster = self.closest_center_cluster(yellow_clusters, center_x)

            if left_cluster is not None:
                point = self.pixel_to_vehicle(left_cluster, row, center_x, roi_top, roi_bottom)
                if point is not None:
                    left_points.append(point)
                    cv2.circle(debug, (int(left_cluster), row), 3, (255, 255, 255), -1)
            if right_cluster is not None:
                point = self.pixel_to_vehicle(right_cluster, row, center_x, roi_top, roi_bottom)
                if point is not None:
                    right_points.append(point)
                    cv2.circle(debug, (int(right_cluster), row), 3, (255, 255, 255), -1)
            if yellow_cluster is not None:
                point = self.pixel_to_vehicle(yellow_cluster, row, center_x, roi_top, roi_bottom)
                if point is not None:
                    yellow_points.append(point)
                    cv2.circle(debug, (int(yellow_cluster), row), 3, (0, 220, 255), -1)

        left_points = self.smooth_points(left_points)
        right_points = self.smooth_points(right_points)
        yellow_points = self.smooth_points(yellow_points)

        road_segments = RoadSegmentArray()
        road_segments.header = header
        left_segment = self.make_segment(RoadSegment.TYPE_WHSOL, left_points)
        right_segment = self.make_segment(RoadSegment.TYPE_WHSOL, right_points)
        yellow_segment = self.make_segment(RoadSegment.TYPE_YEDOT, yellow_points)
        for segment in (left_segment, right_segment, yellow_segment):
            if segment is not None:
                road_segments.segments.append(segment)

        centerline = self.build_centerline(header, left_segment, right_segment, yellow_segment)
        self.draw_centerline_on_debug(debug, centerline, center_x, roi_top, roi_bottom)
        return road_segments, centerline, debug

    def row_clusters(self, row_mask: np.ndarray) -> list[float]:
        xs = np.flatnonzero(row_mask)
        if xs.size == 0:
            return []
        clusters: list[np.ndarray] = []
        start = 0
        for index in range(1, xs.size):
            if xs[index] - xs[index - 1] > self.max_cluster_gap_px:
                clusters.append(xs[start:index])
                start = index
        clusters.append(xs[start:])

        centers = []
        for cluster in clusters:
            if cluster.size < self.min_cluster_width_px:
                continue
            if int(cluster[-1] - cluster[0] + 1) < self.min_cluster_width_px:
                continue
            centers.append(float(np.mean(cluster)))
        return centers

    def closest_left_cluster(self, clusters: list[float], center_x: float) -> float | None:
        left = [cluster for cluster in clusters if cluster < center_x - 4.0]
        return max(left) if left else None

    def closest_right_cluster(self, clusters: list[float], center_x: float) -> float | None:
        right = [cluster for cluster in clusters if cluster > center_x + 4.0]
        return min(right) if right else None

    def closest_center_cluster(self, clusters: list[float], center_x: float) -> float | None:
        if not clusters:
            return None
        return min(clusters, key=lambda value: abs(value - center_x))

    def pixel_to_vehicle(
        self,
        pixel_x: float,
        pixel_y: int,
        center_x: float,
        roi_top: int,
        roi_bottom: int,
    ) -> Point | None:
        if self.projection_mode == "vanishing_point":
            return self.pixel_to_vehicle_vanishing_point(pixel_x, pixel_y, center_x)

        span = max(1, roi_bottom - roi_top)
        t = (roi_bottom - pixel_y) / float(span)
        t = max(0.0, min(1.0, t))
        x_m = self.near_x_m + t * (self.far_x_m - self.near_x_m)
        meters_per_pixel = self.near_m_per_px + t * (self.far_m_per_px - self.near_m_per_px)
        y_m = (center_x - pixel_x) * meters_per_pixel
        return make_point(x_m, y_m, 0.0)

    def pixel_to_vehicle_vanishing_point(
        self,
        pixel_x: float,
        pixel_y: int,
        center_x: float,
    ) -> Point | None:
        vanishing_x = self.vanishing_point_x_px
        if vanishing_x < 0.0:
            vanishing_x = center_x

        denom = float(pixel_y) - self.horizon_row_px
        if denom < self.horizon_min_denom_px:
            return None
        if self.ipm_x_scale_m_px <= 0.0 or self.ipm_y_scale_m_px <= 0.0:
            return None

        x_m = self.ipm_x_scale_m_px / denom
        if x_m < self.min_projected_x_m or x_m > self.max_projected_x_m:
            return None

        y_m = (vanishing_x - float(pixel_x)) * self.ipm_y_scale_m_px / denom
        return make_point(x_m, y_m, 0.0)

    def smooth_points(self, points: list[Point]) -> list[Point]:
        if len(points) < 3:
            return sorted(points, key=lambda point: point.x)
        points = sorted(points, key=lambda point: point.x)
        ys = moving_average([point.y for point in points], window=3)
        return [make_point(point.x, y, 0.0) for point, y in zip(points, ys)]

    def make_segment(self, segment_type: int, points: list[Point]) -> RoadSegment | None:
        if len(points) < self.min_segment_points:
            return None
        segment = RoadSegment()
        segment.detection_id = self.next_detection_id()
        segment.track_id = 0
        segment.type = segment_type
        segment.points = points
        expected_count = max(1, int((self.roi_bottom_row - self.roi_top_row) / self.row_step_px))
        segment.confidence = float(min(1.0, 0.30 + 0.70 * len(points) / expected_count))
        segment.source = self.source_name
        return segment

    def build_centerline(
        self,
        header,
        left: RoadSegment | None,
        right: RoadSegment | None,
        yellow: RoadSegment | None,
    ) -> Centerline:
        centerline = Centerline()
        centerline.header = header
        centerline.track_id = 0
        centerline.source = self.source_name

        if self.centerline_mode == "lane_midline" and yellow is not None:
            centerline.points = self.best_yellow_boundary_midline(yellow, [left, right])
            if len(centerline.points) >= self.min_centerline_points:
                confidence_candidates = [
                    segment.confidence for segment in (left, right, yellow) if segment is not None
                ]
                centerline.confidence = float(min(confidence_candidates)) if confidence_candidates else 0.0
        elif self.use_yellow_as_centerline and yellow is not None and len(yellow.points) >= self.min_centerline_points:
            centerline.points = list(yellow.points)
            centerline.confidence = float(yellow.confidence)

        if not centerline.points and left is not None and right is not None:
            centerline.points = self.midpoint_centerline_points(left.points, right.points)
            centerline.confidence = float(min(left.confidence, right.confidence))
        elif not centerline.points and left is not None:
            centerline.points = [make_point(point.x, point.y - self.lane_width_m * 0.5, 0.0) for point in left.points]
            centerline.confidence = float(left.confidence * 0.70)
        elif not centerline.points and right is not None:
            centerline.points = [make_point(point.x, point.y + self.lane_width_m * 0.5, 0.0) for point in right.points]
            centerline.confidence = float(right.confidence * 0.70)
        elif not centerline.points:
            centerline.confidence = 0.0

        centerline.points = self.smooth_points(centerline.points)
        if len(centerline.points) < self.min_centerline_points:
            centerline.points = []
            centerline.confidence = 0.0
            return centerline

        centerline.detection_id = self.next_detection_id()
        return centerline

    def best_yellow_boundary_midline(
        self,
        yellow: RoadSegment,
        boundaries: list[RoadSegment | None],
    ) -> list[Point]:
        best_points: list[Point] = []
        best_score = -1.0
        for boundary in boundaries:
            if boundary is None:
                continue
            candidate = self.midpoint_centerline_points(yellow.points, boundary.points)
            if len(candidate) < self.min_centerline_points:
                continue
            score = self.midline_score(yellow.points, boundary.points, candidate)
            if score > best_score:
                best_points = candidate
                best_score = score
        return best_points

    def midline_score(
        self,
        yellow_points: list[Point],
        boundary_points: list[Point],
        midline_points: list[Point],
    ) -> float:
        width_errors = []
        for point in midline_points:
            yellow_y = self.y_at_x(yellow_points, point.x)
            boundary_y = self.y_at_x(boundary_points, point.x)
            if yellow_y is None or boundary_y is None:
                continue
            width_errors.append(abs(abs(yellow_y - boundary_y) - self.lane_width_m))
        mean_error = sum(width_errors) / max(1, len(width_errors))
        return float(len(midline_points) - 3.0 * mean_error)

    def midpoint_centerline_points(self, left: list[Point], right: list[Point]) -> list[Point]:
        left_bounds = self.x_bounds(left)
        right_bounds = self.x_bounds(right)
        if left_bounds is None or right_bounds is None:
            return []
        start_x = max(left_bounds[0], right_bounds[0])
        end_x = min(left_bounds[1], right_bounds[1])
        if end_x <= start_x:
            return []
        sample_count = max(3, min(24, int(math.ceil((end_x - start_x) / 0.15)) + 1))
        points = []
        for index in range(sample_count):
            ratio = index / float(sample_count - 1)
            target_x = start_x + (end_x - start_x) * ratio
            left_y = self.y_at_x(left, target_x)
            right_y = self.y_at_x(right, target_x)
            if left_y is None or right_y is None:
                continue
            points.append(make_point(target_x, (left_y + right_y) * 0.5, 0.0))
        return points

    def x_bounds(self, points: list[Point]) -> tuple[float, float] | None:
        if len(points) < 2:
            return None
        return min(point.x for point in points), max(point.x for point in points)

    def y_at_x(self, points: list[Point], target_x: float) -> float | None:
        if not points:
            return None
        points = sorted(points, key=lambda point: point.x)
        if target_x <= points[0].x:
            return points[0].y
        if target_x >= points[-1].x:
            return points[-1].y
        for start, end in zip(points, points[1:]):
            if start.x <= target_x <= end.x:
                dx = end.x - start.x
                if abs(dx) < 1.0e-6:
                    return start.y
                ratio = (target_x - start.x) / dx
                return start.y + (end.y - start.y) * ratio
        return None

    def draw_centerline_on_debug(
        self,
        image: np.ndarray,
        centerline: Centerline,
        center_x: float,
        roi_top: int,
        roi_bottom: int,
    ) -> None:
        for point in centerline.points:
            pixel = self.vehicle_to_pixel(point, center_x, roi_top, roi_bottom)
            if pixel is not None:
                cv2.circle(image, pixel, 3, (0, 0, 255), -1)
        cv2.putText(
            image,
            f"centerline pts={len(centerline.points)} conf={centerline.confidence:.2f}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )

    def vehicle_to_pixel(
        self,
        point: Point,
        center_x: float,
        roi_top: int,
        roi_bottom: int,
    ) -> tuple[int, int] | None:
        if self.projection_mode == "vanishing_point":
            return self.vehicle_to_pixel_vanishing_point(point, center_x)

        if self.far_x_m <= self.near_x_m:
            return None
        t = (point.x - self.near_x_m) / (self.far_x_m - self.near_x_m)
        if t < 0.0 or t > 1.0:
            return None
        meters_per_pixel = self.near_m_per_px + t * (self.far_m_per_px - self.near_m_per_px)
        if meters_per_pixel <= 1.0e-6:
            return None
        pixel_y = int(round(roi_bottom - t * (roi_bottom - roi_top)))
        pixel_x = int(round(center_x - point.y / meters_per_pixel))
        return pixel_x, pixel_y

    def vehicle_to_pixel_vanishing_point(
        self,
        point: Point,
        center_x: float,
    ) -> tuple[int, int] | None:
        if point.x <= 1.0e-6 or self.ipm_x_scale_m_px <= 0.0 or self.ipm_y_scale_m_px <= 0.0:
            return None
        vanishing_x = self.vanishing_point_x_px
        if vanishing_x < 0.0:
            vanishing_x = center_x
        denom = self.ipm_x_scale_m_px / float(point.x)
        if denom < self.horizon_min_denom_px:
            return None
        pixel_y = int(round(self.horizon_row_px + denom))
        pixel_x = int(round(vanishing_x - float(point.y) * denom / self.ipm_y_scale_m_px))
        return pixel_x, pixel_y

    def build_markers(
        self,
        header,
        road_segments: RoadSegmentArray,
        centerline: Centerline,
    ) -> MarkerArray:
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.header = header
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        marker_id = 1
        for segment in road_segments.segments:
            marker = self.line_marker(header, marker_id, "road_segments", segment.points)
            if segment.type in {RoadSegment.TYPE_YESOL, RoadSegment.TYPE_YEDOT}:
                marker.color.r = 1.0
                marker.color.g = 0.75
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            markers.markers.append(marker)
            marker_id += 1

        marker = self.line_marker(header, marker_id, "centerline", centerline.points)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.scale.x = 0.035
        markers.markers.append(marker)
        return markers

    def line_marker(self, header, marker_id: int, namespace: str, points: list[Point]) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.color.a = 1.0
        marker.points = points
        return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraPerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
