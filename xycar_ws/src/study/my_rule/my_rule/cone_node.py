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
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray

from my_rule.perception.camera_input import CameraRectifier
from my_rule.perception.lidar_camera_association import (
    ImageBox,
    associate_lidar_clusters_with_boxes,
    load_lidar_camera_extrinsic,
)
from my_rule.perception.object_perception import normalize_class_name

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
        self.declare_parameter("seed_min_angle_deg", 20.0)
        self.declare_parameter("seed_max_angle_deg", 100.0)
        self.declare_parameter("seed_max_range_m", 3.0)
        self.declare_parameter("allow_single_boundary_fallback", True)
        self.declare_parameter("single_boundary_min_cones", 2)
        self.declare_parameter("single_boundary_min_span_m", 0.20)
        self.declare_parameter("single_boundary_switch_frames", 3)
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
        # Keep the vehicle-tested 9.0~17 profile for bends. A higher target is
        # allowed only when a fresh bilateral path is long, confident and
        # straight throughout the preview window.
        self.declare_parameter("cone_preview_speed_control", True)
        self.declare_parameter("cone_straight_boost_speed", 21.0)
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
        self.declare_parameter("pair_max_forward_delta_m", 0.30)
        # LiDAR cluster centres can measure a few centimetres inside the physical
        # cone spacing, so keep a small tolerance below the 0.78 m course minimum.
        self.declare_parameter("min_corridor_width_m", 0.68)
        self.declare_parameter("max_corridor_width_m", 0.98)
        self.declare_parameter("expected_corridor_width_m", 0.85)
        self.declare_parameter("min_path_midpoints", 2)
        self.declare_parameter("min_path_span_m", 0.15)
        # A geometrically validated bilateral path may legitimately move this
        # far when it replaces a less reliable single-boundary estimate.
        self.declare_parameter("max_path_target_jump_m", 0.45)
        self.declare_parameter("inferred_max_path_target_jump_m", 0.25)
        self.declare_parameter("steering_median_window", 1)
        self.declare_parameter("steering_max_delta_deg", 14.0)
        self.declare_parameter("inferred_steering_max_delta_deg", 10.0)
        self.declare_parameter("allow_nearest_gate_fallback", True)
        self.declare_parameter("fallback_pair_min_lateral_separation_m", 0.4)
        self.declare_parameter("fallback_pair_max_center_offset_m", 0.65)
        self.declare_parameter("blind_recovery_frames", 10)
        self.declare_parameter("blind_recovery_min_clusters", 2)
        self.declare_parameter("blind_recovery_steer_decay", 0.92)

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
        self.path_miss_count = 0
        self.path_is_held = False
        history_frames = max(1, int(self.get_parameter("sparse_cluster_history_frames").value))
        self.cluster_candidate_history = deque(maxlen=history_frames)
        self.midpoints_inferred = False
        self.midpoint_source = "none"
        self.active_inferred_boundary: Optional[str] = None
        self.pending_inferred_boundary: Optional[str] = None
        self.pending_inferred_frames = 0
        self.steering_history = deque(maxlen=max(1, int(self.get_parameter("steering_median_window").value)))
        self.stabilized_steering: Optional[float] = None
        self.last_valid_steering = 0.0
        self.blind_recovery_count = 0
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
            f"{'enabled' if self.cone_yolo_association_enabled else 'disabled'}"
        )

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
        if not boxes:
            return
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
        age_sec = time.monotonic() - self.cone_yolo_boxes_time
        if scan_stamp_ns > 0 and self.cone_yolo_boxes_stamp_ns > 0:
            stamp_delta_sec = (
                scan_stamp_ns - self.cone_yolo_boxes_stamp_ns
            ) / 1.0e9
            # Prefer sensor timestamps when both devices share the ROS clock.
            # Fall back to receipt time for drivers with unrelated clocks.
            if abs(stamp_delta_sec) < 60.0:
                age_sec = max(0.0, stamp_delta_sec)
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
            or not self.prev_path
        ):
            return confirmed

        max_forward_delta = float(
            self.get_parameter("pair_max_forward_delta_m").value
        )
        min_width = float(self.get_parameter("min_corridor_width_m").value)
        max_width = float(self.get_parameter("max_corridor_width_m").value)
        expected_width = float(
            self.get_parameter("expected_corridor_width_m").value
        )
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
                path_deviation = self.point_to_path_distance(
                    midpoint_rear,
                    self.prev_path,
                )
                if path_deviation > max_path_deviation:
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
        self.path_miss_count = 0
        self.path_is_held = False
        self.cluster_candidate_history.clear()
        self.midpoints_inferred = False
        self.midpoint_source = "none"
        self.active_inferred_boundary = None
        self.pending_inferred_boundary = None
        self.pending_inferred_frames = 0
        self.steering_history.clear()
        self.stabilized_steering = None
        self.last_valid_steering = 0.0
        self.blind_recovery_count = 0
        self.had_valid_path = False

    def scan_callback(self, msg: LaserScan) -> None:
        if not self.processing_enabled:
            return
        points = self.scan_to_points(msg)
        clusters = self.cluster_cones(points)
        clusters = self.filter_front_clusters_by_angle(clusters)
        self.publish_cluster_array(self.lidar_cluster_pub, clusters)
        fused_clusters = self.associate_clusters_with_yolo(
            clusters,
            scan_stamp_ns=self.message_stamp_ns(msg),
        )
        fused_clusters = self.recover_corridor_partners(
            clusters,
            fused_clusters,
        )
        self.publish_cluster_array(self.fused_cluster_pub, fused_clusters)
        # Retain the historical topic as a compatibility alias, but its content
        # is now the fused result rather than the raw LiDAR candidates.
        self.publish_clusters(fused_clusters)

        left_cones, right_cones = self.form_cone_groups(fused_clusters)
        midpoints = self.calculate_midpoints(left_cones, right_cones)
        boundary_switch_pending = (
            self.pending_inferred_boundary is not None
            and self.pending_inferred_frames > 0
        )
        if not midpoints and not boundary_switch_pending:
            fallback_midpoint = self.nearest_gate_midpoint(fused_clusters)
            if fallback_midpoint is not None:
                self.midpoints_inferred = True
                self.midpoint_source = "nearest_gate"
                midpoints = [fallback_midpoint]
        path = self.interpolate_path(midpoints)
        self.publish_path(path)

        if not path:
            if self.publish_blind_recovery(len(fused_clusters)):
                return
            self.steering_history.clear()
            self.stabilized_steering = None
            self.publish_cmd(0.0, 0.0, 0.0)
            return

        raw_angle = self.pure_pursuit(path)
        angle = self.stabilize_steering(raw_angle)
        confidence = self.path_confidence(len(midpoints))
        speed = self.compute_speed(angle, confidence, path)
        self.last_valid_steering = angle
        self.had_valid_path = True
        if not self.path_is_held:
            self.blind_recovery_count = 0
        self.publish_cmd(angle, speed, confidence)

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

    def form_cone_groups(self, centers: Sequence[Point2]) -> Tuple[List[Point2], List[Point2]]:
        min_angle = float(self.get_parameter("seed_min_angle_deg").value)
        max_angle = float(self.get_parameter("seed_max_angle_deg").value)
        max_range = float(self.get_parameter("seed_max_range_m").value)
        left_seed = self.find_seed(centers, min_angle, max_angle, max_range)
        right_seed = self.find_seed(centers, 360.0 - max_angle, 360.0 - min_angle, max_range)
        if left_seed is not None and right_seed is not None:
            return self.grow_groups_competitively(left_seed, right_seed, centers)
        used = set()
        left = self.grow_group(left_seed, centers, used) if left_seed is not None else []
        right = self.grow_group(right_seed, centers, used) if right_seed is not None else []
        return left, right

    def grow_groups_competitively(
        self,
        left_seed: Point2,
        right_seed: Point2,
        centers: Sequence[Point2],
    ) -> Tuple[List[Point2], List[Point2]]:
        """Grow both boundaries together so the first side cannot consume the other."""
        threshold = float(self.get_parameter("group_grow_distance_m").value)
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

        left = sorted(groups[0], key=lambda p: p[0])
        right = sorted(groups[1], key=lambda p: p[0])
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
        threshold = float(self.get_parameter("group_grow_distance_m").value)
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
        return group

    def calculate_midpoints(self, left: Sequence[Point2], right: Sequence[Point2]) -> List[Point2]:
        self.midpoints_inferred = False
        self.midpoint_source = "none"
        if not left and not right:
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return []
        if not left or not right:
            return self.infer_midpoints_from_single_boundary(left, right)
        max_forward_delta = float(self.get_parameter("pair_max_forward_delta_m").value)
        min_width = float(self.get_parameter("min_corridor_width_m").value)
        max_width = float(self.get_parameter("max_corridor_width_m").value)
        expected_width = float(self.get_parameter("expected_corridor_width_m").value)

        candidates = []
        for left_index, left_point in enumerate(left):
            for right_index, right_point in enumerate(right):
                forward_delta = abs(left_point[0] - right_point[0])
                width = math.hypot(left_point[0] - right_point[0], left_point[1] - right_point[1])
                if forward_delta > max_forward_delta or not min_width <= width <= max_width:
                    continue
                cost = 2.0 * forward_delta + abs(width - expected_width)
                candidates.append((cost, left_index, right_index))

        used_left = set()
        used_right = set()
        midpoints: List[Point2] = []
        for _cost, left_index, right_index in sorted(candidates):
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
        midpoints = sorted(midpoints, key=lambda p: p[0])
        minimum_bilateral = max(2, int(self.get_parameter("min_path_midpoints").value))
        if len(midpoints) >= minimum_bilateral:
            # Two measured boundaries constrain the corridor directly. A denser
            # single boundary must never replace this path just because it has
            # more points; that caused left/right source flapping in S turns.
            self.midpoint_source = "paired"
            self.active_inferred_boundary = None
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return midpoints
        inferred = self.infer_midpoints_from_richer_boundary(left, right)
        if inferred:
            return inferred
        if midpoints:
            self.midpoint_source = "paired_sparse"
            return midpoints
        return self.infer_midpoints_from_single_boundary(left, right)

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
        threshold = float(self.get_parameter("group_grow_distance_m").value)
        ordered = sorted(boundary, key=lambda p: p[0])
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
        span = max(point[0] for point in boundary) - min(point[0] for point in boundary)
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
        expected_width = float(self.get_parameter("expected_corridor_width_m").value)
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
        points = sorted(boundary, key=lambda p: p[0])
        expected_width = float(self.get_parameter("expected_corridor_width_m").value)
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
                return float(
                    np.mean(
                        [
                            self.point_to_path_distance(
                                (x + rear_offset, y),
                                self.prev_path,
                            )
                            for x, y in candidate_path
                        ]
                    )
                )

            if path_score(opposite_centerline) < path_score(default_centerline):
                default_centerline = opposite_centerline
        return sorted(default_centerline, key=lambda p: p[0])

    def interpolate_path(self, midpoints: Sequence[Point2]) -> List[Point2]:
        min_midpoints = max(1, int(self.get_parameter("min_path_midpoints").value))
        if len(midpoints) < min_midpoints:
            return self.hold_previous_path()
        rear_offset = float(self.get_parameter("lidar_to_rear_axle_m").value)
        pts = sorted([(x + rear_offset, y) for x, y in midpoints], key=lambda p: p[0])
        if len(pts) == 1:
            return self.accept_new_path(pts)
        xs = np.array([p[0] for p in pts], dtype=np.float32)
        ys = np.array([p[1] for p in pts], dtype=np.float32)
        unique_xs, unique_idx = np.unique(xs, return_index=True)
        unique_ys = ys[unique_idx]
        if len(unique_xs) < min_midpoints:
            return self.hold_previous_path()
        if float(unique_xs.max() - unique_xs.min()) < float(self.get_parameter("min_path_span_m").value):
            return self.hold_previous_path()
        sample_count = max(2, int(self.get_parameter("path_sample_count").value))
        interp_x = np.linspace(float(unique_xs.min()), float(unique_xs.max()), sample_count)
        method = str(self.get_parameter("path_interpolation_method").value).lower().strip()
        if method in ("cubic", "cubic_spline", "spline"):
            interp_y = self.cubic_interpolate(unique_xs, unique_ys, interp_x)
        else:
            interp_y = np.interp(interp_x, unique_xs, unique_ys)
        new_path = [(float(x), float(y)) for x, y in zip(interp_x, interp_y)]
        return self.accept_new_path(new_path)

    def accept_new_path(self, new_path: List[Point2]) -> List[Point2]:
        if self.prev_path is not None and len(self.prev_path) > 1 and len(new_path) == 1:
            return self.hold_previous_path()
        if self.prev_path is not None:
            previous_target = self.path_target_lateral(self.prev_path)
            new_target = self.path_target_lateral(new_path)
            source = str(getattr(self, "midpoint_source", "none"))
            inferred = source in ("left_offset", "right_offset", "nearest_gate", "paired_sparse")
            parameter = "inferred_max_path_target_jump_m" if inferred else "max_path_target_jump_m"
            max_jump = float(self.get_parameter(parameter).value)
            if abs(new_target - previous_target) > max_jump:
                return self.hold_previous_path()
        self.path_miss_count = 0
        self.path_is_held = False
        self.prev_path = new_path
        return self.prev_path

    def hold_previous_path(self) -> List[Point2]:
        self.path_miss_count += 1
        hold_frames = max(0, int(self.get_parameter("path_hold_frames").value))
        if self.prev_path is not None and self.path_miss_count <= hold_frames:
            self.path_is_held = True
            return self.prev_path
        self.path_is_held = False
        self.prev_path = None
        return []

    def path_target_lateral(self, path: Sequence[Point2]) -> float:
        if not path:
            return 0.0
        lookahead = float(self.get_parameter("lookahead_min_m").value)
        for x, y in path:
            if x >= lookahead:
                return float(y)
        return float(path[-1][1])

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
        angle_cmd = gain * ((1.0 - far_weight) * near_angle + far_weight * far_angle)
        limit = float(self.get_parameter("max_steer_cmd").value)
        return float(np.clip(angle_cmd, -limit, limit))

    def pure_pursuit_at_distance(self, path: Sequence[Point2], lookahead: float) -> float:
        dists = np.array([math.hypot(x, y) for x, y in path], dtype=np.float32)
        candidates = np.where(dists > lookahead)[0]
        idx = int(candidates[0]) if candidates.size > 0 else len(path) - 1
        tx, ty = path[idx]
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

    def stabilize_steering(self, angle: float) -> float:
        self.steering_history.append(float(angle))
        if len(self.steering_history) < self.steering_history.maxlen:
            filtered = float(angle)
        else:
            filtered = float(np.median(np.asarray(self.steering_history, dtype=np.float32)))

        source = str(getattr(self, "midpoint_source", "none"))
        inferred = source in ("left_offset", "right_offset", "nearest_gate", "paired_sparse")
        parameter = "inferred_steering_max_delta_deg" if inferred else "steering_max_delta_deg"
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
        return filtered

    def path_confidence(self, midpoint_count: int) -> float:
        if self.path_is_held:
            hold_frames = max(1, int(self.get_parameter("path_hold_frames").value))
            remaining = max(0.0, 1.0 - float(self.path_miss_count - 1) / float(hold_frames))
            return max(0.21, float(self.get_parameter("min_confidence").value) * remaining)
        source = str(getattr(self, "midpoint_source", "none"))
        if source == "paired":
            return min(1.0, 0.5 + 0.2 * max(0, midpoint_count - 1))
        if source in ("left_offset", "right_offset"):
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
            hold_frames = max(1, int(self.get_parameter("path_hold_frames").value))
            hold_ratio = min(1.0, float(self.path_miss_count) / float(hold_frames))
            speed = minimum + (speed - minimum) * (1.0 - 0.55 * hold_ratio)
        source = str(getattr(self, "midpoint_source", "none"))
        if source in ("left_offset", "right_offset"):
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
        max_frames = max(0, int(self.get_parameter("blind_recovery_frames").value))
        min_clusters = max(1, int(self.get_parameter("blind_recovery_min_clusters").value))
        if not self.had_valid_path or cluster_count < min_clusters or self.blind_recovery_count >= max_frames:
            return False

        self.blind_recovery_count += 1
        decay = float(np.clip(self.get_parameter("blind_recovery_steer_decay").value, 0.0, 1.0))
        angle = self.last_valid_steering * (decay ** self.blind_recovery_count)
        speed = float(self.get_parameter("cone_min_drive_speed").value)
        confidence = max(0.21, float(self.get_parameter("min_confidence").value) * 0.75)
        self.publish_cmd(angle, speed, confidence)
        return True

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
