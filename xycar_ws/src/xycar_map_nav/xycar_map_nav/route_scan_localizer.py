#!/usr/bin/env python3
"""Initialize slam_toolbox by matching LiDAR along a known route."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .route_scan_matching import (
    MatchConfig,
    RouteMatch,
    load_likelihood_field,
    match_scan_to_route,
    normalize_angle,
    route_distance,
    scan_points_in_base,
)


class RouteScanLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("route_scan_localizer")
        self._declare_parameters()
        self.field = load_likelihood_field(
            str(self.get_parameter("map_yaml").value)
        )
        self.closed_route = bool(
            self.get_parameter("closed_route").value
        )
        self.config = MatchConfig(
            route_sample_spacing_m=float(
                self.get_parameter("route_sample_spacing_m").value
            ),
            lateral_search_m=float(
                self.get_parameter("lateral_search_m").value
            ),
            lateral_step_m=float(
                self.get_parameter("lateral_step_m").value
            ),
            yaw_search_rad=math.radians(
                float(self.get_parameter("yaw_search_deg").value)
            ),
            yaw_step_rad=math.radians(
                float(self.get_parameter("yaw_step_deg").value)
            ),
            refinement_xy_m=float(
                self.get_parameter("refinement_xy_m").value
            ),
            refinement_xy_step_m=float(
                self.get_parameter("refinement_xy_step_m").value
            ),
            refinement_yaw_rad=math.radians(
                float(self.get_parameter("refinement_yaw_deg").value)
            ),
            refinement_yaw_step_rad=math.radians(
                float(
                    self.get_parameter("refinement_yaw_step_deg").value
                )
            ),
            inlier_distance_m=float(
                self.get_parameter("inlier_distance_m").value
            ),
            maximum_distance_m=float(
                self.get_parameter("maximum_distance_m").value
            ),
            ambiguity_separation_m=float(
                self.get_parameter("ambiguity_separation_m").value
            ),
            minimum_inlier_ratio=float(
                self.get_parameter("minimum_inlier_ratio").value
            ),
            maximum_mean_distance_m=float(
                self.get_parameter("maximum_mean_distance_m").value
            ),
            minimum_score_margin=float(
                self.get_parameter("minimum_score_margin").value
            ),
            batch_size=int(self.get_parameter("batch_size").value),
            allow_reverse_heading=bool(
                self.get_parameter("allow_reverse_heading").value
            ),
            lateral_prior_weight=float(
                self.get_parameter("lateral_prior_weight").value
            ),
            yaw_prior_weight=float(
                self.get_parameter("yaw_prior_weight").value
            ),
        )
        self.route_points: list[tuple[float, float]] = []
        self.pending_match: RouteMatch | None = None
        self.consistent_scans = 0
        self.last_match_time = 0.0
        self.initial_pose_publish_time = 0.0
        self.initial_pose_published = False
        self.initial_pose_match: RouteMatch | None = None
        self.ready = False
        self.last_status = ""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initial_pose_topic").value),
            10,
        )
        self.best_pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("best_pose_topic").value),
            transient_qos,
        )
        self.ready_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("ready_topic").value),
            transient_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            transient_qos,
        )
        self.debug_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("debug_topic").value),
            transient_qos,
        )
        self.create_subscription(
            PathMessage,
            str(self.get_parameter("path_topic").value),
            self._on_path,
            transient_qos,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            sensor_qos,
        )
        self.create_service(Trigger, "~/relocalize", self._relocalize)
        self.create_timer(0.10, self._settle_step)
        self._publish_ready(False)
        self._publish_status("WAIT_PATH")
        self.get_logger().info(
            "route scan localizer ready; motor topics are never published"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("path_topic", "/map_nav/global_path")
        self.declare_parameter("scan_topic", "/slam/scan_filtered")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter(
            "ready_topic", "/map_nav/route_localization/ready"
        )
        self.declare_parameter(
            "status_topic", "/map_nav/route_localization/status"
        )
        self.declare_parameter(
            "debug_topic", "/map_nav/route_localization/debug"
        )
        self.declare_parameter(
            "best_pose_topic", "/map_nav/route_localization/best_pose"
        )
        self.declare_parameter("closed_route", True)
        self.declare_parameter("laser_x", 0.065)
        self.declare_parameter("laser_y", 0.0)
        self.declare_parameter("laser_yaw", 0.0)
        self.declare_parameter("minimum_scan_points", 80)
        self.declare_parameter("maximum_scan_points", 180)
        self.declare_parameter("maximum_scan_range_m", 6.0)
        self.declare_parameter("matching_rate_hz", 2.0)
        self.declare_parameter("route_sample_spacing_m", 0.40)
        self.declare_parameter("lateral_search_m", 0.30)
        self.declare_parameter("lateral_step_m", 0.15)
        self.declare_parameter("yaw_search_deg", 20.0)
        self.declare_parameter("yaw_step_deg", 5.0)
        self.declare_parameter("refinement_xy_m", 0.12)
        self.declare_parameter("refinement_xy_step_m", 0.04)
        self.declare_parameter("refinement_yaw_deg", 4.0)
        self.declare_parameter("refinement_yaw_step_deg", 2.0)
        self.declare_parameter("inlier_distance_m", 0.15)
        self.declare_parameter("maximum_distance_m", 0.60)
        self.declare_parameter("ambiguity_separation_m", 2.0)
        self.declare_parameter("minimum_inlier_ratio", 0.45)
        self.declare_parameter("maximum_mean_distance_m", 0.22)
        self.declare_parameter("minimum_score_margin", 0.06)
        self.declare_parameter("batch_size", 128)
        self.declare_parameter("allow_reverse_heading", False)
        self.declare_parameter("lateral_prior_weight", 0.02)
        self.declare_parameter("yaw_prior_weight", 0.04)
        self.declare_parameter("required_consistent_scans", 4)
        self.declare_parameter("consistency_route_distance_m", 0.75)
        self.declare_parameter("consistency_position_m", 0.50)
        self.declare_parameter("consistency_yaw_deg", 12.0)
        self.declare_parameter("localization_settle_sec", 2.0)
        self.declare_parameter("alignment_position_tolerance_m", 0.75)
        self.declare_parameter("alignment_yaw_tolerance_deg", 20.0)
        self.declare_parameter("initial_covariance_xy", 0.04)
        self.declare_parameter(
            "initial_covariance_yaw",
            math.radians(5.0) ** 2,
        )

    def _publish_ready(self, value: bool) -> None:
        self.ready = bool(value)
        message = Bool()
        message.data = self.ready
        self.ready_pub.publish(message)

    def _publish_status(self, value: str) -> None:
        if value == self.last_status:
            return
        self.last_status = value
        message = String()
        message.data = value
        self.status_pub.publish(message)

    def _on_path(self, message: PathMessage) -> None:
        points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
            )
            for pose in message.poses
        ]
        if len(points) < 2:
            return
        self.route_points = points
        if not self.initial_pose_published:
            self._publish_status(
                f"WAIT_SCAN route_points={len(self.route_points)}"
            )

    def _on_scan(self, message: LaserScan) -> None:
        if self.initial_pose_published or len(self.route_points) < 2:
            return
        now = time.monotonic()
        rate = max(
            0.1,
            float(self.get_parameter("matching_rate_hz").value),
        )
        if now - self.last_match_time < 1.0 / rate:
            return
        self.last_match_time = now
        points = scan_points_in_base(
            message.ranges,
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=min(
                float(message.range_max),
                float(
                    self.get_parameter("maximum_scan_range_m").value
                ),
            ),
            laser_x=float(self.get_parameter("laser_x").value),
            laser_y=float(self.get_parameter("laser_y").value),
            laser_yaw=float(self.get_parameter("laser_yaw").value),
            maximum_points=int(
                self.get_parameter("maximum_scan_points").value
            ),
        )
        minimum_points = int(
            self.get_parameter("minimum_scan_points").value
        )
        if points.shape[0] < minimum_points:
            self.pending_match = None
            self.consistent_scans = 0
            self._publish_status(
                f"WAIT_SCAN_POINTS points={points.shape[0]}/"
                f"{minimum_points}"
            )
            return

        try:
            match = match_scan_to_route(
                self.field,
                self.route_points,
                points,
                closed=self.closed_route,
                config=self.config,
            )
        except ValueError as exc:
            self._publish_status(f"MATCH_ERROR {exc}")
            return
        if not match.accepted:
            self.pending_match = None
            self.consistent_scans = 0
            self._publish_match(match)
            self._publish_status(
                "AMBIGUOUS "
                f"s={match.route_s:.2f} score={match.score:.3f} "
                f"margin={match.score_margin:.3f} "
                f"inlier={match.inlier_ratio:.2f} "
                f"mean={match.mean_distance_m:.3f}m"
            )
            return

        if self.pending_match is not None and self._consistent(
            self.pending_match,
            match,
        ):
            self.consistent_scans += 1
        else:
            self.consistent_scans = 1
        self.pending_match = match
        self._publish_match(match)
        required = max(
            1,
            int(self.get_parameter("required_consistent_scans").value),
        )
        self._publish_status(
            "MATCHING "
            f"{self.consistent_scans}/{required} "
            f"x={match.x:.2f} y={match.y:.2f} "
            f"yaw={math.degrees(match.yaw):.1f}deg "
            f"margin={match.score_margin:.3f}"
        )
        if self.consistent_scans >= required:
            self._publish_initial_pose(match)

    def _consistent(
        self,
        previous: RouteMatch,
        current: RouteMatch,
    ) -> bool:
        if route_distance(previous, current) > float(
            self.get_parameter("consistency_route_distance_m").value
        ):
            return False
        if math.hypot(
            previous.x - current.x,
            previous.y - current.y,
        ) > float(self.get_parameter("consistency_position_m").value):
            return False
        yaw_error = abs(normalize_angle(previous.yaw - current.yaw))
        return yaw_error <= math.radians(
            float(self.get_parameter("consistency_yaw_deg").value)
        )

    def _publish_match(self, match: RouteMatch) -> None:
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.frame_id = str(
            self.get_parameter("map_frame").value
        )
        pose.header.stamp = stamp
        pose.pose.position.x = match.x
        pose.pose.position.y = match.y
        pose.pose.orientation.z = math.sin(match.yaw * 0.5)
        pose.pose.orientation.w = math.cos(match.yaw * 0.5)
        self.best_pose_pub.publish(pose)

        debug = Float32MultiArray()
        debug.data = [
            match.score,
            match.second_score,
            match.score_margin,
            match.inlier_ratio,
            match.mean_distance_m,
            match.route_s,
            float(self.consistent_scans),
            1.0 if match.accepted else 0.0,
        ]
        self.debug_pub.publish(debug)

    def _publish_initial_pose(self, match: RouteMatch) -> None:
        if self.initial_pose_pub.get_subscription_count() < 1:
            self._publish_status("WAIT_SLAM_TOOLBOX_INITIALPOSE_SUBSCRIBER")
            return
        message = PoseWithCovarianceStamped()
        message.header.frame_id = str(
            self.get_parameter("map_frame").value
        )
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = match.x
        message.pose.pose.position.y = match.y
        message.pose.pose.orientation.z = math.sin(match.yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(match.yaw * 0.5)
        covariance_xy = max(
            1.0e-4,
            float(self.get_parameter("initial_covariance_xy").value),
        )
        message.pose.covariance[0] = covariance_xy
        message.pose.covariance[7] = covariance_xy
        message.pose.covariance[35] = max(
            1.0e-4,
            float(self.get_parameter("initial_covariance_yaw").value),
        )
        self.initial_pose_pub.publish(message)
        self.initial_pose_published = True
        self.initial_pose_match = match
        self.initial_pose_publish_time = time.monotonic()
        self._publish_status(
            "SETTLING "
            f"x={match.x:.2f} y={match.y:.2f} "
            f"yaw={math.degrees(match.yaw):.1f}deg"
        )
        self.get_logger().info(
            "published route-constrained initial pose: "
            f"x={match.x:.3f}, y={match.y:.3f}, "
            f"yaw={math.degrees(match.yaw):.2f} deg, "
            f"margin={match.score_margin:.3f}"
        )

    def _settle_step(self) -> None:
        if not self.initial_pose_published or self.ready:
            return
        settle_sec = max(
            0.0,
            float(self.get_parameter("localization_settle_sec").value),
        )
        if time.monotonic() - self.initial_pose_publish_time < settle_sec:
            return
        if self.initial_pose_match is None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                Time(),
            )
        except TransformException:
            self._publish_status("WAIT_LOCALIZATION_ALIGNMENT_TF")
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        localized_yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        position_error = math.hypot(
            float(translation.x) - self.initial_pose_match.x,
            float(translation.y) - self.initial_pose_match.y,
        )
        yaw_error = abs(
            normalize_angle(localized_yaw - self.initial_pose_match.yaw)
        )
        if position_error > float(
            self.get_parameter("alignment_position_tolerance_m").value
        ) or yaw_error > math.radians(
            float(
                self.get_parameter(
                    "alignment_yaw_tolerance_deg"
                ).value
            )
        ):
            self._publish_status(
                "WAIT_LOCALIZATION_ALIGNMENT "
                f"position_error={position_error:.2f}m "
                f"yaw_error={math.degrees(yaw_error):.1f}deg"
            )
            return
        self._publish_ready(True)
        self._publish_status("READY")
        self.get_logger().info(
            "route localization ready; navigation gate released"
        )

    def _relocalize(self, _, response):
        self.pending_match = None
        self.consistent_scans = 0
        self.initial_pose_published = False
        self.initial_pose_match = None
        self.initial_pose_publish_time = 0.0
        self._publish_ready(False)
        self._publish_status(
            "WAIT_SCAN" if self.route_points else "WAIT_PATH"
        )
        response.success = True
        response.message = "route localization restarted"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteScanLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
