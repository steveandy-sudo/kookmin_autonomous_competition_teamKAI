import math

import rclpy
from kaiev26_msgs.msg import Centerline, RoadSegmentArray
from rclpy.node import Node
from std_msgs.msg import Bool
from teamkai_interfaces.msg import LaneFallbackCommand

from .lane_fallback_adapter import (
    centerline_values_are_valid,
    lane_fallback_command_values_are_valid,
    lane_path_evidence_is_valid,
    lane_fallback_source_is_fresh,
)


class MissionLaneFallbackAdapterNode(Node):
    """Convert Centerline quality and freshness into a mission validity flag."""

    def __init__(self) -> None:
        super().__init__("mission_lane_fallback_adapter")

        self.declare_parameter(
            "source_centerline_topic",
            "/perception/centerline",
        )
        self.declare_parameter(
            "source_road_segments_topic",
            "/perception/road_segments",
        )
        self.declare_parameter(
            "source_command_topic",
            "/lane_fallback/command",
        )
        self.declare_parameter(
            "output_valid_topic",
            "/mission/input/lane_fallback_valid",
        )
        self.declare_parameter("source_timeout_sec", 0.4)
        self.declare_parameter("command_timeout_sec", 0.2)
        self.declare_parameter("minimum_point_count", 3)
        self.declare_parameter("minimum_confidence", 0.25)
        self.declare_parameter("max_steering_angle_deg", 26.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        source_topic = str(
            self.get_parameter("source_centerline_topic").value
        )
        road_segments_topic = str(
            self.get_parameter("source_road_segments_topic").value
        )
        command_topic = str(
            self.get_parameter("source_command_topic").value
        )
        output_topic = str(
            self.get_parameter("output_valid_topic").value
        )
        self._source_timeout_sec = max(
            float(self.get_parameter("source_timeout_sec").value),
            0.0,
        )
        self._command_timeout_sec = max(
            float(self.get_parameter("command_timeout_sec").value),
            0.0,
        )
        self._minimum_point_count = max(
            int(self.get_parameter("minimum_point_count").value),
            1,
        )
        self._minimum_confidence = float(
            self.get_parameter("minimum_confidence").value
        )
        if not 0.0 <= self._minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be between 0.0 and 1.0")
        self._max_steering_angle_deg = float(
            self.get_parameter("max_steering_angle_deg").value
        )
        if (
            not math.isfinite(self._max_steering_angle_deg)
            or self._max_steering_angle_deg <= 0.0
        ):
            raise ValueError("max_steering_angle_deg must be positive")
        publish_rate_hz = max(
            float(self.get_parameter("publish_rate_hz").value),
            1.0,
        )

        self._latest_values_valid = False
        self._last_receive_sec: float | None = None
        self._latest_path_evidence_valid = False
        self._last_path_evidence_receive_sec: float | None = None
        self._latest_command_values_valid = False
        self._last_command_receive_sec: float | None = None

        self._valid_publisher = self.create_publisher(
            Bool,
            output_topic,
            10,
        )
        self.create_subscription(
            Centerline,
            source_topic,
            self._on_centerline,
            10,
        )
        self.create_subscription(
            RoadSegmentArray,
            road_segments_topic,
            self._on_road_segments,
            10,
        )
        self.create_subscription(
            LaneFallbackCommand,
            command_topic,
            self._on_command,
            10,
        )
        self.create_timer(1.0 / publish_rate_hz, self._publish_validity)

        self.get_logger().info(
            "Lane-fallback adapter ready | centerline={} segments={} "
            "command={} valid={}".format(
                source_topic,
                road_segments_topic,
                command_topic,
                output_topic,
            )
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_centerline(self, message: Centerline) -> None:
        self._latest_values_valid = centerline_values_are_valid(
            points=message.points,
            confidence=message.confidence,
            minimum_point_count=self._minimum_point_count,
            minimum_confidence=self._minimum_confidence,
        )
        self._last_receive_sec = self._now_sec()
        self._publish_validity()

    def _on_road_segments(self, message: RoadSegmentArray) -> None:
        self._latest_path_evidence_valid = lane_path_evidence_is_valid(
            segments=message.segments,
            minimum_point_count=self._minimum_point_count,
            minimum_confidence=self._minimum_confidence,
        )
        self._last_path_evidence_receive_sec = self._now_sec()
        self._publish_validity()

    def _on_command(self, message: LaneFallbackCommand) -> None:
        self._latest_command_values_valid = (
            lane_fallback_command_values_are_valid(
                steering_angle_deg=message.steering_angle_deg,
                command_valid=message.valid,
                max_steering_angle_deg=self._max_steering_angle_deg,
            )
        )
        self._last_command_receive_sec = self._now_sec()
        self._publish_validity()

    def _publish_validity(self) -> None:
        centerline_valid = lane_fallback_source_is_fresh(
            now_sec=self._now_sec(),
            last_receive_sec=self._last_receive_sec,
            timeout_sec=self._source_timeout_sec,
            source_values_valid=self._latest_values_valid,
        )
        path_evidence_valid = lane_fallback_source_is_fresh(
            now_sec=self._now_sec(),
            last_receive_sec=self._last_path_evidence_receive_sec,
            timeout_sec=self._source_timeout_sec,
            source_values_valid=self._latest_path_evidence_valid,
        )
        command_valid = lane_fallback_source_is_fresh(
            now_sec=self._now_sec(),
            last_receive_sec=self._last_command_receive_sec,
            timeout_sec=self._command_timeout_sec,
            source_values_valid=self._latest_command_values_valid,
        )

        message = Bool()
        message.data = (
            centerline_valid
            and path_evidence_valid
            and command_valid
        )
        self._valid_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionLaneFallbackAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
