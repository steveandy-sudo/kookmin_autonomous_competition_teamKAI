"""ROS wrapper for the Lane Fallback Pure Pursuit controller."""

import math

from builtin_interfaces.msg import Time
from kaiev26_msgs.msg import Centerline
import rclpy
from rclpy.node import Node
from teamkai_interfaces.msg import LaneFallbackCommand

from .integration.lane_fallback_adapter import (
    centerline_values_are_valid,
    lane_fallback_source_is_fresh,
)
from .lane_fallback_controller import (
    INVALID_STEERING,
    LaneFallbackParameters,
    LaneFallbackSteering,
    compute_lane_fallback_steering,
)


class LaneFallbackControllerNode(Node):
    """Convert a valid metric Centerline into a physical steering candidate."""

    def __init__(self) -> None:
        super().__init__("lane_fallback_controller")

        self.declare_parameter(
            "source_centerline_topic",
            "/perception/centerline",
        )
        self.declare_parameter(
            "output_command_topic",
            "/lane_fallback/command",
        )
        self.declare_parameter("expected_frame_id", "base_footprint")
        self.declare_parameter("source_timeout_sec", 0.4)
        self.declare_parameter("minimum_confidence", 0.25)
        self.declare_parameter("minimum_point_count", 3)
        self.declare_parameter("wheelbase_m", 0.33)
        self.declare_parameter("near_lookahead_m", 0.70)
        self.declare_parameter("far_preview_distance_m", 0.75)
        self.declare_parameter("max_lookahead_m", 1.45)
        self.declare_parameter("far_preview_weight", 0.65)
        self.declare_parameter("steering_gain", 1.0)
        self.declare_parameter("max_steering_angle_deg", 26.0)
        self.declare_parameter("minimum_path_distance_m", 0.70)
        self.declare_parameter("publish_rate_hz", 20.0)

        source_topic = str(
            self.get_parameter("source_centerline_topic").value
        )
        output_topic = str(
            self.get_parameter("output_command_topic").value
        )
        self._expected_frame_id = str(
            self.get_parameter("expected_frame_id").value
        )
        self._source_timeout_sec = float(
            self.get_parameter("source_timeout_sec").value
        )
        self._minimum_confidence = float(
            self.get_parameter("minimum_confidence").value
        )
        minimum_point_count = int(
            self.get_parameter("minimum_point_count").value
        )
        publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )

        if (
            not math.isfinite(self._source_timeout_sec)
            or self._source_timeout_sec < 0.0
        ):
            raise ValueError("source_timeout_sec must be non-negative")
        if (
            not math.isfinite(self._minimum_confidence)
            or not 0.0 <= self._minimum_confidence <= 1.0
        ):
            raise ValueError("minimum_confidence must be between 0 and 1")
        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self._parameters = LaneFallbackParameters(
            wheelbase_m=float(
                self.get_parameter("wheelbase_m").value
            ),
            near_lookahead_m=float(
                self.get_parameter("near_lookahead_m").value
            ),
            far_preview_distance_m=float(
                self.get_parameter("far_preview_distance_m").value
            ),
            max_lookahead_m=float(
                self.get_parameter("max_lookahead_m").value
            ),
            far_preview_weight=float(
                self.get_parameter("far_preview_weight").value
            ),
            steering_gain=float(
                self.get_parameter("steering_gain").value
            ),
            max_steering_angle_deg=float(
                self.get_parameter("max_steering_angle_deg").value
            ),
            minimum_path_distance_m=float(
                self.get_parameter("minimum_path_distance_m").value
            ),
            minimum_point_count=minimum_point_count,
        )

        self._latest_result = INVALID_STEERING
        self._latest_source_stamp: Time | None = None
        self._last_receive_sec: float | None = None

        self._command_publisher = self.create_publisher(
            LaneFallbackCommand,
            output_topic,
            10,
        )
        self.create_subscription(
            Centerline,
            source_topic,
            self._on_centerline,
            10,
        )
        self.create_timer(
            1.0 / publish_rate_hz,
            self._publish_command,
        )

        self.get_logger().info(
            "Lane Fallback controller ready | source={} command={}".format(
                source_topic,
                output_topic,
            )
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_centerline(self, message: Centerline) -> None:
        self._last_receive_sec = self._now_sec()
        self._latest_source_stamp = message.header.stamp

        values_valid = centerline_values_are_valid(
            points=message.points,
            confidence=message.confidence,
            minimum_point_count=self._parameters.minimum_point_count,
            minimum_confidence=self._minimum_confidence,
        )
        frame_valid = (
            message.header.frame_id == self._expected_frame_id
        )
        if not values_valid or not frame_valid:
            self._latest_result = INVALID_STEERING
        else:
            self._latest_result = compute_lane_fallback_steering(
                message.points,
                self._parameters,
            )
        self._publish_command()

    def _source_is_fresh(self) -> bool:
        return lane_fallback_source_is_fresh(
            now_sec=self._now_sec(),
            last_receive_sec=self._last_receive_sec,
            timeout_sec=self._source_timeout_sec,
            source_values_valid=self._latest_result.valid,
        )

    def _publish_command(self) -> None:
        result: LaneFallbackSteering = (
            self._latest_result
            if self._source_is_fresh()
            else INVALID_STEERING
        )
        message = LaneFallbackCommand()
        message.stamp = (
            self._latest_source_stamp
            if self._latest_source_stamp is not None
            else self.get_clock().now().to_msg()
        )
        message.steering_angle_deg = result.steering_angle_deg
        message.valid = result.valid
        self._command_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneFallbackControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
