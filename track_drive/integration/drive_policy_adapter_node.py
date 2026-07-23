import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from .drive_policy_adapter import (
    drive_policy_source_is_fresh,
    policy_debug_values_are_valid,
)


class MissionDrivePolicyAdapterNode(Node):
    """일반 IL 모델의 성공 추론 여부만 Mission Manager에 전달한다."""

    def __init__(self) -> None:
        super().__init__("mission_drive_policy_adapter")

        self.declare_parameter("source_debug_topic", "/il/policy_debug")
        self.declare_parameter(
            "output_valid_topic",
            "/mission/input/drive_policy_valid",
        )
        self.declare_parameter("source_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        source_topic = str(
            self.get_parameter("source_debug_topic").value
        )
        output_topic = str(
            self.get_parameter("output_valid_topic").value
        )
        publish_rate_hz = max(
            float(self.get_parameter("publish_rate_hz").value),
            1.0,
        )

        self._latest_values_valid = False
        self._last_receive_sec: float | None = None

        self._valid_publisher = self.create_publisher(
            Bool,
            output_topic,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            source_topic,
            self._on_policy_debug,
            10,
        )
        self.create_timer(1.0 / publish_rate_hz, self._publish_validity)

        self.get_logger().info(
            "Drive-policy adapter ready | source={} valid={}".format(
                source_topic,
                output_topic,
            )
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_policy_debug(self, message: Float32MultiArray) -> None:
        self._latest_values_valid = policy_debug_values_are_valid(
            message.data
        )
        self._last_receive_sec = self._now_sec()
        self._publish_validity()

    def _publish_validity(self) -> None:
        valid = drive_policy_source_is_fresh(
            now_sec=self._now_sec(),
            last_receive_sec=self._last_receive_sec,
            timeout_sec=max(
                float(
                    self.get_parameter("source_timeout_sec").value
                ),
                0.0,
            ),
            source_values_valid=self._latest_values_valid,
        )

        message = Bool()
        message.data = valid
        self._valid_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionDrivePolicyAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
