import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .start_signal_adapter import (
    normalize_start_signal_state,
    start_signal_source_is_fresh,
)
from .states import StartSignal


class MissionStartSignalAdapterNode(Node):
    """기존 신호등 상태 토픽을 Mission Manager 입력 계약으로 변환한다."""

    def __init__(self) -> None:
        super().__init__("mission_start_signal_adapter")

        self.declare_parameter(
            "source_state_topic",
            "/track_drive/traffic_light_debug/state",
        )
        self.declare_parameter(
            "output_signal_topic",
            "/mission/input/start_signal",
        )
        self.declare_parameter(
            "output_valid_topic",
            "/mission/input/start_signal_valid",
        )
        self.declare_parameter("source_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)

        source_topic = str(
            self.get_parameter("source_state_topic").value
        )
        output_signal_topic = str(
            self.get_parameter("output_signal_topic").value
        )
        output_valid_topic = str(
            self.get_parameter("output_valid_topic").value
        )
        publish_rate_hz = max(
            float(self.get_parameter("publish_rate_hz").value),
            1.0,
        )

        self._latest_signal = StartSignal.UNKNOWN
        self._latest_source_state_valid = False
        self._last_receive_sec: float | None = None

        self._signal_publisher = self.create_publisher(
            String,
            output_signal_topic,
            10,
        )
        self._valid_publisher = self.create_publisher(
            Bool,
            output_valid_topic,
            10,
        )
        self.create_subscription(
            String,
            source_topic,
            self._on_source_state,
            10,
        )
        self.create_timer(1.0 / publish_rate_hz, self._publish_latest)

        self.get_logger().info(
            "Start-signal adapter ready | source={} signal={} valid={}".format(
                source_topic,
                output_signal_topic,
                output_valid_topic,
            )
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_source_state(self, message: String) -> None:
        signal, valid = normalize_start_signal_state(message.data)
        self._latest_signal = signal
        self._latest_source_state_valid = valid
        self._last_receive_sec = self._now_sec()
        self._publish_latest()

    def _publish_latest(self) -> None:
        valid = start_signal_source_is_fresh(
            now_sec=self._now_sec(),
            last_receive_sec=self._last_receive_sec,
            timeout_sec=max(
                float(
                    self.get_parameter("source_timeout_sec").value
                ),
                0.0,
            ),
            source_state_valid=self._latest_source_state_valid,
        )

        signal_message = String()
        signal_message.data = (
            self._latest_signal.name
            if valid
            else StartSignal.UNKNOWN.name
        )
        valid_message = Bool()
        valid_message.data = valid

        self._signal_publisher.publish(signal_message)
        self._valid_publisher.publish(valid_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionStartSignalAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
