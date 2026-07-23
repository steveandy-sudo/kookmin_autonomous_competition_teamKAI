import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

from .camera_cone_adapter import camera_cone_source_is_fresh


class MissionCameraConeAdapterNode(Node):
    """Convert a camera cone count into fresh Mission Manager inputs."""

    def __init__(self) -> None:
        super().__init__("mission_camera_cone_adapter")

        self.declare_parameter(
            "source_count_topic",
            "/perception/camera_cone_count",
        )
        self.declare_parameter(
            "output_count_topic",
            "/mission/input/camera_cone_count",
        )
        self.declare_parameter(
            "output_valid_topic",
            "/mission/input/camera_cone_valid",
        )
        self.declare_parameter("source_timeout_sec", 0.2)
        self.declare_parameter("publish_rate_hz", 20.0)

        source_topic = str(
            self.get_parameter("source_count_topic").value
        )
        output_count_topic = str(
            self.get_parameter("output_count_topic").value
        )
        output_valid_topic = str(
            self.get_parameter("output_valid_topic").value
        )
        self._source_timeout_sec = max(
            float(self.get_parameter("source_timeout_sec").value),
            0.0,
        )
        publish_rate_hz = max(
            float(self.get_parameter("publish_rate_hz").value),
            1.0,
        )

        self._latest_count = -1
        self._last_receive_sec: float | None = None

        self._count_publisher = self.create_publisher(
            Int32,
            output_count_topic,
            10,
        )
        self._valid_publisher = self.create_publisher(
            Bool,
            output_valid_topic,
            10,
        )
        self.create_subscription(
            Int32,
            source_topic,
            self._on_source_count,
            10,
        )
        self.create_timer(1.0 / publish_rate_hz, self._publish_inputs)

        self.get_logger().info(
            "Camera-cone adapter ready | source={} count={} valid={}".format(
                source_topic,
                output_count_topic,
                output_valid_topic,
            )
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_source_count(self, message: Int32) -> None:
        self._latest_count = int(message.data)
        self._last_receive_sec = self._now_sec()
        self._publish_inputs()

    def _publish_inputs(self) -> None:
        valid = camera_cone_source_is_fresh(
            now_sec=self._now_sec(),
            last_receive_sec=self._last_receive_sec,
            timeout_sec=self._source_timeout_sec,
            source_count=self._latest_count,
        )

        count_message = Int32()
        count_message.data = self._latest_count if valid else 0
        self._count_publisher.publish(count_message)

        valid_message = Bool()
        valid_message.data = valid
        self._valid_publisher.publish(valid_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionCameraConeAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
