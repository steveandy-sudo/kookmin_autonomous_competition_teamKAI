"""ROS adapter for the vehicle-tested LiDAR cone source."""

import math

from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from .lidar_cone_adapter import (
    ConeCommand,
    evaluate_lidar_cone_status,
    parse_cone_command,
)


class MissionLidarConeAdapterNode(Node):
    """Convert ``my_rule/cone_node`` outputs into semantic mission inputs."""

    def __init__(self) -> None:
        super().__init__("mission_lidar_cone_adapter")

        self.declare_parameter(
            "source_command_topic",
            "/my_rule/cone_cmd",
        )
        self.declare_parameter(
            "source_cluster_topic",
            "/my_rule/cone_clusters",
        )
        self.declare_parameter(
            "output_source_valid_topic",
            "/mission/input/lidar_cone_source_valid",
        )
        self.declare_parameter(
            "output_path_ready_topic",
            "/mission/input/lidar_cone_path_ready",
        )
        self.declare_parameter(
            "output_present_topic",
            "/mission/input/lidar_cone_present",
        )
        self.declare_parameter("source_timeout_sec", 0.2)
        self.declare_parameter("path_ready_confidence", 0.35)
        self.declare_parameter("presence_confidence", 0.2)
        self.declare_parameter("presence_min_clusters", 2)
        self.declare_parameter("publish_rate_hz", 20.0)

        source_command_topic = str(
            self.get_parameter("source_command_topic").value
        )
        source_cluster_topic = str(
            self.get_parameter("source_cluster_topic").value
        )
        output_source_valid_topic = str(
            self.get_parameter("output_source_valid_topic").value
        )
        output_path_ready_topic = str(
            self.get_parameter("output_path_ready_topic").value
        )
        output_present_topic = str(
            self.get_parameter("output_present_topic").value
        )
        self._source_timeout_sec = max(
            float(self.get_parameter("source_timeout_sec").value),
            0.0,
        )
        self._path_ready_confidence = float(
            self.get_parameter("path_ready_confidence").value
        )
        self._presence_confidence = float(
            self.get_parameter("presence_confidence").value
        )
        self._presence_min_clusters = max(
            int(self.get_parameter("presence_min_clusters").value),
            1,
        )
        publish_rate_hz = max(
            float(self.get_parameter("publish_rate_hz").value),
            1.0,
        )

        self._latest_command: ConeCommand | None = None
        self._command_receive_sec: float | None = None
        self._cluster_count: int | None = None
        self._cluster_receive_sec: float | None = None

        self._source_valid_publisher = self.create_publisher(
            Bool,
            output_source_valid_topic,
            10,
        )
        self._path_ready_publisher = self.create_publisher(
            Bool,
            output_path_ready_topic,
            10,
        )
        self._present_publisher = self.create_publisher(
            Bool,
            output_present_topic,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            source_command_topic,
            self._on_source_command,
            10,
        )
        self.create_subscription(
            PoseArray,
            source_cluster_topic,
            self._on_source_clusters,
            10,
        )
        self.create_timer(1.0 / publish_rate_hz, self._publish_inputs)

        self.get_logger().info(
            "LiDAR-cone adapter ready | command={} clusters={} "
            "valid={} path_ready={} present={}".format(
                source_command_topic,
                source_cluster_topic,
                output_source_valid_topic,
                output_path_ready_topic,
                output_present_topic,
            )
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_source_command(self, message: Float32MultiArray) -> None:
        self._latest_command = parse_cone_command(message.data)
        self._command_receive_sec = self._now_sec()

    def _on_source_clusters(self, message: PoseArray) -> None:
        positions = (pose.position for pose in message.poses)
        valid = all(
            all(math.isfinite(value) for value in (p.x, p.y, p.z))
            for p in positions
        )
        self._cluster_count = len(message.poses) if valid else None
        self._cluster_receive_sec = self._now_sec()

    def _publish_inputs(self) -> None:
        status = evaluate_lidar_cone_status(
            now_sec=self._now_sec(),
            command=self._latest_command,
            command_receive_sec=self._command_receive_sec,
            cluster_count=self._cluster_count,
            cluster_receive_sec=self._cluster_receive_sec,
            timeout_sec=self._source_timeout_sec,
            path_ready_confidence=self._path_ready_confidence,
            presence_confidence=self._presence_confidence,
            presence_min_clusters=self._presence_min_clusters,
        )

        source_valid_message = Bool()
        source_valid_message.data = status.source_valid
        self._source_valid_publisher.publish(source_valid_message)

        path_ready_message = Bool()
        path_ready_message.data = status.path_ready
        self._path_ready_publisher.publish(path_ready_message)

        present_message = Bool()
        present_message.data = status.present
        self._present_publisher.publish(present_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionLidarConeAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
