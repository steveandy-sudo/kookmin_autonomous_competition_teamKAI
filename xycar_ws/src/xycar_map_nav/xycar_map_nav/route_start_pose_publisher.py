import math
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml


def route_start_pose(waypoints_yaml: Path) -> tuple[float, float, float]:
    data = yaml.safe_load(waypoints_yaml.read_text(encoding="utf-8"))
    waypoints = data.get("waypoints", [])
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    first = waypoints[0]
    second = waypoints[1]
    x = float(first["x"])
    y = float(first["y"])
    yaw = math.atan2(
        float(second["y"]) - y,
        float(second["x"]) - x,
    )
    return x, y, yaw


class RouteStartPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("route_start_pose_publisher")
        self.declare_parameter("waypoints_yaml", "")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("startup_delay_sec", 1.0)
        self.declare_parameter("publish_period_sec", 0.40)
        self.declare_parameter("publish_count", 5)

        waypoint_path = Path(
            str(self.get_parameter("waypoints_yaml").value)
        ).expanduser().resolve()
        self.start_x, self.start_y, self.start_yaw = route_start_pose(
            waypoint_path
        )
        self.remaining = max(
            1,
            int(self.get_parameter("publish_count").value),
        )
        self.start_time = self.get_clock().now()
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initial_pose_topic").value),
            10,
        )
        self.timer = self.create_timer(
            max(
                0.10,
                float(self.get_parameter("publish_period_sec").value),
            ),
            self._publish_pose,
        )

    def _publish_pose(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed < max(
            0.0,
            float(self.get_parameter("startup_delay_sec").value),
        ):
            return
        if self.publisher.get_subscription_count() < 1:
            return

        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = self.start_x
        message.pose.pose.position.y = self.start_y
        message.pose.pose.orientation.z = math.sin(self.start_yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(self.start_yaw * 0.5)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.06853891909122467
        self.publisher.publish(message)

        self.remaining -= 1
        if self.remaining <= 0:
            self.timer.cancel()
            self.get_logger().info(
                "initialized localization from route start: "
                f"x={self.start_x:.3f}, y={self.start_y:.3f}, "
                f"yaw={math.degrees(self.start_yaw):.2f} deg"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteStartPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
