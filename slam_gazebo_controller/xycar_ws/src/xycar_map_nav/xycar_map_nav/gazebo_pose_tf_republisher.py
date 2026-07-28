"""Publish Gazebo's absolute model pose as the map-to-vehicle TF."""

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def select_pose_transform(
    message: TFMessage, pose_index: int
) -> TransformStamped | None:
    """Return the configured model pose from Gazebo's Pose_V bridge."""
    index = int(pose_index)
    if index < 0 or index >= len(message.transforms):
        return None
    return message.transforms[index]


class GazeboPoseTfRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_pose_tf_republisher")
        self.declare_parameter(
            "pose_topic",
            "/world/kookmin_xycar_track/dynamic_pose/info",
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("pose_index", 0)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._pose_index = int(self.get_parameter("pose_index").value)
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            TFMessage,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            qos_profile_sensor_data,
        )

    def _on_pose(self, message: TFMessage) -> None:
        source = select_pose_transform(message, self._pose_index)
        if source is None:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._base_frame
        transform.transform = source.transform
        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboPoseTfRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
