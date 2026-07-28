"""Publish occupied map cells as a point cloud for RViz shader compatibility."""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class OccupancyGridToCloud(Node):
    def __init__(self) -> None:
        super().__init__("occupancy_grid_to_cloud")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("cloud_topic", "/map_points")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("maximum_points", 250_000)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            cloud_qos,
        )
        self._subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        height = int(message.info.height)
        width = int(message.info.width)
        if height <= 0 or width <= 0:
            return

        grid = np.asarray(message.data, dtype=np.int16).reshape(height, width)
        rows, columns = np.nonzero(
            grid >= int(self.get_parameter("occupied_threshold").value)
        )
        maximum_points = int(self.get_parameter("maximum_points").value)
        if maximum_points > 0 and rows.size > maximum_points:
            step = int(math.ceil(rows.size / maximum_points))
            rows = rows[::step]
            columns = columns[::step]

        resolution = float(message.info.resolution)
        local_x = (columns.astype(np.float32) + 0.5) * resolution
        local_y = (rows.astype(np.float32) + 0.5) * resolution
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        origin = message.info.origin.position
        world_x = origin.x + cos_yaw * local_x - sin_yaw * local_y
        world_y = origin.y + sin_yaw * local_x + cos_yaw * local_y
        points = np.column_stack(
            (
                world_x,
                world_y,
                np.zeros(world_x.shape, dtype=np.float32),
            )
        ).astype(np.float32, copy=False)

        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = message.header.frame_id or "map"
        self._publisher.publish(
            point_cloud2.create_cloud_xyz32(header, points)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OccupancyGridToCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
