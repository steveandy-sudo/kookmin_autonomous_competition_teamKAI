"""Publish a metric top-down track reference for RViz waypoint capture."""

from __future__ import annotations

from pathlib import Path

import cv2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
import yaml


def display_occupancy_from_image(
    image: np.ndarray,
    *,
    line_threshold: int = 180,
    background_value: int = 18,
    line_value: int = 88,
) -> np.ndarray:
    """Convert the track preview into a visible, non-planning occupancy map."""
    if image.ndim == 3:
        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    elif image.ndim == 2:
        grayscale = image
    else:
        raise ValueError(f"unsupported map image shape: {image.shape}")
    occupancy = np.full(
        grayscale.shape,
        int(background_value),
        dtype=np.int8,
    )
    occupancy[grayscale >= int(line_threshold)] = int(line_value)
    return occupancy


class StaticTrackMapNode(Node):
    def __init__(self) -> None:
        super().__init__("static_track_map")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("line_threshold", 180)
        self.declare_parameter("background_value", 18)
        self.declare_parameter("line_value", 88)

        map_yaml = Path(
            str(self.get_parameter("map_yaml").value)
        ).expanduser().resolve()
        if not map_yaml.is_file():
            raise ValueError(f"map YAML does not exist: {map_yaml}")
        config = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
        image_path = Path(str(config["image"]))
        if not image_path.is_absolute():
            image_path = map_yaml.parent / image_path
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"cannot read map image: {image_path}")

        occupancy = display_occupancy_from_image(
            image,
            line_threshold=int(
                self.get_parameter("line_threshold").value
            ),
            background_value=int(
                self.get_parameter("background_value").value
            ),
            line_value=int(self.get_parameter("line_value").value),
        )
        origin = [float(value) for value in config["origin"]]
        self.message = OccupancyGrid()
        self.message.header.frame_id = str(
            self.get_parameter("frame_id").value
        )
        self.message.info.resolution = float(config["resolution"])
        self.message.info.width = int(occupancy.shape[1])
        self.message.info.height = int(occupancy.shape[0])
        self.message.info.origin.position.x = origin[0]
        self.message.info.origin.position.y = origin[1]
        self.message.info.origin.position.z = 0.0
        self.message.info.origin.orientation.z = float(
            np.sin(origin[2] * 0.5)
        )
        self.message.info.origin.orientation.w = float(
            np.cos(origin[2] * 0.5)
        )
        self.message.data = np.flipud(occupancy).reshape(-1).tolist()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            qos,
        )
        self.create_timer(1.0, self._publish)
        self._publish()
        self.get_logger().info(
            f"track map: {image_path} "
            f"{self.message.info.width}x{self.message.info.height} "
            f"at {self.message.info.resolution:.3f} m/px"
        )

    def _publish(self) -> None:
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticTrackMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
