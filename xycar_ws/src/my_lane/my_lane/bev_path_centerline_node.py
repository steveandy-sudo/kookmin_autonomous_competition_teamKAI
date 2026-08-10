#!/usr/bin/env python3
"""Convert the pre-canonical BEV pixel path to a metric centerline."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from my_msgs.msg import Centerline
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


def bev_pixels_to_metric_path(
    values: list[float],
    *,
    bev_width: int,
    bev_height: int,
    lateral_range_m: float,
    forward_range_m: float,
    vehicle_x_px: float,
    vehicle_y_px: float,
    minimum_forward_m: float = 0.03,
) -> np.ndarray:
    """Return sorted [forward, lateral] points in the vehicle frame."""
    if len(values) < 6 or len(values) % 2:
        return np.empty((0, 2), dtype=np.float64)
    pixels = np.asarray(values, dtype=np.float64).reshape((-1, 2))
    pixels = pixels[np.all(np.isfinite(pixels), axis=1)]
    if pixels.shape[0] < 3:
        return np.empty((0, 2), dtype=np.float64)

    lateral_scale = float(lateral_range_m) / max(1.0, float(bev_width))
    forward_scale = float(forward_range_m) / max(1.0, float(bev_height - 1))
    forward = (float(vehicle_y_px) - pixels[:, 1]) * forward_scale
    lateral = (float(vehicle_x_px) - pixels[:, 0]) * lateral_scale
    metric = np.column_stack((forward, lateral))
    valid = (
        (metric[:, 0] >= float(minimum_forward_m))
        & (metric[:, 0] <= float(forward_range_m) + 1.0e-6)
    )
    metric = metric[valid]
    if metric.shape[0] < 3:
        return np.empty((0, 2), dtype=np.float64)
    return metric[np.argsort(metric[:, 0])]


class BevPathCenterlineNode(Node):
    def __init__(self) -> None:
        super().__init__("bev_path_centerline")
        self.declare_parameter("path_pixels_topic", "/lane_path/path_pixels")
        self.declare_parameter("source_image_topic", "/lane_seg/source_image")
        self.declare_parameter(
            "centerline_topic", "/perception/bev_direct_centerline"
        )
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("bev_width", 640)
        self.declare_parameter("bev_height", 480)
        self.declare_parameter("lateral_range_m", 1.4)
        self.declare_parameter("forward_range_m", 1.5)
        self.declare_parameter("vehicle_x_px", 320.0)
        self.declare_parameter("vehicle_y_px", 479.0)
        self.declare_parameter("minimum_forward_m", 0.03)

        self.latest_header = None
        self.sequence = 0
        self.publisher = self.create_publisher(
            Centerline,
            str(self.get_parameter("centerline_topic").value),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("source_image_topic").value),
            self.on_source_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("path_pixels_topic").value),
            self.on_path_pixels,
            10,
        )
        self.get_logger().info(
            "direct BEV path adapter ready: canonical conversion disabled"
        )

    def on_source_image(self, message: Image) -> None:
        self.latest_header = message.header

    def on_path_pixels(self, message: Float32MultiArray) -> None:
        path = bev_pixels_to_metric_path(
            list(message.data),
            bev_width=int(self.get_parameter("bev_width").value),
            bev_height=int(self.get_parameter("bev_height").value),
            lateral_range_m=float(
                self.get_parameter("lateral_range_m").value
            ),
            forward_range_m=float(
                self.get_parameter("forward_range_m").value
            ),
            vehicle_x_px=float(self.get_parameter("vehicle_x_px").value),
            vehicle_y_px=float(self.get_parameter("vehicle_y_px").value),
            minimum_forward_m=float(
                self.get_parameter("minimum_forward_m").value
            ),
        )
        output = Centerline()
        if self.latest_header is not None:
            output.header = self.latest_header
        else:
            output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = str(
            self.get_parameter("base_frame_id").value
        )
        self.sequence += 1
        output.detection_id = self.sequence
        output.track_id = 0
        for forward_m, lateral_m in path:
            point = Point()
            point.x = float(forward_m)
            point.y = float(lateral_m)
            point.z = 0.0
            output.points.append(point)
        output.confidence = 1.0 if path.shape[0] >= 3 else 0.0
        output.source = "lane_seg_direct_bev"
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BevPathCenterlineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
