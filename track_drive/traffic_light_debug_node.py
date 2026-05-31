#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from track_drive.traffic_light_detector import (
    YoloTrafficLightDetector,
    declare_traffic_light_parameters,
)


class TrafficLightDebugNode(Node):
    """Run traffic-light recognition only and publish RViz-friendly debug output."""

    def __init__(self):
        super().__init__('traffic_light_debug')

        declare_traffic_light_parameters(self)
        self.bridge = CvBridge()
        self.detector = YoloTrafficLightDetector(self)
        self.last_log_sec = 0.0

        camera_topic = str(self.get_parameter('camera_topic').value)
        image_topic = str(self.get_parameter('traffic_light_debug_topic').value)
        state_topic = str(self.get_parameter('traffic_light_state_topic').value)

        self.debug_pub = self.create_publisher(Image, image_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.create_subscription(Image, camera_topic, self.image_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'Traffic-light debug ready | image={camera_topic}, debug={image_topic}, state={state_topic}'
        )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'image conversion failed: {exc}')
            return

        result = self.detector.detect(frame)
        self._publish_state(result.state)
        self._publish_debug_image(result.debug_image, msg)
        self._log_result(result)

    def _publish_debug_image(self, image, source_msg: Image):
        if image is None:
            return
        try:
            out = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            out.header.stamp = source_msg.header.stamp
            out.header.frame_id = source_msg.header.frame_id
            self.debug_pub.publish(out)
        except Exception as exc:
            self.get_logger().warn(f'traffic-light debug image publish failed: {exc}')

    def _publish_state(self, state: str):
        msg = String()
        msg.data = str(state)
        self.state_pub.publish(msg)

    def _log_result(self, result):
        now = time.monotonic()
        period = max(float(self.get_parameter('traffic_light_log_period_sec').value), 0.0)
        if period > 0.0 and now - self.last_log_sec < period:
            return
        self.last_log_sec = now
        score_text = ' '.join(f'{idx}:{score:.2f}' for idx, score in enumerate(result.class_scores[:6]))
        self.get_logger().info(
            f'traffic_light={result.state} det={len(result.detections)} scores=[{score_text}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
