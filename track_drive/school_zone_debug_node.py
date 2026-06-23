#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

from track_drive.school_zone_detector import (
    BEVSchoolZoneDetector,
    declare_school_zone_bev_parameters,
)


class SchoolZoneDebugNode(Node):
    """Publish BEV school-zone debug images without motor control."""

    def __init__(self):
        # SchoolZoneDebugNode 객체를 초기화하고 필요한 파라미터와 내부 상태를 준비한다.
        super().__init__('school_zone_debug')

        declare_school_zone_bev_parameters(self)
        self.bridge = CvBridge()
        self.detector = BEVSchoolZoneDetector(self)
        self.last_log_sec = 0.0

        camera_topic = str(self.get_parameter('camera_topic').value)
        prefix = str(self.get_parameter('school_zone_debug_topic_prefix').value).rstrip('/')

        self.roi_pub = self.create_publisher(Image, f'{prefix}/roi', 10)
        self.bev_pub = self.create_publisher(Image, f'{prefix}/bev', 10)
        self.mask_pub = self.create_publisher(Image, f'{prefix}/mask', 10)
        self.debug_pub = self.create_publisher(Image, f'{prefix}/debug', 10)
        self.active_pub = self.create_publisher(Bool, f'{prefix}/active', 10)
        self.candidate_pub = self.create_publisher(Bool, f'{prefix}/candidate', 10)
        self.speed_limit_pub = self.create_publisher(Float32, f'{prefix}/speed_limit', 10)
        self.left_ratio_pub = self.create_publisher(Float32, f'{prefix}/left_ratio', 10)
        self.right_ratio_pub = self.create_publisher(Float32, f'{prefix}/right_ratio', 10)
        self.pair_ratio_pub = self.create_publisher(Float32, f'{prefix}/pair_row_ratio', 10)

        self.create_subscription(Image, camera_topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'School-zone BEV debug ready | image={camera_topic}, topics={prefix}/roi,bev,mask,debug'
        )

    def image_callback(self, msg: Image):
        # ROS 토픽 콜백으로 들어온 메시지를 내부 상태에 반영한다.
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'image conversion failed: {exc}')
            return

        result = self.detector.detect(frame)
        self._publish_bool(self.active_pub, result.active)
        self._publish_bool(self.candidate_pub, result.candidate)
        self._publish_float(
            self.speed_limit_pub,
            float(self.get_parameter('school_zone_speed').value) if result.candidate else -1.0,
        )
        self._publish_float(self.left_ratio_pub, result.left_ratio)
        self._publish_float(self.right_ratio_pub, result.right_ratio)
        self._publish_float(self.pair_ratio_pub, result.pair_row_ratio)
        self._publish_image(self.roi_pub, result.roi_debug_image, 'bgr8', msg)
        self._publish_image(self.bev_pub, result.bev_image, 'bgr8', msg)
        self._publish_image(self.mask_pub, result.mask_image, 'mono8', msg)
        self._publish_image(self.debug_pub, result.debug_image, 'bgr8', msg)
        self._log_result(result)

    def _publish_image(self, pub, image, encoding: str, source_msg: Image):
        # publish 이미지 결과를 ROS 토픽이나 디버그 출력으로 발행한다.
        if image is None:
            return
        try:
            out = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
            out.header.stamp = source_msg.header.stamp
            out.header.frame_id = source_msg.header.frame_id
            pub.publish(out)
        except Exception as exc:
            self.get_logger().warn(f'debug image publish failed: {exc}')

    @staticmethod
    def _publish_bool(pub, value: bool):
        # publish bool 결과를 ROS 토픽이나 디버그 출력으로 발행한다.
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

    @staticmethod
    def _publish_float(pub, value: float):
        # publish float 결과를 ROS 토픽이나 디버그 출력으로 발행한다.
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    def _log_result(self, result):
        # 로그 result 정보를 사람이 읽기 쉬운 로그 문자열로 만든다.
        now = time.monotonic()
        period = max(float(self.get_parameter('school_zone_debug_log_period_sec').value), 0.0)
        if period > 0.0 and now - self.last_log_sec < period:
            return
        self.last_log_sec = now
        self.get_logger().info(
            f'school_zone={int(result.active)} candidate={int(result.candidate)} '
            f'L={result.left_ratio:.4f} R={result.right_ratio:.4f} '
            f'pair={result.pair_row_ratio:.3f} bottom={result.bottom_pair_row_ratio:.3f} '
            f'sep={result.separation_ratio:.2f} rows={result.pair_rows} '
            f'hold={result.hold_remaining_sec:.1f}s'
        )


def main(args=None):
    # ROS2 노드를 초기화하고 실행 루프를 시작한다.
    rclpy.init(args=args)
    node = SchoolZoneDebugNode()
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
