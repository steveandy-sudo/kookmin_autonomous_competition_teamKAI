#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from typing import Optional, Set

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from track_drive.package_paths import default_yolo_model_path
from track_drive.stop_line_detector import BEVStopLineDetector, declare_stop_line_bev_parameters
from track_drive.traffic_light_detector import (
    TrafficLightResult,
    YoloTrafficLightDetector,
)


class IntersectionDebugNode(Node):
    """Debug stop-line + traffic-light + left-cone intersection decisions."""

    def __init__(self):
        # IntersectionDebugNode 객체를 초기화하고 필요한 파라미터와 내부 상태를 준비한다.
        super().__init__('intersection_debug')

        declare_stop_line_bev_parameters(self)
        self._declare_traffic_light_parameters()
        self._declare_intersection_parameters()

        self.bridge = CvBridge()
        self.stop_line_detector = BEVStopLineDetector(self)
        self.traffic_light_detector = YoloTrafficLightDetector(self)
        self.last_log_sec = 0.0

        camera_topic = str(self.get_parameter('camera_topic').value)
        prefix = str(self.get_parameter('intersection_debug_topic_prefix').value).rstrip('/')
        self.image_pub = self.create_publisher(Image, f'{prefix}/image', 10)
        self.stop_bev_pub = self.create_publisher(Image, f'{prefix}/stop_line_bev', 10)
        self.stop_mask_pub = self.create_publisher(Image, f'{prefix}/stop_line_mask', 10)
        self.state_pub = self.create_publisher(String, f'{prefix}/state', 10)
        self.route_pub = self.create_publisher(String, f'{prefix}/route', 10)
        self.speed_pub = self.create_publisher(Float32, f'{prefix}/suggested_speed', 10)
        self.steer_pub = self.create_publisher(Float32, f'{prefix}/suggested_steer', 10)

        self.create_subscription(Image, camera_topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Intersection debug ready | image={camera_topic}, debug={prefix}/image'
        )

    def _declare_intersection_parameters(self):
        # 교차로 디버그 노드의 declare 교차로 parameters 로직을 수행한다.
        self.declare_parameter('intersection_debug_topic_prefix', '/track_drive/intersection_debug')
        self.declare_parameter('intersection_debug_log_period_sec', 0.5)
        self.declare_parameter('intersection_left_cone_min_count', 1)
        self.declare_parameter('intersection_camera_cone_class_ids', [0])
        self.declare_parameter('intersection_camera_cone_min_score', 0.30)
        self.declare_parameter('intersection_camera_cone_left_min_ratio', 0.00)
        self.declare_parameter('intersection_camera_cone_left_max_ratio', 0.68)
        self.declare_parameter('intersection_camera_cone_min_height_ratio', 0.012)
        self.declare_parameter('intersection_camera_cone_min_bottom_ratio', 0.12)
        self.declare_parameter('intersection_left_turn_speed', 5.0)
        self.declare_parameter('intersection_left_turn_steer_deg', -45.0)
        self.declare_parameter('intersection_straight_speed', 5.0)
        self.declare_parameter('intersection_straight_steer_deg', 0.0)

    def _declare_traffic_light_parameters(self):
        # 교차로 디버그 노드의 declare 교통 신호등 parameters 로직을 수행한다.
        self.declare_parameter('traffic_light_debug_topic', '/track_drive/traffic_light_debug/image')
        self.declare_parameter('traffic_light_state_topic', '/track_drive/traffic_light_debug/state')
        self.declare_parameter('traffic_light_log_period_sec', 0.5)
        self.declare_parameter('yolo_light_model_path', default_yolo_model_path())
        self.declare_parameter('yolo_light_input_size', 640)
        self.declare_parameter('yolo_light_class_count', 6)
        self.declare_parameter('yolo_dnn_backend', 'auto')
        self.declare_parameter('yolo_dnn_target', 'auto')
        self.declare_parameter('yolo_light_conf_threshold', 0.35)
        self.declare_parameter('yolo_stop_light_conf_threshold', 0.55)
        self.declare_parameter('yolo_left_light_conf_threshold', 0.28)
        self.declare_parameter('yolo_light_class_ids', [0, 1, 2, 3, 4, 5])
        self.declare_parameter('yolo_red_light_class_ids', [4, 5])
        self.declare_parameter('yolo_go_light_class_ids', [1])
        self.declare_parameter('yolo_left_light_class_ids', [2])
        self.declare_parameter('yolo_nms_threshold', 0.45)
        self.declare_parameter('yolo_light_min_box_height_ratio', 0.025)
        self.declare_parameter('yolo_light_min_box_width_ratio', 0.015)
        self.declare_parameter('yolo_light_max_box_height_ratio', 0.65)
        self.declare_parameter('yolo_light_min_box_area_ratio', 0.00012)
        self.declare_parameter('yolo_light_max_box_area_ratio', 0.20)
        self.declare_parameter('yolo_light_max_box_bottom_ratio', 0.98)
        self.declare_parameter('red_light_min_area', 12.0)
        self.declare_parameter('red_light_min_ratio', 0.0012)
        self.declare_parameter('red_light_min_dominance', 1.20)
        self.declare_parameter('red_light_min_circularity', 0.10)

    def image_callback(self, msg: Image):
        # ROS 토픽 콜백으로 들어온 메시지를 내부 상태에 반영한다.
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'image conversion failed: {exc}')
            return

        stop_result = self.stop_line_detector.detect(frame)
        light_result = self.traffic_light_detector.detect(frame)
        left_cone_count = self._left_cone_count(frame, light_result)
        route = self._route_from_left_cones(left_cone_count)
        light_flags = self._light_flags(light_result)
        state, speed, steer = self._decision(stop_result.detected, route, light_flags)
        debug = self._draw_debug(frame, stop_result, light_result, left_cone_count, route, state, speed, steer)

        self._publish_string(self.state_pub, state)
        self._publish_string(self.route_pub, route)
        self._publish_float(self.speed_pub, speed)
        self._publish_float(self.steer_pub, steer)
        self._publish_image(self.image_pub, debug, 'bgr8', msg)
        self._publish_image(self.stop_bev_pub, stop_result.bev_image, 'bgr8', msg)
        self._publish_image(self.stop_mask_pub, stop_result.mask_image, 'mono8', msg)
        self._log_result(stop_result.detected, light_flags, left_cone_count, route, state, speed, steer)

    def _route_from_left_cones(self, left_cone_count: int) -> str:
        # 교차로 디버그 노드의 경로 선택 from 왼쪽/좌회전 콘 로직을 수행한다.
        min_count = max(int(self.get_parameter('intersection_left_cone_min_count').value), 1)
        # Current course rule: left-side cone present blocks the left route.
        return 'straight' if left_cone_count >= min_count else 'left'

    def _decision(self, stop_line_ready: bool, route: str, flags):
        # 교차로 디버그 노드의 decision 로직을 수행한다.
        red, green, left, yellow = flags
        if not stop_line_ready:
            return 'APPROACH_STOP_LINE', 0.0, 0.0
        if red or yellow:
            return 'STOP_RED_LIGHT', 0.0, 0.0
        if route == 'left':
            if left:
                return (
                    'GO_LEFT',
                    max(float(self.get_parameter('intersection_left_turn_speed').value), 0.0),
                    float(self.get_parameter('intersection_left_turn_steer_deg').value),
                )
            return 'WAIT_LEFT_SIGNAL', 0.0, 0.0
        if green:
            return (
                'GO_STRAIGHT',
                max(float(self.get_parameter('intersection_straight_speed').value), 0.0),
                float(self.get_parameter('intersection_straight_steer_deg').value),
            )
        return 'WAIT_GREEN_LIGHT', 0.0, 0.0

    def _left_cone_count(self, image: np.ndarray, light_result: TrafficLightResult) -> int:
        # 교차로 디버그 노드의 왼쪽/좌회전 콘 count 로직을 수행한다.
        height, width = image.shape[:2]
        class_ids = self._int_set_parameter('intersection_camera_cone_class_ids')
        min_score = max(float(self.get_parameter('intersection_camera_cone_min_score').value), 0.0)
        left_min_ratio = float(np.clip(
            self.get_parameter('intersection_camera_cone_left_min_ratio').value, 0.0, 1.0))
        left_max_ratio = float(np.clip(
            self.get_parameter('intersection_camera_cone_left_max_ratio').value,
            left_min_ratio,
            1.0,
        ))
        min_height_ratio = float(np.clip(
            self.get_parameter('intersection_camera_cone_min_height_ratio').value, 0.0, 1.0))
        min_bottom_ratio = float(np.clip(
            self.get_parameter('intersection_camera_cone_min_bottom_ratio').value, 0.0, 1.0))

        count = 0
        for detection in light_result.detections:
            if not self._class_id_allowed(detection.class_id, class_ids):
                continue
            if detection.score < min_score:
                continue
            x0, y0, x1, y1 = detection.box
            box_height = max(int(y1) - int(y0), 0)
            center_ratio = (0.5 * (float(x0) + float(x1))) / float(max(width, 1))
            bottom_ratio = float(y1) / float(max(height, 1))
            if box_height < min_height_ratio * height:
                continue
            if bottom_ratio < min_bottom_ratio:
                continue
            if left_min_ratio <= center_ratio <= left_max_ratio:
                count += 1
        return count

    def _light_flags(self, light_result: TrafficLightResult):
        # 교차로 디버그 노드의 신호등 flags 로직을 수행한다.
        red = any(item.red_present for item in light_result.detections)
        green = any(
            item.valid and self._class_id_allowed(item.class_id, self._int_set_parameter('yolo_go_light_class_ids'))
            for item in light_result.detections
        )
        left_ids = self._int_set_parameter('yolo_left_light_class_ids') or set()
        left_threshold = max(float(self.get_parameter('yolo_left_light_conf_threshold').value), 0.0)
        left = any(
            item.valid
            and item.score >= left_threshold
            and self._class_id_allowed(item.class_id, left_ids)
            for item in light_result.detections
        )
        if not left:
            left = any(
                0 <= int(class_id) < len(light_result.class_scores)
                and float(light_result.class_scores[int(class_id)]) >= left_threshold
                for class_id in left_ids
            )
        yellow = any(item.valid and int(item.class_id) == 5 for item in light_result.detections)
        return red, green, left, yellow

    def _draw_debug(
        self,
        image: np.ndarray,
        stop_result,
        light_result: TrafficLightResult,
        left_cone_count: int,
        route: str,
        state: str,
        speed: float,
        steer: float,
    ) -> np.ndarray:
        # draw 디버그 정보를 디버그 이미지 위에 그린다.
        debug = light_result.debug_image.copy() if light_result.debug_image is not None else image.copy()
        height, width = debug.shape[:2]
        left_min = float(np.clip(self.get_parameter('intersection_camera_cone_left_min_ratio').value, 0.0, 1.0))
        left_max = float(np.clip(
            self.get_parameter('intersection_camera_cone_left_max_ratio').value,
            left_min,
            1.0,
        ))
        x0 = int(left_min * width)
        x1 = int(left_max * width)
        cv2.rectangle(debug, (x0, 0), (x1, height - 1), (255, 180, 0), 2)

        color = (0, 220, 0) if state.startswith('GO') else ((0, 0, 255) if state.startswith('STOP') else (0, 180, 255))
        cv2.rectangle(debug, (6, 120), (min(width - 6, 860), 238), (0, 0, 0), thickness=-1)
        cv2.putText(debug, f'INTERSECTION: {state}', (14, 148), cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2)
        cv2.putText(
            debug,
            f'stop_line={int(stop_result.detected)} row={stop_result.row_ratio:.2f} dist={-1.0 if stop_result.distance_m is None else stop_result.distance_m:.2f}m',
            (14, 174),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (230, 230, 230),
            1,
        )
        flags = self._light_flags(light_result)
        cv2.putText(
            debug,
            f'light red={int(flags[0])} green={int(flags[1])} left={int(flags[2])} yellow={int(flags[3])}',
            (14, 198),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (230, 230, 230),
            1,
        )
        cv2.putText(
            debug,
            f'left_cones={left_cone_count} route={route} cmd_speed={speed:.1f} steer={steer:.1f}',
            (14, 222),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (230, 230, 230),
            1,
        )
        return debug

    def _log_result(self, stop_line: bool, flags, left_cone_count: int, route: str, state: str, speed: float, steer: float):
        # 로그 result 정보를 사람이 읽기 쉬운 로그 문자열로 만든다.
        now = time.monotonic()
        period = max(float(self.get_parameter('intersection_debug_log_period_sec').value), 0.0)
        if period > 0.0 and now - self.last_log_sec < period:
            return
        self.last_log_sec = now
        self.get_logger().info(
            f'state={state} route={route} stop_line={int(stop_line)} '
            f'light(red={int(flags[0])},green={int(flags[1])},left={int(flags[2])},yellow={int(flags[3])}) '
            f'left_cones={left_cone_count} cmd=({speed:.1f},{steer:.1f})'
        )

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
    def _publish_string(pub, value: str):
        # publish string 결과를 ROS 토픽이나 디버그 출력으로 발행한다.
        msg = String()
        msg.data = str(value)
        pub.publish(msg)

    @staticmethod
    def _publish_float(pub, value: float):
        # publish float 결과를 ROS 토픽이나 디버그 출력으로 발행한다.
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    def _int_set_parameter(self, name: str) -> Optional[Set[int]]:
        # 리스트형 ROS 파라미터를 정수 집합으로 변환한다.
        value = self.get_parameter(name).value
        if value is None:
            return None
        if isinstance(value, str):
            text = value.strip()
            if not text or text.lower() in ('all', 'any', '*'):
                return None
            value = text.strip('[]()').replace(';', ',').split(',')
        if isinstance(value, np.ndarray):
            value = value.tolist()
        if isinstance(value, (list, tuple)):
            ids = {int(item) for item in value if str(item).strip()}
            return ids or None
        return {int(value)}

    @staticmethod
    def _class_id_allowed(class_id: int, allowed_class_ids: Optional[Set[int]]) -> bool:
        # 검출 클래스 ID가 허용 목록에 포함되는지 확인한다.
        return allowed_class_ids is None or int(class_id) in allowed_class_ids


def main(args=None):
    # ROS2 노드를 초기화하고 실행 루프를 시작한다.
    rclpy.init(args=args)
    node = IntersectionDebugNode()
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
