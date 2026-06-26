#!/usr/bin/env python3
import time
from collections import deque
from pathlib import Path
from typing import Deque, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import XycarMotor

from cone_il.preprocess import orange_ratio_bgr, preprocess_image_bgr


class ConeAIDriver(Node):
    """Run a trained imitation-learning model for cone driving.

    The model must be a TorchScript file produced by scripts/train_cone_bc.py.
    This node publishes XycarMotor commands directly, so do not run another
    driver node publishing to the same motor topic at the same time.
    """

    # 설명: 학습된 콘 주행 모델을 불러오고 카메라/라이다 기반 추론 주행 노드를 초기화한다.
    def __init__(self):
        super().__init__('cone_ai_driver')

        self.declare_parameter('image_topic', '/usb_cam/image_raw/front')
        self.declare_parameter('image_qos_depth', 1)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('motor_topic', 'xycar_motor')
        self.declare_parameter('model_path', '')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('speed', 3.0)
        self.declare_parameter('max_steer_deg', 50.0)
        self.declare_parameter('invert_steering', False)
        self.declare_parameter('steer_smoothing', 0.20)
        self.declare_parameter('resize_width', 160)
        self.declare_parameter('resize_height', 90)
        self.declare_parameter('roi_top_ratio', 0.45)
        self.declare_parameter('require_orange_gate', False)
        self.declare_parameter('orange_ratio_threshold', 0.002)
        self.declare_parameter('no_cone_speed', 0.0)
        self.declare_parameter('use_lidar_emergency_stop', True)
        self.declare_parameter('emergency_stop_distance', 0.45)
        self.declare_parameter('emergency_front_half_width', 0.25)
        self.declare_parameter('scan_front_index', -1)
        self.declare_parameter('scan_reverse', False)
        self.declare_parameter('scan_angle_offset', 0.0)
        self.declare_parameter('scan_min_range', 0.05)
        self.declare_parameter('scan_max_range', 8.0)

        self.bridge = CvBridge()
        self.image: Optional[np.ndarray] = None
        self.scan: Optional[LaserScan] = None
        self.prev_steer = 0.0
        self.motor_msg = XycarMotor()
        self.last_motor_subscription_count = -1
        self.last_status_sec = -1
        self.image_receive_times: Deque[float] = deque()
        self.image_stamp_times: Deque[float] = deque()

        model_path = str(Path(str(self.get_parameter('model_path').value)).expanduser())
        if not model_path:
            raise RuntimeError('model_path parameter is empty. Train first and pass model_scripted.pt')

        try:
            import torch
            self.torch = torch
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.model = torch.jit.load(model_path, map_location=self.device)
            self.model.eval()
        except Exception as exc:
            raise RuntimeError(f'failed to load TorchScript model: {model_path} | {exc}')

        image_topic = str(self.get_parameter('image_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        motor_topic = str(self.get_parameter('motor_topic').value)
        image_qos_depth = max(int(self.get_parameter('image_qos_depth').value), 1)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=image_qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Image, image_topic, self.image_callback, image_qos)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.motor_pub = self.create_publisher(XycarMotor, motor_topic, 10)

        rate = max(float(self.get_parameter('control_rate_hz').value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.control_once)

        self.get_logger().info(
            f'Cone AI driver ready | model={model_path}, device={self.device}, image={image_topic}, motor={motor_topic}'
        )

    # 설명: 수신한 ROS 메시지를 내부 최신 상태로 반영한다.
    def image_callback(self, msg: Image):
        try:
            self._record_image_timing(msg)
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warn(f'image conversion failed: {exc}')

    # 설명: 수신한 ROS 메시지를 내부 최신 상태로 반영한다.
    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    # 설명: 현재 센서 상태를 이용해 한 주기의 AI 주행 제어를 수행한다.
    def control_once(self):
        if self.image is None:
            self._log_status_once('waiting_for_image')
            return

        speed = float(self.get_parameter('speed').value)

        if bool(self.get_parameter('require_orange_gate').value):
            ratio = orange_ratio_bgr(
                self.image,
                roi_top_ratio=float(self.get_parameter('roi_top_ratio').value),
            )
            if ratio < float(self.get_parameter('orange_ratio_threshold').value):
                self.drive(self.prev_steer * 0.5, float(self.get_parameter('no_cone_speed').value))
                return

        steer = self.predict_angle(self.image)

        if bool(self.get_parameter('use_lidar_emergency_stop').value):
            nearest = self.nearest_front_obstacle()
            if nearest is not None and nearest < float(self.get_parameter('emergency_stop_distance').value):
                speed = 0.0

        self.prev_steer = steer
        self.drive(steer, speed)
        self._log_status_once(f'cmd angle={steer:.1f}, speed={speed:.1f}')

    # 설명: 전처리된 카메라 이미지로 학습 모델의 조향각을 예측한다.
    def predict_angle(self, image_bgr: np.ndarray) -> float:
        tensor_np = preprocess_image_bgr(
            image_bgr,
            roi_top_ratio=float(self.get_parameter('roi_top_ratio').value),
            width=int(self.get_parameter('resize_width').value),
            height=int(self.get_parameter('resize_height').value),
        )
        x = self.torch.from_numpy(tensor_np).unsqueeze(0).to(self.device)
        with self.torch.no_grad():
            pred_norm = float(self.model(x).detach().cpu().numpy().reshape(-1)[0])

        max_steer = float(self.get_parameter('max_steer_deg').value)
        steer = float(np.clip(pred_norm * max_steer, -max_steer, max_steer))
        if bool(self.get_parameter('invert_steering').value):
            steer = -steer

        smoothing = float(np.clip(self.get_parameter('steer_smoothing').value, 0.0, 0.95))
        steer = (1.0 - smoothing) * steer + smoothing * self.prev_steer
        return steer

    # 설명: 전방 라이다에서 가장 가까운 장애물 거리를 계산한다.
    def nearest_front_obstacle(self) -> Optional[float]:
        msg = self.scan
        if msg is None or not msg.ranges:
            return None

        min_range = float(self.get_parameter('scan_min_range').value)
        max_range = float(self.get_parameter('scan_max_range').value)
        half_width = float(self.get_parameter('emergency_front_half_width').value)
        front_index_param = int(self.get_parameter('scan_front_index').value)
        front_index = front_index_param if front_index_param >= 0 else len(msg.ranges) // 2
        scan_reverse = bool(self.get_parameter('scan_reverse').value)
        angle_offset = float(self.get_parameter('scan_angle_offset').value)
        angle_step = abs(float(msg.angle_increment)) if abs(float(msg.angle_increment)) > 1e-6 else np.deg2rad(1.0)
        scan_dir = -1.0 if scan_reverse else 1.0

        nearest = None
        for idx, r in enumerate(msg.ranges):
            r = float(r)
            if not np.isfinite(r) or r < min_range or r > max_range:
                continue
            angle = scan_dir * (idx - front_index) * angle_step + angle_offset
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            if x <= 0.05 or abs(y) > half_width:
                continue
            if nearest is None or x < nearest:
                nearest = float(x)
        return nearest

    # 설명: 계산된 조향각과 속도를 차량 모터 명령으로 발행한다.
    def drive(self, angle: float, speed: float):
        if not rclpy.ok():
            return

        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        try:
            self.motor_pub.publish(self.motor_msg)
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().warn(f'motor publish failed: {exc}')

        try:
            count = self.motor_pub.get_subscription_count()
        except Exception:
            return
        if count != self.last_motor_subscription_count:
            self.last_motor_subscription_count = count
            self.get_logger().info(f'{self.get_parameter("motor_topic").value} subscribers={count}')

    # 설명: 상태 로그가 너무 자주 출력되지 않도록 같은 내용을 한 번만 기록한다.
    def _log_status_once(self, text: str):
        now_sec = self.get_clock().now().nanoseconds // 1_000_000_000
        if now_sec == self.last_status_sec:
            return
        self.last_status_sec = now_sec
        self.get_logger().info(f'{text}{self._image_hz_log_text()}')

    # 설명: 이미지 수신 시각을 기록해 카메라 FPS 계산에 사용한다.
    def _record_image_timing(self, msg: Image):
        now = time.monotonic()
        self.image_receive_times.append(now)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            self.image_stamp_times.append(float(stamp))
        self._trim_timing_window(self.image_receive_times, now)
        if self.image_stamp_times:
            self._trim_timing_window(self.image_stamp_times, self.image_stamp_times[-1])

    # 설명: FPS 계산용 시간 기록에서 오래된 값을 제거한다.
    @staticmethod
    def _trim_timing_window(times: Deque[float], now: float, window_sec: float = 3.0):
        while len(times) > 1 and now - times[0] > window_sec:
            times.popleft()

    # 설명: 최근 수신 시각 목록으로 주파수를 계산한다.
    @staticmethod
    def _hz_from_times(times: Deque[float]) -> Optional[float]:
        if len(times) < 2:
            return None
        duration = times[-1] - times[0]
        if duration <= 1e-6:
            return None
        return (len(times) - 1) / duration

    # 설명: 현재 모듈 상태를 로그에 넣기 좋은 짧은 문자열로 만든다.
    def _image_hz_log_text(self) -> str:
        receive_hz = self._hz_from_times(self.image_receive_times)
        stamp_hz = self._hz_from_times(self.image_stamp_times)
        receive_text = 'n/a' if receive_hz is None else f'{receive_hz:.1f}'
        stamp_text = 'n/a' if stamp_hz is None else f'{stamp_hz:.1f}'
        return f' image_hz recv={receive_text} stamp={stamp_text}'


# 설명: ROS 노드나 스크립트 실행을 시작하는 진입점이다.
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ConeAIDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.drive(0.0, 0.0)
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
