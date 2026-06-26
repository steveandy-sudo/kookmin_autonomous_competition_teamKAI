#!/usr/bin/env python3
# 전방 카메라 이미지를 CNN 조향 모델에 넣어 조향각을 예측하는 ROS2 노드이다.
# 라바콘 구간에서 모델이 예측한 조향각과 설정 속도를 /xycar_motor 명령으로 변환한다.
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

from .camera_preprocess import orange_ratio_bgr, preprocess_image_bgr


# CNN End-to-End 모델만으로 기본 차선/라바콘 주행 명령을 생성하는 노드이다.
class ConeAIDriver(Node):
    """전방 카메라 이미지를 CNN 모델에 입력해 조향각을 예측하는 노드이다."""

    def __init__(self):
        # 모델 경로, 카메라/라이다 토픽, 제어 주기, 조향 제한값을 ROS 파라미터로 준비한다.
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

        # TorchScript 모델은 제출 폴더에 포함된 .pt 파일을 그대로 불러와 실시간 조향 추론에 사용한다.
        try:
            import torch
            self.torch = torch
            # CUDA가 있으면 GPU로 추론하고, 없으면 CPU로 동작해 심사 환경 차이를 흡수한다.
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.model = torch.jit.load(model_path, map_location=self.device)
            self.model.eval()
        except Exception as exc:
            raise RuntimeError(f'failed to load TorchScript model: {model_path} | {exc}')

        image_topic = str(self.get_parameter('image_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        motor_topic = str(self.get_parameter('motor_topic').value)
        image_qos_depth = max(int(self.get_parameter('image_qos_depth').value), 1)
        # 카메라 이미지는 유실되면 조향 반응이 흔들리므로 RELIABLE QoS와 제한된 depth를 사용한다.
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

    # 최신 카메라 프레임을 저장하고 수신 주기를 기록한다.
    def image_callback(self, msg: Image):
        try:
            # 수신 주기와 header stamp 주기를 기록해 카메라 토픽 지연 여부를 로그에서 확인할 수 있게 한다.
            self._record_image_timing(msg)
            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().warn(f'image conversion failed: {exc}')

    # 라이다 값은 비상 정지나 전방 장애물 거리 확인에 사용한다.
    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    # 주기적으로 최신 이미지에서 조향을 예측하고 속도 제한/비상정지를 적용한다.
    def control_once(self):
        # 최신 이미지가 있으면 모델 추론으로 조향각을 계산하고, 라이다 비상 정지 조건을 함께 반영한다.
        if self.image is None:
            self._log_status_once('waiting_for_image')
            return

        # 기본 목표 속도는 launch 파라미터에서 받고, 아래 조건들이 필요하면 속도를 줄인다.
        speed = float(self.get_parameter('speed').value)

        # 라바콘 색상 게이트를 사용할 때는 주황색 비율이 낮으면 AI 주행 명령을 약하게 만든다.
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
            # 라이다 전방 가까운 장애물은 CNN 조향과 별도로 비상 정지 조건으로 처리한다.
            nearest = self.nearest_front_obstacle()
            if nearest is not None and nearest < float(self.get_parameter('emergency_stop_distance').value):
                speed = 0.0

        self.prev_steer = steer
        self.drive(steer, speed)
        self._log_status_once(f'cmd angle={steer:.1f}, speed={speed:.1f}')

    # OpenCV 이미지를 Torch 텐서로 바꾼 뒤 CNN 모델의 조향 예측값을 degree 단위로 반환한다.
    def predict_angle(self, image_bgr: np.ndarray) -> float:
        # 카메라 이미지를 CNN 입력 텐서로 변환한 뒤 정규화된 출력값을 실제 조향각으로 바꾼다.
        # 전처리 결과는 batch 차원을 추가해 모델 입력 형태인 NCHW 텐서로 만든다.
        tensor_np = preprocess_image_bgr(
            image_bgr,
            roi_top_ratio=float(self.get_parameter('roi_top_ratio').value),
            width=int(self.get_parameter('resize_width').value),
            height=int(self.get_parameter('resize_height').value),
        )
        x = self.torch.from_numpy(tensor_np).unsqueeze(0).to(self.device)
        # 주행 중에는 학습이 필요 없으므로 gradient 계산을 끄고 추론 지연을 줄인다.
        with self.torch.no_grad():
            pred_norm = float(self.model(x).detach().cpu().numpy().reshape(-1)[0])

        # 모델 출력이 과도하게 튀어도 실제 조향 명령은 차량 한계 안으로 제한한다.
        max_steer = float(self.get_parameter('max_steer_deg').value)
        steer = float(np.clip(pred_norm * max_steer, -max_steer, max_steer))
        # 학습 데이터의 좌우 부호 체계가 시뮬레이터 조향 부호와 다를 때를 대비한 보정 옵션이다.
        if bool(self.get_parameter('invert_steering').value):
            steer = -steer

        # 조향 smoothing은 프레임별 예측 흔들림을 줄이되 반응 지연이 너무 커지지 않도록 제한한다.
        smoothing = float(np.clip(self.get_parameter('steer_smoothing').value, 0.0, 0.95))
        steer = (1.0 - smoothing) * steer + smoothing * self.prev_steer
        return steer

    # 전방 라이다 빔 중 유효한 최단 거리를 찾아 충돌 위험을 판단한다.
    def nearest_front_obstacle(self) -> Optional[float]:
        # 라이다 전방 영역에서 가장 가까운 장애물 거리를 계산해 비상 정지 판단에 사용한다.
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
        # 전방 폭 안에 들어오는 라이다 빔만 사용해 바로 앞 충돌 위험을 계산한다.
        for idx, r in enumerate(msg.ranges):
            r = float(r)
            # inf/NaN 또는 센서 유효 범위 밖의 값은 실제 장애물 거리로 쓰지 않는다.
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

    # CNN 예측 조향과 목표 속도를 실제 차량 명령 메시지로 발행한다.
    def drive(self, angle: float, speed: float):
        # 예측 조향각과 속도를 XycarMotor 메시지로 발행한다.
        if not rclpy.ok():
            return

        # 최종 조향각과 속도는 xycar_msgs/XycarMotor 형식으로 시뮬레이터에 전달된다.
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

    def _log_status_once(self, text: str):
        now_sec = self.get_clock().now().nanoseconds // 1_000_000_000
        if now_sec == self.last_status_sec:
            return
        self.last_status_sec = now_sec
        self.get_logger().info(f'{text}{self._image_hz_log_text()}')

    # receive 시간과 header stamp 시간을 함께 기록해 카메라 수신 Hz를 추정한다.
    def _record_image_timing(self, msg: Image):
        now = time.monotonic()
        self.image_receive_times.append(now)
        # header stamp 기준 주기는 시뮬레이터가 실제로 발행한 이미지 시간 간격을 확인하는 데 사용한다.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            self.image_stamp_times.append(float(stamp))
        self._trim_timing_window(self.image_receive_times, now)
        if self.image_stamp_times:
            self._trim_timing_window(self.image_stamp_times, self.image_stamp_times[-1])

    @staticmethod
    def _trim_timing_window(times: Deque[float], now: float, window_sec: float = 3.0):
        while len(times) > 1 and now - times[0] > window_sec:
            times.popleft()

    @staticmethod
    def _hz_from_times(times: Deque[float]) -> Optional[float]:
        if len(times) < 2:
            return None
        duration = times[-1] - times[0]
        if duration <= 1e-6:
            return None
        return (len(times) - 1) / duration

    def _image_hz_log_text(self) -> str:
        receive_hz = self._hz_from_times(self.image_receive_times)
        stamp_hz = self._hz_from_times(self.image_stamp_times)
        receive_text = 'n/a' if receive_hz is None else f'{receive_hz:.1f}'
        stamp_text = 'n/a' if stamp_hz is None else f'{stamp_hz:.1f}'
        return f' image_hz recv={receive_text} stamp={stamp_text}'


def main(args=None):
    # CNN 조향 노드를 ROS2 프로세스로 실행한다.
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
