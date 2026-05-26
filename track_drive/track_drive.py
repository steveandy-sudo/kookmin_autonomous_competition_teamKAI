"""Team KAI 자율주행 메인 노드(스타터 코드)."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from xycar_msgs.msg import XycarMotor

from track_drive.control import (
    clamp_speed,
    clamp_steering,
    compute_lane_center_steering,
    select_speed_by_state,
)
from track_drive.lidar_utils import (
    detect_pedestrian_vehicle_obstacles,
    get_front_obstacle_distance,
    get_left_right_obstacle_distance,
)
from track_drive.mission_state import MissionStateMachine
from track_drive.perception import PerceptionModule
from track_drive.utils import log_debug, log_info


class TrackDriveNode(Node):
    """센서 입력을 받아 조향/속도를 계산해 모터 명령을 발행하는 노드."""

    def __init__(self) -> None:
        super().__init__('track_drive_node')

        # 센서 최신값만 저장해 메인 루프에서 사용한다.
        self.latest_image: Optional[Image] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_imu: Optional[Imu] = None

        self.perception = PerceptionModule()
        self.mission_sm = MissionStateMachine()

        # 시뮬레이터 토픽 연결
        self.create_subscription(Image, '/usb_cam/image_raw/front', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)

        # 약 20Hz 주기 메인 루프
        self.timer = self.create_timer(0.05, self.timer_callback)
        log_info(self, 'TrackDriveNode started (starter mode).')

    def image_callback(self, msg: Image) -> None:
        """카메라 콜백: 최신 프레임만 저장."""
        self.latest_image = msg

    def scan_callback(self, msg: LaserScan) -> None:
        """LiDAR 콜백: 최신 스캔만 저장."""
        self.latest_scan = msg

    def imu_callback(self, msg: Imu) -> None:
        """IMU 콜백: 최신 데이터만 저장."""
        self.latest_imu = msg

    def timer_callback(self) -> None:
        """메인 제어 루프: 인지 -> 상태판단 -> 제어 -> 모터 발행."""
        # 1) Perception
        perception_result = self.perception.run(self.latest_image)

        # 2) LiDAR 기반 장애물 정보 추출
        obstacle_result = detect_pedestrian_vehicle_obstacles(self.latest_scan)
        obstacle_result['front_distance'] = get_front_obstacle_distance(self.latest_scan)
        obstacle_result.update(get_left_right_obstacle_distance(self.latest_scan))

        # 3) 미션 상태 업데이트
        state = self.mission_sm.update(perception_result, obstacle_result)

        # 4) 제어값 계산
        steering = compute_lane_center_steering(
            lane_center_x=perception_result.get('lane_center_x'),
            image_width=perception_result.get('image_width'),
        )
        speed = select_speed_by_state(state)

        # 전방 장애물이 매우 가까우면 일단 정지하는 안전 로직
        if obstacle_result.get('front_distance', float('inf')) < 0.8:
            speed = 0.0

        # 5) 안전 제한 적용
        steering = clamp_steering(steering)
        speed = clamp_speed(speed)

        # 6) 모터 명령 발행
        motor_msg = XycarMotor()
        motor_msg.angle = float(steering)
        motor_msg.speed = float(speed)
        self.motor_pub.publish(motor_msg)

        log_debug(
            self,
            f'state={state.name} steering={motor_msg.angle:.2f} speed={motor_msg.speed:.2f}',
        )


def main(args: Optional[list[str]] = None) -> None:
    """노드 실행 진입점."""
    rclpy.init(args=args)
    node = TrackDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
