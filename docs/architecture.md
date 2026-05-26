# ROS2 아키텍처 개요

## 1) 노드 구조

- 단일 실행 노드: `TrackDriveNode` (`track_drive/track_drive.py`)
- 내부 모듈 분리:
  - `perception.py`: 카메라 기반 인지(초록불/차선 중심/placeholder 탐지)
  - `lidar_utils.py`: LiDAR 전처리 및 장애물 거리/감지 helper
  - `mission_state.py`: 미션 상태머신 스켈레톤
  - `control.py`: 조향/속도 기본 제어
  - `utils.py`: clamp, 안전 숫자 처리, 로깅 helper

## 2) 토픽 플로우

- 입력
  - `/usb_cam/image_raw/front` -> perception
  - `/scan` -> lidar_utils
  - `/imu` -> 현재는 저장만 수행, 향후 자세 안정화에 사용 예정
- 출력
  - `/xycar_motor` (`xycar_msgs/msg/XycarMotor`)

## 3) 실행 루프

- 20Hz 타이머 루프에서 다음 순서로 처리
  1. 최신 센서 데이터 snapshot 사용
  2. perception 계산
  3. 미션 상태 갱신
  4. 제어값 계산 (조향/속도)
  5. 안전 제한 적용 후 모터 명령 발행

## 4) 모듈 책임

- perception: 영상 기반 신호/차선/표지 탐지의 진입점
- lidar_utils: 거리 계산 및 장애물 탐지의 공통 로직
- mission_state: 대회 미션 흐름 제어
- control: 상태별 목표 속도 및 차선 중심 조향
- track_drive: ROS2 인터페이스 및 전체 파이프라인 orchestration
