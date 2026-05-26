# Team KAI - ROS2 Starter Project (`track_drive`)

2026 국민대학교 자율주행 경진대회 예선을 준비하기 위한 Team KAI의 ROS2 Humble 파이썬 스타터 프로젝트입니다.

> 이 저장소는 **초기 개발 뼈대(starter code)** 목적이며, 미션 로직/인지 알고리즘은 TODO 중심의 기본 구현입니다.

## 목표

- ROS2 패키지 이름: `track_drive`
- 실행 명령:

```bash
ros2 run track_drive track_drive
```

- 메인 자율주행 소스 파일: `track_drive.py`

## 환경

- Ubuntu 22.04
- ROS2 Humble
- (시뮬레이터 연동) 카메라, LiDAR, IMU 토픽 수신 + 모터 토픽 송신

## 토픽 구성

- Subscribe: `/usb_cam/image_raw/front` (`sensor_msgs/msg/Image`)
- Subscribe: `/scan` (`sensor_msgs/msg/LaserScan`)
- Subscribe: `/imu` (`sensor_msgs/msg/Imu`)
- Publish: `/xycar_motor` (`xycar_msgs/msg/XycarMotor`)

## 빌드 및 실행

1. `~/xycar_ws/src` 아래에 이 패키지를 위치시키거나 clone
2. `cd ~/xycar_ws`
3. `colcon build --symlink-install --packages-select track_drive`
4. `source install/setup.bash`
5. 한 터미널에서 ROS-TCP endpoint 실행
6. Windows에서 시뮬레이터 실행
7. `ros2 run track_drive track_drive`

## 미션 순서 (예선 기준)

1. 신호등 출발(초록불)
2. 라바콘 주행
3. 차선 주행
4. 보행자 회피
5. 차량 장애물 회피
6. 스쿨존 감속 제어
7. 좌회전/지름길 분기 판단
8. 3바퀴 완주 후 종료

## 개발 로드맵 (예선 제출 전)

- 주차별 센서 파이프라인 안정화 (카메라/LiDAR/IMU)
- 미션 상태머신 확장 및 전이 조건 수치화
- 제어 파라미터 튜닝(조향/속도 제한 포함)
- 시뮬레이터 반복 주행 로그 분석 자동화
- 제출 산출물 점검 자동화(`scripts/export_submission.py`)

## 팀 역할 (5인)

1. 통합/제출 리드
2. 카메라 인지 리드
3. LiDAR/장애물 리드
4. 제어/상태머신 리드
5. 문서/QA/영상 리드
