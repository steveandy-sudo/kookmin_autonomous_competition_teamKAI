# Team KAI - ROS2 Starter Project (`track_drive`)

2026 국민대학교 자율주행 경진대회 예선을 준비하기 위한 Team KAI의 ROS2 Humble 파이썬 스타터 프로젝트입니다.

> 현재 저장소에는 `track_drive` 주행 코드와 `cone_il` 모방학습 도구 패키지가 함께 들어 있습니다.

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
7. 기본 노드 실행: `ros2 run track_drive track_drive`
8. AI 주행 + 신호등 정지 + 어린이 보호구역 하이브리드 실행:

```bash
ros2 launch track_drive ai_direct_hybrid.launch.py
```

기본 AI 모델 경로는 launch 파일 안의 `model_path` 기본값을 사용합니다. 모델 파일은 Git에 포함하지 않으므로, 다른 PC에서는 같은 경로에 모델을 복사하거나 `model_path:=/path/to/cone_bc_scripted.pt`로 지정하세요.

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

## 추가 패키지

- `cone_il/`: 라바콘 구간 모방학습 데이터 수집, 키보드 조종, 학습, 추론 노드 패키지

모방학습 데이터셋과 학습된 모델 파일은 용량이 커서 git에 포함하지 않습니다.

## 팀 역할 (5인)

1. 통합/제출 리드
2. 카메라 인지 리드
3. LiDAR/장애물 리드
4. 제어/상태머신 리드
5. 문서/QA/영상 리드
