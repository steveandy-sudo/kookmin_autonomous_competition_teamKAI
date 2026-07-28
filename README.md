# kookmin_autonomous_competition_teamKAI — `main`

ROS 2 Humble 기반 Xycar 자율주행 프로젝트다. 현재 코드는 다음 기능을 하나의 주행 관리 노드로 통합한다.

- LR-ASPP 차선 인지와 차선 추종
- LiDAR 기반 라바콘 주행
- YOLO 기반 신호등·라바콘·장애물 확인
- 정적/동적 장애물 회피
- ROS 2 VESC 구동

- 미니 PC 워크스페이스: `/home/xytron/xycar_ws`
- ROS 배포판: ROS 2 Humble

## 1. 프로젝트 개요 및 저장소 구조

```text
kookmin_autonomous_competition_teamKAI/
├── README.md
└── xycar_ws/
    └── src/
        ├── study/
        │   ├── my_rule/             # 통합 주행 로직
        │   └── my_rule_msgs/        # 전용 ROS 2 메시지
        ├── xycar_application/
        │   ├── wide_camera/         # MJPEG 카메라
        │   └── xycar_perception/    # BEV·카메라 보정
        ├── xycar_device/
        │   ├── xycar_lidar/
        │   ├── xycar_msgs/
        │   ├── xycar_ultrasonic/
        │   └── xycar_vesc_driver/
        └── yolo_ros/
```

주요 노드:

| 노드 | 역할 |
| --- | --- |
| `wide_camera` | `1280x1024` MJPEG 영상 발행 |
| `my_rule_lane_perception_node` | LR-ASPP 차선 분할과 Canonical BEV 생성 |
| `my_rule_object_detection_node` | 저주기 YOLO 객체 판단 |
| `my_rule_cone_node` | LiDAR 라바콘 경로 생성 |
| `my_rule_drive_manager` | 미션 판단과 최종 모터 명령 |
| `xycar_vesc_driver` | VESC 직렬 구동과 상태 발행 |
| `my_rule_perception_view` | 선택형 2×2 디버깅 화면 |

## 2. 전체 주행 로직과 핵심 원리
- 주행 중 신호등 제어는 아직 비활성 상태다.
- 동적 장애물 추월도 구현만 되어 있고 비활성 상태다.

```
- 카메라
  ├─ LR-ASPP 7 Hz → Canonical 경로 → Stanley + Pure Pursuit   ┐
  └─ YOLO 3 Hz → 신호등·cone·car·yellow_centerline 확인         ├ drive_manager
- LiDAR 9.6 Hz → 라바콘 경로·장애물 거리                           │
- 초음파 → 정적 장애물 측면·후방 확인                                ┘
                               ↓
                     /xycar_motor 100 Hz
                               ↓
                     ROS 2 VESC driver
```


## 3. LR-ASPP 차선 인지·경로 생성·차선 추종

- `yellow_centerline` YOLO는 경로를 직접 만들지 않는다.
- 이는 LR-ASPP의 잘못된 노란 후보를 제거할 때만 보조한다.

1. `my_rule_lane.pt`에 `256x144` 영상을 입력한다.
2. 흰 차선과 노란 중앙선을 분할한다.
3. 실차 ROI를 BEV와 Canonical 영상으로 변환한다.
4. 흰 차선은 곡선 피팅으로 연결한다.
5. 노란 점선은 허용 간격 안에서 연결한다.
6. 노란 중앙선 오른쪽 `0.10 m`를 기본 목표로 사용한다.
7. Stanley와 Pure Pursuit를 결합해 조향한다.


현재 차선 속도:

| 상태 | 속도 명령 |
| --- | ---: |
| 직선 최고 | 8.0 |
| 곡선 최저 | 6.0 |
| 차선 완전 손실 | 4.0 |

차선 경로는 최대 `7 Hz`로 갱신된다.
제어 지연을 고려해 약 `0.30초` 앞을 예측한다.

## 4. LiDAR 기반 라바콘 경로 생성 및 주행

1. 전방 `-70~70도`, `0.18~1.6 m` 점을 사용한다.
2. DBSCAN으로 라바콘 군집을 만든다.
3. 좌우 군집을 연결해 gate를 만든다.
4. gate 중점을 중앙 경로로 사용한다.
5. Pure Pursuit로 조향한다.
6. 경로와 조향량에 따라 속도를 정한다.

현재 주요 값:

| 항목 | 값 |
| --- | ---: |
| 예상 통로 폭 | 0.85 m |
| 일반 최고속도 | 17.0 |
| 최소속도 | 9.0 |
| 한쪽 경계 상한 | 9.5 |
| 직선 부스트 | 21.0 |
| 비상정지 거리 | 0.35 m |

라바콘 진입은 YOLO와 LiDAR가 함께 확인해야 한다.
종료 후 `0.65초` 동안 라바콘 조향과 차선 조향을 혼합한다.

## 5. 라바콘 전용 주행 실행 매뉴얼

- `cone_only.launch.py`는 라바콘 주행만 수행한다.
- 차선주행, 신호등, 장애물 회피는 실행하지 않는다.
- YOLO는 라바콘 진입 확인에만 사용한다.
- 차량을 첫 라바콘 약 `1 m` 앞에 두고 실행해야 한다.


### 터미널 1 (주행 명령어)

- 차량을 첫 라바콘 약 `1 m` 앞에 두고 실행해야 한다.
- 라바콘 구간이 끝나도 차선 주행을 실시하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash

ros2 launch my_rule cone_only.launch.py \
  motor_drive_enabled:=true
```

### 터미널 2 (확인용 2×2 화면)

- 주행 상태를 확인하고 싶을 때, 입력한다.
- 주행과 관련되어 hz에 영향을 줄 수 있으므로, 가능하면 실행하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash

ros2 launch my_rule perception_view.launch.py \
  mode:=auto \
  view_rate_hz:=5.0
```

| 위치 | 화면 |
| --- | --- |
| 좌측 상단 | 카메라 + LiDAR 투영 |
| 우측 상단 | 차선 또는 라바콘 BEV |
| 좌측 하단 | 카메라 + ROI + YOLO |
| 우측 하단 | 추종 경로 + 조향·속도 |


## 6. 차선·라바콘 통합 주행 실행 매뉴얼

`integrated_drive.launch.py`는 현재 활성 기능을 모두 실행한다.

- 최초 초록불 대기
- 차선 주행
- 라바콘 주행
- 정적 장애물 회피

### 터미널 1 (주행 명령어)

- 통합 주행은 반드시 시작 신호등 밑에서 출발해야 한다.
- 차선 중간에 두면, 신호등을 확인 못하기에 작동하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash

ros2 launch my_rule integrated_drive.launch.py \
  motor_drive_enabled:=true
```

### 터미널 2 (확인용 2×2 화면)

- 주행 상태를 확인하고 싶을 때, 입력한다.
- 주행과 관련되어 hz에 영향을 줄 수 있으므로, 가능하면 실행하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash

ros2 launch my_rule perception_view.launch.py \
  mode:=auto \
  view_rate_hz:=5.0
```

| 위치 | 화면 |
| --- | --- |
| 좌측 상단 | 카메라 + LiDAR 투영 |
| 우측 상단 | 차선 또는 라바콘 BEV |
| 좌측 하단 | 카메라 + ROI + YOLO |
| 우측 하단 | 추종 경로 + 조향·속도 |


## 7. 주요 설정값·실측 주기 및 안전 주의사항

| 항목 | 실측 주기 |
| --- | ---: |
| 광각 카메라 | 약 27 Hz |
| LiDAR | 약 9.65 Hz |
| 차선 경로·명령 | 약 7.1 Hz |
| 객체 YOLO | 약 3.0 Hz |
| 라바콘 명령 | 약 9.66 Hz |
| 최종 모터 명령 | 약 100~106 Hz |
| VESC 상태 | 약 50 Hz |

주요 설정:

- 차선 인지: `config/lane_perception.yaml`
- 차선 제어: `config/lane_control.yaml`
- 라바콘: `config/cone_control.yaml`
- 미션: `config/drive_manager.yaml`
- 객체 YOLO: `config/object_detection.yaml`
- 디버깅 화면: `config/perception_view.yaml`

## 8. 현재 AI 모델과 향후 작업

### 차선 모델

- 파일: `models/my_rule_lane.pt`
- 형식: TorchScript LR-ASPP
- 클래스: `background`, `white`, `yellow`
- 역할: 차선 경로의 주 인지

### 객체 모델

- 파일: `models/my_rule_objects.pt`
- 형식: Ultralytics YOLO detection
- 실행 주기: `3 Hz`

| 클래스 | 사용 방식 |
| --- | --- |
| `cone` | 라바콘 진입 확인 |
| `car` | 정적 장애물 확인 |
| `red`, `green` | 최초 출발 신호 |
| `yellow` | 아직은 진단만 수행 |
| `yellow_centerline` | 중앙선 후보 보조 |
| `obstacle_vehicle` | 동적 추월용 클래스 (아직 모델링 안됨) |

## 9. 향후 작업

0. 차선 주행 속도 높이기 + 라바콘 주행 속도 최적화
1. 정적 장애물 회피 실차 검증
2. `obstacle_vehicle` 모델 학습 (동적 장애물 클래스 추가)
3. 동적 장애물 회피 실차 검증
4. 주행 중 신호등 빨간불에 따른 정지 위치 조정
5. 지름길로의 좌회전 로직 구현
6. SLAM에서의 경로를 전역경로로 한 로직 통합
