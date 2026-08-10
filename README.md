# kookmin_autonomous_competition_teamKAI — `main`

ROS 2 Humble 기반 Xycar 룰베이스 자율주행 프로젝트다. 현재 코드는 다음 기능을 하나의 주행 선택기로 통합한다.

- X-bin 차선 인지와 Canonical 경로 생성
- Stanley + Pure Pursuit 차선 추종
- LiDAR 기반 라바콘 주행
- YOLO + LiDAR 기반 장애물 회피
- 4구 신호등 출발·정지와 1회 지름길 선택
- ROS 2 VESC 구동

- 미니 PC 워크스페이스: `/home/xytron/xycar_ws`
- ROS 배포판: ROS 2 Humble

## 1. 프로젝트 개요 및 저장소 구조

```text
kookmin_autonomous_competition_teamKAI/
├── README.md
└── xycar_ws/
    └── src/
        ├── my_lane/                 # X-bin 차선 인지·Canonical 경로
        ├── my_control/              # Stanley + Pure Pursuit 추종
        ├── my_rule/                 # YOLO 객체 인지·LiDAR 라바콘
        ├── my_drive/                # 미션 선택·회피·최종 명령
        ├── my_road/                 # 카메라 보정·BEV 공통 코드
        ├── my_msgs/                 # 차선·경로 메시지
        ├── my_rule_msgs/            # 객체 인지 메시지
        └── xycar_device/
            ├── xycar_camera/
            ├── xycar_lidar/
            ├── xycar_msgs/
            └── xycar_vesc_driver/
```

주요 노드:

| 노드 | 역할 |
| --- | --- |
| `xycar_wide_camera` | `1280x1024`, 30 FPS MJPEG 영상 발행 |
| `lane_seg_lraspp_inference` | X-bin 모델 추론과 차선 마스크 생성 |
| `canonical_stanley_pursuit_driver` | Canonical 경로 추종 명령 생성 |
| `my_rule_object_detection_node` | 저주기 YOLO 객체 판단 |
| `my_rule_cone_node` | LiDAR 라바콘 중앙 경로 생성 |
| `sequential_hybrid_driver` | 차선·라바콘·회피 명령 선택 |
| `traffic_shortcut_gate` | 통합 주행 신호등·지름길 상태 판단 |
| `xycar_vesc_driver` | VESC 직렬 구동과 상태 발행 |

## 2. 전체 주행 로직과 핵심 원리

- `basic_launch.py`는 차선·라바콘·장애물 회피를 실행한다.
- `integrated_launch.py`는 기본 주행에 4구 신호등과 지름길 판단을 추가한다.
- 두 런치는 같은 인지·제어 노드를 사용하므로 차선과 라바콘 처리 주기는 같다.

```text
카메라 1280x1024
├─ X-bin 차선 모델 → Canonical 경로 → Stanley + Pure Pursuit ┐
└─ 객체 YOLO 3 Hz → cone·4구 신호등·차량                  ├─ 주행 선택기
LiDAR → 라바콘 중앙 경로·차량 거리                         ┘
                                  ↓
                         /xycar_motor
                                  ↓
                         ROS 2 VESC driver
```

## 3. 차선 인지·경로 생성·차선 추종

1. `512x288` 영상을 X-bin 차선 모델에 입력한다.
2. 노란 중앙선 위치와 흰 차선 연장 정보를 얻는다.
3. 실차 보정값으로 `256x144` Canonical 경로를 만든다.
4. 직선에서는 Stanley 비중을 높여 조향 진동을 줄인다.
5. 곡선에서는 Pure Pursuit, 지연 미리보기와 곡률 정보를 함께 사용한다.
6. 차선이 짧거나 불완전하면 감속하고 기억 경로를 제한적으로 사용한다.

현재 차선 속도:

| 상태 | 속도 명령 |
| --- | ---: |
| 직선 | 22.0 |
| 곡선 | 14.0 |
| 불완전 경로 | 12.0 |
| 차선 완전 손실 | 4.0 |

차선 인지는 최대 `15 Hz`, 추종 명령은 `10 Hz`, 최종 주행 선택은 `20 Hz`로 동작한다.

## 4. LiDAR 기반 라바콘·장애물 주행

### 라바콘

1. 전방 `-70~70도`, `0.18~1.60 m` LiDAR 점을 사용한다.
2. DBSCAN으로 라바콘 후보 군집을 만든다.
3. YOLO `cone`과 LiDAR 군집으로 진입과 존재 여부를 확인한다.
4. 좌우 경계를 연결하고 유효하지 않은 대응은 제거한다.
5. 양쪽 경계의 중점 또는 한쪽 경계의 가상 반대편으로 중앙 경로를 만든다.
6. 경로 조향을 실차 서보 명령으로 변환한다.

| 항목 | 값 |
| --- | ---: |
| 예상 통로 폭 | 0.85 m |
| 허용 통로 폭 | 0.68~0.98 m |
| 런치 기본 속도 | 10.0 |
| 라바콘 목표 조향 한계 | 26° |
| 최종 서보 명령 한계 | 42 |
| 센서 존재 유지 시간 | 0.50초 |

### 장애물 회피

- `red_car`, `green_car`는 주행 시 공통 차량 장애물로 정규화한다.
- YOLO가 장애물 회피를 활성화하고, LiDAR 군집이 거리와 통과 공간을 확인한다.
- 노란 중앙선과 차선 추종 명령이 안정적인 직선일 때 반대 차선 방향을 고른다.
- 횡방향 목표를 연속적으로 이동하고 회피 중 속도는 최대 `8.0`으로 제한한다.
- 고장 차량 뒤의 단일 라바콘은 라바콘 구간 진입 조건으로 사용하지 않는다.

## 5. 기본·통합 주행 실행 매뉴얼

두 명령 모두 카메라, LiDAR, 객체 인지, 차선 추종, 라바콘, 회피, VESC를 함께 실행한다. 실제 구동 전 `/dev/ttyLIDAR`, `/dev/ttyMOTOR` 연결과 비상 정지 공간을 확인한다.

### 차선 + 라바콘 + 회피 주행

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash
ros2 launch my_drive basic_launch.py motor_drive_enabled:=true
```

### 신호등 + 지름길까지 포함한 통합 주행

- 최초에는 4구 신호등의 초록불을 확인해야 출발한다.
- 주행 중 빨간불을 두 프레임 확인하면 정지하고 초록불을 두 프레임 확인하면 재출발한다.
- 초록불과 `left_4`가 함께 확인되면 대회 전체에서 한 번만 지름길 좌회전을 선택한다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash
ros2 launch my_drive integrated_launch.py motor_drive_enabled:=true
```

## 6. 주요 설정값·주기 및 안전 주의사항

| 항목 | 설정 주기 |
| --- | ---: |
| 광각 카메라 | 30 Hz |
| 차선 인지 상한 | 15 Hz |
| 차선 추종 명령 | 10 Hz |
| 객체 YOLO | 3 Hz |
| 주행 선택·신호 게이트 | 20 Hz |

주요 설정:

- 차선 인지: `my_lane/launch/lane_seg_far_centerline_extended_real.launch.py`
- 차선 제어: `my_control/config/canonical_stanley_pursuit_real.yaml`
- 라바콘: `my_rule/config/cone_control.yaml`
- 객체 YOLO: `my_rule/config/object_detection.yaml`
- 회피·선택기: `my_drive/config/sequential_hybrid_real.yaml`
- 실행 프로필: `my_drive/launch/basic_launch.py`, `my_drive/launch/integrated_launch.py`

`motor_drive_enabled`의 기본값은 `false`다. 실제 주행할 때만 `true`를 명시한다.

## 7. 현재 AI 모델

### 차선 모델

- 파일: `my_lane/models/model_centerline_xbin.pt`
- 입력: `512x288`
- 역할: 노란 중앙선과 흰 차선 연장을 이용한 Canonical 경로 생성
- 오프라인 테스트 가시점 MAE: 약 `1.91 px`

### 객체 모델

- 파일: `my_rule/models/model_object_0804.pt`
- 형식: Ultralytics YOLO detection
- 실행 주기: `3 Hz`

| 원본 클래스 | 사용 방식 |
| --- | --- |
| `cone` | 라바콘 존재와 진입 확인 |
| `red_car`, `green_car` | 차량 장애물 회피 |
| `red_4`, `green_4`, `yellow_4` | 4구 신호등 상태 판단 |
| `left_4` | 초록불과 함께 지름길 선택 |
| `*_3` | 현재 대회에서 사용하지 않음 |

## 8. 통합 주행 상태

```text
WAIT_GREEN
  └─ green_4 → WAIT_START_CLEAR → RUNNING

RUNNING
  ├─ cone + LiDAR → 라바콘 주행
  ├─ car + LiDAR → 장애물 회피
  ├─ red_4 2회 → RED_HOLD
  └─ green_4 + left_4 → SHORTCUT_LEFT 1회

RED_HOLD
  └─ green_4 2회 → RUNNING
```

지름길 좌회전의 현재 초기값은 `-18`, 최대 속도 `8.0`, 유지 `1.8초`다. 실제 지름길 형상과 카메라 입력을 확인한 뒤 조향 시작점과 차선 재진입을 실차에서 조정해야 한다.

## 9. 향후 검증

1. 기본 주행의 차선 직선·곡선 속도와 조향 실차 검증
2. 라바콘 진입·한쪽 경계·탈출 실차 검증
3. `red_car`, `green_car` 회피 방향과 복귀 실차 검증
4. 주행 중 빨간불 정지 위치 조정
5. 지름길 좌회전 시작점·조향·재진입 조정
