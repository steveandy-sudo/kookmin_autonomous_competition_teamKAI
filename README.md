# kookmin_autonomous_competition_teamKAI — `main`

ROS 2 Humble 기반 Xycar 룰베이스 자율주행 프로젝트다. 현재 코드는 다음 기능을 하나의 주행 선택기와 최종 Space 안전 게이트로 통합한다.

- X-bin 차선 인지와 Canonical 경로 생성
- Stanley + Pure Pursuit 차선 추종
- LiDAR 기반 정적 장애물 감시와 YOLO + LiDAR 차량 회피
- LiDAR 기반 라바콘 통로 주행과 단일 라바콘 회피
- 4구 신호등 정지·출발 및 `left_4` 연계 지름길 주행
- ROS 2 VESC 구동

- 미니 PC 워크스페이스: `/home/xytron/xycar_ws`
- ROS 배포판: ROS 2 Humble

## 1. 프로젝트 개요 및 저장소 구조

```text
kookmin_autonomous_competition_teamKAI/
├── README.md
├── .gitignore
└── src/
    ├── kaiev26_msgs/                 # Canonical 차선·도로 메시지
    ├── lane_seg_control/             # X-bin/LR-ASPP 차선 인지
    ├── shortcut_entry_review/        # 지름길 진입·인계 시퀀스
    ├── study/
    │   ├── my_rule/                  # YOLO 객체 인지·LiDAR 라바콘
    │   └── my_rule_msgs/             # 객체 인지 메시지
    ├── wide_camera/                  # 광각 MJPEG 카메라
    ├── xycar_map_nav/                # 미션 선택·회피·최종 Space 게이트
    ├── xycar_perception/             # 카메라 보정·Canonical 공통 코드
    ├── xycar_rule_drive/             # Stanley + Pure Pursuit 추종
    └── xycar_device/
        ├── xycar_lidar/
        ├── xycar_msgs/
        └── xycar_vesc_driver/
```

주요 노드:

| 노드 | 역할 |
| --- | --- |
| `wide_camera_node` | `1280x1024` MJPEG 영상 발행 |
| `lane_seg_lraspp_inference` | X-bin/LR-ASPP 모델 추론과 Canonical 경로 생성 |
| `canonical_stanley_pursuit_driver` | Canonical 경로 추종 후보 명령 생성 |
| `my_rule_object_detection_node` | 저주기 YOLO 객체·신호등 판단 |
| `my_rule_cone_node` | LiDAR 라바콘 중앙 경로 생성 |
| `sequential_hybrid_driver` | 차선·라바콘·회피·신호등·지름길 후보 선택 |
| `shortcut_sequence_entry` | W1 진입선과 지름길 시퀀스 판단 |
| `space_drive_gate` | Space 입력 확인 후에만 `/xycar_motor` 최종 발행 |
| `xycar_vesc_driver` | VESC 직렬 구동·고장 감시·상태 발행 |

## 2. 전체 주행 로직과 핵심 원리

- `run_complete_space_hybrid.sh`가 카메라·LiDAR·VESC를 실행하고 장치와 토픽을 검사한다.
- `run_space_hybrid_test.sh`가 차선 인지, 객체 인지, 라바콘, 장애물 회피, 신호등, 지름길 및 최종 Space 게이트를 실행한다.
- 최종 선택 우선순위는 `TRAFFIC > SHORTCUT > CONE > YOLO+LiDAR AVOIDANCE > RULE`이다.

```text
카메라 1280x1024
├─ X-bin 차선 모델 → Canonical 경로 → Stanley + Pure Pursuit ┐
├─ 객체 YOLO 3 Hz → cone·car·4구 신호등                  ├─ 통합 선택기
└─ 지름길 전용 LR-ASPP → W1 진입·추종                     │
LiDAR → 정적 장애물·라바콘 경계·차량 거리                  ┘
                                      ↓
                               Space 안전 게이트
                                      ↓
                                /xycar_motor
                                      ↓
                              ROS 2 VESC driver
```

## 3. 차선 인지·경로 생성·차선 추종

1. 주행 카메라 영상을 `kookmin_far_centerline_xbin_512x288.pt`에 입력한다.
2. 노란 중앙선 위치와 흰 차선 연장 정보를 이용해 전방 Canonical 경로를 만든다.
3. Stanley의 횡오차 제어와 Pure Pursuit의 미리보기 조향을 혼합한다.
4. 직선·곡선·짧거나 기억된 경로를 분류해 속도 후보를 제한한다.
5. 차선이 완전히 끊기면 stale command를 사용하지 않고 안전 정지 조건으로 전환한다.

실행 인자의 속도는 최종 출력 상한이다. 예를 들어 실행 인자를 `4`로 주면 라바콘·회피·지름길 내부 후보가 더 높아도 실제 최종 속도는 `4`를 넘지 않는다.

| 항목 | 현재 값 |
| --- | ---: |
| 실행 가능 속도 인자 | 3.0~30.0 |
| 저속 실차 확인 권장값 | 4.0 |
| 차선 인지 상한 | 20 Hz |
| 차선 추종 후보 명령 | 10 Hz |
| 최종 선택·안전 게이트 | 20 Hz |

## 4. LiDAR 기반 라바콘·장애물 주행

### 라바콘

1. 전방 `-94~94도`, `0.18~2.20 m` LiDAR 점을 사용한다.
2. DBSCAN과 시간 누적을 이용해 라바콘 후보 군집을 만든다.
3. YOLO에서 기준 신뢰도 이상의 콘이 두 개 이상이면 라바콘 통로로 분류한다.
4. 좌우 경계의 중점 또는 한쪽 경계와 학습한 통로 폭으로 중앙 경로를 만든다.
5. 끊긴 경계는 제한 거리 안에서 연결하고, 추정 경로의 조향 변화는 별도로 제한한다.
6. 기준 콘이 한 개만 보이면 라바콘 통로가 아니라 국소 장애물 회피 대상으로 전달한다.

| 항목 | 값 |
| --- | ---: |
| 예상 통로 폭 | 0.85 m |
| 허용 통로 폭 | 0.68~0.98 m |
| 통합 런치 내부 라바콘 속도 후보 | 8.0 |
| 물리 조향 명령 한계 | 42 |
| 추정 경로 1회 조향 변화 제한 | 4° |
| 센서 존재 유지 시간 | 0.50초 |

### 정적·차량 장애물 회피

- `lidar_obstacle.py`가 주행 경로상의 LiDAR 군집과 좌우 통과 공간을 확인한다.
- YOLO의 `red_car`, `green_car`는 공통 `car` 클래스로 정규화한다.
- 차량 박스의 카메라 방위와 LiDAR 군집을 결합해 거리와 회피 방향을 정한다.
- 노란 중앙선 위치와 직선 안정성을 이용해 좌우 횡방향 목표를 연속적으로 이동한다.
- 회피 내부 속도 후보는 최대 `8.0`이며 실행 인자의 최종 속도 상한을 다시 적용받는다.
- 신호등 기둥에 해당하는 LiDAR 구간은 차량 장애물 결합 대상에서 제외한다.

## 5. 통합 주행 실행 매뉴얼

빌드:

```bash
cd /home/xytron/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

속도 상한 `4`의 전체 통합 주행:

```bash
cd /home/xytron/xycar_ws
unset XYCAR_WS

XYCAR_TEST_PROFILE=integrated \
XYCAR_STEERING_ONLY=false \
XYCAR_ENABLE_RVIZ=false \
bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh 4
```

장치·토픽 검사 후 `READY`가 표시되면 `Space`를 눌러 모터 출력을 허용한다. 다시 `Space`를 누르면 정지한다.

조향만 확인할 때:

```bash
cd /home/xytron/xycar_ws
unset XYCAR_WS

XYCAR_TEST_PROFILE=integrated \
XYCAR_STEERING_ONLY=true \
XYCAR_ENABLE_RVIZ=false \
bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh
```

RViz는 별도 터미널에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/xytron/xycar_ws/install/setup.bash
export ROS_DOMAIN_ID=7
ros2 launch my_rule drive_visualization_rviz.launch.py
```

## 6. 주요 설정값·주기 및 안전 주의사항

| 항목 | 설정 주기 |
| --- | ---: |
| 광각 카메라 | 장치 설정 최대 30 Hz |
| 차선 인지 상한 | 20 Hz |
| 차선 추종 명령 | 10 Hz |
| 객체 YOLO | 3 Hz |
| 주행 선택·Space 게이트 | 20 Hz |

주요 설정:

- 차선 인지: `lane_seg_control/launch/lane_seg_far_centerline_extended_real.launch.py`
- 차선 제어: `xycar_rule_drive/config/canonical_stanley_pursuit_real.yaml`
- 라바콘: `study/my_rule/config/cone_control.yaml`
- 객체 YOLO: `study/my_rule/config/object_detection.yaml`
- 회피·선택기: `xycar_map_nav/config/sequential_hybrid_real.yaml`
- 센서 런치: `xycar_map_nav/launch/real_hybrid_test_sensors.launch.py`
- 통합 런치: `xycar_map_nav/launch/real_sequential_hybrid_drive.launch.py`

통합 launch의 인지·판단 노드는 `/xycar_motor`를 직접 발행하지 않는다. 실제 모터 출력은 `space_drive_gate`가 활성화되고 VESC 드라이버가 정상 상태일 때만 허용된다. 저전압·과전류·USB 단절 fault가 발생하면 배터리와 배선 원인을 확인한 뒤 재실행해야 한다.

## 7. 현재 AI 모델

### 차선 모델

- 일반 차선주행: `lane_seg_control/models/kookmin_far_centerline_xbin_512x288.pt`
- 지름길 진입: `xycar_perception/models/kookmin_lane_lraspp_mbv3s_256x144.pt`
- 역할: 노란 중앙선과 흰 차선 연장을 이용한 Canonical 경로 생성

### 객체 모델

- 파일: `study/my_rule/models/final.pt`
- 형식: Ultralytics YOLO detection
- 실행 주기: `3 Hz`

| 원본 클래스 | 사용 방식 |
| --- | --- |
| `cone` | 다중 콘 통로 진입 또는 단일 콘 장애물 회피 |
| `red_car`, `green_car` | `car`로 정규화하여 차량 장애물 회피 |
| `red_4`, `yellow_4` | 두 프레임 확인 후 정지 유지 |
| `green_4` | 두 프레임 확인 후 정지 해제 |
| `left_4` | 지름길 진입 시퀀스 요청 |
| `null_4` | 미션 판단에서 사용하지 않음 |

## 8. 통합 주행 상태

```text
SPACE DISARMED
  └─ Space → RUNNING/RULE

RUNNING
  ├─ red_4 또는 yellow_4 2회 → TRAFFIC STOP
  ├─ left_4 확인·소실 → SHORTCUT SEARCH → SHORTCUT
  ├─ cone 2개 이상 + LiDAR → CONE_RULE
  ├─ car 또는 단일 cone + LiDAR → AVOIDANCE
  └─ 그 외 → RULE

TRAFFIC STOP
  └─ green_4 2회 → RUNNING

SHORTCUT
  └─ 진입·추종·T 출구 완료 → RULE
```

지름길은 `left_4`를 두 프레임 확인한 뒤 표지가 사라지고 설정된 접근 지연이 지난 시점에 시작한다. W1 경로를 먼저 추종하고 인계 조건이나 손실 조건을 만족하면 기존 RULE 제어로 자연스럽게 연결한다.

### W1/W2 지름길 진입 로직

통합 주행에서는 `--shortcut-mode w1`과 `--shortcut-mode yellow_count` 중 하나를 선택한다. 옵션을 생략하면 기존 W1/W2 방식인 `w1`을 사용하며 두 인지·제어 노드는 동시에 실행되지 않는다.

#### 방식 1: `w1`

1. `left_4` 진입 시퀀스가 열리면 흰선 후보 중 기존 진행 방향으로 이어지는 선을 W2로 먼저 추적한다.
2. W2 왼쪽에서 차량 기준 왼쪽으로 갈라지고, BEV 하단까지 충분히 내려오며, W2와 최소 간격을 확보한 흰선만 W1 후보로 인정한다. 기본 확인은 최근 4개 인지 프레임 중 2회이며, 길이·기울기·분리도가 충분한 후보는 빠른 잠금 조건을 적용한다.
3. 추적 중인 W2와 잠긴 W1의 직선 교차점이 유효한 BEV 범위에 생기면 해당 프레임에서 분기점 조건을 만족한 것으로 판단한다. 기본 설정에서는 과거 거리 임계값을 추가로 기다리지 않고 W1 조향 권한을 즉시 연다.
4. 진입 직후에는 합성 차선이나 고정 `-30` 조향을 사용하지 않고 검출된 W1 경로를 직접 추종한다. 조향 급변은 기본 `90 command/s` 변화율 제한으로 완화한다.
5. 노란 중앙선 Y1이 2개 인지 프레임 동안 확인되면 W1 대신 검출된 Y1 경로를 직접 추종한다. W2와 다른 노란선 후보는 조향 경로로 사용하지 않는다.
6. 진입 후 최소 `0.30 m` 진행한 상태에서 정렬, W1/Y1 동시 추적 또는 Y1 확인 후 W1 소실 조건이 만족되면 기존 노란 중앙선 RULE 주행으로 인계한다. 시각 조건이 끝까지 만족되지 않아도 W1 조향 시작 후 최대 `1.3 s`가 지나면 RULE로 복귀한다.

지름길 내부 속도 명령 상한은 기본 `9.0`이며, 통합 실행 명령에서 지정한 전체 속도 상한이 더 낮으면 최종 출력은 그 값을 넘지 않는다. 주요 조절값은 `SHORTCUT_W1_STEERING_START_DELAY_FRAMES`, `SHORTCUT_MINIMUM_ENTRY_PROGRESS_M`, `SHORTCUT_MAXIMUM_ENTRY_STEERING_SEC`, `SHORTCUT_W1_STEERING_HOLD_SEC`, `SHORTCUT_ENTRY_SPEED_COMMAND`, `SHORTCUT_ENTRY_STEERING_RATE_LIMIT_CMD_PER_SEC` 환경변수로 변경할 수 있다.

W1 방식의 터미널 이벤트 색상은 다음과 같다.

| 색상 | 이벤트 |
| --- | --- |
| 노랑 | `left_4` 검출 누적 및 검출 확정 뒤 소실 누적 |
| 핑크 | 유효한 W1/W2 교차점이 생겨 W1 직접 조향을 실제 시작한 순간 |
| 초록 | 지름길 후보가 완료되고 최종 선택기가 노란 X-bin RULE 주행으로 복귀한 순간 |

#### 방식 2: `yellow_count`

1. 공통 YOLO 판단에서 `left_4`를 2개 프레임 확인하고, 이어서 2개 프레임 동안 사라지면 노란선 카운트 인지를 시작한다.
2. W1 방식과 동일한 `kookmin_lane_lraspp_mbv3s_256x144.pt` 모델의 노란 클래스 마스크만 사용하며, W1 입력과 동일한 mask-only BEV 투영을 적용한다.
3. 노란 점선 성분이 BEV 카운트 띠에 2개 인지 프레임 동안 들어온 뒤 2개 인지 프레임 동안 빠져나가면 한 개가 사라진 것으로 센다. 카운트 띠 아래에 남아 있는 이전 점선은 다음 점선과 합치지 않는다.
4. 첫 번째 이탈은 노란색 `YELLOW LINE DISAPPEARED 1/2`, 두 번째 이탈은 노란색 `YELLOW LINE DISAPPEARED 2/2` 로그로 기록한다.
5. 두 번째 이탈 즉시 기본 `-42` 좌조향 후보를 출력한다. 진입 속도 명령 상한은 `9.0`이며 실제 모터 출력은 통합 선택기와 Space 안전 게이트만 소유한다.
6. `--shortcut-return-sec`로 정한 시간이 지나면 완료 신호와 최신 RULE 후보를 내보낸다. 최종 선택기가 RULE 권한으로 돌아간 실제 순간에만 초록색 복귀 로그를 출력한다.
7. RULE 후보가 오래됐으면 강제조향 타이머를 시작하지 않고 안전 정지 후보를 내며, RULE 후보가 정상화된 시점부터 지정 시간을 잰다.

`yellow_count`의 강제 조향값은 `--shortcut-angle`로 `-42~0`, 유지 시간은 `--shortcut-return-sec`로 `0.1~5.0`초 범위에서 지정한다.

### Xycar `kty_publish` 통합 주행

소스를 갱신하고 한 번 빌드한 뒤 아래 명령 하나만 실행한다. 질문에는 Enter를 눌러 기본값을 사용할 수 있고, 모든 검사 뒤 `READY`가 나오면 Space로 출발한다. 주행 중 Space를 다시 누르면 즉시 정지하고, 다시 누르면 재개한다.

W1/W2 방식:

```bash
cd /home/xytron/kty_publish && unset XYCAR_WS && XYCAR_TEST_PROFILE=integrated XYCAR_STEERING_ONLY=false XYCAR_ENABLE_RVIZ=false bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh 4 --shortcut-mode w1
```

노란선 2개 이탈 방식, 강제 좌조향 `-42`, `0.7`초 뒤 RULE 복귀:

```bash
cd /home/xytron/kty_publish && unset XYCAR_WS && XYCAR_TEST_PROFILE=integrated XYCAR_STEERING_ONLY=false XYCAR_ENABLE_RVIZ=false bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh 4 --shortcut-mode yellow_count --shortcut-angle -42 --shortcut-return-sec 0.7
```

첫 번째 숫자 `4`는 전체 주행 속도 상한이다. 실차 저속 확인이 끝난 뒤 필요한 값으로 올린다.

## 9. 현재 검증 상태 및 향후 확인

- 정리된 12개 ROS 패키지 전체 `colcon build --symlink-install` 성공
- 차선·라바콘·객체·지름길·통합 선택·VESC 안전 핵심 테스트 301개 통과
- 센서를 연결하지 않은 shadow 통합 launch에서 차선, YOLO, 라바콘, 지름길 및 selector 노드 기동 확인
- YDLIDAR upstream 코드 스타일과 `xycar_msgs/package.xml` XML 태그 순서 lint는 기존 상태로 남아 있다.

향후 실차 확인:

1. 속도 상한 `4`에서 차선 직선·곡선 조향 확인
2. 라바콘 양쪽 경계·한쪽 경계·경계 단절·탈출 확인
3. 단일 라바콘과 차량 장애물 회피 방향·복귀 확인
4. 빨간불 정지 위치와 초록불 재출발 확인
5. 지름길 W1 진입점·조향 인계·T 출구 복귀 확인

## 10. 2026-08-19 태윤 회피주행 완료본

실차 미니 PC의 `/home/xytron/kty_publish/xycar_ws/src`를 기준으로 `main/src`를 동기화했다. 겹치는 주행 코드는 KTY 버전을 우선 적용했다.

- 차량 회피는 직선뿐 아니라 곡선에서도 최신 노란 중앙선을 기준으로 장애물 좌우를 판단한다.
- 장애물이 차량 오른쪽에 있으면 왼쪽으로, 왼쪽에 있으면 오른쪽으로 회피한다.
- 신호등 정지는 bbox 면적을 거리 조건으로 사용하지 않고 confidence와 연속 프레임으로 판단한다.
- 라바콘이 YOLO와 LiDAR에서 사라진 뒤 센서 유효시간 `0.5초`가 지나면 RULE 주행으로 복귀한다.
- VESC 가속 제한은 `0.4 m/s²`, 저전압 정상 복구 확인 시간은 `1.0초`를 사용한다.
- 제어 지연예측은 직선 `0.20초`, 곡선 `0.40초`이며 곡선 상태는 최소 `0.50초` 유지한다.
