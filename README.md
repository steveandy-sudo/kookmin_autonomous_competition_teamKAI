# kookmin_autonomous_competition_teamKAI — `New-Perception`

ROS 2 Humble 기반 Xycar 룰베이스 자율주행 프로젝트다. 현재 실차 검증 코드는 차선 주행, 신호등, 지름길, 라바콘 및 차량 회피를 하나의 주행 선택기와 최종 Space 안전 게이트로 통합한다.

- 미니 PC 워크스페이스: `/home/xytron/xycar_ws`
- ROS 배포판: ROS 2 Humble
- direct Xbin 시험 속도: 직선 `20`, 곡선 `12`, DEGRADED `12`, 라바콘 `8`
- 통합 주행 전체 속도 상한 기본값: `20`
- 제어 우선순위: `TRAFFIC > SHORTCUT > CONE > YOLO+LiDAR AVOIDANCE > RULE`

## 1. 프로젝트 개요 및 저장소 구조

```text
kookmin_autonomous_competition_teamKAI/
├── README.md
├── .gitignore
└── src/
    ├── il_data_tools/                # rosbag 데이터셋 생성·분석
    ├── kaiev26_msgs/                 # Canonical 차선·도로 메시지
    ├── lane_bev_tools/               # BEV 경로 미리보기 도구
    ├── lane_seg_control/             # X-bin/LR-ASPP 차선 인지
    ├── shortcut_entry_review/        # 지름길 진입·인계 시퀀스
    ├── study/
    │   ├── my_rule/                  # 객체 인지·LiDAR 라바콘 주행
    │   └── my_rule_msgs/             # 객체 인지 메시지
    ├── track_drive_sve/              # W1/W2 지름길 예비 주행
    ├── wide_camera/                  # 광각 MJPEG 카메라
    ├── xycar_device/                 # LiDAR·VESC·차량 메시지
    ├── xycar_gazebo_bridge/          # 오프라인·시뮬레이션 검증 브리지
    ├── xycar_map_nav/                # 미션 선택·회피·최종 Space 게이트
    ├── xycar_perception/             # 카메라 보정·Canonical 공통 코드
    ├── xycar_rl/                     # rosbag 정책 분석·시뮬레이션 도구
    └── xycar_rule_drive/             # Stanley + Pure Pursuit 추종
```

실차 통합주행과 함께 W1/W2 지름길 예비 방식, 라바콘 단독시험, rosbag 재생 분석 및 RViz 검증에 필요한 패키지만 유지한다. 기록한 rosbag은 Git 저장소 밖의 `/home/xytron/rosbags`에 날짜·용도별로 보관한다.

주요 노드:

| 노드 | 역할 |
| --- | --- |
| `wide_camera_node` | `1280x1024` MJPEG 영상 발행 |
| `lane_seg_lraspp_inference` | Xbin/LR-ASPP 추론과 direct 미터 경로 생성 |
| `canonical_stanley_pursuit_driver` | direct 미터 경로 또는 Canonical 경로의 RULE 후보 명령 생성 |
| `my_rule_object_detection_node` | YOLO 객체·신호등 판단 |
| `my_rule_cone_node` | LiDAR 라바콘 중앙 경로 생성 |
| `sequential_hybrid_driver` | 차선·라바콘·회피·신호등·지름길 후보 선택 |
| `space_drive_gate` | Space 입력 확인 후 `/xycar_motor` 최종 발행 |
| `xycar_vesc_driver` | VESC 직렬 구동·고장 감시·상태 발행 |

## 2. 전체 주행 로직과 핵심 원리

- `run_complete_space_hybrid.sh`가 카메라·LiDAR·VESC를 실행하고 장치와 토픽을 검사한다.
- `run_space_hybrid_test.sh`가 차선 인지, 객체 인지, 라바콘, 차량 회피, 신호등, 지름길 및 최종 Space 게이트를 실행한다.
- 각 인지·제어 노드는 후보 명령만 만들고, 최종 선택기와 Space 게이트가 실제 모터 출력을 결정한다.

```text
카메라 ─┬─ Xbin 차선 모델 → direct 미터 경로 → RULE 후보 ─┐
        ├─ 객체 YOLO → 신호등·차량·라바콘 판단             ├─ 통합 선택기
        └─ 지름길 차선 인지 → SHORTCUT 후보                 │
LiDAR ──── 라바콘 경계·장애물 거리 → CONE/AVOIDANCE 후보 ──┘
                                              ↓
                                      Space 안전 게이트
                                              ↓
                                        VESC 모터 출력
```

## 3. 차선 인지·경로 생성·차선 추종

1. 광각 카메라 영상을 왜곡 보정한 뒤 `kookmin_far_centerline_xbin_512x288.pt`에 `512x288`로 입력한다.
2. 모델이 출력한 노란 중앙선 마스크를 4픽셀 간격 anchor row로 읽고, 각 행의 확률 가중 중심을 구한다.
3. 전체 BEV/Canonical 영상을 렌더링하지 않고 중심 픽셀만 평면 homography로 투영한다. `x=전방 m`, `y=왼쪽 m`인 `/perception/xbin_direct_centerline`을 발행한다.
4. 미터 환산은 좌우 `0.0021875 m/px`, 전방 `0.002272727273 m/px`을 사용하며 유효 범위는 전방 최대 `2.5 m`, 좌우 `+/-0.7 m`이다.
5. RULE 제어기는 이 미터 점 목록을 직접 받아 Stanley와 Pure Pursuit을 혼합한다. 직선, 곡선, DEGRADED 경로 분류와 미션 우선순위는 기존 통합 제어기를 그대로 사용한다.
6. direct Xbin 경로가 `0.50초` 이상 갱신되지 않으면 유효하지 않은 경로로 처리한다. 호모그래피는 고정된 카메라 각도와 평탄한 노면을 가정한다.

## 4. LiDAR 기반 라바콘·장애물 주행

### 라바콘

- LiDAR 군집으로 좌우 라바콘 경계를 구성하고 통로 중앙 경로를 생성한다.
- 검증된 코스 순서인 `좌조향 → 우조향 → 마지막 큰 좌조향`을 상태로 추적한다.
- 한쪽 경계 소실, 넓어진 통로 및 간헐적인 큰 라바콘 간격은 시간 누적과 이전 경로 연속성으로 제한적으로 보완한다.
- YOLO·LiDAR 사전 검출 시 먼저 감속하고, 전방 `0.95 m` 이내에서 유효 CONE 명령이 3프레임 확인되면 조향권을 넘긴다.
- 라바콘 정상 주행 속도는 상수 `8`이며, 마지막 큰 좌조향의 경로 소실 복구 등 안전 예외에서만 더 낮은 속도를 사용한다.

### 차량·단일 장애물 회피

- 카메라 bbox 방위와 LiDAR 군집을 결합해 장애물 거리와 회피 방향을 정한다.
- `red_car`와 `green_car`를 별도 클래스로 유지하면서 같은 회피 경로·조향 상태기를 사용한다.
- 차량 회피 속도 상한은 클래스와 관계없이 `20`이다.
- 회피 중 반대쪽 판단은 2프레임 연속 확인한 뒤 방향을 재선택한다.
- 지름길 구간에서는 차량 회피 전환을 억제한다.
- 통합 주행 전체 속도 상한이 더 낮으면 최종 출력은 그 값을 넘지 않는다.

## 5. 통합 주행 실행 매뉴얼

아래 명령은 새 direct Xbin 경로를 사용한다. 초기 질문은 모두 `Enter`를 눌러 지정된 값을 사용하고, `READY`가 표시되면 `Space`로 출발한다. 직선 `20`, 곡선 `12`, DEGRADED `12`, 라바콘 `8`, 전체 속도 상한 `20`이 적용된다.

```bash
cd /home/xytron/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source install_xycar_only/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

XYCAR_RULE_PERCEPTION_BACKEND=direct_xbin \
XYCAR_PERCEPTION_MAX_OUTPUT_RATE_HZ=20.0 \
DIRECT_XBIN_PATH_TIMEOUT_SEC=0.50 \
OVERALL_SPEED_LIMIT_COMMAND=20 \
CURVATURE_SPEED_CONTROL_ENABLED=true \
CURVE_SPEED_COMMAND=12 \
DEGRADED_PATH_SPEED_COMMAND=12 \
XYCAR_ENABLE_RVIZ=false \
bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh \
  20 0.30 20 0
```

## 6. 주요 설정값·주기 및 안전 주의사항

| 항목 | 현재 값 |
| --- | ---: |
| 차선 인지 상한 | 20 Hz |
| 차선 추종 후보 명령 | 10 Hz |
| 객체 YOLO | 3 Hz |
| 최종 선택·Space 게이트 | 20 Hz |
| 전체 최종 속도 상한 | 20 |
| 직선 속도 | 20 |
| 곡선 / DEGRADED 경로 속도 | 12 / 12 |
| 라바콘 속도 | 8 |
| `red_car` / `green_car` 회피 상한 | 20 / 20 |
| 지름길 감지 기준 | `yellow_count`, 노란 점선 2개 |
| 지름길 강제 조향 / RULE 복귀 | -37 / 1.2초 |

실행 스크립트는 다른 ROS 워크스페이스의 환경을 제거하고 `/home/xytron/xycar_ws/install_xycar_only`만 사용한다. 이 전용 설치 결과는 `bash src/xycar_map_nav/scripts/build_xycar_only.sh`로 생성한다. 통합 launch의 인지·판단 노드는 `/xycar_motor`를 직접 발행하지 않으며, 실제 출력은 Space 게이트와 VESC 안전 조건이 모두 정상일 때만 허용된다. 시각화가 필요할 때만 실행 명령의 `XYCAR_ENABLE_RVIZ=false`를 `true`로 바꾼다.

## 7. 현재 AI 모델

### 차선 모델

- 일반 차선 주행: `lane_seg_control/models/kookmin_far_centerline_xbin_512x288.pt`
- 지름길 인지: `xycar_perception/models/kookmin_lane_lraspp_mbv3s_256x144.pt`

### 객체 모델

- 파일: `study/my_rule/models/final.pt`
- 형식: Ultralytics YOLO detection
- 실행 주기: `3 Hz`

| 클래스 | 사용 방식 |
| --- | --- |
| `cone` | 다중 라바콘 통로 진입 또는 단일 장애물 회피 |
| `red_car` | 기존 차량 회피, 속도 상한 `20` |
| `green_car` | 차량 회피, 속도 상한 `20` |
| `red_4`, `yellow_4`, `green_4` | 최초 신호등 출발 판단 |
| `left_4` | 지름길 진입 요청 |
| `null_4` | 미션 판단에서 사용하지 않음 |

## 8. 통합 주행 상태

```text
READY/STOP
  └─ Space + 초록불 확인 → RULE

RULE
  ├─ 라바콘 진입 조건 → CONE → 탈출 후 RULE
  ├─ 차량·단일 장애물 → AVOIDANCE → 복귀 후 RULE
  ├─ 지름길 조건 → SHORTCUT → 복귀 후 RULE
  └─ Space → STOP
```

차선 주행을 기본으로 하며 미션 조건이 확인된 동안에만 해당 후보가 조향·속도 권한을 가진다. 센서 또는 후보 명령이 오래되거나 VESC fault가 발생하면 안전 정지한다.
