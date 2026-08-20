# kookmin_autonomous_competition_teamKAI — `main`

ROS 2 Humble 기반 Xycar 룰베이스 자율주행 프로젝트다. 현재 실차 검증 코드는 차선 주행, 신호등, 지름길, 라바콘 및 차량 회피를 하나의 주행 선택기와 최종 Space 안전 게이트로 통합한다.

- 미니 PC 워크스페이스: `/home/xytron/xycar_ws`
- ROS 배포판: ROS 2 Humble
- 실차 기본 속도: 직선 `25`, 곡선·구불구불길 `11`, 라바콘 `8`
- 제어 우선순위: `TRAFFIC > SHORTCUT > CONE > YOLO+LiDAR AVOIDANCE > RULE`

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
    │   ├── my_rule/                  # 객체 인지·LiDAR 라바콘 주행
    │   └── my_rule_msgs/             # 객체 인지 메시지
    ├── wide_camera/                  # 광각 MJPEG 카메라
    ├── xycar_map_nav/                # 미션 선택·회피·최종 Space 게이트
    ├── xycar_perception/             # 카메라 보정·Canonical 공통 코드
    ├── xycar_rule_drive/             # Stanley + Pure Pursuit 추종
    └── xycar_device/                 # LiDAR·VESC·차량 장치 드라이버
```

주요 노드:

| 노드 | 역할 |
| --- | --- |
| `wide_camera_node` | `1280x1024` MJPEG 영상 발행 |
| `lane_seg_lraspp_inference` | X-bin/LR-ASPP 추론과 Canonical 경로 생성 |
| `canonical_stanley_pursuit_driver` | Canonical 경로 추종 후보 명령 생성 |
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
카메라 ─┬─ 차선 모델 → Canonical 경로 → RULE 후보 ─────────┐
        ├─ 객체 YOLO → 신호등·차량·라바콘 판단             ├─ 통합 선택기
        └─ 지름길 차선 인지 → SHORTCUT 후보                 │
LiDAR ──── 라바콘 경계·장애물 거리 → CONE/AVOIDANCE 후보 ──┘
                                              ↓
                                      Space 안전 게이트
                                              ↓
                                        VESC 모터 출력
```

## 3. 차선 인지·경로 생성·차선 추종

1. 광각 카메라 영상을 `kookmin_far_centerline_xbin_512x288.pt`에 입력한다.
2. 노란 중앙선과 흰 차선 정보를 이용해 전방 Canonical 경로를 만든다.
3. Stanley의 횡오차 제어와 Pure Pursuit의 미리보기 조향을 혼합한다.
4. 직선, 확정 곡선, 짧거나 기억된 경로를 분류해 속도 후보를 제한한다.
5. 기본 속도는 직선 `25`, 곡선과 짧거나 기억된 경로 `11`이다.
6. 조향 명령에는 직선·곡선별 평활화, 변화율 제한과 제어 지연 예측을 적용한다.

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
- `red_car` 회피 속도 상한은 `8`, `green_car` 회피 속도 상한은 `15`이다.
- 통합 주행 전체 속도 상한이 더 낮으면 최종 출력은 그 값을 넘지 않는다.

## 5. 통합 주행 실행 매뉴얼

아래 한 명령으로 신호등 출발부터 실제 대회용 통합 주행을 실행한다. 초기 질문은 모두 `Enter`를 눌러 기본값을 사용하며, `READY`가 표시되면 `Space`로 출발한다. 이때 직선 `25`, 곡선·구불구불길 `11`, 라바콘 `8`, 신호등 활성 설정이 적용된다.

```bash
cd /home/xytron/xycar_ws && unset XYCAR_WS && XYCAR_TEST_PROFILE=integrated XYCAR_STEERING_ONLY=false XYCAR_ENABLE_RVIZ=false XYCAR_TRAFFIC_LIGHT_CONTROL_ENABLED=true bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh 25 --shortcut-mode yellow_count --shortcut-yellow-count 2 --shortcut-angle -37 --shortcut-return-sec 1.2
```

## 6. 주요 설정값·주기 및 안전 주의사항

| 항목 | 현재 값 |
| --- | ---: |
| 차선 인지 상한 | 20 Hz |
| 차선 추종 후보 명령 | 10 Hz |
| 객체 YOLO | 3 Hz |
| 최종 선택·Space 게이트 | 20 Hz |
| 직선 속도 | 25 |
| 곡선·짧거나 기억된 경로 속도 | 11 |
| 라바콘 속도 | 8 |
| `red_car` / `green_car` 회피 상한 | 8 / 15 |

실행 스크립트는 다른 ROS 워크스페이스의 환경을 제거하고 `/home/xytron/xycar_ws/install_xycar_only`만 사용한다. 이 전용 설치 결과는 `build_xycar_only.sh`로 생성한다. 통합 launch의 인지·판단 노드는 `/xycar_motor`를 직접 발행하지 않으며, 실제 출력은 Space 게이트와 VESC 안전 조건이 모두 정상일 때만 허용된다.

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
| `red_car` | 기존 차량 회피, 속도 상한 `8` |
| `green_car` | 차량 회피, 속도 상한 `15` |
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

## 9. 현재 검증 상태

- `xycar_ws` 단독으로 필요한 17개 ROS 패키지 빌드가 완료되며 다른 워크스페이스 설치 결과를 사용하지 않는다.
- `my_rule`, `lane_seg_control`, `shortcut_entry_review`, `track_drive_sve`, `xycar_map_nav` 핵심 테스트 355개를 통과했다.
- 실제 경기장과 동일한 라바콘 배치에서 속도 `8` 주행 및 통합 라바콘 진입·탈출을 확인했다.
- 신호등 초록불 출발, 직선 `25`, 곡선 `11`, 라바콘 `8`, 차량 회피 및 지름길을 포함한 통합 주행 완주를 확인했다.
- 현재 완주 범위에서 S자 곡선 재진입 안정화는 제외되어 있다.

## 10. 추가 발전 가능성

- 가끔 신호등에서 직진과 좌회전을 구분하지 못하기도 합니다.
- 라바콘 주행 이후 탈출하기 전에 약 1초 정도 가끔 멈추기도 합니다.
- 2/3바퀴 주행 때 직진 구간 멀리서부터 빠르게 주행하면서 라바콘 구간에 진입할 때, 진입 구간 가운데로 차량이 들어가지 않으면 가끔 라바콘과 충돌합니다.
- S자 곡선 진입 구간에서 속도 `25` 정도로 너무 빨라 조향을 크게 하면서 마치 드리프트하듯 차선을 이탈했다가, 차선을 다시 인지해 S자 주행을 하기도 합니다.
