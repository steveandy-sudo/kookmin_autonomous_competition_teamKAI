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
- 차량 회피 속도 상한은 클래스와 관계없이 `20`이다.
- 회피 중 반대쪽 판단은 2프레임 연속 확인한 뒤 방향을 재선택한다.
- 지름길 구간에서는 차량 회피 전환을 억제한다.
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

## 11. 팀 개발 환경 및 Git 운영 규칙

### 11.1 자이카 미니 PC 워크스페이스

자이카 내부 미니 PC의 작업 기준 경로는 `/home/xytron`이며, 아래 세 워크스페이스의 목적을 섞지 않는다.

| 경로 | 용도 | 운영 원칙 |
| --- | --- | --- |
| `/home/xytron/xycar_ws` | 대회 당일 사용할 통합주행 워크스페이스 | 실차에서 반복 검증된 안정 로직만 유지한다. GitHub `main` 브랜치와 항상 동일한 상태로 관리한다. |
| `/home/xytron/xycar_ws_demo` | 대회용 로직을 개발·시험하는 워크스페이스 | `xycar_ws`를 기준으로 새로운 로직을 개발하고 실차 검증한다. 각 팀원은 본인 영문 이니셜 브랜치를 연결해 clone/pull/push한다. 검증 전 코드는 이곳과 개인 브랜치에만 둔다. |
| `/home/xytron/parking_ws` | 주차 로직 전용 워크스페이스 | 통합주행과 별개의 대회 로직이다. 주차 개발을 명시적으로 진행할 때만 사용하며 통합주행 변경과 섞지 않는다. |

모든 rosbag은 Git 워크스페이스 밖의 `/home/xytron/rosbags`에 저장한다. 폴더명은 `MMDD_주요키워드및구절` 형식을 사용하고, 같은 목적의 여러 주행은 그 아래에 시간 또는 성공·실패 정보를 구분해 보관한다. 대용량 rosbag 원본은 GitHub에 올리지 않는다.

워크스페이스 간 ROS overlay가 섞이지 않도록 각 워크스페이스에서 빌드·실행할 때 다른 작업공간 환경을 제거하고 해당 작업공간의 `install_xycar_only`만 사용한다. 실차 통합주행에 반영하기 전에는 최소한 패키지 테스트, 독립 빌드, 실행 파라미터 확인과 rosbag 기록을 수행한다.

### 11.2 GitHub 브랜치 구성

저장소: [kookmin_autonomous_competition_teamKAI](https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI)

| GitHub 브랜치 | 대응 워크스페이스 | 포함해야 하는 내용 |
| --- | --- | --- |
| `main` | `/home/xytron/xycar_ws` | 대회 당일 바로 사용할 수 있고 실차 검증까지 끝난 안정 로직 |
| 각 팀원 영문 이니셜 브랜치 (`hwj`, `kty` 등) | `/home/xytron/xycar_ws_demo` | 개발 중인 기능, 비교 시험, 파라미터 조정 및 실차 검증 대기 로직 |

개발 결과를 `main`으로 올리는 순서는 다음과 같다.

1. 최신 `main`을 기준으로 개인 이니셜 브랜치와 `xycar_ws_demo`를 준비한다.
2. 개인 브랜치에서 코드 수정, 테스트, 독립 빌드와 정적 검증을 완료한다.
3. 실제 차량에서 같은 조건을 여러 번 주행하고 rosbag으로 성공·실패 원인을 확인한다.
4. 실차 검증이 끝난 변경만 `main`에 반영하고 `/home/xytron/xycar_ws`를 새 `main`과 동기화한다.
5. 아직 검증되지 않았거나 성공률이 불충분한 코드는 개인 브랜치와 `xycar_ws_demo`에 남긴다.

### 11.3 2026-08-20 현재 코드 경계

- `main = /home/xytron/xycar_ws`: 기존 실차 완주 검증본이다. 아래 1·2번 문제를 해결하기 위해 새로 만든 코드는 아직 포함하지 않는다.
- `hwj = /home/xytron/xycar_ws_demo`: S자 진입 감속과 신호등 3회 통과 순서 고정 코드를 개발·테스트·빌드한 상태다. 최신 개발 커밋은 `30a773c`이며 아직 실차 검증 전이다.
- 따라서 1·2번 기능의 실차 검증이 끝나기 전에는 `xycar_ws`와 `xycar_ws_demo`의 코드가 의도적으로 다르다.

### 11.4 현재 문제와 우선순위

#### 필수 해결 항목

1. S자 곡선 진입 속도
   - 속도 command `11`로 중앙선 위에서 진입하면 S자 곡선을 안정적으로 통과할 수 있었다.
   - 지름길 좌회전 탈출 후 RULE로 복귀할 때 S자 진입 전 직선에서 속도 `25`까지 다시 가속하는 경우가 있다.
   - `green_car` 동적 장애물 회피가 끝난 뒤 RULE로 복귀할 때도 같은 직선에서 속도 `25`까지 다시 가속하는 경우가 있다.
   - `hwj`에서는 위 두 미션 종료만 트리거로 사용해 S자 첫 정상 좌조향까지 속도 상한 `11`을 유지하도록 개발했다. 다른 좌회전·라바콘·일반 차선 구간은 건드리지 않는다.

2. 좌회전 신호와 초록불 혼동
   - 첫 번째 신호등은 항상 빨간불에서 초록불로 바뀌므로 좌회전 오인식을 무시하고 직진하도록 한다.
   - 두 번째 신호등에서 좌회전했으면 세 번째 신호등은 직진만 허용한다.
   - 두 번째 신호등에서 직진했으면 세 번째 신호등은 좌회전만 허용한다.
   - 별도 바퀴 수 추정은 사용하지 않고 실제 신호 통과가 확정될 때마다 신호등 전용 횟수를 증가시킨다.
   - 두 번째와 세 번째에서 빨간불 또는 노란불이면 기존 정지 latch가 항상 최우선으로 작동한다.

#### 선택적 개선 항목

3. 라바콘 구간 탈출 직전 정지
   - 라바콘 주행 종료 직전에 차량이 가끔 약 1초 멈춘 뒤 RULE로 복귀하는 현상이 남아 있다.

4. S자 곡선 주행 속도 상향
   - S자 곡선 속도가 `11`보다 커지면 조향 정확도가 떨어지는 문제가 있다.
   - 현재 `11`로 완주하는 데에는 문제가 없으므로 필수 항목을 모두 해결한 뒤 속도 상향 가능성을 검토한다.

1·2번은 완주 안정성에 직접 영향을 주므로 반드시 먼저 해결한다. 3·4번은 현재 속도와 로직으로도 완주할 수 있어 선택적인 개선 영역이며, 1·2번 검증 중 문제가 나오면 해당 두 문제의 수정과 재시험을 우선한다.

### 11.5 2026-08-21 금요일 국민대학교 연습장 검증 계획

시험 대상은 `hwj` 브랜치와 동일한 `/home/xytron/xycar_ws_demo`이며, 안정본 `/home/xytron/xycar_ws`에는 아직 1·2번 개발 로직이 없다는 점을 확인하고 시작한다.

1. 지름길 좌회전 탈출 후 S자 진입에서 속도 command 상한 `11`, 실제 차량 속도, 첫 좌조향 시점과 중앙선 진입 위치를 기록한다.
2. `green_car` 회피 종료 후 S자 진입에서도 같은 값을 기록한다.
3. 중앙선 위 출발과 중앙선보다 약간 왼쪽 출발을 각각 반복해 정상 중앙 복귀 조향이 유지되는지 확인한다.
4. 첫 번째 신호등이 항상 직진하는지 확인한다.
5. `두 번째 좌회전 → 세 번째 직진`과 `두 번째 직진 → 세 번째 좌회전`을 각각 시험한다.
6. 두 번째와 세 번째 신호등에서 빨간불 정지가 기존과 동일하게 유지되는지 확인한다.
7. 모든 시험은 `/home/xytron/rosbags`에 규칙에 맞는 이름으로 기록하고 성공·실패 순서를 별도로 메모한다.

1·2번 중 하나라도 실패하면 해당 문제부터 수정하고 같은 조건으로 다시 검증한다. 두 항목이 모두 반복 성공하면 실차 검증 결과와 rosbag 근거를 정리해 `main`과 `/home/xytron/xycar_ws`에 승격한다. 이후 시간이 남을 때 3번 라바콘 탈출 정지 문제를 먼저 분석하고, 마지막으로 4번 S자 곡선 속도 상향을 시험한다.

### 11.6 2026-08-22 `hwj` 전체 속도 8 완주 참고

현재 `main`의 실행 로직은 `ce84e5c` 계열이며, 아래 기록은 `main`의 현재 기본 실행값이 아니라 별도 `hwj` 브랜치의 실차 완주 참고 자료다. 2026-08-22 오전 `10:12:03 KST` 완주 코드는 `hwj`의 `32a7071`이고, README를 제외한 코드 트리는 당시 실행한 `6098fc7`과 동일하다.

```bash
cd /home/xytron/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

CURVATURE_SPEED_CONTROL_ENABLED=true \
CURVE_SPEED_COMMAND=8 \
DEGRADED_PATH_SPEED_COMMAND=8 \
XYCAR_ENABLE_RVIZ=false \
bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh \
  8 0.30 20 0
```

- 위 명령은 `hwj` 코드에서 일반 RULE, 곡선, DEGRADED 및 최종 출력 속도 상한을 `8`로 맞춘다.
- `hwj` 스냅샷의 라바콘 속도도 `8`이며, 안전 정지나 미션 상태에 따라 실제 출력은 더 낮거나 `0`일 수 있다.
- `hwj`에는 라바콘 우조향 최소 유지 `0.75초`, 마지막 좌조향 복구 속도 `8`, 라바콘 소실 후 탈출 `1.0초`, S자 진입 전 라바콘 재진입 차단, `red_car` 소실 복귀 `0.30초`, VESC 가속 제한 `0.6 m/s²`가 포함된다.
- `main`과 `hwj`는 소스·설정·테스트 20개 파일이 다르다. 이 조건을 재현하려면 `hwj` 커밋을 사용해야 하며, README의 명령만 실행한다고 `main` 코드가 해당 스냅샷으로 바뀌지는 않는다.
