# Kookmin Autonomous Competition Team KAI

국민대 자율주행대회용 Xycar RULE·콘·차량 회피 통합 저장소다. 현재 실차
통합 브랜치는 `jsb`이며, 일반주행·신호등·지름길 제어는 검증된
`agent/yellow-center-curve-test` 기준을 사용한다. 제어 우선순위는
`TRAFFIC > SHORTCUT > CONE > YOLO_LIDAR_AVOIDANCE > RULE`이다.

- [실차 RULE·콘·차량 회피 실행 상세](docs/REAL_CAR_RUNBOOK_KO.md)
- [통합 주행 패키지 상세](xycar_ws/src/xycar_map_nav/README.md)
- [2026-08-07 통합 주행 구현·검증 현황](docs/INTEGRATED_RULE_DRIVE_STATUS_20260807_KO.md)

## 1. 처음 한 번 준비

저장소를 어느 디렉터리에 clone했든 사용할 수 있다. 아래 첫 번째 `cd`만
실제 clone 위치로 바꾼다.

```bash
cd /path/to/kookmin_autonomous_competition_teamKAI
git switch jsb
git pull --ff-only

REPO_ROOT="$(git rev-parse --show-toplevel)"
export XYCAR_WS="$REPO_ROOT/xycar_ws"
cd "$XYCAR_WS"

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to xycar_map_nav
source install/setup.bash
```

모델 실행 환경은 ROS 2 Humble의 시스템 Python을 기준으로 한다. Conda를
활성화하거나 launch 파일의 Python 경로를 PC별로 바꾸지 않는다. clone 후
다음 두 모델이 있는지 확인한다.

```bash
test -f src/lane_seg_control/models/kookmin_far_centerline_xbin_512x288.pt
test -f src/study/my_rule/models/kookmin_objects_best_20260804.pt
```

명령이 출력 없이 종료되면 두 모델이 모두 존재한다.

## 2. 새 터미널 공통 준비

아래 블록은 bag 재생과 실차 실행에서 새 터미널을 열 때마다 먼저 실행한다.
터미널을 저장소 내부에서 연 뒤 실행하면 사용자명이나 clone 위치에
의존하지 않는다.

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
export XYCAR_WS="$REPO_ROOT/xycar_ws"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
cd "$XYCAR_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

`git rev-parse`가 실패하면 먼저 clone한 저장소 안으로 이동한 뒤 다시
실행한다. 같은 실행에 참여하는 모든 터미널은 동일한 `ROS_DOMAIN_ID`를
사용해야 한다.

## 3. bag 원본 센서로 현재 코드 시험

이 절차는 bag에 저장된 과거 경로를 재생하지 않는다. 카메라 원본, LiDAR,
odom만 재생하고 **현재 체크아웃된 코드와 현재 YAML 파라미터**로 차선·콘
경로를 다시 계산한다. `/xycar_motor`는 재생하지 않으며
`drive_enabled:=false`이므로 차량으로 모터 명령도 내보내지 않는다.

먼저 bag 디렉터리를 지정한다. `BAG_PATH`는 `metadata.yaml`이 들어 있는
디렉터리여야 한다.

```bash
export BAG_PATH="/path/to/rosbag_directory"
test -f "$BAG_PATH/metadata.yaml"
```

### 터미널 1: 카메라 압축 해제·보정

공통 준비 블록을 실행한 뒤:

```bash
ros2 launch my_rule compressed_camera_republish.launch.py \
  use_sim_time:=true
```

### 터미널 2: 현재 코드로 경로 계산

공통 준비 블록을 실행한 뒤:

```bash
ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  use_sim_time:=true \
  drive_enabled:=false \
  gate_arming_required:=false \
  enable_rviz:=false \
  camera_image_topic:=/wide_camera/lane_rect/image_raw \
  object_camera_image_topic:=/wide_camera/object_rect/image_raw \
  camera_use_compressed_image:=false \
  camera_enable_rectify:=false \
  direct_model_rectify_enabled:=false
```

### 터미널 3: RViz2 시각화

공통 준비 블록을 실행한 뒤:

```bash
ros2 launch my_rule drive_visualization_rviz.launch.py \
  use_sim_time:=true
```

### 터미널 4: bag 진행률

공통 준비 블록과 `BAG_PATH` 지정을 다시 실행한 뒤:

```bash
ros2 run my_rule bag_progress "$BAG_PATH"
```

### 터미널 5: bag 재생

공통 준비 블록과 `BAG_PATH` 지정을 다시 실행한 뒤 마지막으로 시작한다.
`/odom`이 없는 bag은 명령에서 `/odom`만 빼면 된다.

```bash
ros2 bag play "$BAG_PATH" \
  --clock 40 \
  --rate 0.5 \
  --start-paused \
  --topics /wide_camera_mjpeg/image_raw/compressed /scan /odom
```

재생 터미널에서 `Space`를 누르면 시작/일시정지하고, `Right arrow`는 정지
상태에서 다음 메시지 하나를 보낸다. `Up arrow`와 `Down arrow`는 재생
속도를 10%씩 바꾼다. 시작 속도는 `--rate 0.25`, `0.5`, `1.0`, `2.0`
등으로 바꿀 수 있고 `--start-offset 28.0`처럼 시작 시각도 지정할 수 있다.

RViz에서 실제 수동 주행 odom 궤적은 초록색, 현재 코드의 차선 경로는
파랑/민트색, 활성화된 콘 제어 경로는 주황색으로 표시된다. 파생 토픽인
`/my_rule/cone_path`, `/rule_drive/connected_yellow_path`,
`/hybrid_gate/mode`를 bag 재생 목록에 추가하면 과거 결과와 현재 계산
결과가 섞이므로 이 시험에서는 추가하지 않는다.

## 4. 실제 센서와 차량에서 실행

실차에서는 `use_sim_time`이나 bag 재생을 사용하지 않는다. 권장 진입점은
저장소의 `run_complete_space_hybrid.sh`이며, 카메라·LiDAR·VESC를 직접
시작하고 센서 토픽을 확인한 뒤 SPACE 안전 게이트가 있는 제어기를 띄운다.
따라서 이 스크립트를 사용할 때 센서 드라이버를 다른 터미널에서 중복으로
실행하지 않는다.

필수 장치와 토픽은 다음과 같다.

- 카메라: `/wide_camera_mjpeg/image_raw/compressed`
- LiDAR: `/scan`
- VESC 상태: `/vehicle/vesc_state`
- 장치 심볼릭 링크: `/dev/ttyLIDAR`, `/dev/ttyMOTOR`

### 터미널 1: 센서·VESC·주행 제어 전체 실행

공통 준비 블록을 실행한 뒤:

```bash
./src/xycar_map_nav/scripts/run_complete_space_hybrid.sh
```

스크립트가 주행 파라미터를 질문하면 숫자만 입력하거나 Enter로 기본값을
사용한다. `READY`가 출력되기 전에는 차량을 움직이지 않는다.

- `Space`: 주행 시작 또는 즉시 정지
- `Ctrl+C`: 제어기와 이 스크립트가 시작한 센서 전체 종료

첫 시험은 구동 바퀴를 지면에서 띄우거나 차량을 고정한 상태에서 수행한다.
센서 오류가 나면 출력되는 `/tmp/xycar_hybrid_sensors_*.log`를 먼저 확인한다.

### 터미널 2: RViz2 시각화

공통 준비 블록을 실행한 뒤:

```bash
ros2 launch my_rule drive_visualization_rviz.launch.py \
  use_sim_time:=false
```

RViz는 시각화 토픽만 구독하며 모터 명령을 발행하지 않는다.

### 터미널 3: 실시간 토픽 확인(선택)

공통 준비 블록을 실행한 뒤 필요한 항목을 각각 확인한다.

```bash
ros2 topic hz /wide_camera_mjpeg/image_raw/compressed
ros2 topic hz /scan
ros2 topic echo /hybrid_gate/mode --once
ros2 topic info /xycar_motor -v
```

`ros2 topic hz`는 계속 실행되므로 다음 항목을 보려면 `Ctrl+C`로 종료한다.
`/xycar_motor` publisher가 둘 이상이면 중복 제어기를 모두 종료한 뒤 표준
스크립트 하나만 다시 실행한다.

### 센서가 이미 별도 프로세스로 실행 중인 경우

차량 이미지에 카메라·LiDAR·VESC가 systemd 등으로 이미 실행되도록 구성된
경우에는 중복 센서를 시작하는 complete 스크립트 대신 아래 제어기만
실행한다. 세 필수 토픽이 정상임을 먼저 확인해야 한다.

```bash
./src/xycar_map_nav/scripts/run_space_hybrid_test.sh
```

운영 PC마다 launch 파일이나 Python 경로를 수정하지 말고, 차이는 장치 udev
규칙과 `XYCAR_WS`, `ROS_DOMAIN_ID` 같은 실행 환경으로만 맞춘다.

---

국민대 자율주행대회용 Xycar Xbin RULE, 지름길, 라바콘, 차량 회피 통합
저장소다.

- [실차 RULE, 콘, 차량 회피 실행](docs/REAL_CAR_RUNBOOK_KO.md)
- [통합 주행 구현 및 검증 현황](docs/INTEGRATED_RULE_DRIVE_STATUS_20260807_KO.md)

일반주행·지름길 파라미터의 기준 브랜치:
`agent/yellow-center-curve-test`

## 지름길 주행 파라미터

아래 내용은 현재 `gsw`의
`real_sequential_hybrid_drive.launch.py`와 `shortcut_entry_review` 구현을
기준으로 한다. 단위가 다른 값을 한꺼번에 바꾸면 원인을 구분할 수 없으므로
실차 튜닝은 반드시 shadow(`drive_enabled:=false`)와 동일 rosbag에서 한 항목씩
비교한 뒤 적용한다.

현재 지름길 흐름은 다음과 같다.

```text
left_4 >= 0.40을 2 detector 프레임 확인
  -> left_4 미검출 2 detector 프레임 확인(S, 추가 시간 지연 0초)
  -> LR-ASPP로 W1 검색, RULE 유지
  -> W1 lock + 공간 gate + W1 유효 프레임 확인
  -> RULE/W1 조향 혼합
  -> 진입 완료 조건 충족
  -> 기본값은 yellow Xbin RULE로 handoff
```

### 조향 시작 위치와 혼합

공간 gate의 추정 조향 시작 거리는 다음 식으로 정해진다.

```text
entry_speed_cmd = min(현재 RULE speed command, shortcut_entry_speed_command)
estimated_speed_mps = entry_speed_cmd * shortcut_speed_command_to_mps
trigger_distance_m = shortcut_spatial_gate_minimum_distance_m
                   + estimated_speed_mps
                   * shortcut_spatial_gate_response_time_sec
```

차량이 W1/W2 추정 분기점에 접근해 남은 `branch_distance_m`가
`trigger_distance_m` 이하가 되면 gate가 열린다. 즉 **시작거리 값을 크게 하면
더 멀리서 일찍 조향하고, 작게 하면 분기점에 더 가까워진 뒤 늦게 조향한다.**
`minimum_distance`는 분기점을 통과한 뒤 추가로 직진할 거리를 뜻하지 않는다.

| 상위 launch 인자 | 현재 기본값 | 값을 작게 하면 | 값을 크게 하면 |
|---|---:|---|---|
| `shortcut_spatial_gate_minimum_distance_m` | `0.25` m | 분기점에 더 가까워진 뒤 gate가 열려 조향이 늦어진다. | 분기점에서 더 먼 위치에서 gate가 열려 조향이 빨라진다. |
| `shortcut_spatial_gate_response_time_sec` | `0.35` s | 속도 선행 보정이 줄어 고속에서도 조향 시작이 늦어진다. | 속도 선행 보정이 커져 고속일수록 더 일찍 조향한다. |
| `shortcut_speed_command_to_mps` | `0.04` | 추정 속도와 진행거리가 작아져 gate 및 handoff가 늦어진다. | 추정 속도와 진행거리가 커져 gate 및 handoff가 빨라진다. 실제 차속 환산 보정값이므로 임의 조향 gain처럼 쓰지 않는다. |
| `shortcut_spatial_gate_blend_distance_m` | `0.25` m | gate가 열린 뒤 W1 비중이 빠르게 증가한다. | W1 비중이 더 긴 거리 동안 천천히 증가한다. **조향 시작 위치는 바꾸지 않는다.** |
| `shortcut_w1_steering_start_delay_frames` | `4` | gate 뒤 적은 W1 프레임만 보고 빨리 조향한다. `0`이면 프레임 지연이 없다. | W1을 더 오래 확인한 뒤 조향하므로 늦지만 오검출에 강해진다. |
| `shortcut_w1_steering_delay_missing_tolerance_frames` | `2` | W1 한두 번 누락에도 누적 확인값을 빨리 초기화해 시작이 보수적이다. | 간헐적 W1 누락을 더 오래 허용해 조향 시작이 쉬워지지만 불안정한 W1도 통과할 수 있다. |
| `shortcut_w1_path_weight` | `0.60` | 고정 차선 폭 반대편 쪽으로 경로 offset이 커진다. 허용 최솟값은 `0.50`이다. | 중심경로가 W1 자체에 더 가까워지고 고정 횡 offset이 작아진다. `1.0`은 허용되지 않는다. |

현재 경로 식은 W1의 각도와 곡선 형상을 사용하고 고정 폭 `0.31`을 더한다.

```text
target_x = W1_x + (1 - shortcut_w1_path_weight) * 0.31
```

`shortcut_spatial_gate_blend_distance_m:=0.50`은 현재 기본값 `0.25`보다
W1 비중을 약 두 배 긴 거리에서 천천히 올리지만 gate가 열리는 위치는 같다.

### 조향 강제값, 속도와 일시 누락

| 상위 launch 인자 | 현재 기본값 | 값을 작게 하면 | 값을 크게 하면 |
|---|---:|---|---|
| `shortcut_entry_direction_hold_command` | `-30.0` | 음수 크기를 더 키워 `-35`처럼 만들면 더 강한 최소 좌조향을 강제한다. | `-20`, `-10`, `0`처럼 0에 가까워질수록 강제가 약해지고 `0`은 강제를 끈다. 양수는 우조향을 강제하므로 지름길 진입에 사용하지 않는다. |
| `shortcut_entry_speed_command` | `9.0` | 진입 속도 cap이 낮아진다. 동시에 gate 식의 추정 속도도 줄어 조향 시작이 늦어질 수 있다. | 더 높은 RULE 속도를 허용하고 속도 선행거리도 커져 조향 시작이 빨라질 수 있다. RULE 속도보다 높여도 차량을 RULE보다 가속하지는 않는다. |
| `shortcut_w1_steering_hold_sec` | `1.0` s | W1 경로가 끊기면 마지막 조향을 짧게 유지하고 RULE 검색으로 빨리 돌아간다. `0`이면 추가 hold가 없다. | 영상 누락 때 마지막 W1 조향을 더 오래 유지한다. 너무 크면 잘못된 조향도 오래 유지한다. |

주의: 현재 `gsw`의 mux는 유효한 W1 경로가 생기면 RULE/W1 혼합 결과에
`entry_direction_hold_command`를 적용한다. 기본 `-30`에서는 blend가 작아도
최종 후보가 최소 `-30` 좌조향으로 제한될 수 있다. 따라서 blend 효과만
확인하려면 먼저 shadow에서 hold를 `0`으로 비교해야 한다.

### 진입 종료와 handoff

| 상위 launch 인자 | 현재 기본값 | 값을 작게/끄면 | 값을 크게/켜면 |
|---|---:|---|---|
| `shortcut_minimum_entry_progress_m` | `0.50` m | 진입 후 더 짧게 진행해도 handoff가 가능해져 빨리 끝난다. | W1 조향으로 더 멀리 진행한 뒤 handoff한다. |
| `shortcut_pair_track_handoff_required_frames` | `2` | W1/Y1 pair를 적게 확인하고 빨리 handoff한다. | pair를 더 오래 확인하므로 안정적이지만 handoff가 늦어진다. |
| `shortcut_w1_loss_handoff_enabled` | `true` | `false`이면 Y1 확인 뒤 W1 소실을 handoff 조건으로 사용하지 않는다. | `true`이면 최소 진행거리 이후 Y1 확인 + W1 소실로도 handoff할 수 있다. |
| `shortcut_maximum_entry_steering_sec` | `1.5` s | 더 빨리 시간 제한 handoff가 발생한다. `0`이면 이 시간 fallback을 끈다. | W1 조향을 더 오래 허용하고 시간 fallback이 늦어진다. |
| `shortcut_handoff_to_rule` | `true` | `false`이면 semantic 진입 뒤 기존 ShortcutCore cruise/exit 후보를 기다린다. | `true`이면 semantic 진입 완료 뒤 yellow Xbin RULE 후보로 복귀한다. 숫자 크기 파라미터가 아니라 mode 선택이다. |

`minimum_entry_progress_m`를 만족하기 전에는 정렬이나 pair 조건만으로 handoff하지
않는다. 단, `maximum_entry_steering_sec` 시간 fallback은 최소 진행거리와 별개로
작동한다.

### left_4와 안전 timeout

다음 값은 현재 상위 launch 인자가 아니라
`sequential_hybrid_driver`의 파라미터 또는 통합 launch의 고정 연결값이다.
상위 launch 명령에 같은 이름을 임의로 추가해도 변경되지 않는다.

| 내부 파라미터 | 현재값 | 값을 작게 하면 | 값을 크게 하면 |
|---|---:|---|---|
| `shortcut_yolo_min_confidence` | `0.40` | 약한 `left_4`도 인정해 trigger가 쉬워지지만 오검출 위험이 커진다. | 확실한 detection만 인정해 오검출은 줄지만 미검출 가능성이 커진다. |
| `shortcut_yolo_required_frames` | `2` | `left_4`를 빨리 확정하지만 한 프레임 오검출에 약하다. | 여러 프레임을 요구해 안정적이지만 확정이 늦어진다. |
| `shortcut_yolo_absence_frames` | `2` | `left_4` 소실 S를 빨리 선언한다. | 일시 미검출에 강하지만 S와 LR-ASPP 시작이 늦어진다. |
| `shortcut_entry_search_timeout_sec` | `12.0` s | W1을 못 찾으면 검색을 빨리 취소하고 RULE로 돌아간다. | LR-ASPP/W1 검색을 더 오래 유지한다. |
| `shortcut_candidate_timeout_sec` | `0.35` s | 오래된 shortcut 후보를 빨리 거부하지만 주기 지터에도 끊길 수 있다. | 통신 지터에는 강하지만 오래된 조향을 더 오래 유효하게 본다. |
| `shortcut_rearm_absence_sec` | `1.0` s | 지름길 종료 뒤 trigger가 빨리 재무장되어 재진입 위험이 커진다. | 충분히 오래 `left_4`가 없어야 재무장되어 중복 trigger가 줄어든다. |

`shortcut_wait_for_entry_ready`는 현재 `true`를 유지한다. 이를 끄면 W1 공간
gate가 준비되기 전에 shortcut 제어권이 시작될 수 있으므로 튜닝 목적으로
변경하지 않는다.

### 실행 시 값 지정

상위 launch를 직접 실행할 때 기존 검증된 명령 뒤에 다음처럼 값을 붙인다.
아래는 motor를 발행하지 않는 shadow 예시다.

```bash
ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  drive_enabled:=false start_shortcut:=true \
  shortcut_spatial_gate_minimum_distance_m:=0.25 \
  shortcut_spatial_gate_response_time_sec:=0.35 \
  shortcut_spatial_gate_blend_distance_m:=0.50 \
  shortcut_w1_steering_start_delay_frames:=4 \
  shortcut_w1_path_weight:=0.60 \
  shortcut_entry_direction_hold_command:=-30.0 \
  shortcut_entry_speed_command:=9.0
```

`run_space_hybrid_test.sh`는 현재 공간 gate의 세 값
(`minimum_distance`, `response_time`, `blend_distance`)을 환경변수로 전달하지
않으므로 이 세 값은 직접 launch할 때만 바뀐다. 이 스크립트에서 지원하는
지름길 환경변수 예시는 다음과 같다.

```bash
SHORTCUT_W1_STEERING_START_DELAY_FRAMES=4 \
SHORTCUT_W1_STEERING_DELAY_MISSING_TOLERANCE_FRAMES=2 \
SHORTCUT_MINIMUM_ENTRY_PROGRESS_M=0.50 \
SHORTCUT_PAIR_TRACK_HANDOFF_REQUIRED_FRAMES=2 \
SHORTCUT_W1_LOSS_HANDOFF_ENABLED=true \
SHORTCUT_MAXIMUM_ENTRY_STEERING_SEC=1.5 \
SHORTCUT_W1_STEERING_HOLD_SEC=1.0 \
SHORTCUT_ENTRY_DIRECTION_HOLD_COMMAND=-30.0 \
SHORTCUT_ENTRY_SPEED_COMMAND=9.0 \
  bash xycar_ws/src/xycar_map_nav/scripts/run_space_hybrid_test.sh
```

조향이 너무 빠르면 먼저 `shortcut_spatial_gate_minimum_distance_m` 또는
`shortcut_spatial_gate_response_time_sec`를 줄인다. 시작 위치는 맞지만 조향이
급하면 `shortcut_spatial_gate_blend_distance_m`를 키운다. 반대로 조향 시작이
늦으면 minimum/response를 키우고, 시작은 맞지만 W1 반영이 느리면 blend
distance를 줄인다. 한 시험에서 이 두 종류를 동시에 바꾸지 않는다.
## 2026-08-14 실차 튜닝 현황

### 현재 구성

- 일반 차선 및 S자 주행: Xbin 노란 중앙선 기반 RULE 제어
- 곡선 제어: Pure Pursuit 80%, Stanley 20%
- 지름길: `left_4` 확인 후 LR-ASPP W1 진입 제어
- 객체 인지 기본 모델: `xycar_ws/src/study/my_rule/models/final.pt`
- 객체 모델 SHA256:
  `0183997ed5e8510045509e7a36bfe69ff3dc8a64840fc822fa8ff8390e90eff5`
- 객체 클래스: `cone`, `green_4`, `green_car`, `left_4`, `null_4`,
  `red_4`, `yellow_4`, `red_car`
- 객체 모델은 차선 Xbin 모델과 독립적으로 동작하므로 일반 차선 경로 생성은
  이번 모델 교체의 영향을 받지 않는다.

### 확인된 주행 결과

다음 순서로 한 단계씩 속도를 높였으며 모두 실차 주행에 성공했다.

| 직선 속도 | 곡선 속도 | 결과 |
|---:|---:|---|
| 20 | 12 | 일반 차선 및 S자 통과 |
| 20 | 14 | S자 통과 |
| 22 | 14 | 통과 |
| 25 | 14 | 통과 |
| 25 | 16 | 통과 |

현재 효과가 확인된 조향값은 다음과 같다.

- 직선 현재 조향 반영 비율: `STEERING_CURRENT_WEIGHT=0.35`
- 곡선 현재 조향 반영 비율: `STEERING_CURVE_CURRENT_WEIGHT=0.80`
- 곡선 속도: `16`
- 경로 품질 저하 시 속도: `15`

직선에서 간헐적으로 직선을 곡선으로 판단하는 현상을 줄이기 위해 곡선 판정
기준을 `0.24 rad/m`로 설정했다. 아래 값은 현재 통합 주행 스크립트와 launch의
기본 프로파일이다.

### 충전 후 재개 명령

충전 후에는 먼저 `/vehicle/vesc_state`의 `fault_code: 0`과 부하 시 전압을
확인한다. 저속 기준 주행을 한 번 통과한 뒤 아래 목표 설정을 시험한다.

```bash
cd /home/xytron/kookmin_ty/yellow_center_curve_test/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source /home/xytron/xycar_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh
```

인자와 환경변수를 생략했을 때 직선/곡선/저품질 경로 속도는 `25/16/15`,
곡선 LD는 `0.30m`, 곡선 Stanley 비율은 `20%`, 좌측 보정은 `0cm`, 곡률
기준은 `0.24 rad/m`, 직선/곡선 조향 현재값 반영 비율은 `0.35/0.80`, RViz는
OFF로 적용된다. 기존과 같이 환경변수나 위치 인자를 주면 개별 값을 덮어쓸 수
있다.

`0.24`에서 완만한 곡선 진입이 늦어지면 `0.22`, 직선 오판이 계속되면
주행 로그를 확보한 뒤 기준을 다시 조정한다. 곡선을 놓친 상태에서 속도
`25`가 유지되면 즉시 시험을 중단한다.

### VESC 저전압 진단

속도 명령 `25`가 정상 발행되는데 차량이 움찔거리며 출발하지 않는 현상을
확인했다. 진단 당시 상태는 다음과 같았다.

- `/xycar_motor` publisher는 `space_drive_gate` 하나로 정상
- VESC USB 및 `/dev/ttyMOTOR` 연결 정상
- 정지 상태 전압 약 `8.9V`
- 부하 순간 전압 `6.0~7.5V`까지 하락
- VESC `fault_code: 2`, `UNDER_VOLTAGE`
- 전압 보호기가 가속을 제한하거나 모터 출력을 차단

따라서 이 현상은 차선 또는 조향 파라미터 문제가 아니다. 보호 전압을 낮추지
말고 배터리를 완전히 충전한 뒤 배터리 셀, 커넥터, 전원 스위치 및 VESC
전원선을 확인한다. 충전 후에도 부하 전압이 `7.5V` 아래로 반복해서 내려가면
고속 시험을 중단한다.

전압과 fault 상태 확인:

```bash
ros2 topic echo /vehicle/vesc_state
```

### 다음 시험 순서

1. 충전 후 저속 주행으로 VESC 전압과 fault 재확인
2. `25/16`, 곡률 기준 `0.24` 재검증
3. 라바콘 진입, 경로 유지, RULE 복귀를 단독 시험
4. 차량 검출, 좌우 판단, 추월 및 원경로 복귀를 단독 시험
5. 전체 통합 주행과 rosbag 검증

한 번의 시험에서는 파라미터 하나만 변경하고, 성공한 값만 기본 설정 후보로
승격한다.
