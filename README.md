# Kookmin Autonomous Competition Team KAI

국민대 자율주행대회용 Xycar RULE·콘·차량 회피 통합 저장소다. 현재 실차
통합 브랜치는 `jsb`이며, 제어 우선순위는
`CONE_RULE > YOLO_LIDAR_AVOIDANCE > RULE`이다.

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
