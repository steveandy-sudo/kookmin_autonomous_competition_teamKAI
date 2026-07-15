# Kookmin Xycar Sim-to-Real Track

Gazebo Sim에서 국민대학교 Xycar 자율주행 트랙을 최대한 비슷하게 재현하고,
실차 Xycar 주행 코드와 맞춰 rule-based 주행, 데이터 수집, 모방학습, Offline RL까지 이어가기 위한 작업 저장소입니다.

현재 트랙 맵, 카메라 차선 인지, 룰베이스 기본 주행까지 구현됐습니다.
시뮬레이션에서 학습한 BC 모델의 실시간 카메라+LiDAR 추론까지 포함되어 있으며,
다음 목표는 **실차 shadow 및 저속 폐루프 검증**입니다.

## 2026-07-13 12:00 이후 작업과 현재 인수인계

7월 13일 정오 이후 실차 동역학 반영, 실차 카메라 BEV 보정, 룰베이스 완성,
raw RGB 모방학습과 실차 이식 시험을 진행했습니다. 실차에서 raw RGB 모델의
조향이 시뮬레이션과 다르게 동작한 원인을 카메라 색·배경·왜곡의 domain gap으로
판단했고, 현재는 시뮬과 실차를 동일한 `256x144` canonical BEV 차선 영상으로
바꾸는 파이프라인을 사용합니다.

현재 canonical 물리 계약은 좌우 `1.4 m`, 전방 `1.5 m`입니다. 1.0 m에서
중앙 점선이 한 개만 보이고 2.0 m에서 3~4개가 보이던 문제를 절충해, 실차와
시뮬 모두 한 화면에 중앙 점선 2~3개가 들어오도록 보정했습니다. 검증값은
[`docs/2026-07-15_1p5m_canonical_validation.md`](docs/2026-07-15_1p5m_canonical_validation.md)에 있습니다.
현재 canonical 모델 입력은 관측 전용입니다. 현재 카메라 프레임에 보인 선만
출력하며, 사라진 흰 경계를 합성하거나 이전 프레임의 차선을 유지하지 않습니다.

작업 시간순 기록, 발생한 문제와 수정 근거, canonical 수집·학습 상태, 실차 PC의
Codex가 바로 따라야 할 파일과 명령은
[`docs/2026-07-13_canonical_sim_to_real_handoff.md`](docs/2026-07-13_canonical_sim_to_real_handoff.md)에
모두 정리했습니다. 학습이 통과하면 최신 모델 해시와 실행 명령은
[`docs/canonical_model_latest.md`](docs/canonical_model_latest.md)에 자동 기록됩니다.

처음부터 현재 룰베이스 주행까지 전체 구조와 실행 순서를 보려면
[`docs/current_simulation_guide.md`](docs/current_simulation_guide.md)를 먼저 읽습니다.
기존 raw RGB 모델의 실차 명령은
[`docs/real_bc_vehicle_runbook.md`](docs/real_bc_vehicle_runbook.md)에 남겨 두었지만,
새 sim-to-real 시험에는 raw runbook이 아니라 아래 canonical 인수인계 문서를
사용합니다.

조명·배경·카메라·LiDAR·동역학을 seed별로 바꾸고 차선 이탈 복구 데이터를
자동 수집하려면 [`docs/domain_randomized_collection.md`](docs/domain_randomized_collection.md)를
따릅니다. 최신 모델은 canonical BEV를 5천 장씩 6개 독립 세션에 저장한
30,000장으로 학습했으며, 각 세션에 차선 이탈 후 복귀 데이터와 실차에서
관측한 중앙선 위치 튐·흰 경계 휨·차선 소실을 포함합니다.

## 실차 Clone 후 Canonical BC 실행

이 절차는 Ubuntu 22.04, ROS 2 Humble, `ROS_DOMAIN_ID=7`인 실차 Xycar를
기준으로 합니다. **Gazebo는 실차 PC에서 실행하지 않습니다.** 실차에서는
카메라를 canonical BEV로 바꾸는 `xycar_perception`과 TorchScript 정책을
실행하는 `il_data_tools`만 사용합니다.

최신 모델 상태:

```text
파일: drive_canonical_policy_scripted.pt
SHA-256: dd8cb6c2ccfca5a438b08e90f930f50527a88b5169cae9e8239dd129f78dbb21
학습: canonical 30,000장 (정상 21,547 / 복귀 8,453)
실차형 canonical 오류 이벤트: 1,041회
정지 샘플: 0장, 정지 직전 불량 샘플 폐기: 433장
독립 테스트: 5,000장
전체 조향 MAE: 3.379, 정상 MAE: 3.031, 복귀 MAE: 4.284
```

오프라인 평가와 시뮬 초기 구동은 통과했지만 아직 실차 완주를 보장하는 모델은
아닙니다. 실차에서는 아래 shadow 검증을 생략하지 않습니다.

### 1. Clone, 업데이트 및 빌드

처음 받는 실차 PC:

```bash
cd ~
git clone --branch simulation \
  https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  kookmin_sim_to_real
cd ~/kookmin_sim_to_real

export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y python3-pip python3-opencv python3-numpy \
  ros-humble-cv-bridge ros-humble-vision-msgs \
  ros-humble-rqt-image-view

python3 -c "import torch; print(torch.__version__)" || \
  python3 -m pip install --user torch torchvision

rosdep install --from-paths \
  xycar_ws/src/kaiev26_msgs \
  xycar_ws/src/xycar_perception \
  xycar_ws/src/xycar_rule_drive \
  xycar_ws/src/il_data_tools \
  --ignore-src -r -y

colcon build --packages-select \
  kaiev26_msgs xycar_perception xycar_rule_drive il_data_tools \
  --symlink-install
source install/setup.bash
```

이미 clone되어 있다면 임의 로컬 수정을 먼저 확인하고 fast-forward로 받습니다.

```bash
cd ~/kookmin_sim_to_real
git status -sb
git switch simulation
git pull --ff-only origin simulation

source /opt/ros/humble/setup.bash
colcon build --packages-select \
  kaiev26_msgs xycar_perception xycar_rule_drive il_data_tools \
  --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=7
```

모델이 정확한지 반드시 확인합니다.

```bash
MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_canonical_policy_scripted.pt"
test -f "$MODEL"
sha256sum "$MODEL"
```

출력 해시는 위의 `dd8cb6...dbb21`과 같아야 합니다. 새 터미널마다 다음 환경을
다시 적용합니다.

```bash
cd ~/kookmin_sim_to_real
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 2. 실차 토픽과 카메라 형식 판별

차량의 기존 카메라, LiDAR, ROS1 VESC 컨테이너와 dynamic bridge를 평소 방식으로
먼저 실행합니다. 그 뒤 실차 Codex는 다음 결과를 저장하고 확인합니다.

```bash
ros2 topic list -t | grep -E 'wide_camera|image_raw|compressed|scan|xycar_motor'
ros2 topic info /wide_camera/rect/image_raw -v
ros2 topic info /scan -v
ros2 topic info /xycar_motor -v
ros2 topic hz /wide_camera/rect/image_raw
ros2 topic hz /scan
```

기본 계약은 다음과 같습니다.

```text
/wide_camera/rect/image_raw  sensor_msgs/msg/Image, 권장 1280x1024 약 30Hz
/scan                        sensor_msgs/msg/LaserScan, 약 10Hz
/xycar_motor                 std_msgs/msg/Float32MultiArray [angle, speed]
```

카메라 입력별 launch 설정:

| 실차 카메라 토픽 | 설정 |
|---|---|
| 이미 보정된 `sensor_msgs/Image` | `source_image_topic:=/wide_camera/rect/image_raw enable_rectify:=false use_compressed_image:=false` |
| 보정 전 fisheye `sensor_msgs/Image` | 실제 토픽과 `enable_rectify:=true use_compressed_image:=false` |
| `sensor_msgs/CompressedImage` | 실제 토픽과 `use_compressed_image:=true`; 보정 전이면 `enable_rectify:=true` |

이미 rectified인 영상을 다시 보정하면 BEV가 심하게 휘므로
`/wide_camera/rect/image_raw`에는 `enable_rectify:=false`를 유지합니다.

### 3. Canonical 인지만 단독 검증

모터를 발행하지 않는 perception만 먼저 실행합니다.

```bash
ros2 launch xycar_perception real_canonical_perception.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false \
  use_compressed_image:=false
```

다른 터미널에서 출력 주기와 화면을 확인합니다.

```bash
ros2 topic hz /perception/canonical_road_image
ros2 topic echo --once /perception/canonical_road_image | \
  grep -E 'height:|width:|encoding:'
ros2 run rqt_image_view rqt_image_view /perception/canonical_road_image
```

RViz로 source, BEV, mask와 경로를 함께 보려면 다음처럼 표시 설정만 엽니다.

```bash
rviz2 -d "$(ros2 pkg prefix xycar_rule_drive)/share/xycar_rule_drive/rviz/real_lane_drive.rviz"
```

정상 canonical 계약:

```text
크기 256x144 bgr8
범위 좌우 1.4m, 전방 1.5m
배경 BGR (36,36,36)
흰 경계 BGR (255,255,255), 5px
노란 중앙선 BGR (0,220,255), 5px
흰선 중심 간격 약 0.824m, 영상에서 약 151px
30cm 중앙 점선 길이 약 29px
```

직선에서 선이 크게 휘거나 조명 반사가 흰선이 되거나 벽이 중앙선으로 분류되면
모델을 실행하지 않습니다. 먼저
`xycar_ws/src/xycar_perception/config/camera_perception_real.yaml`의 homography와
HSV 임계값을 실차 영상에 맞추고, 수정 전후 rosbag과 canonical 화면을 남깁니다.

### 4. 통합 Shadow 추론

단독 perception을 `Ctrl+C`로 종료한 뒤 아래 통합 launch를 실행합니다. 이 launch는
실차 canonical perception과 모델을 함께 띄우지만 기본값으로 `/xycar_motor`에는
발행하지 않습니다.

```bash
MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_canonical_policy_scripted.pt"

ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  model_path:="$MODEL" \
  source_image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false \
  use_compressed_image:=false \
  scan_topic:=/scan \
  drive_enabled:=false \
  device:=cpu
```

별도 터미널에서 확인합니다.

```bash
ros2 topic hz /perception/canonical_road_image
ros2 topic hz /il/policy_motor_shadow
ros2 topic echo /il/policy_motor_shadow
ros2 topic echo /il/policy_debug
ros2 run rqt_image_view rqt_image_view /il/policy_input_image
```

`/il/policy_input_image`는 모델이 실제로 받은 `160x90` canonical 영상입니다.
`/il/policy_debug` 배열은 다음 순서입니다.

```text
[0] normalized steering
[1] raw Xycar angle command
[2] temporal-filtered angle command
[3] speed command
[4] image-LiDAR timestamp offset (ms)
[5] inference latency (ms)
[6] inference count
[7] missing synchronized scan count
[8] source width
[9] source height
[10] normalized input mean
```

필수 shadow 통과 조건:

1. `/il/policy_motor_shadow`가 약 8~10Hz 이상 지속적으로 나온다.
2. timestamp offset은 기본 `50ms` 안이며 missing scan count가 계속 증가하지 않는다.
3. 카메라나 LiDAR를 끊으면 0.5초 안에 `[0.0, 0.0]`이 나온다.
4. 오른쪽 경로에서 오른쪽 바퀴 방향, 왼쪽 경로에서 왼쪽 바퀴 방향 명령이다.
5. 조향은 `-42~42` 안이고 NaN이나 순간적인 포화가 반복되지 않는다.

방향이 반대면 코드나 학습 데이터를 바꾸기 전에
`steering_output_sign:=-1.0`으로 shadow를 다시 확인합니다. 동기화가 실패할 때만
`sync_tolerance_sec:=0.08`처럼 조금 늘리고, 원인을 기록합니다.

#### 임시 조립식 트랙 profile

`track_run_02`를 촬영한 임시 트랙은 노란 중앙선에서 흰 경계선까지 약
`0.49m`이며, 본선 트랙의 `0.412m`와 다릅니다. 이 트랙에서만 다음 profile을
지정합니다. 바닥 이음새와 곡선 노란 점선 필터도 함께 적용됩니다.

```bash
MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_canonical_policy_scripted.pt"

ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  perception_launch_file:=real_temp_track_canonical_perception.launch.py \
  model_path:="$MODEL" \
  source_image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false \
  use_compressed_image:=false \
  scan_topic:=/scan \
  drive_enabled:=false \
  device:=cpu
```

RViz에서는 `Rectified Camera`, `Canonical Model Input`,
`Canonical Tracking Debug`를 켜고 `/il/policy_motor_shadow`의 조향 부호를 함께
확인합니다. Snap으로 설치한 VS Code 터미널에서 RViz 라이브러리 충돌이 나면
다음 명령을 사용합니다.

```bash
env -u GTK_PATH -u GTK_EXE_PREFIX -u GIO_MODULE_DIR \
  -u GTK_IM_MODULE_FILE -u SNAP -u SNAP_LIBRARY_PATH \
  rviz2 -d "$(ros2 pkg prefix xycar_rule_drive)/share/xycar_rule_drive/rviz/real_lane_drive.rviz"
```

본선 트랙에서는 `perception_launch_file` 인자를 빼고 기본
`real_canonical_perception.launch.py`를 사용합니다.

본선 로스백을 본선 profile로 RViz에서 다시 처리할 때는 다음 통합 launch를
사용합니다. 임시 트랙의 `0.49m`, 밝기 `V 140`, 노란선 `0.08m` 필터는 적용되지
않고 본선의 `0.412m` 계약과 공통 추적기 개선만 적용됩니다.

```bash
ros2 launch xycar_rule_drive real_competition_track_bag_rviz.launch.py
```

### 5. 실차 저속 주행

다음을 모두 만족한 뒤에만 모터 출력을 켭니다.

- 물리 비상 정지 담당자가 차량 옆에 있음
- 바퀴를 띄운 상태의 조향 부호와 watchdog 정지 검증 완료
- `/xycar_motor`의 자율주행 publisher가 현재 launch 하나뿐임
- 룰베이스, 키보드, 기존 모방학습 노드가 모두 종료됨
- `/xycar_motor`에 VESC bridge subscriber가 존재함

```bash
ros2 topic info /xycar_motor -v

MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_canonical_policy_scripted.pt"

ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  model_path:="$MODEL" \
  source_image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false \
  use_compressed_image:=false \
  scan_topic:=/scan \
  motor_topic:=/xycar_motor \
  drive_enabled:=true \
  speed_command:=3.0 \
  max_steer_scale:=100.0 \
  steering_output_sign:=1.0 \
  steering_temporal_alpha:=0.55 \
  sensor_timeout_sec:=0.50 \
  device:=cpu
```

첫 시험 순서는 바퀴를 띄운 상태, 직선 2~3m, 완만한 단일 곡선, 전체 트랙입니다.
모델은 조향만 예측하며 속도 감속, `-42~42` 제한과 sensor timeout 정지는 코드가
담당합니다. 종료는 `Ctrl+C`이며 필요하면 즉시 정지 명령을 한 번 더 보냅니다.

```bash
ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0]}"
```

임시 조립식 트랙에서 shadow 검증을 통과한 뒤 저속으로 전환할 때는 위 명령에
profile 인자 하나를 추가합니다.

```bash
ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  perception_launch_file:=real_temp_track_canonical_perception.launch.py \
  model_path:="$MODEL" \
  source_image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false \
  use_compressed_image:=false \
  scan_topic:=/scan \
  motor_topic:=/xycar_motor \
  drive_enabled:=true \
  speed_command:=3.0 \
  max_steer_scale:=100.0 \
  steering_output_sign:=1.0 \
  steering_temporal_alpha:=0.55 \
  sensor_timeout_sec:=0.50 \
  device:=cpu
```

첫 실차 시험에서 이번 인지 수정의 동작은 확인할 수 있지만, 기존 모델은 수정된
임시 트랙 실차 canonical 영상으로 다시 학습한 모델이 아닙니다. 반드시 shadow
출력을 먼저 확인하고 즉시 전체 속도로 올리지 않습니다.

실차 Codex는 첫 시험 후 canonical 원본/모델 입력 화면, `/il/policy_debug`,
`/il/policy_motor_shadow`, 실제 바퀴 방향과 지연을 함께 보고해야 합니다. 조향량이
일관되게 클 때만 `max_steer_scale`을 줄이고, canonical 형상이나 모델 예측 자체가
틀리면 gain으로 숨기지 말고 인지 보정 또는 실차 canonical 데이터 fine-tuning을
진행합니다.

## 현재 상태

- 최종 Gazebo 월드: `worlds/kookmin_xycar_track_final.sdf`
- Gazebo에서 다시 저장한 작업본: `worlds/kookmin_xycar_track_gz.sdf`
- 흰색 도로 경계선: 개별 Gazebo 객체로 분리되어 Entity Tree에서 이동/조정 가능
- 노란색 중앙 점선: 개별 Gazebo 객체로 분리되어 이동/조정 가능
- 차량: `xycar_ackermann` 모델이 월드 안에 미리 스폰됨
- Gazebo 센서: `/image_raw`, `/camera_info`, `/scan` 토픽 기준으로 카메라/라이다 추가
- 실내 시각 기준: 실차 카메라 영상에 맞춰 벽, 나무 몰딩, 락커, 책상/의자, 천장 조명, 콘을 `room_*` 독립 객체로 추가
- 실차 모터 wrapper: `/xycar_motor` 또는 `xycar_motor`의 `Float32MultiArray [angle, speed]`를 Gazebo Ackermann `cmd_vel`로 변환
- 실차 코드 참고본: `xycar_ws/src`
- 실차-시뮬레이션 보정 문서: `docs/sim_to_real_vehicle_calibration.md`
- hwj 카메라/차선 코드 비교: `docs/hwj_camera_lane_analysis.md`

## 실행

저장소 루트에서 실행합니다.

```bash
export GZ_SIM_RESOURCE_PATH=$PWD:${GZ_SIM_RESOURCE_PATH}
gz sim -r worlds/kookmin_xycar_track_final.sdf
```

Gazebo GUI가 안 뜨거나 빈 화면이면 기존 `gz sim` 프로세스가 남아 있는지 먼저 확인합니다.

```bash
pkill -f "gz sim" || true
export GZ_SIM_RESOURCE_PATH=$PWD:${GZ_SIM_RESOURCE_PATH}
gz sim -r worlds/kookmin_xycar_track_final.sdf
```

`libEGL warning: egl: failed to create dri2 screen` 경고는 그래픽 드라이버/렌더링 경고입니다.
창이 열리고 월드가 보이면 이 경고 자체는 치명적이지 않습니다.

실차 코드와 같은 `/xycar_motor` 명령으로 Gazebo 차량을 움직이려면 다른 터미널에서 bridge를 실행합니다.

```bash
source /opt/ros/humble/setup.bash
cd xycar_ws
colcon build --packages-select xycar_gazebo_bridge
export AMENT_PREFIX_PATH=$PWD/install/xycar_gazebo_bridge:${AMENT_PREFIX_PATH}
source install/xycar_gazebo_bridge/share/xycar_gazebo_bridge/package.bash
ros2 launch xycar_gazebo_bridge xycar_gazebo_bridge.launch.py
```

카메라, 라이다, 인지 결과를 RViz에서 같이 보려면 위 bridge 대신 아래 launch를 실행합니다. 이 launch는 Gazebo bridge, `/xycar_motor` wrapper, 카메라 기반 인지 노드, RViz를 함께 띄웁니다.

```bash
source /opt/ros/humble/setup.bash
cd xycar_ws
colcon build --packages-select kaiev26_msgs xycar_perception xycar_gazebo_bridge --symlink-install
export AMENT_PREFIX_PATH=$PWD/install/xycar_gazebo_bridge:${AMENT_PREFIX_PATH}
source install/xycar_gazebo_bridge/share/xycar_gazebo_bridge/package.bash
ros2 launch xycar_gazebo_bridge xycar_gazebo_rviz.launch.py
```

RViz 표시 토픽:

```bash
ros2 topic hz /image_raw
ros2 topic hz /scan
ros2 topic hz /perception/centerline
```

테스트 명령:

```bash
ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray "{data: [20.0, 10.0]}"
```

카메라 기반 인지 노드는 Gazebo 카메라 `/image_raw`를 직접 처리해서 KAIEV perception mock과 같은 메시지 계약으로 결과를 발행합니다. 이 노드는 semantic mock 데이터를 쓰지 않습니다.

```bash
source /opt/ros/humble/setup.bash
cd xycar_ws
colcon build --packages-select kaiev26_msgs xycar_perception --symlink-install
source install/setup.bash
ros2 launch xycar_perception camera_perception.launch.py
```

출력 토픽:

```bash
/perception/road_segments     kaiev26_msgs/msg/RoadSegmentArray
/perception/centerline        kaiev26_msgs/msg/Centerline
/perception/objects           kaiev26_msgs/msg/PerceptionObjectArray
/perception/traffic_lights    kaiev26_msgs/msg/TrafficLightObservationArray
/perception/debug_image       sensor_msgs/msg/Image
/perception/debug_markers     visualization_msgs/msg/MarkerArray
```

현재 검출 대상은 흰색 차선과 노란 중앙 점선입니다. `/perception/road_segments`에는 실제 검출된 흰/노란 차선이 그대로 나가고, `/perception/centerline`은 기본적으로 `centerline_mode: lane_midline`을 사용해 노란 중앙선과 바깥쪽 흰 차선 사이의 주행 중심 경로를 냅니다. 그래서 곡선 구간에서는 흰 차선의 곡률을 반영한 중심 경로가 생성됩니다.
카메라 인지 좌표 변환은 현재 실차 `gsw` 브랜치의 광각 카메라 흐름처럼 `projection_mode: bev_homography`를 기본으로 씁니다. raw 이미지에서 사다리꼴 ROI를 잡고 BEV로 펼친 뒤, 그 BEV 이미지에서 흰 차선과 노란 중앙선을 검출합니다. 튜닝 값은 `xycar_perception/config/camera_perception.yaml`의 `src_*_ratio`, `bev_width`, `bev_height`, `lateral_m_per_px`, `forward_m_per_px`입니다.

실차 코드와 같은 BEV 프리뷰 창을 따로 보고 싶으면 아래를 실행합니다. Gazebo 입력은 `/image_raw` raw 이미지이므로 실차의 `/wide_camera_mjpeg/image_raw/compressed` 대신 `use_compressed: false` 설정을 씁니다.

```bash
source /opt/ros/humble/setup.bash
cd xycar_ws
source install/setup.bash
ros2 launch lane_bev_tools sim_bev_preview.launch.py
```

카메라 인지 결과로 rule-based 주행을 시작하려면 별도 터미널에서 아래 노드를 실행합니다. 이 노드는 `/perception/road_segments`의 노란 중앙선과 바깥쪽 흰 차선 사이 중앙을 `/rule_drive/target_path`로 만들고, `/xycar_motor`에 `[angle, speed]` 명령을 발행합니다.

```bash
source /opt/ros/humble/setup.bash
cd xycar_ws
colcon build --packages-up-to lane_bev_tools xycar_perception xycar_gazebo_bridge xycar_rule_drive --symlink-install
source install/setup.bash
ros2 launch xycar_rule_drive lane_rule_driver.launch.py
```

첫 정지 테스트만 하고 싶으면 속도를 0으로 덮어씁니다.

```bash
ros2 run xycar_rule_drive lane_rule_driver --ros-args -p speed_command:=0.0 -p min_speed_command:=0.0
```

최신 canonical 카메라+LiDAR BC 모델을 시뮬에서 주행시키려면
기존 Gazebo와 룰베이스를 모두 종료한 뒤 아래 한 명령을 실행합니다.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch il_data_tools sim_policy_drive.launch.py
```

이 launch는 Gazebo, 센서 bridge/RViz, canonical perception과 BC 추론만
시작합니다. 기본 모델은 패키지의 `drive_canonical_policy_scripted.pt`이고
입력 토픽은 `/perception/canonical_road_image`입니다. 룰베이스는 동시에
실행하지 않습니다.

2026-07-14 실차 rosbag의 조명 반사와 compressed fisheye 입력을 반영한
canonical 보정 및 안전한 재생 절차는
[`docs/2026-07-14_real_camera_canonical_tuning.md`](docs/2026-07-14_real_camera_canonical_tuning.md)에 정리되어 있습니다.

수동 조종은 rule-based 주행 노드를 끄고 아래처럼 실행합니다. 같은 `/xycar_motor` 토픽을 쓰므로 `lane_rule_driver`와 동시에 실행하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
cd xycar_ws
source install/setup.bash
ros2 run xycar_rule_drive keyboard_teleop
```

키 조작:

```text
w/s      speed up / slow down
a/d      steer left / steer right
e        center steering
x        speed zero
space    full stop
q        quit
```

작년 국민대 본선 예제의 카메라 기반 rule 주행 방식도 별도 노드로 포팅했습니다. 이 노드는 KAIEV perception 메시지를 거치지 않고, Gazebo 카메라 `/image_raw`를 직접 받아 BEV 변환, Canny, HoughLinesP, 차선 후보 클러스터링, 이전 프레임 예측을 거쳐 곧바로 `/xycar_motor`에 `[angle, speed]`를 발행합니다.

```bash
source /opt/ros/humble/setup.bash
cd xycar_ws
colcon build --packages-select xycar_rule_drive xycar_gazebo_bridge --symlink-install
source install/setup.bash
ros2 launch xycar_gazebo_bridge xycar_legacy_camera_drive_rviz.launch.py
```

같은 `/xycar_motor`를 쓰기 때문에 `keyboard_teleop`, `lane_rule_driver`와 동시에 실행하지 않습니다. 전용 RViz 설정은 raw camera, LiDAR, TF, 그리고 `/rule_drive/legacy_debug_image`를 중심으로 보여줍니다.

## 실차 실행 프로필

실차에서는 Gazebo bridge를 실행하지 않고, 기본적으로 모터 출력을 차단한
통합 launch를 사용합니다. raw/압축 카메라 선택, shadow 검증, 저속 auto
전환 절차는 [실차 배포 가이드](docs/real_vehicle_deployment.md)에 정리되어 있습니다.

```bash
ros2 launch xycar_rule_drive real_lane_drive.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  use_compressed_image:=false \
  drive_enabled:=false
```

## 맵 기준

트랙은 CAD/DXF에서 가져온 차선 데이터를 바탕으로 만들었습니다.
사용자가 Gazebo에서 직접 조정한 최종 위치를 보존하기 위해 `final.sdf`를 기준 파일로 둡니다.

- 도로 바닥: 실제 도로처럼 회색 톤
- 흰색 경계선 두께: `0.024 m`
- 노란색 중앙선 두께: `0.024 m`
- 노란색 중앙 점선: `0.30 m` 길이, `0.30 m` 간격 기준
- 흰색 경계선 안쪽 기준 도로 폭: 최종 저장본 기준 대략 `0.76~0.84 m`
- 전체 기준 크기: 설계도 외곽 `20.150 m x 11.350 m`를 기준으로 정합
- S자 곡선: CAD에서 추출한 흰색 라인의 형태를 유지한 뒤 Gazebo 객체로 분리
- 실내 배경 객체: `room_north_wall`, `room_locker_bank`, `room_table_top_*`, `room_chair_*`, `room_ceiling_light_*`처럼 각각 독립 모델이라 Entity Tree에서 따로 이동 가능
- 차량 초기 위치: 상단 직선 도로의 한쪽 차로 `x=-2.70`, `y=2.25`, `yaw=0`

주의: `scripts/generate_kookmin_track.py`를 다시 실행하면 Gazebo에서 수동으로 저장한 차선 위치가 덮어써질 수 있습니다.
최종 맵을 수정할 때는 먼저 `worlds/kookmin_xycar_track_final.sdf`를 백업한 뒤 진행합니다.

## 차량 모델

월드에는 `xycar_ackermann`이 포함되어 있습니다.
차량 형상은 실측 치수를, 명령 응답은 2026-07-12 및 2026-07-13 실차 주행 로그를 기준으로 보정했습니다.

- 실차 프레임 기준: 약 `0.55 m x 0.30 m x 0.25 m`
- Gazebo 차체 충돌 박스: `0.55 m x 0.30 m x 0.12 m`
- 바퀴 포함 외폭: 약 `0.30 m`
- wheelbase: `0.32 m`
- wheel separation: `0.265 m`
- wheel radius: `0.06 m`
- steering center limit: `0.560 rad` (조향 joint limit `0.700 rad`)
- speed command scale: `speed_mps = 0.080612 * speed_command`
- speed launch threshold: `abs(speed_command) < 3`이면 정지
- steering response: 실측 `+-10/20/30/35/40/42` 좌우 비대칭 곡률 테이블
- measured steering delay: `0.10 s`
- measured speed delay: `0.20 s`
- speed response: 가속 `tau=0.19 s`, 감속 `tau=0.09 s` 1차 지연
- command clamp: `angle -50~100`, `speed -50~100`
- velocity range from command clamp: `-4.0 ~ 8.0 m/s`
- camera: `/image_raw`, 1280x1024, 30Hz, 보정 영상 유효 HFOV 약 102.95도, equidistant fisheye lens
- lidar: `/scan`, 505 samples, 10Hz, -180~180도, range `0.1~12.0 m`
- sensor origin: 앞바퀴 중심 기준
- camera pose from front wheel center: `(-0.04, 0.00, 0.17) m`
- lidar pose from front wheel center: `(0.065, 0.00, 0.080) m`

실차와 맞추기 위한 측정 항목과 ROS2 확인 명령은
`docs/sim_to_real_vehicle_calibration.md`에 정리되어 있습니다.
실차 원본 그룹 요약과 적용 근거는 `data/vehicle_dynamics/2026-07-12`와
`teamkai/data/vehicle-dynamics-20260713` 브랜치의 `data/vehicle_dynamics/2026-07-13`에 있습니다.

## 실차 코드에서 확인한 ROS2 기준

`xycar_ws/src`에 복사된 실차 주행 코드를 기준으로 정리한 내용입니다.

- 카메라 토픽: `/image_raw`
- 라이다 토픽: `/scan`
- 초음파 토픽: `xycar_ultrasonic`
- 모터 토픽 후보: `/xycar_motor` 또는 `xycar_motor`
- 현재 예제 코드 대부분의 모터 메시지: `std_msgs/msg/Float32MultiArray`
- 모터 데이터 순서: `[angle, speed]`
- 일부 예전/시뮬 코드의 모터 메시지: `xycar_msgs/msg/XycarMotor`

실차에서 제일 먼저 확인할 명령입니다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash

ros2 topic list -t
ros2 topic info /image_raw
ros2 topic info /scan
ros2 topic info /xycar_motor
ros2 topic info xycar_motor
```

## 주요 파일 구조

```text
.
├── worlds/
│   ├── kookmin_xycar_track_final.sdf      # 최종 Gazebo Sim 월드
│   └── kookmin_xycar_track_gz.sdf         # Gazebo 저장 작업본
├── media/materials/textures/              # 트랙/차선 텍스처
├── scripts/
│   ├── generate_kookmin_track.py          # 맵 생성 스크립트
│   └── blueprint_alignment_report.py      # 설계도 정합 검증 스크립트
├── docs/
│   └── sim_to_real_vehicle_calibration.md # 실차-시뮬레이션 보정 문서
├── xycar_ws/src/                          # 실차 ROS2 주행 코드 참고본
│   └── xycar_gazebo_bridge/               # 실차 motor command -> Gazebo cmd_vel wrapper
├── preview_topdown.png                    # 트랙 미리보기
└── alignment/fit_report.txt               # 설계도/생성맵 정합 리포트
```

## 다음 단계

1. 시뮬 룰베이스 반복 주행으로 실패 구간과 명령 로그 수집
2. 실차 shadow mode에서 카메라 BEV, 차선 검출, 조향 부호 검증
3. 실차 최대 조향 반경, 저속 deadband, 가감속 응답 추가 측정
4. 이미지, 인지 경로, 모터 명령을 동기화한 학습 데이터 수집
5. Behavioral Cloning 학습과 closed-loop 평가
6. Offline RL 및 안전 supervisor 구조로 확장
