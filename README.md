# Kookmin Xycar Sim-to-Real Track

Gazebo Sim에서 국민대학교 Xycar 자율주행 트랙을 최대한 비슷하게 재현하고,
실차 Xycar 주행 코드와 맞춰 rule-based 주행, 데이터 수집, 모방학습, Offline RL까지 이어가기 위한 작업 저장소입니다.

현재 트랙 맵, 카메라 차선 인지, 룰베이스 기본 주행까지 구현됐습니다.
시뮬레이션에서 학습한 BC 모델의 실시간 카메라+LiDAR 추론까지 포함되어 있으며,
다음 목표는 **실차 shadow 및 저속 폐루프 검증**입니다.

처음부터 현재 룰베이스 주행까지 전체 구조와 실행 순서를 보려면
[`docs/current_simulation_guide.md`](docs/current_simulation_guide.md)를 먼저 읽습니다.
학습 모델을 실차에서 실행하는 최종 명령과 안전 순서는
[`docs/real_bc_vehicle_runbook.md`](docs/real_bc_vehicle_runbook.md)에 있습니다.

조명·배경·카메라·LiDAR·동역학을 seed별로 바꾸고 차선 이탈 복구 데이터를
자동 수집하려면 [`docs/domain_randomized_collection.md`](docs/domain_randomized_collection.md)를
따릅니다. 기본 5만 장 명령은 5천 장씩 10개 독립 세션을 생성하며 기존
데이터는 유지합니다.

## 실차 Clone 후 바로 실행

아래 절차는 Ubuntu 22.04, ROS 2 Humble, `ROS_DOMAIN_ID=7`인 Xycar
실차 PC를 기준으로 합니다. 저장소에는 룰베이스 패키지와 학습 완료된
TorchScript BC 모델이 함께 들어 있습니다.

### 1. Clone 및 빌드

```bash
cd ~
git clone --branch simulation \
  git@github.com:steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  kookmin_sim_to_real
cd ~/kookmin_sim_to_real

source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-pip python3-opencv python3-numpy \
  ros-humble-cv-bridge ros-humble-vision-msgs
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
export ROS_DOMAIN_ID=7

MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_policy_scripted.pt"
sha256sum "$MODEL"
```

새 터미널마다 다음 환경을 다시 적용합니다.

```bash
cd ~/kookmin_sim_to_real
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
```

### 2. 실차 장치 확인

차량의 기존 카메라, LiDAR, ROS1 모터 컨테이너와 dynamic bridge를 먼저
실행한 뒤 확인합니다.

```bash
ros2 topic hz /wide_camera/rect/image_raw
ros2 topic hz /scan
ros2 topic info /xycar_motor -v
```

현재 룰베이스 기본 계약은 `/wide_camera/rect/image_raw`의
`sensor_msgs/msg/Image`, `/scan`의
`sensor_msgs/msg/LaserScan`, `/xycar_motor`의
`std_msgs/msg/Float32MultiArray [angle, speed]`입니다. `/xycar_motor`에는
실차 모터 bridge subscriber가 있어야 하며 자율주행 publisher는 하나만
존재해야 합니다.

### 3. 룰베이스 실차 테스트

먼저 모터를 움직이지 않는 shadow 모드로 실행합니다.

```bash
ros2 launch xycar_rule_drive real_lane_drive.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  use_compressed_image:=false \
  drive_enabled:=false
```

```bash
ros2 topic echo /xycar_motor_shadow
rqt_image_view /perception/debug_image
```

조향 부호, 차선 검출과 정지 동작을 확인한 뒤 안전요원이 비상 정지를
잡은 상태에서 실측 출발 하한인 속도 명령 3으로 처음 주행합니다.

```bash
ros2 launch xycar_rule_drive real_lane_drive.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  motor_topic:=/xycar_motor \
  drive_enabled:=true \
  speed_command:=3.0
```

### 4. 모방학습 BC 실차 테스트

BC는 카메라와 LiDAR를 모두 사용합니다. 포함된 모델의 입력은 카메라
`3x90x160`, LiDAR `2x360`이며, 출력은 조향 명령입니다. 속도는 안전을
위해 실차 launch가 직선과 곡선 모두 출발 하한인 3.0으로 제한합니다.

```bash
ros2 launch il_data_tools real_policy_inference.launch.py \
  image_topic:=/image_raw \
  scan_topic:=/scan \
  device:=cpu \
  drive_enabled:=false
```

```bash
ros2 topic echo /il/policy_motor_shadow
ros2 topic echo /il/policy_debug
```

shadow 출력, 조향 부호, 카메라 또는 LiDAR 단절 시 0 속도 전환을 확인한
뒤 처음 저속 주행을 시작합니다.

```bash
ros2 launch il_data_tools real_policy_inference.launch.py \
  image_topic:=/image_raw \
  scan_topic:=/scan \
  motor_topic:=/xycar_motor \
  device:=cpu \
  drive_enabled:=true \
  speed_command:=3.0 \
  min_speed_command:=3.0
```

룰베이스, BC, 키보드 조종 노드는 모두 같은 `/xycar_motor`를 사용하므로
절대 동시에 실행하지 않습니다. 첫 실차 테스트는 바퀴를 띄운 정지 시험,
넓은 공간의 직선 저속 시험, 곡선 시험 순서로 진행하며 항상 물리 비상
정지 수단을 준비합니다. 상세 체크리스트는
[`docs/real_bc_vehicle_runbook.md`](docs/real_bc_vehicle_runbook.md)를 봅니다.

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

전체 수집 세션으로 학습한 카메라+LiDAR BC 모델을 시뮬에서 주행시키려면
기존 Gazebo와 룰베이스를 모두 종료한 뒤 아래 한 명령을 실행합니다.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch il_data_tools sim_policy_drive.launch.py
```

이 launch는 Gazebo, 센서 bridge/RViz, BC 추론만 시작합니다. 기본 모델은
`models/il_policies/drive_resnet18_lidar_all_20260713/drive_policy_scripted.pt`이고
주행 속도는 룰베이스와 같은 직선 `4`, 곡선 최저 `3`입니다.

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
