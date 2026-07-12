# Kookmin Xycar Sim-to-Real Track

Gazebo Sim에서 국민대학교 Xycar 자율주행 트랙을 최대한 비슷하게 재현하고,
실차 Xycar 주행 코드와 맞춰 rule-based 주행, 데이터 수집, 모방학습, Offline RL까지 이어가기 위한 작업 저장소입니다.

현재 저장소의 1차 목표는 **트랙 맵 안정화와 실차 조건 정리**입니다.

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
현재 값은 실차를 완전히 보정한 값이 아니라 rule-based 주행과 센서 파이프라인을 먼저 붙이기 위한 시작점입니다.

- 실차 프레임 기준: 약 `0.55 m x 0.30 m x 0.25 m`
- Gazebo 차체 충돌 박스: `0.55 m x 0.30 m x 0.12 m`
- 바퀴 포함 외폭: 약 `0.30 m`
- wheelbase: `0.32 m`
- wheel separation: `0.265 m`
- wheel radius: `0.06 m`
- steering limit: `0.289 rad`
- speed command scale: `speed_mps = 0.08 * speed_command`
- steering command scale: `steering_rad = -0.0068 * angle_command`
- command clamp: `angle -50~100`, `speed -50~100`
- velocity range from command clamp: `-4.0 ~ 8.0 m/s`
- camera: `/image_raw`, 1280x1024, 30Hz, HFOV 약 170도, equidistant fisheye lens
- lidar: `/scan`, 505 samples, 10Hz, -180~180도, range `0.1~12.0 m`
- sensor origin: 앞바퀴 중심 기준
- camera pose from front wheel center: `(-0.04, 0.00, 0.17) m`
- lidar pose from front wheel center: `(0.08, 0.00, 0.06) m`

실차와 맞추기 위한 측정 항목과 ROS2 확인 명령은
`docs/sim_to_real_vehicle_calibration.md`에 정리되어 있습니다.

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

1. 실차에서 wheelbase, 카메라/라이다 장착 위치를 실측
2. Gazebo 카메라 pitch/FOV를 실제 `/image_raw` 샘플과 맞게 보정
3. Gazebo 라이다 index/각도 방향을 실제 `/scan`과 맞게 보정
4. rule-based 차선 주행 노드 작성
5. 주행 로그와 이미지/라이다 데이터를 수집
6. Behavioral Cloning 학습 후 Offline RL과 supervisor 구조로 확장
