# Xycar LiDAR parking navigation

국민대 Xycar 주차경기의 제공 PGM/YAML 지도를 사용해 다음 미션을 수행하는
ROS 2 Humble 패키지입니다.

```text
Start (1.8, 0.9, pi)
  -> A 후진주차 (0.0, 4.2, 0.0)
  -> B 평행주차 (2.1, 3.3, -pi/2)
  -> Start 복귀
```

주행 중 새 지도를 만드는 SLAM 패키지가 아니라, 제공된 정적 지도와 2D
LiDAR를 이용하는 `map_server + AMCL` localization 구조입니다. 전역/주차
경로는 `SmacPlannerHybrid (REEDS_SHEPP)`, 추종은 후진 가능한
`MPPIController (Ackermann)`를 사용합니다. 공식 A/B/Start Pose는 그대로 두고,
조향 서보가 따라갈 수 있도록 곡률 변화점과 전진/후진 cusp를 포함한 31개 단계로
미션을 분할했습니다.

`/slam/odom`, `/slam/scan_filtered`는 기존 차량 인터페이스와 맞춘 topic 이름일
뿐이며, visual SLAM이나 `slam_toolbox` 노드는 실행하지 않습니다.

## 안전 기본값

- `drive_enabled:=false`가 기본값이며 `/xycar_motor`로 실차 명령을 보내지
  않습니다.
- AMCL covariance, 갱신시각, pose jump를 모두 통과해야 모터 권한이 열립니다.
- `/cmd_vel` 또는 `/scan`이 stale이면 즉시 정지합니다.
- 곡선 주행 중 차량 전체 polygon의 예상 정지 궤적과 LiDAR 점을 독립적으로
  충돌 검사합니다.
- 전진/후진이 바뀔 때 0.4초 정지하고 조향을 미리 정렬합니다.
- 실측 조향 lookup의 약한 쪽을 기준으로 최소 회전반경을 0.67 m로 제한합니다.
- Ackermann 차량이 수행할 수 없는 rotate-in-place 명령은 거부합니다.
- Nav2 복구 Behavior Tree에서도 spin 동작을 제거했습니다.

## 지도와 좌표

`maps/parking_map_original.yaml`은 대회 원본 값입니다. 원본의
`free_thresh: 0.25`는 회색 205 픽셀을 FREE로 해석하므로, 실행용
`maps/parking_map.yaml`은 `free_thresh: 0.196`을 사용해 회색을 UNKNOWN으로
보존합니다. `resolution`과 `origin`은 원본 그대로입니다.

도면의 십자는 차량 기하 중심이지만 실차의 `base_footprint`는 전륜 중심입니다.
`config/parking_mission.yaml`의 `base_from_reference_x_m: 0.16`이 모든 공식
Pose를 TF 기준 Pose로 변환합니다. 이 값은 국민대 Gazebo 생성기의 전·후륜
위치 `x=+0.16/-0.16 m`와 일치합니다.

차량 형상도 같은 생성기의 실차 측정값을 사용합니다. Wheelbase `0.32 m`, 중앙
차체 `0.50 x 0.20 m`, 직진 타이어 외폭 `0.28 m`, 휠 반경 `0.06 m`, 전륜축
기준 LiDAR `x=+0.065 m`입니다. 차체·LiDAR·최대 조향 타이어를 모두 감싸도록
planning footprint는 전륜축 기준 `x=[-0.46,+0.11]`, `y=+-0.18 m`로 바깥쪽
반올림했습니다. 대회에 투입되는 차체가 해당 Gazebo 모델과 같은 조립 상태인지는
현장에서 마지막으로 확인해야 합니다.

## 설치와 빌드

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  python3-pil python3-yaml

cd ~/xycar_kookmin_gazebo_track/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  xycar_msgs xycar_vesc_driver xycar_parking_nav
source install/setup.bash
```

## 오프라인 검증

지도 및 목표 차체 충돌검사:

```bash
ros2 run xycar_parking_nav validate_map
```

Nav2와 독립된 Hybrid-A* 전체 미션 검증:

```bash
PYTHONPATH=src/xycar_parking_nav \
python3 src/xycar_parking_nav/scripts/plan_mission_offline.py \
  --package src/xycar_parking_nav
```

단위 테스트:

```bash
python3 -m pytest -q src/xycar_parking_nav/test
```

실차와 Gazebo 없이 제공 PGM을 직접 레이캐스팅해 AMCL/Nav2/미션/안전 어댑터
전체를 시험할 수도 있습니다. 시뮬레이터는 `/cmd_vel`을 직접 사용하지 않고 실제
어댑터가 만든 `/parking/xycar_motor_shadow`를 실측 조향·속도 lookup으로 다시
해석하므로 조향 slew, 방향전환 dwell, LiDAR stop shield까지 시험합니다. 이
launch는 `/xycar_motor`를 발행하지 않습니다.

```bash
ros2 launch xycar_parking_nav parking_sim.launch.py \
  autostart_mission:=true enable_rviz:=true
```

진행 상태는 `/parking/mission_state`, 정확한 모의 궤적은
`/parking/sim_ground_truth_path`에서 확인합니다.

## 실차 실행 순서

먼저 모터 출력이 없는 shadow 모드로 실행합니다.

```bash
ros2 launch xycar_parking_nav parking_real.launch.py \
  drive_enabled:=false \
  autostart_mission:=false
```

이 통합 launch는 기본적으로 `parking_preflight`를 함께 실행합니다. 실행 직후
터미널에서 `/dev/ttyLIDAR`, `/dev/ttyIMU`, `/dev/ttyMOTOR` 장치와 다음 입력을
자동 확인합니다.

```text
/scan                    LiDAR 원본
/imu                     IMU
/vehicle/vesc_state      VESC 텔레메트리
/slam/scan_filtered      주차용 LiDAR 필터
/slam/odom               VESC+IMU 오도메트리
/amcl_pose               지도 위치추정
map <- slam_odom <- base_footprint <- laser_frame TF
```

정상이면 `========== 모든 주차 센서·TF 정상 ==========`과 미션 시작 명령을
출력합니다. 20초 안에 준비되지 않으면 `[FAIL]` 항목별 원인과 조치 방법을
출력하며, 이때는 미션을 시작하지 않습니다. 점검 시간은
`preflight_timeout_sec:=30.0`처럼 바꿀 수 있고, 외부 점검기를 사용하는 경우에만
`enable_preflight:=false`로 끌 수 있습니다.

다음 항목을 확인합니다.

```bash
ros2 topic hz /scan
ros2 topic hz /slam/scan_filtered
ros2 topic hz /slam/odom
ros2 topic echo /amcl_pose --once
ros2 topic echo /parking/mission_state
ros2 run tf2_ros tf2_echo map slam_odom
ros2 run tf2_ros tf2_echo slam_odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_frame
```

RViz에서 지도 위 LiDAR 점이 벽과 일치하고 mission state가 `READY`가 되면
미션을 시작합니다. Shadow 모드에서는 계산된 명령이
`/parking/xycar_motor_shadow`에만 나옵니다.

```bash
ros2 service call /parking_mission_manager/start std_srvs/srv/Trigger '{}'
```

중지 및 초기화:

```bash
ros2 service call /parking_mission_manager/abort std_srvs/srv/Trigger '{}'
ros2 service call /parking_mission_manager/reset std_srvs/srv/Trigger '{}'
```

바퀴를 띄운 시험, 최저속 전·후진 시험, 장애물 정지 시험을 모두 통과한 후에만
실차 출력을 켭니다.

```bash
ros2 launch xycar_parking_nav parking_real.launch.py \
  drive_enabled:=true \
  autostart_mission:=false
```

`drive_enabled:=true`는 VESC 드라이버와 최종 어댑터를 동시에 활성화하지만,
미션 start 서비스와 localization gate가 열리기 전에는 계속 정지 명령만 냅니다.

센서와 VESC 드라이버를 다른 launch에서 이미 실행 중이면 중복 포트/TF를 피하도록
하드웨어 드라이버가 포함되지 않은 launch를 사용합니다.

```bash
ros2 launch xycar_parking_nav parking_navigation.launch.py \
  start_odometry:=true start_scan_filter:=true \
  drive_enabled:=false
```

## 주요 설정

- `config/nav2_parking.yaml`: AMCL, Hybrid-A*, Ackermann MPPI, costmap과 footprint
- `config/parking_mission.yaml`: 공식 Pose와 31단계 곡률/cusp/정지 구간
- `config/mission_manager.yaml`: localization gate, 재시도, 최종 Pose 허용오차
- `config/cmd_vel_adapter.yaml`: 실측 조향/속도 map과 LiDAR 독립 안전영역
- `config/vesc_imu_odom.yaml`: 타코미터 거리 및 IMU gyro yaw 보정
- `behavior_trees/ackermann_navigate_to_pose.xml`: spin 없는 재계획/복구

설계 근거와 실차 튜닝 항목은 `docs/DESIGN_KO.md`에 정리되어 있습니다.
