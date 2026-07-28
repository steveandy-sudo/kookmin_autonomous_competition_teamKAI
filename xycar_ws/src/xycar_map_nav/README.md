# Xycar map waypoint navigation

저장된 ROS occupancy map에서 여러 체크포인트를 순서대로 연결해 구간별
A* 전역경로를 만들고, 실차 동역학을 반영한 Stanley 제어기로 추종하는
ROS 2 패키지다. SLAM은 `map -> base_footprint` TF를 계속 제공해야 한다.

새 장소에서 지도 생성부터 실차 저속 주행까지의 전체 순서는
[`docs/new_map_real_vehicle_test_KO.md`](../../../docs/new_map_real_vehicle_test_KO.md)에
정리되어 있다.

## 매핑과 localization

기본 실차 odometry는 native ROS2 `xycar_vesc_driver`가 발행하는
`/vehicle/vesc_state`의 tachometer 이동량과 `/imu`의 상대 yaw를 결합해
`/slam/odom`을 발행한다.
기본 `slam_toolbox_mapping.yaml`은 공식 Karto scan matching과 automatic
loop closure를 사용한다. 반복 문 복도에서는 서로 다른 문 홈의 스캔이 비슷해
잘못 정합될 수 있으므로, 첫 바퀴 정합을 확인한 다음 같은 방향으로 2~3회
재관측하며 기존 벽과 현재 LaserScan이 계속 한 줄로 겹치는지 확인한다.

```text
VESC tachometer + IMU yaw
  -> /slam/odom + slam_odom -> base_footprint
LiDAR scans
  -> occupancy map + pose graph
```

VESC는 독립 휠 엔코더가 아니므로 바퀴 미끄러짐 오차는 남지만, 모터 명령을
거리로 간주하던 command odom보다 실제 이동을 더 직접적으로 반영한다.
`command` 모드는 측정 telemetry를 사용할 수 없을 때만 쓰는 fallback이다.
한 바퀴 뒤 시작점에서 자동 정합이 실패했다면 어긋난 맵을 이어 쓰지 않는다.
rosbag을 보존하고 고유한 모서리와 비대칭 구조가 포함되도록 새 빈 맵에서 다시
측정한다.

motor 없이 저장 bag을 재처리할 때는 다음 launch를 사용한다.

```bash
ros2 launch xycar_map_nav replay_mapping.launch.py enable_rviz:=true
```

재생 토픽은 `/slam/scan_filtered`와 `/slam/odom`만 사용한다. 기존 bag의
`/tf`, `/map`, 모터 토픽은 이 launch에 재생하지 않는다.

```bash
ros2 launch xycar_map_nav real_mapping.launch.py \
  odom_source:=vesc_imu \
  vesc_drive_enabled:=false \
  laser_x:=0.065 laser_y:=0.00 laser_z:=0.080 laser_yaw:=0.00

ros2 launch xycar_map_nav real_localization.launch.py \
  pose_graph:=$HOME/xycar_maps/new_site_02/map \
  odom_source:=vesc_imu \
  vesc_drive_enabled:=false \
  laser_x:=0.065 laser_y:=0.00 laser_z:=0.080 laser_yaw:=0.00
```

위 LiDAR 위치는 simulation 브랜치의 2026-07-12 실차 정합값이며 앞바퀴
중심 기준 `(0.065, 0.000, 0.080) m`, yaw `0`이다. `/scan`과 `/imu`는
별도로 먼저 실행한다. mapping/localization launch가 native VESC 드라이버를
기본으로 시작하며, driver raw TF는 끄고 `/slam/odom`과
`slam_odom -> base_footprint`는 융합 노드 하나만 발행한다.

전체 실행 및 검증 절차는
[`docs/real_vesc_imu_lidar_odometry_KO.md`](../../../docs/real_vesc_imu_lidar_odometry_KO.md)에
정리되어 있다.

## 제어권

`controller_to_next`는 해당 체크포인트에서 다음 체크포인트까지의 제어
방식을 뜻한다.

- `global_path`: 전역경로 Stanley 추종
- `cone_rule`: 전역 모터 명령을 멈추고 `xycar_hybrid_drive`의 라바콘
  shadow 명령만 전달
- `dynamic_vehicle_rule`: 전역경로를 기준으로 YOLO 차량 검출에 따라
  미터 단위 좌우 오프셋을 적용

최종 `/xycar_motor` 발행자는 이 노드 하나여야 한다. 라바콘용
`xycar_hybrid_drive`는 반드시 `drive_enabled:=false`로 실행한다.

## RViz에서 좌표 찍기

먼저 `drive_enabled:=false`로 실행한다.

```bash
ros2 launch xycar_map_nav real_waypoint_nav.launch.py \
  drive_enabled:=false \
  map_yaml:=/absolute/path/to/map.yaml \
  waypoints_yaml:=/absolute/path/to/my_waypoints.yaml \
  capture_output_yaml:=/absolute/path/to/my_waypoints.yaml
```

RViz의 `Publish Point` 도구로 진행 순서대로 클릭한다. 클릭할 때마다 경로를
다시 계획하고 `capture_output_yaml`에 저장한다. RViz에 다음을 추가한다.

- Map: `/map`
- Path: `/map_nav/global_path`
- MarkerArray: `/map_nav/waypoints`
- TF: `map -> slam_odom -> base_footprint`

되돌리기와 초기화:

```bash
ros2 service call /xycar_waypoint_nav/undo_waypoint std_srvs/srv/Trigger {}
ros2 service call /xycar_waypoint_nav/clear_waypoints std_srvs/srv/Trigger {}
```

좌표를 다 찍은 뒤 YAML에서 라바콘 진입점의 `controller_to_next`를
`cone_rule`, 동적차량 구간 진입점부터 이탈점 전까지를
`dynamic_vehicle_rule`로 바꾸고 다음 서비스로 다시 읽는다.

```bash
ros2 service call /xycar_waypoint_nav/reload_route std_srvs/srv/Trigger {}
```

실제 위치를 측정하기 전까지 제공된 example 파일은 실차 기준값이 아니다.
순환 코스는 waypoint YAML의 `closed`를 `true`로 설정한다. 마지막 좌표가
출발점 근처인데 `closed: false`이면 시작 직후 `ROUTE_COMPLETE`가 될 수 있다.

### 기존 사용자 제작 Gazebo 트랙

`worlds/kookmin_xycar_track_final.sdf`에서 새 경로를 직접 찍을 때는 저장된
회귀시험 CSV를 사용하지 않는다. 저장 파일은
`routes/kookmin_custom_user_waypoints.yaml`이며 첫 점은 차량 시작 위치,
이후 점은 실제 주행 순서대로 찍는다. 곡선은 직선보다 촘촘하게 찍어야
평활화된 경로가 차도 밖으로 지름길을 만들지 않는다.

```bash
cd ~/slam
source /opt/ros/humble/setup.bash
source xycar_ws/install/setup.bash

ros2 launch xycar_map_nav sim_custom_track_waypoint_nav.launch.py \
  project_root:=$PWD \
  drive_enabled:=false
```

RViz 상단의 `Publish Point`를 선택하고 트랙 위를 클릭한다. 초록 구는
waypoint, 선은 실제 Stanley 제어기가 받을 `/map_nav/global_path`다.
이 모드에서는 `/xycar_motor` publisher를 만들지 않으므로 차량은 움직이지
않는다. 점을 다 찍은 다음 전체 launch를 종료하고 다음처럼 명시적으로
주행을 허용한다.

```bash
ros2 launch xycar_map_nav sim_custom_track_waypoint_nav.launch.py \
  project_root:=$PWD \
  drive_enabled:=true \
  drive_start_delay_sec:=3.0 \
  reposition_vehicle:=true
```

사용자 트랙 launch의 기본 주행 파라미터는 2026-07-28 병렬 Gazebo
탐색에서 확정한 전진/후진 속도 계획 프로파일이다. 4회 독립 반복에서
`24.09~24.20 s`, 평균 `24.14 s`, P95 CTE 평균 `0.215 m`, 큰 직선
조향 반전 `0회`를 기록했다. 캡처가 목적이면 반드시 위쪽 예시처럼
`drive_enabled:=false`를 사용한다.

경로를 다시 만들려면 capture 모드에서 다음 서비스를 호출한다.

```bash
ros2 service call /xycar_waypoint_nav/undo_waypoint \
  std_srvs/srv/Trigger {}
ros2 service call /xycar_waypoint_nav/clear_waypoints \
  std_srvs/srv/Trigger {}
```

오실레이션 시험은 주행 중 아래 세 토픽을 기록한다.

```bash
ros2 bag record -o analysis/custom_track_stanley \
  /map_nav/debug /map_nav/control_mode /map_nav/global_path

python3 xycar_ws/src/xycar_map_nav/scripts/analyze_waypoint_oscillation.py \
  analysis/custom_track_stanley \
  --output-json analysis/custom_track_stanley_summary.json
```

결과의 `steering_command.large_straight_reversals`는 직선에서 2초 안에
`+12 -> -12` 또는 반대로 크게 바뀐 횟수다. 경로가 아직 비어 있거나 두 점
미만이면 주행 명령을 내지 않는다.

## 검증 순서

1. SLAM localization을 실행하고 TF가 끊기지 않는지 확인한다.
2. shadow 상태에서 `/map_nav/global_path`, 조향 부호, 현재 구간 mode를
   rosbag으로 확인한다.
3. 바퀴를 든 상태에서 `drive_enabled:=true`로 바꾸고 정지 명령과 조향
   방향을 확인한다.
4. 빈 공간에서 속도 명령 `3`으로 전역경로만 시험한다.
5. 정지 라바콘으로 `GLOBAL_PATH -> CONE_RULE -> GLOBAL_PATH`를 시험한다.
6. 정지 차량 모형부터 시작해
   `GLOBAL_PATH -> DYNAMIC_VEHICLE_RULE -> GLOBAL_PATH`를 시험한다.

LiDAR 전방 `0.38 m` 이내 물체는 모드와 관계없이 `EMERGENCY_STOP`이다.
동적차량 구간에서 YOLO count 토픽이 stale이면 주행하지 않고 정지한다.

## 고속 오실레이션 억제

2026-07-26 실차에서는 command `3`보다 높일 때 기존 고정 lookahead
Pure Pursuit의 큰 좌우 반전이 관찰됐다. 현재 전역경로 제어에는 다음을
적용했다.

- 충돌 검사 후 곡률을 제한하는 경로 평활화
- 측정 속도와 조향 지연을 반영한 front-axle Stanley 제어
- 곡률 feedforward와 실제 yaw-rate 감쇠
- 직선/곡선별 steering rate limit 및 저역통과 필터
- 곡률, 횡가속도, CTE, heading 정렬을 함께 보는 속도 계획
- 빠른 감속과 완만한 재가속

Gazebo 실차 동역학 회귀시험에서 command `10` 상한으로 한 바퀴를 완주했고
CTE 평균 `0.049 m`, 95% `0.103 m`, 최대 `0.132 m`, 큰 직선 조향 반전
`0회`를 기록했다. 이는 실차 command `10`을 바로 승인한다는 뜻이 아니다.
실차는 기존 검증값 `3`부터 `4 -> 5 -> 7 -> 10` 순서로 올리고 각 단계의
rosbag을 확인해야 한다.

구현 원리, Gazebo 재현 명령, rosbag 분석법과 실차 중단 조건은
[`docs/global_path_stanley_oscillation_20260728_KO.md`](../../../docs/global_path_stanley_oscillation_20260728_KO.md)에
정리되어 있다.

같은 문서의 `사용자 제작 트랙의 논문 기반 전역경로 개선` 절에는
전진/후진 속도 프로파일, 구동 지연을 반영한 제동 미리보기, 제한형
최소곡률 스무딩, 4개 Gazebo 병렬 탐색 방법과 2026-07-28 비교 결과가
추가되어 있다.
