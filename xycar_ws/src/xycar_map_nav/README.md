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

### 주행 중 localization 점프 보호

`slam_toolbox_localization.yaml`은 반복 복도에서 오래된 유사 스캔이 동시에
후보가 되지 않도록 localization rolling buffer를 `100`에서 공식 예제와
같은 `3`으로 줄였다. 먼 loop 후보와 낮은 점수의 loop constraint도 제한하고,
단일 오정합이 pose graph 전체를 당기는 영향을 줄이기 위해 Ceres
`HuberLoss`를 사용한다.

전역경로 제어기는 별도로 `map -> slam_odom` 변화를 감시한다.

- 한 번에 `0.20 m` 또는 `5 deg`를 넘는 보정: raw TF를 조향에 사용하지 않고
  마지막 정상 `map -> slam_odom`과 현재 VESC/IMU odom으로 잠시 진행
- raw TF가 `0.30 s` 안에 정상 범위로 복귀: 자동으로 정상 추종 복귀
- 큰 보정이 `0.30 s` 이상 지속: `LOCALIZATION_JUMP_STOP`, 모터 `[0, 0]`
- fault는 자동 해제하지 않음

상태와 수치는 항상 rosbag에 포함한다.

```bash
ros2 topic echo /map_nav/localization_guard/status
ros2 topic echo /map_nav/localization_guard/debug
```

`debug` 배열은 다음 순서다.

```text
[state, translation_m, yaw_rad, outlier_age_sec,
 raw_x, raw_y, raw_yaw, accepted_x, accepted_y, accepted_yaw]
state: 0=DISABLED, 1=TRACKING, 2=HOLDING, 3=FAULT
```

fault가 발생하면 차량을 먼저 정지시킨다. RViz에서 지도와 LaserScan 정합을
확인하고 `route_scan_localizer/relocalize`를 수행한 다음에만 guard를
초기화한다.

```bash
ros2 service call /route_scan_localizer/relocalize \
  std_srvs/srv/Trigger {}

ros2 service call /xycar_waypoint_nav/reset_localization_guard \
  std_srvs/srv/Trigger {}
```

실차 시작 전에는 `/tf`의 `map -> slam_odom` publisher가 하나인지 확인한다.

```bash
ros2 topic info /tf --verbose
ros2 run tf2_ros tf2_monitor map slam_odom
```

### 경로 위 자동 초기 위치 추정

실차가 저장된 순환 경로 위 어딘가에 있고 진행 방향을 바라본다는 조건에서는
`route_scan_localizer`가 `/slam/scan_filtered`를 지도와 대조해 현재 위치를
찾을 수 있다. 경로 전체를 `0.4 m` 간격으로 탐색하고, 좌우 위치와 yaw를
세부 탐색한 다음 같은 결과가 4개 LiDAR 프레임에서 반복될 때만
`/initialpose`를 한 번 발행한다.

반복되는 직선 복도처럼 서로 2 m 이상 떨어진 두 후보의 점수가 비슷하면
`AMBIGUOUS` 상태를 유지하고 위치를 추측하지 않는다. 이때 차량을 움직이지
말고 문 홈이나 코너가 더 잘 보이는 위치로 옮긴다. 첫 웨이포인트 강제
초기화와 동시에 사용하지 않는다.

먼저 모터를 발행하지 않는 상태로 확인한다.

```bash
MAP_DIR=$HOME/xycar_maps/map_20260728_143906

ros2 launch xycar_map_nav real_waypoint_nav.launch.py \
  map_yaml:=$MAP_DIR/map.yaml \
  waypoints_yaml:=$MAP_DIR/waypoints_pure_pursuit.yaml \
  auto_localize_on_route:=true \
  initialize_pose_from_first_waypoint:=false \
  drive_enabled:=false
```

상태와 후보 자세를 확인한다.

```bash
ros2 topic echo /map_nav/route_localization/status
ros2 topic echo /map_nav/route_localization/debug
ros2 topic echo /map_nav/route_localization/best_pose
ros2 topic echo /map_nav/route_localization/ready
```

`READY`가 되고 RViz의 분홍색 `Route Match` 화살표와 실제 차량 방향이
일치해야 한다. 다시 위치를 찾게 하려면 다음 서비스를 호출한다.

```bash
ros2 service call /route_scan_localizer/relocalize \
  std_srvs/srv/Trigger {}
```

실차 주행에서도 `auto_localize_on_route:=true`를 유지하면 내비게이션 노드는
준비 신호 전까지 `WAIT_ROUTE_LOCALIZATION`과 속도 `0`만 발행한다. 준비된
뒤에도 `drive_start_delay_sec` 동안 기다린 다음 출발한다.

```bash
ros2 launch xycar_map_nav real_waypoint_nav.launch.py \
  map_yaml:=$MAP_DIR/map.yaml \
  waypoints_yaml:=$MAP_DIR/waypoints_pure_pursuit.yaml \
  auto_localize_on_route:=true \
  initialize_pose_from_first_waypoint:=false \
  drive_enabled:=true \
  drive_start_delay_sec:=3.0 \
  cruise_speed_command:=3.0 \
  minimum_speed_command:=3.0
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

기본 `mission_trigger_mode: semantic`에서는 전역경로가 항상 기본
제어기다. 웨이포인트 위치만으로 임무를 미리 시작하지 않고, 카메라 의미
검출과 같은 시각의 LiDAR 기하가 함께 확인된 경우에만 제어권을 바꾼다.

```text
카메라 -> my_rule_object_detection_node -> /my_rule/object_detections
                                      -> /my_rule/cone_processing_enabled
LiDAR  -> my_rule_cone_node (평소 sleep) -> /my_rule/cone_clusters
                                          /my_rule/cone_cmd
SLAM   -> map -> slam_odom -> base_footprint
                                         |
                          xycar_waypoint_nav
                 [상황 판단 + 전역경로 + 최종 선택]
                                         |
                              /xycar_motor (유일)
```

전환 우선순위는 다음과 같다.

1. localization fault, 센서 timeout, 전방 비상정지
2. 빨강·노랑 신호등 정지
3. 동적 차량 회피
4. 라바콘 주행
5. LiDAR 임시 장애물 회피
6. 전역경로 주행

기본 전환값:

- 라바콘: YOLO `cone` 2프레임, LiDAR cone cluster가 `0.50 m` 이내,
  `/my_rule/cone_cmd`가 모두 유효할 때 `CONE_RULE`
- 동적 차량: YOLO `obstacle_vehicle` 또는 임시 `car` 2프레임과 해당
  bounding box 각도 안의 LiDAR가 `2.40 m` 이내일 때
  `DYNAMIC_VEHICLE_RULE`
- 임시 일반 장애물: 객체 모델이 완성되기 전까지 전역경로의 차량 폭
  구간 안에 있는 LiDAR 연속 점군을 2프레임 확인하면
  `LIDAR_OBSTACLE_RULE`
- 신호등: `red` 또는 `yellow` 2프레임이면 정지 latch, `green`
  2프레임이면 해제
- 차선 이탈 개입: 구현 자리는 있으나 `lane_intervention_enabled: false`
  로 보류

라바콘 검출이 끊겨도 즉시 전역경로로 돌아가지 않는다. 최소 1초 주행하고
카메라와 LiDAR가 모두 사라진 상태가 0.7초 유지되어야 복귀한다. 동적 차량
회피도 오프셋이 중앙으로 돌아온 뒤에만 전역경로 상태로 해제한다.

저사양 실차의 CPU를 아끼기 위해 `my_rule_cone_node`는 평소 `/scan`
구독 자체를 제거한 sleep 상태다. YOLO가 신뢰도 기준을 넘는 `cone`을 처음
검출하면 `/my_rule/cone_processing_enabled=true`가 발행되고 그때부터만
LiDAR 클러스터링과 경로 생성을 수행한다. 카메라 검출이 잠깐 흔들려도
1.5초 동안 처리를 유지하며, `CONE_RULE` 중에는 항상 켜져 있다. 임무가
끝나면 구독과 경로 이력을 모두 정리하고 다시 sleep 상태로 돌아간다.

### 임시 LiDAR 장애물 회피

`gazebo_sitl`의 route-progress obstacle memory와 부드러운 우회 경로 개념을
Xycar 크기로 축소했다. `/scan`의 모든 물체를 장애물로 취급하지 않고,
현재 SLAM 전역경로를 차량 좌표로 변환한 다음 그 경로 중심에서 좌우
`0.18 m` 안에 걸리는 점만 검사한다.

- 감지 거리: `1.50 m`
- 연속 LiDAR 점: 최소 3개
- 점군 폭: `0.09~0.70 m`
- 확인: 2프레임
- 우회 오프셋: 장애물 반대편 `0.28 m`
- 회피 속도 상한: Xycar command `4.0`
- 장애물 뒤 여유: `0.45 m`

장애물이 왼쪽에 있으면 오른쪽으로, 오른쪽에 있으면 왼쪽으로 피한다.
중앙에 있으면 좌우 LiDAR 여유 공간이 더 큰 쪽을 선택한다. 장애물이
조향 때문에 센서에서 잠깐 사라져도 곧바로 중앙으로 복귀하지 않는다.
처음 검출한 전역경로 index부터 `장애물 거리 + 예상 길이 + 0.45 m`를
통과할 때까지 우회 오프셋을 기억하고, 이후 오프셋을 서서히 0으로 만든다.

이 기능은 객체 모델이 없는 동안의 임시 fallback이다. 작은 단일 노이즈와
라바콘 크기 점군은 필터링하지만 LiDAR만으로 물체 종류를 완전히 구분할
수는 없다. 장애물 YOLO가 준비되면
`lidar_obstacle_fallback_enabled: false`로 끄고 카메라 class와 LiDAR
거리를 함께 확인하는 엄격한 진입 조건으로 교체한다.

`/map_nav/lidar_obstacle_debug` 배열 순서는
`[valid, distance, lateral, width, point_count, x, y,
left_minus_right_clearance, mode_code, path_offset, remaining_clear_distance]`
이다. `mode_code`는 `0=NORMAL`, `1=BYPASS_LEFT`, `-1=BYPASS_RIGHT`,
`2=RETURN_CENTER`다.

최종 `/xycar_motor` 발행자는 이 노드 하나여야 한다. 라바콘용
`my_rule_cone_node`는 후보 명령만 발행하며 모터를 직접 발행하지 않는다.
기존 `my_rule_drive_manager`는 중복 모터 권한을 피하려고 이 통합에서
제외했다. 별도 lane driver와 keyboard teleop도 동시에 실행하지 않는다.

SLAM localization과 카메라·LiDAR 드라이버가 실행된 상태에서 먼저 shadow로
통합 stack을 확인한다.

```bash
cd ~/slam/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

MAP_DIR=$HOME/xycar_maps/map_20260728_143906
ros2 launch xycar_map_nav semantic_hybrid_nav.launch.py \
  map_yaml:=$MAP_DIR/map.yaml \
  waypoints_yaml:=$MAP_DIR/waypoints_pure_pursuit.yaml \
  drive_enabled:=false
```

다음 토픽에서 현재 선택과 이유를 확인한다.

```bash
ros2 topic echo /map_nav/control_mode
ros2 topic echo /map_nav/mission_reason
ros2 topic echo /map_nav/xycar_motor_shadow
ros2 topic echo /my_rule/object_detections
ros2 topic echo /my_rule/cone_processing_enabled
ros2 topic echo /my_rule/cone_clusters
ros2 topic echo /map_nav/lidar_obstacle_debug
```

shadow 검증과 바퀴를 든 시험을 통과한 뒤에만 `drive_enabled:=true`로
바꾼다.

기존 지도처럼 웨이포인트별 임무 구간을 유지해야 할 때만
`mission_trigger_mode: route_segments`를 사용한다. 이 호환 모드에서
`controller_to_next`는 해당 체크포인트부터 다음 체크포인트까지
`global_path`, `cone_rule`, `dynamic_vehicle_rule` 중 하나를 뜻한다.

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

상황 기반 모드에서는 좌표를 다 찍은 뒤 모든 `controller_to_next`를
`global_path`로 두어도 된다. 기존 구간 기반 호환 모드만 라바콘 진입점의
값을 `cone_rule`, 동적차량 구간을 `dynamic_vehicle_rule`로 바꾼다.

```bash
ros2 service call /xycar_waypoint_nav/reload_route std_srvs/srv/Trigger {}
```

실제 위치를 측정하기 전까지 제공된 example 파일은 실차 기준값이 아니다.
순환 코스는 waypoint YAML의 `closed`를 `true`로 설정한다. 마지막 좌표가
출발점 근처인데 `closed: false`이면 시작 직후 `ROUTE_COMPLETE`가 될 수 있다.

곡선 구간에서 속도 적응형 Pure Pursuit를 시험하려면 실차 launch에 다음
인자를 명시한다. 직선은 낮은 gain의 Stanley를 계속 사용하며, 곡선
lookahead는 `0.30 + measured_speed * 0.12` m로 계산된다.

```bash
ros2 launch xycar_map_nav real_waypoint_nav.launch.py \
  curve_controller:=pure_pursuit \
  curve_pure_pursuit_lookahead_m:=0.30 \
  curve_pure_pursuit_speed_preview_sec:=0.12
```

기본값 `curve_controller:=stanley`는 2026-07-28 고속 오실레이션 회귀시험을
통과한 실차 기준이다. Pure Pursuit는 먼저 `drive_enabled:=false` shadow와
speed command `3`에서 검증한다.

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
5. `0.50 m` 경계 앞뒤의 정지 라바콘으로
   `GLOBAL_PATH -> CONE_RULE -> GLOBAL_PATH`를 시험한다.
6. 정지 차량 모형부터 시작해
   `GLOBAL_PATH -> DYNAMIC_VEHICLE_RULE -> GLOBAL_PATH`를 시험한다.
7. 모터를 띄운 상태에서 빨강·노랑은 `[0, 0]`, 초록 확인 뒤에만
   전역경로 명령이 복구되는지 시험한다.

LiDAR 전방 `0.38 m` 이내 물체는 모드와 관계없이 `EMERGENCY_STOP`이다.
상황 기반 모드는 YOLO가 stale이면 새 임무에 진입하지 않으며, 이미 시작한
임무는 각 상태의 안전한 이탈 조건을 거친다.

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
