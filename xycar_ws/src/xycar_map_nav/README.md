# Xycar map waypoint navigation

저장된 ROS occupancy map에서 여러 체크포인트를 순서대로 연결해 구간별
A* 전역경로를 만들고 Pure Pursuit으로 추종하는 ROS 2 패키지다. SLAM은
`map -> base_footprint` TF를 계속 제공해야 한다.

새 장소에서 지도 생성부터 실차 저속 주행까지의 전체 순서는
[`docs/new_map_real_vehicle_test_KO.md`](../../../docs/new_map_real_vehicle_test_KO.md)에
정리되어 있다.

## 매핑과 localization

엔코더 odom이 아직 없을 때는 `/xycar_motor`의 병진 명령과 `/imu`의 상대
yaw를 결합한 `/slam/odom`으로 scan matching을 시작한다. LiDAR는
`/slam/scan_filtered`에서 `0.20~6.0 m`만 사용한다. 병진 누적 오차가
남으므로 저속 사전 시험용이며, 정확한 엔코더 odom이 생기면
`use_command_odom:=false`로 교체한다.

```bash
ros2 launch xycar_map_nav real_mapping.launch.py \
  laser_x:=0.065 laser_y:=0.00 laser_z:=0.080 laser_yaw:=0.00

ros2 launch xycar_map_nav real_localization.launch.py \
  pose_graph:=$HOME/xycar_maps/new_site_02/map \
  laser_x:=0.065 laser_y:=0.00 laser_z:=0.080 laser_yaw:=0.00
```

위 LiDAR 위치는 simulation 브랜치의 2026-07-12 실차 정합값이며 앞바퀴
중심 기준 `(0.065, 0.000, 0.080) m`, yaw `0`이다. `/scan`, `/imu`와
차량의 기존 모터 bridge는 별도로 먼저 실행해야 한다.

## 제어권

`controller_to_next`는 해당 체크포인트에서 다음 체크포인트까지의 제어
방식을 뜻한다.

- `global_path`: 전역경로 Pure Pursuit
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
