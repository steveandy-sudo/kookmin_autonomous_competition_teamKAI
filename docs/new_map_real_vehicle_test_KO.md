# 새 장소 지도 생성·실차 주행 테스트 순서

이 문서는 본선 트랙에 가기 전에 다른 공간에서 다음 전체 흐름을 검증하기 위한
실차 체크리스트다.

```text
LiDAR/TF 확인
  -> 수동 저속 주행으로 SLAM 지도 생성
  -> 지도와 pose graph 저장
  -> 같은 출발점에서 localization 반복
  -> RViz로 체크포인트 지정
  -> 전역경로 shadow 검증
  -> 바퀴 공중 테스트
  -> 빈 공간 저속 주행
  -> 라바콘/동적차량 rule 구간을 단계적으로 추가
```

첫 테스트의 목적은 빠르게 달리는 것이 아니라 이 연결이 끊기지 않는지 확인하는
것이다. 실차에서는 물리 비상정지 담당자와 수동 조종 담당자를 별도로 둔다.

## 0. 이번 구성의 전제와 제한

- ROS 2 Humble, Ubuntu 22.04 실차 환경을 기준으로 한다.
- LiDAR 입력은 `/scan`, 프레임은 `laser_frame`이다.
- 모터 명령은 `/xycar_motor`,
  `std_msgs/msg/Float32MultiArray [angle, speed]`이다.
- 지도 프레임 연결은
  `map -> slam_odom -> base_footprint -> laser_frame`이다.
- 현재 실차 보정 절차에서는 `/imu`의 상대 yaw를 사용한다. IMU가 끊겼다는
  경고가 나오면 지도를 저장하지 않는다.
- 현재 저장소에는 실차 엔코더 odom 발행기가 없다. 따라서 기본 launch는
  `/xycar_motor` 명령으로 만든 근사 odom을 SLAM scan matching의 초기 추정값으로
  쓴다. 바퀴 미끄러짐과 배터리 상태를 측정하지 못하므로 누적 오차가 생긴다.
- 엔코더 odom을 확보하면 `use_command_odom:=false`로 바꾸고, 엔코더 노드가
  `/odom`과 `odom -> base_footprint` TF를 발행하게 한다. 본선 정밀 주행은 이
  구성이 권장된다.
- 새 장소 시험에서는 처음부터 라바콘과 동적차량을 넣지 않는다. 먼저 모든
  구간을 `global_path`로 통과시킨 후 두 rule 구간을 하나씩 추가한다.

`command_odom_real.yaml`의 `0.080612 m/s/command`는 기존 실차/시뮬레이션
캘리브레이션에서 가져온 시작값이다. 새 차량에서 확정값으로 간주하면 안 된다.
2026-07-24 백의 LiDAR/IMU 재분석 결과와 임시 비교 설정은
[`data/odom_calibration/2026-07-24/README.md`](../data/odom_calibration/2026-07-24/README.md)에
있다. 이 분석은 도면 치수를 거리 정답으로 사용하지 않았고, 분석기 간 차이가
커서 production 값은 자동 변경하지 않았다.

## 1. 설치와 빌드

실차 PC에서 저장소를 갱신하고 필요한 패키지를 설치한다.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-rviz2

cd ~/xycar_ws
colcon build --packages-up-to \
  xycar_imu xycar_lidar xycar_rule_drive xycar_map_nav xycar_hybrid_drive \
  --symlink-install
source install/setup.bash
```

매 터미널에서 차량과 같은 `ROS_DOMAIN_ID`를 적용한다. 저장소의 기존 실차
설정이 `7`이라면 다음처럼 사용한다.

```bash
export ROS_DOMAIN_ID=7
```

## 2. 출발 전에 실측할 값

차량 바닥 중심의 `base_footprint`에서 LiDAR 중심까지 줄자로 측정한다.

- `laser_x`: 전방이 양수인 거리(m)
- `laser_y`: 좌측이 양수인 거리(m)
- `laser_z`: 바닥에서 LiDAR 스캔면까지 높이(m)
- `laser_yaw`: 차량 정면 대비 LiDAR 회전(rad)

simulation 브랜치의 2026-07-12 실차 정합값은 앞바퀴 중심 기준
`x=0.065, y=0, z=0.080, yaw=0`이다. 이 차량에서는 해당 값을 기본으로
사용한다.

바닥에는 차량의 시작 위치와 방향을 테이프로 표시한다. 매핑 종료점과
localization 재시작점을 이 표시에 맞춘다.

유리벽이 있는 경우, 현재 도면 기준 오른쪽 위 유리 구간 앞에 사람이 없는
비투명 임시 경계물을 둔다. 무광 폼보드, 천을 씌운 안전 펜스처럼 LiDAR가
안정적으로 반사하는 연속 경계를 사용하고 실제 주행 가능 폭 바깥에 설치한다.
유리 자체에서 안정적인 점이 나온다고 가정하지 않는다. 임시 경계물의 위치와
폭을 사진과 실측값으로 남긴다.

## 3. 센서와 모터 인터페이스 확인

차량의 기존 모터/ROS bridge를 먼저 실행한다. 다른 자율주행 노드는 모두 끈다.

터미널 A에서 IMU를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash
export ROS_DOMAIN_ID=7
ros2 launch xycar_imu xycar_imu.launch.py
```

터미널 B에서 LiDAR를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash
export ROS_DOMAIN_ID=7
ros2 launch xycar_lidar xycar_lidar.launch.py
```

다른 터미널에서 확인한다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash
export ROS_DOMAIN_ID=7

ros2 topic list -t | grep -E '/imu|/scan|/xycar_motor'
ros2 topic hz /imu
ros2 topic type /scan
ros2 topic hz /scan
ros2 topic info /xycar_motor -v
ros2 topic echo /scan --once
```

통과 조건:

- `/scan` 타입이 `sensor_msgs/msg/LaserScan`이다.
- `/imu`가 약 35 Hz로 연속 수신된다.
- `/scan`이 약 10 Hz로 연속 수신되고 `frame_id`가 `laser_frame`이다.
- `range_min`, `range_max`, `angle_min`, `angle_max`가 비정상 값이 아니다.
- `/xycar_motor` 구독자에 실제 모터 bridge가 보인다.
- 키보드나 자율주행 publisher가 아직 실행 중이지 않다.

## 4. 근사 odom 캘리브레이션

처음에는 바퀴를 띄우거나 차량을 롤러 위에 고정한다. 터미널 C에서 매핑 launch를
실측 LiDAR 위치로 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash
export ROS_DOMAIN_ID=7

ros2 launch xycar_map_nav real_mapping.launch.py \
  laser_x:=0.065 laser_y:=0.00 laser_z:=0.080 laser_yaw:=0.00 \
  use_command_odom:=true
```

다른 터미널에서 TF와 odom을 확인한다.

```bash
ros2 topic hz /slam/odom
ros2 topic hz /slam/scan_filtered
ros2 run tf2_ros tf2_echo slam_odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_frame
```

수동 조종은 저장소의 키보드 노드를 사용한다.

```bash
ros2 run xycar_rule_drive keyboard_teleop
```

`w/s`는 속도, `a/d`는 조향, `e`는 조향 중앙, `x`와 `space`는 정지,
`q`는 종료다. 바퀴 방향과 조향 부호가 맞고, 키 입력을 멈춘 뒤 0.3초 안에
`/slam/odom` 속도가 0이 되는지 확인한다.

그다음 평평한 바닥에서 줄자로 5.00 m를 표시한다. 첫 표시 1 m 전부터 직진
명령 하나를 고정해 정상속도로 두 표시를 통과하고 실제 거리와 odom 거리 차이를
잰다.

```text
새 speed_gain_mps_per_command
  = 시험에 사용한 speed_gain * 5.00 / odom 측정거리
```

정방향 5회와 역방향 5회의 중앙값을
`xycar_map_nav/config/command_odom_real.yaml`의
`speed_gain_mps_per_command`에 넣고 다시 빌드한다. 직진 5 m에서 추정 이동거리
오차가 5%를 넘으면 매핑을 시작하지 않는다. 이 검사는 엔코더 odom을 대신하지
않으며 scan matching이 시작될 정도의 초기값만 맞추는 절차다.

## 5. rosbag 기록 시작

시험마다 새 이름으로 센서, TF, 명령을 함께 기록한다.
rosbag은 매핑 launch와 주행을 시작하기 전에 켠다. 기록하지 않은 주행은
pose graph가 잘못된 뒤 원시 센서 기준으로 재처리할 수 없다.

```bash
mkdir -p ~/xycar_test_bags
ros2 bag record \
  -o ~/xycar_test_bags/new_site_mapping_02 \
  /scan /slam/scan_filtered /imu /slam/odom \
  /tf /tf_static /map /xycar_motor
```

기록 직후 `ros2 bag info`로 모든 토픽과 메시지 수를 확인한다.

## 6. 새 장소 지도 만들기

RViz에 `Map`, `LaserScan`, `TF`를 추가하고 Fixed Frame을 `map`으로 둔다.
키보드 주행 속도는 처음에는 command `2~3`만 사용한다.

매핑 주행 순서:

1. 시작 표시에서 정면으로 출발한다.
2. 외곽을 한 방향으로 천천히 한 바퀴 돈다.
3. 직선은 차선 중앙 부근으로 가고, 곡선은 급조향하거나 제자리 회전하지 않는다.
4. 꼬불꼬불한 구간과 모서리는 속도를 낮춰 LiDAR가 같은 벽을 여러 각도에서
   보게 한다.
5. 처음 지나간 특징적인 모서리와 직선을 다시 통과해 loop closure를 만든다.
6. 반대 방향으로 한 바퀴 더 돌고 시작 표시로 복귀한다.
7. RViz에서 시작 부근 스캔과 기존 벽이 겹친 뒤 지도가 보정되는지 확인한다.

매핑 중 사람과 이동 물체를 최소화한다. 라바콘과 차량 모형은 아직 놓지 않는다.
오른쪽 위 유리 구간은 임시 비투명 경계물을 유지한다.

다음 현상이 보이면 저장하지 말고 다시 만든다.

- 벽이 평행한 두 줄 이상으로 찢어진다.
- 출발점으로 돌아왔을 때 기존 지도가 크게 밀린다.
- 차량이 정지했는데 `map -> slam_odom`이 계속 크게 뛴다.
- 유리 구간이 열린 공간처럼 사라지거나 반대쪽 물체가 벽으로 찍힌다.
- 복도 폭이 주행 방향에 따라 눈에 띄게 달라진다.

## 7. 지도와 pose graph 저장

매핑이 안정된 상태에서 새 터미널을 연다.

```bash
MAP_DIR=~/xycar_maps/new_site_02
mkdir -p "$MAP_DIR"

ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_DIR/map"

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${MAP_DIR}/map'}"
```

다음 결과물을 보관한다.

```text
~/xycar_maps/new_site_02/
├── map.yaml
├── map.pgm
├── map.posegraph
└── map.data
```

설치된 slam_toolbox 버전에 따라 직렬화 파일의 확장자 표시는 조금 다를 수 있다.
`map.yaml` 안의 `image`, `resolution`, `origin`을 확인하고 `map.pgm`과 함께
복사한다. pose graph는 localization에, `map.yaml`은 전역경로 생성에 사용한다.

## 8. 지도 품질 판정

아래를 모두 만족해야 다음 단계로 간다.

| 항목 | 통과 조건 |
|---|---|
| 벽 정합 | 같은 벽이 두껍거나 이중선으로 갈라진 부분이 대부분 0.15 m 이내 |
| loop closure | 시작 표시로 돌아온 후 스캔이 기존 지도와 다시 겹침 |
| 주행 통로 | 차량 폭과 양쪽 안전 여유를 포함한 free space가 끊기지 않음 |
| 유리 구간 | 임시 경계가 연속 점유영역이며 반대쪽 가짜 벽이 주 경로를 막지 않음 |
| 재현성 | 같은 표시에서 localization을 3회 재시작해 모두 수렴 |
| 정지 안정성 | 차량 정지 중 위치가 갑자기 0.20 m 이상 뛰지 않음 |

실측 통로 폭과 지도상의 폭 차이가 0.10 m보다 크면 LiDAR 위치, scan 방향,
command odom gain부터 다시 확인한다.

## 9. 저장 지도에서 localization

매핑 launch와 키보드 노드를 모두 종료하고 IMU와 LiDAR만 다시 실행한다. 차량을
바닥의 시작 표시에 같은 방향으로 놓는다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash
export ROS_DOMAIN_ID=7

ros2 launch xycar_map_nav real_localization.launch.py \
  pose_graph:=$HOME/xycar_maps/new_site_02/map \
  laser_x:=0.065 laser_y:=0.00 laser_z:=0.080 laser_yaw:=0.00 \
  use_command_odom:=true
```

RViz에서 저장된 벽과 현재 LaserScan이 겹치는지 확인한다. 필요하면 RViz의
`2D Pose Estimate`로 시작 자세를 지정한다. 다음 TF가 모두 끊기지 않아야 한다.

```bash
ros2 run tf2_ros tf2_echo map slam_odom
ros2 run tf2_ros tf2_echo map base_footprint
```

차량을 1~2 m 수동으로 움직인 뒤 제자리로 돌려놓는 시험을 3회 한다.
한 번이라도 다른 복도나 반대 방향으로 수렴하면 자율주행으로 넘어가지 않는다.

## 10. RViz에서 전역경로 좌표 만들기

localization을 유지한 채 waypoint 노드를 shadow로 실행한다.

```bash
WAYPOINTS=$HOME/xycar_maps/new_site_02/waypoints.yaml

ros2 launch xycar_map_nav real_waypoint_nav.launch.py \
  drive_enabled:=false \
  map_yaml:=$HOME/xycar_maps/new_site_02/map.yaml \
  capture_output_yaml:=$WAYPOINTS
```

RViz의 `Publish Point`로 주행 순서대로 좌표를 찍는다.

- 직선 시작과 끝
- 꼬불꼬불한 구간의 진입점, 방향이 바뀌는 지점, 이탈점
- 직선에서 회전 곡선으로 들어가기 전과 곡선이 끝난 직후
- 라바콘 rule 진입/이탈 후보
- 동적차량 rule 진입/이탈 후보

처음 저장할 때는 모든 `controller_to_next`를 `global_path`로 둔다.
`/map_nav/global_path`가 벽과 inflation 영역을 침범하지 않는지 확인한다.

잘못 찍은 점:

```bash
ros2 service call /xycar_waypoint_nav/undo_waypoint \
  std_srvs/srv/Trigger {}
```

전체 초기화:

```bash
ros2 service call /xycar_waypoint_nav/clear_waypoints \
  std_srvs/srv/Trigger {}
```

## 11. Shadow와 바퀴 공중 검증

키보드 노드를 끈 뒤 publisher 수를 확인한다.

```bash
ros2 topic info /xycar_motor -v
ros2 topic echo /map_nav/control_mode
ros2 topic echo /map_nav/xycar_motor_shadow
```

shadow에서는 waypoint 노드가 `/xycar_motor` publisher를 만들지 않아야 한다.
차량을 수동으로 천천히 움직이며 다음을 확인한다.

- 현재 위치와 전역경로의 최근점이 함께 이동한다.
- 직선 조향은 0 부근이고 좌/우 곡선 부호가 실차와 맞는다.
- LiDAR를 가리면 `WAIT_SCAN`, TF를 끊으면 `WAIT_LOCALIZATION`으로 바뀐다.
- 전방 0.38 m 안에 물체를 두면 `EMERGENCY_STOP`과 속도 0이 나온다.

그 후 차량을 고정하고 구동 바퀴를 띄운 상태에서만 실제 publisher를 켠다.

```bash
ros2 launch xycar_map_nav real_waypoint_nav.launch.py \
  drive_enabled:=true \
  cruise_speed_command:=3.0 \
  minimum_speed_command:=3.0 \
  map_yaml:=$HOME/xycar_maps/new_site_02/map.yaml \
  waypoints_yaml:=$HOME/xycar_maps/new_site_02/waypoints.yaml
```

`Ctrl+C`, LiDAR 차단, localization 중단 각각에서 속도 0이 되는지 확인한다.
프로세스 강제 종료나 ROS 통신 단절은 소프트웨어 정지 명령을 보장하지 않으므로
물리 비상정지가 최종 안전장치다.

## 12. 빈 공간 실차 주행 단계

각 단계를 통과하고 rosbag을 검토한 뒤에만 다음 단계로 간다.

1. 2~3 m 직선, speed command `3`
2. 완만한 단일 곡선, speed command `3`
3. 장애물 없는 전체 전역경로 1바퀴
4. 장애물 없는 전체 전역경로 5바퀴 반복
5. 정지 라바콘을 둔 rule 구간
6. 정지 차량 모형을 둔 동적차량 rule 구간
7. 충분히 떨어진 저속 이동 모형과 동적차량 rule 구간

주행 중 지도 위치가 0.20 m 이상 순간 이동하거나 경로와 실제 차량 차이가
0.20 m를 넘으면 즉시 멈추고 localization과 TF를 먼저 조사한다. 속도를 올려
해결하지 않는다.

## 13. 라바콘과 동적차량 rule 구간 추가

전역경로 5바퀴가 먼저 안정적으로 끝난 뒤 `waypoints.yaml`을 수정한다.

- 라바콘 진입 waypoint의 `controller_to_next`: `cone_rule`
- 라바콘 이탈점부터: `global_path`
- 동적차량 구간 진입 waypoint의 `controller_to_next`:
  `dynamic_vehicle_rule`
- 동적차량 이탈점부터: `global_path`

수정 후 다시 읽는다.

```bash
ros2 service call /xycar_waypoint_nav/reload_route \
  std_srvs/srv/Trigger {}
```

라바콘은 `xycar_hybrid_drive`를 반드시 shadow로 실행한다. 최종 모터 publisher는
`xycar_waypoint_nav` 하나만 둔다. 동적차량 구간은
`/yolo_obstacle/stable_counts`가 stale이면 `WAIT_DYNAMIC_DETECTOR`로 정지한다.

확인할 mode 순서:

```text
GLOBAL_PATH -> CONE_RULE -> GLOBAL_PATH
GLOBAL_PATH -> DYNAMIC_VEHICLE_RULE_* -> GLOBAL_PATH
```

## 14. 즉시 중단 조건

- 물리 비상정지 담당자가 자리를 비움
- `/xycar_motor`에 의도하지 않은 publisher가 추가됨
- `/scan`, `/slam/odom`, `map -> base_footprint` 중 하나가 stale
- 지도상 차량이 다른 통로나 벽 너머로 이동
- 유리벽 쪽 점유영역이 사라져 계획 경로가 유리 방향으로 생성
- 조향 부호가 반대이거나 정지 명령 후 차량이 계속 움직임
- 동적차량 검출이 끊겼는데 주행 명령이 계속 나옴

ROS가 살아 있을 때의 추가 정지 명령:

```bash
ros2 topic pub --once /xycar_motor \
  std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0]}"
```

이 명령은 물리 비상정지를 대신하지 않는다.

## 15. 시험 후 보관할 자료

- `map.yaml`, `map.pgm`
- slam_toolbox pose graph와 data 파일
- `waypoints.yaml`
- 실측 `laser_x/y/z/yaw`
- command odom gain 측정 3회 원자료
- 각 단계 rosbag과 성공/중단 메모
- 유리벽 임시 경계물 사진과 위치
- 주행 시작점 바닥 표시 사진

큰 rosbag은 일반 Git 커밋에 바로 넣지 않는다. 지도, 설정, 체크리스트와 작은
분석 결과만 저장소에 올리고 rosbag은 Git LFS 또는 팀 저장소에 보관한 뒤 링크와
SHA-256을 기록한다.
