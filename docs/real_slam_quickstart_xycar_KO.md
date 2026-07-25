# Kookmin TY 실차 SLAM 빠른 실행

작업 위치:

```text
/home/xytron/kookmin_ty/slam_gazebo_controller
```

이 절차는 새 장소에서 `/scan`으로 지도를 만들고, 정적 지도와
`slam_toolbox` pose graph를 함께 저장한다. 매핑 중에는 다른 자율주행
노드를 모두 종료하고 키보드 주행 노드 하나만 `/xycar_motor`를 발행한다.

## 1. 최초 1회 설치

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-rviz2
```

## 2. 빌드와 공통 환경

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller
set +u
source /opt/ros/humble/setup.bash

colcon build --packages-up-to \
  xycar_imu xycar_lidar xycar_rule_drive xycar_map_nav xycar_hybrid_drive \
  --symlink-install

source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE
```

## 3. IMU

터미널 1:

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch xycar_imu xycar_imu.launch.py
```

다른 터미널에서 다음 값이 연속으로 나오는지 확인한다.

```bash
ros2 topic hz /imu
ros2 topic echo /imu --once
```

현재 실차에서는 약 35 Hz가 정상이다. 매핑 launch는 IMU의 상대 yaw를
`slam_odom -> base_footprint` 회전에 사용한다. `/imu`가 끊기면 명령 기반
회전 추정으로 자동 전환되므로, 경고가 나오면 지도를 저장하지 않는다. 차량이
정지한 동안에는 AHRS 정지 편향이 지도를 돌리지 않도록 IMU yaw 기준을
재설정한다.

## 4. LiDAR

터미널 2:

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch xycar_lidar xycar_lidar.launch.py
```

다른 터미널에서 확인:

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 topic hz /scan
ros2 topic echo /scan --once --field header
```

`frame_id: laser_frame`, 약 10 Hz가 나와야 한다.

## 5. 새 지도 생성

`simulation` 브랜치의 2026-07-12 실차 정합값을 사용한다. 앞바퀴 중심
기준 LiDAR 위치는 `(x=0.065, y=0.000, z=0.080) m`, yaw는 `0`이다.

터미널 3:

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch xycar_map_nav real_mapping.launch.py \
  laser_x:=0.065 \
  laser_y:=0.00 \
  laser_z:=0.080 \
  laser_yaw:=0.00 \
  use_command_odom:=true \
  enable_rviz:=true
```

이 launch는 다음 전용 입력을 만든다.

- `/slam/scan_filtered`: `0.20~6.0 m` 범위만 사용하는 LiDAR, 약 10 Hz
- `/slam/odom`: IMU yaw와 모터 속도 명령을 합친 초기 추정, 약 50 Hz
- TF: `map -> slam_odom -> base_footprint -> laser_frame`
- 지도 갱신: `0.5초`, 차량 이동 `0.04 m` 또는 회전 `0.04 rad`마다 스캔 추가
- Karto 상관 탐색: `0.80 m / 0.01 m`로 coarse grid 경계 정렬

RViz의 `Map`에서 검정은 점유 장애물, 흰색은 관측된 빈 공간, 회색은 미관측
공간이다. 별도의 `LaserScan` 점은 현재 프레임 측정값이며 검은 셀로 누적되기
전에는 저장된 지도 장애물이 아니다.

실행 전에 기존 SLAM/odom 노드는 모두 종료한다. 다음 명령에서 각 SLAM 입력의
publisher가 정확히 하나여야 한다.

```bash
ros2 topic info /slam/scan_filtered -v
ros2 topic info /slam/odom -v
ros2 topic hz /slam/scan_filtered
ros2 topic hz /imu
```

RViz가 OpenGL 오류로 종료되면 같은 터미널에서 먼저 다음을 적용한다.

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

터미널 4에서 바탕화면의 모터 구동 아이콘을 먼저 실행한 뒤 수동 주행한다.

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 run xycar_rule_drive keyboard_teleop
```

`w/s` 속도, `a/d` 조향, `e` 조향 중앙, `x` 또는 `space` 정지, `q` 종료다.
처음에는 속도 명령 2~3으로 외곽 한 바퀴, 반대 방향 한 바퀴를 천천히 돌고
시작점으로 복귀해 loop closure를 확인한다.

## 6. 매핑 rosbag

터미널 5:

```bash
mkdir -p "$HOME/xycar_test_bags"

ros2 bag record \
  -o "$HOME/xycar_test_bags/new_site_mapping_02" \
  /scan /slam/scan_filtered /imu /slam/odom \
  /tf /tf_static /map /xycar_motor
```

`Ctrl+C`로 종료한 뒤 확인한다.

```bash
ros2 bag info "$HOME/xycar_test_bags/new_site_mapping_02"
```

## 7. 지도와 pose graph 저장

매핑 노드와 LiDAR를 계속 실행한 상태에서:

```bash
MAP_DIR="$HOME/xycar_maps/new_site_02"
mkdir -p "$MAP_DIR"

ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_DIR/map"

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${MAP_DIR}/map'}"

find "$MAP_DIR" -maxdepth 1 -type f -printf '%f %s bytes\n' | sort
```

최소한 `map.yaml`, `map.pgm`, pose graph 관련 파일이 있어야 한다.

기존 `new_site_01`은 비교용으로 보존하고 navigation에는 사용하지 않는다.

## 8. 저장 pose graph 재개와 정합 주의사항

차량이 매핑 시작 위치와 같은 방향에 있을 때 저장 pose graph를 다시 연다.
먼저 새 rosbag 기록을 시작한다. 복구 주행을 나중에 다시 처리하려면 이 순서를
바꾸면 안 된다.

```bash
mkdir -p "$HOME/xycar_test_bags"

ros2 bag record \
  -o "$HOME/xycar_test_bags/new_site_02_recovery_01" \
  /scan /slam/scan_filtered /imu /slam/odom \
  /tf /tf_static /map /xycar_motor
```

저장 당시 차량이 첫 노드와 실제로 같은 장소에 있고 pose graph에서도 오차가
작을 때만 dock 모드를 사용한다.

```bash
ros2 launch xycar_map_nav real_mapping.launch.py \
  pose_graph:="$HOME/xycar_maps/new_site_02/map" \
  map_start_at_dock:=true \
  use_imu_yaw:=true \
  enable_rviz:=true
```

`resume_from_pose`는 차량의 실제 위치와 저장 pose graph의 추정 위치가 이미
일치할 때 다른 지점에서 매핑을 이어가는 일반 기능이다. 큰 누적 오차를
보정하는 기능이 아니다. 시작 노드와 마지막 노드가 실제로 같은 장소인데
pose graph에서 수 m 이상 떨어졌다면 이 옵션으로 마지막 추정 좌표에서
재개하지 않는다.

2026-07-25 `new_site_02` 시험에서는 시작-종료 pose graph 오차가 `12.57 m`였다.
마지막 추정 자세 `(1.1535, 12.5180, -0.107 rad)`와 루프 검색 거리 `15 m`로
재개하자 현재 스캔이 오차가 난 끝부분에 계속 붙었으며, 같은 구간을 반복해도
기존 시작점과 정합되지 않고 지도가 더 망가졌다. 이 조합은 사용 금지다.
세부 결과는
[`real_slam_20260725_findings_KO.md`](real_slam_20260725_findings_KO.md)에
정리되어 있다.

직선 복도는 서로 비슷해 잘못된 loop closure가 생길 수 있다. 정합이 확인된
뒤에만 특징적인 모서리까지 천천히 이동했다가 시작점으로 복귀한다. rosbag은
주행을 마친 뒤 `Ctrl+C`로 종료하고 `ros2 bag info`로 `/scan`, `/imu`,
`/slam/odom`, `/tf`, `/map`의 메시지 수를 확인한다.

RViz의 `SlamToolboxPlugin`에서 `Interactive Mode`를 켜면 그래프 노드를
수동으로 움직일 수 있다. 어긋난 구간을 정렬한 뒤 `Save Changes`로 최적화하고,
잘못 움직였으면 `Clear Changes`를 누른다. 단일 마지막 노드를 시작점으로
옮기는 것만으로는 새로운 loop constraint가 생기지 않으므로 큰 누적 오차를
해결할 수 없다. 보정 후에는 기존 파일을 덮어쓰지 말고, 시작-종료 정합을
눈으로 확인한 경우에만 새 이름으로 map과 pose graph를 저장한다.

## 9. 저장 지도 localization

매핑과 키보드 노드를 종료하고 IMU와 LiDAR만 실행한 상태에서:

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch xycar_map_nav real_localization.launch.py \
  pose_graph:="$HOME/xycar_maps/new_site_02/map" \
  laser_x:=0.065 \
  laser_y:=0.00 \
  laser_z:=0.080 \
  laser_yaw:=0.00 \
  use_command_odom:=true \
  enable_rviz:=true
```

RViz에서 `2D Pose Estimate`로 시작 자세를 지정하고 다음 TF가 안정적으로
이어지는지 확인한다.

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

## 10. Waypoint 생성

localization을 유지한 상태에서 motor를 발행하지 않는 shadow로 실행한다.

```bash
WAYPOINTS="$HOME/xycar_maps/new_site_02/waypoints.yaml"

ros2 launch xycar_map_nav real_waypoint_nav.launch.py \
  drive_enabled:=false \
  map_yaml:="$HOME/xycar_maps/new_site_02/map.yaml" \
  capture_output_yaml:="$WAYPOINTS"
```

RViz의 `Publish Point`로 진행 순서대로 좌표를 찍는다. 실제 주행 전에는
`/xycar_motor` publisher가 하나뿐인지 반드시 확인한다.

```bash
ros2 topic info /xycar_motor -v
```
