# Native ROS2 VESC·IMU odometry와 LiDAR SLAM

## 구성

ROS1 컨테이너와 `ros1_bridge` 없이 ROS2 노드 하나가 VESC 시리얼을 직접
소유한다.

```text
/xycar_motor
  -> xycar_vesc_driver
  -> /dev/ttyMOTOR

/dev/ttyMOTOR
  -> /vehicle/vesc_state
  -> vesc_imu_odom_node + /imu
  -> /slam/odom
  -> TF slam_odom -> base_footprint

/scan -> /slam/scan_filtered -> slam_toolbox
  -> TF map -> slam_odom
```

최종 TF는 다음 한 줄이어야 한다.

```text
map -> slam_odom -> base_footprint -> laser_frame
```

native 드라이버의 raw `/odom`은 호환 출력이지만, SLAM launch에서는
`publish_tf=false`로 덮어쓴다. 차량 TF는 VESC tachometer와 IMU yaw를
결합한 `vesc_imu_odom_node` 하나만 발행한다.

## 빌드

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source /home/xytron/xycar_ws/install/setup.bash

colcon build --packages-up-to xycar_map_nav --symlink-install \
  --allow-overriding xycar_msgs xycar_cam

source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE
```

## 금지 조합

native 드라이버를 실행할 때 다음 프로세스가 없어야 한다.

```bash
docker ps --format '{{.Names}}' | grep '^ros1_container$'
pgrep -af 'ros1_bridge|dynamic_bridge|vesc_driver_node'
```

`/dev/ttyMOTOR`는 한 프로세스만 열 수 있다. native 드라이버는 Linux
exclusive serial open을 사용하므로 두 번째 드라이버는 연결에 실패한다.

## 최초 shadow 확인

바퀴를 띄우고, 먼저 출력이 비활성화된 상태로 실행한다.

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
  drive_enabled:=false
```

다른 터미널에서 확인한다.

```bash
ros2 topic hz /vehicle/vesc_state
ros2 topic echo /vehicle/vesc_state --once
ros2 topic echo /diagnostics
ros2 topic info /xycar_motor -v
```

통과 조건:

- firmware `2.18`
- `/vehicle/vesc_state` 약 50 Hz
- `fault_code=0`
- 전압 8.0V 이상
- `/xycar_motor` subscriber가 `xycar_vesc_driver` 하나
- 명령을 publish해도 `drive_enabled=false`에서는 ERPM이 0

## 센서 실행

IMU:

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE
ros2 launch xycar_imu xycar_imu.launch.py
```

LiDAR:

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE
ros2 launch xycar_lidar xycar_lidar.launch.py
```

## 새 지도 생성

shadow 확인을 끝내고 별도 native 드라이버를 종료한 뒤 실행한다.
`vesc_drive_enabled:=true`를 생략하면 지도와 odometry는 보이지만 키보드
명령으로 차량이 움직이지 않는다.

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch xycar_map_nav real_mapping.launch.py \
  odom_source:=vesc_imu \
  start_native_vesc_driver:=true \
  vesc_drive_enabled:=true \
  laser_x:=0.065 \
  laser_y:=0.00 \
  laser_z:=0.080 \
  laser_yaw:=0.00 \
  enable_rviz:=true
```

수동 조종:

```bash
ros2 run xycar_rule_drive keyboard_teleop
```

이 launch 자체는 `/xycar_motor`를 발행하지 않는다. 키보드 노드를 종료하거나
명령이 0.5초 이상 끊기면 native 드라이버가 RPM 0을 전송한다.

## SLAM 입력 검증

```bash
ros2 topic hz /vehicle/vesc_state
ros2 topic hz /imu
ros2 topic hz /scan
ros2 topic hz /slam/odom

ros2 topic info /vehicle/vesc_state -v
ros2 topic info /slam/odom -v
ros2 run tf2_ros tf2_echo slam_odom base_footprint
ros2 run tf2_ros tf2_echo map slam_odom
```

`/vehicle/vesc_state`, `/slam/odom`, `slam_odom -> base_footprint`는 각각
publisher가 하나여야 한다.

native 드라이버를 이미 별도로 실행한 경우에는 포트 중복을 피한다.

```bash
ros2 launch xycar_map_nav real_mapping.launch.py \
  odom_source:=vesc_imu \
  start_native_vesc_driver:=false
```

## 저장 지도 localization

```bash
ros2 launch xycar_map_nav real_localization.launch.py \
  pose_graph:=$HOME/xycar_maps/new_site_02/map \
  odom_source:=vesc_imu \
  start_native_vesc_driver:=true \
  vesc_drive_enabled:=false \
  laser_x:=0.065 \
  laser_y:=0.00 \
  laser_z:=0.080 \
  laser_yaw:=0.00 \
  enable_rviz:=true
```

실제 waypoint 주행을 시작할 때만 localization launch를
`vesc_drive_enabled:=true`로 다시 실행한다.

## 안전 동작

- launch 기본 `drive_enabled=false`
- command timeout 0.5초
- telemetry timeout 0.25초
- 가속 0.3 m/s², 감속 1.5 m/s²
- 7.5~6.0V 출력 선형 감소
- 6.0V 이하 또는 VESC fault에서 출력 latch
- 6.0V 저전압 latch 또는 `UNDER_VOLTAGE` fault는 fault가 사라진 뒤
  8.0V 이상이 3초간 유지되면 자동 해제
- `UNDER_VOLTAGE` 이외의 VESC fault latch는 자동 해제하지 않음
- FW 2.18이 아니면 출력 금지
- 종료 시 마지막 `SET_RPM 0`

저전압 fault가 사라지고 전압이 8.0V 이상으로 3초간 안정되면 driver가
자동 해제한다. 저전압만 원인이었던 latch에는 아래 서비스가 필요하지 않다.
다른 VESC fault는 원인을 점검하고 정상 전압이 확보된 뒤 아래 서비스로
수동 해제한다.

```bash
ros2 service call /vehicle/clear_motor_fault \
  std_srvs/srv/Trigger '{}'
```
