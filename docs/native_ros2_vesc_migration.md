# Xycar 모터 스택 native ROS2 전환

## 실행 경로

```text
이전:
/xycar_motor -> ros1_bridge -> ROS1 xycar_motor/ackermann/VESC

현재:
/xycar_motor -> ROS2 xycar_vesc_driver -> /dev/ttyMOTOR
```

`xycar_vesc_driver`가 FW 2.18 UART framing, CRC, telemetry polling, RPM,
steering servo, raw odometry, diagnostics와 시스템 telemetry를 직접 처리한다.
ROS1 컨테이너, roscore, `ros1_bridge`는 필요하지 않다.

외부 입력 계약은 유지한다.

```text
/xycar_motor  std_msgs/msg/Float32MultiArray
data[0] = angle command
data[1] = speed command
```

## ROS2 출력

| 토픽 | 타입 |
|---|---|
| `/vehicle/vesc_state` | `xycar_msgs/msg/XycarVescState` |
| `/vehicle/system_telemetry` | `xycar_msgs/msg/XycarSystemTelemetry` |
| `/odom` | `nav_msgs/msg/Odometry` |
| `/xycar_motor_bridge/debug` | `std_msgs/msg/Float32MultiArray` |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |

## 안전 기본값

- `drive_enabled=false`
- FW 2.18 불일치 시 출력 금지
- command timeout 0.5초
- telemetry timeout 0.25초
- 가속 0.3 m/s², 감속 1.5 m/s²
- 7.5~6.0V에서 출력 100%에서 0%로 감소
- 6.0V 이하 또는 VESC fault에서 출력 latch
- 6.0V 저전압 latch 또는 VESC `UNDER_VOLTAGE` fault는 fault가 사라진
  뒤 8.0V 이상이 3초간 안정되면 자동 clear
- `UNDER_VOLTAGE` 이외의 VESC fault latch는 점검 후 수동 clear
- serial exclusive open
- 종료 시 `SET_RPM 0`

실제 VESC 비휘발성 설정은 이 노드가 쓰지 않는다.

## 빌드

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
source /opt/ros/humble/setup.bash
source /home/xytron/xycar_ws/install/setup.bash

colcon build --symlink-install \
  --packages-select xycar_msgs xycar_vesc_driver \
  --allow-overriding xycar_msgs

source install/setup.bash
```

## 출력 비활성 최초 실행

```bash
docker stop ros1_container 2>/dev/null || true
pkill -TERM -f '[r]os1_bridge.*dynamic_bridge' 2>/dev/null || true

ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
  drive_enabled:=false
```

`motor` 명령도 이 저장소의 native 실행 스크립트를 가리킨다.

```bash
motor --shadow
motor --drive
```

바탕화면의 `모터 구동` 아이콘은 `motor --drive`를 실행한다. 이전 ROS1
스크립트는 `~/.local/bin/motor.ros1-backup-20260726`에 보존돼 있지만 실행
경로에서는 사용하지 않는다.

```bash
ros2 topic hz /vehicle/vesc_state
ros2 topic echo /vehicle/vesc_state --once
ros2 topic echo /diagnostics
ros2 topic info /xycar_motor -v
```

FW `2.18`, 약 50Hz telemetry, `fault_code=0`, 정상 전압과 단일 subscriber를
확인한 뒤에만 바퀴 공중 시험을 한다.

```bash
ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
  drive_enabled:=true
```

VESC fault latch 해제:

```bash
ros2 service call /vehicle/clear_motor_fault \
  std_srvs/srv/Trigger '{}'
```

저전압만 원인이었던 latch는 위 서비스 없이 자동 해제된다. 자동 해제 후에도
가속 제한을 적용해 출력이 0부터 다시 증가한다.

SLAM과 함께 실행하는 절차는
[`real_vesc_imu_lidar_odometry_KO.md`](real_vesc_imu_lidar_odometry_KO.md)에
있다.
