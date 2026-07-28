# Xycar 모터 스택 완전 ROS2 전환

## 전환 결과

기존 경로:

```text
ROS2 /xycar_motor
  -> ROS1-ROS2 dynamic bridge
  -> ROS1 xycar_motor.py
  -> ROS1 ackermann_to_vesc
  -> ROS1 vesc_driver
  -> /dev/ttyMOTOR
```

새 경로:

```text
ROS2 /xycar_motor
  -> ROS2 xycar_vesc_driver
  -> /dev/ttyMOTOR
```

`xycar_vesc_driver`가 VESC UART framing, CRC, 펌웨어 확인, 50 Hz telemetry,
ERPM 명령, steering servo, odometry, 시스템 telemetry, diagnostics를 직접
처리한다. 따라서 모터 주행을 위해 ROS1 컨테이너, `ros1_bridge`,
`rostopic echo -p` telemetry 우회가 필요하지 않다.

외부 주행 노드의 계약은 변경하지 않았다.

```text
/xycar_motor  std_msgs/msg/Float32MultiArray
data[0] = angle command
data[1] = speed command
```

## 호환 출력

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/vehicle/vesc_state` | `xycar_msgs/msg/XycarVescState` | 전압, 전류, ERPM, fault |
| `/vehicle/system_telemetry` | `xycar_msgs/msg/XycarSystemTelemetry` | 차량 PC 상태 |
| `/odom` | `nav_msgs/msg/Odometry` | VESC 속도 기반 추정 |
| `/xycar_motor_bridge/debug` | `std_msgs/msg/Float32MultiArray` | 변환·적용 명령 확인 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 직렬·timeout·저전압 상태 |

## 안전 동작

- launch 기본값은 `drive_enabled:=false`다.
- 0.5초 동안 새 명령이 없으면 ERPM을 0으로 만든다.
- VESC telemetry가 오래되면 출력을 중지한다.
- 가속 명령에 0.3 m/s² slew-rate limit를 적용한다. 속도 명령 0→10은
  약 2.7초에 걸쳐 상승하며 최종 최고속도는 제한하지 않는다.
- 7.5~6.0 V 구간에서는 전압에 비례해 추진 출력을 줄인다.
- 6.0 V 이하 또는 VESC fault 발생 시 출력을 latch한다.
- 기본 설정에서는 전압이 회복돼도 자동 재출발하지 않는다.

저전압 원인을 제거하고 전압이 8.0 V 이상으로 안정화된 뒤 수동으로
latch를 해제한다.

```bash
ros2 service call /vehicle/clear_motor_fault std_srvs/srv/Trigger '{}'
```

VESC 비휘발성 설정도 Xytron `old_vesc_tool` 기준에 맞춘다.

```text
Minimum input voltage: 6.0 V
Maximum input voltage: 30.0 V
Battery cutoff start:  7.5 V
Battery cutoff end:    6.0 V
```

저장된 XML은
`~/xycar_ws/etc/motor_vesc/2026_0617_vesc_Motor_cfg.xml`에 반영돼 있다.
XML 수정만으로 실제 VESC가 변경되지는 않으므로 바퀴를 띄운 상태에서
기존 설정을 먼저 읽어 백업하고, 해당 XML을 연 뒤 `Write Configuration`을
한 번 수행해야 한다. 전류와 ERPM 제한은 기존 차량 값을 유지한다.

## 빌드와 실행

```bash
cd ~/kookmin_ty/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select xycar_msgs xycar_vesc_driver
source install/setup.bash

ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
  drive_enabled:=false
```

아래 조건을 확인한 다음에만 바퀴 공중 시험으로 넘어간다.

```bash
ros2 topic hz /vehicle/vesc_state
ros2 topic echo /vehicle/vesc_state
ros2 topic echo /diagnostics
ros2 topic info /xycar_motor -v
```

- VESC firmware가 인식된다.
- telemetry가 약 50 Hz다.
- `fault_code=0`이다.
- `/xycar_motor` subscriber가 `xycar_vesc_driver` 하나다.
- ROS1 모터 컨테이너와 dynamic bridge가 종료돼 있다.

바퀴를 띄우고 물리 비상 정지를 준비한 뒤:

```bash
ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
  drive_enabled:=true

ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray \
  '{data: [0.0, 1.0]}'
```

속도 1, 2, 3 순서와 speed 0 조향 시험을 통과한 뒤 바닥 주행을 시작한다.
