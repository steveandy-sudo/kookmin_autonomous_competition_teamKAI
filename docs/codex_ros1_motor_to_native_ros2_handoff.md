# Codex 작업 지시서: Xycar ROS1 모터 스택을 네이티브 ROS2로 전환

아래 `Codex에 그대로 전달할 프롬프트` 부분을 새 Codex 작업에 복사해서
사용한다. 경로가 다른 저장소에서 작업한다면 `대상 ROS2 작업공간`만 실제
경로로 바꾼다.

## Codex에 그대로 전달할 프롬프트

```text
Xycar 실차 모터 스택을 ROS1이나 ros1_bridge 없이 네이티브 ROS2 Humble로
전환해줘. 설명만 하지 말고 코드를 구현하고 빌드·테스트까지 완료해.

작업 전 저장소의 AGENTS.md와 기존 변경 사항을 확인하고, 사용자 변경은
덮어쓰지 마. 실제 VESC 설정 쓰기나 모터 구동은 내가 명시적으로 요청하지
않는 한 수행하지 말고, 가상 시리얼 VESC로 검증해.

[기존 ROS1 소스]
- /home/xytron/noetic_ws/src/xycar_motor/src/xycar_motor.py
- /home/xytron/noetic_ws/src/xycar_motor/launch/xycar_motor.launch
- /home/xytron/noetic_ws/src/vesc/vesc_driver/
- /home/xytron/noetic_ws/src/vesc/vesc_ackermann/
- /home/xytron/noetic_ws/src/vesc/vesc_msgs/

ROS1 경로는 동작을 비교하기 위한 원본 자료로 취급해. catkin/noetic
작업공간을 ROS2와 섞지 말고, 현재 작업 중인 저장소의 ROS2
`xycar_ws/src` 아래에 ament 패키지를 만들거나 기존 ROS2 패키지를 수정해.

[검증된 ROS2 기준 구현]
- /home/xytron/kookmin_ty/xycar_ws/src/xycar_device/xycar_vesc_driver/
- /home/xytron/kookmin_ty/xycar_ws/src/xycar_device/xycar_msgs/
- /home/xytron/kookmin_ty/docs/native_ros2_vesc_migration.md

[목표 구조]
기존:
ROS2 /xycar_motor
 -> ros1_bridge
 -> ROS1 xycar_motor.py
 -> ROS1 ackermann_to_vesc
 -> ROS1 vesc_driver
 -> /dev/ttyMOTOR

목표:
ROS2 /xycar_motor
 -> ROS2 xycar_vesc_driver
 -> /dev/ttyMOTOR

ROS1 컨테이너, roscore, ros1_bridge, Ackermann 중간 토픽을 실행 경로에서
완전히 제거해. ROS2 드라이버 하나만 /dev/ttyMOTOR를 소유해야 한다.

[반드시 보존할 외부 입력 계약]
- topic: /xycar_motor
- type: std_msgs/msg/Float32MultiArray
- data[0]: angle command
- data[1]: speed command
- angle clamp: -50.0 ~ 100.0
- speed clamp: -50.0 ~ 100.0
- NaN/Inf와 길이 2 미만 메시지는 무시

[보정값]
- angle_to_steering_gain: -0.0068 rad/command
- speed_to_mps_gain: 0.08 m/s/command
- speed_to_erpm_gain: 4614.0
- speed_to_erpm_offset: 0.0
- steering_to_servo_gain: -1.2135
- steering_to_servo_offset: 0.5004
- servo_min/max: 0.15 / 0.85
- wheelbase_m: 0.32
- serial: /dev/ttyMOTOR, 115200 baud
- VESC firmware: 2.18

[필수 안전 동작]
- launch 기본값 drive_enabled=false
- 명시적으로 true일 때만 0이 아닌 ERPM 출력
- command timeout 0.5초
- telemetry timeout 0.25초
- 종료 시 SET_RPM 0을 전송하고 시리얼 reader를 정상 종료
- acceleration_limit_mps2=0.3
- deceleration_limit_mps2=1.5
- 속도 명령 0→10은 약 2.7초에 걸쳐 상승하지만 최종 최고속도는 제한하지 않음
- 7.5~6.0V에서 출력 배율을 선형으로 100%→0% 감소
- 6.0V 이하 또는 VESC fault_code != 0이면 출력 latch
- 회복 전압 8.0V가 2초간 안정된 뒤 Trigger 서비스로 수동 해제
- clear service: /vehicle/clear_motor_fault, std_srvs/srv/Trigger
- telemetry가 없으면 절대 주행하지 않음

[VESC UART 프로토콜]
- FW 2.18의 short/long frame, CRC-16/CCITT를 구현
- COMM_FW_VERSION=0
- COMM_GET_VALUES=4
- COMM_SET_RPM=8
- COMM_SET_SERVO_POS=11
- 펌웨어 확인 후 약 50Hz로 값을 요청
- 기존 ROS1 vesc_packet.cpp의 FW 2.18 payload offset과 동일하게 해석
- 패킷 parser는 분할 수신, 앞쪽 noise, 잘못된 CRC를 안전하게 처리

[ROS2 출력 계약]
- /vehicle/vesc_state: xycar_msgs/msg/XycarVescState
  전압, PCB 온도, 모터/입력 전류, ERPM, m/s, 전력, duty, charge,
  energy, tachometer/displacement, fault_code 포함
- /vehicle/system_telemetry: xycar_msgs/msg/XycarSystemTelemetry
- /odom: nav_msgs/msg/Odometry
- /tf: odom -> base_link, publish_tf 파라미터 제공
- /xycar_motor_bridge/debug: std_msgs/msg/Float32MultiArray
- /diagnostics: diagnostic_msgs/msg/DiagnosticArray

[패키징]
- ROS2 Humble
- ament_python 또는 대상 저장소에 맞는 ament 패키지
- package.xml, setup.py/setup.cfg, resource marker, config YAML,
  Python launch 파일을 포함
- xycar_msgs에 필요한 ROS2 msg를 추가하고 rosidl_generate_interfaces에 등록
- launch 인자: config, port, drive_enabled

[실차에서 이미 확인된 특성]
- speed command 1 ≈ 369 ERPM, 2 ≈ 738 ERPM, 3 ≈ 1107 ERPM
- 실제 VESC speed PID minimum ERPM은 900
- 따라서 속도 1·2에서 바퀴가 안 도는 것은 기존 차량의 정상 특성
- 속도 3부터 센서리스 기동
- 바퀴 공중 속도 3 시험: 최저 7.0V, 최대 약 3685 ERPM,
  모터 전류 피크 약 21.9A, fault_code=0
- 지상에서 큰 속도를 즉시 주면 전압 강하로 멈추지만 천천히 가속하면 주행됨
- 해결 방향은 최고속도/ERPM을 낮추는 것이 아니라 출발 가속 slew 적용

[현재 실제 VESC 비휘발성 설정]
- Min/Max ERPM: -10000 / 10000
- Minimum/Maximum input voltage: 6.0 / 30.0V
- Battery cutoff start/end: 7.5 / 6.0V
- 실제 전류 제한은 Motor +60/-60A, Battery +60/-20A
- 코딩 작업 중 이 값을 자동으로 다시 쓰지 말 것

[테스트와 완료 조건]
1. 프로토콜 CRC reference vector 테스트
2. short/long frame과 분할/noise/CRC 오류 parser 테스트
3. FW 2.18 COMM_GET_VALUES decode 테스트
4. RPM/servo command encoding 테스트
5. voltage output scale 테스트:
   - 7.5V=100%, 7.0V≈66.7%, 6.75V=50%, 6.2V≈13.3%, 6.0V=0%
6. fault latch, 안정 회복, 수동 clear 테스트
7. acceleration/deceleration slew 테스트
8. PTY 기반 가상 VESC 통합 테스트:
   firmware 인식, telemetry 발행, 명령 ramp, 0.5초 watchdog,
   종료 시 마지막 RPM 0, traceback 없음
9. colcon build와 colcon test 성공
10. drive_enabled=false가 기본인 launch --show-args 확인

[금지 사항]
- ROS1 메시지나 rospy/roscpp를 새 실행 경로에 남기지 말 것
- ros1_bridge를 필요 조건으로 만들지 말 것
- /xycar_motor 타입이나 angle/speed 순서를 변경하지 말 것
- 포트를 두 드라이버가 동시에 열지 말 것
- 저전압/fault를 무시하고 강제로 출력하지 말 것
- 테스트에서 실제 모터를 자동 구동하지 말 것

완료 후 변경 파일, 빌드/테스트 결과, 남은 실차 시험 항목을 한국어로
간단히 정리해줘.
```

## 기존 ROS1 파일별 역할

| ROS1 파일 | 역할 | ROS2 처리 |
|---|---|---|
| `xycar_motor.py` | `/xycar_motor` clamp, 단위 변환, Ackermann 발행 | ROS2 driver 입력 callback과 slew로 통합 |
| `ackermann_to_vesc.cpp` | m/s→ERPM, steering→servo | ROS2 control loop로 통합 |
| `vesc_driver.cpp` | VESC 명령/상태 ROS 토픽 연결 | ROS2 serial driver로 통합 |
| `vesc_interface.cpp` | 시리얼 reader/writer | ROS2 reader thread로 통합 |
| `vesc_packet.cpp` | FW 2.18 packet encode/decode | ROS2 `protocol.py`로 재구현 |
| `vesc_to_odom.cpp` | ERPM 기반 odometry/TF | ROS2 driver odometry로 통합 |
| `vesc_msgs` | ROS1 상태 메시지 | ROS2 `xycar_msgs` 메시지로 대체 |

## 전환 시 특히 틀리기 쉬운 부분

1. ROS1 `xycar_motor.py`는 조향값을 Ackermann에 넣을 때 부호를 한 번
   반전한다. 최종 ROS2 보정값은 `-0.0068`이다.
2. 속도 command는 ERPM이 아니라 먼저 `command × 0.08 m/s`로 변환한 뒤
   `m/s × 4614`를 적용한다.
3. 기존 ROS1 ramp는 정지 출발이나 방향 전환에만 적용되고 같은 방향의
   큰 속도 증가는 즉시 전달됐다. 저전압 정지의 직접적인 소프트웨어 원인이다.
4. ROS2에서는 모든 증가 명령에 시간 기반 slew를 적용해야 한다.
5. VESC `Battery cutoff start/end`와 ROS2 voltage guard는 별개다. VESC
   fault가 발생하면 ROS2 기준과 관계없이 반드시 정지해야 한다.
6. 실제 VESC 설정은 XML 파일만 수정해서는 바뀌지 않는다. 설정 기록은
   별도 승인과 복구용 백업이 필요한 하드웨어 작업이다.

## 기준 구현 빌드

```bash
cd /home/xytron/kookmin_ty/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select xycar_msgs xycar_vesc_driver
source install/setup.bash
colcon test --packages-select xycar_msgs xycar_vesc_driver
```

## 안전한 최초 실행

```bash
ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
  drive_enabled:=false
```

`/vehicle/vesc_state`, `/diagnostics`, `fault_code=0`, `/xycar_motor`의 유일한
subscriber를 확인하고 ROS1 모터 컨테이너와 bridge가 완전히 종료된 경우에만
바퀴 공중 시험으로 넘어간다.
