# track_drive_sve Gazebo 주행 패키지

국민대 자율주행 예선 과제용 ROS2 Python 통합 노드를 Gazebo Kookmin 트랙에 맞춘 패키지입니다. 기존 대회 코드의 차선/FSM 구조는 유지하되, Gazebo 토픽(`/image_raw`, `/scan`)과 우리 브리지의 `/xycar_motor` `Float32MultiArray [angle, speed]` 형식으로 바꿨습니다.

Gazebo 단독 트랙 주행에서는 신호등/라바콘 미션을 생략하고 바로 `LANE` 상태에서 차선 주행을 시작합니다.

---

## Gazebo 실행 순서

터미널 1: Gazebo 월드

```bash
cd ~/xycar_kookmin_gazebo_track
export GZ_SIM_RESOURCE_PATH=$PWD:${GZ_SIM_RESOURCE_PATH}
gz sim -r worlds/kookmin_xycar_track_final.sdf
```

터미널 2: Gazebo ROS 브리지 + RViz

```bash
cd ~/xycar_kookmin_gazebo_track/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xycar_gazebo_bridge xycar_gazebo_rviz.launch.py
```

터미널 3: track_drive_sve 주행

```bash
cd ~/xycar_kookmin_gazebo_track/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select track_drive_sve --symlink-install
source install/setup.bash
ros2 launch track_drive_sve gazebo_drive.launch.py drive_mode:=auto
```

검증만 하고 모터 발행을 막으려면:

```bash
ros2 launch track_drive_sve gazebo_drive.launch.py drive_mode:=shadow
```

---

## 1. 검증할 때 이 창/이 로그만 보면 됩니다

실행하면 OpenCV 창 **`TrackDrive FSM Debug`**가 뜹니다.

이 창에서 제일 중요한 것은 맨 위 큰 글씨입니다.

- `WAIT_START`: 시작 신호 대기 stub
- `CONE`: 라바콘 통과 중
- `LANE`: 기본 차선주행 중
- `DECISION`: 4구 신호/경찰차/좌회전 판단 중
- `TURN_LEFT`: 좌회전 실행 stub 중
- `FINISH`: 3바퀴 완료, 정지

그 아래 줄에는 “다음 상태로 가려면 무엇이 필요한지”가 한국어로 나옵니다. 예를 들어 `CONE`에서는 `콘 미검출 3/8, 차선 4/5`처럼 현재 조건 충족 정도가 보입니다.

터미널은 평소에 조용히 요약만 찍고, 상태가 바뀌는 순간에만 아래처럼 굵은 로그가 나옵니다.

```text
================ 상태 전환 ================
  CONE → LANE
  이유: 콘 미검출 8/8 + 차선 안정 5/5 충족
  현재 lap=0, route=MAIN
==========================================
```

---

## 2. 어느 파일을 언제 건드리는가

| 하고 싶은 일 | 건드릴 파일 | 주의 |
|---|---|---|
| 임계값, 시간, 속도, 디바운스 바꾸기 | `track_drive_sve/config.py` | 가장 먼저 여기만 수정하세요. |
| 좌회전 동작 채우기 | `track_drive_sve/fsm.py`의 `TURN_LEFT` 부분 | 현재는 시간+고정 조향 stub입니다. IMU yaw 방식 안내 주석도 들어 있습니다. |
| 보행자 정지 판정 넣기 | `track_drive_sve/interrupts.py`의 `_pedestrian_action_stub()` | 감지되면 `active=True`, `speed_cap=0.0`으로 바꾸면 됩니다. |
| 차량 회피/감속 넣기 | `track_drive_sve/interrupts.py`의 `_vehicle_avoid_action_stub()` | 감속은 `speed_cap`, 조향 개입은 `angle_override`를 쓰면 됩니다. |
| 3구/4구 신호등 인식 넣기 | `track_drive_sve/interrupts.py`, `world_model.py` | Gazebo 단독 주행에서는 기본 생략입니다. |
| 경찰차 차단 판정 넣기 | `track_drive_sve/lidar_utils.py`, `interrupts.py` | 기본은 stub=False입니다. LiDAR ROI 조건은 `config.py`에 있습니다. |
| 도착선 검출 튜닝 | `track_drive_sve/config.py`, `world_model.py` | ROI/흑백 비율/transition 값을 조정하세요. |
| 차선 알고리즘 수정 | `track_drive_sve/lane_core.py` | **검증된 코드이므로 건드리지 않는 것을 권장합니다.** |
| 라바콘 알고리즘 수정 | `track_drive_sve/cone_core.py` | Gazebo 단독 주행에서는 기본 생략입니다. |
| 디버그 화면 문구/레이아웃 수정 | `track_drive_sve/debug_tools.py` | 한글 폰트가 없으면 `python3-pil`과 한글 폰트를 설치하세요. |

---

## 3. track_drive_sve 단독 실행

처음 검증은 shadow 모드로 실행합니다.

```bash
cd ~/xycar_kookmin_gazebo_track/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select track_drive_sve --symlink-install
source install/setup.bash
ros2 run track_drive_sve track_drive_sve
```

실제 `/xycar_motor`를 발행하려면 auto 모드로 실행합니다.

```bash
ros2 run track_drive_sve track_drive_sve --ros-args -p drive_mode:=auto
```

- `shadow`: FSM/인지/제어 계산은 하지만 **모터 publish 없음**
- `auto`: FSM/인지/제어 계산 후 **모터 publish 있음**

---

## 4. 각 상태를 손코딩으로 확인하는 체크리스트

- `WAIT_START`: Gazebo 단독 주행에서는 실행 직후 잠깐 보이고 바로 `LANE`으로 넘어가면 정상입니다.
- `CONE`: 전체 미션 launch에서는 라바콘 구간입니다. Gazebo 단독 주행에서는 기본 생략됩니다.
- `LANE`: 대부분 구간에서 계속 `LANE`이면 정상입니다. 차선 출력과 최종 출력이 거의 같아야 합니다.
- `DECISION`: 현재 4구 anchor 인식은 stub입니다. OpenCV 창에서 `d` 키를 누르면 `DECISION` 진입을 강제로 확인할 수 있습니다.
- `TURN_LEFT`: `DECISION`에서 `a` 키를 누르면 `TURN_LEFT`로 들어갑니다. 시간이 지나거나 `t` 키를 누르면 `LANE`으로 복귀해야 합니다.
- `FINISH`: `LANE`에서 도착선이 검출되면 lap이 올라갑니다. 수동 검증은 `l` 키로 lap 카운트를 강제할 수 있습니다. lap이 3이 되면 `FINISH`입니다.

검증용 키는 다음과 같습니다.

```text
d = DECISION 강제 진입
l = 도착선 1회 카운트
c = 라바콘 종료 강제
a = 좌회전 선택
s = 직진 선택
t = TURN_LEFT 완료
```

---

## 5. 지금 stub인 부분

- `TURN_LEFT`: 실제 교차로 좌회전 알고리즘은 아직 없습니다. 현재는 `config.TURN_LEFT_ANGLE`, `TURN_LEFT_SPEED`, `TURN_LEFT_SEC`를 쓰는 시간+고정 조향 임시 동작입니다. 더 정확하게 하려면 `fsm.py`의 IMU yaw 기반 안내 주석대로 바꾸세요.
- 보행자 정지: `interrupts.py`의 `_pedestrian_action_stub()`에 실제 인식 코드를 넣어야 합니다.
- 차량 회피: `interrupts.py`의 `_vehicle_avoid_action_stub()`에 실제 회피/감속 코드를 넣어야 합니다.
- 3구/4구 신호등: 3구 출발은 현재 바로 허용, 4구 좌회전 화살표는 현재 항상 False입니다.
- 경찰차 판단: 기본은 항상 False입니다. LiDAR 기반 후보 함수는 `lidar_utils.police_blocking_left_stub()`에 준비되어 있습니다.

---

## 6. 파일 구조

```text
track_drive_sve/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/track_drive_sve
└── track_drive_sve/
    ├── __init__.py
    ├── track_drive.py
    ├── config.py
    ├── lane_core.py
    ├── cone_core.py
    ├── fsm.py
    ├── world_model.py
    ├── interrupts.py
    ├── lidar_utils.py
    └── debug_tools.py
```

---

## 7. 설치 의존성

`package.xml`에 다음 실행 의존성을 넣었습니다.

- `rclpy`
- `sensor_msgs`
- `std_msgs`
- `cv_bridge`
- `python3-opencv`
- `python3-numpy`
- `python3-scipy`
- `python3-pil`

한글이 네모로 나오면 Ubuntu에 한글 폰트가 없을 수 있습니다.

```bash
sudo apt update
sudo apt install fonts-noto-cjk python3-pil
```
