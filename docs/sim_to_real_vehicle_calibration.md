# Sim-to-Real 차량 조건 정합 체크리스트

이 문서는 `worlds/kookmin_xycar_track_final.sdf`를 최종 맵으로 두고, 룰베이스 주행 전에 시뮬레이션 차량을 실제 Xycar 조건에 맞추기 위해 실차에서 확인해야 할 항목을 정리한다.

핵심 목표는 외형을 예쁘게 맞추는 것이 아니라, 같은 명령을 넣었을 때 실차와 시뮬 차량이 비슷한 카메라 시야, 조향 반경, 속도, 지연시간을 갖게 만드는 것이다.

## 현재 시뮬 차량 기준값

현재 월드의 차량 모델은 `xycar_ackermann`이고 주요 값은 다음과 같다.

| 항목 | 현재 시뮬 값 | SDF에서 바꿀 곳 |
|---|---:|---|
| 차량 모델 | `xycar_ackermann` | `<model name='xycar_ackermann'>` |
| 실차 프레임 기준 | `0.55 x 0.30 x 0.25 m` | Y모델 스펙 자료 |
| Gazebo body 충돌 박스 | `0.55 x 0.30 x 0.12 m` | `chassis` box size |
| 대략 차량 폭 | `0.30 m` | wheel y pose + wheel length |
| wheelbase | `0.32 m` | `<wheel_base>` |
| wheel separation | `0.265 m` | `<wheel_separation>` |
| kingpin width | `0.240 m` | `<kingpin_width>` |
| wheel radius | `0.06 m` | `<wheel_radius>` |
| steering limit | `0.289 rad` 약 `16.6 deg` | `<steering_limit>`, steering joint limit |
| max velocity | `8.0 m/s` | `<max_velocity>` |
| min velocity | `-4.0 m/s` | `<min_velocity>` |
| camera | `/image_raw`, 1280x1024, 30Hz, HFOV 약 170도, equidistant fisheye | `front_camera` sensor |
| lidar | `/scan`, 505 samples, 10Hz, `-pi~pi`, `0.1~12 m` | `lidar` sensor |
| sensor origin | 앞바퀴 중심 기준 | 실차 측정 기준 |
| camera pose | `(-0.04, 0.00, 0.17) m` | 앞바퀴 중심 기준 |
| lidar pose | `(0.08, 0.00, 0.06) m` | 앞바퀴 중심 기준 |

실차 카메라 영상에서 보이는 실내 기준물도 시뮬에 추가했다. 벽, 나무 몰딩, 락커, 책상/의자, 천장 조명, 콘은 `room_*` 접두어를 가진 **독립 Gazebo 모델**이다. Entity Tree에서 각각 선택해 이동/저장할 수 있고, 주행 물리에는 간섭하지 않도록 충돌 없는 시각 객체로만 둔다.

2026-07-07 실차 조사에서 ROS1 VESC 모터 경로의 변환식이 확인되었으므로, Gazebo wrapper는 아래 값을 우선 기준으로 둔다.

```text
steering_angle(rad) = -0.0068 * angle_command
speed_mps = 0.08 * speed_command
angle_command clamp = -50 ~ 100
speed_command clamp = -50 ~ 100
servo clipping steering range ~= -0.2881 ~ +0.2888 rad
```

이를 반영해 `xycar_ws/src/xycar_gazebo_bridge` 패키지를 추가했다. 이 패키지는 `std_msgs/msg/Float32MultiArray [angle, speed]`를 받아 `/model/xycar_ackermann/cmd_vel` `geometry_msgs/msg/Twist`로 바꾼다.

최종 맵 파일:

```bash
gz sim -r worlds/kookmin_xycar_track_final.sdf
```

실차 코드와 같은 모터 명령으로 Gazebo 차량을 움직일 때:

```bash
source /opt/ros/humble/setup.bash
cd xycar_ws
colcon build --packages-select xycar_gazebo_bridge
export AMENT_PREFIX_PATH=$PWD/install/xycar_gazebo_bridge:${AMENT_PREFIX_PATH}
source install/xycar_gazebo_bridge/share/xycar_gazebo_bridge/package.bash
ros2 launch xycar_gazebo_bridge xycar_gazebo_bridge.launch.py
```

카메라/라이다를 RViz에서 확인할 때:

```bash
source /opt/ros/humble/setup.bash
cd xycar_ws
colcon build --packages-select xycar_gazebo_bridge --symlink-install
export AMENT_PREFIX_PATH=$PWD/install/xycar_gazebo_bridge:${AMENT_PREFIX_PATH}
source install/xycar_gazebo_bridge/share/xycar_gazebo_bridge/package.bash
ros2 launch xycar_gazebo_bridge xycar_gazebo_rviz.launch.py
```

RViz는 Gazebo bridge의 원본 센서 토픽을 직접 본다.

| 토픽 | RViz 표시 | frame_id |
|---|---|---|
| `/scan` | LaserScan display | `xycar_ackermann/chassis/lidar` |
| `/image_raw` | Image display | `xycar_ackermann/chassis/front_camera` |
| `/camera_info` | Camera info 확인용 | `xycar_ackermann/chassis/front_camera` |

주의: `python3 scripts/generate_kookmin_track.py`를 다시 실행하면 Gazebo에서 수동 저장한 맵/차선 위치가 덮일 수 있다.

## `xycar_ws` 코드 분석 결과

현재 이 폴더에 가져온 `xycar_ws`는 ROS2 Humble 기반 코드로 보인다. 실차 주행 코드와 장치 드라이버를 보면, 앞으로 시뮬레이션을 맞출 때 아래 인터페이스를 우선 기준으로 잡아야 한다.

### 코드에서 확인된 주요 토픽

| 역할 | 코드에서 쓰는 토픽 | 코드에서 쓰는 타입 | 확인한 파일 |
|---|---|---|---|
| 카메라 이미지 | `/image_raw` | `sensor_msgs/msg/Image` | `app_hough_drive`, `my_hough`, `my_cam` |
| 모터 명령 | `xycar_motor` | 주로 `std_msgs/msg/Float32MultiArray` | `track_drive`, `my_motor`, `app_8_drive`, `app_hough_drive`, `app_sensor_drive` |
| 모터 명령, 구형/시뮬 | `xycar_motor` 또는 `/xycar_motor` | `xycar_msgs/msg/XycarMotor` | `app_rule_drive_sim`, `xycar_sim_driving`, `xycar_sim_parking` |
| LiDAR | `scan` 또는 `/scan` | `sensor_msgs/msg/LaserScan` | `app_sensor_drive`, `my_lidar`, `xycar_lidar` |
| 초음파 | `xycar_ultrasonic` | `std_msgs/msg/Int32MultiArray` | `my_ultra`, `app_ultra_viewer` |
| IMU | 설정 파일 기반 | 보통 `sensor_msgs/msg/Imu` 계열 | `xycar_imu` |

중요한 점: `xycar_msgs/msg/XycarMotor` 메시지는 `header`, `float32 angle`, `float32 speed`로 정의되어 있다. 하지만 현재 주행 예제 대부분은 이 타입을 주석 처리하고 `std_msgs/msg/Float32MultiArray`에 `[angle, speed]` 순서로 넣어 발행한다.

따라서 실차에서 제일 먼저 아래 명령으로 실제 모터 드라이버가 어떤 타입을 받는지 확인해야 한다.

```bash
source /opt/ros/humble/setup.bash
source xycar_ws/install/setup.bash
ros2 topic list -t | grep xycar_motor
ros2 topic info /xycar_motor
ros2 topic info xycar_motor
```

`/xycar_motor`와 `xycar_motor`는 ROS2에서 네임스페이스 상태에 따라 다르게 보일 수 있으니 둘 다 확인한다. 최종 rule-based와 Gazebo bridge는 실차에서 실제로 살아 있는 이름과 타입에 맞춘다.

### 코드에서 확인된 주행 노드별 의미

| 패키지/파일 | 센서 입력 | 모터 출력 | 현재 명령 스케일 힌트 | Sim-to-Real에서 할 일 |
|---|---|---|---|---|
| `track_drive/track_drive.py` | 없음 | `Float32MultiArray` `xycar_motor` | angle `0`, speed를 `-100~100`까지 변화 | 직진 speed command-to-m/s 측정에 사용 가능 |
| `study/my_motor/go.py` | 없음 | `Float32MultiArray` `xycar_motor` | angle `0`, speed `20` 고정 | 직진 주행/속도 기본 확인 |
| `app_8_drive/app_8_drive.py` | 없음 | `Float32MultiArray` `xycar_motor` | angle `-100~100`, speed 기본 `12` | 조향 command-to-wheel-angle 측정에 사용 가능 |
| `app_hough_drive/app_hough_drive.py` | `/image_raw` | `Float32MultiArray` `xycar_motor` | speed `12`, angle = `x_midpoint - 320` | 카메라 ROI/FOV/노출을 시뮬과 맞출 기준 |
| `study/my_hough/hough_drive.py` | `/image_raw` | `Float32MultiArray` `xycar_motor` | speed `12`, angle = `x_midpoint - 320` | 위와 같은 차선 인식 baseline |
| `app_sensor_drive/app_sensor_drive.py` | `scan` | `Float32MultiArray` `xycar_motor` | angle `-50/0/50`, speed `10` | LiDAR 방향/인덱스 정합 확인 |
| `app_rule_drive_sim/app_rule_drive_sim.py` | `ultrasonic` | `XycarMotor` `xycar_motor` | speed `10`, angle = R-L | 기존 pygame 시뮬용이라 Gazebo 실차정합 기준으로는 보조 참고 |

현재 Gazebo camera lane 코드가 가정하는 이미지 조건:

| 항목 | 코드 기준값 |
|---|---:|
| 이미지 폭 | `1280 px` |
| 이미지 높이 | `1024 px` |
| FPS 가정 | `30 Hz` |
| 입력 카메라 모델 | `170 deg`, `equidistant fisheye` |
| 인지 투영 | `bev_homography` |
| BEV 출력 | `640 x 220 px` |
| BEV ROI row | `0 ~ 219` |
| 화면 중심 | `320 px` |
| Canny threshold | `60, 75` 또는 예제에 따라 `60, 70` |
| HoughLinesP | rho `1`, theta `1 deg`, threshold `50`, minLineLength `50`, maxLineGap `20` |
| 노출 고정 | `/dev/videoCAM`, `exposure_time_absolute=100` |

이 값들이 실차 카메라의 실제 시야와 맞아야 한다. 특히 시뮬 카메라에서 차선이 row `300~380`에 비슷하게 들어오도록 카메라 높이, pitch, FOV를 맞추는 것이 중요하다.

현재 LiDAR 코드가 가정하는 조건:

| 항목 | 코드/설정 기준값 |
|---|---:|
| 드라이버 launch | `xycar_lidar/launch/xycar_lidar.launch.py` |
| 파라미터 파일 | `xycar_lidar/params/ydlidar.yaml` |
| 포트 | `/dev/ttyLIDAR` |
| frame_id | `laser_frame` |
| baudrate | `512000` |
| angle_min / angle_max | `-180 deg / 180 deg` |
| range_min / range_max | 코드 설정 후보 `0.1 m / 16.0 m`, Y모델 스펙 기준 `0.1 m / 12.0 m` |
| frequency | `10 Hz` |
| 코드 사용 range | `msg.ranges[1:505]` |
| app_sensor_drive 비교 인덱스 | right `252-63`, left `252+63` |

LiDAR는 실차에서 `ros2 topic echo /scan --once`로 `angle_min`, `angle_increment`, `ranges` 길이가 코드 가정과 일치하는지 확인해야 한다. 인덱스가 안 맞으면 rule-based에서 좌우 판단이 뒤집히거나 엉뚱한 방향을 보게 된다.

### 이 복사본에서 아직 불확실한 것

1. 여러 launch 파일에서 `xycar_motor.launch.py` include가 주석 처리되어 있다.
2. 이 `xycar_ws/src` 안에는 실제 `xycar_motor` 드라이버 패키지가 보이지 않는다.
3. 따라서 실차에서는 모터 드라이버가 별도 설치되어 있거나, 다른 launch/system service에서 이미 떠 있을 가능성이 있다.
4. web controller 일부 파일은 `XycarMotor`를 쓰는 버전과 `Float32MultiArray`를 쓰는 버전이 섞여 있다.
5. 기존 pygame 시뮬레이터는 `XycarMotor` 타입을 구독하지만, 실차 예제들은 대체로 `Float32MultiArray`로 바뀌어 있다.

결론: Gazebo 쪽 제어 인터페이스는 처음부터 둘 중 하나로 단정하지 말고, 실차에서 확인한 `/xycar_motor` 타입을 기준으로 wrapper를 만든다. 현재 코드만 보면 우선 후보는 `std_msgs/msg/Float32MultiArray`이며 `data[0]=angle`, `data[1]=speed`이다.

### 실차에서 바로 실행할 확인 명령

실차에서 `xycar_ws`를 source한 뒤 아래를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash

ros2 topic list -t
ros2 node list

ros2 topic info /image_raw
ros2 topic hz /image_raw

ros2 topic info /scan
ros2 topic hz /scan
ros2 topic echo /scan --once

ros2 topic info /xycar_motor
ros2 topic info xycar_motor
```

모터 타입이 `std_msgs/msg/Float32MultiArray`로 확인되면, 바퀴를 띄운 상태에서 아주 작은 명령부터 테스트한다.

```bash
ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0]}"
ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray "{data: [0.0, 5.0]}"
ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray "{data: [20.0, 0.0]}"
ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray "{data: [-20.0, 0.0]}"
```

모터 타입이 `xycar_msgs/msg/XycarMotor`로 확인되면 아래처럼 테스트한다.

```bash
ros2 interface show xycar_msgs/msg/XycarMotor
ros2 topic pub --once /xycar_motor xycar_msgs/msg/XycarMotor "{angle: 0.0, speed: 0.0}"
ros2 topic pub --once /xycar_motor xycar_msgs/msg/XycarMotor "{angle: 0.0, speed: 5.0}"
ros2 topic pub --once /xycar_motor xycar_msgs/msg/XycarMotor "{angle: 20.0, speed: 0.0}"
ros2 topic pub --once /xycar_motor xycar_msgs/msg/XycarMotor "{angle: -20.0, speed: 0.0}"
```

명령 테스트는 반드시 바퀴를 띄우거나 차량이 움직여도 안전한 공간에서 한다. 처음부터 `speed=20`, `angle=100` 같은 큰 값을 넣지 않는다.

## 실차에서 반드시 알아와야 할 값

### 1. 차량 기구 치수

줄자나 캘리퍼스로 직접 재야 한다. 단위는 모두 meter로 기록한다.

| 항목 | 측정 방법 | 시뮬 반영 |
|---|---|---|
| wheelbase | 앞바퀴 축 중심부터 뒷바퀴 축 중심까지 | `<wheel_base>` |
| front tread / rear tread | 좌우 바퀴 중심 간 거리 | `<wheel_separation>`, wheel y pose |
| 바퀴 반지름 | 바닥부터 바퀴 축 중심까지 또는 지름/2 | `<wheel_radius>` |
| 바퀴 폭 | 타이어 좌우 폭 | wheel cylinder `<length>` |
| 차량 전체 길이 | 가장 앞부터 가장 뒤까지 | chassis size |
| 차량 전체 폭 | 가장 왼쪽부터 가장 오른쪽까지 | chassis size, wheel pose |
| 차량 높이 | 바닥부터 가장 높은 구조물까지 | chassis/sensor pose |
| 무게 | 배터리 포함 주행 상태 | `<mass>` |
| 카메라 위치 | 앞바퀴 중심 기준 x/y/z | camera sensor pose |
| 라이다 위치 | 앞바퀴 중심 기준 x/y/z/yaw | lidar sensor pose |
| 카메라 pitch | 바닥 기준 아래로 숙인 각도 | camera pose |

추천 좌표계:

- `x`: 차량 앞쪽이 `+`
- `y`: 차량 왼쪽이 `+`
- `z`: 위쪽이 `+`
- 원점: 앞바퀴 중심

현재 시뮬과 RViz TF는 실차에서 측정한 앞바퀴 중심 기준 센서 좌표로 통일한다.
Gazebo SDF 내부에서는 센서가 `chassis` 링크 아래에 붙어 있으므로,
앞바퀴 중심 기준 좌표를 `chassis` 링크 기준 pose로 변환해서 넣는다.

### 2. 조향 범위와 조향 명령 매핑

실차에서 가장 중요한 값이다. 같은 조향 명령을 넣었을 때 실제 앞바퀴 각도가 몇 도가 되는지 알아야 한다.

기록해야 할 것:

| 항목 | 예시 |
|---|---|
| 조향 명령 토픽 | 우선 후보: `/xycar_motor` 또는 `xycar_motor` |
| 메시지 타입 | 우선 확인: `std_msgs/msg/Float32MultiArray`; 보조/구형: `xycar_msgs/msg/XycarMotor` |
| 조향 명령 범위 | 코드 예제 기준 `-100 ~ +100`, 센서주행 기준 `-50/0/50`; 실차에서 최종 확인 |
| 좌회전 부호 | `+`가 좌회전인지 우회전인지 |
| 우회전 부호 | `-`가 우회전인지 좌회전인지 |
| 최대 좌 조향각 | deg 또는 rad |
| 최대 우 조향각 | deg 또는 rad |
| 명령 0일 때 직진 오프셋 | 예: 실제 직진은 `angle=2` |
| 조향 응답 지연 | 명령 후 바퀴가 움직이기 시작하는 시간 |
| 조향 속도 제한 | 중앙에서 최대 조향까지 걸리는 시간 |

실측 방법:

1. 차량을 바닥에서 들어 올리거나 바퀴가 자유롭게 움직이게 둔다.
2. 아주 작은 조향 명령부터 넣는다.
3. 앞바퀴가 실제로 몇 도 돌아갔는지 각도기 또는 촬영 영상으로 측정한다.
4. `명령값 -> 실제 조향각` 표를 만든다.

권장 측정 표:

| command | left wheel deg | right wheel deg | 차량 기준 평균 조향각 deg | 비고 |
|---:|---:|---:|---:|---|
| -50 | | | | |
| -30 | | | | |
| -10 | | | | |
| 0 | | | | |
| 10 | | | | |
| 20 | | | | |
| 30 | | | | |
| 50 | | | | |
| 100 | | | | |

시뮬 반영:

- 최대 조향각은 `<steering_limit>`에 rad로 넣는다.
- 명령값이 실차 전용 범위라면 시뮬 제어 노드에서 `command -> rad` 변환 함수를 둔다.
- 좌우 최대각이 다르면 우선 작은 쪽에 맞추고, 나중에 비대칭 보정 테이블을 둔다.

### 3. 속도 명령 매핑

실차가 `speed=5`일 때 실제 몇 m/s로 가는지 알아야 한다.

기록해야 할 것:

| 항목 | 예시 |
|---|---|
| 속도 명령 범위 | 코드 예제 기준 `-100 ~ +100`, 보통 주행 예제는 `10~20`; 실차에서 최종 확인 |
| 출발 deadband | 예: speed 4 이하는 움직이지 않음 |
| 최대 안정 속도 | 트랙에서 안전하게 돌 수 있는 속도 |
| 후진 가능 여부 | 가능/불가능 |
| 가속 지연 | 명령 후 속도가 올라가는 시간 |
| 감속/브레이크 특성 | 명령 0에서 얼마나 밀리는지 |

실측 방법:

1. 직선 구간에 1m 또는 2m 기준선을 붙인다.
2. 고정 속도 명령을 넣고 통과 시간을 영상으로 잰다.
3. `속도 = 거리 / 시간`으로 계산한다.
4. 같은 명령을 3회 이상 반복한다.

권장 측정 표:

| speed command | distance m | time sec 1 | time sec 2 | time sec 3 | mean m/s | 비고 |
|---:|---:|---:|---:|---:|---:|---|
| 5 | 1.0 | | | | | |
| 10 | 1.0 | | | | | |
| 15 | 1.0 | | | | | |
| 20 | 1.0 | | | | | |
| 50 | 1.0 | | | | | |
| 100 | 1.0 | | | | | |

시뮬 반영:

- 실제 최대 주행 속도에 맞춰 `<max_velocity>`를 수정한다.
- 실차가 명령값 기반이면 시뮬 wrapper에서 `speed_command -> m/s` lookup table을 쓴다.
- 출발 deadband가 있으면 시뮬 제어에도 deadband를 넣어야 rule-based가 실차에서 갑자기 약해지지 않는다.

### 4. 최소 회전 반경

조향각을 정확히 재기 어렵다면 최소 회전 반경을 직접 재도 된다.

실측 방법:

1. 조향을 최대 좌/우로 고정한다.
2. 낮은 속도로 원을 그리며 주행한다.
3. 바닥에 그려진 원 궤적의 반지름을 잰다.
4. 좌회전/우회전 각각 측정한다.

Ackermann 근사식:

```text
steering_angle = atan(wheelbase / turning_radius)
```

예를 들어 wheelbase가 `0.32m`, 최소 회전 반경이 `0.70m`이면:

```text
atan(0.32 / 0.70) = 0.429 rad = 24.6 deg
```

이 값을 `<steering_limit>` 후보로 넣는다.

### 5. 카메라 조건

차선 기반 rule-based와 BC 학습에서는 카메라가 거의 전부다. 실차 카메라 조건을 최대한 정확히 알아와야 한다.

기록해야 할 것:

| 항목 | 확인 방법 | 시뮬 반영 |
|---|---|---|
| image topic | 코드 기준 `/image_raw` | bridge/remap |
| message type | `ros2 topic info` | subscriber 타입 |
| 해상도 | image echo/script | camera sensor width/height |
| FPS | `ros2 topic hz` | update_rate |
| FOV | 카메라 모델/캘리브레이션 | horizontal_fov |
| 카메라 높이 | 줄자 | pose z |
| 카메라 x/y 위치 | 앞바퀴 중심 기준 측정 | pose x/y |
| pitch angle | 각도기/영상 기반 | pose pitch |
| exposure/brightness | 실제 주행 이미지 저장 | sim noise/domain randomization |
| 왜곡 여부 | fish-eye/일반 렌즈 | undistort 여부 |

실차에서 이미지 확인:

```bash
ros2 topic list
ros2 topic info /image_raw
ros2 topic hz /image_raw
ros2 run rqt_image_view rqt_image_view
```

ROS1 차량이면:

```bash
rostopic list
rostopic info /image_raw
rostopic hz /image_raw
rqt_image_view
```

이미지 한 장 저장용 ROS2 예시:

```python
#!/usr/bin/env python3
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class SaveOneImage(Node):
    def __init__(self):
        super().__init__("save_one_image")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/image_raw", self.cb, 10)

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite("real_camera_sample.png", img)
        print("saved real_camera_sample.png", img.shape)
        rclpy.shutdown()


rclpy.init()
rclpy.spin(SaveOneImage())
```

저장한 이미지는 다음을 확인한다.

- 차선이 이미지에서 몇 픽셀 높이에 보이는지
- 차량 앞 범퍼/바퀴가 보이는지
- 노란 중앙선과 흰 테두리의 색이 잘 분리되는지
- 트랙 조명에서 노출이 흔들리는지

### 6. LiDAR 또는 거리 센서 조건

라이다를 rule-based에 쓸지 말지는 나중 문제지만, 시뮬 모델에는 위치를 맞춰두는 편이 좋다.

기록해야 할 것:

| 항목 | 확인 방법 |
|---|---|
| scan topic | 코드 기준 `/scan` 또는 `scan` |
| message type | `sensor_msgs/msg/LaserScan` |
| angle_min / angle_max | topic echo |
| angle_increment | topic echo |
| range_min / range_max | topic echo |
| FPS | `ros2 topic hz /scan` |
| 라이다 높이 | 줄자 |
| 라이다 x/y/yaw | 앞바퀴 중심 기준 측정 |

확인 명령:

```bash
ros2 topic info /scan
ros2 topic hz /scan
ros2 topic echo /scan --once
```

ROS1:

```bash
rostopic info /scan
rostopic hz /scan
rostopic echo /scan -n 1
```

### 7. 토픽/메시지 인터페이스

시뮬과 실차의 코드 재사용을 위해 토픽 이름과 메시지 타입을 반드시 기록한다.

실차에서 실행:

```bash
ros2 topic list -t
ros2 node list
ros2 node info /사용중인_드라이버_노드명
```

ROS1:

```bash
rostopic list
rostopic type /xycar_motor
rosnode list
rosnode info /사용중인_드라이버_노드명
```

기록 표:

| 역할 | 실차 토픽 | 타입 | 주기 | 시뮬 토픽 후보 |
|---|---|---|---:|---|
| 카메라 | `/image_raw` | `sensor_msgs/msg/Image` | | |
| 카메라 정보 | `/camera_info` | | | |
| 라이다 | `/scan` | `sensor_msgs/msg/LaserScan` | | |
| 모터 명령 | `/xycar_motor` 또는 `xycar_motor` | `std_msgs/msg/Float32MultiArray`인지 `xycar_msgs/msg/XycarMotor`인지 확인 | | |
| 속도/odom | `/odom` 또는 없음 | | | |
| IMU | `/imu` 또는 없음 | | | |

### 8. 지연시간과 주기

실차와 시뮬이 다르면 학습/제어가 흔들리는 부분이다.

확인할 것:

| 항목 | 목표 |
|---|---|
| 카메라 FPS | 실제 FPS와 시뮬 update_rate 맞추기 |
| 라이다 FPS | 실제 FPS와 시뮬 update_rate 맞추기 |
| 제어 주기 | rule-based loop Hz 결정 |
| 명령 지연 | command publish 후 차량 반응까지 시간 |
| 이미지 timestamp 지연 | 카메라 timestamp와 현재 시간 차이 |

명령:

```bash
ros2 topic hz /image_raw
ros2 topic hz /scan
ros2 topic hz /xycar_motor
```

timestamp 지연 확인용 ROS2 예시:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageDelay(Node):
    def __init__(self):
        super().__init__("image_delay_probe")
        self.sub = self.create_subscription(Image, "/image_raw", self.cb, 10)

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        print(f"image delay: {(now - stamp) * 1000:.1f} ms")


rclpy.init()
rclpy.spin(ImageDelay())
```

## 실차에서 가져와야 할 최소 자료 묶음

아래 자료만 있으면 시뮬 차량 1차 정합을 시작할 수 있다.

1. 차량 상면 사진 1장, 측면 사진 1장
2. 카메라가 보는 실제 트랙 이미지 5장
3. wheelbase, 차량 폭, 바퀴 반지름, 카메라 높이/pitch
4. 조향 command별 실제 바퀴각 표
5. speed command별 실제 m/s 표
6. `ros2 topic list -t` 또는 `rostopic list` 결과
7. `/image_raw`, `/scan`, `/xycar_motor`의 topic info 결과
8. 카메라 FPS, 라이다 FPS
9. 가능하면 짧은 rosbag: 직선 5초, 좌회전 5초, 우회전 5초

ROS2 bag 예시:

```bash
ros2 bag record /image_raw /scan /xycar_motor -o bags/real_xycar_probe
```

ROS1 bag 예시:

```bash
rosbag record /image_raw /scan /xycar_motor -O real_xycar_probe.bag
```

## 실차 측정 후 시뮬에 반영할 순서

1. 차량 치수 반영: wheelbase, wheel separation, wheel radius, chassis size
2. 조향 한계 반영: max steering angle, 좌우 부호, command scale
3. 속도 scale 반영: command-to-m/s table, max velocity, acceleration
4. 카메라 반영: 위치, 높이, pitch, FOV, 해상도, FPS
5. 라이다 반영: 위치, yaw, range, angle, FPS
6. 최종 확인: 같은 조향/속도 명령을 넣었을 때 최소 회전 반경과 직선 속도가 실차와 비슷한지 비교

## 최종 목표 파라미터 파일 형태

실차 측정이 끝나면 아래처럼 별도 YAML로 고정해두는 것이 좋다.

```yaml
vehicle:
  wheelbase_m:
  wheel_separation_m:
  wheel_radius_m:
  outer_length_m:
  outer_width_m:
  mass_kg:

steering:
  topic: /xycar_motor
  message_type: std_msgs/msg/Float32MultiArray
  float32_multi_array_order: [angle, speed]
  command_min: -50
  command_max: 100
  center_command: 0
  steering_gain_rad_per_command: -0.0068
  servo_limited_min_rad: -0.2881
  servo_limited_max_rad: 0.2888

speed:
  topic: /xycar_motor
  message_type: std_msgs/msg/Float32MultiArray
  command_min: -50
  command_max: 100
  speed_gain_mps_per_command: 0.08
  command_limited_min_mps: -4.0
  command_limited_max_mps: 8.0

camera:
  topic: /image_raw
  width: 1280
  height: 1024
  fps: 30
  horizontal_fov_deg: 170
  lens: equidistant
  frame: front_wheel_center
  x_m: -0.04
  y_m: 0.0
  z_m: 0.17
  gazebo_chassis_pose_m: [0.120, 0.0, 0.095]
  pitch_rad: 0.220
  roi_start_row: 300
  roi_end_row: 380
  roi_reference_row: 40
  exposure_device: /dev/videoCAM
  exposure_time_absolute: 100

lidar:
  topic: /scan
  fps: 10
  angle_min_rad: -3.14159
  angle_max_rad: 3.14159
  samples: 505
  range_min_m: 0.1
  range_max_m: 12.0
  used_range_slice: [1, 505]
  left_index_in_slice: 315
  right_index_in_slice: 189
  frame: front_wheel_center
  x_m: 0.08
  y_m: 0.0
  z_m: 0.06
  gazebo_chassis_pose_m: [0.240, 0.0, -0.015]
  yaw_deg: 0.0
```

이 YAML을 만든 뒤 시뮬 SDF와 rule-based 노드가 같은 값을 읽도록 만들면, 이후 BC/Offline RL 데이터셋도 같은 기준으로 정리할 수 있다.
