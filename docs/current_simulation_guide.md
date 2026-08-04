# 국민대 Xycar 시뮬레이션 전체 안내서

이 문서는 이 저장소를 처음 받은 사람이 현재까지 만든 국민대학교 대회 트랙
시뮬레이션을 이해하고, 빌드하고, 룰베이스 주행까지 실행할 수 있도록 정리한
현재 상태 기준 안내서다.

기준일: 2026-07-13

## 1. 프로젝트 목표

전체 개발 순서는 다음과 같다.

1. 국민대학교 대회 트랙과 비슷한 Gazebo 맵 제작
2. 실차와 비슷한 차량 크기, 센서, 명령 응답 구현
3. Gazebo 카메라만 이용한 차선 인지 구현
4. 노란 중앙선과 흰 경계선 사이를 따라가는 룰베이스 주행 완성
5. 주행 데이터 수집
6. Behavioral Cloning(BC) 모방학습
7. Offline RL 및 안전 supervisor 구조로 확장

현재는 1~4단계가 구현된 상태다. 다음 중심 작업은 실차 저속 검증과 데이터
수집 파이프라인 구축이다.

## 2. 초보자를 위한 용어

- Gazebo Sim: 차량, 도로, 센서를 물리적으로 흉내 내는 시뮬레이터다.
- ROS2 노드: 카메라 처리, 차량 제어처럼 한 가지 역할을 실행하는 프로그램이다.
- 토픽: 노드 사이에서 데이터를 주고받는 이름 있는 통신 채널이다.
- RViz: 카메라, LiDAR, 차선, 목표 경로를 시각화하는 ROS 도구다.
- BEV: 카메라 영상을 위에서 내려다본 도로처럼 펼친 Bird's Eye View 영상이다.
- 룩어헤드: 차량이 현재 위치보다 앞쪽에서 조향 목표로 선택하는 거리다.
- Pure Pursuit: 앞쪽 목표점을 향하도록 필요한 경로 곡률을 계산하는 제어법이다.
- Shadow mode: 제어 명령을 계산하되 실차 모터에는 보내지 않는 검증 모드다.

## 3. 전체 데이터 흐름

```text
Gazebo 최종 월드
  -> 가상 카메라 /image_raw, /camera_info
  -> 가상 LiDAR /scan

/image_raw
  -> xycar_camera_perception
  -> 광각 보정 + BEV 변환 + HSV 차선 검출
  -> /perception/road_segments
  -> /perception/centerline
  -> /perception/debug_image, /perception/debug_markers

/perception/road_segments
  -> xycar_lane_rule_driver
  -> 노란선과 오른쪽 흰선으로 목표 경로 생성
  -> Pure Pursuit + 실측 조향 맵으로 angle 계산
  -> 곡선 정도에 따라 speed 계산
  -> /xycar_motor = Float32MultiArray [angle, speed]

/xycar_motor
  -> xycar_motor_bridge
  -> 실차 측정 지연, 속도, 좌우 비대칭 조향 반영
  -> /model/xycar_ackermann/cmd_vel
  -> Gazebo Ackermann 차량 이동
```

중요한 점은 인지 노드가 Gazebo 내부의 정답 차선이나 mock 데이터를 읽지
않는다는 것이다. 실제 차와 마찬가지로 `/image_raw`의 픽셀만 보고 차선을
찾는다.

## 4. 트랙 맵

최종 기준 파일은 `worlds/kookmin_xycar_track_final.sdf`다.

- CAD/DXF의 차선 형상을 국민대 설계도 외곽 `20.150 x 11.350 m`에 맞춰 사용했다.
- 바닥은 실차 도로와 비슷한 회색이다.
- 흰 경계선과 노란 중앙선 두께는 모두 `0.024 m`다.
- 노란 점선은 길이 `0.30 m`, 점선 사이 간격 `0.30 m` 기준이다.
- 양쪽 흰선 사이 전체 도로 폭은 약 `0.80 m`이며, 실제 최종본은 위치에 따라
  대략 `0.76~0.84 m`다.
- 노란 중앙선부터 한쪽 흰 경계선까지 한 차로 폭은 약 `0.40 m`다.
- 흰선 31개와 노란 점선 44개가 개별 Gazebo 객체다.
- 벽, 몰딩, 락커, 책상, 의자, 조명, 콘도 `room_*` 독립 객체다.
- 최종 월드에는 총 109개 모델이 있으며, Entity Tree에서 선택하고 이동할 수 있다.
- 차량 초기 위치는 상단 직선의 `x=-2.70`, `y=2.25`, `yaw=0`이다.

`scripts/generate_kookmin_track.py`를 다시 실행하면 Gazebo에서 직접 조정해 저장한
위치를 덮을 수 있다. 최종 맵을 수정하기 전에 반드시 `final.sdf`를 복사해 둔다.

Gazebo에서 수정한 월드는 `Save World As`를 사용해 먼저 작업본으로 저장하고,
검증이 끝났을 때만 `kookmin_xycar_track_final.sdf`를 갱신한다.

## 5. 차량과 센서 모델

차량 모델 이름은 `xycar_ackermann`이다.

| 항목 | 현재 값 |
|---|---:|
| 실차 프레임 크기 | 약 `0.55 x 0.30 x 0.25 m` |
| Gazebo 충돌 차체 | `0.55 x 0.30 x 0.12 m` |
| 바퀴 포함 외폭 | 약 `0.30 m` |
| wheelbase | `0.32 m` |
| wheel separation | `0.265 m` |
| wheel radius | `0.06 m` |
| 조향 중심 제한 | `0.560 rad` |
| 조향 joint 제한 | `0.700 rad` |

센서 원점은 실차 측정과 동일하게 앞바퀴 중심을 기준으로 한다.

| 센서 | 앞바퀴 중심 기준 위치 | 시뮬 출력 |
|---|---|---|
| 카메라 | `(-0.04, 0.00, 0.17) m` | `/image_raw`, `/camera_info` |
| LiDAR | `(0.065, 0.00, 0.080) m` | `/scan` |

카메라는 `1280x1024`, `30 Hz`를 유지한다. 현재 MobileNetV3 인지 경로에는
실차의 왜곡 보정 영상과 맞춘 pinhole 투영, HFOV `97.59 deg`, pitch
`0.187 rad`를 사용한다. 센서 원점은 바꾸지 않았다. LiDAR는 `505 samples`,
`10 Hz`, `-180~180 deg`, `0.1~12 m`를 기준으로 한다. 카메라 아래에는 실제
영상처럼 앞쪽으로 튀어나온 둥근 LiDAR만 보이도록 구성했다.

## 6. 실차 동역학 반영

실차 모터 인터페이스는 다음 형식으로 통일했다.

```text
topic: /xycar_motor
type: std_msgs/msg/Float32MultiArray
data[0]: angle command
data[1]: speed command
```

2026-07-12 및 2026-07-13 실차 주행 데이터에서 얻은 값을 Gazebo bridge에 적용했다.

- 속도 변환: `speed_mps = 0.080612 * speed_command`
- 출발 데드존: `abs(speed_command) < 3`이면 정지
- 조향 응답: `angle=+-10, +-20, +-30, +-35, +-40, +-42` 실측 곡률 테이블
- 같은 절댓값에서도 좌우 반경이 달라 좌우 비대칭 테이블 사용
- 조향 지연: `0.10 s`
- 조향 응답: Ackermann `steer_p_gain=20`, 관절 한계 `20 rad/s`, effort `80`
- 속도 지연: `0.20 s`
- 속도 1차 응답: 가속 `tau=0.19 s`, 감속 `tau=0.09 s`
- Ackermann 플러그인의 공통 가속도 제한은 yaw까지 늦추므로 사용하지 않음
- 명령이 `0.5 s` 동안 없으면 Gazebo bridge가 정지 명령을 보냄
- 양수 angle command는 기존 Xycar 규약상 우회전

전원 저하로 움직이지 않은 일부 `speed=8` 및 제동 반복은 모델 계산에서 제외했다.
속도는 VESC ERPM 기반 odometry이므로 독립 거리 센서로 다시 검증할 필요가 있다.

근거 데이터는 `data/vehicle_dynamics/2026-07-12/`와
`teamkai/data/vehicle-dynamics-20260713` 브랜치의 `data/vehicle_dynamics/2026-07-13/`,
시험 방법은 `docs/real_vehicle_dynamics_tests.md`에 있다.

## 7. 카메라 차선 인지 원리

인지 설정은 `xycar_perception/config/camera_perception.yaml`에 있다.

1. `/image_raw`의 `1280x1024` 광각 영상을 받는다.
2. Gazebo 입력은 이미 설정된 카메라 투영이므로 실차 K/D를 다시 적용하지 않는다.
3. 도로가 있는 사다리꼴 영역을 homography로 `640x220` BEV로 펼친다.
4. 원본 카메라 픽셀을 함께 warp한 유효 마스크로 보이지 않는 근거리와 모서리를
   검출에서 제외한다.
5. HSV 색 공간에서 흰색과 노란색 마스크를 각각 만든다.
6. morphology open/close로 작은 점 노이즈를 없애고 끊어진 픽셀을 연결한다.
7. 아래에서 위로 `6 px` 간격으로 가로줄을 검사한다.
8. 같은 가로줄의 연속 픽셀을 하나의 차선 덩어리로 묶고 평균 x를 구한다.
9. 화면 중심 왼쪽/오른쪽의 가까운 흰색 덩어리와 중심에 가까운 노란색
   덩어리를 선택한다.
10. 픽셀 좌표를 차량 좌표계 meter로 변환한다.

BEV meter 변환 기준은 다음과 같다.

- 좌우: `0.0021875 m/px`
- 전후: `0.006818182 m/px`
- canonical 범위: `256x144`, 좌우 `1.4 m`, 전방 `1.5 m`
- 좌표계: `x`는 차량 앞쪽, `y`는 차량 왼쪽, 원점 frame은 `base_footprint`

canonical 모델 입력은 현재 프레임 관측을 우선 사용한다. 색이 바랜 노란 점선을
현재 프레임의 좌우 경계 기하로 재분류하고 반사 후보를 제거한다. 시뮬레이터의
S자 진입에서 발생하는 짧은 검출 누락은 마지막으로 확인된 선을 0.25초 동안
coast하고, 최대 0.60초의 search 구간까지 표시해 입력이 한두 프레임 전체
배경으로 바뀌지 않게 한다. 이 시간이 지나도 새 선을 찾지 못하면 빈 관측으로
전환한다.

인지 노드는 흰 실선은 `TYPE_WHSOL`, 노란 점선은 `TYPE_YEDOT`으로
`/perception/road_segments`에 발행한다. 인지용 `/perception/centerline`은
노란선과 선택된 흰선의 중간점을 2차 곡선으로 맞추고 `0.05 m` 간격으로
재표본화한다. 프레임 사이 흔들림은 temporal alpha `0.45`로 완화한다.

현재 객체와 신호등 검출기는 아직 구현하지 않았으므로 `/perception/objects`와
`/perception/traffic_lights`는 빈 배열을 발행한다.

## 8. 디버그 영상과 RViz 읽는 법

RViz 설정은 `xycar_gazebo_bridge/rviz/xycar_gazebo_sensors.rviz`다.

- Camera: Gazebo 원본 `/image_raw`
- LaserScan: 원본 `/scan`
- Perception Debug Image: BEV로 펼친 `/perception/debug_image`
- Perception Markers: 카메라가 인식한 차선과 인지 중앙 경로
- Rule Drive Markers: 실제 제어기가 사용할 목표 경로와 룩어헤드점

`/perception/debug_image`에서 흰 점은 검출한 흰선, 노란 점은 검출한 노란선,
빨간 점과 글자는 인지 노드가 만든 중앙 경로다. 수직 guide는 차량 기준
`y=+0.40`, `0.00`, `-0.40 m`를 나타낸다.

RViz 3D marker 색은 다음 의미다.

- 흰색 선: 인식된 흰 도로 경계
- 노란색 선: 인식된 노란 중앙선
- 빨간색 선: 인지 노드의 중앙 경로
- 초록색 선: 룰베이스 제어기가 실제로 따라갈 목표 경로
- 주황색 선: 카메라 인지가 끊겨 마지막 경로를 예측 중인 상태
- 파란 구: 현재 선택된 룩어헤드 목표점
- 주황 구: Pure Pursuit 계산에 쓰는 차량 제어 기준점

원본 영상과 BEV 영상은 서로 다르게 보여야 정상이다. BEV는 차선을 위에서 본
것처럼 펴기 위한 제어용 영상이다.

## 9. 룰베이스 경로 생성과 제어

제어 설정은 `xycar_rule_drive/config/lane_rule_driver.yaml`에 있다.

1. `/perception/road_segments`에서 노란선과 주행 방향 오른쪽 흰선을 고른다.
2. 두 선이 동시에 보이면 같은 전방 x 위치의 두 y 좌표 사이를 계산한다.
3. 흰선 쪽으로 너무 파고드는 현상을 줄이기 위해 중앙선 방향으로 편향한다.
4. 흰선이 잠시 안 보이면 노란선에서 일정 거리 떨어진 평행 경로를 만든다.
5. 경로에서 전방 `0.50 m`에 가장 가까운 점을 룩어헤드 목표로 선택한다.
6. Pure Pursuit로 필요한 곡률을 계산한다.
7. 실차에서 측정한 command-to-curvature 표를 역으로 사용해 angle command를 구한다.
8. 조향이 커질수록 속도를 `4`에서 `2`까지 선형으로 낮춘다.
9. `[angle, speed]`를 `/xycar_motor`와 `/xycar_motor_shadow`에 발행한다.

현재 시뮬 주행 핵심값은 다음과 같다.

| 항목 | 값 | 의미 |
|---|---:|---|
| `target_lane` | `right` | 노란선과 오른쪽 흰선 사이 주행 |
| `path_bias_toward_yellow_m` | `0.10 m` | 계산된 중간 경로를 중앙선 쪽으로 총 10 cm 이동 |
| `lookahead_distance_m` | `0.50 m` | 50 cm 앞 목표점을 보고 조향 |
| `control_point_x_m` | `-0.08 m` | 차체 중심보다 8 cm 뒤 기준으로 곡률 계산 |
| `speed_command` | `4.0` | 직선 기본 속도 명령 |
| `min_speed_command` | `2.0` | 큰 곡선에서의 최저 속도 명령 |
| `slow_down_angle_cmd` | `18.0` | 이 조향량부터 감속 시작 |
| `max_abs_angle_for_drive` | `43.0` | 이 이상이면 안전상 정지 |
| `perception_timeout_sec` | `0.40 s` | 이 시간 동안 새 인지가 없으면 예측 시작 |
| `hold_last_path_sec` | `5.0 s` | 마지막 경로를 차량 운동으로 갱신하며 유지 |
| `prediction_speed_command` | `2.0` | 예측 주행 중 최대 속도 |

폭이 정확히 `0.40 m`인 한쪽 차로에서는 원래 중간 경로가 노란선에서 `0.20 m`
떨어진 곳이다. 현재 시뮬 편향 `0.10 m`를 적용하면 목표는 노란선에서 약
`0.10 m`, 흰선에서 약 `0.30 m`인 위치가 된다. 이 값은 현재 시뮬의 안쪽
파고듦을 보상하기 위해 조정한 값이며 실차 프로필에는 그대로 복사하지 않는다.

인지가 끊기면 마지막 조향각만 고정하는 것이 아니라 마지막 목표 경로를 이전
속도와 곡률로 차량 좌표계에서 계속 이동/회전시킨다. 5초 예측 시간이 끝나거나
처음부터 유효한 경로가 없으면 `[0, 0]`을 발행해 정지한다.

## 10. 주요 토픽

| 토픽 | 타입 | 생산자 | 소비자/용도 |
|---|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | Gazebo camera | 인지, RViz |
| `/camera_info` | `sensor_msgs/CameraInfo` | Gazebo camera | 카메라 정보 |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo LiDAR | RViz, 향후 장애물 인지 |
| `/perception/road_segments` | `RoadSegmentArray` | 인지 | 룰베이스 경로 생성 |
| `/perception/centerline` | `Centerline` | 인지 | 인지 중앙 경로 확인 |
| `/perception/debug_image` | `sensor_msgs/Image` | 인지 | BEV 검출 디버그 |
| `/perception/debug_markers` | `MarkerArray` | 인지 | RViz 차선 표시 |
| `/rule_drive/target_path` | `Centerline` | 룰베이스 | 실제 제어 목표 경로 |
| `/rule_drive/debug_markers` | `MarkerArray` | 룰베이스 | RViz 제어 상태 |
| `/xycar_motor_shadow` | `Float32MultiArray` | 룰베이스 | 계산 명령 상시 확인 |
| `/xycar_motor` | `Float32MultiArray` | 룰베이스/키보드 | 실제 제어 명령 |
| `/xycar_motor_bridge/debug` | `Float32MultiArray` | bridge | 변환 결과 확인 |

## 11. 처음 실행하는 방법

모든 빌드와 source는 저장소 루트 기준으로 통일한다.

### 최초 1회 또는 코드 변경 후 빌드

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
colcon build --packages-up-to \
  kaiev26_msgs xycar_perception xycar_gazebo_bridge xycar_rule_drive \
  --symlink-install
source install/setup.bash
```

`kaiev26_msgs package.sh가 없다`는 오류는 의존 메시지 패키지를 먼저 빌드하지
않았을 때 발생한다. 위 `--packages-up-to` 명령을 사용하면 의존 순서대로 빌드된다.

### 터미널 1: Gazebo 최종 월드

```bash
cd ~/xycar_kookmin_gazebo_track
export GZ_SIM_RESOURCE_PATH=$PWD:${GZ_SIM_RESOURCE_PATH}
gz sim -r worlds/kookmin_xycar_track_final.sdf
```

### 터미널 2: bridge + 인지 + RViz

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xycar_gazebo_bridge xycar_gazebo_rviz.launch.py
```

### 터미널 3: 룰베이스 자동주행

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xycar_rule_drive lane_rule_driver.launch.py
```

자동주행 노드가 둘 이상 실행되면 같은 이름의 노드와 `/xycar_motor` publisher가
중복되어 차량이 이상하게 움직인다. `ros2 node list`에서
`/xycar_lane_rule_driver`가 하나인지 확인한다.

### 수동 키보드 주행

자동주행 노드를 먼저 끈 뒤 실행한다.

```bash
ros2 run xycar_rule_drive keyboard_teleop
```

`w/s` 속도, `a/d` 조향, `e` 조향 중앙, `x` 속도 0, `space` 완전 정지,
`q` 종료다. 자동주행과 키보드 주행을 동시에 실행하면 안 된다.

## 12. 상태 확인 명령

```bash
ros2 node list
ros2 topic list -t
ros2 topic hz /image_raw
ros2 topic hz /scan
ros2 topic hz /perception/road_segments
ros2 topic echo /xycar_motor --once
ros2 topic echo /rule_drive/target_path --once
ros2 param get /xycar_lane_rule_driver lookahead_distance_m
ros2 param get /xycar_lane_rule_driver speed_command
ros2 param get /xycar_lane_rule_driver path_bias_toward_yellow_m
```

현재 정상 실행값은 각각 `0.5`, `4.0`, `0.1`이다.

## 13. 종료 명령

가장 좋은 종료 방법은 각 터미널에서 `Ctrl+C`를 한 번 누르는 것이다. 남은
백그라운드 프로세스를 정리해야 할 때만 다음 명령을 사용한다.

```bash
pkill -TERM -f "lane_rule_driver" || true
pkill -TERM -f "xycar_gazebo_rviz.launch.py" || true
pkill -TERM -f "ros_gz_bridge" || true
pkill -TERM -f "gz sim" || true
```

강제 종료인 `pkill -KILL`은 정상 종료가 되지 않을 때 마지막 수단으로 쓴다.
여러 번 `Ctrl+C`를 연속 입력하면 Gazebo Ruby launcher에서 `Errno::ESRCH`가
보일 수 있다. 이는 이미 끝난 프로세스를 다시 죽이려 할 때 생기는 메시지다.

## 14. 자주 생기는 문제

### Gazebo 창이 안 뜨고 EGL 경고만 보임

`libEGL warning: failed to create dri2 screen`만으로 실패가 확정되지는 않는다.
남아 있는 Gazebo 프로세스를 종료하고 다시 실행한다.

```bash
pkill -TERM -f "gz sim" || true
export GZ_SIM_RESOURCE_PATH=$PWD:${GZ_SIM_RESOURCE_PATH}
gz sim -r worlds/kookmin_xycar_track_final.sdf
```

### LiDAR는 보이는데 카메라가 안 보임

```bash
ros2 topic info /image_raw -v
ros2 topic hz /image_raw
ros2 topic echo /camera_info --once
```

RViz Camera와 Image display가 `/image_raw`를 구독하는지 확인한다.

### 룰베이스를 실행했는데 차량이 안 감

```bash
ros2 topic hz /perception/road_segments
ros2 topic echo /xycar_motor --once
ros2 topic info /xycar_motor -v
ros2 node list | sort
```

유효 경로가 없거나, 인지가 오래됐거나, 요구 조향량이 `43` 이상이면 정지한다.
주행 노드 중복과 다른 키보드/자동주행 publisher도 확인한다.

### 차가 흰선 안쪽으로 파고듦

한 번에 여러 값을 바꾸지 말고 다음 순서로 본다.

1. `/perception/debug_image`의 검출점이 실제 선 위에 있는지 확인
2. 빨간 인지 경로가 노란선과 흰선 사이인지 확인
3. 초록 제어 경로와 파란 룩어헤드점 확인
4. `path_bias_toward_yellow_m`를 작은 단위로 조정
5. 그래도 이르면 `lookahead_distance_m`와 동역학 조향 맵 확인

현재 편향 `0.10 m`는 큰 보정값이다. 더 키우기 전에 바퀴가 노란 중앙선을
침범하지 않는지 반드시 위에서 확인한다.

### 설정 파일을 바꿨는데 값이 그대로임

이 노드는 시작할 때 YAML을 읽으므로 패키지를 빌드하고 주행 노드를 재시작한다.

```bash
colcon build --packages-select xycar_rule_drive --symlink-install
source install/setup.bash
```

## 15. 실차로 옮길 때

실차에는 다음 세 패키지만 먼저 복사하면 된다.

```text
kaiev26_msgs
xycar_perception
xycar_rule_drive
```

Gazebo bridge와 Gazebo 월드는 실차에서 실행하지 않는다. 실차 통합 launch는
기본적으로 motor 출력을 차단한 shadow mode다.

```bash
export ROS_DOMAIN_ID=7
ros2 launch xycar_rule_drive real_lane_drive.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  use_compressed_image:=false \
  drive_enabled:=false
```

확인할 항목:

1. `/perception/debug_image`의 흰/노란 검출이 실제 페인트 위에 있는가
2. 목표 경로가 노란선과 흰선 사이에 있는가
3. 직선에서 조향 명령이 0 근처인가
4. 좌우 조향 부호가 실차와 일치하는가
5. 카메라를 가리면 마지막 경로 유지 시간 뒤 shadow 속도가 0이 되는가
6. 다른 노드가 `/xycar_motor`를 발행하고 있지 않은가

실차 프로필은 시뮬 프로필과 분리돼 있다.

| 항목 | 시뮬 | 실차 기본값 |
|---|---:|---:|
| motor 출력 | 켜짐 | 꺼짐(shadow) |
| 기본 속도 | `4.0` | `3.0` |
| 곡선 최저 속도 | `3.0` | `3.0` |
| 룩어헤드 | `0.50 m` | `0.40 m` |
| 중앙선 방향 편향 | `0.10 m` | `0.02 m` |
| 인지 소실 예측 | `5.0 s` | `1.0 s` |

실차에서 실제 모터 출력을 켜는 절차와 안전 확인은
`docs/real_vehicle_deployment.md`를 따른다. IMU는 현재 차선 주행에 필수 입력이
아니므로 고장 상태에서도 카메라 기반 첫 검증은 가능하다.

## 16. 검증 상태

2026-07-13 기준:

- `gz sdf -k worlds/kookmin_xycar_track_final.sdf`: Valid
- `xmllint --noout`: 통과
- motor bridge 수학 테스트: 통과
- 카메라 중앙 경로 fitting 테스트: 통과
- raw/compressed 이미지 처리 테스트: 통과
- 룰베이스 경로/조향/예측 테스트: 통과
- 관련 단위 테스트 총 19개 통과
- `git diff --check`: 통과

이 테스트는 코드와 수학 계산의 일관성을 확인한다. 실차의 타이어 마찰,
배터리 전압, 바닥 상태, 카메라 노출까지 보장하지는 않는다.

## 17. 핵심 파일

| 역할 | 파일 |
|---|---|
| 최종 월드 | `worlds/kookmin_xycar_track_final.sdf` |
| 맵 생성기 | `scripts/generate_kookmin_track.py` |
| Gazebo-RViz 통합 launch | `xycar_gazebo_bridge/launch/xycar_gazebo_rviz.launch.py` |
| 모터 bridge | `xycar_gazebo_bridge/xycar_gazebo_bridge/xycar_motor_bridge.py` |
| bridge 동역학 설정 | `xycar_gazebo_bridge/config/xycar_gazebo_bridge.yaml` |
| 카메라 인지 | `xycar_perception/xycar_perception/camera_perception_node.py` |
| 시뮬 인지 설정 | `xycar_perception/config/camera_perception.yaml` |
| 실차 인지 설정 | `xycar_perception/config/camera_perception_real.yaml` |
| 룰베이스 제어 | `xycar_rule_drive/xycar_rule_drive/lane_rule_driver.py` |
| 시뮬 제어 설정 | `xycar_rule_drive/config/lane_rule_driver.yaml` |
| 실차 제어 설정 | `xycar_rule_drive/config/lane_rule_driver_real.yaml` |
| 실차 통합 launch | `xycar_rule_drive/launch/real_lane_drive.launch.py` |
| RViz 설정 | `xycar_gazebo_bridge/rviz/xycar_gazebo_sensors.rviz` |

실제 경로는 모두 `xycar_ws/src/` 아래에서 시작한다. 표에서는 읽기 쉽도록
그 공통 접두어를 생략했다.

## 18. 다음 개발 순서

1. 현재 룰베이스를 여러 바퀴 반복해 이탈·정지 구간과 명령 로그 기록
2. 실차에서 shadow mode로 카메라 BEV와 조향 부호 검증
3. 실차 `+-35, +-40, +-42` 조향 반경과 저속 deadband 추가 측정
4. 실차 속도/조향 step 응답으로 시뮬 지연과 가감속 재보정
5. 이미지, 인지 경로, angle/speed, 차량 상태를 동기화한 rosbag 수집
6. 룰베이스 명령을 teacher label로 사용해 BC 데이터셋 생성
7. train/validation 구간을 분리해 BC 학습 및 closed-loop 평가
8. 실패 구간 수집, DAgger 성격의 데이터 보강
9. Offline RL은 BC baseline과 안전 supervisor가 안정된 뒤 진행

현재 룰베이스 완성은 끝이 아니라 데이터 수집을 위한 안정적인 teacher가 생긴
단계다. 다음 품질 기준은 한 번 주행 성공이 아니라 여러 바퀴에서 재현되는지,
인지 소실과 조명 변화에서도 안전하게 정지하거나 복구하는지다.
