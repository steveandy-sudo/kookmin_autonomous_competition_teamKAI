# SLAM 전역경로 고속 오실레이션 억제

## 문제와 원인

2026-07-26 실차 전역경로 시험은 speed command `3`에서 안정적이었지만,
속도를 높이면 직선에서 큰 좌우 조향이 반복됐다. 기존 제어는 A*의 각진
polyline을 고정 `0.65 m` lookahead Pure Pursuit으로 추종했다. 다음 요소가
겹치면 속도가 높을수록 보정 시점이 늦고 반대쪽으로 다시 과보정된다.

- A* 격자 경로의 불연속 heading과 과도한 국부 곡률
- 실차 조향 지연 약 `0.10 s`
- 속도 응답 지연 약 `0.20 s`
- SLAM pose와 yaw의 작은 프레임 간 흔들림
- 곡선 출구에서 차량 heading이 남아 있는데 즉시 직선 최고속도로 가속
- 좌우 조향 command 대 곡률의 비대칭과 약 `42` 부근 포화

## 적용한 제어 구조

### 1. 주행 가능한 경로

A*에는 장애물 거리 비용을 넣어 벽에서 여유를 둔다. 계획 후 경로 평활화는
모든 이동마다 occupancy map 충돌을 검사하므로 선이 벽이나 inflation
영역을 가로지르지 않는다. Gazebo 회귀시험은 SLAM 맵에서 추출한
`one_lap_path.csv`를 `0.10 m` 간격으로 재표본화하고 보수적으로 평활화했다.
최종 시험 경로 최대 곡률은 `1.458 1/m`로, 실측 좌조향 한계
`1.502 1/m` 안에 있다.

### 2. 속도 적응형 Stanley

제어점은 차량 중심이 아니라 front axle이며 다음처럼 조향 지연만큼 전방으로
예측한다.

```text
control_offset = front_axle_offset + measured_speed * steering_delay
```

조향은 경로 곡률 feedforward, heading 오차, cross-track 오차를 합친다.
cross-track 항은 속도가 높을수록 부드러워진다.

```text
steer =
  atan(wheelbase * feedforward_gain * path_curvature)
  + heading_gain * heading_error
  - atan2(stanley_gain * cross_track_error, speed + softening)
```

직선은 작은 gain과 강한 필터를 사용하고, 곡선은 빠른 gain과 높은 조향
변화율을 사용한다. 실제 odometry의 yaw rate가 경로가 요구하는 yaw rate보다
크면 조향을 미리 줄여 곡선 출구의 잔여 회전을 감쇠한다.

### 3. 조향과 속도 전환

조향 command에는 저역통과 필터와 초당 변화량 제한을 동시에 적용한다.
직선에서 작은 localization 변화가 바로 큰 반대 조향으로 바뀌는 것을 막되,
곡선에서는 별도 빠른 제한으로 진입 시점을 놓치지 않는다.

속도는 다음 순서로 결정한다.

1. 경로 곡률이 크면 `minimum_speed_command` 쪽으로 감속
2. 최대 횡가속도 `0.90 m/s^2`를 넘지 않도록 속도 제한
3. CTE 또는 heading 오차가 크면 최고속도 재가속 보류
4. 감속은 command 기준 초당 `30`, 가속은 초당 `5`로 제한

따라서 곡률이 0에 가까워졌다는 이유만으로 speed `3 -> 10`이 한 프레임에
바뀌지 않는다. 차체가 경로와 정렬된 뒤에만 최고속도로 올라간다.

## Gazebo 회귀시험

Gazebo에는 2026-07-13 실차 측정값을 사용했다.

- wheelbase: `0.32 m`
- 속도 환산: `0.080612 m/s/command`
- 조향 지연: `0.10 s`
- 속도 지연: `0.20 s`
- 가속 시정수: `0.19 s`
- 제동 시정수: `0.09 s`
- 비대칭 command 대 곡률 표: `waypoint_nav_real.yaml`

결과:

| 시험 | 완주 | CTE 평균 | CTE 95% | CTE 최대 | 큰 직선 반전 |
|---|---:|---:|---:|---:|---:|
| 기존 고속 경로/제어 | 1 | `0.089 m` | `0.247 m` | `0.413 m` | 6 |
| 개선 speed 7 | 1 | `0.043 m` | `0.108 m` | `0.180 m` | 0 |
| 정렬 제한 전 speed 10 | 1 | `0.081 m` | `0.240 m` | `0.340 m` | 0 |
| 최종 speed 10 | 1 | `0.049 m` | `0.103 m` | `0.132 m` | 0 |

`큰 직선 반전`은 앞뒤 `0.5 s` 동안 경로 곡률이 계속 `0.16 1/m` 이하인
구간에서 조향 command가 `+12` 이상과 `-12` 이하 사이를 바꾼 횟수다.
작은 중심 유지 조향과 S형 경로의 정상 곡률 부호 전환은 오실레이션으로 세지
않는다.

## Gazebo에서 재현

```bash
cd ~/slam/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select xycar_map_nav --symlink-install
source install/setup.bash

ros2 launch xycar_map_nav sim_global_waypoint_nav.launch.py \
  headless:=false \
  enable_rviz:=true \
  cruise_speed_command:=10.0 \
  minimum_speed_command:=3.0
```

GUI가 필요 없으면 `headless:=true enable_rviz:=false`를 사용한다.
이 launch의 `emergency_stop_distance_m=0.05`는 SLAM 맵을 세운 Gazebo
가상 벽에 대한 회귀시험 전용이다. 실차 설정 `0.38 m`는 바꾸지 않았다.

bag 기록:

```bash
ros2 bag record \
  -o /tmp/xycar_global_nav_speed10 \
  /map_nav/debug /map_nav/control_mode /map_nav/global_path \
  /model/xycar_ackermann/odometry /xycar_motor \
  /xycar_motor_bridge/debug /scan
```

분석:

```bash
python3 \
  install/xycar_map_nav/share/xycar_map_nav/scripts/analyze_waypoint_oscillation.py \
  /tmp/xycar_global_nav_speed10
```

## 실차 적용 순서

실차에서는 저장소를 갱신하고 package를 빌드한 뒤 기존 localization과
waypoint launch를 사용한다. 첫 시험은 반드시 장애물 없는 구간에서 speed
command `3`으로 한다.

```bash
cd ~/kookmin_ty/slam_gazebo_controller
git pull --ff-only

cd xycar_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select xycar_map_nav --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=7

ros2 launch xycar_map_nav real_waypoint_nav.launch.py \
  drive_enabled:=true \
  cruise_speed_command:=3.0 \
  minimum_speed_command:=3.0 \
  map_yaml:=$HOME/xycar_maps/new_site_02/map.yaml \
  waypoints_yaml:=$HOME/xycar_maps/new_site_02/waypoints.yaml
```

통과 순서는 `3 -> 4 -> 5 -> 7 -> 10`이다. 각 단계에서 최소 5바퀴를
기록하고 다음 조건을 모두 통과한 뒤에만 올린다.

- `map -> base_footprint` 순간이동 없음
- CTE 최대 `0.20 m` 미만
- 직선 큰 조향 반전이 연속해서 증가하지 않음
- 곡선 진입과 출구에서 차선 또는 안전 통로 침범 없음
- `/slam/odom`, `/scan`, `/vehicle/vesc_state` stale 없음
- LiDAR `0.38 m` emergency stop 정상

CTE가 `0.20 m`를 넘거나 큰 좌우 반전 진폭이 커지면 물리 비상정지로 즉시
중단한다. Gazebo의 speed `10` 성공은 실차 speed `10` 자동 승인이 아니다.
