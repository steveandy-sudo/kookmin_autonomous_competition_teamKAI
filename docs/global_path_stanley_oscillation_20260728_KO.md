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

## 사용자 제작 트랙의 논문 기반 전역경로 개선

2026-07-28에는 `worlds/kookmin_xycar_track_final.sdf`와 사용자가 RViz에서
지정한 폐곡선 `routes/kookmin_custom_user_waypoints.yaml`을 대상으로 추가
개선했다.

연구 기준은 다음과 같다.

- Stanford Stanley: front-axle CTE, 속도 의존 비선형 피드백, 조향 지연과
  yaw damping을 함께 사용한다.
  <https://robotics.stanford.edu/~dstavens/jfr06/thrun_etal_jfr06.pdf>
- TUM minimum-curvature trajectory: 트랙 경계 안에서 곡률이 작은 경로를
  만들고, 경로 전체에 대해 횡가속도 제한과 전진/후진 가감속 패스로 속도
  프로파일을 계산한다.
  <https://doi.org/10.1080/00423114.2019.1631455>
- Stanford feedback-feedforward steering: 경로 곡률 feedforward와 추종
  feedback을 분리해 고속 정상상태 오차와 안정성을 함께 다룬다.
  <https://ddl.stanford.edu/publications/journal/design-feedback-feedforward-steering-controller-accurate-path-tracking-and>
- ETH MPCC: 횡오차만 줄이는 대신 트랙 제약 안에서 진행률을 최대화한다.
  온라인 최적화가 필요한 다음 단계의 기준이며 현재 경량 제어기에 바로
  넣지는 않았다.
  <https://old.control.ee.ethz.ch/publications/2014/4623.html>

현재 코드에는 다음을 반영했다.

1. 전역경로의 RMS 곡률 프로파일을 계산한다. S자 변곡점에서 좌·우 곡률의
   부호가 평균으로 상쇄되어 감속이 사라지는 문제를 막는다.
2. 각 점의 횡가속도 제한으로 최고속도를 구하고 전진 패스에서 가속 한계,
   후진 패스에서 제동 한계를 적용한다. 폐곡선 시작/끝에도 제약이 전파될
   때까지 반복한다.
3. 실측 구동 지연을 고려해 `현재 속도 x braking_preview_sec`만큼 전방의
   속도 프로파일 최솟값을 미리 사용한다. 이 보정 전에는 직선에서
   `2.47 m/s`까지 오른 뒤 급곡선 진입 시 `2.28 m/s`가 남아 이탈했다.
4. 원래 전역경로에서 지정 거리 이상 벗어나지 않는 제한형 스무딩을
   적용한다. 현재 최선은 최대 허용 `0.15 m`, 실제 최대 이동 약
   `0.087 m`다.
5. 병렬 시험은 인스턴스마다 `ROS_DOMAIN_ID`, `GZ_PARTITION`,
   `IGN_PARTITION`을 분리한다. 동일한 월드·노드·토픽 이름을 사용해도
   결과가 섞이지 않는다.

### 병렬 탐색 결과

| 프로파일 | 완주 | 한 바퀴 | 평균 speed cmd | CTE 95% | 큰 직선 반전 |
|---|---:|---:|---:|---:|---:|
| 기존 안정 기준 `cruise=15` | 1 | `32.50 s` | `9.99` | `0.224 m` | 0 |
| 단순 `cruise=21` | 1 | `30.55 s` | `11.16` | `0.326 m` | 2 |
| 전진/후진 계획, 지연 보정 전 고속 | 0 | - | `11.27` | `0.352 m` | 0 |
| 전진/후진 + 제동 미리보기 | 1 | `32.98 s` | `10.50` | `0.187 m` | 0 |
| 최종 제한형 경로 + 속도 계획 | 1 | `29.00 s` | `10.86` | `0.192 m` | 0 |

최종 프로파일은 기존 안정 기준보다 약 10.8% 빠르고 큰 직선 오실레이션은
발생하지 않았다. 평균 speed command `17`은 아직 달성하지 못했다. 이
트랙은 반경 약 `0.58 m` 수준의 연속 급곡선이 있고 조향 command가
`42`에서 포화되므로, 최고속도만 높여 평균을 맞춘 후보는 모두 이탈했다.

### 최종 프로파일 실행

```bash
cd ~/slam
source /opt/ros/humble/setup.bash
source xycar_ws/install/setup.bash

ros2 launch xycar_map_nav sim_custom_track_waypoint_nav.launch.py \
  project_root:=$PWD \
  headless:=false \
  enable_rviz:=true \
  drive_enabled:=true \
  drive_start_delay_sec:=3.0 \
  reposition_vehicle:=true \
  start_x:=-2.725 \
  start_y:=2.4456 \
  start_yaw_deg:=-173.257623 \
  speed_planner_mode:=forward_backward \
  cruise_speed_command:=40.0 \
  minimum_speed_command:=5.0 \
  maximum_lateral_accel_mps2:=0.35 \
  speed_profile_max_accel_mps2:=1.2 \
  speed_profile_max_decel_mps2:=1.8 \
  speed_profile_braking_preview_sec:=0.30 \
  speed_alignment_cross_track_hard_m:=0.60 \
  speed_alignment_heading_hard_rad:=1.20 \
  path_smoothing_data_weight:=0.008 \
  path_smoothing_weight:=0.40 \
  path_smoothing_iterations:=1000 \
  path_smoothing_anchor_weight:=0.15 \
  path_smoothing_maximum_deviation_m:=0.15 \
  curvature_feedforward_gain:=0.50
```

### 병렬 재탐색

```bash
cd ~/slam
source /opt/ros/humble/setup.bash
source xycar_ws/install/setup.bash

python3 scripts/parallel_waypoint_profile_search.py \
  --project-root "$PWD" \
  --workers 4 \
  --domain-base 140 \
  --timeout-sec 85 \
  --profiles-json scripts/profile_sets/global_path_paper_stage6.json
```

결과는 실행 시각별
`analysis/parallel_profile_search_YYYYMMDD_HHMMSS/summary.json`에 저장된다.
현재 PC의 20 CPU thread와 30 GiB RAM에서는 headless Gazebo 4개가 안정적인
출발점이었다.

다음 성능 단계는 흰선·노란선 양쪽 경계를 경로 계획기에 제공해 단순
`0.15 m` 원형 제한 대신 실제 차도 경계를 제약으로 쓰는 minimum-curvature
QP를 만드는 것이다. 그 뒤에도 속도가 부족하면 MPCC로 진행률, contour
error, 조향 변화율을 동시에 최적화한다.
