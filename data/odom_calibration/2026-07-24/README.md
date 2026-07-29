# ROS bag 기반 command odom 보정 결과 (2026-07-24)

## 결론

도면의 `49.2 m`와 CAD 치수는 보정에 사용하지 않았다. 도면이 실제
시공 상태와 다를 수 있고, 누적 SLAM 궤적 길이도 잡음 때문에 실제 거리의
정답이 아니기 때문이다.

이번 데이터만으로 production 값을 자동 교체하는 것은 보류한다.

| 근거 | `speed_gain` (m/s/command) | speed 명령 3의 속도 |
|---|---:|---:|
| 기존 VESC 실차 동역학 시험 | 0.080191 | 0.2406 m/s |
| 현재 production 설정 | 0.080612 | 0.2418 m/s |
| Cartographer 전역 scan-to-submap 정합 | **0.086683** | **0.2600 m/s** |
| 짧은 구간 LiDAR-LiDAR ICP | 0.098779 | 0.2963 m/s |

LiDAR-LiDAR ICP와 전역 정합값의 차이가 `13.95%`이므로 절대거리 정답이
없는 상태에서 큰 값을 바로 넣는 것은 위험하다. 현재 백으로 시험할 때의
보수적인 임시값은 전역 정합 결과인 `0.086683`이다. production
`command_odom_real.yaml`의 `0.080612`는 그대로 유지했고 별도의
`command_odom_rosbag_20260724_experimental.yaml`만 추가했다.

이 임시값은 기존값보다 `7.53%` 크다. 임시값이 실제값이라고 가정하면
기존 odom은 실제 10 m를 약 9.30 m로 짧게 계산한다.

## 입력 데이터와 확인된 한계

- 백: `track_full_sensor_20260724_163034`
- 기록 시간: 385.78초
- `/scan`: 3,724개
- `/imu`: 13,791개
- `/xycar_motor`: 31,198개
- `/odom`: 없음
- 속도 명령: `3`이 31,188개, `0`이 10개

따라서 이번 결과가 직접 식별한 것은 사실상 `speed=3` 한 점이다.
`speed=5`, `8` 등의 비선형성은 이 백으로 보정할 수 없다. 휠 엔코더,
RTK, 모션 캡처, 실측 직선거리 중 어느 것도 백에 포함되어 있지 않다.

## 계산 방법

1. LiDAR의 range 단위인 m를 로컬 거리 기준으로 사용했다.
2. 유리 반사의 영향을 줄이려고 `0.30~4.00 m` 유효점만 사용했다.
3. 5개 스캔 간격의 2D trimmed ICP를 여러 초기 거리에서 반복했다.
4. 초기값에 민감한 긴 평행벽 구간은 제외했다.
5. 통과한 188개 구간에서 속도 환산값과 IMU yaw를 회귀했다.
6. 별도로 Cartographer의 최적화된 node 시간 2,400개와 전역 궤적을
   연결해 scan-to-submap 결과를 교차검증했다.
7. 기존 VESC 실차 동역학 시험값과 세 결과를 비교했다.

상세 수치와 모든 구간의 승인/거절 사유는 다음 파일에 있다.

- `calibration_result.json`
- `cartographer_trajectory_nodes_full.csv`
- `lidar_icp_windows.csv`
- `calibration_report.png`

## IMU 보정 결과

정지 구간 gyro z의 중앙값은 `0.0300 rad/s`, 10~90% trimmed mean은
`0.028365 rad/s`였다. LiDAR yaw에 대한 강건 회귀 결과는 다음과 같다.

```yaml
gyro_z_bias_rad_s: 0.026983
gyro_z_sign: -1.0
gyro_z_scale: 1.061768
```

LiDAR yaw와 보정 IMU yaw의 상관계수는 `0.9750`, 구간 yaw RMSE는
`0.0221 rad`였다. 이 값은 해당 전원 인가 세션의 bias이므로 매번
출발 전 5초 정지 상태로 다시 확인하는 것이 좋다.

`command_odom_node`는 IMU가 최신일 때만 gyro와 명령 기반 yaw를 섞는다.
`/imu`가 없거나 0.15초 이상 stale이면 자동으로 조향 곡률 모델로
돌아가므로 IMU 없는 백도 같은 설정으로 재생할 수 있다.

## 임시 설정으로 백 재생

먼저 빌드한다.

```bash
cd ~/kookmin_autonomous_competition_teamKAI/xycar_ws
colcon build --symlink-install --packages-select xycar_map_nav
source install/setup.bash
```

터미널 1:

```bash
ros2 launch xycar_map_nav real_mapping.launch.py \
  use_sim_time:=true \
  command_odom_params_file:=$(ros2 pkg prefix xycar_map_nav)/share/xycar_map_nav/config/command_odom_rosbag_20260724_experimental.yaml
```

터미널 2:

```bash
ros2 bag play ~/work/rosbags/extracted/track_full_sensor_20260724_163034 \
  --clock
```

baseline 비교 때는 `command_odom_params_file`만
`command_odom_real.yaml`로 바꾼다. 두 결과에서 다음을 비교한다.

- 같은 고정 구조물이 왕복 때 겹치는지
- 직선 벽이 이중으로 생기지 않는지
- 출발점 근처 재방문 오차
- `/odom` 거리와 Cartographer/SLAM 궤적 거리의 구간별 비율

## production 값을 확정하는 실차 시험

1. 평평한 직선에 줄자로 5.00 m 두 지점을 표시한다.
2. 첫 표시 1 m 전부터 `speed=3`으로 주행해 표시 통과 시 이미
   정상속도가 되게 한다.
3. 두 표시 통과 시점의 `/odom` 위치 차이를 기록한다.
4. 정방향 5회, 역방향 5회를 반복해 중앙값을 사용한다.
5. 다음 식으로 갱신한다.

```text
새 speed_gain = 시험에 사용한 speed_gain × 5.00 / odom 중앙거리
```

6. 같은 절차를 `speed=5`, `8`에서 반복한다. 각 속도별 환산값 차이가
   3%를 넘으면 단일 gain 대신 속도별 lookup table을 사용한다.
7. 3 m 이상 원을 좌우 각각 3회 주행해 `curvature = yaw 변화 / 이동거리`
   로 조향 표를 다시 확인한다.

실측 5 m 결과가 나오기 전에는 `0.086683`을 저속 shadow/맵핑 비교용으로만
사용하고 production 설정에는 승격하지 않는다.

## 재현 명령

ROS 2 Humble 환경에서 실행한다.

```bash
python3 scripts/calibrate_command_odom_from_bag.py \
  ~/work/rosbags/extracted/track_full_sensor_20260724_163034 \
  --output-dir data/odom_calibration/2026-07-24 \
  --cartographer-pbstream \
    ~/work/rosbags/slam_output/track_full_sensor_20260724_163034_glass_balanced.pbstream \
  --trajectory-csv \
    data/odom_calibration/2026-07-24/cartographer_trajectory_nodes_full.csv
```

Python 의존성은 ROS 2의 `rosbag2_py`, NumPy, SciPy, Matplotlib이다.
전역 궤적 CSV는 `scripts/export_cartographer_trajectory_nodes.py`로
`/trajectory_node_list`의 2,400개 점을 중복 제거 없이 저장한 것이다.
