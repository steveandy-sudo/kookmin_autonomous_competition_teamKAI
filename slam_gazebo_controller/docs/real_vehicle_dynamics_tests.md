# 실차 동역학 측정 코드

이 패키지는 시뮬 차량을 실차에 맞추기 위해 실제 Xycar의 세 가지 값을 측정한다.

- `speed_cmd -> 실제 속도`
- `angle_cmd -> 실제 회전반경 / 조향각`
- 명령을 보낸 뒤 IMU/오도메트리 반응이 나타나기까지의 지연시간

## 빌드

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
colcon build --packages-select xycar_dynamics_test --symlink-install
source install/setup.bash
```

실차 PC에서 `ROS_NAMESPACE=xycar`를 쓰고 있으면 토픽이 `/xycar/imu`, `/xycar/xycar_motor`처럼 보일 수 있다. 그 경우 launch 인자로 토픽명을 바꿔서 실행한다.

## 안전 기본값

기본 실행은 `dry_run:=true`라서 모터 명령을 발행하지 않는다. 실제 차를 움직일 때만 `dry_run:=false`를 넣는다.

실차 구동 전 확인:

- 바퀴가 떠 있는 상태에서 먼저 `steer_step`을 짧게 확인한다.
- `speed_step`, `turn_radius`는 넓고 평평한 공간에서 한다.
- 같은 순간에 다른 주행 노드가 `/xycar_motor`를 발행하지 않게 한다.
- 중단은 `Ctrl+C`를 누른다. 노드는 종료 시 `[0, 0]` 명령을 여러 번 발행한다.

## 속도 커맨드 측정

```bash
ros2 launch xycar_dynamics_test dynamics_test.launch.py \
  test_name:=speed_step \
  dry_run:=false
```

기본 속도 명령은 `5,10,15,20`이다. 더 낮게 하려면:

```bash
ros2 run xycar_dynamics_test dynamics_test_runner --ros-args \
  -p test_name:=speed_step \
  -p dry_run:=false \
  -p speed_commands:="3,5,8,10"
```

오도메트리 토픽이 있으면 더 정확하다.

```bash
ros2 run xycar_dynamics_test dynamics_test_runner --ros-args \
  -p test_name:=speed_step \
  -p dry_run:=false \
  -p use_odom:=true \
  -p odom_topic:=/odom
```

## 조향 반응 지연 측정

차가 아주 천천히 움직이는 상태에서 조향 명령을 단계적으로 바꾼다. IMU yaw-rate 변화가 기준값을 넘으면 자동으로 지연시간을 CSV에 기록한다.

```bash
ros2 launch xycar_dynamics_test dynamics_test.launch.py \
  test_name:=steer_step \
  dry_run:=false
```

속도를 더 줄이고 싶으면:

```bash
ros2 run xycar_dynamics_test dynamics_test_runner --ros-args \
  -p test_name:=steer_step \
  -p dry_run:=false \
  -p steer_step_speed_cmd:=3 \
  -p angle_commands:="10,-10,20,-20"
```

눈으로 봤을 때 반응 시작 순간을 직접 찍고 싶으면 다른 터미널에서:

```bash
ros2 topic pub --once /xycar_dynamics_test/manual_event \
  std_msgs/msg/String "{data: steering_started}"
```

## 회전반경 측정

고정 속도와 고정 조향 명령으로 원호 주행을 시키고, `radius = speed / yaw_rate`로 회전반경을 추정한다.

```bash
ros2 launch xycar_dynamics_test dynamics_test.launch.py \
  test_name:=turn_radius \
  dry_run:=false
```

더 안전한 저속 설정:

```bash
ros2 run xycar_dynamics_test dynamics_test_runner --ros-args \
  -p test_name:=turn_radius \
  -p dry_run:=false \
  -p turn_speed_cmd:=5 \
  -p turn_angle_commands:="15,-15,25,-25"
```

## 로그 분석

로그는 기본적으로 `~/xycar_dynamics_logs`에 저장된다.

```bash
ros2 run xycar_dynamics_test analyze_dynamics_log \
  --csv ~/xycar_dynamics_logs/20260710_120000_turn_radius.csv
```

분석 결과에서 확인할 값:

- `speed_gain_mps_per_cmd`: 시뮬의 `speed_cmd -> m/s` 변환값으로 반영
- `radius_m`: 각 조향 명령에서 실제 회전반경
- `inferred_steering_rad`: 휠베이스 0.32 m 기준으로 역산한 실제 앞바퀴 조향각
- `steering_gain_rad_per_cmd`: 시뮬의 `angle_cmd -> steering_rad` 변환값으로 반영
- `auto_*_response_delay`: 명령 후 실제 반응이 잡힌 시간

CSV 원본에는 매 tick마다 `target_angle_cmd`, `target_speed_cmd`, `yaw_rate_z_rad_s`, `accel_x_mps2`, `estimated_speed_mps`, `estimated_turn_radius_m`, `event`가 저장된다.
