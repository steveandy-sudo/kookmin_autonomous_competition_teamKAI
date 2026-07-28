# 실차 BC 모델 최종 실행 절차

> 2026-07-26 이후 ROS1 VESC 컨테이너와 dynamic bridge는 사용하지 않는다.
> native 모터 실행은
> [`native_ros2_vesc_migration.md`](native_ros2_vesc_migration.md)를 따른다.

이 문서는 `simulation` 브랜치에 포함된 카메라+LiDAR 모방학습 모델을 Xycar
실차에서 처음 검증할 때 사용하는 순서다. Gazebo, `ros_gz_bridge`,
`xycar_gazebo_bridge`는 실차에서 실행하지 않는다.

## 모델 정보

- 학습 데이터: 14개 시뮬 세션, 정지 프레임 제외 `73,553`장
- 입력: RGB 카메라 `3x90x160`, LiDAR 거리+유효성 `2x360`
- 출력: 조향 정규값 하나
- 조향 제한: Xycar angle command `-42~42`
- 속도: 모델 출력이 아니라 추론 노드의 안전 규칙으로 결정
- 첫 실차 속도: 직선/곡선 모두 command `3`
- 보류 테스트 세션 MAE: `2.18` command
- 모델 SHA-256:
  `960d5dcb64d7cae42f23e1038d36c7d866c510f35bab2f5240f898283ff1fbfa`

## 0. 안전 조건

다음 조건을 모두 만족하기 전에는 `drive_enabled:=true`를 사용하지 않는다.

1. 물리 비상 정지 담당자가 차량 옆에 있다.
2. 처음에는 구동 바퀴를 바닥에서 띄운다.
3. 룰베이스, 키보드 조종, 다른 자율주행 publisher를 모두 종료한다.
4. `/xycar_motor` subscriber는 `xycar_vesc_driver` 하나이고, 자율주행
   publisher는 아직 0개인지 확인한다.
5. 카메라 또는 LiDAR를 끊었을 때 shadow speed가 0으로 바뀌는지 확인한다.

## 1. Clone 및 빌드

실차 PC의 새 터미널에서 실행한다.

```bash
cd ~
git clone --branch simulation \
  git@github.com:steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  kookmin_sim_to_real
cd ~/kookmin_sim_to_real

source /opt/ros/humble/setup.bash
python3 -c "import torch; print(torch.__version__)" || \
  python3 -m pip install --user torch

colcon build --packages-select il_data_tools --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=7
```

이미 clone한 저장소를 갱신할 때는 다음을 사용한다.

```bash
cd ~/kookmin_sim_to_real
git switch simulation
git pull --ff-only origin simulation
source /opt/ros/humble/setup.bash
colcon build --packages-select il_data_tools --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=7
```

설치된 모델이 이번 최종 모델인지 확인한다.

```bash
MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_policy_scripted.pt"
test -f "$MODEL" && ls -lh "$MODEL"
sha256sum "$MODEL"
```

출력 해시는 문서 상단의 `960d5d...fbfa`와 같아야 한다.

## 2. 실차 장치 실행 및 계약 확인

카메라와 LiDAR를 실행하고 native VESC driver를 먼저 shadow로 확인한다.

```bash
source /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws/install/setup.bash
ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
  drive_enabled:=false
```

그 다음 이 저장소의 새 터미널에서:

```bash
cd ~/kookmin_sim_to_real
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7

ros2 topic list -t | grep -E 'image|scan|xycar_motor'
ros2 topic hz /image_raw
ros2 topic hz /scan
ros2 topic info /xycar_motor -v
```

BC 모델은 시뮬의 `/image_raw`로 학습했으므로 실차에서도 640x480 raw
`/image_raw`를 우선 사용한다.

```bash
export CAMERA_TOPIC=/image_raw
```

차량 구성상 raw 토픽이 없고 보정 영상만 있으면 아래처럼 바꿀 수 있지만,
입력 영상 분포가 달라지므로 반드시 shadow 결과를 다시 확인한다.

```bash
export CAMERA_TOPIC=/wide_camera/rect/image_raw
```

필수 타입은 다음과 같다.

```text
$CAMERA_TOPIC  sensor_msgs/msg/Image
/scan          sensor_msgs/msg/LaserScan
/xycar_motor   std_msgs/msg/Float32MultiArray
```

## 3. Shadow 검증

터미널 1에서 모터 출력을 만들지 않는 shadow 모드로 시작한다.

```bash
cd ~/kookmin_sim_to_real
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
export CAMERA_TOPIC=${CAMERA_TOPIC:-/image_raw}

ros2 launch il_data_tools real_policy_inference.launch.py \
  image_topic:=$CAMERA_TOPIC \
  scan_topic:=/scan \
  device:=cpu \
  drive_enabled:=false \
  speed_command:=3.0 \
  min_speed_command:=3.0
```

터미널 2에서 출력과 처리 상태를 확인한다.

```bash
cd ~/kookmin_sim_to_real
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7

ros2 topic echo /il/policy_motor_shadow
ros2 topic echo /il/policy_debug
```

`/il/policy_debug` 배열 순서는 다음과 같다.

```text
[steer_norm, raw_angle, filtered_angle, speed,
 scan_offset_ms, inference_ms, inference_count, dropped_sync_count]
```

확인 기준:

- `inference_count`가 계속 증가한다.
- `scan_offset_ms`가 대부분 `50ms` 이하다.
- 직선에서 `filtered_angle`이 0 근처이고 좌우 부호가 실차와 맞는다.
- CPU `inference_ms`가 카메라 주기보다 충분히 짧다.
- 카메라 또는 LiDAR를 중지하면 0.5초 안에 shadow speed가 0이 된다.
- shadow 중에는 `/xycar_motor`의 자율주행 publisher가 생기지 않는다.

하나라도 만족하지 않으면 실제 모터 출력을 켜지 않는다.

## 4. 바퀴를 띄운 실제 출력 시험

shadow launch를 `Ctrl+C`로 완전히 종료한다. 차량 구동 바퀴를 띄우고 물리
비상 정지를 준비한다. native VESC driver도 disabled 인스턴스를 종료하고
별도 터미널에서 명시적으로 다시 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws/install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
  drive_enabled:=true
```

그 뒤 터미널 1에서 다음을 실행한다.

```bash
cd ~/kookmin_sim_to_real
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
export CAMERA_TOPIC=${CAMERA_TOPIC:-/image_raw}

ros2 launch il_data_tools real_policy_inference.launch.py \
  image_topic:=$CAMERA_TOPIC \
  scan_topic:=/scan \
  motor_topic:=/xycar_motor \
  device:=cpu \
  drive_enabled:=true \
  speed_command:=3.0 \
  min_speed_command:=3.0
```

다른 터미널에서 publisher가 BC 하나뿐인지 확인한다.

```bash
ros2 topic info /xycar_motor -v
```

조향 방향, 바퀴 회전 방향, `Ctrl+C` 후 `[0, 0]` 정지, 카메라/LiDAR 단절
정지를 확인한다.

## 5. 바닥 첫 주행 최종 명령

바퀴 공중 시험을 통과한 뒤 넓은 공간에서 차량을 차선 중앙에 놓는다. 속도
command `3`은 실측 출발 데드존을 넘는 최소 명령이므로 더 낮추지 않는다.

```bash
cd ~/kookmin_sim_to_real
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
export CAMERA_TOPIC=/image_raw

ros2 launch il_data_tools real_policy_inference.launch.py \
  image_topic:=$CAMERA_TOPIC \
  scan_topic:=/scan \
  motor_topic:=/xycar_motor \
  device:=cpu \
  drive_enabled:=true \
  speed_command:=3.0 \
  min_speed_command:=3.0
```

정상 종료는 실행 터미널에서 한 번의 `Ctrl+C`다. 프로세스 오류나 bridge
단절에는 소프트웨어 정지만 믿지 말고 반드시 물리 비상 정지를 사용한다.

## 6. 절대 동시에 실행하지 않을 노드

다음 노드들은 모두 `/xycar_motor`를 발행할 수 있으므로 BC 실제 주행과 동시에
실행하지 않는다.

- `xycar_lane_rule_driver`
- `keyboard_teleop`
- `kookmin_legacy_camera_driver`
- 다른 모방학습/자율주행 노드

실차 첫 결과는 카메라 원본, `/scan`, `/il/policy_debug`, `/xycar_motor`를 rosbag으로
기록해 시뮬과 조향 분포 및 추론 지연을 비교한다.
