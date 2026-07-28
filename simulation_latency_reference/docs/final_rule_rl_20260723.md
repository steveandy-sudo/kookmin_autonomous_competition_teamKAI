# 2026-07-23 중앙선 룰베이스와 무제한-cap TD3+BC

이 문서는 2026-07-23 기준 최종 주행 경로를 재현하기 위한 기준 문서다.
두 주행기는 모두 `/perception/canonical_road_image`를 7 Hz로 받는다.

## 공통 입력과 모터 계약

```text
canonical: 256x144 BGR
배경:      (36, 36, 36)
흰 경계:   (255, 255, 255)
노란선:    (0, 220, 255)
모터:      /xycar_motor, Float32MultiArray [angle_command, speed_command]
```

실차와 시뮬은 같은 canonical 영상 계약을 사용해야 한다. 주행 노드에서 원본
카메라 색상이나 BEV 파라미터를 다시 보정하지 않는다.

## 실차 통합 실행

실차 카메라 `/wide_camera/rect/image_raw`가 먼저 발행 중이어야 한다. 아래
통합 launch는 LR-ASPP canonical 인지와 선택한 주행기 하나만 실행한다.
룰베이스와 RL을 동시에 모터 topic에 연결하면 안 된다.

룰베이스 shadow:

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=rule drive_enabled:=false
```

RL shadow:

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=rl drive_enabled:=false device:=cpu
```

이미 canonical 인지를 별도로 실행 중이라면 `start_perception:=false`를
추가한다. 아래 확인을 통과한 뒤 같은 명령의 `drive_enabled`만 `true`로 바꾼다.

```bash
ros2 topic hz /perception/canonical_road_image
ros2 topic info /xycar_motor -v
ros2 topic echo /xycar_motor_shadow
ros2 topic echo /rl/policy_motor_shadow
```

## 1. 중앙선 룰베이스

파일:

```text
xycar_ws/src/xycar_rule_drive/xycar_rule_drive/canonical_stanley_pursuit_driver.py
xycar_ws/src/xycar_rule_drive/config/canonical_stanley_pursuit.yaml
```

처리 순서:

1. 노란 점선을 최대 `0.38m` 간격까지 같은 중앙선으로 연결한다.
2. 현재 보이는 노란선이 주 경로이며, 완전히 사라질 때만 흰 경계와 실측
   반차폭 `0.20m`로 중앙선을 복원한다.
3. 차량이 실제 중앙에 놓이도록 노란선 기준 오른쪽 `0.10m`를 목표 경로로 쓴다.
4. 곡선에서는 Pure Pursuit의 먼 위치 오차와 Stanley의 근거리 횡오차·heading
   오차를 융합한다.
5. 직선에서는 Stanley를 주 제어기로 사용한다. `±0.10m` 안에서 중앙선에
   접근 중이면 선을 넘기 전에 반대 보정을 시작한다.
6. 직선의 작은 좌우 부호 변화는 곡선 전환으로 간주하지 않는다. 직선 조향
   반영률은 `0.25`, 변화율은 `180 command/s`이며 곡선 값은 유지한다.
7. 속도 명령은 곡선에서도 최소 `17`, 직선에서 최대 `20`이다.
8. 흰선과 노란선이 모두 사라지면 마지막 유효 조향각을 선이 다시 검출될
   때까지 시간 제한 없이 유지한다. 이때 속도만 `4`로 낮춘다.

시뮬레이션 검증:

```text
3/3 lap complete
mean speed command: 18.07, 18.22, 18.13
max cross-track error: 0.307, 0.320, 0.350m
```

### Gazebo에서 보기

터미널 1:

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xycar_rl rl_sim.launch.py headless:=gui enable_rviz:=true
```

터미널 2:

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xycar_rule_drive canonical_stanley_pursuit.launch.py
```

### 실차 shadow와 구동

먼저 현재 실차용 LR-ASPP 인지를 실행해 canonical 영상이 7 Hz로 정상인지
확인한다. 이후 룰베이스는 반드시 shadow부터 실행한다.

```bash
ros2 launch xycar_rule_drive canonical_stanley_pursuit_real.launch.py \
  drive_enabled:=false

ros2 topic echo /xycar_motor_shadow
ros2 topic hz /rule_drive/connected_yellow_path
```

바퀴를 공중에 띄운 상태에서 모터 publisher가 하나뿐인지 확인한 뒤에만:

```bash
ros2 launch xycar_rule_drive canonical_stanley_pursuit_real.launch.py \
  drive_enabled:=true
```

## 2. Camera-only TD3+BC

선택 모델:

```text
package:
  xycar_ws/src/xycar_rl/models/final_rule_td3_bc_uncapped_avg17_20260723/
  camera_speed_td3_bc_best.pth
selected epoch: 14
input: previous + current canonical image, 6 channels
output: normalized steering + learned speed command
training range: speed command 4..24
extra deployment cap: disabled, 0.0
LiDAR policy input: none
```

학습에는 완주한 룰베이스 세션만 사용했다. 정상 시작과 `4~10cm`, 최대 `6°`
복귀 시작을 섞었고 off-track 세션은 accepted dataset에 포함하지 않았다.
보상은 진행 거리, 중앙선 횡오차, heading, 조향 변화량, 안전한 실제 속도,
위험 상태 과속, 시간 효율, 큰 좌우 반복 진동 및 완주 여부를 사용한다.

추가 cap이 없다는 뜻은 모델의 학습 범위 `4..24` 안에서 나온 속도를 배포
단계에서 다시 자르지 않는다는 뜻이다. 물리 모터 명령 범위 자체가 사라지는
것은 아니다.

검증:

```text
fixed start:      5/5 lap complete, mean command 18.46..18.63
recovery starts:  5/5 lap complete, mean command 18.51..18.76
recovery max CTE: 0.247..0.265m
observed max command: 23.16
reported speed_cap: 0.00
```

학습 라벨은 이미 7 Hz에서 실제 적용된 최종 명령이다. 따라서 이 모델에는
추가 adaptive steering과 시간 필터를 다시 적용하지 않는다:

```text
adaptive_steering_enabled=false
steering_temporal_alpha=1.0
speed_temporal_alpha=1.0
max_inference_rate_hz=7.0
```

### Gazebo 전체 트랙

터미널 1:

```bash
ros2 launch xycar_rl rl_sim.launch.py headless:=gui enable_rviz:=true
```

터미널 2:

```bash
MODEL_DIR="$(ros2 pkg prefix xycar_rl)/share/xycar_rl/models/final_rule_td3_bc_uncapped_avg17_20260723"

ros2 run xycar_rl rollout_policy \
  --project-root "$PWD" \
  --policy-kind camera_speed_td3_bc \
  --checkpoint "$MODEL_DIR/camera_speed_td3_bc_best.pth" \
  --episodes 1 --max-steps 1500 --start-progress-fraction 0.0 \
  --recovery-probability 0 --s-curve-focus-probability 0 \
  --action-noise 0 --speed-action-noise 0 \
  --min-speed-command 4 --max-speed-command 24 \
  --speed-cap-command 0 \
  --disable-adaptive-steering \
  --steering-temporal-alpha 1.0 --speed-temporal-alpha 1.0 \
  --device cuda
```

### 실차 shadow

전용 launch는 위 런타임 계약과 패키지 모델을 자동 선택한다.

```bash
ros2 launch xycar_rl final_uncapped_avg17_real.launch.py \
  drive_enabled:=false device:=cpu

ros2 topic echo /rl/policy_motor_shadow
ros2 topic echo /rl/policy_status
ros2 topic hz /rl/policy_motor_shadow
```

이 모델은 속도 명령 17 이상을 적극적으로 출력하므로 첫 실차 시험에서 바로
`drive_enabled:=true`로 실행하면 안 된다. shadow, 바퀴 공중, 비상 정지,
짧은 직선, 단일 곡선 검증을 통과한 뒤 폐쇄 트랙에서:

```bash
ros2 launch xycar_rl final_uncapped_avg17_real.launch.py \
  drive_enabled:=true device:=cpu
```

## 3. 이전 RL 모델을 하나씩 비교

이전 모델은 모두 현재 최종 모델보다 느린 `4..12` 속도 범위로 학습되었다.
먼저 실차 모터 출력을 막은 shadow로 같은 구간을 반복 재생하며 조향 출력,
추론 주기와 stale 정지를 비교한다. 한 번에 주행 노드는 하나만 실행하고,
각 명령을 `Ctrl+C`로 종료한 뒤 다음 명령을 실행한다.

공통 준비:

```bash
cd ~/kookmin_autonomous_competition_teamKAI
source /opt/ros/humble/setup.bash
source install/setup.bash

MODEL_ROOT="$(ros2 pkg prefix xycar_rl)/share/xycar_rl/models"

test_old_rl_shadow() {
  ros2 launch xycar_rl real_shadow.launch.py \
    policy_kind:=camera_speed_td3_bc \
    checkpoint_path:="$MODEL_ROOT/$1" \
    min_speed_command:=4.0 max_speed_command:=12.0 \
    deployment_speed_cap:=4.0 \
    max_inference_rate_hz:=7.0 \
    drive_enabled:=false lidar_safety_enabled:=false device:=cpu
}
```

각 계열의 체크포인트를 오래된 순서로 시험하는 명령:

```bash
# reward v1: epoch 015가 선택 모델, 020은 이탈한 비교 모델
test_old_rl_shadow high_speed_td3_bc_reward_v1_20260717/camera_speed_td3_bc_epoch_005.pth
test_old_rl_shadow high_speed_td3_bc_reward_v1_20260717/camera_speed_td3_bc_epoch_010.pth
test_old_rl_shadow high_speed_td3_bc_reward_v1_20260717/camera_speed_td3_bc_epoch_015.pth
test_old_rl_shadow high_speed_td3_bc_reward_v1_20260717/camera_speed_td3_bc_epoch_020.pth

# focus v3: epoch 042가 선택 모델
test_old_rl_shadow high_speed_td3_bc_focus_v3_20260717/camera_speed_td3_bc_epoch_033.pth
test_old_rl_shadow high_speed_td3_bc_focus_v3_20260717/camera_speed_td3_bc_epoch_036.pth
test_old_rl_shadow high_speed_td3_bc_focus_v3_20260717/camera_speed_td3_bc_epoch_039.pth
test_old_rl_shadow high_speed_td3_bc_focus_v3_20260717/camera_speed_td3_bc_epoch_042.pth

# dual DAgger v6: epoch 051이 선택 모델, 054는 4/5 완주 비교 모델
test_old_rl_shadow high_speed_td3_bc_dual_dagger_v6_20260717/camera_speed_td3_bc_epoch_051.pth
test_old_rl_shadow high_speed_td3_bc_dual_dagger_v6_20260717/camera_speed_td3_bc_epoch_054.pth

# cap 8 계열: 둘 다 이탈 이력이 있으므로 shadow 비교만 허용
test_old_rl_shadow high_speed_td3_bc_cap8_v7_20260717/camera_speed_td3_bc_epoch_054.pth
test_old_rl_shadow high_speed_td3_bc_cap8_v7_20260717/camera_speed_td3_bc_epoch_057.pth
test_old_rl_shadow high_speed_td3_bc_cap8_v11_20260717/camera_speed_td3_bc_epoch_072.pth
test_old_rl_shadow high_speed_td3_bc_cap8_v11_20260717/camera_speed_td3_bc_epoch_075.pth
```

실차에서 우선 비교할 대표 모델은 다음 세 개다.

| 순서 | 체크포인트 | Gazebo 결과 | 실차 시작 cap |
|---:|---|---|---:|
| 1 | `reward_v1/epoch_015` | cap 6, 5/5 완주 | 4 |
| 2 | `focus_v3/epoch_042` | cap 7, 5/5 완주 | 4 |
| 3 | `dual_dagger_v6/epoch_051` | cap 7.5, 5/5 완주 | 4 |

각 shadow 실행 중 다음 명령으로 결과를 기록한다.

```bash
ros2 topic hz /rl/policy_motor_shadow
ros2 topic echo /rl/policy_motor_shadow
ros2 topic echo /rl/policy_status
```

### 이전 모델을 Gazebo에서 한 바퀴씩 보기

먼저 다른 터미널에서 Gazebo를 한 번만 실행한다.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xycar_rl rl_sim.launch.py headless:=gui enable_rviz:=true
```

모델 비교용 터미널에서 함수를 만든다.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

MODEL_ROOT="$(ros2 pkg prefix xycar_rl)/share/xycar_rl/models"

test_old_rl_gazebo() {
  ros2 run xycar_rl rollout_policy \
    --project-root "$PWD" \
    --policy-kind camera_speed_td3_bc \
    --checkpoint "$MODEL_ROOT/$1" \
    --episodes 1 --max-steps 3000 --seed 20260724 \
    --start-progress-fraction 0.0 --recovery-probability 0.0 \
    --s-curve-focus-probability 0.0 \
    --action-noise 0.0 --speed-action-noise 0.0 \
    --min-speed-command 4.0 --max-speed-command 12.0 \
    --speed-cap-command "$2" --device cuda
}
```

대표 모델을 순서대로 한 바퀴씩 실행한다.

```bash
test_old_rl_gazebo high_speed_td3_bc_reward_v1_20260717/camera_speed_td3_bc_epoch_015.pth 6.0
test_old_rl_gazebo high_speed_td3_bc_focus_v3_20260717/camera_speed_td3_bc_epoch_042.pth 7.0
test_old_rl_gazebo high_speed_td3_bc_dual_dagger_v6_20260717/camera_speed_td3_bc_epoch_051.pth 7.5
```

cap 8 계열은 과거에 트랙 이탈이 확인되었으므로 실차 구동 후보가 아니다.
Gazebo에서 실패 상태를 재현하거나 새 모델과 비교할 때만 사용한다.

## 빌드와 회귀 테스트

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash

colcon build --packages-up-to \
  xycar_final_drive \
  --symlink-install
source install/setup.bash

python3 -m pytest -q \
  xycar_ws/src/xycar_rule_drive/test \
  xycar_ws/src/xycar_rl/test
```
