# 고속 temporal TD3+BC 학습 및 실차 단계 검증

작성일: 2026-07-17

## 1. 목적

현재 고속 BC+DAgger 모델을 처음부터 다시 학습하지 않고 초기 Actor로 사용한다.
강화학습은 카메라 canonical BEV만 보고 조향과 속도를 함께 개선한다. 목표는
다음 세 가지다.

1. 노란 중앙선을 벗어나기 전에 곡선 진입 조향을 시작한다.
2. 중앙선과 heading이 안정적일 때 평균 속도를 높인다.
3. 직선에서 작은 조향 보정은 허용하되 큰 좌우 왕복만 줄인다.

초기 모델:

```text
models/rl/camera_speed_compact_temporal2_lead1_dagger_iter2_46k_20260717/camera_speed_bc_best.pth
```

## 2. 상태와 행동

```text
state      = [이전 canonical, 현재 canonical] = 6x90x160
next_state = [현재 canonical, 다음 canonical] = 6x90x160
action[0]  = steering_norm -1..1 -> angle command -42..42
action[1]  = speed_norm -1..1 -> speed command 4..12
```

LiDAR는 차선 주행 Actor와 Critic 입력에 사용하지 않는다. reset, episode 변경,
step 누락 또는 timestamp 불연속이 있으면 이전 프레임 대신 현재 프레임을 두 번
사용해 서로 다른 episode를 연결하지 않는다.

## 3. 보상

기본 보상:

- 전진 진행량: `+8.0 * progress_delta_m`
- 역방향 진행: `-4.0 * reverse_delta_m`
- 중앙선 횡오차: `-1.5 * abs(cross_track_error_m)`
- heading 오차: `-0.8 * abs(heading_error_rad) / pi`
- 조향 변화량과 크기: 작은 패널티
- 충돌 `-50`, 이탈 `-30`, 정지 `-10`, 완주 `+20`

2026-07-17 추가 보상:

- 중앙선 오차 `0.12m`, heading `12deg` 안에서 실제 속도가 높으면 속도 보상
- 위 안전 범위를 벗어나면서 빠르면 위험 속도 패널티
- 매 step `-0.01` 시간 비용을 적용해 같은 진행 거리를 빨리 통과하도록 유도
- CAD 곡률 `0.35 1/m` 이하 직선에서만 큰 조향 왕복 패널티 적용
- 현재 위치뿐 아니라 전방 `1.5m`의 최대 곡률을 확인해 급곡선 진입 전에
  과속 패널티 적용
- 급곡선 목표 속도는 약 command 7, 직선 목표 속도는 학습 상한 command 12로
  분리

큰 조향 왕복 판정은 다음 조건을 모두 만족해야 한다.

```text
최근 12 action, 약 1.2초
abs(steering_norm) >= 0.18인 값만 사용
좌/우 양쪽에 큰 조향 peak가 존재
부호 흐름이 좌->우->좌 또는 우->좌->우처럼 두 번 이상 반전
```

따라서 `+0.10, -0.10, +0.10` 같은 작은 보정은 패널티가 0이다. 큰 조향도
한 번만 방향을 바꾸거나 CAD 곡선 구간이면 오실레이션 패널티가 0이다.

## 4. 강화학습 방식

TD3의 두 Critic은 `(state, steering, speed)`의 장기 return을 학습하고 작은 Q를
사용해 과대평가를 줄인다. Actor는 높은 Q를 선택하면서 기존 데이터의 행동과
가까이 있도록 BC loss를 함께 사용한다. 초기 실험은 `bc_alpha=0.5`로 RL 변화량을
보수적으로 제한한다.

과거 transition의 이미지·행동·next state는 재사용하지만 저장된 구 reward는
사용하지 않는다. 위치, 진행량, heading, 속도와 조향 기록으로 최신 보상을
재계산한다.

Critic에는 저장된 `action`이 실제로 저장된 `next_state`를 만든 transition만
사용한다. 일부 과거 DAgger 데이터는 expert trace를 action으로 저장했지만 실제
next state는 blend action으로 만들어졌으므로 현재 TD3+BC 재학습에서는 제외한다.

## 5. 학습 명령

아래 명령은 최초 reward-v1 실험 기록이다. 최종 focus-v3은 이 결과를 전체
TD3+BC 상태로 이어받아 추가 학습했다.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run xycar_rl train_camera_speed_td3_bc \
  --transitions "$PWD/datasets/rl/high_speed_expert_feedback_min7_max12_gui_20260716" \
  --transitions "$PWD/datasets/rl/high_speed_expert_recovery_min7_max12_gui_20260716" \
  --transitions "$PWD/datasets/rl/high_speed_dagger_blend20_min7_max12_gui_20260716" \
  --transitions "$PWD/datasets/rl/high_speed_dagger_iter2_blend20_min7_max12_gui_20260716" \
  --transitions "$PWD/datasets/rl/high_speed_expert_stable_10k_20260717" \
  --transitions "$PWD/datasets/rl/high_speed_dagger_straight_stable_iter1_5k_20260717" \
  --transitions "$PWD/datasets/rl/high_speed_dagger_lead1_iter2_3k_20260717" \
  --transitions "$PWD/datasets/rl/high_speed_dagger_lead1_seed20260727_1k_20260717" \
  --transitions "$PWD/datasets/rl/high_speed_dagger_lead1_seed20260724_26_2k_20260717" \
  --initial-checkpoint "$PWD/models/rl/camera_speed_compact_temporal2_lead1_dagger_iter2_46k_20260717/camera_speed_bc_best.pth" \
  --output-dir "$PWD/models/rl/camera_speed_temporal_td3_bc_high_speed_reward_v1_20260717" \
  --milestone-dir "$PWD/xycar_ws/src/xycar_rl/models/high_speed_td3_bc_reward_v1_20260717" \
  --min-speed-command 4 --max-speed-command 12 \
  --recompute-rewards --world-sdf "$PWD/worlds/kookmin_xycar_track_final.sdf" \
  --epochs 20 --batch-size 128 --num-workers 8 \
  --actor-lr 0.000005 --critic-lr 0.0003 --bc-alpha 0.5 \
  --policy-noise 0.10 --noise-clip 0.25 \
  --checkpoint-every-epochs 5 --device cuda
```

전체 critic/optimizer 체크포인트는 로컬 `models/`에 저장한다. GitHub에는 실차
평가에 필요한 actor-only 체크포인트와 명령만 저장한다.

### 5.1 focus-v3 추가 학습

실패한 cap 8 정책이 실행한 action과 expert action을 섞지 않고, 실제 적용 action과
next state가 일치하는 expert/feedback transition 및 cap 8 transition만 사용했다.
특히 `16~20m` 곡선 expert 데이터를 5배 확률로 샘플링했다.

```bash
BASE="$PWD/models/rl/camera_speed_temporal_td3_bc_high_speed_preview_v2_20260717/camera_speed_td3_bc_epoch_030.pth"

ros2 run xycar_rl train_camera_speed_td3_bc \
  --transitions "$PWD/datasets/rl/high_speed_expert_feedback_min7_max12_gui_20260716" \
  --transitions "$PWD/datasets/rl/high_speed_expert_recovery_min7_max12_gui_20260716" \
  --transitions "$PWD/datasets/rl/high_speed_expert_stable_10k_20260717" \
  --transitions "$PWD/datasets/rl/high_speed_expert_min6_max12_gui_20260716" \
  --transitions "$PWD/datasets/rl/high_speed_cap8_policy_actual_3k_20260717" \
  --transitions "$PWD/datasets/rl/high_speed_expert_target_progress19_2k_20260717" \
  --focus-transitions "$PWD/datasets/rl/high_speed_expert_target_progress19_2k_20260717" \
  --focus-repeat 5 \
  --initial-checkpoint "$BASE" --resume-checkpoint "$BASE" \
  --output-dir "$PWD/models/rl/camera_speed_temporal_td3_bc_high_speed_focus_v3_20260717" \
  --milestone-dir "$PWD/xycar_ws/src/xycar_rl/models/high_speed_td3_bc_focus_v3_20260717" \
  --min-speed-command 4 --max-speed-command 12 \
  --recompute-rewards --world-sdf "$PWD/worlds/kookmin_xycar_track_final.sdf" \
  --epochs 12 --batch-size 128 --num-workers 8 \
  --actor-lr 0.000003 --critic-lr 0.0001 --bc-alpha 1.5 \
  --policy-noise 0.08 --noise-clip 0.20 --policy-delay 2 \
  --checkpoint-every-epochs 3 --device cuda
```

`--resume-checkpoint`는 Actor만 불러오는 것이 아니라 두 Critic, target network,
optimizer, update count까지 이어받는다.

## 6. Gazebo 단계 검증

터미널 1:

```bash
ros2 launch xycar_rl rl_sim.launch.py headless:=gui enable_rviz:=false
```

터미널 2에서 `EPOCH`과 `CAP`을 단계별로 바꾼다.

```bash
EPOCH=005
CAP=4
MODEL="$PWD/xycar_ws/src/xycar_rl/models/high_speed_td3_bc_reward_v1_20260717/camera_speed_td3_bc_epoch_${EPOCH}.pth"

ros2 run xycar_rl rollout_policy \
  --project-root "$PWD" --policy-kind camera_speed_td3_bc \
  --checkpoint "$MODEL" \
  --episodes 5 --max-steps 3000 --seed 20260724 \
  --start-progress-fraction 0.0 --recovery-probability 0.0 \
  --s-curve-focus-probability 0.0 --action-noise 0.0 \
  --min-speed-command 4 --max-speed-command 12 \
  --speed-cap-command "$CAP" --device cuda
```

단계 순서:

| Epoch | Gazebo 시험 cap | 1차 결과 |
|---:|---:|---|
| 005 | 4 | 120초에 24.58m, 완주선 직전 time limit |
| 010 | 5 | seed 20260724 완주, 최대 횡오차 0.124m |
| 015 | 6 | seed 20260724~28 모두 완주, 큰 오실레이션 0회 |
| 020 | 8 | 19.07m에서 이탈, 승인하지 않음 |

epoch 15의 속도 경계를 추가로 확인한 결과 cap 7은 4/5만 완주했고 한 seed에서
이탈했다. cap 8도 이탈했다. epoch 20은 cap 6 한 회를 완주했지만 같은 seed의
epoch 15보다 최대 횡오차가 컸다. 이 결과로 1차 baseline은 epoch 15와 cap 6으로
정했다. 아래 6.1절의 추가 학습에서 이 baseline을 갱신했다.

epoch 15/cap 6의 5개 seed 결과:

```text
lap success       5/5
max CTE range     0.141..0.179m
mean speed cmd    6.00
large oscillation 0 event / 0 penalized step
```

출력의 `small_straight_flips`는 허용되는 작은 좌우 보정 횟수다. 모델 탈락 판단은
`large_osc_events`, `large_osc_steps`, 최대 횡오차와 이탈 여부를 사용한다.

### 6.1 추가 고속 학습 결과

focus-v3 epoch 42를 같은 고정 시작점과 seed `20260724~20260728`로 다시
검증했다.

```text
checkpoint          focus-v3 epoch 42
approved speed cap  7.0
lap success         5/5
mean speed command  6.94..6.97
max CTE             0.141..0.202m
large oscillation   1 event in 1/5, 0 in 4/5
```

이전 승인 cap 6보다 평균 command 기준 약 15.8% 빨라졌다. cap 7.25는 두 번째
seed에서 이탈해 즉시 탈락했고, cap 8은 1/5만 완주했다. 조향 손실을 더 크게 둔
steer-v4도 cap 7.5에서 3/5, cap 8에서 반복 성공하지 못해 선택하지 않았다.
따라서 **현재 최종 시뮬레이션 모델은 focus-v3 epoch 42, 승인 cap은 7.0**이다.

각 단계는 같은 seed에서 BC 기준보다 완주율이 낮아지거나 최대 횡오차, 큰 조향
왕복 횟수가 증가하면 탈락시킨다. 선택한 focus-v3 epoch 42도 아직 5개 seed만
통과했으므로 이후 20 seed, 최종 100 seed 순서로 확대해야 한다.

## 7. 실차 shadow

```bash
cd ~/kookmin_sim_to_real
source /opt/ros/humble/setup.bash
colcon build --packages-up-to xycar_rl --symlink-install
source install/setup.bash

MODEL_DIR="$(ros2 pkg prefix xycar_rl)/share/xycar_rl/models/high_speed_td3_bc_focus_v3_20260717"

ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=camera_speed_td3_bc \
  checkpoint_path:="$MODEL_DIR/camera_speed_td3_bc_epoch_042.pth" \
  min_speed_command:=4.0 max_speed_command:=12.0 \
  deployment_speed_cap:=4.0 \
  drive_enabled:=false lidar_safety_enabled:=false device:=cpu
```

확인 토픽:

```bash
ros2 topic echo /rl/policy_motor_shadow
ros2 topic echo /rl/policy_debug
ros2 topic echo /rl/policy_status
```

`drive_enabled=true`는 shadow 출력, 조향 부호, 추론 지연, 센서 stale 정지와
물리 비상정지를 확인한 뒤에만 사용한다. epoch 숫자가 커졌다는 이유만으로 더 높은
속도를 승인하지 않는다. 시뮬 cap 7 통과와 관계없이 실차 shadow는 cap 4부터
시작한다. cap 4 shadow와 바퀴 공중 시험, 직선, 단일 곡선을 모두 통과한 뒤에만
cap 5, cap 6, cap 7 순서로 한 단계씩 올린다. cap 7.25 이상은 현재 시뮬
gate에서 탈락했으므로 실차 시험 대상이 아니다.
