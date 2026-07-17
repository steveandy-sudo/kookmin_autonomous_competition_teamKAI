# 다른 PC Codex용 고속 RL 인수인계

작성일: 2026-07-17
기준 브랜치: `simulation`

## 1. Codex가 먼저 알아야 할 목표

국민대학교 Xycar 시뮬레이션에서 canonical BEV 카메라만 사용해 조향과 속도를
동시에 출력하는 temporal TD3+BC 정책을 개선한다. LiDAR는 이 차선 주행 정책의
입력이 아니다. 현재 목표는 큰 직선 오실레이션을 만들지 않으면서 전체 트랙의
승인 속도 상한을 `7.5 -> 8.0 이상`으로 단계적으로 높이는 것이다.

실차에는 시뮬레이션 승인 속도를 바로 적용하지 않는다. 모든 새 모델은 cap 4
shadow부터 시작한다.

## 2. 현재 확정 상태

현재 승인 모델:

```text
xycar_ws/src/xycar_rl/models/high_speed_td3_bc_dual_dagger_v6_20260717/
  camera_speed_td3_bc_epoch_051.pth
```

Gazebo 고정 seed `20260724..20260728`, 시작 progress 0, action noise 0에서:

```text
speed cap          7.5
lap success        5/5
mean speed command 7.21..7.31
max CTE            0.198..0.280m
large oscillation  0 event in 4/5, 1 event / 4 steps in 1/5
```

`epoch 054`는 4/5라 선택하지 않았다. 승인 모델 `epoch 051`도 cap 8 첫 시험에서
`18.82m`, CTE `0.390m`로 이탈했다.

cap 8 실패 상태를 추가 수집해 다음 후보까지 학습했다.

```text
xycar_ws/src/xycar_rl/models/high_speed_td3_bc_cap8_v7_20260717/
  camera_speed_td3_bc_epoch_054.pth
  camera_speed_td3_bc_epoch_057.pth
```

오프라인 `17..21m` 전문가 조향 MAE는 v6 epoch 51의 `0.1941`에서 v7 epoch 54
`0.1773`, epoch 57 `0.1743`으로 감소했다. 하지만 v7 epoch 57의 cap 8 폐루프
첫 시험은 seed `20260724`에서 `18.90m`, CTE `0.429m`, heading `-38.9deg`로
이탈했다. 따라서 오프라인 MAE 개선만으로 승격하지 않았고, 현재 승인 기준선은
계속 v6 epoch 51 / cap 7.5다.

그 뒤 v8, v9, v11까지 추가 학습했다. 가장 최신 actor-only 후보는 다음이다.

```text
xycar_ws/src/xycar_rl/models/high_speed_td3_bc_cap8_v11_20260717/
  camera_speed_td3_bc_epoch_072.pth
  camera_speed_td3_bc_epoch_075.pth
```

v11 epoch 75는 cap 8 seed `20260724`를 완주했지만 seed `20260726`에서
`19.49m` 이탈했고, epoch 72도 같은 seed에서 `19.44m` 이탈했다. 승인 모델 v6
epoch 51을 cap 7.6과 7.55로 올린 시험도 각각 4/5였다. 그러므로 **GitHub에 더
높은 속도 후보가 존재해도 승인 cap은 7.5 그대로**다.

## 3. 중요한 데이터 계약

모델 상태와 행동:

```text
state      = [이전 canonical, 현재 canonical], 6x90x160
action[0]  = normalized steering, -1..1 -> command -42..42
action[1]  = normalized speed, -1..1 -> command 4..12
policy     = camera only, no LiDAR
```

DAgger transition schema는 `5`다. 한 transition 안에서 두 행동을 반드시 구분한다.

```text
action_norm / speed_command
  실제 차량에 적용되어 next_state를 만든 행동
  Critic의 (state, action, next_state)에 사용

expert_action_norm / expert_speed_command
  같은 state에서 privileged track expert가 제시한 교정 행동
  Actor의 BC target에 사용
```

blend 행동으로 움직였는데 expert 행동을 Critic action으로 저장하면 잘못된
transition이 된다. `/rl/action_applied`와 `/rl/action_expert`는 state timestamp와
정확히 맞춰 기록한다. `require_action_trace`와
`require_expert_action_trace`를 모두 켠다.

DAgger 조향 안정화 순서도 중요하다. learner raw 조향에 실제 runtime과 같은
stabilizer를 한 번 적용한 뒤, 이미 자체 stabilizer가 적용된 expert 행동과 blend한다.
blend 결과를 다시 안정화하면 expert 쪽에 이중 필터가 걸려 잘못된 궤적이 된다.
순수 expert 수집은 `--learner-blend 0`을 사용하며 현재 코드는 이중 필터를 만들지
않는다. 과거 잘못 수집한 `high_speed_expert_cap8_multiseed_v9_invalid_double_filter_20260717`
세션은 절대 학습에 사용하지 않는다.

## 4. 새 PC 설치와 검증

```bash
cd ~
git clone --branch simulation \
  git@github.com:steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  xycar_kookmin_gazebo_track
cd ~/xycar_kookmin_gazebo_track

source /opt/ros/humble/setup.bash
rosdep install --from-paths xycar_ws/src --ignore-src -r -y
colcon build --packages-up-to xycar_rl --symlink-install
source install/setup.bash

python3 -m pytest -q xycar_ws/src/xycar_rl/test
```

Gazebo와 승인 모델 확인:

```bash
# terminal 1
ros2 launch xycar_rl rl_sim.launch.py headless:=gui enable_rviz:=false

# terminal 2
MODEL="$PWD/xycar_ws/src/xycar_rl/models/high_speed_td3_bc_dual_dagger_v6_20260717/camera_speed_td3_bc_epoch_051.pth"
ros2 run xycar_rl rollout_policy \
  --project-root "$PWD" --policy-kind camera_speed_td3_bc \
  --checkpoint "$MODEL" \
  --episodes 5 --max-steps 3000 --seed 20260724 \
  --start-progress-fraction 0.0 --recovery-probability 0 \
  --s-curve-focus-probability 0 --action-noise 0 \
  --min-speed-command 4 --max-speed-command 12 \
  --speed-cap-command 7.5 --device cuda
```

## 5. GitHub에 없는 로컬 학습 자산

`datasets/`와 루트 `models/`는 `.gitignore` 대상이다. GitHub의
`xycar_ws/src/xycar_rl/models/`에는 배포·평가용 actor-only 파일만 들어 있다.
Critic, target, optimizer까지 정확히 이어서 학습하려면 원본 PC에서 다음 자산을
별도로 복사한다.

```text
datasets/rl/high_speed_dagger_dual_action_focus_v5_4500_20260717       8.9MB
datasets/rl/high_speed_dagger_dual_action_iter2_v6_20260717            5.4MB
datasets/rl/high_speed_dagger_cap8_focus_v7_20260717                   4.8MB
datasets/rl/high_speed_dagger_cap8_expert75_v8_20260717
datasets/rl/high_speed_expert_cap8_multiseed_v9_20260717
datasets/rl/high_speed_dagger_runtime_matched_v11_20260717
models/rl/camera_speed_temporal_td3_bc_high_speed_dual_dagger_v6_20260717
models/rl/camera_speed_temporal_td3_bc_high_speed_cap8_v7_20260717
models/rl/camera_speed_temporal_td3_bc_high_speed_cap8_v9_20260717
models/rl/camera_speed_temporal_td3_bc_high_speed_cap8_v11_20260717
```

예시:

```bash
rsync -av --info=progress2 \
  as@SOURCE_PC:/home/as/xycar_kookmin_gazebo_track/datasets/rl/high_speed_dagger_dual_action_focus_v5_4500_20260717 \
  datasets/rl/
rsync -av --info=progress2 \
  as@SOURCE_PC:/home/as/xycar_kookmin_gazebo_track/datasets/rl/high_speed_dagger_dual_action_iter2_v6_20260717 \
  datasets/rl/
rsync -av --info=progress2 \
  as@SOURCE_PC:/home/as/xycar_kookmin_gazebo_track/datasets/rl/high_speed_dagger_cap8_focus_v7_20260717 \
  datasets/rl/
rsync -av --info=progress2 \
  as@SOURCE_PC:/home/as/xycar_kookmin_gazebo_track/models/rl/camera_speed_temporal_td3_bc_high_speed_cap8_v7_20260717 \
  models/rl/
```

복사할 수 없다면 actor-only v11 epoch 75 후보로 새 dual-action DAgger 데이터를 수집하고,
그 actor를 `--initial-checkpoint`로 사용해 Critic을 새로 학습한다.

## 6. cap 8 DAgger 재수집

터미널 1에서 server-only Gazebo:

```bash
ros2 launch xycar_rl rl_sim.launch.py headless:=-s enable_rviz:=false
```

터미널 2에서 recorder:

```bash
SESSION="$PWD/datasets/rl/high_speed_dagger_cap8_next"
ros2 run xycar_rl rl_transition_recorder --ros-args \
  -p output_dir:="$SESSION" -p max_transitions:=3000 \
  -p require_action_trace:=true \
  -p require_expert_action_trace:=true \
  -p require_lidar:=false -p max_rate_hz:=10.0
```

터미널 3에서 실패 구간 DAgger:

```bash
MODEL="$PWD/xycar_ws/src/xycar_rl/models/high_speed_td3_bc_cap8_v11_20260717/camera_speed_td3_bc_epoch_075.pth"
ros2 run xycar_rl rollout_dagger \
  --project-root "$PWD" --checkpoint "$MODEL" \
  --episodes 30 --max-steps 700 --total-steps 2500 \
  --seed 20260780 --learner-blend 0.50 \
  --policy-min-speed-command 4 --expert-min-speed-command 8 \
  --max-speed-command 12 --minimum-speed-curvature 0.9 \
  --curvature-preview-m 2.0 \
  --start-progress-fraction 0.55 \
  --start-progress-fraction 0.60 \
  --start-progress-fraction 0.65 \
  --start-progress-fraction 0.70 \
  --start-progress-fraction 0.74 \
  --start-progress-fraction 0.78 \
  --start-progress-jitter 0.008 \
  --recovery-probability 0.25 --device cuda
```

종료 후 `metadata.json`에서 다음을 확인한다.

```text
schema_version == 5
fallback_action_count == 0
exact_action_count == expert_action_count == transition_count
```

## 7. 다음 작업 순서와 통과 기준

1. v11 epoch 72/75는 cap 8 다중 seed gate에서 실패했으므로 승인 모델로 사용하지 않는다.
2. 실패 progress `19.4m` 전후(`17.5..20.5m`) runtime-matched DAgger를 추가한다.
3. 새 후보는 cap 7.5, seed `20260724..20260728`에서 먼저 5/5 회귀 검증한다.
4. 회귀 검증을 통과한 후보만 cap 8의 1 seed, 5 seed, 20 seed gate로 올린다.
5. cap 8의 20 seed가 통과한 뒤에만 cap 8.5를 시도한다.

승격 기준:

```text
lap_complete 5/5 이상
off_track / collision / observation_timeout 0
max CTE가 이전 승인 모델보다 명백히 악화되지 않음
large_osc_events가 반복적으로 증가하지 않음
동일 seed에서 재실행해도 성공
```

`observation_timeout`은 정책 실패와 분리한다. Gazebo와 bridge를 재시작한 뒤 같은
seed를 다시 시험한다. 더 높은 epoch 번호만으로 모델을 선택하지 않는다.

## 8. 작업 범위와 안전 규칙

- 트랙 월드, Gazebo bridge, perception 설정은 이 고속 학습 작업의 범위가 아니다.
- 기존 변경이 있는 worktree에서는 관련 없는 파일을 되돌리거나 함께 commit하지 않는다.
- GitHub에는 선택 actor, 평가 결과, 코드와 문서를 올린다.
- full checkpoint와 대용량 데이터는 별도 전송 여부를 명확히 기록한다.
- 실차 `drive_enabled=true`는 shadow, 바퀴 공중, 직선, 단일 곡선 검증 후에만 사용한다.
- 실차 속도는 항상 cap 4부터 한 단계씩 올린다.

## 9. 새 PC의 Codex에게 줄 지시문

아래 내용을 새 컴퓨터의 Codex에 그대로 전달한다.

```text
simulation 브랜치의 docs/codex_handoff_high_speed_rl_20260717.md와
docs/high_speed_rl_20260717.md를 먼저 끝까지 읽어라. 현재 승인 기준선은
high_speed_td3_bc_dual_dagger_v6_20260717 epoch 051, Gazebo cap 7.5의 5/5
완주다. v11 epoch 072/075는 cap 8 다중 seed gate에서 실패했으므로 승인하지 말고
문서 7절의 runtime-matched DAgger부터 계속하라. DAgger에서는 learner 조향만
runtime stabilizer를 거친 뒤 expert와 blend해야 하며 blend 결과를 다시 필터링하지
마라. transition schema 5에서 applied action은 Critic,
expert action은 Actor
BC target이라는 계약을 절대 섞지 마라. 관련 없는 world/perception/bridge의 기존
변경은 되돌리거나 commit하지 마라. 각 후보는 고정 seed 전체 트랙으로 검증하고,
통과한 actor와 결과 및 재현 명령을 문서화한 뒤 simulation 브랜치에 올려라.
실차 실행은 하지 말고 shadow 명령만 제공하며 cap 4부터 시작하라.
```
