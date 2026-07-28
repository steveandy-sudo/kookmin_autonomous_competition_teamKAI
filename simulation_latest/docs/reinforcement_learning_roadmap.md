# 국민대 Xycar 강화학습 진행 계획

작성일: 2026-07-16

## 1. 현재 구현 상태

강화학습은 완성된 canonical 인지와 BC 기본 주행을 버리지 않고 그 위에서
진행한다. 첫 RL action은 속도가 아니라 **조향 하나만** 사용한다. 속도까지 동시에
학습하면 조향 개선인지 속도 감소 효과인지 구분하기 어렵고, 실차 위험도와 필요한
데이터 양이 크게 늘기 때문이다.

현재 `xycar_rl`에는 다음 기능이 있다.

1. SDF의 노란 CAD dash 44개를 읽어 닫힌 Catmull-Rom 기준 경로를 만든다. CAD
   dash 순서는 본선 대회 주행 방향과 반대이므로 기본 경로는 역순으로 만든다.
   RL 기준 경로는 노란 중앙선 자체를 사용하므로 횡방향 오프셋은 0m이다.
2. 기준 경로 진행량, signed 횡오차, heading 오차를 월드 좌표에서 계산한다.
3. Gazebo를 pause한 채 매 action마다 0.1초만 진행하는 Gymnasium 환경을 제공한다.
4. 같은 seed로 진행 위치, 횡오차, yaw 오차를 재현해 BC와 RL을 비교한다.
5. canonical image, LiDAR, action, reward, next state, done을 transition으로 저장한다.
6. 기존 `resnet18_lidar` BC state dict를 strict하게 RL Actor에 초기화한다.
7. Offline TD3+BC와 bounded residual online TD3를 제공한다.
8. 실차에서는 기본 shadow, 센서 stale 정지, 전방 LiDAR 정지, 조향 clamp를 적용한다.

Gazebo Ackermann odometry는 teleport/reset 뒤에도 odometry 누적 좌표가 남을 수
있으므로 CAD 보상에 직접 사용하지 않는다. 보상과 이탈 판정은
`/world/kookmin_xycar_track/dynamic_pose/info`의 절대 pose를 우선 사용하고,
frame 이름이 없는 현재 `Pose_V` bridge에서는 첫 transform을 차량 pose로 해석한다.
bridge가 차량 frame을 제공하지 못할 때만 odometry pose를 fallback으로 사용하고,
속도는 odometry에서 읽는다.

카메라와 LiDAR는 stamp 차이 최대 `0.25s`의 bounded join gate를 사용한다. canonical
인지가 CPU에서 원본 카메라보다 늦게 발행될 수 있기 때문에 `0.08s`처럼 좁은
게이트를 쓰면 RL 환경이 관측을 계속 버리고 timeout으로 종료할 수 있다.

## 2. 상태, 행동, 보상

### 상태

- canonical road image: `256x144` ROS 영상에서 모델 입력 `RGB 3x90x160`
- LiDAR: range와 validity mask를 합친 `2x360`
- Gym 보조값: 직전 조향과 고정 속도 명령

BC와 RL은 같은 canonical 인지와 같은 LiDAR 전처리를 사용한다. 원본 RGB 배경을
다시 학습 입력으로 바꾸지 않는다.

### 행동

- RL action `-1~1`
- 실제 Xycar 조향 명령 `-42~42`
- 속도는 시뮬 `4`, 첫 실차 저속 시험 `3`

기존 BC는 조향 명령을 `angle/100`으로 학습했다. RL은 실질 조향 포화인 42를
action 1로 사용하므로 Actor 로드 직후 `100/42`를 곱한다. 이 변환이 없으면 같은
BC 출력도 시뮬에서 조향이 42%로 줄어든다.

### 보상

- 전진 진행량: 양의 보상
- 역방향 진행: 별도 패널티
- 기준 경로 횡오차: 절댓값 패널티
- heading 오차: 절댓값 패널티
- 급격한 조향 변화와 불필요한 큰 조향: 작은 패널티
- 충돌, 이탈, 정지: 큰 terminal 패널티
- 한 바퀴 완료: terminal 보너스

RL의 기본 목표 경로는 노란 중앙선 그 자체이며 횡방향 오프셋은 `0.0m`이다.
따라서 보상 횡오차와 heading 오차 모두 노란 중앙선 기준으로 계산된다.

### 종료조건

- 기준 경로 횡오차가 `0.38m`보다 큼
- chassis contact 센서가 충돌을 보고함
- 이동 명령이 3 이상인데 속도 `0.03m/s` 미만 상태가 2초 지속됨
- 누적 전진이 CAD 경로 길이의 95% 이상
- 120초 제한 또는 센서 timeout

## 3. 데이터 정책

기존 canonical BC 약 20만 장은 Actor 초기화와 행동복제 regularization의 근거다.
하지만 기존 CSV에는 `next_state`, 절대 pose, reward, terminal이 없으므로 TD3
critic 데이터로 사용할 수 없다. 파일 수를 늘리기 위해 가짜 next state를 만들지
않는다.

RL transition은 새로 수집한다.

- 정상 한 바퀴
- S자 시작 전, 내부, 출구
- 좌우 횡오차 `8~22cm`
- yaw 오차 `4~14도`
- 한쪽 선 또는 중앙선이 가려진 canonical 입력
- rule-based와 BC의 서로 다른 행동 분포
- 이탈·충돌·정지 직전과 terminal transition

첫 목표는 10Hz 기준 5만 transition으로 파이프라인을 검사하고, 부족하면
10만~20만 transition으로 늘린다. train/validation은 연속 프레임을 무작위로
섞지 않고 `(session, episode)` 단위로 나눈다.

## 4. 단계별 진행

### 단계 A: BC rollout transition 수집

터미널 1:

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xycar_rl rl_sim.launch.py headless:=-s enable_rviz:=false
```

화면을 보면서 실행하려면 `headless:=gui enable_rviz:=true`를 사용한다.

터미널 2:

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run xycar_rl rl_transition_recorder --ros-args \
  -p output_dir:=$PWD/datasets/rl/bc_rollout_01 \
  -p max_transitions:=100000
```

터미널 3:

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run xycar_rl rollout_policy \
  --project-root "$PWD" --policy-kind bc \
  --checkpoint "$PWD/models/il_policies/drive_canonical_real_reference_200k_20260715/drive_resnet18_lidar_best.pth" \
  --episodes 150 --max-steps 1200 --total-steps 100000 \
  --action-noise 0.02 --device cuda
```

Gym reset은 `/rl/episode_reset`을 발행하므로 recorder는 reset 전 pose와 reset 후
pose를 하나의 transition으로 연결하지 않는다.

### 단계 B: Offline TD3+BC

```bash
ros2 run xycar_rl train_td3_bc \
  --transitions "$PWD/datasets/rl/bc_rollout_01" \
  --bc-checkpoint "$PWD/models/il_policies/drive_canonical_real_reference_200k_20260715/drive_resnet18_lidar_best.pth" \
  --output-dir "$PWD/models/rl/td3_bc_01" \
  --epochs 30 --batch-size 64 --num-workers 8 --device cuda
```

TD3는 두 critic 중 작은 Q를 사용하고 target action에 작은 noise를 더해 Q의
과대평가를 줄인다. Actor loss에는 행동복제 MSE를 함께 넣는다. 따라서 초기에는
BC와 가깝게 움직이고, transition에서 반복적으로 더 높은 return이 확인되는
조향만 점진적으로 선택한다.

### 단계 C: 동일 seed 폐루프 gate

```bash
ros2 run xycar_rl evaluate_closed_loop \
  --project-root "$PWD" \
  --bc-checkpoint "$PWD/models/il_policies/drive_canonical_real_reference_200k_20260715/drive_resnet18_lidar_best.pth" \
  --rl-checkpoint "$PWD/models/rl/td3_bc_01/td3_bc_best.pth" \
  --rl-kind td3_bc --episodes 20 --seed 20260716 \
  --output-dir "$PWD/analysis/rl_eval_td3_bc_01" --device cuda
```

20 seed 탐색 후 후보를 고르고 100 seed로 다시 평가한다. 다음 조건을 모두
만족해야 residual 단계로 간다.

- BC보다 lap 성공률이 낮아지지 않음
- 충돌과 off-track 횟수가 늘지 않음
- 평균·최대 횡오차 중 적어도 하나가 개선됨
- S자 seed와 복귀 seed에서 조향 진동이 늘지 않음
- 검은 canonical 입력이나 센서 timeout을 이용해 보상을 편법으로 얻지 않음

### 단계 D: Residual online RL

```bash
ros2 run xycar_rl train_residual_online \
  --project-root "$PWD" --base-kind scripted \
  --base-checkpoint "$PWD/models/rl/td3_bc_01/td3_bc_actor_scripted.pt" \
  --output-dir "$PWD/models/rl/residual_td3_01" \
  --total-steps 100000 --max-residual-norm 0.25 --device cuda
```

reset의 70%를 S자 진행률 `0.17~0.48`에 놓는다. residual은 기본 정책 조향에
최대 action `0.25`, 즉 약 `10.5` 조향 명령만 더할 수 있다. residual 크기에도
패널티를 주므로 직선에서는 0으로 돌아가고 S자·복귀에서만 개입하도록 유도한다.

Online update는 Gazebo에서만 수행한다. 초기 실차 검증 중에는 가중치를 실시간
업데이트하지 않는다.

### 단계 E: 실차 shadow와 저속 검증

실차 canonical perception과 `/scan`이 먼저 정상이어야 한다.

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash

ros2 launch xycar_rl real_shadow.launch.py \
  policy_kind:=scripted \
  checkpoint_path:=~/kookmin_sim_to_real/models/rl/td3_bc_01/td3_bc_actor_scripted.pt \
  drive_enabled:=false speed_command:=3.0 steering_gain:=1.0 device:=cpu
```

shadow에서 확인할 토픽:

```bash
ros2 topic echo /rl/policy_motor_shadow
ros2 topic echo /rl/policy_debug
ros2 topic echo /rl/policy_status
```

`/rl/policy_debug` 순서는 다음과 같다.

```text
[base_norm, residual_norm, final_norm, angle_command,
 speed_command, front_lidar_min_m, inference_ms, sensor_age_ms]
```

확인 항목:

- 좌우 조향 부호가 실차와 일치
- angle이 `-42~42` 안에 있음
- camera-LiDAR sync 실패와 stale 센서에서 정지
- 전방 `0.25m` 이내 장애물에서 speed 0
- CPU 추론 지연과 sensor age의 p50/p95
- BC보다 포화 조향 또는 좌우 편향이 늘지 않음

현재 BC 실차 기준으로 알려진 140% 보정을 비교할 때는
`steering_gain:=1.4`를 shadow에서만 먼저 확인한다. RL은 시뮬 action 단위가 이미
`-42~42`이므로 첫 비교는 `steering_gain:=1.0`에서 시작한다.

shadow 로그를 먼저 저장한 뒤 사람의 물리 비상정지 수단을 준비하고 속도 3에서
`drive_enabled:=true`를 사용한다. 중단할 때는 먼저 runtime을 종료하고 정지 명령을
보낸다.

```bash
ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0]}"
```

## 5. 그 다음 확장

조향 전용 residual이 시뮬 100 seed와 실차 저속에서 통과한 뒤에만 다음을 검토한다.

1. 속도를 `3, 4, 5` discrete action으로 추가
2. 조명·선 소실·마찰·배터리 전압 domain randomization 확대
3. uncertainty가 클 때 BC나 rule-based로 되돌아가는 supervisor
4. 실차 shadow transition을 이용한 offline 재학습
5. 검증된 데이터만 사용하는 conservative Q-learning 비교

실차 online RL은 최종 단계다. 그 전에는 실차 데이터를 저장만 하고 업데이트는
오프라인에서 수행한다.

## 6. 2026-07-16 검증 기록

- `xycar_rl` 단위 테스트 22개 통과
- SDF validation과 XML validation 통과
- 같은 seed random reset 후 CAD 횡오차 정상 확인
- paused Gazebo `reset + 5 step`에서 image, LiDAR, world pose, speed 갱신 확인
- 실제 Gazebo transition 4개와 파일/CSV/metadata 연속성 확인
- BC checkpoint strict load와 TD3+BC 1 epoch, TorchScript export 확인
- residual rollout 3 step과 residual actor/critic update 확인
- BC/RL 동일 seed 보고서 생성 확인
- shadow 출력 확인: 이 PC의 CPU inference 약 `35.8ms`

마지막 지연값은 이 PC에서 한 번 측정한 smoke 결과이며 실차 성능 보장이 아니다.
실차 rosbag과 실제 주행에서 p50/p95를 다시 측정한다.
