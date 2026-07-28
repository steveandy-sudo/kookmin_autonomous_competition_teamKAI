# 완주 우선 고속 강화학습

기준일: 2026-07-23

## 1. 목표

정책 선택 기준은 단순 보상 합이나 평균 속도가 아니라 다음 두 조건을
순서대로 적용한다.

1. 평가한 모든 바퀴를 차선 이탈과 충돌 없이 완주해야 한다.
2. 첫 조건을 만족한 후보끼리만 평균 한 바퀴 시간이 짧은 모델을 선택한다.

한 바퀴가 더 빠르더라도 5회 평가 중 한 번이라도 이탈하면 최종 모델이 될 수
없다. 현재 시뮬레이터의 이탈 판정은 CAD 중심 경로 기준 횡오차 `0.38m`이며,
바퀴 외곽을 직접 재는 판정은 아직 아니다.

## 2. 관측과 행동

- 관측: 현재와 직전의 `256x144` canonical 카메라 영상
- 제어 주기: 실차와 동일한 `7Hz`
- 조향 행동: Xycar angle command
- 속도 행동: Xycar speed command
- 학습 속도 범위: `4..100`
- LiDAR: 차선 주행 정책 입력에서 사용하지 않음

`100`은 무한한 속도가 아니라 현재 실차 모터 명령 인터페이스의 물리적 상한이다.
정책 내부에는 이 범위보다 낮은 추가 speed cap을 두지 않는다.

기존 `4..24` 정책은 새 범위로 바꿀 때 출력이 갑자기 변하지 않도록
`RangeExpandedCameraSpeedActor`로 감싼다. 확장 직후에는 모든 입력에서 기존
조향과 속도를 그대로 출력하고, 추가 학습으로만 더 높은 속도를 사용한다.

## 3. 보상과 데이터 사용

완주 보상과 이탈 패널티가 시간 단축보다 우선하도록 설정한다.

- 완주: 큰 양의 보상
- 차선 이탈 또는 충돌: 큰 음의 보상
- 정체: 음의 보상
- 전진 진행 거리: 양의 보상
- 경과 시간: 매 step 작은 음의 보상
- 이탈 경계에 가까운 주행: 추가 음의 보상
- 직선에서 큰 좌우 조향이 반복됨: 오실레이션 패널티

실패 주행도 삭제하지 않는다. 실패 transition은 Critic이 위험 행동의 낮은
가치를 학습하는 데 사용한다. 단, 실패한 episode의 행동은 Actor의 BC 정답으로
절대 사용하지 않는다. `--bc-successful-episodes-only`가 이 계약을 적용한다.

## 4. 엄격한 모델 선택

모든 후보는 먼저 고정 조건 5바퀴로 screening하고, 최종 승격 전에는 10바퀴를
추가 평가한다.

```text
screening eligible = 5/5 lap_complete
final eligible = 10/10 lap_complete
best = eligible 후보 중 mean_lap_time이 가장 작은 모델
```

초기 5바퀴 screening 기준 모델:

```text
models/rl/lap_time_candidate_eval5_20260723/camera_speed_lap_time_best.pth
5/5 screening safe laps
mean lap time: 16.642s
speed command range: 4..100
```

오프라인 추가 학습 후보:

```text
models/rl/lap_time_offline_round01_20260723/camera_speed_td3_bc_best.pth
3/5 safe laps
mean completed-lap time: 16.377s
result: rejected
```

두 번째 후보는 완주한 바퀴만 보면 빨랐지만 5/5 조건을 만족하지 못했으므로
배포 대상으로 승격하지 않는다.

10바퀴 최종 gate 결과:

| 후보 | 완주 | 성공 바퀴 평균 | 판단 |
|---|---:|---:|---|
| 초기 screening 모델 | `8/10` | `16.587s` | 최종 승인 보류 |
| 조향 고정 speed-only | `8/10` | `16.295s` | 빠르지만 최종 승인 보류 |
| 성공 episode head BC | `7/10` | `16.229s` | 탈락 |

speed-only 후보는 5바퀴에서는 `5/5`, `16.188s`였지만 10바퀴에서 두 번
이탈했다. 따라서 5회 결과만으로 승격하지 않는다. 현재 10/10을 만족한 고속
최종 모델은 아직 없다.

관련 결과:

```text
xycar_ws/src/xycar_rl/models/lap_time_speed_only_round02_20260723/evaluation_5_laps.csv
xycar_ws/src/xycar_rl/models/lap_time_speed_only_round02_20260723/evaluation_10_laps.csv
```

## 5. 병렬 Gazebo 수집

각 worker는 서로 다른 `ROS_DOMAIN_ID`와 `GZ_PARTITION`을 사용한다. 따라서
Gazebo world, ROS topic, reset service와 recorder가 서로 섞이지 않는다.

이 PC에서 2026-07-23에 측정한 결과:

| worker 수 | 작업자별 실효 Hz | 합산 실효 Hz | 판단 |
|---:|---:|---:|---|
| 8 | 약 `4.3..5.5` | 약 `37` | 안정 |
| 10 | 약 `4.9..5.6` | 약 `52` | 장시간 기본값 |
| 12 | 약 `5.4..6.2` | 약 `66` | 짧은 고부하 수집만 |

12개 시험 중 가용 RAM이 약 `3.3GiB`까지 줄고 swap 사용량이 `5.8GiB`까지
올라갔다. GPU 메모리는 약 `5.5GiB`만 사용했으므로 현재 병목은 GPU가 아니라
시스템 RAM이다. 14개 이상은 장시간 수집 안정성을 해칠 가능성이 커 사용하지
않는다.

장시간 수집:

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run xycar_rl collect_parallel_policy \
  --project-root "$PWD" \
  --checkpoint "$PWD/models/rl/lap_time_candidate_eval5_20260723/camera_speed_lap_time_best.pth" \
  --output-root "$PWD/datasets/rl/lap_time_parallel_10w" \
  --workers 10 \
  --episodes 100 \
  --max-steps 250 \
  --control-rate-hz 7 \
  --action-noise 0.01 \
  --speed-action-noise 0.01 \
  --speed-action-bias 0.0 \
  --recovery-probability 0.20 \
  --s-curve-focus-probability 0.30 \
  --device cuda
```

12개 burst 수집은 다른 무거운 프로그램을 종료하고 `--workers 12`로 바꾼다.
각 출력 폴더의 `manifest.json`에는 완주·실패 수, transition 수, worker별 Hz,
합산 Hz와 시작 시간을 포함한 wall-clock 처리량이 기록된다.

실제 10-worker 장기 수집 결과:

```text
datasets/rl/lap_time_parallel_round04_10w_explore_20260723
episodes: 100
lap complete: 56
failed: 44
transitions: 6,825
steady aggregate rate: 53.76Hz
wall-clock transition rate: 27.08Hz
```

## 6. 오프라인 TD3+BC

```bash
ros2 run xycar_rl train_camera_speed_td3_bc \
  --transitions "$PWD/datasets/rl/lap_time_parallel_10w" \
  --initial-checkpoint "$PWD/models/rl/lap_time_candidate_eval5_20260723/camera_speed_lap_time_best.pth" \
  --output-dir "$PWD/models/rl/lap_time_offline_next" \
  --min-speed-command 4 \
  --max-speed-command 100 \
  --epochs 20 \
  --batch-size 64 \
  --num-workers 8 \
  --device cuda \
  --recompute-rewards \
  --reward-objective lap_time \
  --target-right-offset-m 0.10 \
  --off-track-threshold-m 0.38 \
  --lane-margin-start-m 0.24 \
  --bc-successful-episodes-only \
  --speed-extension-only \
  --actor-lr 0.000003 \
  --critic-lr 0.0001 \
  --bc-alpha 0.5
```

학습 스크립트의 validation loss가 가장 낮은 파일은 폐루프 최종 모델을 뜻하지
않는다. 학습 후 반드시 Gazebo 5바퀴 평가를 거쳐야 한다.

`--speed-extension-only`는 승인 모델의 encoder와 조향 head를 동결하고 속도
확장 layer만 학습한다. 이미 안정적인 조향이 오프라인 Q 업데이트 때문에
바뀌는 회귀를 막기 위한 현재 기본 실험 방식이다.

첫 speed-only 실험은 가중치만 동결하고 ResNet BatchNorm running statistics를
고정하지 않아 조향 출력이 변했다. 해당
`models/rl/lap_time_speed_only_round01_20260723` 모델은 사용하지 않는다.
현재 코드는 encoder와 조향 head를 `eval()` 상태로 유지하며 수정 후 모델은
조향 출력 차이 `0.0`, 속도만 약 `+0.39..+0.67 command` 변했다.

10바퀴 평가에서 초기 모델은 진행 거리 약 `4.0..4.3m`, speed-only 후보는
약 `17.3m`, `22.9m`에서 이탈했다. 다음 수집은 이 세 진행 구간의 recovery
시작을 집중적으로 포함하고, 10/10을 회복한 뒤에만 다시 속도를 올린다.

## 7. 실차 적용 제한

`4..100` 모델을 실차 모터에 바로 연결하지 않는다. 먼저 shadow mode에서
조향 부호, canonical 입력 주기, 추론 지연과 출력 속도 분포를 확인한다. 이후
외부 supervisor cap을 낮은 값부터 단계적으로 올린다. 시뮬레이터에서 추가 cap이
없다는 것과 실차 안전 시험에서 cap이 필요하다는 것은 서로 다른 조건이다.
