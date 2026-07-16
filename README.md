# Kookmin Xycar Sim-to-Real Autonomous Driving

국민대학교 Xycar 자율주행 트랙을 Gazebo Sim에 재현하고, 시뮬레이션에서 만든
인지·제어·학습 결과를 실제 Xycar까지 옮기기 위한 프로젝트입니다.

```text
트랙 재현
  -> 실차 동역학을 반영한 차량·센서 모델
  -> 카메라 룰베이스 주행
  -> 실차 shadow 검증
  -> raw RGB 모방학습
  -> canonical BEV 기반 Sim-to-Real
  -> 실차 저속 폐루프 검증              <- 현재
  -> 실차 데이터 보강 및 Offline RL      <- 다음
```

> 기준일: 2026-07-16  
> 기준 브랜치: `simulation`

---

## 1. 현재 진행 상황

현재 주력 경로는 **canonical BEV 카메라 영상 + LiDAR를 사용하는 Behavioral
Cloning(BC) 정책의 실차 검증**입니다.

### 완료된 범위

- 국민대학교 대회 트랙 기반 Gazebo 최종 월드
- 실차 크기, 센서 위치, 속도 deadzone, 좌우 비대칭 조향과 응답 지연 반영
- Gazebo 카메라 픽셀만 사용하는 차선 인지와 룰베이스 주행
- 실차 perception-only, shadow mode와 sensor-timeout 정지
- raw RGB BC의 시뮬 학습·실차 이식 및 domain gap 확인
- 실차/Gazebo 공통 `256x144`, 좌우 `1.4m`, 전방 `1.5m` canonical BEV
- 실차 가시성과 recovery 상황을 반영한 약 20만 장 모델 학습

### 최신 모델

```text
모델:       drive_canonical_policy_scripted.pt
SHA-256:    8da35fa8a56904f9970679da0af68d938f60848991af3abf9a37027d9195b58b

유효 데이터:                         199,898장
신규 실차 가시성 시뮬:              100,000장
  - 일반 주행 60,348 / recovery 39,652
기존 clean 50k 가시성 파생본:         99,898장

validation MAE:                       1.906 angle command
held-out test MAE:                    1.892 angle command
```

최신 수치와 모델 해시는
[`docs/canonical_model_latest.md`](docs/canonical_model_latest.md)를 source of truth로
사용합니다.

### 지금 해야 하는 일

1. 최신 모델 실차 shadow 검증
2. 조향 scale `100 / 130 / 140 / 150%` 비교
3. 가장 낮은 안정 scale로 바퀴 공중 시험
4. 직선 2~3m와 완만한 곡선에서 speed command `3` 저속 시험
5. 카메라·LiDAR 동기화, perception, inference, 조향 전달 지연 분리 측정
6. 필요하면 실차 canonical shadow 데이터로 fine-tuning
7. BC와 safety supervisor가 안정된 뒤 Offline RL로 확장

오프라인 MAE와 시뮬 주행 성공은 실차 완주를 보장하지 않습니다. 현재 단계의
완료 기준은 `shadow -> 바퀴 공중 -> 직선 -> 단일 곡선 -> 전체 트랙` 시험을
순서대로 통과하는 것입니다.

---

## 2. 처음 받은 사람을 위한 빠른 시작

### 2.1 Clone과 빌드

Private repository이므로 GitHub 인증이 필요합니다.

```bash
cd ~
git clone --branch simulation \
  git@github.com:steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  kookmin_sim_to_real
cd ~/kookmin_sim_to_real

source /opt/ros/humble/setup.bash

rosdep install --from-paths \
  xycar_ws/src/kaiev26_msgs \
  xycar_ws/src/xycar_perception \
  xycar_ws/src/xycar_rule_drive \
  xycar_ws/src/xycar_gazebo_bridge \
  xycar_ws/src/il_data_tools \
  --ignore-src -r -y

colcon build --packages-select \
  kaiev26_msgs xycar_perception xycar_rule_drive \
  xycar_gazebo_bridge il_data_tools \
  --symlink-install

source install/setup.bash
```

이미 clone되어 있다면 로컬 변경을 확인한 뒤 갱신합니다.

```bash
git status -sb
git switch simulation
git pull --ff-only origin simulation
```

### 2.2 목적별 실행 경로

| 하고 싶은 일 | 먼저 볼 문서/명령 |
|---|---|
| 최신 BC 모델을 실차에서 검증 | 아래 3절과 [`canonical_model_latest.md`](docs/canonical_model_latest.md) |
| 최신 BC 모델을 Gazebo에서 실행 | 아래 4절 |
| 트랙·인지·룰베이스를 처음부터 이해 | [`current_simulation_guide.md`](docs/current_simulation_guide.md) |
| 데이터 수집과 재학습 | [`domain_randomized_collection.md`](docs/domain_randomized_collection.md) |
| 실차 룰베이스만 실행 | [`real_vehicle_deployment.md`](docs/real_vehicle_deployment.md) |

---

## 3. 현재 주력 경로: 실차 Canonical BC

실차 PC에서는 Gazebo, `ros_gz_bridge`, `xycar_gazebo_bridge`를 실행하지 않습니다.
차량의 기존 카메라, LiDAR, ROS1 VESC stack과 ROS1-ROS2 dynamic bridge를 먼저
실행합니다.

### 3.1 입력 계약 확인

```bash
export ROS_DOMAIN_ID=7

ros2 topic list -t | grep -E 'wide_camera|image_raw|scan|xycar_motor'
ros2 topic info /wide_camera/rect/image_raw -v
ros2 topic info /scan -v
ros2 topic info /xycar_motor -v
ros2 topic hz /wide_camera/rect/image_raw
ros2 topic hz /scan
```

| 역할 | 기본 계약 |
|---|---|
| 카메라 | `/wide_camera/rect/image_raw`, `sensor_msgs/msg/Image`, 약 30Hz |
| LiDAR | `/scan`, `sensor_msgs/msg/LaserScan`, 약 10Hz |
| 모터 | `/xycar_motor`, `Float32MultiArray [angle, speed]` |

이미 rectified인 카메라에는 `enable_rectify:=false`를 사용합니다.

### 3.2 Canonical perception 확인

```bash
ros2 launch xycar_perception real_canonical_perception.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false use_compressed_image:=false
```

```bash
ros2 topic hz /perception/canonical_road_image
ros2 run rqt_image_view rqt_image_view /perception/canonical_road_image
```

정상 출력:

```text
256x144 BGR, 좌우 1.4m, 전방 1.5m
배경 (36,36,36)
흰 경계 (255,255,255), 5px
노란선 (0,220,255), 5px
```

현재 runtime은 observation-only입니다. 현재 프레임에서 관측된 선만 출력하며,
사라진 경계를 합성하거나 이전 선을 무기한 유지하지 않습니다.

### 3.3 최신 모델 확인과 shadow 실행

```bash
MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_canonical_policy_scripted.pt"
sha256sum "$MODEL"
```

해시가 이 README 상단의 값과 같은지 확인한 뒤, 단독 perception을 종료하고
통합 shadow를 실행합니다.

```bash
ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  model_path:="$MODEL" \
  source_image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false use_compressed_image:=false \
  scan_topic:=/scan \
  max_steer_scale:=140.0 \
  drive_enabled:=false device:=cpu
```

```bash
ros2 topic hz /il/policy_motor_shadow
ros2 topic echo /il/policy_motor_shadow
ros2 topic echo /il/policy_debug
ros2 run rqt_image_view rqt_image_view /il/policy_input_image
```

Shadow 통과 조건:

- 출력이 약 8~10Hz 이상 지속됨
- 영상-LiDAR timestamp 차가 대부분 50ms 안임
- 카메라나 LiDAR를 끊으면 0.5초 안에 `[0, 0]`이 나옴
- 좌·우 곡선에서 실제 바퀴 방향과 같은 부호가 나옴
- 조향이 `-42~42`이며 NaN이나 반복 포화가 없음
- `/xycar_motor`에는 아직 자율주행 publisher가 없음

과거 5만 장 모델은 130~150% 조향 scale이 더 안정적이었습니다. 최신 모델에서는
`100 / 130 / 140 / 150%`를 다시 비교하고 가장 낮은 안정값을 선택합니다.

### 3.4 바퀴 공중과 저속 시험

물리 비상 정지, 바퀴 공중 시험, 단일 motor publisher와 sensor-timeout 정지를
모두 확인한 뒤에만 실제 출력을 켭니다.

```bash
VALIDATED_SCALE=140.0  # shadow에서 확정한 값으로 변경

ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  model_path:="$MODEL" \
  source_image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false use_compressed_image:=false \
  scan_topic:=/scan motor_topic:=/xycar_motor \
  drive_enabled:=true speed_command:=3.0 \
  max_steer_scale:="$VALIDATED_SCALE" \
  steering_output_sign:=1.0 \
  steering_temporal_alpha:=0.55 \
  sensor_timeout_sec:=0.50 device:=cpu
```

상세 안전 절차와 `/il/policy_debug` 해석은
[`docs/2026-07-13_canonical_sim_to_real_handoff.md`](docs/2026-07-13_canonical_sim_to_real_handoff.md)를
따릅니다.

---

## 4. 시뮬레이션에서 최신 모델 실행

기존 Gazebo, 룰베이스와 다른 `/xycar_motor` publisher를 모두 종료합니다.

```bash
cd ~/kookmin_sim_to_real
source /opt/ros/humble/setup.bash
source install/setup.bash

MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_canonical_policy_scripted.pt"

ros2 launch il_data_tools sim_policy_drive.launch.py \
  project_root:="$PWD" \
  model_path:="$MODEL" \
  image_topic:=/perception/canonical_road_image \
  drive_enabled:=true device:=cuda
```

룰베이스 baseline을 실행하거나 트랙·인지·RViz를 단계별로 확인하려면
[`docs/current_simulation_guide.md`](docs/current_simulation_guide.md)를 사용합니다.

---

## 5. 전체 구조

```text
Gazebo camera / Real camera
             │
             ▼
      xycar_perception
      ├─ environment-specific BEV
      ├─ white/yellow lane detection
      └─ common canonical 256x144
             │
       ┌─────┴──────────────┐
       ▼                    ▼
xycar_rule_drive        il_data_tools
rule-based teacher      BC policy
Pure Pursuit            camera + LiDAR
       │                    │
       └────────┬───────────┘
                ▼
      /xycar_motor [angle, speed]
                │
       ┌────────┴────────────┐
       ▼                     ▼
Gazebo motor bridge      Real VESC bridge
```

### 주요 패키지

| 패키지 | 역할 |
|---|---|
| `kaiev26_msgs` | perception 공통 메시지 |
| `xycar_gazebo_bridge` | `/xycar_motor`를 Gazebo Ackermann 명령으로 변환 |
| `xycar_perception` | BEV, 차선 검출, canonical 영상 |
| `xycar_rule_drive` | 경로 생성, Pure Pursuit, keyboard, 실차 rule shadow |
| `il_data_tools` | 수집, randomization, 학습, 평가, BC inference |
| `lane_bev_tools` | BEV 확인과 이전 카메라 도구 |

### 핵심 토픽

```text
/image_raw 또는 실차 camera       sensor_msgs/msg/Image
/scan                             sensor_msgs/msg/LaserScan
/perception/canonical_road_image  sensor_msgs/msg/Image, 256x144 BGR
/xycar_motor                      Float32MultiArray [angle, speed]
/xycar_motor_shadow               rule-based shadow
/il/policy_motor_shadow           BC shadow
```

---

## 6. 과거부터 현재까지의 개발 과정

| 단계 | 진행 내용 | 다음 단계로 넘어간 이유 |
|---:|---|---|
| 1 | CAD/DXF 기반 국민대 트랙과 실내 배경을 Gazebo에 재현 | 실제 트랙과 비슷한 반복 시험 환경 확보 |
| 2 | 차량 치수, 카메라·LiDAR와 `/xycar_motor [angle, speed]` bridge 구현 | 실차 코드를 같은 인터페이스로 시험하기 위해 |
| 3 | Gazebo camera 픽셀만 사용하는 BEV/HSV 차선 인지 | semantic 정답 없이 실차와 같은 입력 경로 확보 |
| 4 | 노란선·흰선 기반 경로와 Pure Pursuit 룰베이스 완성 | 안정적인 baseline과 학습 teacher 확보 |
| 5 | 7월 12~13일 실차 속도·조향·지연 측정 및 시뮬 동역학 보정 | 시뮬 명령 응답을 실차에 가깝게 맞추기 위해 |
| 6 | 실차 homography/HSV 분리, shadow/RViz와 timeout 정지 | motor를 켜기 전 perception과 명령을 검증하기 위해 |
| 7 | 14개 세션, 73,553장 raw RGB+LiDAR BC | 시뮬 폐루프는 성공했지만 실차 domain gap 확인 |
| 8 | raw 배경·색을 제거한 canonical BEV로 전환 | 조명·가구·노출·렌즈 차이가 조향에 직접 전달되는 문제 해결 |
| 9 | 전방 1.0/1.2/2.0m를 비교하고 1.5m canonical 확정 | 실차와 시뮬에서 중앙 점선 2~3개가 보이는 공통 기하 확보 |
| 10 | recovery, sensor/dynamics randomization, session split | 중앙주행만이 아니라 이탈 복귀와 환경 변화 학습 |
| 11 | 실차 가시성 기반 199,898장 모델 학습 | 선을 임의 변형한 데이터보다 실제 관측 분포가 더 효과적이었음 |
| 12 | 최신 모델 실차 shadow·저속 검증 | 현재 진행 중 |

### 중요한 방향 전환

#### Raw RGB BC에서 canonical BEV로

Raw RGB 모델은 시뮬에서 성공했지만 실차에서는 차선 외의 배경, 조명, 색감과
fisheye 차이를 함께 보면서 조향이 불안정했습니다. 그래서 각 환경의 perception이
차선을 검출한 뒤 같은 물리 범위·색·두께의 canonical 영상으로 정규화하도록
구조를 바꿨습니다.

#### 임의 선 변형에서 실차 가시성 재현으로

선을 직접 옮기거나 굽힌 legacy 3만 장은 test MAE를 `2.70 -> 3.38`로
악화시켰습니다. 현재는 트랙 형상과 차량 경로를 유지하고, 실차에서 관측된
흰선/노란선 소실 비율만 시간 연속적으로 재현합니다.

#### 한 번의 성공에서 반복 가능한 안전 검증으로

현재 완료 기준은 한 바퀴 성공이 아닙니다. shadow, sensor timeout, 단일 motor
publisher, 조향 scale, recovery와 반복 주행을 모두 기록해 재현 가능해야 합니다.

---

## 7. 문서 안내

| 목적 | 문서 | 사용 시점 |
|---|---|---|
| 전체 시뮬·룰베이스 구조 | [`current_simulation_guide.md`](docs/current_simulation_guide.md) | 처음 읽을 때 |
| 최신 모델 해시·성능 | [`canonical_model_latest.md`](docs/canonical_model_latest.md) | 매 실행 전 |
| 7월 13일 이후 시간순 인수인계 | [`2026-07-13_canonical_sim_to_real_handoff.md`](docs/2026-07-13_canonical_sim_to_real_handoff.md) | 문제 원인·실차 절차 확인 |
| 데이터 수집·재학습 | [`domain_randomized_collection.md`](docs/domain_randomized_collection.md) | dataset/pipeline 작업 |
| 1.5m canonical 근거 | [`2026-07-15_1p5m_canonical_validation.md`](docs/2026-07-15_1p5m_canonical_validation.md) | geometry 변경 전 |
| 실차 perception 튜닝 과정 | [`2026-07-14_real_camera_canonical_tuning.md`](docs/2026-07-14_real_camera_canonical_tuning.md) | 역사와 원인 분석 |
| 룰베이스 실차 배포 | [`real_vehicle_deployment.md`](docs/real_vehicle_deployment.md) | rule baseline 시험 |
| 차량·센서 정합 | [`sim_to_real_vehicle_calibration.md`](docs/sim_to_real_vehicle_calibration.md) | 동역학/센서 재측정 |
| 과거 raw RGB BC | [`real_bc_vehicle_runbook.md`](docs/real_bc_vehicle_runbook.md) | legacy 비교용 |

7월 14일 문서의 temporal coasting, missing-boundary synthesis와 persistent
prediction은 당시 실험 기록입니다. 현재 runtime은 observation-only입니다.

문서가 서로 다른 값을 말할 때 우선순위:

```text
현재 launch/config/code
  > docs/canonical_model_latest.md
  > 이 README의 현재 상태
  > 날짜가 있는 과거 실험 문서
```

---

## 8. 저장소 구조

```text
.
├── worlds/                    # 최종/작업 Gazebo 월드
├── media/materials/textures/  # 트랙·차선 texture
├── scripts/                   # 맵·정합·profile 도구
├── docs/                      # 실행 가이드, 근거, 인수인계
├── data/                      # 실차 동역학과 분석 자료
└── xycar_ws/src/
    ├── kaiev26_msgs/
    ├── lane_bev_tools/
    ├── xycar_gazebo_bridge/
    ├── xycar_perception/
    ├── xycar_rule_drive/
    └── il_data_tools/
```

`scripts/generate_kookmin_track.py`를 다시 실행하면 Gazebo에서 수동 조정한 최종
차선 위치를 덮을 수 있습니다. `worlds/kookmin_xycar_track_final.sdf`를 수정하기
전에는 반드시 백업합니다.

---

## 9. 안전 원칙

1. 실차에서는 perception-only와 shadow부터 시작합니다.
2. 첫 실제 출력은 바퀴를 띄운 상태에서 수행합니다.
3. 물리 비상 정지 담당자 없이 자율주행을 켜지 않습니다.
4. `/xycar_motor` publisher는 동시에 하나만 실행합니다.
5. 룰베이스, keyboard, legacy driver와 BC를 동시에 실행하지 않습니다.
6. 카메라 또는 LiDAR가 stale이면 0.5초 안에 정지하는지 확인합니다.
7. gain을 바꾸기 전에 canonical 영상, 부호, timestamp와 지연을 확인합니다.
8. Offline MAE와 시뮬 성공을 실차 안전성의 증거로 간주하지 않습니다.

ROS가 살아 있을 때의 추가 정지 명령:

```bash
ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0]}"
```

---

## 10. 다음 마일스톤

- [ ] 최신 199,898장 모델 실차 shadow 통과
- [ ] 최신 모델 조향 scale 재확정
- [ ] 바퀴 공중 sensor-timeout/종료 정지 통과
- [ ] 직선 및 단일 곡선 speed `3` 통과
- [ ] 전체 트랙 반복 주행과 실패 구간 rosbag 확보
- [ ] 실차 canonical fine-tuning 필요성 판단
- [ ] BC와 룰베이스의 동일 구간 정량 비교
- [ ] safety supervisor 고정
- [ ] Offline RL 관측·행동·보상 계약 설계

현재의 핵심은 기능을 더 많이 붙이는 것이 아니라, 최신 canonical BC 모델을
실차에서 안전하고 반복 가능하게 검증하는 것입니다.
