# Windows Codex 인수인계

작성일: 2026-07-24

기준 브랜치: `simulation`

기준 커밋: `3fc12fef99669cf407aab698f71850c241c033b2`

이 문서는 Ubuntu PC에서 진행한 긴 Codex 대화를 다른 Windows PC의 Codex가
이어받기 위한 요약본이다. 원문 대화를 그대로 옮긴 파일이 아니라, 실제 작업을
계속하는 데 필요한 목표, 결정, 코드 위치, 검증 결과와 명령을 모은 실행 가능한
인수인계 문서다.

## 1. Windows Codex에 처음 보낼 지시문

저장소를 연 다음 새 Codex 대화에 아래 문장을 그대로 보낸다.

```text
이 저장소의 CODEX_WINDOWS_HANDOFF.md를 처음부터 끝까지 읽고,
README.md와 문서에서 링크한 현재 상태를 확인해라.

기준은 simulation 브랜치와 커밋
3fc12fef99669cf407aab698f71850c241c033b2이다.

이 프로젝트는 국민대학교 Xycar 대회를 위한 ROS2 Humble/Gazebo
sim-to-real 프로젝트다. 현재 최종 주행 구조는 하나의
xycar_hybrid_drive 노드가 다음 우선순위로 /xycar_motor를 직접 발행한다.

1. CONE_RULE
2. LANE_RULE_CURVE
3. MODEL_STRAIGHT

기존 동작이나 모델을 임의로 교체하지 말고, 먼저 git 상태와 빌드 환경,
모델 파일, 토픽, 테스트 결과를 확인해서 현재 상태를 보고해라.
Windows 네이티브 PowerShell에서 ROS/Gazebo를 실행하려 하지 말고
WSL2 Ubuntu 22.04 또는 원래 Ubuntu PC를 사용해라.
사용자 승인 없이 실차 모터를 활성화하지 말고 항상 shadow 모드부터 확인해라.
```

## 2. 프로젝트의 최종 목표

전체 방향은 다음 순서다.

1. 국민대학교 대회 트랙과 치수가 유사한 Gazebo 월드 제작
2. 실차 카메라, 조향, 속도, 지연과 유사한 시뮬레이션 구축
3. 안정적인 룰베이스 기본 주행
4. 룰베이스와 키보드 주행 데이터 수집
5. Canonical 이미지 기반 모방학습
6. 차선 이탈 없이 완주하는 것을 최우선으로 하는 강화학습
7. 성공한 주행 중 랩타임을 줄이고 평균 속도를 높이는 강화학습
8. 직선은 학습 정책, 곡선은 저지연 룰베이스, 라바콘은 LiDAR
   룰베이스로 전환하는 최종 실차 주행

현재는 8번 구조까지 코드로 구현한 상태다.

## 3. 현재 확정된 최종 주행 구조

패키지:

```text
src/xycar_hybrid_drive
```

최종 명령 경로:

```text
카메라 canonical + LaserScan
  -> xycar_hybrid_drive
  -> /xycar_motor : std_msgs/msg/Float32MultiArray
```

중간 후보 명령 토픽과 별도 arbiter를 두지 않는다. 하나의 노드가 모드를
선택하고 최종 모터 명령을 직접 발행한다.

모드 우선순위:

```text
1. CONE_RULE
2. LANE_RULE_CURVE
3. MODEL_STRAIGHT
```

### MODEL_STRAIGHT

- Canonical 카메라 기반 temporal camera-speed 정책을 사용한다.
- 직선에서 학습 모델로 조향과 속도를 출력한다.
- 현재 기본 실차 속도 cap은 안전을 위해 `8.0`이다.
- 곡선으로 판단된 프레임에서는 모델 추론 자체를 생략한다.

### LANE_RULE_CURVE

- Canonical 이미지에서 노란 중앙선과 흰 경계선을 사용한다.
- Stanley와 Pure Pursuit 계열의 기존 룰베이스를 사용한다.
- 곡선 진입은 빠르게 하고, 직선 복귀는 3프레임 확인해 모드 떨림을 막는다.
- 곡선에서는 모델 추론 대기 없이 룰 명령을 바로 발행한다.

### CONE_RULE

- `teamkai/hwj` 커밋 `30a4b6a`의 라바콘 알고리즘을 패키지 내부로 옮겼다.
- 처리 순서는 다음과 같다.

```text
LaserScan
  -> 거리와 시야각 필터
  -> DBSCAN 군집화
  -> 라바콘 크기 필터
  -> 좌우 라바콘 그룹
  -> 통로 중심 경로
  -> Pure Pursuit
  -> 실측 조향 command 변환
```

- 3프레임 연속 유효한 통로가 검출되면 진입한다.
- 짧은 LiDAR 누락에는 이전 경로를 제한된 시간 동안 유지한다.
- 기본 라바콘 속도 cap은 `9.5`이다.
- 현재 기본은 hwj와 같이 LiDAR 통로 검출을 사용하며 YOLO 라바콘 확인은
  필수 조건이 아니다.

디버그 토픽:

```text
/hybrid/mode
/hybrid/debug
/hybrid/cone_clusters
/hybrid/cone_path
/xycar_motor_shadow
```

자세한 설명:

```text
src/xycar_hybrid_drive/README.md
```

## 4. 지금까지 확정된 실차 인터페이스

실차 운영 환경:

```text
OS              Ubuntu 22.04
ROS             ROS2 Humble
ROS_DOMAIN_ID   7
ROS_NAMESPACE   xycar
```

모터 입력:

```text
/xycar_motor
std_msgs/msg/Float32MultiArray
data[0] = steering command
data[1] = speed command
```

기존 ROS1 VESC 변환:

```text
steering_angle(rad) = -0.0068 * angle_command
speed(m/s)          =  0.08   * speed_command
```

확인된 실질 조향 포화는 약 `-42.5 .. +42.4 command`다. 프로젝트의 조향
명령은 보통 `-42 .. +42` 범위를 사용한다.

VESC 주요 값:

```text
speed_to_erpm_gain             4614
steering_angle_to_servo_gain  -1.2135
steering_angle_to_servo_offset 0.5004
servo_min                      0.15
servo_max                      0.85
```

동역학 실측과 추가 측정 절차:

```text
docs/sim_to_real_vehicle_calibration.md
docs/real_vehicle_dynamics_tests.md
```

## 5. 현재 카메라와 인지 계약

차선 주행 정책의 최종 입력은 원본 RGB 카메라가 아니라 domain 차이를 줄인
Canonical 도로 이미지다.

현재 실차 저지연 경로:

```text
1280x1024 MJPEG 최신 프레임
  -> 선택된 7 Hz 프레임만 decode 및 fisheye rectify
  -> LR-ASPP MobileNetV3-Small segmentation, 256x144
  -> white/yellow mask
  -> BEV
  -> canonical 256x144
  -> 룰베이스 또는 camera-only 정책
```

현재 차선 인지 주력은 YOLO가 아니다. 실차 CPU에서 YOLO segmentation이
약 2~4 Hz로 느렸기 때문에 LR-ASPP MobileNetV3-Small 256x144 경량 경로로
바꿨다. Canonical 출력과 정책 명령 주기는 실차 기준 약 `7 Hz`다.

중요 문서:

```text
docs/2026-07-22_lane_seg_control_canonical.md
docs/real_low_latency_handoff_20260724.md
docs/2026-07-21_real_canonical_range_fix.md
docs/2026-07-15_1p5m_canonical_validation.md
```

실차 rosbag 분석에서 기존 전체 지연은 약 400 ms 수준까지 관찰됐고,
2026-07-24 코드에서 중간 ROS 이미지 토픽과 중복 warp를 줄여 인지 경로를
단축했다. 개발 PC rosbag 기준 인지 callback p50은 약 21 ms였지만 실차
ASUS에서 다시 측정해야 한다.

## 6. 트랙과 시뮬레이션

트랙 월드의 현재 주력 파일:

```text
worlds/kookmin_xycar_track_final.sdf
```

CAD/DXF의 센티미터 단위를 미터로 변환해 차선을 만들었고, 도로는 회색,
흰 경계선과 노란 점선을 사용한다. 선 두께는 24 mm 기준이다. 트랙은 실제
설계도의 길이와 곡선 형상을 최대한 반영하도록 여러 차례 수정했다.

다음 항목은 이전 대화에서 반복적으로 조정했으므로 임의로 원상복구하지 않는다.

- 오른쪽 S자 곡선의 흰 경계선
- 중앙선 점선 간격과 곡률
- 정지선 제거
- 차량 시작 위치
- 카메라와 LiDAR 위치 및 카메라에 차체가 보이는 범위
- Gazebo GUI 패널과 월드 저장 구조

현재 시뮬레이션 전체 설명:

```text
docs/current_simulation_guide.md
README.md
```

## 7. 룰베이스, 모방학습, 강화학습 연혁

### 룰베이스

- Canonical BEV에서 흰 경계선과 노란 중앙선을 검출한다.
- 노란 중앙선을 주 기준으로 경로를 만든다.
- 한쪽 선만 보일 때는 차폭을 이용한 제한된 보완을 사용한 이력이 있다.
- 직선 오실레이션을 줄이기 위해 Stanley를 주력으로 조정했다.
- 곡선에서는 Pure Pursuit 성분과 preview를 사용한다.
- 실차 곡선 이탈은 인지와 정책 지연 때문에 조향이 늦는 문제가 핵심이었다.

관련 문서:

```text
docs/real_vehicle_deployment.md
docs/real_curve_fix_handoff_20260724.md
```

### 모방학습

- Gazebo 룰베이스와 키보드 주행으로 데이터를 수집했다.
- Canonical 이미지와 steering/speed 명령을 저장한다.
- 정지 직전 10초처럼 차선 이탈 또는 실패로 판정한 구간을 폐기하는 수집
  규칙을 만들었다.
- 정상 주행뿐 아니라 복귀 가능한 차선 이탈 복구 데이터도 수집했다.
- 여러 세션을 합쳐 train/validation을 세션 단위로 나눈다.
- 카메라-only 정책으로 전환했으며 차선 주행에는 LiDAR를 입력하지 않는다.

관련 문서:

```text
docs/sim_imitation_learning.md
docs/domain_randomized_collection.md
docs/canonical_model_latest.md
```

### 강화학습

목표 우선순위:

```text
1. 차선 이탈 없이 완주
2. 성공한 주행 중 랩타임 최소화
```

보상에는 진행, 중앙선 오차, 위험 속도, 큰 조향 진동, 랩타임을 반영했다.
직선에서 작은 조향 부호 변경 자체보다 큰 좌우 오실레이션을 억제하는 것이
목표다. 여러 Gazebo worker로 병렬 수집하는 코드와 고속 후보 모델이 있다.

현재 Git에 포함된 최종 정책 기본 경로:

```text
src/xycar_rl/models/final_rule_td3_bc_uncapped_avg17_20260723/
  camera_speed_td3_bc_best.pth
```

관련 문서:

```text
docs/high_speed_rl_20260717.md
docs/codex_handoff_high_speed_rl_20260717.md
docs/lap_time_rl_20260723.md
docs/final_rule_rl_20260723.md
docs/reinforcement_learning_roadmap.md
```

## 8. Windows PC 준비

이 저장소는 ROS2 Humble, Gazebo Sim과 Linux 장치 경로를 사용하므로
Windows 네이티브 PowerShell만으로 전체 시뮬레이션을 재현하지 않는다.

권장 구성:

1. Windows에 Codex가 포함된 최신 ChatGPT 데스크톱 앱 설치
2. WSL2에 Ubuntu 22.04 설치
3. WSL2 안에 ROS2 Humble과 Gazebo Sim 설치
4. 저장소를 WSL2의 Linux 파일 시스템에 clone
5. Windows Codex에서 해당 WSL 프로젝트 폴더를 열거나 WSL 터미널을 사용

WSL2 Ubuntu에서:

```bash
cd ~
git clone --branch simulation \
  git@github.com:steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  xycar_kookmin_gazebo_track
cd ~/xycar_kookmin_gazebo_track

git status -sb
git log -1 --oneline
```

예상 HEAD:

```text
3fc12fe Add single-node hybrid model rule and cone drive
```

ROS가 설치된 뒤:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to \
  xycar_rule_drive xycar_rl xycar_hybrid_drive xycar_final_drive \
  --symlink-install
source install/setup.bash
```

Windows PC가 시뮬레이션을 실행하지 않고 코드 검토와 대화 인수인계만 할
목적이라면 ROS 설치 없이 저장소만 clone해도 된다. 실제 실행 검증은 원래
Ubuntu PC에 SSH로 접속해 진행할 수 있다.

## 9. 새 PC에서 검증할 명령

하이브리드 단위 테스트:

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

PYTHONPATH="$PWD/src/xycar_rule_drive:$PWD/src/xycar_hybrid_drive:$PYTHONPATH" \
python3 -m pytest -q \
  src/xycar_rule_drive/test/test_canonical_stanley_pursuit.py \
  src/xycar_hybrid_drive/test
```

기준 결과:

```text
48 passed
```

Gazebo에서 하이브리드 실행:

```bash
ros2 launch xycar_hybrid_drive hybrid_sim.launch.py \
  headless:=gui \
  enable_rviz:=true \
  model_speed_cap:=8.0 \
  cone_speed_cap:=9.5
```

현재 대회 월드에 라바콘 통로가 없다면 직선 모델과 곡선 룰베이스 전환만
확인된다. 라바콘 모드는 폭 `0.68~0.98 m` 정도의 LiDAR 라바콘 통로가 있어야
활성화된다.

## 10. 실차 실행 안전 절차

실차 PC에서는 Gazebo를 실행하지 않는다. 먼저 카메라와 ROS1 VESC bridge 등
차량의 기존 장치를 실행한다.

빌드 후 shadow 모드:

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=hybrid \
  drive_enabled:=false \
  model_speed_cap:=8.0 \
  cone_speed_cap:=9.5
```

반드시 확인:

```bash
ros2 topic info /xycar_motor -v
ros2 topic echo /hybrid/mode
ros2 topic echo /xycar_motor_shadow
```

확인 조건:

- `/xycar_motor` 최종 publisher가 하나인지 확인
- 조향 부호가 실제 차량과 맞는지 확인
- 직선, 곡선, 라바콘 전환이 예상대로인지 확인
- emergency stop 방법 확인
- 바퀴를 든 상태에서 먼저 확인

현장 책임자가 승인한 뒤에만:

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=hybrid \
  drive_enabled:=true \
  model_speed_cap:=8.0 \
  cone_speed_cap:=9.5
```

## 11. Git과 파일 취급 규칙

- 기준 브랜치는 `simulation`이다.
- 원격 저장소:

```text
git@github.com:steveandy-sudo/kookmin_autonomous_competition_teamKAI.git
git@github.com:yunny22/kookmin_sim_to_real.git
```

- Ubuntu 개발 PC에는 아직 다른 실험의 수정 및 대용량 분석 결과가 많이
  남아 있다.
- `git reset --hard`, `git checkout --`, 무분별한 `git clean`을 사용하지 않는다.
- 요청한 작업의 파일만 골라 stage하고 커밋한다.
- dataset, rosbag, `analysis/`, `runs/`의 대용량 파일은 Git에 없을 수 있다.
- 새 PC에서 누락된 데이터가 필요하면 먼저 Git 추적 여부와 원래 Ubuntu PC의
  경로를 확인한다.

## 12. 새 Codex가 문서를 읽는 순서

1. `CODEX_WINDOWS_HANDOFF.md`
2. `README.md`
3. `src/xycar_hybrid_drive/README.md`
4. `docs/real_low_latency_handoff_20260724.md`
5. `docs/real_curve_fix_handoff_20260724.md`
6. `docs/current_simulation_guide.md`
7. 수행할 작업에 맞는 RL, 인지, 실차 문서

이 순서로 읽은 다음 바로 코드를 수정하지 말고 먼저 다음을 보고한다.

```text
- 현재 branch와 HEAD
- dirty worktree 여부
- 사용 가능한 ROS/Gazebo 환경
- 모델 파일 존재 여부
- 실행하려는 노드와 /xycar_motor publisher 수
- 다음 작업에서 수정할 파일 범위
```

## 13. 현재 다음 단계

가장 가까운 다음 검증은 실차 shadow 모드에서 세 모드 전환과 지연을 rosbag으로
측정하는 것이다.

1. 직선에서 `MODEL_STRAIGHT`
2. 곡선 진입 즉시 `LANE_RULE_CURVE`
3. 곡선 프레임에서 모델 추론이 생략되는지 확인
4. 라바콘 통로에서 `CONE_RULE`
5. 라바콘 종료 후 안정적으로 차선 주행 복귀
6. 최종 `/xycar_motor` publisher가 항상 하나인지 확인

이 검증이 끝나기 전에는 속도 cap을 높이지 않는다.
