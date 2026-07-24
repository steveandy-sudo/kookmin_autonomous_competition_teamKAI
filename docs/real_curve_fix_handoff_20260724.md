# 2026-07-24 실차 곡선 이탈 수정 및 Copilot 인수인계

이 문서는 실차 PC의 Copilot/Codex가 `simulation` 브랜치를 받은 뒤
룰베이스와 학습 모델을 같은 조건으로 재현하고, shadow부터 안전하게 검증하기
위한 기준 문서다. 실차에서는 Gazebo를 실행하지 않는다.

## 1. 이번 실차 데이터에서 확인한 문제

분석에 사용한 정상 기록:

```text
rule_speed8_run01_20260724_131500
lap_time_4to100_run01_20260724_133049
```

`rule_speed8_run02`는 3.36초뿐이고 모터 명령이 없어 비교에서 제외했다. 두
정상 기록의 canonical adapter 설정은 체크섬까지 같았다.

| 측정값 | 룰베이스 | 학습 모델 |
|---|---:|---:|
| canonical/명령 입력 속도 | 6.93 Hz | 입력 6.83 Hz, 출력 4.47 Hz |
| 평균 명령 간격 | 144 ms | 224 ms |
| 200 ms를 넘은 명령 간격 | 0.9% | 50.0% |
| 평균 실차 속도 | 0.639 m/s | 0.641 m/s |
| 명령 시점 영상 나이 | 175 ms | 249 ms |
| 카메라 촬영부터 yaw 반응까지 | 약 371 ms | 약 414 ms |
| 그 지연 동안 이동한 거리 | 약 24 cm | 약 27 cm |

명령에서 yaw 반응까지의 시간은 IMU yaw rate와 명령의 상관관계로 구한
추정치다. 앞바퀴 조향각 센서로 직접 측정한 값은 아니다.

### 룰베이스 원인

- 차선 검출은 정상에 가까웠다. 주행 중 흰선은 100%, 노란선은 99.1%
  존재했다.
- 그러나 곡선 경로 길이는 평균 0.58 m, 최소 0.10 m까지 짧아졌다.
- 기존 지연 예측은 0.10초뿐이어서 실제 약 0.37초의 반응을 따라가지 못했다.
- 차량이 약 24 cm 진행한 뒤 큰 조향을 시작해 직선은 통과하지만 곡선에서
  늦고 급하게 꺾었다.

### 학습 모델 원인

- 약 7 Hz 입력에 다시 `max_inference_rate_hz=7` 제한을 적용했다.
- 프레임 도착 흔들림 때문에 정상 프레임까지 건너뛰어 출력이 4.47 Hz로
  떨어졌다.
- 2프레임 모델이 학습 때의 143 ms 간격 대신 최대 637 ms 떨어진 영상을
  한 쌍으로 받았다.
- 추론 자체는 평균 45 ms였다. 모델 계산보다 인지 지연, 큐, 중복 rate
  limit가 더 큰 문제였다.
- 조향 명령의 52.2%가 절댓값 35 이상이었고 `-40 -> +27 -> -38`과 같은 큰
  방향 전환 뒤 중앙선이 화면 밖으로 나갔다.

상세 수치와 재분석 방법은
[`analysis/real_vehicle_rule_vs_model_20260724/REPORT.md`](../analysis/real_vehicle_rule_vs_model_20260724/REPORT.md)에 있다.

## 2. 이번 코드 변경

### 학습 모델 실행기

파일:

```text
xycar_ws/src/xycar_rl/xycar_rl/policy_runtime_node.py
xycar_ws/src/xycar_rl/launch/real_shadow.launch.py
xycar_ws/src/xycar_rl/launch/lap_time_round02_curve_fix_real.launch.py
```

변경 사항:

1. canonical 영상 subscriber를 Best Effort, `KeepLast(1)`로 설정했다.
2. `max_inference_rate_hz=0`이면 들어오는 canonical 프레임마다 한 번
   추론한다. 인지 노드가 이미 약 7 Hz로 제한하므로 두 번째 7 Hz gate를
   두지 않는다.
3. 지정한 나이보다 오래된 영상은 사용하지 않는다.
4. 2프레임 간격이 0.25초보다 벌어지거나 timestamp가 역행하면 temporal
   history를 초기화한다. 멀리 떨어진 두 프레임을 연속 움직임으로 해석하지
   않는다.
5. `/rl/policy_debug` 끝에 rate-limit 누적 횟수와 stale-frame 누적 횟수를
   추가했다.

`/rl/policy_debug` 배열:

```text
0  base steering norm
1  residual steering norm
2  final steering norm
3  angle command
4  speed command
5  front obstacle distance
6  inference time [ms]
7  image age at command [ms]
8  preview steering
9  preview confidence
10 preview curve hint
11 rate-limited frame count
12 stale frame count
```

### 룰베이스 실차 프로필

파일:

```text
xycar_ws/src/xycar_rule_drive/config/canonical_stanley_pursuit_real.yaml
xycar_ws/src/xycar_rule_drive/launch/canonical_stanley_pursuit_real.launch.py
```

실차 프로필에서만 다음을 적용한다.

```text
extend_fused_path_to_white: true
temporal_path_ego_compensation_enabled: true
control_latency_preview_sec: 0.30
steering_lead_time_sec: 0.08
steering_max_lead_command: 6.0
```

노란 곡선이 짧고 검증된 바깥 흰선이 더 멀리 보이면 흰선으로 경로를
연장한다. 기존의 노란선-흰선 간격 검사와 시간 연속성 검사는 그대로
사용하므로 다른 도로 경계를 무조건 연결하지 않는다.

0.30초는 측정된 전체 지연 약 0.37초보다 보수적인 시작값이다. 실제 결과를
보지 않고 0.37초 이상으로 바로 올리지 않는다. Gazebo 기본 프로필의 기존
0.10초 설정은 유지한다.

## 3. 실차 Copilot이 먼저 할 일

실차 진단 당시 실제 저장소는 `/home/xytron/kookmin_ty`였으며 일부 파일에
미커밋 변경이 있었다. 그 변경을 `reset --hard`나 `checkout --`로 지우지
말아야 한다.

가장 안전한 방법은 새 폴더에 clone하는 것이다.

```bash
cd /home/xytron
git clone --branch simulation \
  git@github.com:steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  kookmin_ty_curve_fix_20260724
cd /home/xytron/kookmin_ty_curve_fix_20260724
git status -sb
```

기존 clone을 갱신해야 한다면 먼저 상태와 diff를 보존한다.

```bash
cd /home/xytron/kookmin_ty
git status -sb
git diff > ~/kookmin_ty_before_curve_fix_20260724.patch
```

필요한 로컬 변경인지 확인하기 전에는 pull, stash, reset을 자동 실행하지
않는다. 깨끗한 상태가 확인됐을 때만:

```bash
git switch simulation
git pull --ff-only origin simulation
```

빌드:

```bash
cd /home/xytron/kookmin_ty_curve_fix_20260724
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash

colcon build --packages-up-to \
  lane_seg_control xycar_rule_drive xycar_rl xycar_final_drive \
  --symlink-install

source install/setup.bash
```

## 4. 공통 사전 점검

카메라, rectifier, VESC/ROS bridge는 차량의 기존 순서로 먼저 실행한다.
이 문서의 주행 launch는 카메라 드라이버나 VESC 드라이버를 대신하지 않는다.

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source /home/xytron/kookmin_ty_curve_fix_20260724/install/setup.bash

ros2 topic hz /wide_camera/rect/image_raw
ros2 topic info /xycar_motor -v
ros2 node list
```

그다음 LR-ASPP 인지를 실행한다.

```bash
ros2 launch lane_seg_control lane_seg_lraspp_canonical_only.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  pipeline_qos_depth:=1 \
  max_output_rate_hz:=7.0
```

별도 터미널에서:

```bash
ros2 topic hz /perception/canonical_road_image
```

정상 기준은 약 `6.5~7.2 Hz`다. canonical이 6.5 Hz 아래라면 주행기를 켜기
전에 인지 부하부터 해결한다.

## 5. 룰베이스 검증

다른 주행 노드가 없는지 확인한 뒤 속도 4 shadow부터 시작한다.

```bash
ros2 node list | grep -E \
  'canonical_stanley_pursuit_driver|rl_policy_inference'

ros2 launch xycar_rule_drive canonical_stanley_pursuit_real.launch.py \
  drive_enabled:=false \
  cruise_speed_command:=4.0 \
  minimum_speed_command:=4.0
```

별도 터미널:

```bash
ros2 topic hz /xycar_motor_shadow
ros2 topic hz /rule_drive/connected_yellow_path
ros2 topic echo /xycar_motor_shadow
```

shadow 조향 방향과 경로가 맞을 때만 바퀴를 띄워 첫 구동을 한다.

```bash
ros2 launch xycar_rule_drive canonical_stanley_pursuit_real.launch.py \
  drive_enabled:=true \
  cruise_speed_command:=4.0 \
  minimum_speed_command:=4.0
```

속도 4에서 곡선 3회 성공 후 `4`를 `6`으로, 속도 6에서 3회 성공 후 `8`로
바꾼다. 한 단계라도 곡선 이탈, 반대 조향, 긴 명령 공백이 나오면 다음
속도로 올리지 않는다.

## 6. 학습 모델 검증

분석했던 실차 모델:

```text
xycar_ws/src/xycar_rl/models/lap_time_speed_only_round02_20260723/
camera_speed_lap_time_speed_only_actor.pth
sha256: 8f6673176631c70e6dbeac2d5830fa282e87f7b2825b36c16beeeee3f8ba70cf
```

전용 launch는 shadow와 속도 상한 4가 기본이다.

```bash
ros2 launch xycar_rl lap_time_round02_curve_fix_real.launch.py \
  drive_enabled:=false \
  deployment_speed_cap:=4.0 \
  device:=cpu
```

별도 터미널:

```bash
ros2 topic hz /rl/policy_motor_shadow
ros2 topic echo /rl/policy_debug
```

확인 기준:

- `/rl/policy_motor_shadow`가 canonical과 비슷한 `6.5~7.2 Hz`
- debug index 6의 추론 시간이 대체로 80 ms 미만
- debug index 7의 영상 나이가 300 ms 미만
- debug index 11이 계속 0
- debug index 12가 계속 증가하지 않음
- 0.25초보다 긴 출력 공백이 반복되지 않음

이 조건을 통과한 뒤에만 바퀴를 띄우고:

```bash
ros2 launch xycar_rl lap_time_round02_curve_fix_real.launch.py \
  drive_enabled:=true \
  deployment_speed_cap:=4.0 \
  device:=cpu
```

룰베이스와 동일하게 4, 6, 8 순서로 각각 곡선 3회 성공을 확인한다. 모델은
학습 당시 조향을 그대로 출력하므로 이번 첫 검증에서는 adaptive steering과
추가 preview steering을 켜지 않는다. 문제가 남으면 출력 필터부터 바꾸지
말고 새 bag으로 실제 주기와 영상 나이를 다시 확인한다.

## 7. 중지와 금지 사항

주행 터미널의 `Ctrl+C`가 첫 번째 중지 수단이다. 노드 종료 시 0 명령을
발행한다. 필요하면 별도 터미널에서 한 번 더:

```bash
ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0]}"
```

다음은 금지한다.

- 룰베이스와 학습 모델을 동시에 `drive_enabled:=true`로 실행
- canonical 입력이 6.5 Hz 미만인데 실차 주행
- shadow 없이 바로 속도 8 주행
- 실차에서 Gazebo 및 `xycar_gazebo_bridge` 실행
- 카메라/BEV 보정값을 이번 제어 수정과 동시에 변경
- 실차 저장소의 미커밋 변경을 확인 없이 삭제

## 8. 다음 실차 기록에 반드시 포함할 토픽

```text
/wide_camera_mjpeg/image_raw/compressed
/recording/canonical_road_image/compressed
/perception/canonical_road_image
/xycar_motor
/xycar_motor_shadow
/rl/policy_motor_shadow
/rl/policy_debug
/rl/policy_status
/rule_drive/diagnostics
/imu
/vehicle/vesc_state
```

각 주행의 폴더명과 메모에 driver, 속도 상한, 실제 사용 commit, 이탈 위치를
남긴다. 룰베이스와 모델은 같은 프로세스에서 동시에 명령하지 말고 각각
별도 bag으로 기록한다.

실차 Copilot의 다음 목표는 “감으로 gain을 변경”하는 것이 아니다. 먼저
새 코드에서 출력 주기가 7 Hz로 회복됐는지, 영상 나이가 줄었는지, 속도
4에서 곡선 이탈이 사라졌는지를 수치로 확인한 뒤 다음 변경을 결정한다.
