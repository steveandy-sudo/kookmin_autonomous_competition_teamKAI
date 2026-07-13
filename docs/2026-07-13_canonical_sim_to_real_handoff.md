# 2026-07-13 이후 Canonical Sim-to-Real 작업 기록과 실차 인수인계

## 문서 목적

이 문서는 2026-07-13 12:00 KST 이후 진행한 작업, 실제로 발견한 문제,
canonical BEV 방식으로 전환한 이유, 현재 자동 수집·학습 절차를 한곳에 남긴다.
실차 PC에서 이 저장소를 처음 보는 Codex도 이 문서만 읽고 다음 검증을 이어갈
수 있도록 실행 순서와 수정 지점을 함께 적었다.

## 최종 목표

1. Gazebo와 실차 카메라 영상을 각각 BEV로 변환한다.
2. 두 환경 모두 도로 배경, 흰 경계선, 노란 중앙선을 같은 크기·색·두께로
   정규화한 canonical 영상으로 만든다.
3. canonical 영상과 LiDAR, 룰베이스 조향 명령으로 모방학습 모델을 학습한다.
4. 실차에서도 같은 canonical 토픽을 모델에 넣어 시뮬레이션의 배경·조명·렌즈
   차이가 정책 입력까지 전달되지 않게 한다.
5. 모방학습 폐루프 검증 후 같은 상태 표현을 강화학습 관측으로 확장한다.

## 시간순 작업 기록

### 7월 13일 12:00-13:30: 실차 기준 통합

- 실차 모터 계약을 `/xycar_motor`의
  `std_msgs/msg/Float32MultiArray [angle, speed]`로 확정했다.
- 실차 측정 CSV를 바탕으로 속도 dead zone, 속도 gain, 가속·제동 지연,
  조향 지연과 좌우 조향 곡률을 Gazebo bridge에 반영했다.
- 유효 측정이 부족한 `angle=35~42` 구간은 당시 확보한 값으로 임시 외삽했고,
  이후 7월 13일 고조향 측정값으로 갱신했다.
- 카메라·LiDAR 위치와 실차 토픽, 차량 치수, 조향 포화 범위를 문서와 launch에
  통합했다.
- `13:22`의 `a98cd7b` 커밋에서 실차용 룰베이스, 데이터 수집, 모방학습,
  실차 배포 기반을 한 번에 정리했다.

### 7월 13일 13:30-17:30: 실차 카메라 BEV 문제 수정

- 실차에서 BEV가 심하게 휘어 중앙선과 흰 경계를 안정적으로 검출하지 못했다.
- 실제 `my_rule_driver`의 ROI 사각형 비율과 `/wide_camera/rect/image_raw` 계약을
  확인해 실차용 homography를 분리했다.
- 이미 rectified인 토픽에 fisheye 보정을 다시 적용하지 않도록
  `enable_rectify=false`를 기본값으로 정했다.
- 실차 HSV 범위와 차선 클러스터링 값을 별도 real config에 반영했다.
- 이 수정은 `17:30`의 `b84fc77` 커밋에 기록되어 있다.

### 7월 13일 17:30-18:20: 룰베이스 실차 실행 준비

- RViz에서 source/BEV/차선/목표 경로를 모터 구동 없이 확인하는 shadow launch를
  추가했다(`bc4f6a2`).
- 한쪽 경계가 사라질 때 perception centerline과 노란 중앙선 offset을 사용하는
  fallback을 추가했다(`e1c7963`).
- 실차 조향 부호, 제한, 속도 command 3의 저속 시작 조건과 sensor timeout
  정지를 launch 인자로 노출했다(`b05b396`, `dd79ae2`).
- 시뮬 룰베이스는 최종적으로 lookahead `0.50m`, 직선 speed `4`, 곡선 최저
  speed `3`, 인지 손실 시 마지막 경로 최대 5초 전파를 사용한다.

### 7월 13일 18:20-19:40: raw RGB 모방학습

- Gazebo raw 카메라, LiDAR, 룰베이스 조향을 세션 단위로 저장했다.
- 세션 단위 train/validation/test 분할, 조향 bin 균형, recovery 가중치,
  `ResNet18+LiDAR` 조향 정책과 TorchScript 추론을 구현했다.
- 14개 시뮬 세션 73,553장으로 raw RGB 모델을 학습하고 시뮬레이션에서
  폐루프 시험했다.
- 당시 검증 모델과 실차 runbook은 `19:37`의 `e076fa9`에 게시했다.

### 7월 13일 19:40 이후: 실차 이식에서 확인한 한계

- raw RGB 모델을 실차로 옮기자 시뮬레이션과 달리 조향을 안정적으로 잡지
  못했다.
- 실차에서는 흰 경계가 더 흐리고, 조명·가구·사람·파란 테이프·카메라 노출과
  렌즈 왜곡이 Gazebo와 크게 달랐다.
- raw RGB 모델은 차선뿐 아니라 시뮬 배경과 색감까지 특징으로 사용할 수 있어
  이 차이가 곧바로 조향 오차가 되었다.
- 단순히 실차 HSV만 넓히거나 raw 이미지 augmentation만 늘리는 것으로는 입력
  기하와 배경 차이를 충분히 제거하기 어렵다고 판단했다.

### 7월 14일: canonical BEV 입력으로 전환

- Gazebo와 실차 perception이 같은 토픽
  `/perception/canonical_road_image`를 발행하도록 구현했다.
- canonical 계약은 `256x144`, 전방 `1.2m`, 횡방향 `1.4m`, 배경 gray `36`,
  흰 선 `255`, 노란 선 BGR `(0,220,255)`, 선 두께 `5px`이다.
- raw 색·배경을 버리고 흰 경계와 노란 중앙선의 기하만 모델 입력에 남긴다.
- 한쪽 흰 경계만 보이는 실차 상황을 대비해 canonical 영상에 없는 선을 억지로
  생성하지 않고, 학습 때 30% 확률로 좌우 흰 경계 중 하나를 제거한다.
- 실차 HSV는 `white S<=120, V>=145`, `yellow H=14~45, S>=60,
  V>=80`을 초기값으로 사용한다. 실차 현장에서 canonical 결과를 보고 조정한다.

## 발견한 문제와 현재 해결 상태

| 문제 | 원인 | 현재 처리 |
|---|---|---|
| raw 모델이 실차에서 조향 실패 | 시뮬/실차의 배경·색·노출·왜곡 차이 | canonical BEV 입력으로 전환 |
| 실차 BEV가 휘어짐 | 잘못된 ROI와 rectified 영상의 중복 보정 | 실차 homography 분리, 기본 rectify off |
| 한쪽 흰 선이 자주 안 보임 | 곡선·가림·노출·카메라 시야 | 단일 경계 처리와 30% boundary dropout |
| Gazebo canonical 아래 모서리에 가짜 흰 선 | homography 바깥 검은 영역 경계를 흰 선으로 오검출 | 유효 사각형 mask를 8px 침식하고 최종 mask에도 재적용 |
| canonical 처리로 제어가 늦어질 우려 | BEV/마스크/골격화 연산 | RViz off 측정 시 추가 지연 약 19ms, canonical age 약 26ms |
| 카메라가 30Hz 설정인데 실제 9~11Hz | Gazebo GUI와 고해상도 렌더 부하 | 첫 세션만 GUI, 이후 headless 수집 |
| 한 세션만 학습하면 validation 0장 | 이 프로젝트는 연속 프레임 누수를 막기 위해 세션 단위 분할 | 5천 장씩 10개 독립 세션 사용 |
| 정상 중앙주행만 모으면 이탈 복귀를 못 배움 | 분포에 복구 상태가 부족 | 위치·yaw 교란 후 8초 `recovery` 라벨 수집 |
| 차량이 곡선 안쪽 흰 선에 가까움 | 목표 경로의 횡방향 bias 부족 | 시뮬 경로를 노란 중앙선 쪽으로 총 15cm 이동 |

## 현재 데이터 상태

- `datasets/il_canonical/drive/sim_canonical_drive_01`에는 594장이 있다.
- 이 세션은 아래 모서리 가짜 흰 선 수정 전에 수집했기 때문에 삭제하지는 않되
  이번 학습에서 명시적으로 제외한다.
- 새 파이프라인은 `baseline`, `visual_light`, `visual_dark`, `sensor`,
  `dynamics`, `mixed` preset을 순환한다.
- 총 50,000장을 5,000장씩 10세션으로 저장한다.
- 복구 관리기는 정상 목표 경로에서 좌우 `6/10/15cm`, yaw `4/7/10도`를
  무작위로 주고, 순간이동 후 0.8초는 `bad_data`로 버린 뒤 8초를
  `recovery`로 저장한다.
- recorder는 후보 프레임을 10초간 메모리 지연 버퍼에 둔다. 완전히 차선을 잃어
  룰베이스가 `speed=0` 정지 명령을 내리면 정지 프레임뿐 아니라 그 직전 10초의
  후보 프레임도 `bad_data`로 간주해 디스크에 기록하지 않는다. 따라서 실패로
  이어진 조향과 멈춘 화면이 복구 정답으로 학습되지 않는다.
- 한 번 이상 움직인 차량이 복구 중 1초 이상 `speed=0`이면 해당 시나리오를
  즉시 실패 처리하고 1초 뒤 새 위치로 순간이동한다. 정지 상태에서 다음 30초
  주기를 기다리지 않으므로 5만 장 수집이 불필요하게 멈추지 않는다.
- 각 세션의 이미지·LiDAR 파일, 총 행 수, recovery 비율을 검증한 뒤에만
  학습으로 넘어간다.

## 자동 5만 장 수집·학습·종료

코드와 문서를 먼저 빌드한 뒤 다음 명령을 실행한다. 첫 세션은 Gazebo/RViz로
보이고 나머지는 headless로 진행된다.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
colcon build --packages-select kaiev26_msgs xycar_perception xycar_rule_drive \
  xycar_gazebo_bridge il_data_tools --symlink-install
source install/setup.bash

ros2 run il_data_tools run_canonical_50k_pipeline \
  --project-root "$PWD" \
  --total-samples 50000 \
  --batch-samples 5000 \
  --seed 20260714 \
  --epochs 50 \
  --batch-size 256 \
  --num-workers 8 \
  --device cuda \
  --show-gui-first \
  --publish-model \
  --git-remotes origin,teamkai \
  --poweroff-on-success
```

파이프라인은 다음 조건을 모두 만족할 때만 전원을 끈다.

1. 새 세션이 정확히 10개 생성된다.
2. 새 canonical 이미지와 LiDAR가 정확히 50,000쌍 존재한다.
3. recovery 비율이 10% 이상이다.
4. train/validation/test가 서로 다른 세션으로 생성된다.
5. best validation epoch의 TorchScript 모델이 생성된다.
6. held-out test MAE가 Xycar angle command 기준 8.0 이하이다.
7. held-out `recovery` MAE가 Xycar angle command 기준 12.0 이하이다.
8. 새 canonical 모델·해시·성능 문서가 두 Git remote에 게시된다.

어느 단계든 실패하면 전원을 끄지 않고 로그와 중간 세션을 그대로 남긴다.
수집 세션은 자동 증가 이름을 사용하므로 기존 데이터가 덮어써지지 않는다.

## 실차 PC Codex가 바로 해야 할 일

### 1. 저장소와 브랜치 확인

Gazebo는 실차 PC에서 실행하지 않는다. 아래 `simulation` 브랜치의 최신
canonical 모델 게시 커밋까지 받은 뒤 시작한다.

```bash
cd ~
git clone --branch simulation \
  git@github.com:steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  kookmin_sim_to_real
cd ~/kookmin_sim_to_real

source /opt/ros/humble/setup.bash
colcon build --packages-select kaiev26_msgs xycar_perception il_data_tools \
  --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=7
```

이미 clone되어 있으면 `git switch simulation`, `git pull --ff-only origin
simulation` 후 다시 빌드한다.

### 2. 실차 입력 계약 확인

실차의 카메라, LiDAR, ROS1 VESC와 dynamic bridge를 평소 방식으로 먼저 띄운다.

```bash
ros2 topic hz /wide_camera/rect/image_raw
ros2 topic hz /scan
ros2 topic info /xycar_motor -v
```

필수 계약:

```text
/wide_camera/rect/image_raw  sensor_msgs/msg/Image
/scan                        sensor_msgs/msg/LaserScan
/xycar_motor                 std_msgs/msg/Float32MultiArray
```

절대 토픽을 사용하므로 `ROS_NAMESPACE=xycar` 환경에서도 이름이 바뀌지 않아야
한다. `/xycar_motor`의 실차 subscriber가 없으면 구동 시험을 시작하지 않는다.

### 3. canonical perception만 먼저 실행

```bash
ros2 launch xycar_perception real_canonical_perception.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false
```

다른 터미널에서 세 영상을 확인한다.

```bash
rqt_image_view /perception/canonical_road_image
rqt_image_view /perception/canonical_white_mask
rqt_image_view /perception/canonical_yellow_mask
```

확인 기준:

- 출력 크기는 정확히 `256x144`여야 한다.
- 도로 배경은 gray 36, 흰 경계는 5px 흰색, 중앙선은 5px 노란색이어야 한다.
- 직선에서 노란 중앙선은 대략 영상 중앙에 와야 한다.
- 양쪽 흰 경계 간격과 30cm 중앙 점선 길이가 시뮬 canonical과 비슷해야 한다.
- 한쪽 선이 안 보일 때 반대쪽에 가짜 선을 만들면 안 된다.
- 아래 모서리나 유효 ROI 경계에 가짜 흰 선이 생기면 주행하지 않는다.

실차 장착이 달라졌으면 먼저
`xycar_ws/src/xycar_perception/config/camera_perception_real.yaml`의
`src_*_ratio`, `lateral_m_per_px`, `forward_m_per_px`, HSV 값만 조정한다.
`/wide_camera/rect/image_raw`에 `enable_rectify=true`를 켜 중복 보정하지 않는다.

### 4. 지연과 갱신률 측정

```bash
ros2 topic hz /wide_camera/rect/image_raw
ros2 topic hz /perception/canonical_road_image
ros2 topic delay /perception/canonical_road_image
```

canonical 출력이 지속적으로 10Hz 아래이거나 age가 100ms를 넘으면 모터 구동
전에 원인을 해결한다. 창을 많이 띄운 상태와 닫은 상태를 각각 측정한다.

### 5. 모델 shadow 실행

`docs/canonical_model_latest.md`의 해시를 확인한 뒤 모터가 아닌 shadow 토픽으로
시작한다.

단독 perception 점검을 종료한 뒤 전용 통합 launch를 사용한다. 이 launch가 실차
canonical perception과 모델 추론을 함께 시작하며 기본값은 shadow다.

```bash
MODEL="$(ros2 pkg prefix il_data_tools)/share/il_data_tools/models/drive_canonical_policy_scripted.pt"
sha256sum "$MODEL"

ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  model_path:="$MODEL" \
  source_image_topic:=/wide_camera/rect/image_raw \
  scan_topic:=/scan \
  drive_enabled:=false \
  device:=cpu
```

```bash
ros2 topic echo /il/policy_motor_shadow
rqt_image_view /il/policy_input_image
```

차량을 손으로 좌우로 돌려 보며 오른쪽 곡선에서 오른쪽 명령, 왼쪽 곡선에서
왼쪽 명령이 나오는지 확인한다. 반대면 코드를 임의로 뒤집지 말고
`steering_output_sign:=-1.0`으로 shadow를 다시 검증한다. 크기만 과하면
`max_steer_scale`을 낮추고 기록한다.

### 6. 저속 폐루프

다음 조건을 모두 만족한 뒤에만 실행한다.

- 물리 비상 정지 담당자가 차량 옆에 있다.
- 룰베이스, 키보드, 다른 자율주행 publisher가 모두 종료되어 있다.
- 카메라 또는 LiDAR를 끊으면 shadow speed가 0이 된다.
- 조향 부호가 맞고 출력이 `-42~42` 안에 있다.

```bash
ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  model_path:="$MODEL" \
  source_image_topic:=/wide_camera/rect/image_raw \
  scan_topic:=/scan \
  motor_topic:=/xycar_motor \
  drive_enabled:=true \
  speed_command:=3.0 \
  min_speed_command:=3.0 \
  device:=cpu
```

첫 시험은 바퀴를 띄운 상태, 직선 2~3m, 완만한 곡선, 전체 트랙 순서로 한다.
모델은 조향만 예측하며 속도와 sensor timeout 정지는 계속 규칙 기반이다.

## 실차 결과에 따라 수정할 파일

| 현상 | 먼저 수정할 곳 |
|---|---|
| BEV 위치·폭·원근이 다름 | `camera_perception_real.yaml`의 homography/metric scale |
| 흰 선이 약함 또는 바닥이 흰 선이 됨 | real config의 white HSV/relative threshold |
| 노란 선이 끊김 | real config의 yellow HSV와 morphology |
| 조향 방향 반대 | `real_policy_inference.launch.py` 인자 `steering_output_sign` |
| 조향 진폭이 큼/작음 | 인자 `max_steer_scale` |
| 조향 진동 | 인자 `steering_temporal_alpha`, 먼저 지연도 함께 확인 |
| 실차 canonical 자체는 맞지만 정책이 실패 | 실차 canonical shadow 데이터를 수집해 낮은 learning rate로 fine-tuning |

실차에서 보정한 값과 테스트 결과는 저장소에 커밋한다. 실차 데이터로 추가
학습할 때도 raw 이미지가 아니라 `/perception/canonical_road_image`, `/scan`,
룰베이스 또는 안전운전자 조향을 같은 schema로 수집해야 한다.

## 중요한 한계

- canonical 변환은 색·배경 domain gap을 크게 줄이지만 차선 검출 자체가 틀리면
  그 오차가 그대로 모델에 들어간다.
- offline MAE 통과는 좋은 후보를 뜻할 뿐 실차 안전성을 보장하지 않는다.
- 강화학습으로 넘어가기 전 canonical 갱신률, 실차 지연, 조향 부호와 gain,
  recovery 폐루프 성공률을 먼저 확정해야 한다.
