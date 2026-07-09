# il_data_tools

Team K.A.I. imitation learning 데이터 수집용 ROS2 Humble Python 패키지입니다.

이 패키지는 자율주행 로직을 바꾸지 않습니다. `/xycar_motor`를 publish하지 않고, 이미 주행 코드나 사람이 만든 `/xycar_motor` 명령을 구독해서 학습용 데이터로 저장하는 역할만 합니다.

## 직접 쓰는 명령

기본 사용에서는 아래 명령만 직접 실행하면 됩니다. 나머지 build/train helper 스크립트는 `train_from_raw_dataset.py`가 내부에서 호출합니다.

```bash
ros2 launch il_data_tools record_drive_dataset.launch.py session_name:=drive
ros2 launch il_data_tools record_cone_dataset.launch.py session_name:=cone
ros2 launch il_data_tools record_overtake_dataset.launch.py session_name:=overtake
ros2 run il_data_tools train_from_raw_dataset.py --profile drive
```

`--profile`은 `drive`, `cone`, `overtake` 중 하나를 사용합니다.

수집 명령은 같은 `session_name`을 반복해서 써도 자동으로 다음 번호를 붙입니다. 예를 들어 `session_name:=drive`를 반복 실행하면 `drive_01`, `drive_02`, `drive_03` 순서로 저장됩니다.

## 처음 읽을 문서

처음 보는 사람은 [docs/IL_DATA_TOOLS_OVERVIEW.md](../docs/IL_DATA_TOOLS_OVERVIEW.md)부터 읽으세요.

- [docs/IL_DATA_TOOLS_OVERVIEW.md](../docs/IL_DATA_TOOLS_OVERVIEW.md): 전체 목적, 구조, 안전 원칙
- [docs/IL_DATA_TOOLS_FILE_MAP.md](../docs/IL_DATA_TOOLS_FILE_MAP.md): 파일별 역할 지도
- [docs/IL_DATA_COLLECTION_QUICKSTART.md](../docs/IL_DATA_COLLECTION_QUICKSTART.md): build, 토픽 확인, 실차 수집 절차
- [docs/IL_DATASET_BUILDING_GUIDE.md](../docs/IL_DATASET_BUILDING_GUIDE.md): raw session을 processed CSV로 변환하는 방법
- [docs/IL_TRAINING_AND_MODEL_SELECTION.md](../docs/IL_TRAINING_AND_MODEL_SELECTION.md): 학습 명령과 모델 선택 기준
- [docs/IL_EVALUATION_AND_BENCHMARK.md](../docs/IL_EVALUATION_AND_BENCHMARK.md): offline evaluation, 시각화, latency benchmark
- [docs/IL_TROUBLESHOOTING.md](../docs/IL_TROUBLESHOOTING.md): 자주 만나는 문제와 해결법
- [docs/IL_COMMAND_CHEATSHEET.md](../docs/IL_COMMAND_CHEATSHEET.md): 자주 쓰는 명령 요약

## 역할 분리

이 패키지의 책임:

- 전방 카메라 이미지 저장
- LiDAR scan 저장
- 현재 `/xycar_motor`의 조향각과 속도 기록
- 현재 mission label 기록
- 데이터셋 세션 폴더 생성
- `samples.csv` 중심의 학습용 인덱스 생성

이 패키지가 하지 않는 일:

- `/xycar_motor` publish
- rule-based mission 판단
- 신호등 stop/go 판단
- 보행자 정지 판단
- 최종 속도 결정
- 긴급정지 판단
- 주행 코드 수정

최종 `/xycar_motor` publish는 rule-based 주행/미션 코드 한 곳에서만 해야 합니다.

## 설치 위치

기존 repository root가 `track_drive` ROS2 패키지라면, 그 안에 `package.xml`을 또 만들면 안 됩니다.

권장 구조는 `~/xycar_ws/src` 아래 sibling 패키지로 두는 방식입니다.

```bash
/home/xytron/xycar_ws/src/
  track_drive/
  il_data_tools/
```

## 빌드

```bash
cd ~/xycar_ws
colcon build --packages-select il_data_tools
source install/setup.bash
```

필요한 ROS2 의존성이 빠져 있으면 먼저 설치합니다.

```bash
sudo apt update
sudo apt install -y ros-humble-cv-bridge python3-opencv python3-numpy
```

## 실행 전 확인

자이카에서 토픽이 정상적으로 나오는지 확인합니다.

```bash
ros2 run il_data_tools check_topics.sh
```

확인 대상:

- `/image_raw`
- `/scan`
- `/imu`
- `/xycar_motor`
- `~/xycar_ws` 디스크 용량

## motor message type

다운로드된 Xycar workspace의 `track_drive`, `app_hough_drive`, `app_sensor_drive` 계열 코드는
`/xycar_motor`를 `std_msgs/msg/Float32MultiArray` 타입으로 publish합니다.
그래서 이 패키지의 기본 `motor_msg_type`도 `float32_multi_array`로 맞춰져 있습니다.

- recorder 기본값은 `motor_msg_type:=float32_multi_array`입니다.
- recorder는 어떤 설정에서도 `/xycar_motor`를 publish하지 않고 구독만 합니다.
- 사용하는 주행 코드가 `xycar_msgs/msg/XycarMotor`를 publish하는 경우에만 `motor_msg_type:=xycar`를 사용합니다.

## mission label

데이터 수집 중 label을 바꾸고 싶으면 `/il/mission_label`에 `std_msgs/String`을 publish합니다.
기본 수집에서는 launch 파일의 `default_mission_label`이 자동으로 사용됩니다.

예:

```bash
ros2 topic pub /il/mission_label std_msgs/msg/String "{data: recovery}" --rate 5
```

publish topic:

```bash
/il/mission_label
```

publish rate:

```bash
5 Hz
```

키보드 매핑:

| 키 | label |
|---|---|
| `0` | `idle` |
| `1` | `general_drive` |
| `2` | `lane_drive` |
| `3` | `hill_drive` |
| `4` | `shortcut` |
| `5` | `recovery` |
| `6` | `cone_drive` |
| `7` | `vehicle_overtake` |
| `8` | `overtake_start` |
| `9` | `overtake_end` |
| `p` | `parking` |
| `b` | `bad_data` |
| `r` | `red_light_wait` |
| `h` | `pedestrian_wait` |
| `q` | quit |

## recorder

공통 데이터 수집 노드입니다.

실행 파일:

```bash
ros2 run il_data_tools il_common_recorder
```

기본 구독 토픽:

| 데이터 | 기본 토픽 |
|---|---|
| front camera | `/image_raw` |
| LiDAR | `/scan` |
| IMU | `/imu` |
| odom | `/odom` |
| motor command | `/xycar_motor` |
| mission label | `/il/mission_label` |

## launch presets

일반 주행 데이터:

```bash
ros2 launch il_data_tools record_drive_dataset.launch.py
```

저장 label:

- `general_drive`
- `lane_drive`
- `hill_drive`
- `shortcut`
- `recovery`

기본값:

- `save_scan_npz=true`

콘 주행 데이터:

```bash
ros2 launch il_data_tools record_cone_dataset.launch.py
```

저장 label:

- `cone_drive`
- `recovery`

기본값:

- `save_scan_npz=true`

추월 데이터:

```bash
ros2 launch il_data_tools record_overtake_dataset.launch.py
```

저장 label:

- `vehicle_overtake`
- `overtake_start`
- `overtake_end`
- `recovery`

기본값:

- `save_scan_npz=true`

## 저장 구조

기본 저장 위치:

```bash
~/xycar_ws/datasets/il/<profile>/<YYYYMMDD_HHMMSS_session_name>/
```

예:

```bash
~/xycar_ws/datasets/il/drive/20260708_153012_lane_test/
```

세션 내부 구조:

```bash
metadata.json
samples.csv
images/front/*.jpg
scan/*.npz
debug/
debug/imu.csv
debug/odom.csv
README_session.md
```

`samples.csv` 한 줄은 학습에서 하나의 sample index 역할을 합니다.

컬럼:

| 컬럼 | 의미 |
|---|---|
| `timestamp_ns` | sample 기준 시간, nanosecond |
| `front_image_path` | 전방 이미지 상대 경로 |
| `scan_npz_path` | LiDAR npz 상대 경로 |
| `motor_angle` | `/xycar_motor`에서 읽은 조향값 |
| `motor_speed` | `/xycar_motor`에서 읽은 속도값 |
| `mission_label` | 저장 시점 label |
| `dataset_profile` | `drive`, `cone`, `overtake` |
| `session_id` | 세션 ID |

이미지와 scan 파일 경로는 세션 폴더 기준 상대 경로로 저장됩니다. 데이터셋 폴더를 다른 컴퓨터로 옮겨도 `samples.csv`를 그대로 읽기 쉽도록 하기 위해서입니다.

## LiDAR 저장 형식

LiDAR는 `sensor_msgs/LaserScan` 메시지를 `.npz`로 저장합니다.

저장 key:

| key | 의미 |
|---|---|
| `ranges` | 각 angle bin의 거리값 배열 |
| `intensities` | 반사 강도 배열 |
| `angle_min` | 첫 ray 각도, rad |
| `angle_max` | 마지막 ray 각도, rad |
| `angle_increment` | ray 사이 각도 간격, rad |
| `time_increment` | ray 사이 시간 간격 |
| `scan_time` | scan 한 바퀴 또는 한 frame 시간 |
| `range_min` | 센서 최소 유효 거리 |
| `range_max` | 센서 최대 유효 거리 |
| `stamp_ns` | scan timestamp |
| `frame_id` | scan frame id |

학습에서 LiDAR를 쓰지 않는 모델은 `scan_npz_path`를 무시하면 됩니다. 추후 drive/cone/overtake에서 전방 장애물 거리, 빈 공간, 차량 추월 phase 판단 보조 feature로 사용할 수 있습니다.

## IMU/odom 저장

IMU와 odom은 기본적으로 구독만 하고 파일로 저장하지 않습니다. 필요할 때만 아래 파라미터를 켭니다.

```bash
save_imu:=true
save_odom:=true
```

켜면 세션의 `debug/` 폴더 아래에 보조 CSV가 생깁니다.

- `debug/imu.csv`
- `debug/odom.csv`

이 파일들은 학습 기본 index인 `samples.csv` 형식을 바꾸지 않기 위한 보조 기록입니다.

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `output_root` | `~/xycar_ws/datasets/il` | 데이터셋 root |
| `session_name` | `session` | 세션 이름 |
| `dataset_profile` | `drive` | `drive`, `cone`, `overtake` |
| `allowed_labels` | profile 기본값 | 저장 허용 label |
| `save_front_image` | `true` | 전방 이미지 저장 |
| `save_scan_npz` | launch preset 기준 `true` | LiDAR npz 저장 |
| `save_imu` | `false` | `debug/imu.csv` 저장 |
| `save_odom` | `false` | `debug/odom.csv` 저장 |
| `image_format` | `jpg` | 이미지 포맷 |
| `jpeg_quality` | `90` | JPG 품질 |
| `max_save_rate_hz` | `10.0` | 최대 저장 주기 |
| `min_abs_speed_to_save` | `0.0` | 저장할 최소 절대 속도 |
| `save_when_stopped` | `false` | 정지 상태 저장 허용 |
| `require_motor_command` | `true` | motor command 없으면 저장 안 함 |
| `approximate_sync_tolerance_sec` | `0.10` | timestamp 근사 동기화 허용 범위 |
| `flush_every_n_samples` | `20` | CSV flush 주기 |
| `max_session_duration_sec` | `0.0` | 세션 최대 시간, 0이면 제한 없음 |
| `max_disk_usage_gb` | `0.0` | 세션 최대 용량, 0이면 제한 없음 |
| `enable_recording_on_start` | `true` | 시작하자마자 저장 |
| `exclude_bad_data` | `true` | `bad_data` 제외 |
| `exclude_idle` | `true` | `idle` 제외 |
| `exclude_zero_speed` | `false` | 속도 0 제외 |
| `debug_print_period_sec` | `5.0` | 상태 로그 주기 |

## 예시 실행

터미널 1, 필요할 때만 label override:

```bash
source ~/xycar_ws/install/setup.bash
ros2 topic pub /il/mission_label std_msgs/msg/String "{data: recovery}" --rate 5
```

터미널 2:

```bash
source ~/xycar_ws/install/setup.bash
ros2 launch il_data_tools record_drive_dataset.launch.py session_name:=lane_practice_01
```

터미널 3:

사람이 직접 운전하거나 기존 rule-based 주행 코드를 실행합니다.

수집 중 labeler 터미널에서 현재 구간에 맞는 키를 누릅니다.

예:

- 차선 주행: `2`
- 언덕: `3`
- 지름길: `4`
- 복구 주행: `5`
- 실패 구간: `b`

`bad_data`, `idle`은 기본 설정에서 저장되지 않습니다.

## 데이터셋 요약

수집 후 세션 상태를 빠르게 확인합니다.

```bash
ros2 run il_data_tools summarize_dataset.py ~/xycar_ws/datasets/il/drive/20260708_153012_lane_test
```

확인 내용:

- 총 sample 수
- label별 개수
- 조향각 min/max/mean
- 속도 min/max/mean
- 이미지 파일 누락 여부
- scan 파일 누락 여부

## 학습 연결 기준

현재 설계에서는 학습 모델이 steering만 예측합니다.

- 입력: 이미지, 필요하면 LiDAR
- 정답: `motor_angle`
- 참고 정보: `mission_label`, `dataset_profile`
- 사용하지 않을 수 있는 값: `motor_speed`

속도는 rule-based 코드에서 결정하므로, 학습 target으로 쓰지 않아도 됩니다. 다만 데이터 분석과 상황 재현을 위해 `motor_speed`는 기록합니다.

학습용 필터 예:

- drive model: `general_drive`, `lane_drive`, `hill_drive`, `shortcut`
- cone model: `cone_drive`
- overtake model: `vehicle_overtake`, `overtake_start`, `overtake_end`

실패/대기/정지 구간은 기본적으로 학습에서 제외하는 것을 권장합니다.

- `bad_data`
- `idle`
- `red_light_wait`
- `pedestrian_wait`
- `parking`

## 안전 원칙

이 패키지는 기록 장치입니다.

- `/xycar_motor` publish 금지
- rule-based mission manager 수정 금지
- 최종 motor command 결정 금지
- 데이터 수집 중 저장 label을 잘못 누른 구간은 `bad_data`로 표시
- 실차 수집 전 `check_topics.sh`로 토픽과 디스크 용량 확인

## 학습용 CSV 빌더

`il_data_tools`로 모은 원본 세션을 바로 학습에 넣기보다, 먼저 정책별 processed CSV로 변환합니다.

생성되는 파일:

```bash
train.csv
val.csv
test.csv
dataset_report.json
```

분할은 frame 단위가 아니라 session 단위로 합니다. 즉 같은 수집 세션의 frame이 train과 val/test에 섞이지 않습니다.

공통 처리:

- `samples.csv` 읽기
- 이미지 파일 존재 확인
- mission label 필터링
- 기본 제외 label 제거
- 정지 frame 제거
- `steer_norm = motor_angle / max_steer_deg` 계산
- `steer_norm`을 `[-1, 1]`로 clamp
- train/val/test CSV 저장
- 처리 통계를 `dataset_report.json`에 저장

기본 제외 label:

- `bad_data`
- `idle`
- `red_light_wait`
- `pedestrian_wait`
- `parking`

기본 출력 컬럼:

| 컬럼 | 의미 |
|---|---|
| `image_path` | 학습 입력 이미지 절대 경로 |
| `steer_norm` | 정규화 조향값 |
| `angle_deg` | 원본 조향값 |
| `speed` | 원본 속도값 |
| `mission_label` | 수집 당시 label |
| `session_id` | 수집 세션 ID |
| `timestamp_ns` | sample timestamp |

### Drive Dataset

사용 label:

- `general_drive`
- `lane_drive`
- `hill_drive`
- `shortcut`
- `recovery`

제외되는 대표 label:

- `cone_drive`
- `vehicle_overtake`
- `parking`
- `idle`
- `bad_data`
- `red_light_wait`
- `pedestrian_wait`

실행 예:

```bash
python3 scripts/build_drive_dataset.py \
  --dataset-root /home/xytron/xycar_ws/datasets/il/drive \
  --output-dir /home/xytron/xycar_ws/datasets/processed/drive \
  --max-steer-deg 100
```

선택 옵션:

```bash
--balance-steering
--steering-bins 21
--max-bin-samples 500
--recovery-oversample-factor 2
```

`--balance-steering`은 train split에서 조향 분포가 한쪽으로 몰릴 때 bin별 downsample을 합니다. `--recovery-oversample-factor`는 recovery sample을 train split에서 복제해 복구 상황 학습 비중을 높입니다.

### Cone Dataset

사용 label:

- `cone_drive`
- `recovery`

Cone dataset은 현재 image-only 학습 CSV로 쓰되, 나중에 LiDAR 모델을 붙일 수 있도록 `scan_npz_path` 컬럼을 보존합니다.

실행 예:

```bash
python3 scripts/build_cone_dataset.py \
  --dataset-root /home/xytron/xycar_ws/datasets/il/cone \
  --output-dir /home/xytron/xycar_ws/datasets/processed/cone \
  --max-steer-deg 100
```

추가 출력 컬럼:

| 컬럼 | 의미 |
|---|---|
| `scan_npz_path` | LiDAR `.npz` 절대 경로, 없으면 빈 값 |

### Overtake Dataset

사용 label:

- `vehicle_overtake`
- `overtake_start`
- `overtake_end`
- `recovery`

Overtake dataset은 `phase` 컬럼을 추가합니다.

- 같은 session 안에 `overtake_start`와 `overtake_end`가 있으면 그 사이를 `0.0`에서 `1.0`으로 선형 계산합니다.
- start만 있으면 `start + --default-duration-sec`를 end로 봅니다.
- end만 있으면 `end - --default-duration-sec`를 start로 봅니다.
- 둘 다 없으면 첫 `vehicle_overtake` frame부터 `--default-duration-sec` 동안 phase를 계산합니다.

실행 예:

```bash
python3 scripts/build_overtake_dataset.py \
  --dataset-root /home/xytron/xycar_ws/datasets/il/overtake \
  --output-dir /home/xytron/xycar_ws/datasets/processed/overtake \
  --max-steer-deg 100 \
  --default-duration-sec 4.0
```

추가 출력 컬럼:

| 컬럼 | 의미 |
|---|---|
| `phase` | 추월 진행도, `0.0`~`1.0` |
| `scan_npz_path` | LiDAR `.npz` 절대 경로, 없으면 빈 값 |

### 공통 옵션

여러 dataset root나 session을 직접 지정할 수 있습니다.

```bash
python3 scripts/build_drive_dataset.py \
  --dataset-root /home/xytron/xycar_ws/datasets/il/drive \
  --session-dir /home/xytron/xycar_ws/datasets/il/drive/20260708_153012_lane_test \
  --output-dir /home/xytron/xycar_ws/datasets/processed/drive \
  --max-steer-deg 100
```

정지 frame 기준을 바꾸려면:

```bash
--min-abs-speed 1.0
```

정지 frame도 학습에 포함하려면:

```bash
--keep-stopped
```

split 비율과 random seed를 바꾸려면:

```bash
--val-ratio 0.1 --test-ratio 0.1 --seed 2026
```

## 학습 모델 후보

최종 모델은 validation loss 하나만 보고 고르지 않습니다.

최종 선택 기준:

- 조향 오차가 낮을 것
- 저속 실차 주행이 안정적일 것
- Jetson Orin Nano에서 p95 inference latency가 35~50ms 아래일 것
- 심한 steering oscillation이 없을 것
- TorchScript 또는 ONNX 배포가 쉬울 것
- rule-based safety 쪽에 regression을 만들지 않을 것

모든 policy는 steering만 출력합니다. speed, stop/go, emergency stop, 최종 `/xycar_motor` publish는 rule-based 코드가 계속 담당합니다.

학습 스크립트는 PyTorch, torchvision, OpenCV가 필요합니다. 학습 PC나 Jetson 환경에 맞는 PyTorch를 먼저 설치해야 합니다.

지원 `model_type`:

| model_type | 용도 |
|---|---|
| `pilotnet` | 가볍고 빠른 baseline |
| `mobilenet_v3_small` | 경량 CNN 비교 후보 |
| `resnet18` | drive 기본 후보, 성능 비교용 CNN |
| `vit_tiny` | 실험용 후보, 최종 후보로 바로 선택하지 않음 |

정책별 기본값:

| policy | 기본 모델 |
|---|---|
| `drive` | `resnet18` |
| `cone` | `pilotnet` |
| `overtake` | `pilotnet_phase` |

`model_type` 옵션에는 기본 모델 이름만 넣습니다.

```bash
--model-type pilotnet
--model-type mobilenet_v3_small
--model-type resnet18
--model-type vit_tiny
```

추월 정책은 기본 `phase_mode=auto`에서 phase 입력을 자동으로 사용합니다. 그래서 `policy=overtake`, `model_type=pilotnet`이면 실제 variant는 `pilotnet_phase`가 됩니다. 비교 실험으로 `resnet18_phase`를 만들고 싶으면 `--policy overtake --model-type resnet18`을 사용합니다.

### 학습 실행 예시

Drive 기본 후보:

```bash
python3 scripts/train_policy.py \
  --policy drive \
  --train-csv /home/xytron/xycar_ws/datasets/processed/drive/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/drive/val.csv \
  --output-dir /home/xytron/xycar_ws/models/drive_resnet18 \
  --model-type resnet18 \
  --max-steer-deg 100
```

Cone 기본 후보:

```bash
python3 scripts/train_policy.py \
  --policy cone \
  --train-csv /home/xytron/xycar_ws/datasets/processed/cone/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/cone/val.csv \
  --output-dir /home/xytron/xycar_ws/models/cone_pilotnet \
  --model-type pilotnet \
  --max-steer-deg 100
```

Cone 비교 후보:

```bash
python3 scripts/train_policy.py \
  --policy cone \
  --train-csv /home/xytron/xycar_ws/datasets/processed/cone/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/cone/val.csv \
  --output-dir /home/xytron/xycar_ws/models/cone_resnet18 \
  --model-type resnet18 \
  --max-steer-deg 100
```

Overtake 기본 후보:

```bash
python3 scripts/train_policy.py \
  --policy overtake \
  --train-csv /home/xytron/xycar_ws/datasets/processed/overtake/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/overtake/val.csv \
  --output-dir /home/xytron/xycar_ws/models/overtake_pilotnet_phase \
  --model-type pilotnet \
  --max-steer-deg 100
```

Overtake 비교 후보:

```bash
python3 scripts/train_policy.py \
  --policy overtake \
  --train-csv /home/xytron/xycar_ws/datasets/processed/overtake/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/overtake/val.csv \
  --output-dir /home/xytron/xycar_ws/models/overtake_resnet18_phase \
  --model-type resnet18 \
  --max-steer-deg 100
```

학습 결과:

```bash
<policy>_policy_<model_variant>.pth
<policy>_policy_<model_variant>_scripted.pt
<policy>_policy_scripted.pt
<policy>_policy_<model_variant>_report.json
```

`.pth`는 학습 checkpoint이고, `.pt`는 실시간 실행에 넣기 쉬운 TorchScript 모델입니다.

ONNX도 같이 만들려면:

```bash
--export-onnx
```

### Jetson Orin Nano Benchmark

Jetson Orin Nano에서 TorchScript latency를 확인합니다.

```bash
python3 scripts/benchmark_policy_model.py \
  --model /home/xytron/xycar_ws/models/drive_resnet18/drive_policy_scripted.pt \
  --device cuda \
  --iterations 500
```

Phase 입력이 있는 overtake 모델은:

```bash
python3 scripts/benchmark_policy_model.py \
  --model /home/xytron/xycar_ws/models/overtake_pilotnet_phase/overtake_policy_scripted.pt \
  --phase-enabled \
  --device cuda \
  --iterations 500
```

보고서의 `p95_ms`가 35~50ms 목표를 넘으면, validation loss가 좋아도 실차 후보에서 제외하거나 더 가벼운 모델로 비교해야 합니다.

## 오프라인 학습 파이프라인

이 학습 파이프라인은 ROS 없이 CSV와 이미지 파일만으로 실행됩니다. 목적은 `drive`, `cone`, `overtake` 세 steering-only policy를 학습하고, 여러 후보 모델을 같은 기준으로 비교하는 것입니다.

이 단계에서 하지 않는 일:

- `track_drive` 주행 코드 수정
- rule-based mission logic 구현
- `/xycar_motor` publish
- 속도 학습
- 신호등/보행자/긴급정지 판단

모든 학습 모델은 조향값만 출력합니다. 속도, stop/go, emergency stop, sensor timeout, parking, 최종 `/xycar_motor` publish는 rule-based 코드가 담당합니다.

### 모델을 여러 개 비교하는 이유

validation loss가 낮은 모델이 실차에서 항상 좋은 모델은 아닙니다. 실제 최종 후보는 다음을 함께 만족해야 합니다.

- offline validation 조향 오차가 낮음
- Jetson Orin Nano에서 p95 latency가 35~50ms 이하
- 저속 closed-loop 실차 주행이 안정적
- 심한 steering oscillation 없음
- TorchScript 또는 ONNX 배포가 쉬움
- penalty risk와 safety compatibility가 좋음

### 모델 후보

| 후보 | 설명 |
|---|---|
| `pilotnet` | 가장 가볍고 빠른 baseline CNN |
| `mobilenet_v3_small` | 경량 CNN 비교 후보 |
| `resnet18` | drive 기본 후보, 안정적인 CNN baseline |
| `vit_tiny` | experimental only |

ViT-Tiny는 데이터가 많이 필요하고, Jetson latency가 불리할 수 있으며, timm 같은 선택 의존성이 필요합니다. 그래서 기본 후보가 아니라 실험용으로만 둡니다.

phase-conditioned 후보:

- `pilotnet_phase`
- `mobilenet_v3_small_phase`
- `resnet18_phase`
- `vit_tiny_phase`

Overtake policy는 phase 입력을 사용합니다.

- `phase=0.0`: shift-out start
- `phase=0.5`: passing
- `phase=1.0`: return/finish

### 추천 기본 후보

| policy | 추천 시작점 |
|---|---|
| drive | `resnet18` |
| cone | `pilotnet` 먼저, `resnet18` 비교 |
| overtake | `pilotnet_phase` 먼저, `resnet18_phase` 비교 |

### 주요 스크립트

| 파일 | 역할 |
|---|---|
| `scripts/policy_models.py` | PilotNet, MobileNetV3-Small, ResNet18, ViT-Tiny, phase wrapper |
| `scripts/policy_dataset.py` | processed CSV 이미지 Dataset |
| `scripts/train_policy.py` | 통합 학습 스크립트 |
| `scripts/train_drive_policy.py` | drive 기본 wrapper |
| `scripts/train_cone_policy.py` | cone 기본 wrapper |
| `scripts/train_overtake_policy.py` | overtake 기본 wrapper |
| `scripts/eval_policy.py` | TorchScript offline 평가 |
| `scripts/visualize_policy_predictions.py` | prediction debug 이미지/그래프 생성 |
| `scripts/export_policy_torchscript.py` | `.pth`에서 TorchScript `.pt` export |
| `scripts/export_policy_onnx.py` | `.pth`에서 ONNX export |
| `scripts/benchmark_policy_model.py` | Jetson Orin Nano latency benchmark |
| `scripts/compare_models.py` | eval/benchmark 결과 비교 report |

### 학습 예시

Drive ResNet18:

```bash
python3 scripts/train_drive_policy.py \
  --train-csv /home/xytron/xycar_ws/datasets/processed/drive/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/drive/val.csv \
  --output-dir /home/xytron/xycar_ws/models/il_policies/drive_resnet18 \
  --model-type resnet18 \
  --epochs 50 \
  --batch-size 128 \
  --lr 1e-4 \
  --max-steer-deg 100 \
  --input-width 160 \
  --input-height 90
```

Drive MobileNetV3-Small:

```bash
python3 scripts/train_drive_policy.py \
  --train-csv /home/xytron/xycar_ws/datasets/processed/drive/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/drive/val.csv \
  --output-dir /home/xytron/xycar_ws/models/il_policies/drive_mobilenet \
  --model-type mobilenet_v3_small \
  --epochs 50 \
  --batch-size 128 \
  --lr 1e-4 \
  --max-steer-deg 100 \
  --input-width 160 \
  --input-height 90
```

Cone PilotNet:

```bash
python3 scripts/train_cone_policy.py \
  --train-csv /home/xytron/xycar_ws/datasets/processed/cone/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/cone/val.csv \
  --output-dir /home/xytron/xycar_ws/models/il_policies/cone_pilotnet \
  --model-type pilotnet \
  --epochs 50 \
  --batch-size 128 \
  --lr 1e-4 \
  --max-steer-deg 100 \
  --input-width 160 \
  --input-height 90
```

Cone ResNet18:

```bash
python3 scripts/train_cone_policy.py \
  --train-csv /home/xytron/xycar_ws/datasets/processed/cone/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/cone/val.csv \
  --output-dir /home/xytron/xycar_ws/models/il_policies/cone_resnet18 \
  --model-type resnet18 \
  --epochs 50 \
  --batch-size 128 \
  --lr 1e-4 \
  --max-steer-deg 100 \
  --input-width 160 \
  --input-height 90
```

Overtake PilotNet + phase:

```bash
python3 scripts/train_overtake_policy.py \
  --train-csv /home/xytron/xycar_ws/datasets/processed/overtake/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/overtake/val.csv \
  --output-dir /home/xytron/xycar_ws/models/il_policies/overtake_pilotnet_phase \
  --model-type pilotnet_phase \
  --use-phase \
  --epochs 50 \
  --batch-size 128 \
  --lr 1e-4 \
  --max-steer-deg 100 \
  --input-width 160 \
  --input-height 90
```

Overtake ResNet18 + phase:

```bash
python3 scripts/train_overtake_policy.py \
  --train-csv /home/xytron/xycar_ws/datasets/processed/overtake/train.csv \
  --val-csv /home/xytron/xycar_ws/datasets/processed/overtake/val.csv \
  --output-dir /home/xytron/xycar_ws/models/il_policies/overtake_resnet18_phase \
  --model-type resnet18_phase \
  --use-phase \
  --epochs 50 \
  --batch-size 128 \
  --lr 1e-4 \
  --max-steer-deg 100 \
  --input-width 160 \
  --input-height 90
```

학습 결과:

```bash
<policy_name>_<model_type>_best.pth
<policy_name>_<model_type>_scripted.pt
train_config.json
metrics.json
val_predictions.csv
```

최종 후보로 확정한 모델만 generic runtime 파일명으로 복사합니다.

```bash
--mark-final
```

그러면 다음 파일도 같이 만들어집니다.

```bash
drive_policy_scripted.pt
cone_policy_scripted.pt
overtake_policy_scripted.pt
```

### Offline Evaluation

```bash
python3 scripts/eval_policy.py \
  --csv /home/xytron/xycar_ws/datasets/processed/drive/test.csv \
  --model /home/xytron/xycar_ws/models/il_policies/drive_resnet18/drive_resnet18_scripted.pt \
  --model-type resnet18 \
  --output-dir /home/xytron/xycar_ws/eval/drive_resnet18 \
  --input-width 160 \
  --input-height 90 \
  --max-steer-deg 100
```

저장:

- `predictions.csv`
- `eval_metrics.json`

### Prediction Visualization

```bash
python3 scripts/visualize_policy_predictions.py \
  --predictions-csv /home/xytron/xycar_ws/eval/drive_resnet18/predictions.csv \
  --output-dir /home/xytron/xycar_ws/eval/drive_resnet18/debug_images \
  --top-n 50 \
  --random-n 50
```

저장:

- 큰 오차 top N 이미지
- random N 이미지
- target vs prediction plot
- error histogram
- phase-bin error plot, overtake일 때

### TorchScript Export

```bash
python3 scripts/export_policy_torchscript.py \
  --checkpoint /home/xytron/xycar_ws/models/il_policies/drive_resnet18/drive_resnet18_best.pth \
  --output /home/xytron/xycar_ws/models/il_policies/drive_resnet18/drive_resnet18_scripted.pt \
  --model-type resnet18 \
  --input-width 160 \
  --input-height 90
```

Overtake phase 모델:

```bash
python3 scripts/export_policy_torchscript.py \
  --checkpoint /home/xytron/xycar_ws/models/il_policies/overtake_pilotnet_phase/overtake_pilotnet_phase_best.pth \
  --output /home/xytron/xycar_ws/models/il_policies/overtake_pilotnet_phase/overtake_pilotnet_phase_scripted.pt \
  --model-type pilotnet_phase \
  --use-phase \
  --input-width 160 \
  --input-height 90
```

### ONNX Export

```bash
python3 scripts/export_policy_onnx.py \
  --checkpoint /home/xytron/xycar_ws/models/il_policies/drive_resnet18/drive_resnet18_best.pth \
  --output /home/xytron/xycar_ws/models/il_policies/drive_resnet18/drive_resnet18.onnx \
  --model-type resnet18 \
  --input-width 160 \
  --input-height 90
```

ONNX/TensorRT FP16은 TorchScript가 Jetson에서 충분히 빠르지 않을 때 다음 단계로 검토합니다.

### Jetson Orin Nano Benchmark

Jetson에서 직접 실행합니다.

```bash
python3 scripts/benchmark_policy_model.py \
  --model /home/xytron/xycar_ws/models/il_policies/drive_resnet18/drive_resnet18_scripted.pt \
  --device cuda \
  --image-width 160 \
  --image-height 90 \
  --iterations 500
```

Overtake phase 모델:

```bash
python3 scripts/benchmark_policy_model.py \
  --model /home/xytron/xycar_ws/models/il_policies/overtake_pilotnet_phase/overtake_pilotnet_phase_scripted.pt \
  --phase-enabled \
  --device cuda \
  --image-width 160 \
  --image-height 90 \
  --iterations 500
```

Latency 기준:

| p95 latency | 판단 |
|---:|---|
| `<20 ms` | excellent |
| `20~35 ms` | good |
| `35~50 ms` | acceptable |
| `50~80 ms` | risky |
| `>80 ms` | too slow |

### Model Comparison

```bash
python3 scripts/compare_models.py \
  --eval-metrics /home/xytron/xycar_ws/eval/drive_resnet18/eval_metrics.json \
  --eval-metrics /home/xytron/xycar_ws/eval/drive_mobilenet/eval_metrics.json \
  --benchmark-json /home/xytron/xycar_ws/eval/drive_resnet18/benchmark.json \
  --benchmark-json /home/xytron/xycar_ws/eval/drive_mobilenet/benchmark.json \
  --output-dir /home/xytron/xycar_ws/eval/model_comparison
```

저장:

- `model_comparison.csv`
- `model_comparison.md`

score는 낮을수록 좋습니다.

```text
score =
  0.35 * normalized_val_mae
  + 0.25 * normalized_p95_latency
  + 0.20 * normalized_max_error
  + 0.10 * normalized_model_size
  + 0.10 * penalty_for_missing_phase_bins_or_failures
```

이 점수도 자동 최종 결정을 의미하지 않습니다. 마지막 선택은 offline eval, Jetson benchmark, 저속 실차 주행, steering oscillation, safety compatibility, penalty risk를 같이 보고 정합니다.
