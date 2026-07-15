# Team K.A.I. `il_data_tools`

국민대학교 자율주행 경진대회용 자이카(Xycar) 모방학습 데이터 수집·가공·학습 패키지입니다.

현재 학습 대상은 다음 두 가지입니다.

- **Drive**: 일반 차선, 곡선, 언덕, 지름길 내부 및 복구 주행
- **Cone**: 라바콘 구간 및 복구 주행

보행자 회피, 앞차 추월, 신호등과 미션 전환은 rule-based 코드가 담당합니다.
recorder와 학습 코드는 `/xycar_motor`를 발행하지 않으며, 별도의 추론 노드만
`drive_enabled:=true`일 때 최종 모터 명령을 발행합니다.

---

## 1. 주요 기능

### 데이터 수집

- `/image_raw` 카메라 이미지 구독
- `/scan` LiDAR 구독
- `/xycar_motor` 조향·속도 명령 구독
- 이미지와 LiDAR의 timestamp 근사 동기화
- JPEG, LiDAR NPZ, `samples.csv`, `metadata.json` 저장
- writer thread 기반 비동기 디스크 저장
- 디스크 여유 공간 부족 시 안전 정지
- Drive/Cone 세션 번호 자동 증가
- seed 기반 Gazebo 조명·배경·센서·동역학 랜덤화
- 차선 좌우 이탈과 yaw 오차를 이용한 recovery 구간 자동 라벨링
- 여러 독립 환경 세션을 목표 장수까지 연속 실행

Raw RGB 랜덤 환경 5만 장 수집:

```bash
ros2 run il_data_tools collect_randomized_batches \
  --project-root "$PWD" \
  --total-samples 50000 \
  --batch-samples 5000 \
  --seed 2026 \
  --show-gui-first
```

상세 preset과 검증 방법은 저장소 루트의
`docs/domain_randomized_collection.md`를 참고합니다.

sim-to-real용 canonical BEV는 복구 데이터, 정지 직전 10초 폐기, 학습·held-out
평가, 모델 게시와 성공 시 전원 종료를 하나로 묶은 명령을 쓴다. 최신 게시
모델은 2026-07-15에 5천 장씩 6세션, 총 3만 장으로 생성했다.

```bash
ros2 run il_data_tools run_canonical_50k_pipeline \
  --project-root "$PWD" --total-samples 30000 --batch-samples 5000 \
  --seed 2026071500 --epochs 50 --batch-size 256 --num-workers 8 \
  --device cuda --show-gui-first --publish-model \
  --git-remotes origin,teamkai --poweroff-on-success
```

완전 이탈로 룰베이스가 `speed=0`을 내리면 recorder는 정지 프레임과 직전
10초를 `bad_data`로 폐기한다. 이 기능 때문에 후보 샘플은 디스크 기록 전에
10초간 메모리 지연 버퍼에 머문다.

### 데이터 가공

- raw 세션을 `train.csv`, `val.csv`, `test.csv`로 변환
- 같은 주행의 연속 프레임이 섞이지 않도록 세션 단위 분할
- 이미지 또는 LiDAR 파일이 없으면 제외
- 이미지–LiDAR timestamp 차이가 50ms를 넘으면 제외
- 조향값을 `-1~1` 범위로 정규화

### 학습 및 출력

- Drive/Cone 모두 `ResNet18 + LiDAR 1D CNN` 사용
- 모델 출력은 조향값 하나
- validation MAE/RMSE와 조향 구간별 오차 저장
- `.pth` checkpoint와 TorchScript `.pt` 생성
- ONNX export와 latency benchmark 스크립트 제공

---

## 2. 전체 처리 흐름

```text
/image_raw + /scan + /xycar_motor
              ↓
엄격한 이미지–LiDAR 동기화 수집
              ↓
raw session
  images/front/*.jpg
  scan/*.npz
  samples.csv
  metadata.json
              ↓
train_from_raw_dataset.py
              ↓
train.csv / val.csv / test.csv
              ↓
ResNet18 + LiDAR 1D CNN 학습
              ↓
*_best.pth + *_scripted.pt
```

---

## 3. 사용 모델

Drive와 Cone은 같은 구조를 사용하지만 서로 다른 데이터로 별도 학습합니다.

```text
카메라 [B, 3, 90, 160]
        ↓
ResNet18 image encoder
        ↓
image feature
                           ┐
                           ├─ feature fusion → MLP → steering
                           ┘
LiDAR [B, 2, 360]
        ↓
1D CNN LiDAR encoder
        ↓
LiDAR feature
```

LiDAR 입력 채널은 다음과 같습니다.

1. 최대 측정 거리 기준으로 `0~1` 정규화한 거리
2. 정상 측정값 여부를 나타내는 validity mask

TorchScript 실행 계약은 다음과 같습니다.

```python
steer_norm = model(image, lidar)
```

출력 범위는 `-1~1`이며 실제 조향 명령은 학습에 사용한
`max_steer_deg=100`을 곱한 뒤 실차 유효 범위 `-42~42`로 제한합니다.

시뮬에서 학습 완료 모델을 폐루프 주행시키는 통합 실행:

```bash
ros2 launch il_data_tools sim_policy_drive.launch.py
```

이 launch는 Gazebo, 센서 bridge/RViz, canonical perception과 BC 추론을 함께
시작하며 룰베이스 주행 노드는 시작하지 않습니다. 기본 모델은 패키지의
`drive_canonical_policy_scripted.pt`, 기본 입력은
`/perception/canonical_road_image`입니다.

실차에서는 raw 추론 launch를 직접 쓰지 않고
`real_canonical_policy_drive.launch.py`로 실차 canonical perception과 모델을
함께 실행합니다. 기본값은 모터 출력이 차단된 shadow 모드입니다.

실차 shadow 실행:

```bash
ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  source_image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false use_compressed_image:=false \
  scan_topic:=/scan drive_enabled:=false device:=cpu
ros2 topic echo /il/policy_motor_shadow
rqt_image_view /il/policy_input_image
```

`track_run_02` 임시 조립식 트랙에서는 측정된 `0.49m` 중앙선-흰선 간격과
바닥 이음새 필터를 적용하도록 perception profile을 바꾼다. 본선 트랙에서는
이 인자를 사용하지 않는다.

```bash
ros2 launch il_data_tools real_canonical_policy_drive.launch.py \
  perception_launch_file:=real_temp_track_canonical_perception.launch.py \
  source_image_topic:=/wide_camera/rect/image_raw \
  enable_rectify:=false use_compressed_image:=false \
  scan_topic:=/scan drive_enabled:=false device:=cpu
```

`/il/policy_input_image`는 crop, RGB 변환, 160x90 resize를 모두 거친 뒤
모델이 실제로 받는 영상이다. 실차 원본이 정상이어도 이 영상에서 차선이나
소실점이 시뮬 학습 영상과 다르면 먼저 카메라 정합 또는 실차 fine-tuning이
필요하다. `/il/policy_debug`의 뒤쪽 세 값은 원본 width, height, 정규화된
입력 평균 밝기이며 기존 앞 8개 값의 순서는 유지된다.

Shadow의 조향 방향은 맞지만 실차 바퀴 방향만 반대라면
`steering_output_sign:=-1.0`으로 바꾼다. 방향은 맞고 조향량만 일관되게
부족할 때만 `max_steer_scale`을 조금씩 조정한다. 모델의 raw 예측 자체가
틀리면 이 두 값으로 보상하지 말고 카메라 정합과 실차 fine-tuning을 먼저 한다.

### Canonical BEV 기반 Sim-to-Real

`xycar_perception`은 원본 영상의 BEV에서 공통 미터 범위를 잘라 다음 토픽을
발행한다.

```text
/perception/canonical_road_image   256x144 bgr8
/perception/canonical_white_mask  256x144 mono8
/perception/canonical_yellow_mask 256x144 mono8
```

공통 범위는 카메라 렌즈 기준 전방 1.5m, 좌우 1.4m이다. 최종 영상은 배경 BGR
`(36,36,36)`, 흰선 `(255,255,255)`, 노란선 `(0,220,255)`, 선 두께 5px로
고정한다. 실차와 시뮬 모두 0.5m 및 1.5m의 물리 기준점을 homography에 직접
대응시키므로 단순한 화면 resize와 다르다. 카메라에 보이지 않는 약 0~0.5m
근거리 영역은 유효 마스크에서 제외한다.

실차 homography 확인 기준:

```text
가로 해상도: 1.4m / 256px = 약 5.47mm/px
세로 해상도: 1.5m / 144px = 약 10.42mm/px
24mm 차선: 약 4.4px -> canonical에서는 5px
30cm 노란 점선 길이: 약 29px
흰선 중심 간격 82.4cm: 약 151px
```

`rqt_image_view /perception/canonical_road_image`에서 실차와 Gazebo를 각각
정지시켜 위 값을 비교한다. 색과 굵기는 자동으로 고정되지만, 간격과 길이가
다르면 `camera_perception_real.yaml`의 source homography 비율을 실차 기준으로
조정해야 한다. 원본에서 차선이 검출되지 않은 경우에는 canonical 변환이 선을
새로 만들어낼 수 없으므로 카메라 노출과 adaptive threshold를 먼저 조정한다.

Gazebo canonical 데이터 수집:

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch il_data_tools collect_sim_canonical_drive_dataset.launch.py \
  max_samples:=50000
```

출력은 기존 raw 데이터와 분리된
`datasets/il_canonical/drive/sim_canonical_drive_XX`에 PNG로 저장된다.

실차에서는 먼저 실제 카메라 설정으로 perception을 실행한다. 이미 보정된 카메라
토픽이 있으면 기본 명령을 사용한다.

```bash
ros2 launch xycar_perception real_canonical_perception.launch.py
```

기본 실차 프로필은 `/wide_camera/rect/image_raw`, BEV `640x220`, source ROI
`TL/TR/BL/BR=(0.442578,0.688281,0.190625,0.919141)`, y 범위
`0.480781~0.614189`를 사용한다. 0.5m 간격 실측 표식과 2026-07-14 대회 트랙
rosbag을 함께 사용해 전방 1.5m가 canonical 전체 높이에 대응하도록 보정했다.
따라서 사진에 보이는 기존 rectified 영상을 그대로 입력으로 받고, 별도의 카메라
드라이버를 추가로 실행하지 않는다.

차량에 `/image_raw`만 있고 별도 rectified 토픽이 없다면 다음처럼 실행한다.

```bash
ros2 launch xycar_perception real_canonical_perception.launch.py \
  image_topic:=/image_raw enable_rectify:=true
```

그 뒤 다른 터미널에서 canonical 토픽을 recorder 입력으로 사용한다.

```bash
ros2 launch il_data_tools record_drive_dataset.launch.py \
  output_root:=$HOME/xycar_ws/datasets/il_canonical_real \
  session_name:=real_canonical_drive \
  camera_front_topic:=/perception/canonical_road_image \
  image_format:=png
```

Canonical 영상은 raw RGB와 의미가 완전히 다르므로 기존 raw RGB 모델의 `.pth`를
초기값으로 사용하지 않는다. 먼저 canonical 시뮬 데이터로 새 모델을 학습한다.

```bash
ros2 run il_data_tools train_from_raw_dataset.py \
  --profile drive \
  --dataset-root "$PWD/datasets/il_canonical/drive" \
  --processed-dir "$PWD/datasets/processed/drive_canonical" \
  --model-output-dir "$PWD/models/il_policies/drive_canonical" \
  --canonical-input --lane-dropout-probability 0.30 \
  --epochs 50 --batch-size 128 --num-workers 8 --device cuda \
  --balance-steering --mark-final
```

Canonical 학습에서는 원본 RGB용 밝기/감마 증강을 사용하지 않는다. 대신 학습
샘플의 30%에서 왼쪽 또는 오른쪽 흰 경계선 하나를 배경색으로 가린다. 조향 라벨과
노란 중앙선은 유지되므로 실차에서 한쪽 흰선만 보이는 상황을 직접 학습할 수 있다.

학습한 canonical 모델은 시뮬과 실차 모두 다음 image topic으로 실행한다.

```text
image_topic:=/perception/canonical_road_image
```

---

## 4. 요구 환경

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- `rclpy`, `sensor_msgs`, `std_msgs`, `nav_msgs`
- `cv_bridge`, OpenCV, NumPy
- PyTorch, torchvision
- 자이카 기본 `/image_raw`, `/scan`, `/xycar_motor` 토픽

ROS 및 영상 패키지 설치 예시:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  python3-opencv \
  python3-numpy \
  python3-pip
```

학습 환경에는 PyTorch와 torchvision이 필요합니다.

```bash
python3 -m pip install torch torchvision
```

Jetson에서 학습하거나 추론한다면 일반 PyPI 명령 대신 해당 JetPack과 호환되는 NVIDIA PyTorch 패키지를 사용해야 합니다.

설치 확인:

```bash
python3 - <<'PY'
import cv2
import numpy
import torch
import torchvision

print("opencv:", cv2.__version__)
print("numpy:", numpy.__version__)
print("torch:", torch.__version__)
print("torchvision:", torchvision.__version__)
print("cuda:", torch.cuda.is_available())
PY
```

---

## 5. ROS2 workspace 설치

폴더 구조:

```text
~/xycar_ws/
└── src/
    ├── 자이카 기본 패키지
    └── il_data_tools
```

`il_data_tools` 폴더를 `~/xycar_ws/src/`에 복사한 뒤 실행합니다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select il_data_tools
source install/setup.bash
```

설치 확인:

```bash
ros2 pkg executables il_data_tools
ros2 interface show sensor_msgs/msg/LaserScan
```

새 터미널을 열 때마다 다음 명령이 필요합니다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 6. 실차 수집 전 확인

카메라, LiDAR, 모터 명령 토픽을 먼저 실행한 뒤 확인합니다.

```bash
ros2 topic hz /image_raw
ros2 topic hz /scan
ros2 topic info /xycar_motor -v
```

현재 자이카 workspace 기준 기본 모터 메시지는 다음 설정입니다.

```text
std_msgs/msg/Float32MultiArray
data[0] = angle
data[1] = speed
```

실제 토픽 타입이 다르면 launch의 `motor_msg_type`을 차량 설정에 맞게 변경해야 합니다.

데이터 수집 중 `/xycar_motor` publisher는 사람 조종 코드 또는 rule-based 주행 코드 하나만 존재해야 합니다. recorder는 subscriber로만 동작합니다.

---

## 7. Drive 데이터 수집

```bash
cd ~/xycar_ws
source install/setup.bash

ros2 launch il_data_tools record_drive_dataset.launch.py
```

기본값:

```text
카메라: /image_raw
LiDAR: /scan
모터: /xycar_motor
라벨: general_drive
동기화 허용 범위: 50ms
미래 scan 대기: 100ms
최대 저장률: 10Hz
```

세션은 자동으로 증가합니다.

```text
~/xycar_ws/datasets/il/drive/drive_01
~/xycar_ws/datasets/il/drive/drive_02
~/xycar_ws/datasets/il/drive/drive_03
```

일반 차선, 곡선, 언덕, 지름길 내부, 복구 주행을 같은 Drive 세션에 넣어도 됩니다. 별도 labeler를 사용하지 않으면 모두 기본 `general_drive` 라벨로 저장됩니다.

종료는 recorder 터미널에서 `Ctrl+C`를 누릅니다. 종료 시 writer queue를 모두 비운 뒤 CSV와 metadata를 닫습니다.

---

## 8. Cone 데이터 수집

```bash
cd ~/xycar_ws
source install/setup.bash

ros2 launch il_data_tools record_cone_dataset.launch.py
```

기본 라벨은 `cone_drive`입니다.

```text
~/xycar_ws/datasets/il/cone/cone_01
~/xycar_ws/datasets/il/cone/cone_02
~/xycar_ws/datasets/il/cone/cone_03
```

정상 라바콘 주행과 복구 주행을 같은 세션에 함께 넣어도 됩니다.

---

## 9. 동기화 및 저장 조건

Drive와 Cone은 항상 이미지와 LiDAR를 함께 사용하므로 다음 조건을 만족해야 sample이 저장됩니다.

```text
이미지 존재
LiDAR scan 존재
이미지–scan timestamp 차이 ≤ 50ms
모터 명령 존재
허용된 mission label
디스크 여유 공간 충분
writer/sync queue에 여유 공간 존재
```

각 raw session의 `samples.csv`에는 다음이 기록됩니다.

```text
timestamp_ns
image_timestamp_ns
scan_timestamp_ns
scan_time_offset_ms
front_image_path
scan_npz_path
motor_angle
motor_speed
mission_label
dataset_profile
session_id
```

수집 후 `metadata.json`에서 다음 값을 확인합니다.

```text
sample_count
image_count
scan_count
skipped_missing_scan
skipped_unsynced_scan
dropped_sync_queue_full
dropped_queue_full
writer_errors
stop_reason
```

`skipped_unsynced_scan`이 많으면 다음처럼 허용 범위를 임시로 조정해 원인을 확인할 수 있습니다.

```bash
ros2 launch il_data_tools record_drive_dataset.launch.py \
  sync_tolerance_sec:=0.08
```

허용 범위를 넓히기 전에 카메라와 LiDAR timestamp, 주기, QoS 상태를 먼저 점검하는 것을 권장합니다.

---

## 10. Drive 통합 학습

세션이 하나뿐이면 session-based validation을 만들 수 없습니다. 최소 3개, 실제 학습에는 서로 다른 조건의 8~10개 이상 세션을 권장합니다.

자이카 CPU에서 학습하는 예시:

```bash
cd ~/xycar_ws
source install/setup.bash

ros2 run il_data_tools train_from_raw_dataset.py \
  --profile drive \
  --epochs 50 \
  --batch-size 16 \
  --num-workers 2 \
  --device cpu
```

CUDA가 사용 가능한 학습 장비라면:

```bash
ros2 run il_data_tools train_from_raw_dataset.py \
  --profile drive \
  --epochs 50 \
  --batch-size 64 \
  --num-workers 4 \
  --device cuda
```

기본 입출력 위치:

```text
raw:       ~/xycar_ws/datasets/il/drive
processed: ~/xycar_ws/datasets/processed/drive
model:     ~/xycar_ws/models/il_policies/drive_resnet18_lidar
```

---

## 11. Cone 통합 학습

```bash
cd ~/xycar_ws
source install/setup.bash

ros2 run il_data_tools train_from_raw_dataset.py \
  --profile cone \
  --epochs 50 \
  --batch-size 16 \
  --num-workers 2 \
  --device cpu
```

기본 입출력 위치:

```text
raw:       ~/xycar_ws/datasets/il/cone
processed: ~/xycar_ws/datasets/processed/cone
model:     ~/xycar_ws/models/il_policies/cone_resnet18_lidar
```

---

## 12. 학습 결과

각 모델 폴더에 다음 파일이 생성됩니다.

```text
drive_resnet18_lidar_best.pth
drive_resnet18_lidar_scripted.pt
train_config.json
metrics.json
val_predictions.csv
```

Cone은 파일 이름의 `drive`가 `cone`으로 바뀝니다.

- `.pth`: 학습 재개 및 재export용 checkpoint
- `_scripted.pt`: 실차 추론용 TorchScript 모델
- `train_config.json`: 학습 인자와 전처리 계약
- `metrics.json`: validation 결과
- `val_predictions.csv`: 샘플별 정답과 예측 조향값

최종 모델은 validation loss만으로 고르지 말고 다음을 함께 확인해야 합니다.

- 실차 완주율
- 차선 및 라바콘 이탈 횟수
- 조향 진동
- 복구 성공률
- TorchScript p95 latency
- rule-based 안전 로직과의 호환성

### 시뮬 모델을 실차 데이터로 fine-tuning

실차에서는 먼저 수동 또는 검증된 rule-based 주행 명령을 정답으로 사용해
서로 다른 주행 세션을 3개 이상 수집한다. 실패한 학습 모델의 출력은 정답으로
기록하지 않는다. 실차 데이터는 CUDA가 있는 학습 PC로 옮긴 뒤 다음처럼 낮은
학습률로 미세조정한다.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run il_data_tools train_from_raw_dataset.py \
  --profile drive \
  --dataset-root "$PWD/datasets/il/real/drive" \
  --processed-dir "$PWD/datasets/processed/drive_real_ft" \
  --model-output-dir "$PWD/models/il_policies/drive_real_ft" \
  --init-checkpoint \
    "$PWD/models/il_policies/drive_resnet18_lidar_all_20260713/drive_resnet18_lidar_best.pth" \
  --epochs 15 --batch-size 64 --num-workers 4 --device cuda \
  --lr 1e-5 --early-stop-patience 5 --balance-steering --mark-final
```

`--init-checkpoint`에는 재학습 정보가 있는 `.pth` 파일을 사용한다. 실차 실행용
TorchScript `.pt` 파일은 이 옵션에 사용할 수 없다. 모델 종류, 입력 크기,
LiDAR 사용 여부 또는 조향 정규화 범위가 다르면 학습 도구가 시작 전에 중단한다.

---

## 13. 현재 범위와 주의사항

- Drive와 Cone만 학습 모델로 사용합니다.
- Overtake 관련 호환 코드가 일부 남아 있지만 현재 대회 전략에서는 사용하지 않습니다.
- 보행자와 추월은 rule-based로 구현합니다.
- 학습 모델은 속도를 출력하지 않습니다.
- pretrained ResNet18은 전처리 계약 문제를 막기 위해 현재 비활성화되어 있습니다.
- 실제 자이카에서는 먼저 짧은 세션으로 이미지 수, scan 수, timestamp 차이를 확인해야 합니다.
- 자이카에서 학습할 수 있지만 CPU 학습은 오래 걸릴 수 있으므로 가능하면 CUDA 학습 PC를 권장합니다.

---

## 14. 빠른 실행 요약

```bash
# Build
cd ~/xycar_ws
colcon build --symlink-install --packages-select il_data_tools
source install/setup.bash

# Drive record
ros2 launch il_data_tools record_drive_dataset.launch.py

# Cone record
ros2 launch il_data_tools record_cone_dataset.launch.py

# Drive train
ros2 run il_data_tools train_from_raw_dataset.py --profile drive \
  --epochs 50 --batch-size 16 --num-workers 2 --device cpu

# Cone train
ros2 run il_data_tools train_from_raw_dataset.py --profile cone \
  --epochs 50 --batch-size 16 --num-workers 2 --device cpu
```
