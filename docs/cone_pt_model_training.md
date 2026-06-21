# Cone AI `.pt` Model Training Guide

이 문서는 `cone_il` 모방학습 모델을 학습하고, ROS2 주행에서 사용하는 TorchScript `.pt` 파일을 생성하는 과정을 설명합니다.

## 전체 흐름

```text
수동 주행 데이터 수집
  -> labels.csv + images/*.jpg 저장
  -> 필요하면 여러 데이터셋 병합
  -> scripts/train_cone_bc.py로 CNN 학습
  -> cone_bc_best.pth + cone_bc_scripted.pt 생성
  -> cone_bc_scripted_N.pt로 이름 변경
  -> assets/models/에 배치
  -> launch/default model path 수정 또는 model_path로 지정
```

ROS2 주행에는 `.pth`가 아니라 TorchScript로 저장된 `.pt` 파일을 사용합니다.

## 1. 환경 준비

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

현재 저장소 구조에서는 `track_drive`가 colcon 패키지로 잡히고, `cone_il/`은 학습/수집 도구 소스 트리로 함께 들어 있습니다. 따라서 학습 스크립트는 `~/xycar_ws/src/cone_il`에서 직접 실행할 수 있습니다. `ros2 launch cone_il ...`, `ros2 run cone_il ...` 명령은 `cone_il`이 별도 ROS2 패키지로 빌드되어 있거나 기존 install 환경에 들어 있는 경우에 사용합니다.

PyTorch와 학습에 필요한 패키지가 없으면 설치합니다.

```bash
python3 -m pip install torch torchvision tqdm opencv-python
```

## 2. 데이터 수집

시뮬레이터와 ROS-TCP endpoint를 먼저 실행한 뒤 라바콘 구간을 사람이 직접 주행하면서 데이터를 모읍니다.

터미널 1:

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py
```

터미널 2:

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cone_il record_cone_data.launch.py dataset_dir:=~/cone_il_dataset_01
```

터미널 3:

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run cone_il xycar_cv_keyboard_teleop
```

한 터미널에서 수집과 키보드 조종을 같이 하려면 다음 명령을 사용할 수 있습니다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run cone_il cone_keyboard_recorder --ros-args -p dataset_dir:=~/cone_il_dataset_01
```

## 3. 데이터셋 구조

수집이 끝나면 데이터셋 폴더는 다음 형태가 됩니다.

```text
~/cone_il_dataset_01/
├── labels.csv
├── images/
│   ├── 000000.jpg
│   ├── 000001.jpg
│   └── ...
└── scans/
    ├── 000000.npy
    ├── 000001.npy
    └── ...
```

`labels.csv`에는 다음 값이 저장됩니다.

```text
index, stamp, image_path, scan_path, angle, speed, roi_top_ratio, resize_width, resize_height
```

현재 학습 모델은 이미지와 조향각을 사용합니다. LiDAR scan은 나중에 멀티모달 모델로 확장할 수 있도록 같이 저장됩니다.

이미지는 수집 시점에 전처리되어 저장됩니다.

- 원본 카메라 이미지의 위쪽 일부를 잘라냅니다.
- 기본 `roi_top_ratio`는 `0.45`입니다.
- 기본 크기는 `160x90`입니다.
- JPG로 저장됩니다.

학습과 추론은 같은 전처리 조건을 써야 합니다.

## 4. 좋은 데이터 수집 기준

모델 성능은 데이터 품질에 크게 좌우됩니다.

1. 라바콘 사이 중앙을 안정적으로 주행한 데이터를 충분히 모읍니다.
2. 왼쪽으로 치우쳤다가 중앙으로 복구하는 데이터를 따로 모읍니다.
3. 오른쪽으로 치우쳤다가 중앙으로 복구하는 데이터를 따로 모읍니다.
4. 너무 빠른 속도보다 처음에는 `speed=2.5~3.5` 정도의 안정적인 속도가 좋습니다.
5. 같은 방향 조향만 많은 데이터는 피합니다.
6. 정지 상태나 의미 없는 0 명령은 학습에 방해가 될 수 있습니다.

## 5. 여러 데이터셋 병합

여러 번 수집한 데이터셋을 하나로 합칠 때는 `merge_cone_datasets.py`를 사용합니다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 ~/xycar_ws/src/cone_il/scripts/merge_cone_datasets.py \
  --output-dir ~/cone_il_dataset_merged \
  ~/cone_il_dataset_01 \
  ~/cone_il_dataset_02 \
  ~/cone_il_dataset_03
```

출력 폴더가 이미 존재하면 스크립트가 중단됩니다. 기존 데이터가 덮어써지는 것을 막기 위한 동작입니다.

## 6. 모델 학습

학습은 `cone_il/scripts/train_cone_bc.py`로 진행합니다.

```bash
cd ~/xycar_ws/src/cone_il
source ~/xycar_ws/install/setup.bash

python3 scripts/train_cone_bc.py \
  --dataset-dir ~/cone_il_dataset_merged \
  --output-dir ~/cone_il_model_run01 \
  --epochs 30 \
  --batch-size 64 \
  --lr 1e-3 \
  --weight-decay 1e-4 \
  --val-ratio 0.15 \
  --max-steer-deg 50.0
```

GPU가 있으면 자동으로 CUDA를 사용합니다. CPU로 강제로 학습하려면 `--cpu`를 붙입니다.

```bash
python3 scripts/train_cone_bc.py \
  --dataset-dir ~/cone_il_dataset_merged \
  --output-dir ~/cone_il_model_cpu_test \
  --epochs 30 \
  --batch-size 64 \
  --cpu
```

매 epoch마다 모델을 남기고 싶으면 다음 옵션을 사용합니다.

```bash
python3 scripts/train_cone_bc.py \
  --dataset-dir ~/cone_il_dataset_merged \
  --output-dir ~/cone_il_model_epoch_check \
  --epochs 30 \
  --batch-size 64 \
  --save-every-n-epochs 1
```

## 7. 학습 결과 파일

학습이 끝나면 output 폴더에 다음 파일이 생성됩니다.

```text
~/cone_il_model_run01/
├── cone_bc_best.pth
├── cone_bc_scripted.pt
├── model_config.json
├── training_history.json
└── checkpoints/
    ├── cone_bc_epoch_001.pth
    ├── cone_bc_epoch_001.pt
    └── ...
```

각 파일의 의미는 다음과 같습니다.

- `cone_bc_best.pth`: 가장 낮은 validation loss를 기록한 PyTorch checkpoint입니다. 재학습이나 분석용입니다.
- `cone_bc_scripted.pt`: ROS2 추론에 바로 쓰는 TorchScript 모델입니다.
- `model_config.json`: 학습 당시 설정과 best epoch 정보입니다.
- `training_history.json`: epoch별 `train_loss`, `val_loss`, `val_mae_deg` 기록입니다.
- `checkpoints/*.pt`: 특정 epoch의 TorchScript 모델입니다.
- `checkpoints/*.pth`: 특정 epoch의 PyTorch checkpoint입니다.

## 8. 모델 구조

현재 모델은 `ConeBCNet`입니다.

```text
입력: RGB 이미지 tensor, shape = [B, 3, 90, 160], 값 범위 [0, 1]
출력: 정규화된 조향값, shape = [B], 값 범위 [-1, 1]
```

내부 구조는 작은 CNN입니다.

```text
Conv 3->24
Conv 24->36
Conv 36->48
Conv 48->64
AdaptiveAvgPool
Linear 64->64
Linear 64->1
Tanh
```

추론 시에는 모델 출력에 `max_steer_deg`를 곱해서 실제 조향각으로 변환합니다.

```text
steer_deg = model_output * max_steer_deg
```

예를 들어 `max_steer_deg=50.0`이고 모델 출력이 `0.4`라면 실제 조향각은 약 `20도`입니다.

## 9. `.pt` 파일 생성 방식

학습 스크립트는 validation loss가 가장 좋아질 때마다 다음 과정을 수행합니다.

```python
example = torch.zeros(1, 3, resize_height, resize_width, device=device)
scripted = torch.jit.trace(model, example)
scripted.save(str(scripted_path))
```

즉, `cone_bc_scripted.pt`는 일반 checkpoint가 아니라 TorchScript 파일입니다. 그래서 ROS2 추론 노드는 다음처럼 바로 로드합니다.

```python
torch.jit.load(model_path, map_location=device)
```

## 10. 새 모델을 주행에 적용

학습 결과를 새 버전으로 적용하려면 `cone_bc_scripted.pt`를 이름을 바꿔 저장소의 runtime 모델 폴더에 넣습니다.

예시:

```bash
cd ~/xycar_ws/src
cp ~/cone_il_model_run01/cone_bc_scripted.pt assets/models/cone_bc_scripted_3.pt
```

기본 모델로 쓰려면 다음 파일들의 모델명을 맞춥니다.

```text
track_drive/package_paths.py
launch/ai_direct_hybrid.launch.py
launch/drive_test_debug.launch.py
cone_il/launch/drive_cone_ai.launch.py
.gitignore
```

현재 구조에서는 `assets/models/cone_bc_scripted_3.pt`가 기본 cone AI 모델입니다.

빌드합니다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select track_drive
source install/setup.bash
```

launch 기본값이 새 모델을 가리키는지 확인합니다.

```bash
ros2 launch track_drive ai_direct_hybrid.launch.py --show-args | grep -A2 model_path
```

## 11. 모델 파일 검증

TorchScript 모델이 깨지지 않았는지 확인합니다.

```bash
cd ~/xycar_ws/src
python3 - <<'PY'
import torch

path = 'assets/models/cone_bc_scripted_3.pt'
model = torch.jit.load(path, map_location='cpu')
model.eval()
print('torchscript_ok', path)
PY
```

간단히 더미 입력으로 forward도 확인할 수 있습니다.

```bash
cd ~/xycar_ws/src
python3 - <<'PY'
import torch

path = 'assets/models/cone_bc_scripted_3.pt'
model = torch.jit.load(path, map_location='cpu')
model.eval()
x = torch.zeros(1, 3, 90, 160)
with torch.no_grad():
    y = model(x)
print('output=', y)
PY
```

## 12. 주행 테스트

단독 cone AI 주행 테스트:

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cone_il drive_cone_ai.launch.py
```

통합 주행:

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch track_drive ai_direct_hybrid.launch.py
```

기본값 대신 특정 모델을 직접 지정하려면 다음처럼 실행합니다.

```bash
ros2 launch track_drive ai_direct_hybrid.launch.py \
  model_path:=/home/xytron/xycar_ws/src/assets/models/cone_bc_scripted_3.pt
```

## 13. 자주 보는 문제

### 모델이 한쪽으로만 꺾임

- 왼쪽/오른쪽 복구 데이터 균형이 부족할 수 있습니다.
- `invert_steering` 설정이 반대일 수 있습니다.
- 수집 때 저장된 `angle` 부호와 실제 조향 방향을 확인합니다.

### 조향이 너무 흔들림

- 데이터에 급격한 키 입력이 많을 수 있습니다.
- 추론 노드의 `steer_smoothing` 값을 조금 올립니다.
- 학습 데이터에서 너무 빠른 속도의 샘플을 줄입니다.

### 라바콘이 없는데도 계속 주행함

- `require_orange_gate`를 켜거나 `orange_ratio_threshold`를 조정합니다.
- 단, 색 기반 gate는 조명과 시뮬레이터 색상에 민감합니다.

### 학습 loss는 낮은데 실제 주행이 안 좋음

- validation set이 실제 어려운 상황을 포함하지 않을 수 있습니다.
- 중앙 주행 데이터만 많고 복구 데이터가 적을 수 있습니다.
- 수집 환경과 테스트 환경의 카메라 각도, ROI, 속도가 달라졌는지 확인합니다.

## 14. 추천 버전 관리 규칙

- 원본 학습 폴더는 `~/cone_il_model_날짜_설명`처럼 보관합니다.
- 실제 주행에 쓸 모델만 `assets/models/cone_bc_scripted_N.pt`로 복사합니다.
- 새 모델을 기본값으로 바꿀 때는 코드 경로, launch 경로, `.gitignore` 예외를 함께 수정합니다.
- 커밋 전에는 TorchScript load, `colcon build`, launch 기본값 확인을 수행합니다.
