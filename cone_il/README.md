# cone_il: 라바콘 구간 모방학습 패키지

이 패키지는 ROS2 Humble + Xycar 환경에서 라바콘 구간을 모방학습으로 주행하기 위한 최소 구성입니다.

포함 파일:

- `cone_data_recorder`: 수동주행 데이터 수집 노드
- `xycar_keyboard_teleop`: 키보드 수동주행 노드
- `scripts/train_cone_bc.py`: 이미지 -> 조향각 CNN 학습 스크립트
- `cone_ai_driver`: 학습 모델로 라바콘 구간 주행하는 ROS2 노드

기본 토픽:

- 카메라: `/usb_cam/image_raw/front`
- 라이다: `/scan`
- 실제 모터 명령: `/xycar_motor`
- 데이터 라벨 명령: `/cone_il/manual_cmd`

## 1. 설치

```bash
cd ~/xycar_ws/src
unzip /mnt/data/cone_il_package.zip
cd ~/xycar_ws
colcon build --symlink-install --packages-select cone_il
source install/setup.bash
```

PyTorch가 없으면 설치합니다.

```bash
python3 -m pip install torch torchvision pandas tqdm opencv-python
```

## 2. 데이터 수집

터미널 1: 시뮬레이터 실행

터미널 2: 데이터 기록

```bash
source ~/xycar_ws/install/setup.bash
ros2 launch cone_il record_cone_data.launch.py dataset_dir:=~/cone_il_dataset
```

터미널 3: 키보드 수동주행

```bash
source ~/xycar_ws/install/setup.bash
ros2 run cone_il xycar_keyboard_teleop
```

시뮬레이터 창 자체의 WASD가 `/xycar_motor`를 publish하지 않는 환경에서는 아래 OpenCV teleop를 쓰세요.
뜬 작은 창에 포커스를 둔 상태로 WASD를 누르면 `/xycar_motor`가 바뀌고 recorder가 그 값을 저장합니다.

```bash
source ~/xycar_ws/install/setup.bash
ros2 run cone_il xycar_cv_keyboard_teleop
```

`xycar_keyboard_teleop`는 같은 명령을 `/xycar_motor`와 `/cone_il/manual_cmd`에 동시에 보냅니다.
`cone_data_recorder`는 기본적으로 실제 주행 명령 토픽인 `/xycar_motor`를 라벨로 저장합니다.
WASD로 움직이는 노드가 `/xycar_motor`에 `xycar_msgs/XycarMotor`를 publish해야 angle/speed가 저장됩니다.

한 터미널에서 바로 수집/조종을 같이 하려면 아래 명령을 씁니다. 이 모드는 누른 WASD 값을
그대로 모터 명령과 라벨에 함께 사용합니다.

```bash
source ~/xycar_ws/install/setup.bash
ros2 run cone_il cone_keyboard_recorder --ros-args -p dataset_dir:=~/cone_il_dataset
```

키 조작:

- `w`: 속도 증가
- `s`: 속도 감소/후진
- `a`: 왼쪽 조향
- `d`: 오른쪽 조향
- `x`: 속도 0
- `space`: 속도/조향 모두 0
- `q`: 종료

좋은 데이터 수집 방법:

1. 라바콘 사이 중앙을 따라 10회 이상 주행
2. 왼쪽으로 치우친 상태에서 복구하는 주행 5회 이상
3. 오른쪽으로 치우친 상태에서 복구하는 주행 5회 이상
4. 너무 빠르게 달리지 말고 처음에는 `speed=2.5~3.5` 정도로 수집

## 3. 학습

```bash
source ~/xycar_ws/install/setup.bash
cd ~/xycar_ws/src/cone_il
python3 scripts/train_cone_bc.py \
  --dataset-dir ~/cone_il_dataset \
  --output-dir ~/cone_il_model \
  --epochs 30 \
  --batch-size 64
```

학습이 끝나면 다음 파일이 생깁니다.

```text
~/cone_il_model/cone_bc_best.pth
~/cone_il_model/cone_bc_scripted.pt
~/cone_il_model/model_config.json
```

ROS2 추론에는 `cone_bc_scripted.pt`를 씁니다.

## 3.1. 데이터셋 병합

여러 번 수집한 데이터셋을 합쳐서 다시 학습하려면:

```bash
source ~/xycar_ws/install/setup.bash
python3 ~/xycar_ws/src/cone_il/scripts/merge_cone_datasets.py \
  --output-dir ~/cone_il_dataset_merged_new \
  ~/cone_il_dataset_merged \
  ~/cone_il_dataset_new
```

데이터셋 폴더와 학습 중간 산출물(`*.pt`, `*.pth`, `*.onnx`)은 용량이 커서 git에 올리지 않습니다. 주행에 필요한 기본 runtime 모델은 저장소의 `assets/models/`에 포함되어 있습니다.

## 4. 모방학습 주행

```bash
source ~/xycar_ws/install/setup.bash
ros2 launch cone_il drive_cone_ai.launch.py
```

조향 방향이 반대로 꺾이면:

```bash
ros2 launch cone_il drive_cone_ai.launch.py \
  --ros-args -p invert_steering:=true
```

## 5. 기존 rule-based 코드와 같이 쓰는 방법

처음에는 라바콘 구간만 이 노드로 테스트하고, 안정화 후 기존 코드의 `control_once()`에서 라바콘 구간일 때만 모델 출력을 사용하세요.

주의: `cone_ai_driver`와 기존 driver가 동시에 `xycar_motor`에 publish하면 명령이 섞입니다. 동시에 켜지 마세요.
