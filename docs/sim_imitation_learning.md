# Gazebo 모방학습 데이터 파이프라인

이 저장소는 Team K.A.I. `data_set` 브랜치의 `il_data_tools`를 가져오고
(기준 commit `b9d5965dd16cb79d71e0e8b6b6b590763668094f`),
Gazebo 센서 timestamp와 header가 없는 motor command를 맞추기 위한 시뮬 전용
launch만 추가한다.

## 학습 계약

```text
/image_raw + /scan + /xycar_motor
  -> timestamp 근사 동기화
  -> JPEG + LaserScan NPZ + samples.csv
  -> session 단위 train/val/test 분리
  -> ResNet18 image encoder + LiDAR 1D CNN
  -> steering command 하나 출력
```

모델은 속도나 정지 여부를 출력하지 않는다. 학습 모델을 폐루프 주행에 연결할
때도 속도, 장애물, 인지 신뢰도, emergency stop은 룰베이스가 담당해야 한다.

## 1. 빌드

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
colcon build --packages-select il_data_tools --symlink-install
source install/setup.bash
```

학습에는 PyTorch와 torchvision이 필요하다.

```bash
python3 -c "import torch, torchvision; print(torch.__version__, torch.cuda.is_available())"
```

## 2. 시뮬 데이터 수집

5만 장 자동 수집에는 통합 launch를 사용한다. 실행 전에 따로 띄워 둔 Gazebo,
RViz, 인지, 룰베이스 프로세스를 먼저 종료한다. 통합 launch가 이들을 모두 새로
시작하며, 5만 번째 동기화 샘플을 저장한 뒤 CSV와 metadata를 닫고 전체
시뮬레이션을 종료한다.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch il_data_tools collect_sim_drive_dataset.launch.py
```

기본 제한은 `50000`이며 시험할 때는 다음처럼 바꿀 수 있다.

```bash
ros2 launch il_data_tools collect_sim_drive_dataset.launch.py max_samples:=100
```

Gazebo와 주행 노드를 이미 별도 터미널에서 실행한 상태라 recorder만 붙이려면
다음 launch를 사용한다. 이 경우 5만 장에서 recorder는 자동 종료되지만, 외부
터미널에서 실행한 Gazebo까지 종료할 수는 없다.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch il_data_tools record_sim_drive_dataset.launch.py
```

recorder는 `/xycar_motor`를 구독만 하며 차량 명령을 발행하지 않는다. 기본
저장률은 10 Hz이고, image/scan/motor의 허용 timestamp 차이는 50 ms다.
10 Hz가 유지되면 5만 샘플은 약 83분이지만 실제 소요 시간은 Gazebo 렌더링
속도와 센서 발행률에 따라 길어질 수 있다.

세션은 자동 증가한다.

```text
datasets/il/drive/sim_drive_01
datasets/il/drive/sim_drive_02
datasets/il/drive/sim_drive_03
```

각 세션에는 다음 파일이 생긴다.

```text
images/front/*.jpg
scan/*.npz
samples.csv
metadata.json
```

곡선, 직선, S자, 인지가 흔들린 복구 구간을 여러 세션으로 나눠 수집한다.
train/val/test는 프레임 단위가 아니라 session 단위로 분리되므로 최소 3개,
실제 학습에는 조건이 다른 8~10개 이상의 세션이 좋다.

## 3. 수집 결과 확인

```bash
python3 xycar_ws/src/il_data_tools/scripts/summarize_dataset.py \
  ~/xycar_kookmin_gazebo_track/datasets/il/drive/sim_drive_01
```

각 `metadata.json`에서 다음 값이 0에 가까운지 확인한다.

```text
skipped_missing_scan
skipped_unsynced_scan
dropped_sync_queue_full
dropped_queue_full
writer_errors
```

## 4. 데이터 가공과 학습

CUDA 사용 예시:

```bash
ros2 run il_data_tools train_from_raw_dataset.py \
  --profile drive \
  --dataset-root ~/xycar_kookmin_gazebo_track/datasets/il/drive \
  --processed-dir ~/xycar_kookmin_gazebo_track/datasets/processed/drive \
  --model-output-dir ~/xycar_kookmin_gazebo_track/models/il_policies/drive_resnet18_lidar \
  --epochs 50 \
  --batch-size 64 \
  --num-workers 4 \
  --device cuda \
  --balance-steering
```

CUDA가 없으면 `--device cpu --batch-size 16 --num-workers 2`로 바꾼다.

학습 입력은 다음과 같다.

- 카메라: 원본 화면 비율을 유지해 crop하고 RGB `3x90x160`, `0~1`로 정규화
- LiDAR: scan 개수를 360으로 보간한 거리 채널과 validity mask, `2x360`
- label: 룰베이스가 발행한 `motor_angle / 100`, 범위 `-1~1`
- loss: 큰 조향과 `recovery` 샘플에 더 큰 가중치를 주는 Smooth L1

`max_steer_deg`라는 이름은 남아 있지만 현재 데이터에서는 물리적인 degree가
아니라 Xycar angle command 단위다. 실차와 시뮬 모두 기본 스케일 100을 동일하게
써야 TorchScript 출력 복원이 일치한다.

## 5. 결과 파일

```text
drive_resnet18_lidar_best.pth
drive_resnet18_lidar_scripted.pt
train_config.json
metrics.json
val_predictions.csv
```

`metrics.json`에서 전체 MAE뿐 아니라 직진, 좌회전, 우회전, 급회전 구간별 MAE를
본다. session이 섞이지 않은 test 세션과 Gazebo 폐루프 주행 결과가 최종 기준이다.

## 6. Sim-to-Real 사용 원칙

시뮬과 실차는 동일한 `samples.csv`, 이미지 전처리, LiDAR 전처리, 조향 정규화,
모델 구조를 사용한다. 세션 이름으로 출처를 구분한다.

```text
sim_drive_01, sim_drive_02, ...
drive_01, drive_02, ...
```

시뮬 데이터만 학습하면 조명, 바닥 반사, 카메라 노이즈 차이 때문에 실차에서
바로 성공한다고 보장할 수 없다. 먼저 시뮬에서 pretrain하고, 같은 스키마로 모은
실차 데이터를 섞어 fine-tuning한 뒤 shadow mode와 저속 주행으로 검증한다.

## 7. 현재 남은 단계

`il_data_tools` 원본 범위에는 실차/Gazebo 폐루프 추론 노드가 없다. TorchScript
모델을 제어에 연결할 때는 다음 계약을 지키는 공용 inference 노드가 필요하다.

1. 학습과 정확히 같은 image/LiDAR 전처리
2. 출력 `steer_norm * 100`을 angle command로 복원
3. 속도는 룰베이스가 결정
4. 입력 timeout, 출력 clamp, shadow mode, emergency stop 제공
5. 룰베이스와 학습 노드가 동시에 `/xycar_motor`를 발행하지 않도록 selector 사용

데이터를 충분히 수집하고 첫 모델을 만든 뒤 이 inference/selector를 시뮬과
실차가 공유하도록 구현한다.
