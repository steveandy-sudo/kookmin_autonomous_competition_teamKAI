# IL Training and Model Selection

학습의 목표는 이미지와 선택적으로 phase를 입력받아 steering만 출력하는 모델을 만드는 것입니다. 속도와 안전 판단은 학습하지 않습니다.

## 모델 후보

| 모델 | 특징 | 추천 용도 |
|---|---|---|
| PilotNet | 단순하고 빠름 | cone, overtake phase 첫 baseline |
| MobileNetV3-Small | 가볍고 비교적 강함 | latency가 중요할 때 |
| ResNet18 | 정확도와 안정성의 균형 | drive 첫 추천, Jetson Orin Nano 후보 |
| ViT-Tiny | 실험용 | 데이터가 충분할 때만 실험 |

추천 시작 계획:

- `drive_policy`: ResNet18
- `cone_policy`: PilotNet 먼저, 이후 ResNet18 비교
- `overtake_policy`: PilotNet+phase 먼저, 이후 ResNet18+phase 비교
- ViT-Tiny는 experimental only

## Build + training 통합 실행

실차에서 raw session을 수집한 뒤에는 변환과 학습을 한 번에 실행할 수 있습니다.

Drive:

```bash
ros2 run il_data_tools train_from_raw_dataset.py --profile drive
```

Cone:

```bash
ros2 run il_data_tools train_from_raw_dataset.py --profile cone
```

Overtake:

```bash
ros2 run il_data_tools train_from_raw_dataset.py --profile overtake
```

내부 동작은 다음과 같습니다.

- `drive`: `build_drive_dataset.py` 실행 후 `train_drive_policy.py` 실행
- `cone`: `build_cone_dataset.py` 실행 후 `train_cone_policy.py` 실행
- `overtake`: `build_overtake_dataset.py` 실행 후 `train_overtake_policy.py` 실행

중간 결과인 `train.csv`, `val.csv`, `test.csv`는 그대로 남깁니다. 데이터 변환 문제와 학습 문제를 따로 확인할 수 있게 하기 위해서입니다.

## Drive training

```bash
ros2 run il_data_tools train_drive_policy.py \
  --train-csv ~/xycar_ws/datasets/processed/drive/train.csv \
  --val-csv ~/xycar_ws/datasets/processed/drive/val.csv \
  --output-dir ~/xycar_ws/models/il_policies/drive_resnet18 \
  --model-type resnet18 \
  --epochs 50 \
  --batch-size 128 \
  --lr 1e-4 \
  --max-steer-deg 100 \
  --input-width 160 \
  --input-height 90
```

`train_drive_policy.py`는 기본으로 `--policy-name drive --model-type resnet18`을 넣습니다. 위처럼 다시 명시해도 동작합니다.

## Cone training

```bash
ros2 run il_data_tools train_cone_policy.py \
  --train-csv ~/xycar_ws/datasets/processed/cone/train.csv \
  --val-csv ~/xycar_ws/datasets/processed/cone/val.csv \
  --output-dir ~/xycar_ws/models/il_policies/cone_pilotnet \
  --model-type pilotnet \
  --epochs 50 \
  --batch-size 128 \
  --lr 1e-4 \
  --max-steer-deg 100 \
  --input-width 160 \
  --input-height 90
```

## Overtake training

```bash
ros2 run il_data_tools train_overtake_policy.py \
  --train-csv ~/xycar_ws/datasets/processed/overtake/train.csv \
  --val-csv ~/xycar_ws/datasets/processed/overtake/val.csv \
  --output-dir ~/xycar_ws/models/il_policies/overtake_pilotnet_phase \
  --model-type pilotnet_phase \
  --use-phase \
  --epochs 50 \
  --batch-size 128 \
  --lr 1e-4 \
  --max-steer-deg 100 \
  --input-width 160 \
  --input-height 90
```

`train_overtake_policy.py`는 기본으로 `--policy-name overtake --model-type pilotnet_phase --use-phase`를 넣습니다.

## 출력 파일

학습 output directory에는 보통 다음이 생깁니다.

- `{policy}_{model_type}_best.pth`
- `{policy}_{model_type}_scripted.pt`
- `train_config.json`
- `metrics.json`
- `val_predictions.csv`

예:

- `drive_resnet18_best.pth`
- `drive_resnet18_scripted.pt`

의미:

- `.pth`: 학습 checkpoint입니다. 재학습/재export에 사용합니다.
- `.pt`: TorchScript runtime 모델입니다. rule-based 팀원이 나중에 읽어 steering만 사용할 후보입니다.
- `train_config.json`: 학습 설정 기록입니다.
- `metrics.json`: validation MAE/RMSE 등 metric입니다.
- `val_predictions.csv`: validation sample별 target/pred/error입니다.

`--mark-final`을 주면 `{policy}_policy_scripted.pt`도 복사됩니다. 최종 모델 확정 전에는 남발하지 않는 편이 좋습니다.

## 모델 선택 기준

최종 모델은 validation loss만 보고 고르면 안 됩니다.

함께 봐야 할 것:

- offline MAE/RMSE
- steering bin별 오차
- recovery label 오차
- Jetson Orin Nano p95 latency
- 저속 실차 주행 안정성
- steering oscillation
- penalty 위험 장면에서의 보수성

대회는 penalty가 중요하므로, 약간 느리더라도 흔들림과 위험 조향이 적은 모델이 더 좋은 선택일 수 있습니다.

## 안전 원칙

학습 모델은 steering만 출력합니다. speed, stop/go, emergency stop, traffic light, pedestrian stop, mission switching, parking, final `/xycar_motor` publish는 rule-based 코드가 담당해야 합니다.
