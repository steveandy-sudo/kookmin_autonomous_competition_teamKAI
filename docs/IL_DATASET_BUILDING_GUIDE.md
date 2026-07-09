# IL Dataset Building Guide

raw recorder session은 바로 학습에 쓰지 않고, `build_*_dataset.py`로 processed CSV를 만들어 사용합니다.

## 입력과 출력

입력:

- `~/xycar_ws/datasets/il/drive/.../samples.csv`
- `~/xycar_ws/datasets/il/cone/.../samples.csv`
- `~/xycar_ws/datasets/il/overtake/.../samples.csv`
- 각 session의 `images/front/*.jpg`
- profile에 따라 `scan/*.npz`

출력:

- `train.csv`
- `val.csv`
- `test.csv`
- `dataset_report.json`

split은 random frame 단위가 아니라 session 단위입니다. 한 session의 frame이 train과 val/test에 동시에 들어가지 않게 하기 위해서입니다.

## Drive dataset

```bash
ros2 run il_data_tools build_drive_dataset.py \
  --dataset-root ~/xycar_ws/datasets/il/drive \
  --output-dir ~/xycar_ws/datasets/processed/drive \
  --max-steer-deg 100
```

drive는 다음 label을 포함합니다.

- `general_drive`
- `lane_drive`
- `hill_drive`
- `shortcut`
- `recovery`

옵션:

- `--balance-steering`: steering bin별 sample 수를 줄여 균형을 맞춤
- `--recovery-oversample-factor N`: train split의 recovery sample을 N배로 늘림
- `--val-ratio`, `--test-ratio`: session split 비율

## Cone dataset

```bash
ros2 run il_data_tools build_cone_dataset.py \
  --dataset-root ~/xycar_ws/datasets/il/cone \
  --output-dir ~/xycar_ws/datasets/processed/cone \
  --max-steer-deg 100
```

cone은 다음 label을 포함합니다.

- `cone_drive`
- `recovery`

`scan_npz_path`가 있으면 processed CSV에도 보존됩니다. 현재 학습 script는 이미지 중심이지만, 나중에 LiDAR-aware model을 비교할 수 있게 경로를 남겨둡니다.

## Overtake dataset

```bash
ros2 run il_data_tools build_overtake_dataset.py \
  --dataset-root ~/xycar_ws/datasets/il/overtake \
  --output-dir ~/xycar_ws/datasets/processed/overtake \
  --max-steer-deg 100 \
  --default-duration-sec 4.0
```

overtake는 다음 label을 포함합니다.

- `vehicle_overtake`
- `overtake_start`
- `overtake_end`
- `recovery`

`overtake_start`와 `overtake_end`가 있으면 그 사이를 기준으로 `phase`가 계산됩니다. 시작은 0.0, 끝은 1.0에 가깝습니다. 둘 중 하나가 없으면 `--default-duration-sec`를 fallback으로 사용합니다.

## Processed CSV columns

drive:

- `image_path`
- `steer_norm`
- `angle_deg`
- `speed`
- `mission_label`
- `session_id`
- `timestamp_ns`

cone:

- drive 컬럼 전체
- `scan_npz_path`

overtake:

- `image_path`
- `steer_norm`
- `angle_deg`
- `speed`
- `phase`
- `mission_label`
- `session_id`
- `timestamp_ns`
- `scan_npz_path`

## 값의 의미

- `angle_deg`: recorder가 저장한 motor angle
- `speed`: recorder가 저장한 motor speed
- `steer_norm`: `angle_deg / max_steer_deg`를 -1.0에서 1.0 사이로 clamp한 값
- `phase`: overtake 전용 진행도
- `session_id`: session 단위 split을 위한 식별자

## 자주 헷갈리는 점

`val.csv`나 `test.csv`가 비어 있을 수 있습니다. 보통 session이 하나뿐일 때 그렇습니다. 실제 학습/평가에는 여러 session이 필요합니다.

좋은 데이터셋은 frame 수만 많은 데이터가 아니라, session과 상황이 다양하고 penalty 위험 장면이 충분히 들어간 데이터입니다.
