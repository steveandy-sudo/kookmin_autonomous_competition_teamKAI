# Domain-randomized simulation collection

이 파이프라인은 최종 맵의 차선 형상과 차량 실측 기준은 보존하면서 학습에
필요한 시각·센서·동역학 변화와 차선 이탈 복구 상태를 자동 생성합니다.
`worlds/kookmin_xycar_track_final.sdf`는 수정하지 않습니다.

## 적용되는 변화

| preset | 주요 변화 |
|---|---|
| `baseline` | 원본 환경, 차량 이탈·복구만 적용 |
| `visual_light` | 밝은 조명, 배경 색과 가구 위치의 작은 변화 |
| `visual_dark` | 낮은 조도, 색조·그림자 변화 |
| `sensor` | 카메라 위치·각도·노이즈, LiDAR 거리 노이즈 |
| `dynamics` | 마찰, 가속도, 속도 gain, 조향·속도 지연 |
| `mixed` | 위 변화를 실차 주변의 제한된 범위에서 함께 적용 |

흰색 경계선과 노란 중앙선 모델의 pose는 모든 preset에서 변경하지 않습니다.
각 환경은 seed로 결정되므로 같은 preset과 seed는 다시 생성할 수 있습니다.

## 복구 데이터 생성

시나리오 관리자는 트랙 전체의 직선·좌우 곡선·S자에서 다음 오차를
무작위로 만듭니다.

- 기준 경로 좌우 `6, 10, 15cm`
- 기준 진행 방향 대비 `4, 7, 10도`
- 기본 30초 간격
- 순간이동 직후 기본 0.8초: `bad_data`로 라벨링하여 저장 제외
- 이후 기본 8초: `recovery`
- 나머지 정상 주행: `general_drive`

복구 조향 라벨은 현재 카메라 룰베이스가 생성합니다. 첫 GUI 점검에서 흰색
경계를 완전히 놓치는 극단 자세가 보이면 lateral/yaw 범위를 넓히지 말고 해당
구간을 먼저 수정해야 합니다.

## 빌드

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash

colcon build --packages-select \
  kaiev26_msgs xycar_perception xycar_rule_drive \
  xycar_gazebo_bridge il_data_tools \
  --symlink-install
source install/setup.bash
```

## 1. GUI로 먼저 확인

새 데이터 500장을 `mixed/seed 2026` 환경에서 수집합니다.

```bash
ros2 launch il_data_tools collect_randomized_sim_dataset.launch.py \
  project_root:="$PWD" \
  session_name:=sim_randomized_preview \
  max_samples:=500 \
  preset:=mixed \
  seed:=2026 \
  show_gui:=true
```

확인할 항목:

- 차량이 차선 위 여러 위치로 이동하는가
- 순간이동 직후 차량이 올바른 방향을 향하는가
- 룰베이스가 노란선과 흰선 사이로 복귀하는가
- 조명과 배경 변화가 실제 실내 환경 범위를 벗어나지 않는가
- 카메라와 LiDAR가 끊기지 않는가

500장에 도달하면 recorder, 룰베이스, bridge와 Gazebo가 함께 종료됩니다.

## 2. 새 데이터 5만 장 자동 수집

기본값은 5천 장씩 10개 독립 세션입니다. 첫 세션만 GUI로 보고 나머지는
headless로 실행합니다.

```bash
cd ~/xycar_kookmin_gazebo_track
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run il_data_tools collect_randomized_batches \
  --project-root "$PWD" \
  --total-samples 50000 \
  --batch-samples 5000 \
  --seed 2026 \
  --show-gui-first
```

GUI 확인이 이미 끝났다면 `--show-gui-first`를 생략하면 전 세션이 headless로
실행됩니다. 기존 `sim_drive_01` 등의 세션은 삭제하거나 덮어쓰지 않습니다.

생성 예시:

```text
datasets/il/drive/sim_baseline_s2026_01
datasets/il/drive/sim_visual_light_s2027_01
datasets/il/drive/sim_visual_dark_s2028_01
datasets/il/drive/sim_sensor_s2029_01
datasets/il/drive/sim_dynamics_s2030_01
datasets/il/drive/sim_mixed_s2031_01
```

## 저장되는 환경 정보

각 세션의 `metadata.json` 안 `run_manifest`에 다음 값이 들어갑니다.

- preset과 seed
- 원본 world SHA-256
- 조명과 배경 RGB
- 이동한 배경 객체와 pose 변화
- 카메라 pose 및 영상 노이즈
- LiDAR 거리 노이즈
- 마찰·가속도 변화
- 속도 gain과 조향·속도 지연
- scenario event log 위치

생성된 임시 world와 이벤트 로그는 아래에 있습니다.

```text
generated/domain_randomization/
```

## 수집 결과 확인

```bash
python3 - <<'PY'
import csv
from collections import Counter
from pathlib import Path

root = Path('datasets/il/drive')
for session in sorted(root.glob('sim_*')):
    csv_path = session / 'samples.csv'
    if not csv_path.exists():
        continue
    rows = list(csv.DictReader(csv_path.open()))
    labels = Counter(row['mission_label'] for row in rows)
    print(session.name, len(rows), dict(labels))
PY
```

기본 설정에서는 세션 전체의 약 25~30%가 `recovery`가 되는 것을 목표로
합니다. 실제 비율은 카메라 저장률과 scenario 시점에 따라 달라집니다.

## 재학습

기존 데이터와 새 랜덤 세션을 모두 세션 단위로 다시 분할해 학습합니다.

```bash
ros2 run il_data_tools train_from_raw_dataset.py \
  --profile drive \
  --dataset-root "$PWD/datasets/il/drive" \
  --processed-dir "$PWD/datasets/processed/drive_randomized" \
  --model-output-dir "$PWD/models/il_policies/drive_randomized" \
  --epochs 50 \
  --batch-size 256 \
  --num-workers 8 \
  --device cuda \
  --balance-steering \
  --recovery-oversample-factor 2 \
  --mark-final
```

validation과 test에는 학습에 사용하지 않은 seed 세션을 남겨 두는 것이
중요합니다. 같은 세션의 연속 프레임을 train과 validation에 나누면 실제보다
좋은 성능으로 보일 수 있습니다.
