# IL Command Cheatsheet

사용자가 직접 실행하는 명령만 정리한 요약입니다.

## Build

```bash
cd ~/xycar_ws
colcon build --symlink-install --packages-select il_data_tools
source install/setup.bash
```

## Drive

데이터 수집:

```bash
ros2 launch il_data_tools record_drive_dataset.launch.py session_name:=drive
```

통합 학습:

```bash
ros2 run il_data_tools train_from_raw_dataset.py --profile drive
```

저장 위치:

```text
raw 데이터: ~/xycar_ws/datasets/il/drive
학습 CSV: ~/xycar_ws/datasets/processed/drive
모델: ~/xycar_ws/models/il_policies/drive_resnet18
```

## Cone

데이터 수집:

```bash
ros2 launch il_data_tools record_cone_dataset.launch.py session_name:=cone
```

통합 학습:

```bash
ros2 run il_data_tools train_from_raw_dataset.py --profile cone
```

저장 위치:

```text
raw 데이터: ~/xycar_ws/datasets/il/cone
학습 CSV: ~/xycar_ws/datasets/processed/cone
모델: ~/xycar_ws/models/il_policies/cone_pilotnet
```

## Overtake

데이터 수집:

```bash
ros2 launch il_data_tools record_overtake_dataset.launch.py session_name:=overtake
```

통합 학습:

```bash
ros2 run il_data_tools train_from_raw_dataset.py --profile overtake
```

저장 위치:

```text
raw 데이터: ~/xycar_ws/datasets/il/overtake
학습 CSV: ~/xycar_ws/datasets/processed/overtake
모델: ~/xycar_ws/models/il_policies/overtake_pilotnet_phase
```

## Dry Run

통합 학습이 내부에서 어떤 명령을 실행할지 확인:

```bash
ros2 run il_data_tools train_from_raw_dataset.py --profile drive --dry-run
```

`--profile` 값만 `drive`, `cone`, `overtake` 중 하나로 바꾸면 됩니다.

## 기본 라벨

위 수집 명령만 실행해도 기본 라벨로 저장됩니다.

```text
drive: general_drive
cone: cone_drive
overtake: vehicle_overtake
```

세밀한 라벨 전환이 필요하면 `/il/mission_label` topic으로 라벨을 publish하면 기본 라벨보다 우선 사용됩니다.

## 자동 세션 번호

`session_name:=drive`처럼 기본 이름만 넣으면 저장 폴더는 자동으로 다음 번호를 사용합니다.

```text
첫 번째 실행: ~/xycar_ws/datasets/il/drive/drive_01
두 번째 실행: ~/xycar_ws/datasets/il/drive/drive_02
세 번째 실행: ~/xycar_ws/datasets/il/drive/drive_03
```

`cone`, `overtake`도 같은 방식입니다.
