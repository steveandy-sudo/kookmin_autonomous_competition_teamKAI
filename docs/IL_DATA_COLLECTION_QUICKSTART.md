# IL 데이터 수집 Quickstart

처음에는 local dry-run으로 파일이 만들어지는지 확인하고, 그 다음 실제 Xycar에서 drive/cone/overtake profile별로 수집하세요.

## A. Build

```bash
cd ~/xycar_ws
colcon build --symlink-install --packages-select il_data_tools
source install/setup.bash
ros2 pkg executables il_data_tools
```

`il_common_recorder`, `il_mission_labeler`, `record_*`, `build_*`, `train_*`, `eval_policy.py`, `benchmark_policy_runtime.py` 등이 보여야 합니다.

## B. xycar_msgs 없이 local dry-run

개발 노트북 테스트 전용입니다. 실차 제어 topic인 `/xycar_motor`를 쓰지 않고 `/test/xycar_motor`를 사용합니다.

Terminal 1:

```bash
ros2 topic pub /il/mission_label std_msgs/msg/String "{data: general_drive}" --rate 5
```

Terminal 2:

```bash
ros2 launch il_data_tools record_drive_dataset.launch.py \
  session_name:=dummy_drive_test \
  output_root:=/tmp/il_test \
  camera_front_topic:=/test/image \
  motor_topic:=/test/xycar_motor \
  motor_msg_type:=float32_multi_array \
  mission_label_topic:=/il/mission_label
```

Terminal 3:

```bash
ros2 run il_data_tools publish_dummy_il_stream.py \
  --image-topic /test/image \
  --motor-topic /test/xycar_motor \
  --label general_drive \
  --duration-sec 12
```

recorder를 멈춘 뒤 확인합니다.

```bash
find /tmp/il_test -maxdepth 6 -type f | sort | head -80
SAMPLES=$(find /tmp/il_test -name "samples.csv" | tail -n 1)
echo "$SAMPLES"
head -5 "$SAMPLES"
```

기대 결과:

- `metadata.json` 생성
- `samples.csv` 생성
- `images/front/*.jpg` 생성
- `samples.csv`에 `motor_angle`, `motor_speed`, `mission_label`, `dataset_profile` 값 기록

## C. Drive dataset collection

drive profile은 기본 주행 안정성을 위한 데이터입니다.

사용 label:

- `general_drive`
- `lane_drive`
- `hill_drive`
- `shortcut`
- `recovery`

실행:

```bash
ros2 launch il_data_tools record_drive_dataset.launch.py session_name:=drive
```

수집 중 label을 바꾸려면 다른 terminal에서:

```bash
ros2 run il_data_tools il_mission_labeler
```

## D. Cone dataset collection

drive/cone/overtake recorder는 기본적으로 `/scan`이 있으면 scan 파일도 저장합니다. cone profile은 라바콘 구간 데이터를 위한 preset입니다.

사용 label:

- `cone_drive`
- `recovery`

실행:

```bash
ros2 launch il_data_tools record_cone_dataset.launch.py session_name:=cone
```

## E. Overtake dataset collection

overtake profile은 차량 추월 흐름을 학습하기 위한 데이터입니다.

사용 label:

- `overtake_start`
- `vehicle_overtake`
- `overtake_end`
- `recovery`

`overtake_start`와 `overtake_end`는 중요합니다. dataset builder가 이 두 timestamp를 기준으로 `phase`를 0.0에서 1.0 사이로 계산합니다. phase는 추월 시작, 통과, 복귀 흐름을 모델에 알려주는 값입니다.

실행:

```bash
ros2 launch il_data_tools record_overtake_dataset.launch.py session_name:=overtake
```

## F. Real Xycar usage

실차에서는 실제 `/xycar_motor`를 구독해야 합니다.

확인:

```bash
ros2 topic info /xycar_motor -v
```

다운로드된 `track_drive`, `app_hough_drive`, `app_sensor_drive` 계열 코드는 `/xycar_motor`를
`std_msgs/msg/Float32MultiArray`로 publish하므로 기본 명령을 그대로 사용합니다.

```bash
ros2 launch il_data_tools record_drive_dataset.launch.py session_name:=drive
```

만약 사용하는 주행 코드가 `xycar_msgs/msg/XycarMotor`를 publish한다면 그때만 다음처럼 바꿉니다.

```bash
ros2 launch il_data_tools record_drive_dataset.launch.py \
  session_name:=drive \
  motor_msg_type:=xycar
```

주의:

- 실차 주행에서 `/test/xycar_motor`를 쓰지 마세요. 그것은 dry-run용입니다.
- recorder는 `/xycar_motor`를 publish하지 않습니다.
- dummy publisher도 기본적으로 `/xycar_motor`를 publish하지 않습니다.
- `/xycar_motor` publisher가 2개 이상이면 위험합니다. `ros2 topic info /xycar_motor -v`로 확인하세요.
