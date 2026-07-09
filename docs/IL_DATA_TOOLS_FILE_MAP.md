# il_data_tools 파일 지도

이 문서는 `il_data_tools`의 주요 파일이 무엇을 하는지 빠르게 찾기 위한 지도입니다.

## Package Core

| 파일 | 역할 | 언제 사용 | 입력 | 출력 | ROS 필요 | `/xycar_motor` 관련 |
|---|---|---|---|---|---:|---|
| `il_data_tools/common_recorder_node.py` | 공통 recorder ROS node | 주행 데이터 수집 | camera, scan, imu, odom, motor topic, mission label | session 폴더, `samples.csv`, 이미지, metadata | 예 | 기본 topic은 `/xycar_motor`, 구독만 함 |
| `il_data_tools/mission_labeler_node.py` | 키보드 mission label publisher | 사람이 구간 label을 바꿀 때 | 키보드 입력 | `/il/mission_label` `std_msgs/String` | 예 | 무관 |
| `il_data_tools/record_schema.py` | session 이름, CSV schema, motor parser, metadata helper | recorder와 builder 공통 유틸 | message, path, row 값 | CSV row, JSON, parsed motor 값 | 아니오 | motor 값을 angle/speed로 해석 |
| `il_data_tools/sync_buffer.py` | 시간 기준 nearest sample buffer | recorder 내부 동기화 | timestamped message | tolerance 안의 가장 가까운 message | 아니오 | motor/camera/scan 동기화에 사용 |
| `il_data_tools/dataset_builder_common.py` | dataset builder 공통 로직 | raw session을 processed CSV로 만들 때 | `samples.csv`, 이미지 경로 | `train.csv`, `val.csv`, `test.csv`, report | 아니오 | 저장된 motor angle/speed를 읽음 |

현재 실제 CLI 학습 진입점은 `scripts/train_policy.py`와 `scripts/train_*_policy.py`입니다.

## Launch Files

| 파일 | 역할 | 언제 사용 | 입력 | 출력 | ROS 필요 | `/xycar_motor` 관련 |
|---|---|---|---|---|---:|---|
| `launch/record_drive_dataset.launch.py` | 일반 주행 수집 preset | drive 데이터 수집 | launch arguments | recorder node 실행 | 예 | `motor_topic` 기본값 `/xycar_motor`, 구독만 |
| `launch/record_cone_dataset.launch.py` | 라바콘 수집 preset | cone 데이터 수집 | launch arguments | recorder node 실행 | 예 | `motor_topic` 기본값 `/xycar_motor`, 구독만 |
| `launch/record_overtake_dataset.launch.py` | 추월 수집 preset | overtake 데이터 수집 | launch arguments | recorder node 실행 | 예 | `motor_topic` 기본값 `/xycar_motor`, 구독만 |

세 launch 파일 모두 `motor_msg_type` 인자를 지원합니다. 기본값은 다운로드된 Xycar 주행 코드에 맞춘 `motor_msg_type:=float32_multi_array`입니다.

## Scripts

| 파일 | 역할 | 언제 사용 | 입력 | 출력 | ROS 필요 | `/xycar_motor` 관련 |
|---|---|---|---|---|---:|---|
| `scripts/check_topics.sh` | ROS topic, `xycar_msgs`, disk 상태 확인 | 수집 전 점검 | ROS graph | PASS/WARN/FAIL 로그 | 예 | `/xycar_motor` info 확인, publisher 수 경고 |
| `scripts/build_drive_dataset.py` | drive raw sessions 처리 | 학습 전 CSV build | drive `samples.csv`들 | processed CSV/report | 아니오 | CSV의 motor 값 사용 |
| `scripts/build_cone_dataset.py` | cone raw sessions 처리 | cone 학습 CSV build | cone `samples.csv`들 | processed CSV/report | 아니오 | CSV의 motor 값 사용 |
| `scripts/build_overtake_dataset.py` | overtake raw sessions 처리, `phase` 생성 | overtake 학습 CSV build | overtake `samples.csv`들 | processed CSV/report | 아니오 | CSV의 motor 값 사용 |
| `scripts/summarize_dataset.py` | raw session 요약 | 수집 세션 품질 확인 | session dir | sample 수, label, angle/speed 통계 | 아니오 | CSV의 motor 값 통계 |
| `scripts/train_drive_policy.py` | drive 기본 학습 wrapper | drive 모델 학습 | train/val CSV | checkpoint, TorchScript, metrics | 아니오 | 무관 |
| `scripts/train_cone_policy.py` | cone 기본 학습 wrapper | cone 모델 학습 | train/val CSV | checkpoint, TorchScript, metrics | 아니오 | 무관 |
| `scripts/train_overtake_policy.py` | overtake phase 모델 학습 wrapper | overtake 모델 학습 | train/val CSV | checkpoint, TorchScript, metrics | 아니오 | 무관 |
| `scripts/train_policy.py` | 공통 학습 CLI | 직접 model/policy를 지정할 때 | train/val CSV | `*_best.pth`, `*_scripted.pt`, `metrics.json`, `val_predictions.csv` | 아니오 | 무관 |
| `scripts/eval_policy.py` | TorchScript offline 평가 | test CSV로 모델 비교 | test CSV, `.pt` | `predictions.csv`, `eval_metrics.json` | 아니오 | 무관 |
| `scripts/visualize_policy_predictions.py` | prediction error 시각화 | 큰 오차 이미지 확인 | `predictions.csv` | error image/report | 아니오 | 무관 |
| `scripts/benchmark_policy_model.py` | TorchScript latency benchmark | Jetson/PC에서 runtime 속도 확인 | `.pt` | latency JSON/log | 아니오 | 무관 |
| `scripts/compare_models.py` | eval/benchmark 결과 합치기 | 최종 모델 후보 비교 | metrics JSON, benchmark JSON | `model_comparison.csv/.md` | 아니오 | 무관 |
| `scripts/export_policy_torchscript.py` | `.pth`를 TorchScript로 export | 재export가 필요할 때 | checkpoint | `.pt` | 아니오 | 무관 |
| `scripts/export_policy_onnx.py` | `.pth`를 ONNX로 export | TensorRT/ONNX 실험 준비 | checkpoint | `.onnx` | 아니오 | 무관 |

## 보조 테스트 스크립트

| 파일 | 역할 |
|---|---|
| `scripts/test_motor_message_parser.py` | ROS 없이 motor parser helper를 간단히 검증합니다. |
