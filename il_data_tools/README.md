# il_data_tools

`il_data_tools`는 2026 국민대학교 자율주행 경진대회 결선 연습에서 Team K.A.I.가 자이카 ROS2 Y모델의 모방학습 데이터를 안전하게 모으기 위한 ROS2 Humble 패키지입니다.

이 패키지는 기본적으로 기록만 합니다. `/xycar_motor`에 명령을 publish하지 않으므로 기존 `track_drive`, `my_motor`, 수동조종, Docker 기반 VESC 브리지와 충돌하지 않도록 설계했습니다.

## 왜 이미지 + 모터 명령 + 미션 라벨이 필요한가

모방학습 데이터는 "그 상황에서 사람이 어떤 조향/속도 명령을 냈는지"를 배워야 합니다. 그래서 기본 샘플은 앞 카메라 이미지, 같은 시각의 `/xycar_motor` 명령, 현재 미션 라벨, LiDAR scan을 함께 가집니다.

미션 라벨은 나중에 데이터를 거르는 데 중요합니다. 예를 들어 차선주행, 콘주행, 언덕, 보행자 회피, 차량 추종/추월, 신호등 출발, 경로 선택, shortcut, parking, recovery 데이터를 섞어 하나의 모델에 넣을지, 미션별 정책으로 나눌지 결정할 수 있습니다.

`timestamp_ns`는 센서와 명령을 같은 순간 기준으로 묶기 위해 필요합니다. 카메라, LiDAR, 모터 명령은 들어오는 시간이 조금씩 다르므로 timestamp가 있어야 "이 이미지와 가장 가까운 조향각/속도/scan"을 찾을 수 있고, 나중에 raw bag 재변환이나 데이터 품질 검사도 할 수 있습니다.

속도는 꼭 학습 target으로 쓰지 않아도 됩니다. 실차에서는 속도를 rule-based로 정하는 것이 더 안전할 수 있습니다. 그래도 속도 값은 거의 용량을 쓰지 않으므로 저장해두는 것을 추천합니다. 나중에 정지 프레임 제거, 저속/고속 구간 분리, 사람이 어떤 상황에서 속도를 줄였는지 분석할 때 쓸 수 있습니다.

## 안전 경고

실차에서는 `/xycar_motor` publisher가 하나만 있어야 합니다. recorder는 publisher를 만들지 않지만, 수동조종 코드와 자율주행 코드가 동시에 켜져 있으면 차량이 위험하게 움직일 수 있습니다.

주행 전에 항상 다음을 확인하세요.

```bash
ros2 topic info /xycar_motor -v
```

`Publisher count`가 2 이상이면 주행하지 말고 중복 실행된 제어 노드를 먼저 끄세요.

## 설치

이 폴더를 자이카 워크스페이스의 `src` 아래에 둡니다.

```bash
cd /home/xytron/xycar_ws
colcon build --symlink-install --packages-select il_data_tools
source install/setup.bash
ros2 pkg executables il_data_tools
```

기대되는 executable:

```text
il_data_tools dataset_recorder_node
il_data_tools mission_labeler_node
```

## 실차 preflight 체크리스트

1. 메인 배터리와 프로세서 전원이 정상인지 확인합니다.
2. 모터를 사용할 경우 Motor Docker/VESC 브리지가 필요한지 확인하고 별도 터미널에서 실행합니다.
3. 센서 토픽이 살아 있는지 확인합니다.
4. `/xycar_motor` publisher/subscriber 수를 확인합니다.
5. `~/xycar_ws` 디스크 공간을 확인합니다.

추천 명령:

```bash
bash ~/xycar_ws/src/il_data_tools/scripts/check_topics.sh
```

## 추천 데이터 수집 절차

터미널 1: 모터 Docker 브리지 실행. 모터를 실제로 쓸 때만 실행합니다.

터미널 2: 미션 라벨러 실행.

```bash
source ~/xycar_ws/install/setup.bash
ros2 run il_data_tools mission_labeler_node
```

터미널 3: dataset recorder 또는 raw rosbag recorder 실행.

```bash
source ~/xycar_ws/install/setup.bash
ros2 launch il_data_tools record_dataset.launch.py session_name:=lane_run_01
```

또는 raw bag을 먼저 저장합니다.

```bash
bash ~/xycar_ws/src/il_data_tools/scripts/record_bag.sh
```

터미널 4: 기존 주행 코드나 수동조종 코드를 실행합니다. 이 터미널만 `/xycar_motor`를 publish하도록 관리하세요.

## 가장 안전한 첫 실행

아래 명령은 차량을 제어하지 않고 front camera, motor command, mission label만 기록합니다.

```bash
ros2 launch il_data_tools record_dataset.launch.py session_name:=dry_run save_scan_npz:=true save_side_images:=false
```

기본 저장 위치:

```text
~/xycar_ws/datasets/il/YYYYMMDD_HHMMSS_session_name/
  metadata.json
  samples.csv
  images/front/*.jpg
  images/left/*.jpg
  images/right/*.jpg
  images/rear/*.jpg
  scan/*.npz
  debug/
```

`metadata.json`은 세션당 1개만 생기는 작은 설정/요약 파일입니다. 학습에는 필수는 아니지만 나중에 어떤 토픽과 설정으로 모았는지 추적할 때 도움이 됩니다. 용량을 더 줄이고 싶으면 `write_metadata:=false`로 끌 수 있습니다.

`README_session.md`는 기본으로 저장하지 않습니다. 세션별 설명 파일이 필요할 때만 `write_session_readme:=true`로 켜세요.

## Mission labeler 키

```text
0: idle
1: lane_drive
2: cone_drive
3: hill_drive
4: pedestrian_avoid
5: vehicle_follow
6: vehicle_overtake
7: traffic_light_start
8: route_select
9: shortcut
p: parking
r: recovery
x: bad_data
q: quit
```

라벨러는 현재 라벨을 `/il/mission_label`로 5 Hz 주기로 계속 publish합니다.

## 주요 launch 옵션

```bash
ros2 launch il_data_tools record_dataset.launch.py \
  session_name:=cone_run_01 \
  camera_front_topic:=/usb_cam/image_raw/front \
  motor_topic:=/xycar_motor \
  mission_label_topic:=/il/mission_label \
  save_side_images:=false \
  save_scan_npz:=true \
  save_rate_hz:=10.0 \
  image_format:=jpg \
  jpeg_quality:=90
```

기본 motor 메시지는 `xycar_msgs/msg/XycarMotor`입니다. 참고한 `xycar_ws` 안에는 일부 예제가 `std_msgs/Float32MultiArray`를 `/xycar_motor`에 publish하는 경우도 있으므로, 그런 환경에서는 아래처럼 바꿔 실행할 수 있습니다.

```bash
ros2 launch il_data_tools record_dataset.launch.py motor_msg_type:=std_msgs/Float32MultiArray
```

## raw rosbag 기록

```bash
bash ~/xycar_ws/src/il_data_tools/scripts/record_bag.sh
```

환경변수로 토픽을 바꿀 수 있습니다.

```bash
FRONT_CAMERA_TOPIC=/image_raw \
REAR_CAMERA_TOPIC=/usb_cam/image_raw/rear \
bash ~/xycar_ws/src/il_data_tools/scripts/record_bag.sh
```

이 스크립트는 시작 전에 topic list, topic hz 확인 명령, 디스크 사용량, `/xycar_motor` publisher 수를 출력합니다. Docker는 자동으로 시작하거나 종료하지 않습니다.

## rosbag을 dataset으로 변환

ROS2 Humble의 `rosbag2_py`, `cv_bridge`, OpenCV가 사용 가능한 환경에서는 다음처럼 변환할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash
python3 ~/xycar_ws/src/il_data_tools/scripts/bag_to_dataset.py \
  ~/xycar_ws/bags/il/run_YYYYMMDD_HHMMSS \
  --session-name bag_lane_01
```

만약 `rosbag2_py`로 직접 읽기가 실패하면 fallback 방식으로 재생하면서 recorder로 다시 저장하세요.

```bash
ros2 launch il_data_tools record_dataset.launch.py session_name:=bag_replay
ros2 bag play ~/xycar_ws/bags/il/run_YYYYMMDD_HHMMSS
```

## dataset 요약

```bash
python3 ~/xycar_ws/src/il_data_tools/scripts/summarize_dataset.py \
  ~/xycar_ws/datasets/il/YYYYMMDD_HHMMSS_lane_run_01
```

출력 내용:

- 전체 샘플 수
- 라벨별 개수
- angle/speed min, max, mean, std
- 정지/이동 샘플 수
- 누락 이미지 수
- 샘플 rate 추정
- 가장 큰 시간 간격 10개

## 미션별 권장 세션

한 번에 모든 미션을 섞어 모으기보다 아래처럼 짧고 분명한 세션으로 나누는 것을 추천합니다.

- `lane_drive`
- `cone_drive`
- `hill_drive`
- `pedestrian_avoid`
- `vehicle_follow`
- `vehicle_overtake`
- `traffic_light_start`
- `route_select`
- `shortcut`
- `parking`
- `recovery`

## 좋은 데이터와 나쁜 데이터

좋은 데이터:

- 차선 중앙을 유지하는 주행
- 곡선 진입과 탈출
- 좌/우로 벗어난 상태에서 정상 차선으로 복귀
- 장애물에 천천히 접근하고 회피하는 장면
- shortcut 진입/탈출 성공
- parking 접근과 최종 정렬 성공

나쁜 데이터:

- 충돌
- 차량이 멈춰 빠져나오지 못한 구간
- 제어되지 않은 회전
- 사람이 잘못 넣은 조향/속도 명령
- 의도적으로 `idle`로 모으는 경우가 아닌 긴 정지 프레임

나쁜 구간은 가능하면 `x: bad_data`로 라벨링해서 학습 전에 제외할 수 있게 하세요.

## 학습 메모

처음에는 조향 예측부터 시작하는 것을 추천합니다. 속도는 미션별 rule-based 로직으로 두는 편이 실차 안전을 관리하기 쉽습니다.

미션 라벨은 데이터 필터링이나 미션별 모델 분리에 사용하세요. raw bag은 나중에 라벨을 다시 붙이거나 추가 센서를 포함해 재변환할 수 있으므로 가능하면 보관하는 것이 좋습니다.

## 이 패키지가 하지 않는 것

- 딥러닝 학습 파이프라인을 만들지 않습니다.
- 자율주행 로직을 추가하지 않습니다.
- `track_drive`나 기존 패키지를 수정하지 않습니다.
- recorder가 `/xycar_motor`에 publish하지 않습니다.
- GPU가 있다고 가정하지 않습니다.
