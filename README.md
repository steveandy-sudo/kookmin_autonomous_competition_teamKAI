# Team KAI - Kookmin Autonomous Driving Mission 1

## 과제

국민대학교 자율주행 경진대회 예선 과제 1번을 위한 ROS2 Humble 기반 주행 패키지이다. 시뮬레이터에서 제공하는 전방 카메라, LiDAR, odom, IMU 토픽을 사용해 라바콘 구간 주행, 어린이보호구역 감속, 보행자/방해차량 대응, 신호등 및 정지선 판단, 교차로 경로 선택을 수행한다.

## Mission Manager V0.1 초안

새 결정 계층은 `track_drive/mission/`에 있으며, 고수준 상태와 하위 제어 모드,
조향 source, 속도 profile, 정지 필요 여부만 결정한다. 기존 perception·학습 모델·
콘 조향 알고리즘을 수정하거나 실행하지 않으며, publisher와 `/xycar_motor`
출력도 만들지 않는다. 기존 `track_drive/mission_state.py`와는 아직 연결하지
않은 독립 초안이다.

설계와 통합시험 입력은
[`docs/MISSION_MANAGER_V01.md`](docs/MISSION_MANAGER_V01.md)를 참고한다.

## 실행법

### 1. 워크스페이스 빌드

제출 코드를 ROS2 워크스페이스의 `src/track_drive` 위치에 둔 뒤 빌드한다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select track_drive cone_il
source install/setup.bash
```

필요하면 같은 터미널에서 ROS domain을 맞춘다.

```bash
export ROS_DOMAIN_ID=18
```

### 2. ROS-TCP endpoint 실행

터미널 1에서 endpoint를 실행한다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint
```

시뮬레이터의 ROS-TCP 연결 설정은 일반 localhost 연결 기준으로 다음 값을 사용한다.

```text
Host/Address: 127.0.0.1
Port: 10000
```

### 3. 시뮬레이터 실행

Kookmin/Xytron 시뮬레이터를 실행하고 ROS-TCP endpoint와 연결한다. 연결 후 `/usb_cam/image_raw/front`, `/scan`, `/odom`, `/imu` 등의 토픽이 발행되는지 확인한다.

### 4. 주행 노드 실행

터미널 2에서 주행 launch 파일을 실행한다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch track_drive ai_direct_hybrid.launch.py startup_light_require_signal:=false
```

## 코드 계층구조

```text
kookmin_autonomous_competition_teamKAI/
├── assets/models/          # CNN 조향 모델(.pt), YOLO/ONNX 객체 인식 모델
├── cone_il/                # CNN End-to-End 조향 모델 관련 ROS2 패키지
├── launch/                 # 통합 주행 및 디버그 launch 파일
├── config/                 # Mission Manager V0.1 파라미터
├── resource/               # ROS2 ament package marker
├── rviz/                   # 디버그 시각화 설정
├── track_drive/            # 과제 1 통합 주행 로직
│   └── mission/            # 독립 Mission Manager V0.1 결정 계층
├── package.xml             # ROS2 패키지 의존성 정의
├── setup.py                # Python 노드, launch, 모델 파일 설치 설정
├── setup.cfg               # ROS2 Python 실행 파일 설치 경로 설정
├── requirements.txt        # Python 실행 의존성 참고
├── reports/                # 개발 과정과 실험 내용을 정리한 보고서
└── GoogleDrive.txt         # 추가 자료/영상 공유 링크
```

## 각 소스코드의 역할

### track_drive 패키지

- `track_drive/track_drive.py`: 과제 1 전체 주행을 담당하는 메인 노드이다. 여러 판단을 종합해 최종 주행 명령을 만든다.
- `track_drive/switchable_cone_ai_driver.py`: CNN 조향 노드를 외부 enable/speed limit 토픽으로 켜고 끄는 노드이다.
- `track_drive/safety_supervisor.py`: 신호등, 정지선, 차량 등 안전 정지 조건을 통합 관리한다.
- `track_drive/traffic_light_detector.py`: ONNX 객체 인식 모델과 색상 기반 보조 판단으로 신호등 상태를 추정한다.
- `track_drive/stop_line_detector.py`: 카메라 이미지를 BEV로 변환해 흰색 정지선을 검출한다.
- `track_drive/school_zone_detector.py`: 노란색 노면 표식과 BEV 기반 분석으로 어린이보호구역을 판단한다.
- `track_drive/intersection_decider.py`: 교차로 경로 판단 호출 경계를 담당한다.
- `track_drive/perception.py`: 카메라/객체 인식 결과를 주행 판단에서 쓰기 좋은 형태로 정리한다.
- `track_drive/lidar_utils.py`: LiDAR scan 데이터를 전방 장애물/라바콘 판단에 사용할 수 있도록 보조 처리한다.
- `track_drive/control.py`: 조향/속도 명령 계산에 필요한 제어 보조 함수를 담는다.
- `track_drive/mission_state.py`: 미션 진행 상태를 표현하는 보조 구조를 담는다.
- `track_drive/mission/`: V0.1 상태·제어 모드와 source 선택만 담당하는 독립 초안이다.
- `track_drive/package_paths.py`: 설치된 ROS2 패키지 내부의 모델/리소스 경로를 찾는다.
- `track_drive/utils.py`: 공통 유틸리티 함수 모음이다.
- `track_drive/*_debug_node.py`: 신호등, 정지선, 어린이보호구역, 교차로 판단을 개별 확인하기 위한 디버그 노드이다.

### cone_il 패키지

- `cone_il/cone_il/cone_ai_driver_node.py`: 이미지를 모델에 입력해 조향각을 예측하고 `/xycar_motor`를 발행한다.
- `cone_il/cone_il/preprocess.py`: 이미지를 crop, resize, RGB 변환, 정규화, CHW 텐서 형태로 전처리를 수행한다.
- `cone_il/cone_il/model.py`: CNN 조향 모델 구조를 정의한다.
- `cone_il/cone_il/cone_data_recorder_node.py`: 주행 데이터 수집용 노드이다.
- `cone_il/cone_il/xycar_*_teleop_node.py`: 데이터 수집 또는 수동 조작에 사용하는 키보드 teleop 노드이다.
- `cone_il/scripts/train_cone_bc.py`: 수집된 이미지/조향 데이터를 이용해 CNN 조향 모델을 학습한다.
- `cone_il/scripts/merge_cone_datasets.py`: 여러 주행 데이터셋을 학습용 데이터셋으로 병합한다.

## 모델 설명 / 학습 원리

### CNN End-to-End 조향 모델

`assets/models/cone_bc_scripted_*.pt` 파일은 전방 카메라 이미지를 입력으로 받아 조향각을 출력하는 TorchScript CNN 모델이다. 입력 이미지는 하단 도로 영역을 중심으로 crop하고, 고정 크기로 resize한 뒤, RGB 변환과 0~1 정규화를 거쳐 CHW 형태의 float32 텐서로 변환된다. 학습은 사람이 주행하거나 기존 주행 로직으로 얻은 카메라 이미지와 조향각 데이터를 짝지어 진행한다. 모델은 이미지에서 라바콘 배치와 도로 진행 방향을 학습하고, 실제 주행 중에는 매 프레임 조향각을 예측한다. 주행 안정성을 위해 예측 조향각에는 smoothing, 최대 조향각 제한, 조향각 기반 속도 제한을 함께 적용한다.

### 객체 인식 모델

`assets/models/final.onnx`는 신호등, 보행자, 차량, 라바콘 등 미션 객체 인식을 위한 ONNX 모델이다. 메인 주행 노드는 이 모델의 검출 결과와 색상/위치/크기 조건을 함께 사용해 신호등 상태, 보행자 위험, 차량 존재, 좌측 라바콘 존재 여부를 판단한다.

## 미션내용

- 출발 신호등 확인 후 주행 시작
- 라바콘 구간에서 CNN End-to-End 기반 조향 주행
- 아스팔트/구불길 구간 통과
- 보행자 회피 및 안전 정지 판단
- 방해차량 인식 및 추월/회피 판단
- 어린이보호구역 노면 표식 인식 및 속도 제한
- 신호등, 정지선, 좌측 라바콘 상태를 조합한 교차로 직진/좌회전 판단
- 총 3바퀴 주행 후 종료

## 참고

이 패키지는 ROS2 Humble, Python 3, OpenCV, NumPy, PyTorch, ONNX Runtime 환경을 기준으로 작성되었다. CUDA 사용이 가능한 환경에서는 CNN/ONNX 추론이 GPU에서 동작할 수 있으며, CUDA가 없는 경우 CPU 경로로 동작한다.
