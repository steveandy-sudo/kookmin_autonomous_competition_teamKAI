# Team KAI - Kookmin Autonomous Driving Mission 1

## 과제

국민대학교 자율주행 경진대회 예선 과제 1번을 위한 ROS2 Humble 기반 주행 패키지이다. 시뮬레이터에서 제공하는 전방 카메라, LiDAR, odom, IMU 토픽을 사용해 라바콘 구간 주행, 어린이보호구역 감속, 보행자/방해차량 대응, 신호등 및 정지선 판단, 교차로 경로 선택을 수행한다.

## Mission Manager V0.2 초안

새 결정 계층은 `track_drive/mission/`에 있다. `MissionState`가 차선·콘·
고정 장애물·추월·경로 선택·지름길 미션을 나타내고, 각 상태에서 허용된
`ControlMode`만 source 상태에 따라 선택한다. 기존 perception·학습 모델·콘
조향 알고리즘을 수정하거나 실행하지 않으며 publisher와 `/xycar_motor` 출력도
만들지 않는다. 기존 `track_drive/mission_state.py`와는 아직 연결하지 않은
독립 초안이다.

### 출발 신호 조건

신호등 인지 모델의 후처리 adapter는 Mission Manager에 원본 영상이나 모델
출력을 직접 넘기지 않고 `UNKNOWN`, `RED`, `YELLOW`, `GO` 중 하나와 그
판단의 유효 여부를 전달한다. Mission Manager는 다음 순서에서만 출발한다.

1. 유효한 `RED`가 0.3초 연속 확인되면 출발 대기 조건을 arm한다.
2. arm된 뒤 유효한 `GO`가 0.3초 연속 확인되면 `LANE_DRIVING`으로 전환한다.

RED를 확인하기 전에 GO만 보이면 출발하지 않는다. RED 확인 전 YELLOW는
RED 확인 시간을 초기화한다. RED 확인 후 YELLOW, UNKNOWN 또는 무효 판단이
들어오면 arm 상태는 유지하되 진행 중이던 GO 확인 시간을 초기화하고 계속
정지한다. 모델 confidence 임계값과 `BLUE`/`GREEN`을 `GO`로 정규화하는 일은
인지 adapter가 담당한다.

`safety_ready=false`이거나 `safety_stop_required=true`이면
`WAIT_START_SIGNAL + STOP`을 유지하고 RED/GO 확인 이력을 모두 지운다. 수동
`START`는 통합시험 전용이며 실제 대회 출발 조건으로 사용하지 않는다.

GO가 확정되는 주기에는 별도의 준비 확인 시간을 기다리지 않고 현재 source
유효성으로 제어기를 즉시 선택한다. `drive_policy_valid=true`이면
`NORMAL_IL`을 가장 먼저 선택한다. 일반 모델 출력이 무효이고
`lane_fallback_valid=true`이면 `LANE_FALLBACK`, 둘 다 무효이면
`LANE_DRIVING + STOP`을 선택한다. 두 source가 모두 유효한 경우에도
`NORMAL_IL`이 우선이다.

### LANE_DRIVING 제어기 선택

`LANE_DRIVING`은 특별 미션이 아닌 기본 주행 구간을 뜻한다. 일반 모델이나
fallback의 성공 여부 때문에 MissionState를 바꾸지 않는다.

```text
NORMAL_IL 정상
→ NORMAL_IL 유지

NORMAL_IL hard-invalid + YOLO fallback이 0.2초 이상 미리 준비됨
→ 첫 실패 주기에 LANE_FALLBACK

NORMAL_IL hard-invalid + fallback 준비 안 됨
→ 즉시 LANE_DRIVING + STOP

출발 후 STOP 상태에서 fallback 0.2초 확인
→ LANE_FALLBACK

출발 후 일반 모델 0.4초 회복 확인
→ NORMAL_IL
```

입력이 사라진 일반 모델을 fallback 확인 시간 동안 계속 선택하지 않는다.
일반 모델이 정상일 때도 fallback 준비 시간을 백그라운드에서 누적하므로 이미
준비된 fallback으로는 기다리지 않고 전환할 수 있다. 두 source가 모두 무효이면
`LANE_DRIVING + STOP`으로 기다리며 상태를 잃지 않는다.

### CONE_SECTION 진입과 이탈

콘 구간은 카메라 한 센서의 순간 검출만으로 진입하지 않는다. 카메라 콘 입력과
LiDAR 콘 입력이 모두 정상·최신이고, 카메라 콘이 4개 이상이면서 LiDAR도 콘을
감지한 조건이 0.25초 연속 유지되면 `CONE_SECTION + CONE_DRIVE_RULE`로
전환한다. 현재 mode가 `NORMAL_IL`, `LANE_FALLBACK`, `STOP` 중 무엇인지와
관계없이 같은 조건을 적용한다.

진입 후에는 최소 1.0초 동안 콘 구간을 유지한다. 이후 양쪽 센서 입력이 모두
정상·최신인 상태에서 카메라 콘이 1개 이하이고 LiDAR가 콘을 감지하지 않은
조건이 0.7초 연속 유지되면 `LANE_DRIVING`으로 돌아간다. 센서 입력 무효나
stale은 `콘 없음`으로 취급하지 않는다. 진입 기준 4개와 이탈 기준 1개를
분리해 검출 개수가 경계에서 흔들릴 때 상태가 왕복하지 않게 한다.

Mission Manager는 카메라 영상이나 `LaserScan`을 직접 구독하지 않고, 인지
계층이 만든 다음 의미 기반 ROS2 토픽을 구독한다.

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/mission/input/camera_cone_valid` | `std_msgs/Bool` | 최신 카메라 프레임으로 콘 인지가 정상 완료됐는지 |
| `/mission/input/camera_cone_count` | `std_msgs/Int32` | 현재 유효 프레임에서 검출된 cone class 개수 |
| `/mission/input/lidar_cone_valid` | `std_msgs/Bool` | 최신 LiDAR scan을 정상 처리했는지 |
| `/mission/input/lidar_cone_detected` | `std_msgs/Bool` | LiDAR에서 콘 형태의 소형 클러스터가 하나 이상 검출됐는지 |

`valid=true`는 콘이 존재한다는 뜻이 아니라 해당 센서 결과를 이번 판단에
사용할 수 있다는 뜻이다. 예를 들어 카메라 추론이 정상이고 콘이 보이지 않으면
`camera_cone_valid=true`, `camera_cone_count=0`이다. 카메라 프레임 또는
LiDAR scan이 stale이거나 처리에 실패하면 해당 `valid`를 `false`로 보내야
하며, Mission Manager는 이를 콘 미검출로 간주하지 않는다.

카메라 인지 측은 기존 `final.onnx`의 cone class 검출 결과를 세어 카메라 토픽을
발행하고, LiDAR 인지 측은 기존 소형 클러스터 추출 결과로 LiDAR 토픽을
발행한다. 같은 YOLO 추론을 adapter에서 다시 실행하지 않는다. 현재 V0.2에는
위 토픽의 subscriber 계약만 있으며 실제 인지 결과 publisher 연결은 후속
통합 작업이다.

콘 조향 source가 일시적으로 무효가 되더라도 이를 MissionState 전이 조건으로
사용하지 않는다. 마지막으로 확정된 상태가 `CONE_SECTION`이면
`CONE_SECTION + CONE_DRIVE_RULE`을 그대로 유지한다. 앞으로 연결할 유일한
Final Driver는 새 콘 조향값이 유효하고 유한할 때만 `last_valid_steering_angle`을
갱신하고, 무효·stale이면 마지막 유효 조향각을 그대로 사용한다. 무효 지속시간에
따른 자동 `STOP`이나 조향 변화율 제한은 두지 않는다. 이 sample-and-hold는
조향각에만 적용하며 Mission Manager는 숫자 조향값을 저장하거나 계산하지
않는다.

### YOLO Lane Fallback 계약

`simulation` 브랜치의 `YOLO11n-seg 512`는 class 0 흰색 차선과 class 1 노란
중앙선을 segmentation mask로 제공한다. 이 대회 트랙에서 노란선은 경계가
아니라 흰색 차선 사이의 주행 중앙을 뜻하므로 fallback 목표 경로 우선순위는
다음과 같다. 모델과 실행 방법은
[`simulation` 브랜치 YOLO 차선 문서](https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI/blob/simulation/docs/real_vehicle_yolo_lane_20260720.md)를
기준으로 한다.

1. 노란 중앙선이 유효하면 노란선 자체를 목표 경로로 사용한다.
2. 노란선이 없고 양쪽 흰선이 유효하면 두 흰선의 중간을 사용한다.
3. 흰선 하나만 유효하면 검증된 예상 반폭으로 중앙을 추정한다.
4. 어떤 경로도 신뢰할 수 없거나 결과가 stale이면 fallback을 무효로 한다.

흰선은 노란선과 평균을 내기 위한 경계가 아니라 노란 중앙선의 위치 검증과
소실 시 복구에 사용한다. YOLO node와 fallback controller는 조향 후보,
freshness, `lane_fallback_valid`를 만들고 Mission Manager는 그 유효 여부만
사용한다. Mission Manager는 mask, BEV, 경로, Pure Pursuit 또는 숫자 조향값을
계산하지 않는다. 기존 `simulation` 브랜치의 rule driver는 노란선과 한쪽
흰선의 중간을 선택하고 motor를 발행할 수 있으므로 그대로 연결하지 않는다.

### 복구 가능한 안전 정지

`EMERGENCY_STOP` MissionState와 수동 emergency/reset 명령은 사용하지 않는다.
`safety_stop_required=true`이면 현재 MissionState를 보존한 채 `STOP`과
`stop_required=true`를 요청한다. 입력이 해제된 뒤 사용 가능한 제어기의 정상
상태가 확인되면 같은 MissionState에서 자동 복구한다. 물리 비상 차단은 Mission
Manager 상태가 아니라 Safety Supervisor 또는 유일한 Final Driver 아래에서
독립적으로 보장해야 한다.

설계와 통합시험 입력은
[`docs/MISSION_MANAGER_V02.md`](docs/MISSION_MANAGER_V02.md)를 참고한다.

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
├── config/                 # Mission Manager V0.2 파라미터
├── resource/               # ROS2 ament package marker
├── rviz/                   # 디버그 시각화 설정
├── track_drive/            # 과제 1 통합 주행 로직
│   └── mission/            # 독립 Mission Manager V0.2 결정 계층
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
- `track_drive/mission/`: V0.2 미션 상태와 허용 제어기 선택을 담당하는 독립 초안이다.
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
