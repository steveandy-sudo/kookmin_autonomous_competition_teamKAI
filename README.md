# Team K.A.I. Mission Manager V0.2

ROS2 Humble 기반 자율주행 차량의 미션 상태와 제어기 선택을 담당하는
패키지이다. 현재 자동 처리 범위는 출발 신호, 일반 차선 주행, 콘 구간이다.

Mission Manager는 주행 명령을 직접 만들지 않는다. 인지·제어 노드의 결과를
의미 기반 입력으로 받은 뒤 다음 항목만 결정한다.

- 현재 `MissionState`
- 현재 `ControlMode`
- 사용할 조향 source
- 요청할 속도 profile
- 정지 필요 여부

Mission Manager와 이 저장소의 입력 선택 코어는 카메라 영상을 처리하지 않고,
YOLO·IL 모델을 실행하지 않으며, 최종 조향·속도를 계산하거나
`/xycar_motor`를 발행하지 않는다.

## 현재 구조

```text
외부 인지·주행 source
  ├─ 신호등 인지
  ├─ 일반 IL 주행
  ├─ YOLO canonical lane perception
  └─ LiDAR rule-based cone
              │
              ▼
입력 Adapter ── freshness와 값 형식 검증
              │
              ▼
Mission Manager ── MissionDecision 발행
              │
              ▼
Final Driver 입력 코어 ── 선택된 source의 조향·속도 후보만 전달
              │
              ▼
향후 단일 Final Driver ── 미구현, 유일한 /xycar_motor publisher
```

저장소 구성은 다음과 같다.

```text
├── assets/models/final.onnx       # 임시 신호등·카메라 콘 인지 모델
├── config/                        # 노드와 상태 전이 파라미터
├── docs/MISSION_MANAGER_V02.md    # 상세 상태 전이 명세
├── launch/
│   ├── mission_manager_draft.launch.py
│   └── traffic_light_debug.launch.py
├── rviz/traffic_light_debug.rviz
├── teamkai_interfaces/            # ROS2 전용 메시지 패키지
├── test/                          # 상태·입력 계약 단위 테스트
└── track_drive/
    ├── mission/                   # Mission Manager
    ├── integration/               # source별 Adapter
    ├── final_driver/              # ROS 비의존 입력 선택·단위 변환 코어
    ├── lane_fallback_controller.py
    ├── lane_fallback_controller_node.py
    └── traffic_light_*.py         # 임시 출발 신호 source
```

## MissionState와 ControlMode

현재 자동 전이가 구현된 상태는 다음 세 가지다.

| MissionState | 의미 | 사용할 수 있는 ControlMode |
|---|---|---|
| `WAIT_START_SIGNAL` | 출발 신호 대기 | `STOP` |
| `LANE_DRIVING` | 일반 차선 주행 | `NORMAL_IL`, `LANE_FALLBACK`, `STOP` |
| `CONE_SECTION` | 콘 구간 주행 | `CONE_DRIVE_RULE` |

다음 enum 값은 향후 대회 구간을 위한 자리만 있으며 자동 전이는 아직 없다.

- `FIXED_OBSTACLE_SECTION`
- `OVERTAKE_SECTION`
- `ROUTE_SELECTION`
- `SHORTCUT_SECTION`

`MissionState`는 코스 구간을, `ControlMode`는 그 구간에서 현재 선택한
제어기를 나타낸다. 둘을 분리하므로 같은 차선 구간에서도 IL과 Lane Fallback을
안전하게 교체할 수 있다.

## 상태 전이

### 출발

노드 시작 상태는 `WAIT_START_SIGNAL + STOP`이다.

1. 유효한 `RED`가 0.3초 유지되면 출발 준비가 활성화된다.
2. 그 뒤 유효한 `GREEN`이 0.3초 유지되면 `LANE_DRIVING`으로 진입한다.
3. `YELLOW`, `UNKNOWN`, 오래된 입력으로는 출발하지 않는다.
4. `GREEN` 진입 시 일반 IL이 유효하면 즉시 `NORMAL_IL`을 사용한다.
5. IL이 무효이면 Lane Fallback을 확인하고, 둘 다 사용할 수 없으면 `STOP`한다.

### 일반 차선 주행

제어기 우선순위는 `NORMAL_IL → LANE_FALLBACK`이다.

- `/il/policy_debug`가 유효하면 `NORMAL_IL`
- IL이 무효이고 Lane Fallback이 준비되면 `LANE_FALLBACK`
- 현재 source가 일시적으로 무효이면 최대 1.0초 동안 이전 제어 모드와 마지막
  유효 조향을 유지하고 `fallback` 속도 profile을 요청
- 1.0초 이상 새 유효 조향이 없으면 `STOP`
- 시작 직후 한 번도 유효한 조향을 받지 못했다면 즉시 `STOP`

IL 입력 유효 조건은 다음과 같다.

```python
len(data) >= 4
and isfinite(data[2])
and -42.0 <= data[2] <= 42.0
and message_age <= 0.5
```

`/il/policy_debug`에서 Final Driver 입력으로 사용할 값은 다음과 같다.

| 배열 위치 | 의미 |
|---|---|
| `data[2]` | IL 노드가 계산한 최종 Xycar 조향 명령 |
| `data[3]` | IL 노드가 계산한 요청 속도 |

Mission Manager는 `data[3]`의 속도 범위를 검사하거나 수정하지 않는다.
Final Driver 입력 경계에서는 NaN과 Inf만 거부한다.

### 콘 구간

다음 조건이 0.25초 동안 함께 유지되면 `CONE_SECTION`으로 진입한다.

- 카메라 콘 개수 입력이 유효함
- 카메라 콘 개수가 4개 이상
- LiDAR 콘 source가 유효함
- LiDAR 콘 경로가 준비됨

다음 조건이 0.7초 유지되면 `LANE_DRIVING`으로 복귀한다.

- 카메라 입력이 유효함
- 카메라 콘 개수가 1개 이하
- LiDAR에서 콘이 존재하지 않음

콘 구간은 최소 1.0초 유지하며, 이탈 후 1.0초 동안 재진입하지 않는다.
카메라나 LiDAR 입력 자체가 무효이면 현재 상태를 유지한다.

콘 명령 계약은 다음과 같다.

```text
/my_rule/cone_cmd
std_msgs/Float32MultiArray

data[0] = physical_angle_deg
data[1] = requested_speed
data[2] = confidence
```

콘 입력이 일시적으로 무효가 되면 `CONE_SECTION`을 유지하면서 마지막 유효
조향과 최소 콘 속도를 사용한다. 콘 모드를 벗어나거나 정지 결정이 내려지면
저장된 조향을 초기화한다.

`my_rule/rule_driver`는 `/xycar_motor`를 직접 발행하므로 함께 실행하지 않는다.
콘 경로와 `/my_rule/cone_cmd`를 만드는 source 노드만 사용한다.

## Lane Fallback

Lane Fallback은 `/perception/centerline`의 canonical 경로를 이용한다.

- 노란선은 경계가 아니라 차량이 따라야 할 중앙선으로 취급
- 노란선이 있으면 노란선 중심 경로 사용
- 노란선이 없고 흰선 두 개가 있으면 두 흰선의 중간 경로 사용
- 흰선 하나만 있거나 경로가 부족하면 무효

`lane_fallback_controller`는 2단 Pure Pursuit로 물리 조향각을 계산한다.

- wheelbase: 0.33m
- near lookahead: 0.70m
- far lookahead 최대: 1.45m
- far preview weight: 0.65
- 물리 조향각 제한: ±26도

출력 메시지는 다음과 같다.

```text
/lane_fallback/command
teamkai_interfaces/LaneFallbackCommand

builtin_interfaces/Time stamp
float32 steering_angle_deg
bool valid
```

이 노드는 속도나 `/xycar_motor`를 발행하지 않는다. Lane Fallback 속도는
향후 실차 시험으로 정할 공통 `fallback_speed` 파라미터를 사용한다.

## MissionDecision

Mission Manager의 유일한 결정 출력은 다음 메시지다.

```text
/mission/decision
teamkai_interfaces/MissionDecision

builtin_interfaces/Time stamp
uint8 mission_state
uint8 control_mode
uint8 selected_source
uint8 speed_profile
bool stop_required
```

`selected_source`는 “어느 입력을 사용할지”를 지정하고, `speed_profile`은
구체적인 숫자가 아닌 `normal`, `cone`, `fallback`, `stop` 같은 의미 기반
요청이다.

## Final Driver 입력 계약

`track_drive/final_driver`에는 ROS와 독립적인 입력 선택 코어까지만 구현되어
있다. ROS Final Driver 노드와 `/xycar_motor` publisher는 아직 없다.

| 선택 source | 조향 입력 | 속도 입력 | 조향 단위 |
|---|---|---|---|
| `drive_il` | `/il/policy_debug.data[2]` | `data[3]` | Xycar 명령 단위 |
| `lane_fallback` | `/lane_fallback/command.steering_angle_deg` | `fallback_speed` | 물리 각도 |
| `cone_rule` | `/my_rule/cone_cmd.data[0]` | `data[1]` | 물리 각도 |

물리 각도는 현재 다음 보정표로 Xycar 명령 단위에 선형 변환된다.

| 물리 각도 절댓값 | Xycar 명령 절댓값 |
|---:|---:|
| 0° | 0 |
| 4° | 10 |
| 10° | 20 |
| 16° | 30 |
| 26° | 42 |

보정표 밖의 값은 ±42로 제한한다. 조향 변화율 제한이나 smoothing은 적용하지
않는다. 좌우 부호는 `physical_steering_sign`으로 실차에서 확인해야 한다.

## ROS2 토픽

### 외부 source 입력

| 토픽 | 타입 | 역할 |
|---|---|---|
| `/track_drive/traffic_light_debug/state` | `std_msgs/String` | 임시 신호등 상태 |
| `/il/policy_debug` | `std_msgs/Float32MultiArray` | 일반 IL 조향·속도 |
| `/perception/centerline` | `kaiev26_msgs/Centerline` | Lane Fallback 중심 경로 |
| `/perception/road_segments` | `kaiev26_msgs/RoadSegmentArray` | 흰선·노란선 구성 확인 |
| `/perception/camera_cone_count` | `std_msgs/Int32` | 카메라 콘 개수 |
| `/my_rule/cone_cmd` | `std_msgs/Float32MultiArray` | 콘 조향·속도·신뢰도 |
| `/my_rule/cone_clusters` | `geometry_msgs/PoseArray` | LiDAR 콘 존재 확인 |

### Mission Manager 내부 입력

| 토픽 | 타입 |
|---|---|
| `/mission/input/start_signal` | `std_msgs/String` |
| `/mission/input/start_signal_valid` | `std_msgs/Bool` |
| `/mission/input/drive_policy_valid` | `std_msgs/Bool` |
| `/mission/input/lane_fallback_valid` | `std_msgs/Bool` |
| `/mission/input/camera_cone_count` | `std_msgs/Int32` |
| `/mission/input/camera_cone_valid` | `std_msgs/Bool` |
| `/mission/input/lidar_cone_source_valid` | `std_msgs/Bool` |
| `/mission/input/lidar_cone_path_ready` | `std_msgs/Bool` |
| `/mission/input/lidar_cone_present` | `std_msgs/Bool` |
| `/mission/input/safety_stop_required` | `std_msgs/Bool` |
| `/mission/override` | `std_msgs/String` |

수동 override는 통합시험용이며 대회 자동 실행 조건으로 사용하지 않는다.

## 빌드와 실행

ROS2 Humble 워크스페이스의 `src` 아래에 저장소를 둔 뒤 빌드한다.

```bash
cd ~/xycar_ws
colcon build --symlink-install \
  --packages-select teamkai_interfaces track_drive
source install/setup.bash
```

임시 신호등 인지 source가 필요하면 별도로 실행한다.

```bash
ros2 launch track_drive traffic_light_debug.launch.py
```

Mission Manager와 Adapter를 실행한다.

```bash
ros2 launch track_drive mission_manager_draft.launch.py
```

외부 source는 별도로 실행해야 한다.

- 일반 IL 노드는 반드시 motor 발행을 끈 상태(`drive_enabled:=false`)로 실행
- YOLO canonical perception은 `/perception/centerline`과
  `/perception/road_segments` 발행
- LiDAR 콘 노드는 `/my_rule/cone_cmd`와 `/my_rule/cone_clusters`만 발행
- `/xycar_motor`를 직접 발행하는 외부 driver는 함께 실행하지 않음

Mission Manager는 약 1초 간격으로 현재 zone과 차량 제어 상태를 출력한다.

```text
[mission_manager] zone=LANE_DRIVING mode=NORMAL_IL
source=drive_il speed=normal stop=false

[mission_manager] zone=CONE_SECTION mode=CONE_DRIVE_RULE
source=cone_rule speed=cone stop=false
```

## 테스트

ROS 비의존 단위 테스트는 다음과 같이 실행한다.

```bash
python -m unittest discover -s test -p "test_*.py"
```

## 현재 미구현 또는 실차 확인 필요

- 숫자 `fallback_speed` 확정
- 조향 보정 부호와 표의 실차 검증
- 실제 canonical perception rosbag으로 Lane Fallback 검증
- 실제 LiDAR rosbag으로 콘 진입·이탈 threshold 조정
- 고정 장애물, 추월, 경로 선택, 지름길 상태 전이
- 단일 Final Driver ROS 노드와 유일한 `/xycar_motor` publisher

더 상세한 상태 전이와 입력 기억 규칙은
[`docs/MISSION_MANAGER_V02.md`](docs/MISSION_MANAGER_V02.md)를 참고한다.
