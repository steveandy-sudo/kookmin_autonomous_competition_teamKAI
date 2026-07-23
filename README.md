# Team KAI - Kookmin Autonomous Driving Mission 1

## 패키지 목적

국민대학교 자율주행 경진대회용 ROS2 Humble 패키지다. `state_machine`
브랜치의 중심은 각 팀원이 만든 인지·조향 코드를 교체할 수 있도록
Mission Manager와 후단 주행 계층의 경계를 정하는 것이다.

Mission Manager V0.2에서 자동 동작하는 구간은 출발 대기, 일반 차선 주행,
콘 구간이다. 고정 장애물, 추월, 경로 선택과 지름길 상태는 인터페이스만
예약되어 있으며 자동 전이는 실행하지 않는다.

## 현재 V0.2 구조

```text
신호등·IL·Lane·Cone source
              │
              ▼
      source별 integration adapter
      형식 검사 + freshness 판정
              │
              ▼
         Mission Manager
  MissionState + ControlMode 결정
              │
              ▼
       /mission/decision
              │
              ▼
         Final Driver
  조향 후보 선택 + 숫자 속도 결정
              │
              ▼
 /xycar_motor [steering, speed]
```

| 계층 | 현재 V0.2 제공 범위 |
|---|---|
| Source adapter | 출발 신호, 일반 IL 유효성, Lane Fallback 유효성, 카메라·LiDAR 콘 의미 입력 |
| Mission Manager | `WAIT_START_SIGNAL`, `LANE_DRIVING`, `CONE_SECTION` 자동 전이와 제어기 선택 |
| MissionDecision | `/mission/decision` 단일 메시지 출력 |
| 숫자 조향 후보 공통 형식 | V0.2에 포함되지 않음 |
| Final Driver | V0.2에 포함되지 않음 |
| `/xycar_motor` 출력 | `mission_manager_draft.launch.py`에 포함되지 않음 |

`track_drive/track_drive.py`, `cone_il`처럼 저장소에 남아 있는 기존 주행 코드는
참고용 source archive다. 이 노드들의 console entry point와 launch는 V0.2 설치
대상에서 제외되어 있으며, 최종 구조에서는 우리 Final Driver만
`/xycar_motor`를 발행해야 한다.

### 연결 기준 구현

Mission Manager에 알고리즘을 복사하지 않고 다음 브랜치의 source 노드를 별도로
실행해 토픽으로 연결한다.

| 기능 | 기준 구현 | Mission Manager가 사용하는 계약 |
|---|---|---|
| 일반 IL 주행 | [`simulation/il_data_tools/policy_inference_node.py`](https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI/blob/simulation/xycar_ws/src/il_data_tools/il_data_tools/policy_inference_node.py) | `/il/policy_debug`; 반드시 `drive_enabled:=false` |
| LiDAR rule-based 콘 주행 | [`hwj/my_rule/cone_node.py`](https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI/blob/hwj/xycar_ws/src/study/my_rule/my_rule/cone_node.py) | `/my_rule/cone_cmd`, `/my_rule/cone_clusters`; `rule_driver`는 실행하지 않음 |
| YOLO 차선 fallback 인지 | [`simulation/xycar_perception/yolo_lane_segmenter.py`](https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI/blob/simulation/xycar_ws/src/xycar_perception/xycar_perception/yolo_lane_segmenter.py) | `/perception/centerline`의 유효성 |
| 출발 신호·카메라 콘 수 | 이 패키지의 `traffic_light_debug` + `final.onnx` | 신호 상태와 `/perception/camera_cone_count` |

일반 IL의 `/il/policy_motor_shadow`와 콘 노드의 numeric angle·speed는 V0.2
Mission Manager 입력이 아니다. V0.2는 source의 사용 가능 여부와 어떤 source를
선택할지만 결정한다.

## Mission Manager V0.2 기능

Mission Manager는 `track_drive/mission/`에서 현재 코스 구간인 `MissionState`와
그 구간에서 허용되는 `ControlMode`를 결정한다. 또한 선택할 조향 source, 요청할
속도 profile, 주행 또는 정지 요청을 결정한다. perception, 학습 모델, 콘 조향
알고리즘은 각각의 source 노드가 담당하고, 통합 adapter가 source의 출력과
freshness를 Mission Manager용 의미 기반 입력으로 변환한다.

Mission Manager는 원본 센서 처리, 모델 추론, 숫자 조향·속도 계산을 수행하지
않으며 `/xycar_motor`를 발행하지 않는다. `track_drive/mission_state.py`와도
분리된 독립 결정 계층이다.

### MissionDecision 출력

Mission Manager는 각 20 Hz 판단 주기의 완성된 결정을
`/mission/decision` (`teamkai_interfaces/msg/MissionDecision`)에 발행한다.
한 메시지에 다음 필드가 함께 들어가므로 Final Driver는 서로 다른 판단 주기의
상태와 source를 섞어 사용하지 않는다.

| 필드 | 의미 |
|---|---|
| `stamp` | Mission Manager가 결정을 만든 ROS 시각 |
| `mission_state` | 현재 코스 구간 |
| `control_mode` | 현재 구간에서 선택한 제어기 |
| `selected_source` | Final Driver가 사용할 조향 후보 source |
| `speed_profile` | Final Driver에 요청하는 상징적 속도 profile |
| `stop_required` | Final Driver가 정지를 적용해야 하는지 |

상태, 모드, source와 profile은
[`MissionDecision.msg`](teamkai_interfaces/msg/MissionDecision.msg)에 정의한
`uint8` 상수를 사용한다. 메시지에는 `sequence`, 숫자 조향각, 숫자 속도가 없다.
QoS는 `Reliable`, `Volatile`, `Keep Last 1`이다. Mission Manager의 유일한
publisher이며 `/xycar_motor`와 각 제어기의 조향 후보 토픽을 발행하지 않는다.

### 출발 신호 조건

신호등 인지 모델의 후처리 adapter는 Mission Manager에 원본 영상이나 모델
출력을 직접 넘기지 않고 `UNKNOWN`, `RED`, `YELLOW`, `GREEN` 중 하나와 그
판단의 유효 여부를 전달한다. Mission Manager는 다음 순서에서만 출발한다.

`traffic_light_debug`의 class 의미는 class 4가 `RED`, class 5가 `YELLOW`,
class 1이 `GREEN`, class 2가 `LEFT`다. RED와 YELLOW는 각각
`yolo_red_light_class_ids: [4]`,
`yolo_yellow_light_class_ids: [5]`로 분리한다. HSV 빨간색 검사는 class 4
후보를 확인하는 보조 조건이며 class 5를 RED로 바꾸지 않는다.

한 프레임에서 RED·YELLOW·GREEN 중 둘 이상이 동시에 검출되면 상태는
`UNKNOWN`이다. 충돌한 신호로 RED 또는 GREEN 확인 시간을 누적하지 않으며,
서로 충돌하지 않는 유효 신호가 연속으로 들어와야 다음 조건을 만족한다.

1. 유효한 `RED`가 0.3초 연속 확인되면 출발 대기 조건을 arm한다.
2. arm된 뒤 유효한 `GREEN`이 0.3초 연속 확인되면 `LANE_DRIVING`으로 전환한다.

RED를 확인하기 전에 GREEN만 보이면 출발하지 않는다. RED 확인 전 YELLOW는
RED 확인 시간을 초기화한다. RED 확인 후 YELLOW, UNKNOWN 또는 무효 판단이
들어오면 arm 상태는 유지하되 진행 중이던 GREEN 확인 시간을 초기화하고 계속
정지한다. 모델 confidence 임계값 판정은 인지 adapter가 담당하며, ROS
adapter는 `BLUE` 입력만 `GREEN`으로 정규화한다.

`mission_start_signal_adapter`는 신호등 상태 source의
`/track_drive/traffic_light_debug/state` (`String`)를 구독해 다음 두 토픽을
발행한다.

- `/mission/input/start_signal` (`String`)
- `/mission/input/start_signal_valid` (`Bool`)

`red`, `yellow`, `green`은 각각 `RED`, `YELLOW`, `GREEN`으로 변환한다.
`blue`는 `GREEN`, `none`·`left`·`unknown`은 유효한 `UNKNOWN` 판단으로
변환한다. 알 수 없는 문자열이거나 원본 상태가 기본 0.5초 동안 갱신되지 않으면
`start_signal=UNKNOWN`, `start_signal_valid=false`를 발행한다. adapter는
신호등 추론을 다시 실행하지 않는다.

`safety_ready=false`이거나 `safety_stop_required=true`이면
`WAIT_START_SIGNAL + STOP`을 유지하고 RED/GREEN 확인 이력을 모두 지운다. 수동
`START`는 통합시험 전용이며 실제 대회 출발 조건으로 사용하지 않는다.

GREEN이 확정되는 주기에는 별도의 준비 확인 시간을 기다리지 않고 현재 source
유효성으로 제어기를 즉시 선택한다. `drive_policy_valid=true`이면
`NORMAL_IL`을 가장 먼저 선택한다. 일반 모델 출력이 무효이고
`lane_fallback_valid=true`이면 `LANE_FALLBACK`, 둘 다 무효이면
`LANE_DRIVING + STOP`을 선택한다. 두 source가 모두 유효한 경우에도
`NORMAL_IL`이 우선이다.

`mission_drive_policy_adapter`는 일반 IL 추론 노드의 `/il/policy_debug`
(`Float32MultiArray`)를 구독한다. 실제 추론이 성공해 debug 배열에 최소 4개
값이 있고, 최종 조향 후보인 `data[2]`가 유한한 `-42~42` 범위이며 마지막
수신 후 0.5초가 지나지 않았을 때만
`/mission/input/drive_policy_valid=true`를 발행한다. 시작 전, 잘못된 배열,
최종 조향 후보의 NaN/Inf·범위 이탈 또는 stale이면 즉시 `false`다.

`data[0]`의 정규화 조향, `data[1]`의 raw 조향, `data[3]`의 모델 계산 속도와
그 뒤의 동기화·성능·영상 진단값은 Mission Manager 유효성 판단에 사용하지
않는다.

validity 판단에는 `/il/policy_motor_shadow`를 사용하지 않는다. IL 노드의
watchdog은 센서가 끊긴 뒤에도 이 토픽에 `[0.0, 0.0]` 정지 명령을 발행하므로,
메시지가 계속 온다는 이유만으로 모델 추론이 정상이라고 판단할 수 없기 때문이다.
adapter는 모델을 실행하거나 조향·속도를 계산하지 않고 Bool 유효성만 만든다.

이 토픽의 기준 publisher는 `simulation` 브랜치의
`il_data_tools/policy_inference_node.py`다. 이 노드는 동기화된 카메라·LiDAR
입력으로 추론에 성공했을 때 debug를 갱신한다. 우리 구조에서는 이 source
노드가 `/xycar_motor`를 직접 발행하지 않도록 `drive_enabled:=false`로 실행한다.

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
실차 `my_rule/cone_node` 출력이 모두 정상·최신이고, 카메라 콘이 4개 이상이면서
LiDAR 콘 경로가 준비된 조건이 0.25초 연속 유지되면
`CONE_SECTION + CONE_DRIVE_RULE`로 전환한다. 현재 mode가 `NORMAL_IL`,
`LANE_FALLBACK`, `STOP` 중 무엇인지와 관계없이 같은 조건을 적용한다.

진입 후에는 최소 1.0초 동안 콘 구간을 유지한다. 이후 양쪽 센서 입력이 모두
정상·최신인 상태에서 카메라 콘이 1개 이하이고 LiDAR 콘 존재 근거가 없는
조건이 0.7초 연속 유지되면 `LANE_DRIVING`으로 돌아간다. 센서 입력 무효나
stale은 `콘 없음`으로 취급하지 않는다. 진입 기준 4개와 이탈 기준 1개를
분리해 검출 개수가 경계에서 흔들릴 때 상태가 왕복하지 않게 한다.

Mission Manager는 카메라 영상, `LaserScan`, 숫자 조향각 또는 숫자 속도를 직접
처리하지 않고 adapter가 만든 다음 의미 기반 ROS2 토픽을 구독한다.

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/mission/input/camera_cone_valid` | `std_msgs/Bool` | 최신 카메라 프레임으로 콘 인지가 정상 완료됐는지 |
| `/mission/input/camera_cone_count` | `std_msgs/Int32` | 현재 유효 프레임에서 검출된 cone class 개수 |
| `/mission/input/lidar_cone_source_valid` | `std_msgs/Bool` | 최신 cone command와 cluster 결과를 함께 사용할 수 있는지 |
| `/mission/input/lidar_cone_path_ready` | `std_msgs/Bool` | 진입에 사용할 LiDAR 콘 중앙 경로가 준비됐는지 |
| `/mission/input/lidar_cone_present` | `std_msgs/Bool` | 약한 복구 경로 또는 cluster로 콘 구간 존재가 이어지는지 |

`valid=true`는 콘이 존재한다는 뜻이 아니라 해당 센서 결과를 이번 판단에
사용할 수 있다는 뜻이다. 예를 들어 카메라 추론이 정상이고 콘이 보이지 않으면
`camera_cone_valid=true`, `camera_cone_count=0`이다. 카메라 출력이나 두 LiDAR
source 중 하나가 stale 또는 malformed이면 해당 `valid`를 `false`로 보내며,
Mission Manager는 이를 콘 미검출로 간주하지 않는다.

카메라 콘 count source는 `traffic_light_debug` 노드가 신호등 인지를 위해 한 번
실행한 `final.onnx` 결과를 함께 사용한다. class 0(cone), confidence 0.35 이상인
검출 수를 세어 `/perception/camera_cone_count` (`Int32`)로 발행하므로 콘
count를 위한 별도 YOLO 추론은 실행하지 않는다. 정상 추론에서 콘이 없으면 `0`,
이미지 변환 또는 모델 출력이 무효이면 `-1`을 발행한다.

`mission_camera_cone_adapter`는 이 토픽을 받아 다음 두 Mission 입력을 20 Hz로
발행한다.

- `/mission/input/camera_cone_count` (`Int32`)
- `/mission/input/camera_cone_valid` (`Bool`)

0 이상의 count가 마지막으로 수신된 후 0.2초 이내이면 `valid=true`와 실제
count를 내보낸다. 원본 count가 `-1`이거나 0.2초 동안 갱신되지 않으면 즉시 또는
timeout 시점에 `valid=false`, `count=0`을 내보낸다. Mission Manager는 valid가
false인 count를 진입·이탈 판단에 사용하지 않는다. timeout을 콘 진입 확인시간
0.25초보다 짧게 두어 단 한 프레임만으로 진입 조건이 완성되지 않게 한다.

`/perception/objects`의 traffic cone class는 카메라 콘 count source로 사용하지
않는다. `camera_perception_node`가 이 토픽에 빈 객체 배열을 발행하기 때문이다.

LiDAR 콘 source는 실차의 `my_rule/cone_node`다. 이 노드가 `/scan`을 DBSCAN으로
처리하고 다음 결과를 발행한다.

- `/my_rule/cone_cmd` (`Float32MultiArray`):
  `[physical_angle_deg, requested_speed, confidence]`
- `/my_rule/cone_clusters` (`PoseArray`): `laser_frame`의 콘 cluster 중심
- `/my_rule/cone_path` (`Path`): 참고용 콘 중앙 경로

Mission Manager launch는 motor publisher가 포함된 `my_rule/rule_driver`나
`my_rule.launch.py`를 실행하지 않는다. 실차에서는 `cone_node`만 별도로 실행한다.
`cone_node`는 `/xycar_motor`를 발행하지 않는다.

기준 구현은 `hwj` 브랜치의
`xycar_ws/src/study/my_rule/my_rule/cone_node.py`다. 이 노드의 angle은 Xycar
servo 명령이 아니라 물리 조향각(degree)이므로, 후속 Final Driver가 IL의
조향 후보와 같은 단위로 변환한 뒤 한 번만 motor 명령을 만들어야 한다.

`mission_lidar_cone_adapter`는 `/my_rule/cone_cmd`와
`/my_rule/cone_clusters`를 받고, 두 토픽이 모두 0.2초 이내이며 command가
3개의 유한한 값과 0~1 confidence를 가지는 경우에만
`lidar_cone_source_valid=true`로 만든다.

- `lidar_cone_path_ready=true`: confidence가 0.35 이상
- `lidar_cone_present=true`: confidence가 0.2 초과이거나 cluster가 2개 이상
- command 또는 clusters가 stale/malformed: 세 출력 모두 false

`path_ready`는 콘 구간 진입에만 사용하고, 더 약한 `present`는 이미 들어간 콘
구간의 유지·이탈 판단에 사용한다. 따라서 경로 confidence가 잠깐 낮아져도
cluster가 남아 있으면 조기에 차선 주행으로 돌아가지 않는다. 숫자 angle과 speed는
Mission Manager로 전달하지 않는다.

0.2초 freshness는 0.25초 진입 확인보다 짧다. 따라서 source가 한 번만
발행되고 멈추면 진입 확인이 완성되기 전에 `source_valid=false`가 된다.

콘 조향 source가 일시적으로 무효가 되더라도 이를 MissionState 전이 조건으로
사용하지 않는다. 마지막으로 확정된 상태가 `CONE_SECTION`이면
`CONE_SECTION + CONE_DRIVE_RULE`을 그대로 유지한다.

숫자 조향값의 sample-and-hold는 Mission Manager 기능이 아니라 Final Driver
계약이다. Final Driver는 새 콘 조향값이 유효하고 유한할 때만
`last_valid_steering_angle`을 갱신하고, 무효·stale이면 마지막 유효 조향각을
사용한다. 무효 지속시간에 따른 자동 `STOP`이나 조향 변화율 제한은 두지 않는다.

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
계산하지 않는다. fallback source는 노란선과 흰선의 의미를 위 우선순위대로
처리해야 하며 `/xycar_motor`를 발행하지 않아야 한다.

`mission_lane_fallback_adapter`는 fallback controller가 발행하는
`/perception/centerline` (`kaiev26_msgs/Centerline`)을 구독하고
`/mission/input/lane_fallback_valid` (`Bool`)을 발행한다. 다음 조건을 모두
만족할 때만 `true`다.

- 경로점이 3개 이상이다.
- `confidence`가 0.25 이상 1.0 이하이고 유한한 값이다.
- 모든 경로점의 `x`, `y`, `z`가 유한한 값이다.
- 마지막 Centerline 수신 후 0.4초가 지나지 않았다.

adapter는 수신 시각을 기준으로 freshness를 검사할 뿐 경로를 새로 만들거나
노란선과 흰선의 의미를 재판정하지 않는다. 이 fallback 계약의 실행 조건은
perception 설정의 `use_yellow_as_centerline: true`다.
`use_yellow_as_centerline: false`인 Centerline은 Mission Manager가 전제하는
노란 중앙선 의미와 호환되지 않는다.

`Centerline` 메시지 정의는 `simulation` 브랜치의
`xycar_ws/src/kaiev26_msgs` 패키지에 있다. Mission Manager를 별도 workspace에
배치할 때는 해당 메시지 패키지가 같은 workspace에 함께 빌드되어 있어야 한다.

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
colcon build --symlink-install --packages-up-to track_drive
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

### 4. Source 노드 실행

일반 IL, YOLO Lane Fallback, 콘 경로 source는 각 기준 브랜치의 패키지를 같은
workspace에 둔 뒤 별도로 실행한다. 아래는 기준 executable의 예시다. 실차 launch
인자와 카메라 토픽은 실제 환경에 맞춰야 한다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# simulation 브랜치: 일반 IL. motor 직접 발행은 반드시 비활성화한다.
ros2 launch il_data_tools real_policy_inference.launch.py drive_enabled:=false

# simulation 브랜치: YOLO lane canonical perception.
ros2 launch xycar_perception real_yolo_canonical_perception.launch.py

# hwj 브랜치: LiDAR rule-based 콘 source만 실행한다.
ros2 run my_rule cone_node

# 이 패키지: 출발 신호와 카메라 콘 개수 source.
ros2 launch track_drive traffic_light_debug.launch.py use_rviz:=false
```

`my_rule/rule_driver`, `my_rule.launch.py`, 기존 `track_drive` 주행 노드,
`cone_il` driver는 `/xycar_motor` publisher를 포함하므로 이 구조와 동시에
실행하지 않는다.

### 5. Mission Manager zone·차량 상태 확인

Mission Manager V0.2 통합 노드는 다음 명령으로 실행한다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch track_drive mission_manager_draft.launch.py
```

zone 또는 제어 모드가 바뀌면 `event=change` 로그가 즉시 출력된다. 변화가 없어도
`status_log_period_sec`의 기본값인 1초마다 `event=status` 로그가 출력된다.

주요 필드는 다음 의미다.

| 필드 | 의미 |
|---|---|
| `zone` | 현재 코스 구간인 `MissionState` |
| `control_mode` | 현재 zone에서 선택한 제어기 |
| `selected_source` | Final Driver가 선택해야 할 조향 source |
| `speed_profile` | 요청한 상징적 속도 profile |
| `motion_request` | `DRIVE` 또는 `STOP` 요청 |
| `stop_required` | Final Driver가 정지를 적용해야 하는지 |
| `zone_age_sec`, `mode_age_sec` | 현재 zone과 mode 유지 시간 |
| `*_valid` | 각 입력을 현재 판단에 사용할 수 있는지 |
| `camera_cones` | 현재 카메라 cone 수 |
| `lidar_path_ready`, `lidar_present` | LiDAR 콘 진입 경로와 구간 존재 여부 |

출발 대기 예시:

```text
[MISSION] event=change zone=WAIT_START_SIGNAL control_mode=STOP selected_source=none speed_profile=stop motion_request=STOP stop_required=true zone_age_sec=0.00 mode_age_sec=0.00 drive_policy_valid=false lane_fallback_valid=false camera_cones=0 camera_valid=false lidar_path_ready=false lidar_present=false lidar_valid=false start_signal=UNKNOWN start_valid=false start_armed=false safety_ready=true safety_stop=false override=AUTO lap=0 shortcut_used=false
```

일반 차선 주행 예시:

```text
[MISSION] event=change zone=LANE_DRIVING control_mode=NORMAL_IL selected_source=drive_il speed_profile=normal motion_request=DRIVE stop_required=false zone_age_sec=0.00 mode_age_sec=0.00 drive_policy_valid=true lane_fallback_valid=true camera_cones=0 camera_valid=true lidar_path_ready=false lidar_present=false lidar_valid=true start_signal=GREEN start_valid=true start_armed=false safety_ready=true safety_stop=false override=AUTO lap=0 shortcut_used=false
```

콘 구간 진입 예시:

```text
[MISSION] event=change zone=CONE_SECTION control_mode=CONE_DRIVE_RULE selected_source=cone_rule speed_profile=cone motion_request=DRIVE stop_required=false zone_age_sec=0.00 mode_age_sec=0.00 drive_policy_valid=true lane_fallback_valid=true camera_cones=5 camera_valid=true lidar_path_ready=true lidar_present=true lidar_valid=true start_signal=GREEN start_valid=true start_armed=false safety_ready=true safety_stop=false override=AUTO lap=0 shortcut_used=false
```

`motion_request=DRIVE`는 Mission Manager가 주행 가능한 제어기를 선택했다는 뜻이다.
현재 Mission Manager는 odometry나 실제 차속을 입력받지 않으므로 차량이 물리적으로
움직이고 있다는 측정값은 아니다. 결정은 `/mission/decision`으로 발행하고,
source 상태와 전이 진단은 터미널 로그로 확인한다.

## 코드 계층구조

```text
kookmin_autonomous_competition_teamKAI/
├── assets/models/          # 모델 source archive; V0.2는 final.onnx만 설치
├── cone_il/                # 레거시 CNN 콘 주행 source archive
├── teamkai_interfaces/     # MissionDecision ROS2 메시지 패키지
├── launch/                 # source archive; V0.2 launch 두 개만 설치
├── config/                 # Mission Manager V0.2 파라미터
├── resource/               # ROS2 ament package marker
├── rviz/                   # 디버그 시각화 설정
├── track_drive/            # Mission Manager와 기존 주행 source
│   ├── integration/        # source 노드와 Mission Manager 사이의 freshness adapter
│   └── mission/            # Mission Manager V0.2 결정 계층
├── package.xml             # ROS2 패키지 의존성 정의
├── setup.py                # Python 노드, launch, 모델 파일 설치 설정
├── setup.cfg               # ROS2 Python 실행 파일 설치 경로 설정
├── requirements.txt        # Python 실행 의존성 참고
├── reports/                # 개발 과정과 실험 내용을 정리한 보고서
└── GoogleDrive.txt         # 추가 자료/영상 공유 링크
```

## 저장소 내 레거시·보조 코드

아래 코드는 저장소에 source로 남아 있지만 Mission Manager V0.2의 console
entry point나 설치 launch에는 포함되지 않는다. 특히 `/xycar_motor`를 직접
발행하는 노드는 우리 Final Driver와 동시에 실행하지 않는다.

### track_drive 패키지

- `track_drive/track_drive.py`: 기존 과제 1 통합 주행 노드다. Mission Manager V0.2와 분리된 실행 경로다.
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
- `track_drive/mission/`: V0.2 미션 상태와 허용 제어기 선택을 담당한다.
- `track_drive/integration/`: 일반 모델 등 source 노드의 출력을 Mission Manager용 valid 토픽으로 변환한다.
- `track_drive/package_paths.py`: `model_paths.py`의 경로 함수를 다시 내보내는 레거시 호환 모듈이다.
- `track_drive/utils.py`: 공통 유틸리티 함수 모음이다.
- `track_drive/*_debug_node.py`: 개별 인지 디버그 source다. V0.2 설치 대상은 출발 신호와 카메라 콘 수를 제공하는 `traffic_light_debug`뿐이다.

### cone_il 패키지

- `cone_il/cone_il/cone_ai_driver_node.py`: 이미지를 모델에 입력해 조향각을 예측하고 `/xycar_motor`를 발행하는 기존 독립 노드다. Mission Manager V0.2 launch에는 포함하지 않는다.
- `cone_il/cone_il/preprocess.py`: 이미지를 crop, resize, RGB 변환, 정규화, CHW 텐서 형태로 전처리를 수행한다.
- `cone_il/cone_il/model.py`: CNN 조향 모델 구조를 정의한다.
- `cone_il/cone_il/cone_data_recorder_node.py`: 주행 데이터 수집용 노드이다.
- `cone_il/cone_il/xycar_*_teleop_node.py`: 데이터 수집 또는 수동 조작에 사용하는 키보드 teleop 노드이다.
- `cone_il/scripts/train_cone_bc.py`: 수집된 이미지/조향 데이터를 이용해 CNN 조향 모델을 학습한다.
- `cone_il/scripts/merge_cone_datasets.py`: 여러 주행 데이터셋을 학습용 데이터셋으로 병합한다.

## 저장소에 포함된 레거시 모델

### CNN End-to-End 조향 모델

`assets/models/cone_bc_scripted_*.pt` 파일은 전방 카메라 이미지를 입력으로 받아 조향각을 출력하는 TorchScript CNN 모델이다. 입력 이미지는 하단 도로 영역을 중심으로 crop하고, 고정 크기로 resize한 뒤, RGB 변환과 0~1 정규화를 거쳐 CHW 형태의 float32 텐서로 변환된다. 학습은 사람이 주행하거나 기존 주행 로직으로 얻은 카메라 이미지와 조향각 데이터를 짝지어 진행한다. 모델은 이미지에서 라바콘 배치와 도로 진행 방향을 학습하고, 실제 주행 중에는 매 프레임 조향각을 예측한다. 주행 안정성을 위해 예측 조향각에는 smoothing, 최대 조향각 제한, 조향각 기반 속도 제한을 함께 적용한다.

이 모델들은 레거시 source archive이며 V0.2 `track_drive` 패키지 설치 대상과
실행 의존성에 포함되지 않는다. 현재 콘 source는 `hwj` 브랜치의 LiDAR
rule-based `cone_node`다.

### 객체 인식 모델

`assets/models/final.onnx`는 신호등과 카메라 콘 수를 얻는
`traffic_light_debug`의 객체 인식 모델이다. V0.2 설치에 포함되는 유일한 로컬
모델이며 Mission Manager 자체가 이 모델을 실행하지는 않는다.

## 전체 대회 미션 범위

아래 항목은 저장소의 기존 코드가 목표로 한 전체 대회 범위다. Mission Manager
V0.2가 현재 자동 처리하는 범위는 출발, 일반 차선 주행, 콘 구간뿐이다.

- 출발 신호등 확인 후 주행 시작
- 라바콘 구간에서 CNN End-to-End 기반 조향 주행
- 아스팔트/구불길 구간 통과
- 보행자 회피 및 안전 정지 판단
- 방해차량 인식 및 추월/회피 판단
- 어린이보호구역 노면 표식 인식 및 속도 제한
- 신호등, 정지선, 좌측 라바콘 상태를 조합한 교차로 직진/좌회전 판단
- 총 3바퀴 주행 후 종료

## 참고

Mission Manager V0.2 실행 범위는 ROS2 Humble, Python 3, OpenCV, NumPy,
ONNX Runtime을 사용한다. PyTorch는 저장소의 레거시 `cone_il` source를 별도로
실행하거나 학습할 때만 필요하며 V0.2 runtime 의존성이 아니다.
