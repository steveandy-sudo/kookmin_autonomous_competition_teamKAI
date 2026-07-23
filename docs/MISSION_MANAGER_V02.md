# Team K.A.I. Mission Manager V0.2 설계

## 1. 목적

Mission Manager는 직접 운전하지 않는 결정 계층이다. 현재 코스 미션을
`MissionState`로 기억하고, 그 미션에서 허용된 `ControlMode` 중 하나를
선택하여 후단 selector에 전달한다.

Mission Manager는 다음 작업을 하지 않는다.

- 원본 Image 또는 LaserScan 처리
- YOLO나 모방학습 모델 실행
- 콘·차선·장애물 조향 계산
- 최종 steering angle 또는 숫자 speed 계산
- 모터 제어
- `/xycar_motor` 발행

기존 `track_drive/mission_state.py`와 이 초안은 연결하지 않는다.

### 외부 source-of-truth

V0.2는 외부 알고리즘을 이 패키지로 복사하지 않고 다음 구현의 출력 토픽만
사용한다.

| 기능 | 기준 구현 | 입력 계약 |
|---|---|---|
| 일반 IL | [`simulation`의 `policy_inference_node.py`](https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI/blob/simulation/xycar_ws/src/il_data_tools/il_data_tools/policy_inference_node.py) | `/il/policy_debug`, `drive_enabled:=false` |
| LiDAR 콘 주행 | [`hwj`의 `cone_node.py`](https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI/blob/hwj/xycar_ws/src/study/my_rule/my_rule/cone_node.py) | `/my_rule/cone_cmd`, `/my_rule/cone_clusters` |
| YOLO 차선 인지 | [`simulation`의 `yolo_lane_segmenter.py`](https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI/blob/simulation/xycar_ws/src/xycar_perception/xycar_perception/yolo_lane_segmenter.py) | `/perception/centerline` |

IL의 motor shadow, 콘 numeric command와 perception 원본은 Mission Manager에
전달하지 않는다.

## 2. MissionState와 ControlMode의 관계

`MissionState`는 현재 어느 코스 미션을 수행 중인지 나타낸다.
`ControlMode`는 해당 미션 안에서 실제로 사용할 제어기를 나타낸다.

예를 들어 차선 구간은 하나의 미션이지만 source 상태에 따라 제어기가 바뀐다.

```text
MissionState.LANE_DRIVING + ControlMode.NORMAL_IL
MissionState.LANE_DRIVING + ControlMode.LANE_FALLBACK
MissionState.LANE_DRIVING + ControlMode.STOP
```

일반 모델이 실패해도 코스 미션 자체는 차선 구간이므로 MissionState를 바꾸지
않는다. MissionState가 먼저 허용 가능한 ControlMode 집합을 결정하고,
Mission Manager가 그 안에서 source 유효성과 확인 시간을 사용해 하나를 고른다.

## 3. MissionState

| 상태 | 의미 | V0.2 자동 전이 |
|---|---|---|
| `WAIT_START_SIGNAL` | 출발 신호 대기 | 사용 |
| `LANE_DRIVING` | 일반 차선 주행 구간 | 사용 |
| `CONE_SECTION` | 라바콘 구간 | 사용 |
| `FIXED_OBSTACLE_SECTION` | 고정 장애물 구간 | 구조만 예약 |
| `OVERTAKE_SECTION` | 방해차량 추월 구간 | 구조만 예약 |
| `ROUTE_SELECTION` | 직진·지름길 경로 결정 | 구조만 예약 |
| `SHORTCUT_SECTION` | 지름길 주행 구간 | 구조만 예약 |

`BOOT`, `RACING`, `FINISHED`, `MANUAL_RECOVERY`, `RACE_ABORTED`,
`EMERGENCY_STOP`은 사용하지 않는다. 대회 중 reset 명령을 보낼 수 없는 조건에서
latch 상태는 복구 불가능하므로 안전 정지는 현재 미션을 유지하는 `STOP`으로
처리한다. 경기 완료 상태와 실제 lap 종료 로직은 후속 버전에서 검토한다.

## 4. ControlMode

| 모드 | selected_source | speed_profile |
|---|---|---|
| `STOP` | `none` | `stop` |
| `NORMAL_IL` | `drive_il` | `normal` |
| `LANE_FALLBACK` | `lane_fallback` | `fallback` |
| `CONE_DRIVE_RULE` | `cone_rule` | `cone` |
| `FIXED_OBSTACLE_RULE` | `fixed_obstacle_rule` | `obstacle` |
| `VEHICLE_FOLLOW` | `vehicle_rule` | `vehicle_follow` |
| `VEHICLE_OVERTAKE` | `vehicle_rule` | `vehicle_overtake` |
| `SHORTCUT_RULE` | `shortcut` | `shortcut` |

`ROUTE_SELECTION`은 제어기가 아니라 미션 단계이므로 ControlMode에서
제거했다.

## 5. 상태별 허용 제어기

| MissionState | 허용 ControlMode |
|---|---|
| `WAIT_START_SIGNAL` | `STOP` |
| `LANE_DRIVING` | `NORMAL_IL`, `LANE_FALLBACK`, `STOP` |
| `CONE_SECTION` | `CONE_DRIVE_RULE`, `STOP` |
| `FIXED_OBSTACLE_SECTION` | `FIXED_OBSTACLE_RULE`, `STOP` |
| `OVERTAKE_SECTION` | `NORMAL_IL`, `LANE_FALLBACK`, `VEHICLE_FOLLOW`, `VEHICLE_OVERTAKE`, `STOP` |
| `ROUTE_SELECTION` | `NORMAL_IL`, `LANE_FALLBACK`, `STOP` |
| `SHORTCUT_SECTION` | `NORMAL_IL`, `SHORTCUT_RULE`, `STOP` |

허용되지 않은 조합은 방어적으로 `STOP`으로 바꾼다.

고정 장애물·추월·경로 선택·지름길 상태는 현재 실제 알고리즘과 자동 전이가
없으므로 해당 상태에 진입하더라도 V0.2 코어는 `STOP`을 선택한다. 위 표의
제어기는 후속 구현 계약일 뿐이다.

## 6. 입력·기억·출력

| 모델 | 역할 |
|---|---|
| `MissionObservation` | current-cycle inputs |
| `MissionContext` | memory preserved across cycles |
| `MissionDecision` | controller selection result for the current cycle |

`MissionObservation`은 현재 주기의 의미 신호, bool/count/confidence와
`now_sec`만 담는다. 원본 센서와 이전 상태는 포함하지 않는다. 신호등 입력은
`StartSignal.UNKNOWN/RED/YELLOW/GREEN`과 `start_signal_valid`로 전달하며, 모델
confidence 임계값 적용은 인지 adapter가 담당한다. `safety_stop_required`는
현재 미션 상태를 바꾸지 않고 이번 주기에 정지가 필요한지를 전달한다.

`MissionContext`는 현재 MissionState와 ControlMode, 상태 진입 시각,
확인 timer, integration override, 미래용 lap/shortcut 필드를 기억한다.
전이 원인 문자열이나 전이 이력은 저장하지 않는다.

`MissionDecision`은 다음 값만 반환한다.

1. `mission_state`
2. `control_mode`
3. `selected_source`
4. `speed_profile`
5. `stop_required`

ROS wrapper는 각 20 Hz 판단 결과를 `/mission/decision`
(`teamkai_interfaces/msg/MissionDecision`)에 한 메시지로 발행한다. 메시지의
전송 계약은 다음과 같다.

| 필드 | 타입 | 의미 |
|---|---|---|
| `stamp` | `builtin_interfaces/Time` | 결정 생성 ROS 시각 |
| `mission_state` | `uint8` | 현재 코스 미션 |
| `control_mode` | `uint8` | 선택된 제어기 |
| `selected_source` | `uint8` | Final Driver가 선택할 조향 source |
| `speed_profile` | `uint8` | 요청한 상징적 속도 profile |
| `stop_required` | `bool` | 정지 적용 요청 |

네 개의 `uint8` 필드는 `MissionDecision.msg`에 정의한 상수를 사용한다.
`sequence`, 숫자 조향각과 숫자 속도는 포함하지 않는다. QoS는 `Reliable`,
`Volatile`, `Keep Last 1`이다.

### Final Driver 조향·속도 입력 계약

| 선택 모드 | 조향 입력 | 조향 단위 |
|---|---|---|
| `NORMAL_IL` | `/il/policy_debug`의 `data[2]` | Xycar 조향 명령값 |
| `LANE_FALLBACK` | `/lane_fallback/command`의 `steering_angle_deg` | 물리 조향각 |
| `CONE_DRIVE_RULE` | `/my_rule/cone_cmd`의 `data[0]` | 물리 조향각 |

`/lane_fallback/command`의 타입은
`teamkai_interfaces/msg/LaneFallbackCommand`이며 계약은 다음과 같다.

```text
builtin_interfaces/Time stamp
float32 steering_angle_deg
bool valid
```

Lane Fallback controller는 `/perception/centerline`을 받아 이 메시지를 만든다.
`valid=false`인 angle은 사용하지 않는다. Controller는 구현되어 있으며
Final Driver의 ROS node와 motor 출력은 포함되지 않는다.

`track_drive/final_driver/normal_il_input.py`에는 motor를 발행하지 않는
NORMAL_IL 입력 코어가 있다. `NORMAL_IL + drive_il + normal` 결정에서 fresh
`/il/policy_debug.data[2]`를 Xycar 조향 후보로, `data[3]`을 속도 후보로
반환한다. `NormalIlInputConfig`의 `fallback_speed`에는 임의 기본값이 없으며
실차 튜닝으로 정한 값을 외부에서 반드시 전달해야 한다.

`track_drive/final_driver/lane_fallback_input.py`에는 motor를 발행하지 않는
Lane Fallback 입력 코어가 있다. `LANE_FALLBACK + lane_fallback + fallback`
결정에서 fresh `/lane_fallback/command.steering_angle_deg`를 물리 조향 후보로
반환하며 속도는 `fallback_speed`를 사용한다. `LaneFallbackInputConfig`도
임의 기본 속도를 갖지 않는다. 후속 단일 Final Driver는 ROS parameter 하나로
두 입력 코어에 같은 `fallback_speed`를 전달해야 한다.

`track_drive/final_driver/cone_input.py`에는 motor를 발행하지 않는 콘 입력
선택·검증 코어가 있다. `CONE_DRIVE_RULE + cone_rule + cone` 결정이며
`stop_required=false`일 때만 콘 주행 후보를 반환한다. `ConeInputSelector`는
같은 콘 모드에서 마지막으로 유효했던 조향각을 기억한다.

`track_drive/final_driver/input_selector.py`의 `DriveInputSelector`는 세 입력
코어를 하나로 묶고 MissionDecision이 지정한 source 하나만 반환한다. 공통
`DriveInputSelection` 계약은 다음과 같다.

| 필드 | 의미 |
|---|---|
| `steering_value` | 선택 source의 조향값 |
| `steering_unit` | `XYCAR_COMMAND`, `PHYSICAL_DEG`, 또는 `NONE` |
| `requested_speed` | 선택 source의 숫자 속도 후보 |
| `selected_source` | 실제 선택되었거나 시도된 source |
| `steering_held` | 마지막 유효 조향을 재사용 중인지 |
| `valid` | 후단에서 사용할 수 있는 후보인지 |

STOP 또는 선택 source 후보 부재 시 조향·속도는 0, 단위는 `NONE`,
`valid=false`다. 이 중립값은 motor 정지 명령 자체가 아니며 후단 Final Driver가
MissionDecision과 함께 해석한다. selector는 단위 변환과 motor 발행을 하지
않는다.

`track_drive/final_driver/steering_converter.py`는 선택 결과의 조향 단위를
Xycar command로 정규화한다. IL의 `XYCAR_COMMAND`는 그대로 통과시키고
Lane Fallback과 Cone의 `PHYSICAL_DEG`만 다음 표로 구간별 선형 보간한다.

| 물리 조향각 | Xycar 조향 명령 |
|---:|---:|
| 0° | 0 |
| 4° | 10 |
| 10° | 20 |
| 16° | 30 |
| 26° | 42 |

±26° 밖의 물리 입력은 표의 끝값 ±42로 clamp한다. 좌우 부호는
`physical_steering_sign`으로 설정하며 기본값은 `1.0`이다. 변환 과정은
requested speed, selected source와 held 표시를 그대로 보존한다. smoothing과
조향 변화율 제한은 없고 결과 `FinalDriveCommandCandidate`도 motor를 발행하지
않는다.

Mission Manager는 숫자 속도 대신 상징적 profile만 전달한다. Final Driver는
`SPEED_PROFILE_NORMAL`에서 `/il/policy_debug.data[3]`을 사용한다. 이 값은
신경망이 직접 출력한 속도가 아니라 IL 주행 노드가 `data[2]` 조향 크기와
`speed_command`, `min_speed_command` 파라미터로 계산한 요청 속도다. 두
파라미터의 숫자값은 실차 튜닝 후 정한다.

`SPEED_PROFILE_FALLBACK`에서는 `fallback_speed` 설정값을 사용하며 이 숫자값도
실차 튜닝 후 정한다.
`SPEED_PROFILE_CONE`에서는 cone command의 `data[1]` `requested_speed`를
재계산 없이 사용한다. 현재 입력 코어는 command freshness 0.2초, confidence
0.2 초과, 물리 조향각 ±26도 이내를 요구한다. 속도는 실차 `cone_node`의
유효 범위에 맞춰 9.5 미만을 거부하고 21.0 초과를 21.0으로 제한한다.
`stop_required=true`가 언제나 우선한다.
유효한 command를 한 번 이상 받은 뒤 입력이 무효·stale이면 마지막 조향을
유지하고 요청 속도는 최저 콘 속도 9.5를 사용한다.

## 7. 현재 자동 전이

### 출발

출발 신호는 신호등 인지 모델의 후처리 adapter가 제공한다. Mission Manager는
원본 영상이나 모델 confidence를 직접 처리하지 않는다. 출발 순서는 다음과
같다.

`traffic_light_debug` 후처리의 class 계약은 다음과 같다.

| class ID | 의미 |
|---|---|
| 1 | `GREEN` |
| 2 | `LEFT` |
| 4 | `RED` |
| 5 | `YELLOW` |

RED와 YELLOW는 `yolo_red_light_class_ids: [4]`와
`yolo_yellow_light_class_ids: [5]`로 서로 분리한다. HSV 빨간색 검사는 class 4
후보의 보조 확인에만 사용한다. RED·YELLOW·GREEN 중 두 종류 이상이 같은
프레임에서 검출되면 단일 상태는 `UNKNOWN`이며 RED 또는 GREEN 확인 시간을
누적하지 않는다.

1. `start_signal_valid=true`인 `RED`가 0.3초 유지되면 출발 조건을 arm한다.
2. arm된 뒤 유효한 `GREEN`이 0.3초 유지되면 출발한다.

```text
WAIT_START_SIGNAL + STOP
  -- RED 0.3초 --> armed, 계속 STOP
  -- GREEN 0.3초 --> LANE_DRIVING
```

RED를 먼저 확인하지 않은 GREEN은 출발 조건이 아니다. arm 전 YELLOW는 RED 확인
시간을 초기화한다. arm 후 YELLOW, UNKNOWN 또는 무효 신호는 arm을 유지하되
진행 중인 GREEN 확인 시간을 초기화한다. 따라서 노란불 한 프레임 뒤에 이전
GREEN 확인 시간이 이어지지 않는다. ROS adapter는 `BLUE` 입력만 `GREEN`으로
정규화한다.

`mission_start_signal_adapter`는 기존
`/track_drive/traffic_light_debug/state` (`String`)를 구독하고
`/mission/input/start_signal` (`String`)과
`/mission/input/start_signal_valid` (`Bool`)를 발행한다. `red`, `yellow`,
`green`은 표준 열거형 이름으로 바꾸고 `blue`는 `GREEN`으로 바꾼다.
`none`, `left`, `unknown`은 유효한 `UNKNOWN` 판단이다. 알 수 없는 문자열이나
0.5초 이상 갱신되지 않은 source는 무효 처리한다. 이 adapter는 추론을 실행하지
않고 기존 신호등 노드의 의미 상태와 freshness만 변환한다.

`safety_ready=false`이거나 `safety_stop_required=true`이면
`WAIT_START_SIGNAL + STOP`을 유지하고 RED/GREEN 확인과 arm을 모두 초기화한다.
GREEN이 확정되는 바로 그 주기에 현재 source 유효성을 확인하고 다음 우선순위로
제어기를 선택한다.

1. `drive_policy_valid=true`: `NORMAL_IL`
2. 일반 모델은 무효이고 `lane_fallback_valid=true`: `LANE_FALLBACK`
3. 둘 다 무효: `LANE_DRIVING + STOP`

출발 선택에는 fallback 0.2초 준비 확인과 일반 모델 0.4초 복구 확인을 적용하지
않는다. 두 source가 모두 유효하면 `NORMAL_IL`이 우선이다. 이 확인 시간은
출발한 뒤 source를 전환하거나 복구할 때만 적용한다.
통합시험용 `START`는 RED/GREEN 확인 시간만 건너뛰며 `safety_ready` 조건은
건너뛰지 않는다. 실제 대회 출발에는 수동 명령을 사용하지 않는다.

### 차선에서 콘 구간 진입

다음 조건이 0.25초 유지되어야 한다.

- `camera_cone_valid=true`
- `camera_cone_count >= 4`
- `lidar_cone_source_valid=true`
- `lidar_cone_path_ready=true`

```text
LANE_DRIVING + NORMAL_IL/LANE_FALLBACK/STOP
→ CONE_SECTION + CONE_DRIVE_RULE
```

카메라나 LiDAR 어느 한쪽만 조건을 만족하면 진입하지 않는다. 한 주기라도
조건이 깨지면 0.25초 확인을 처음부터 다시 시작한다. 개수 기준은
`minimum_camera_cone_count` 설정값으로 조정한다.

### 콘 구간 이탈

다음 조건을 모두 만족해야 한다.

- 콘 mode가 1.0초 이상 유지됨
- `camera_cone_valid=true`
- `camera_cone_count <= 1`
- `lidar_cone_source_valid=true`
- `lidar_cone_present=false`
- 위 센서 조건이 0.7초 이상 연속 유지됨

```text
CONE_SECTION
→ LANE_DRIVING
```

이탈 후 mode는 drive policy가 유효하면 `NORMAL_IL`, 아니면 lane source가
유효하면 `LANE_FALLBACK`, 둘 다 무효이면 `STOP`이다. 콘 재진입은
1.0초 cooldown 뒤 새 0.25초 확인을 요구한다. 진입 기준 4개와 이탈 기준
1개를 분리해 히스테리시스를 만들며, 카메라 또는 LiDAR 입력 무효/stale은
콘 미검출로 해석하지 않는다. 이탈 개수 기준은
`maximum_camera_cone_count_for_exit` 설정값으로 조정한다.

### 카메라 콘 개수 publisher

기존 `traffic_light_debug` 노드는 신호등 인지에 사용하는 `final.onnx`의 같은
추론 결과에서 class 0(cone)을 센다. confidence 0.35 이상인 검출을 화면 전체에서
세어 `/perception/camera_cone_count` (`Int32`)로 발행하므로 추론을 한 번 더
실행하지 않는다.

원본 count 계약은 다음과 같다.

- `0` 이상: 해당 카메라 프레임과 YOLO 출력이 유효하며 검출된 콘 개수
- `-1`: 이미지 변환 실패, 모델 실행 실패 또는 malformed 출력

`mission_camera_cone_adapter`가 이 토픽의 의미와 freshness를 Mission 입력으로
변환한다. 0 이상의 count를 받은 뒤 0.2초 이내이면 실제 count와
`camera_cone_valid=true`를 발행한다. `-1`을 받거나 source가 0.2초 이상
stale이면 `camera_cone_count=0`, `camera_cone_valid=false`를 발행한다. 따라서
유효한 0개 검출과 인지 실패를 구분한다. 0.2초 timeout은 0.25초 콘 진입
확인시간보다 짧으므로 한 번 수신한 4개 이상 결과만으로는 진입할 수 없다.

`simulation`의 `/perception/objects` (`PerceptionObjectArray`)에는
`CLASS_TRAFFIC_CONE=4`가 정의되어 있지만 현재 perception 노드는 실제 검출
대신 빈 배열만 발행한다. 빈 배열을 유효한 0개 검출로 오판하지 않기 위해 이번
adapter source로 사용하지 않는다.

### 실차 LiDAR 콘 source adapter

별도 `/scan` detector를 만들지 않는다. 실차에서 검증된
`my_rule/cone_node`가 `/scan`을 처리해 다음 결과를 발행한다.
기준 source는 `hwj` 브랜치의
`xycar_ws/src/study/my_rule/my_rule/cone_node.py`다.

```text
/scan (LaserScan)
→ my_rule/cone_node
├─ /my_rule/cone_cmd (Float32MultiArray)
├─ /my_rule/cone_clusters (PoseArray)
└─ /my_rule/cone_path (Path)

/my_rule/cone_cmd + /my_rule/cone_clusters
→ mission_lidar_cone_adapter
├─ /mission/input/lidar_cone_source_valid (Bool)
├─ /mission/input/lidar_cone_path_ready (Bool)
└─ /mission/input/lidar_cone_present (Bool)
```

`/my_rule/cone_cmd`의 배열 계약은
`[physical_angle_deg, requested_speed, confidence]`다. adapter는 숫자 angle과
speed를 Mission Manager에 전달하거나 계산에 사용하지 않고, 배열 길이·유한값과
confidence만 검사한다. `/my_rule/cone_clusters`는 `laser_frame`의 cluster
중심이며 빈 배열도 유효한 미검출이다.

후속 단일 Final Driver는 `/my_rule/cone_cmd`를 직접 구독한다. 콘 입력 코어는
MissionDecision이 콘 source와 콘 speed profile을 함께 선택했을 때 `data[0]`을
물리 조향 후보로, `data[1]`을 숫자 속도 요청으로 묶어 반환한다. 별도의 콘
속도 토픽이나 두 번째 motor publisher는 만들지 않는다.

`physical_angle_deg`는 IL의 Xycar 조향 명령과 단위가 다르다. 후속 Final
Driver가 source별 단위를 공통 단위로 변환한 뒤 최종 motor angle을 한 번만
계산해야 한다.

두 source가 모두 0.2초 이내이고 정상일 때만 `source_valid=true`다. 이
freshness는 0.25초 진입 확인보다 짧으므로 한 번 받은 source 값만으로는
콘 구간 진입을 완성할 수 없다.

- `path_ready=true`: command confidence `>= 0.35`
- `present=true`: command confidence `> 0.2` 또는 cluster 개수 `>= 2`
- source 하나라도 stale/malformed: 세 출력 모두 `false`

진입에는 더 엄격한 `path_ready`를 사용한다. 이미 콘 구간에 들어간 뒤에는
약한 recovery command나 cluster만 남아 있어도 `present=true`로 유지한다.
이렇게 해야 콘 중앙 경로가 잠깐 끊긴 순간을 구간 이탈로 오인하지 않는다.

`mission_manager_draft.launch.py`는 `my_rule/cone_node`를 자동 실행하지 않는다.
실차 source 노드처럼 별도로 실행한다. motor publisher가 포함된
`my_rule/rule_driver`와 이를 포함하는 `my_rule.launch.py`는 현재 구조에서
실행하지 않는다.

### 콘 조향 출력 무효

무효 입력은 상태 전이 근거로 사용하지 않는다. 마지막으로 확정된 상태가
`CONE_SECTION`이면 콘 조향 source가 일시적으로 무효·stale이 되어도
`CONE_SECTION + CONE_DRIVE_RULE`을 유지한다.

숫자 조향값 유지 정책은 Mission Manager가 아니라
`track_drive.final_driver.cone_input.ConeInputSelector`가 담당한다. 선택된 콘
source의 새 조향값이 유효하고 유한할 때만 `last_valid_steering_angle`을
갱신한다. 무효이면 마지막 유효 조향각과 최저 콘 속도 9.5를 반환하고
`steering_held=true`로 표시한다.

무효 지속시간에 따른 자동 `STOP`은 적용하지 않는다. 새 유효 조향은 조향 변화율
제한 없이 즉시 반영한다. 유효한 콘 명령을 한 번도 받지 않았다면 유지할 값이
없으므로 후보를 반환하지 않는다. 콘 모드 이탈 또는 정지 결정에서는 저장된
조향각을 초기화하여 다음 콘 구간에서 이전 값을 재사용하지 않는다. 이 정책은
Mission Manager의 `MissionDecision` 다섯 필드를 변경하지 않는다.

### 차선 source 전환

- YOLO lane source가 이미 0.2초 이상 준비되어 있으면 첫 실패 주기에
  `LANE_FALLBACK`으로 전환한다.
- 대체 source가 준비되지 않았으면 마지막 `NORMAL_IL` 또는 `LANE_FALLBACK`
  mode와 selected source를 최대 1.0초 유지한다.
- 이 유예 중에는 `speed_profile=fallback`을 요청한다. `NORMAL_IL` 경로에서는
  `NormalIlInputSelector`가 마지막 유효 `data[2]`와 `fallback_speed`를
  반환한다. `LANE_FALLBACK` 경로에서는 `LaneFallbackInputSelector`가 마지막
  유효 물리 조향각과 같은 `fallback_speed`를 반환한다.
- 첫 무효 시점부터 정확히 1.0초 이상 두 source가 모두 무효이면
  `LANE_DRIVING + STOP`으로 전환한다.
- 1.0초 안에 마지막 source가 복구되거나 다른 source가 준비되면 loss timer를
  초기화하고 정상 profile 또는 새 source로 복귀한다.
- 출발 시점부터 유효한 source가 없어 마지막 유효 조향이 없으면 즉시
  `LANE_DRIVING + STOP`을 선택한다.
- 정지 중 YOLO lane source가 0.2초 유효하면 `LANE_FALLBACK`으로 복구한다.
- 일반 model이 0.4초 연속 회복되면 `NORMAL_IL`로 복귀한다.

일반 model이 정상인 동안에도 fallback 준비 시간을 백그라운드에서 누적한다.
실제 freshness, inference 성공, 유한한 조향 후보 판정은 각 source adapter가
수행하고 Mission Manager에는 최종 valid만 전달한다. Mission Manager는 숫자
조향각을 저장하지 않고 mode, selected source, speed profile과 STOP 여부만
결정한다.

일반 모델의 `mission_drive_policy_adapter`는 성공 추론 때만 갱신되는
`/il/policy_debug` (`Float32MultiArray`)를 구독한다. 배열에 최소 4개 값이 있고
최종 조향 후보인 `data[2]`가 유한한 `-42~42` 범위이며 마지막 수신 후
0.5초 이내일 때만 `/mission/input/drive_policy_valid=true`를 발행한다.
시작 전, 잘못된 배열, 최종 조향 후보의 NaN/Inf·범위 이탈 또는 stale이면
`false`다. 정규화 조향, raw 조향, IL 주행 노드가 계산한 `data[3]` 속도와
뒤의 진단값은 Mission Manager validity 판단에 사용하지 않는다. `data[3]`은
`NormalIlInputSelector`의 정상 IL 속도 후보로 사용한다. 속도 범위나 clamp는
적용하지 않고 numeric command 경계에서 NaN/Inf만 거부한다.
`/il/policy_motor_shadow`는 센서 timeout 뒤에도
watchdog의 `[0.0, 0.0]` 정지 명령이 발행되므로 validity source로 사용하지
않는다. adapter는 추론 또는 numeric command 계산을 수행하지 않는다.

기준 publisher는 `simulation` 브랜치의
`xycar_ws/src/il_data_tools/il_data_tools/policy_inference_node.py`다. 동기화된
카메라·LiDAR 입력으로 추론에 성공한 경우에만 debug가 갱신된다. 이 구조에서는
source 노드가 `/xycar_motor`를 발행하지 않도록 `drive_enabled:=false`로
실행해야 한다.

### YOLO Lane Fallback source 계약

`simulation` 브랜치의 YOLO lane model은 `YOLO11n-seg 512`이며 흰색 차선과
노란 중앙선을 각각 class 0, class 1 mask로 출력한다. 노란선은 흰색 차선 사이의
주행 중앙이므로 목표 경로는 다음 우선순위로 외부 fallback controller가 만든다.

1. 유효한 노란 중앙선 자체
2. 노란선이 없을 때 양쪽 흰선의 중간
3. 둘 중 어느 조건도 충족하지 못하면 무효

흰선은 노란 중앙선의 geometry 검증과 노란선 소실 시 두 경계의 중앙 경로 생성에
사용한다. 흰선 하나만으로 추정한 경로는 사용하지 않는다. mask freshness,
confidence, 최소 경로 점 수, 차선 폭과 좌우 관계, 유한한 controller 출력이 모두
유효할 때만 `lane_fallback_valid=true`가 된다. 이 controller는 조향 후보를
selector에 제공하지만 `/xycar_motor`를 발행하지 않는다. Mission Manager는
YOLO 추론, mask/BEV 처리, 경로 생성, Pure Pursuit와 숫자 조향 계산을 하지 않는다.

`mission_lane_fallback_adapter`는 `/perception/centerline`
(`kaiev26_msgs/Centerline`)과 `/perception/road_segments`
(`kaiev26_msgs/RoadSegmentArray`), `/lane_fallback/command`
(`teamkai_interfaces/LaneFallbackCommand`)를 구독하고 다음 조건을 검사한다.

- `points` 3개 이상
- 유한한 `confidence`가 0.25 이상 1.0 이하
- 모든 point의 `x`, `y`, `z`가 유한함
- 유효한 노란 선분 1개 이상 또는 유효한 흰 선분 2개 이상
- 두 입력 모두 adapter의 마지막 수신 시각으로부터 0.4초 이내
- `LaneFallbackCommand.valid=true`
- 조향각이 유한한 `±26°` 범위이며 command 수신 후 0.2초 이내

모두 만족하면 `/mission/input/lane_fallback_valid=true`, 하나라도 실패하거나
stale이면 `false`를 발행한다. adapter는 Centerline을 생성하거나 조향값을
계산하지 않는다.

`lane_fallback_controller`는 유효한 Centerline에서 가까운 `0.70 m`와 먼
`1.45 m` 목표점의 Pure Pursuit 조향각을 계산해
`/lane_fallback/command`를 발행한다. 두 조향각은 가까운 값 35%, 먼 값 65%로
합성한다. 초기값은 `wheelbase_m=0.33`, `steering_gain=1.0`,
`max_steering_angle_deg=26.0`이다. 최소 `0.70 m` 전방 경로가 없거나
`base_footprint` frame, confidence, 좌표 또는 freshness가 무효이면
`valid=false`다. Controller는 속도, `/xycar_motor`, 조향 변화율 제한을
사용하지 않는다.

`lane_fallback_valid`는 perception 경로, 색상별 선분 증거와 Controller 조향
출력이 모두 사용 가능할 때만 참이다.

`LaneFallbackInputSelector`는 command가 유효하고 0.2초 이내일 때
`steering_angle_deg`를 갱신한다. 같은 `LANE_FALLBACK` mode에서 command가
무효·stale이면 마지막 유효 각도와 `fallback_speed`를 반환한다. 별도 유예
타이머는 만들지 않으며 Mission Manager가 mode를 바꾸거나 STOP을 결정하면
저장 각도를 초기화한다.

`config/lane_fallback_perception_override.yaml`은 기본 perception 설정 뒤에
적용하며 `centerline_mode: yellow_centerline`,
`use_yellow_as_centerline: true`를 지정한다. perception 알고리즘 자체는
수정하지 않는다.

메시지 정의를 제공하는 `simulation/xycar_ws/src/kaiev26_msgs` 패키지는 실제
ROS2 workspace에 함께 설치·빌드되어 있어야 한다. `track_drive/package.xml`은
이를 runtime dependency로 선언한다.

## 8. 복구 가능한 안전 정지

`safety_stop_required=true`이면 MissionState는 바꾸지 않고 다음을 선택한다.

```text
현재 MissionState + ControlMode.STOP + stop_required=true
```

이 정지는 latch되지 않는다. 입력이 해제되면 같은 MissionState에서 source 정상
확인 시간을 다시 만족한 뒤 자동 복구한다. `EMERGENCY_STOP` MissionState와
`EMERGENCY_STOP`/`RESET_EMERGENCY` 수동 명령은 없다. 물리 비상 차단은 Mission
Manager와 별개로 Safety Supervisor 또는 유일한 Final Driver 아래에 둔다.

## 9. ROS2 통합시험 adapter

Mission Manager 노드 이름은 `mission_manager`, 주기는 20 Hz다. 출발 신호
adapter `mission_start_signal_adapter`, 일반 모델 adapter
`mission_drive_policy_adapter`, Centerline 유효성 adapter
`mission_lane_fallback_adapter`, 카메라 콘 adapter
`mission_camera_cone_adapter`, LiDAR adapter
`mission_lidar_cone_adapter`가 같은 launch 파일에서 함께 시작한다. 실제
`traffic_light_debug`·IL·YOLO lane perception·`my_rule/cone_node` source 노드는
별도로 실행해야 한다. Mission Manager 입력은 다음과 같다.

`my_rule/rule_driver`처럼 `/xycar_motor`를 직접 발행하는 외부 주행 노드는
향후 단일 Final Driver와 함께 실행하지 않는다.

- `/mission/override`
- `/mission/input/safety_stop_required` (`Bool`)
- `/mission/input/start_signal` (`String`: `UNKNOWN`, `RED`, `YELLOW`, `GREEN`)
- `/mission/input/start_signal_valid` (`Bool`)
- `/mission/input/safety_ready` (`Bool`)
- `/mission/input/drive_policy_valid`
- `/mission/input/lane_fallback_valid`
- `/mission/input/camera_cone_valid` (`Bool`)
- `/mission/input/camera_cone_count` (`Int32`)
- `/mission/input/lidar_cone_source_valid` (`Bool`)
- `/mission/input/lidar_cone_path_ready` (`Bool`)
- `/mission/input/lidar_cone_present` (`Bool`)

wrapper는 `/mission/decision` publisher 하나만 가진다. zone 또는 mode가 바뀌면
`event=change`를 즉시 기록하고, 변화가 없어도 기본 1초마다 `event=status`를
기록한다. 로그에는 현재 MissionDecision과 입력 source 상태가 함께 들어간다.
Mission Manager는 `/xycar_motor`를 발행하지 않는다.

```text
[MISSION] event=change zone=LANE_DRIVING control_mode=NORMAL_IL selected_source=drive_il speed_profile=normal motion_request=DRIVE stop_required=false zone_age_sec=0.00 mode_age_sec=0.00 drive_policy_valid=true lane_fallback_valid=true camera_cones=0 camera_valid=true lidar_path_ready=false lidar_present=false lidar_valid=true start_signal=GREEN start_valid=true start_armed=false safety_ready=true safety_stop=false override=AUTO lap=0 shortcut_used=false
[MISSION] event=change zone=CONE_SECTION control_mode=CONE_DRIVE_RULE selected_source=cone_rule speed_profile=cone motion_request=DRIVE stop_required=false zone_age_sec=0.00 mode_age_sec=0.00 drive_policy_valid=true lane_fallback_valid=true camera_cones=5 camera_valid=true lidar_path_ready=true lidar_present=true lidar_valid=true start_signal=GREEN start_valid=true start_armed=false safety_ready=true safety_stop=false override=AUTO lap=0 shortcut_used=false
```

`zone`은 MissionState, `control_mode`는 선택된 제어기다. `motion_request`와
`stop_required`는 후단에 대한 요청이며 실제 차량 속도 측정값은 아니다. 실제
이동 여부를 표시하려면 후속 버전에서 odometry 또는 encoder feedback 계약이
별도로 필요하다.

## 10. 빌드와 테스트

```bash
cd ~/xycar_ws
colcon build --symlink-install --packages-up-to track_drive
source install/setup.bash
colcon test --packages-select track_drive
colcon test-result --verbose
```

순수 Python 테스트:

```bash
cd ~/xycar_ws/src/kookmin_autonomous_competition_teamKAI
python3 -m unittest discover -s test -v
```

통합시험 실행:

```bash
ros2 launch track_drive mission_manager_draft.launch.py
```

## 11. 남은 TODO

- Lane Fallback perception override를 적용한 Centerline의 실차 검증
- 실차 LiDAR rosbag으로 cone cluster 파라미터 검증·조정
- 실차 튜닝으로 공용 `fallback_speed` 확정
- 바퀴 공중 시험으로 `physical_steering_sign`과 조향 보정표 검증
- 고정 장애물 미션 진입·이탈과 전용 controller 연결
- 추월 구간 진입, follow/overtake 전환, 차선 복귀
- 신호등 기반 경로 선택과 3바퀴 중 한 번의 지름길
- 실제 lap crossing debounce
- Safety Supervisor의 recoverable stop 입력과 물리 차단 경로 분리
- 유일한 Final Driver ROS node와 `/xycar_motor` publisher 연결

후속 구현에서도 Mission Manager 자체에는 `/xycar_motor` publisher를
추가하지 않는다.
