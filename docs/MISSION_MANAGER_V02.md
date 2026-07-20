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
`StartSignal.UNKNOWN/RED/YELLOW/GO`와 `start_signal_valid`로 전달하며, 모델
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

## 7. 현재 자동 전이

### 출발

출발 신호는 신호등 인지 모델의 후처리 adapter가 제공한다. Mission Manager는
원본 영상이나 모델 confidence를 직접 처리하지 않는다. 출발 순서는 다음과
같다.

1. `start_signal_valid=true`인 `RED`가 0.3초 유지되면 출발 조건을 arm한다.
2. arm된 뒤 유효한 `GO`가 0.3초 유지되면 출발한다.

```text
WAIT_START_SIGNAL + STOP
  -- RED 0.3초 --> armed, 계속 STOP
  -- GO  0.3초 --> LANE_DRIVING
```

RED를 먼저 확인하지 않은 GO는 출발 조건이 아니다. arm 전 YELLOW는 RED 확인
시간을 초기화한다. arm 후 YELLOW, UNKNOWN 또는 무효 신호는 arm을 유지하되
진행 중인 GO 확인 시간을 초기화한다. 따라서 노란불 한 프레임 뒤에 이전 GO
확인 시간이 이어지지 않는다. ROS adapter는 `BLUE`와 `GREEN` 입력도 `GO`로
정규화한다.

`safety_ready=false`이거나 `safety_stop_required=true`이면
`WAIT_START_SIGNAL + STOP`을 유지하고 RED/GO 확인과 arm을 모두 초기화한다.
GO가 확정되는 바로 그 주기에 현재 source 유효성을 확인하고 다음 우선순위로
제어기를 선택한다.

1. `drive_policy_valid=true`: `NORMAL_IL`
2. 일반 모델은 무효이고 `lane_fallback_valid=true`: `LANE_FALLBACK`
3. 둘 다 무효: `LANE_DRIVING + STOP`

출발 선택에는 fallback 0.2초 준비 확인과 일반 모델 0.4초 복구 확인을 적용하지
않는다. 두 source가 모두 유효하면 `NORMAL_IL`이 우선이다. 이 확인 시간은
출발한 뒤 source를 전환하거나 복구할 때만 적용한다.
통합시험용 `START`는 RED/GO 확인 시간만 건너뛰며 `safety_ready` 조건은
건너뛰지 않는다. 실제 대회 출발에는 수동 명령을 사용하지 않는다.

### 차선에서 콘 구간 진입

다음 조건이 0.25초 유지되어야 한다.

- `camera_cone_valid=true`
- `camera_cone_count >= 4`
- `lidar_cone_valid=true`
- `lidar_cone_detected=true`

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
- `lidar_cone_valid=true`
- `lidar_cone_detected=false`
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

### 콘 조향 출력 무효

무효 입력은 상태 전이 근거로 사용하지 않는다. 마지막으로 확정된 상태가
`CONE_SECTION`이면 콘 조향 source가 일시적으로 무효·stale이 되어도
`CONE_SECTION + CONE_DRIVE_RULE`을 유지한다.

숫자 조향값 유지 정책은 Mission Manager가 아니라 후단의 유일한 Final Driver가
담당한다. Final Driver는 선택된 콘 source의 새 조향값이 유효하고 유한할 때만
`last_valid_steering_angle`을 갱신하고, 무효이면 마지막 유효 조향각을 그대로
재사용한다. 무효 지속시간에 따른 자동 `STOP`과 조향 변화율 제한은 적용하지
않는다. 이 sample-and-hold 정책은 조향각에만 적용하며 Mission Manager의
`MissionDecision` 다섯 필드는 변경하지 않는다.

### 차선 source 전환

- 일반 model의 입력 단절·추론 실패 같은 hard-invalid는 첫 주기에
  `NORMAL_IL` 선택을 해제한다.
- YOLO lane source가 이미 0.2초 이상 준비되어 있으면 첫 실패 주기에
  `LANE_FALLBACK`으로 전환한다.
- fallback이 아직 준비되지 않았으면 `LANE_DRIVING + STOP`으로 기다린다.
- 정지 중 YOLO lane source가 0.2초 유효하면 `LANE_FALLBACK`으로 복구한다.
- 일반 model이 0.4초 연속 회복되면 `NORMAL_IL`로 복귀한다.
- 선택된 fallback 자체가 hard-invalid이면 즉시 `STOP`한다.

일반 model이 정상인 동안에도 fallback 준비 시간을 백그라운드에서 누적한다.
따라서 확인 시간 동안 무효인 일반 모델 출력을 계속 사용하지 않는다. 실제
freshness, inference 성공, 유한한 조향 후보 판정은 각 source adapter가 수행하고
Mission Manager에는 최종 valid만 전달한다.

### YOLO Lane Fallback source 계약

`simulation` 브랜치의 YOLO lane model은 `YOLO11n-seg 512`이며 흰색 차선과
노란 중앙선을 각각 class 0, class 1 mask로 출력한다. 노란선은 흰색 차선 사이의
주행 중앙이므로 목표 경로는 다음 우선순위로 외부 fallback controller가 만든다.

1. 유효한 노란 중앙선 자체
2. 노란선이 없을 때 양쪽 흰선의 중간
3. 흰선 하나와 검증된 예상 반폭으로 추정한 중앙

흰선은 노란 중앙선의 geometry 검증과 소실 시 복구에 사용한다. mask freshness,
confidence, 최소 경로 점 수, 차선 폭과 좌우 관계, 유한한 controller 출력이 모두
유효할 때만 `lane_fallback_valid=true`가 된다. 이 controller는 조향 후보를
selector에 제공하지만 `/xycar_motor`를 발행하지 않는다. Mission Manager는
YOLO 추론, mask/BEV 처리, 경로 생성, Pure Pursuit와 숫자 조향 계산을 하지 않는다.

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

노드 이름은 `mission_manager`, 주기는 20 Hz다. 임시 입력은 다음과 같다.

- `/mission/override`
- `/mission/input/safety_stop_required` (`Bool`)
- `/mission/input/start_signal` (`String`: `UNKNOWN`, `RED`, `YELLOW`, `GO`)
- `/mission/input/start_signal_valid` (`Bool`)
- `/mission/input/safety_ready` (`Bool`)
- `/mission/input/drive_policy_valid`
- `/mission/input/lane_fallback_valid`
- `/mission/input/camera_cone_valid` (`Bool`)
- `/mission/input/camera_cone_count` (`Int32`)
- `/mission/input/lidar_cone_valid` (`Bool`)
- `/mission/input/lidar_cone_detected` (`Bool`)

wrapper에는 publisher가 없다. 상태 또는 mode가 바뀌면 현재 값만 기록한다.

```text
[MISSION] state=LANE_DRIVING mode=NORMAL_IL
[MISSION] state=CONE_SECTION mode=CONE_DRIVE_RULE
```

## 10. 빌드와 테스트

```bash
cd ~/xycar_ws
colcon build --symlink-install --packages-select track_drive cone_il
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

- source freshness와 hard-invalid validator
- 콘 source validity/freshness 전달과 Final Driver의 last-valid 조향 유지
- 고정 장애물 미션 진입·이탈과 전용 controller 연결
- 추월 구간 진입, follow/overtake 전환, 차선 복귀
- 신호등 기반 경로 선택과 3바퀴 중 한 번의 지름길
- 실제 lap crossing debounce
- YOLO lane controller의 candidate/freshness/valid adapter 연결
- Safety Supervisor의 recoverable stop 입력과 물리 차단 경로 분리
- 후단 policy selector와 유일한 final driver 연결

후속 구현에서도 Mission Manager 자체에는 `/xycar_motor` publisher를
추가하지 않는다.
