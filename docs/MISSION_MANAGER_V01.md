# Team K.A.I. Mission Manager V0.1 설계

## 1. 목적과 경계

Mission Manager는 대회의 전체 주행 알고리즘이 아니라 **결정 계층**이다.
현재 고수준 미션 상태와 하위 제어 모드를 관리하고, 후단 selector가 사용할
조향 source와 정성적인 speed profile, 정지 필요 여부만 반환한다.

Mission Manager는 다음 작업을 하지 않는다.

- 원본 Image 또는 LaserScan 처리
- YOLO, ResNet18, 모방학습 모델 실행
- 콘 또는 차선 조향 계산
- 교통 신호 검출
- 장애물 회피 및 차량 추월
- 실제 lap 계산
- 최종 조향각 또는 숫자 속도 계산
- 모터 제어 또는 모터 토픽 발행

기존 `track_drive` 패키지에는 주행·인지 모듈이 있지만 각 steering source의
공통 runtime 계약은 아직 확정되어 있지 않다. V0.1은 기존 알고리즘을 수정하거나
직접 실행하지 않고, 유효성 입력과 상징적인 source 이름으로 연결 계약을 먼저 정의한다.

## 2. MissionState와 ControlMode를 분리하는 이유

`MissionState`는 차량이 출발 대기, 경기 주행, 긴급 정지 중 어느 단계에 있는지
나타낸다. `ControlMode`는 `RACING` 안에서 어느 하위 controller를 사용할지
나타낸다. 두 개를 분리하면 긴급 정지 같은 안전 상태와 IL/콘/차선 선택 같은
제어 정책을 섞지 않고 각각 독립적으로 확장할 수 있다.

### MissionState

| 상태 | 의미 | 정지 규칙 |
|---|---|---|
| `WAIT_START_SIGNAL` | 노드 시작 후 출발 신호 대기 | 항상 정지 |
| `RACING` | 정상 대회 주행 | `ControlMode`에 따라 결정 |
| `EMERGENCY_STOP` | latch되는 안전 정지 | 명시적 reset 전까지 항상 정지 |

V0.1에는 `BOOT`와 `FINISHED`를 사용하지 않는다. 초기화는
`WAIT_START_SIGNAL`에서 시작하고, lap/shortcut 정보가 생기더라도 이번
버전에서 자동으로 경기를 종료하지 않는다.

### ControlMode

| 모드 | V0.1 상태 | selected_source | speed_profile |
|---|---|---|---|
| `STOP` | 사용 | `none` | `stop` |
| `NORMAL_IL` | 사용 | `drive_il` | `normal` |
| `CONE_DRIVE_RULE` | 사용 | `cone_rule` | `cone` |
| `LANE_FALLBACK` | 사용 | `lane_fallback` | `fallback` |
| `FIXED_OBSTACLE_RULE` | placeholder | `fixed_obstacle_rule` 예약 | `obstacle` 예약 |
| `VEHICLE_FOLLOW` | placeholder | `vehicle_rule` 예약 | `vehicle_follow` 예약 |
| `VEHICLE_OVERTAKE` | placeholder | `vehicle_rule` 예약 | `vehicle_overtake` 예약 |
| `ROUTE_SELECT` | placeholder | 후속 결정 | 후속 결정 |
| `SHORTCUT` | placeholder | `shortcut` 예약 | `shortcut` 예약 |

마지막 다섯 모드는 enum 자리만 있으며 V0.1 자동 전이와 수동 명령에서 선택되지
않는다. 특히 장애물 회피나 추월 로직을 구현한 것이 아니다.

## 3. 세 데이터 모델

| 모델 | 역할 |
|---|---|
| `MissionObservation` | current-cycle inputs |
| `MissionContext` | memory preserved across cycles |
| `MissionDecision` | controller selection result for the current cycle |

### MissionObservation

현재 update 주기의 의미 기반 입력만 담는 frozen dataclass다. 시간 기반 판정은
반드시 `now_sec`를 사용한다. 이전 상태, 지속 timer, Image, LaserScan은 넣지
않는다. perception 모듈이 원본 센서를 처리한 뒤 bool/count/confidence로
변환해야 한다.

### MissionContext

여러 update 사이에서 공유되는 영속 메모리다.

- `cone_seen_since`: 유효한 콘 검출이 시작된 시각
- `mode_enter_sec`: 최소 mode dwell time 판정 기준
- `manual_override`: `AUTO` 또는 강제 시험 모드
- `start_signal_seen_since`, `cone_missing_since`, `drive_valid_since`,
  `lane_fallback_valid_since`: 연속 조건 확인 timer
- `lap_count`, `shortcut_used`: 후속 대회 로직용 예약 필드

`MissionContext`에는 전이 원인이나 원인 문자열, 전이 이력을 저장하지 않는다.
이 정보는 현재 source 선택에 필요하지 않고 불필요한 상태 결합을 만들기
때문이다. 제공된 context 스키마에 cone exit 전용 시각이 없으므로, 콘 재진입
cooldown 기준 하나는 `MissionManager` 내부의 비공개 시각값으로만 보존한다.

### MissionDecision

현재 주기에 대한 다음 다섯 값만 담는 frozen dataclass다.

1. `mission_state`
2. `control_mode`
3. `selected_source`
4. `speed_profile`
5. `stop_required`

실제 steering angle과 숫자 speed는 포함하지 않는다. `speed_profile`은 후단
planner가 해석할 이름일 뿐이다.

## 4. 자동 전이 규칙

모든 hold, dwell, cooldown 판정은 frame 수가 아니라 `now_sec`의 차이를
`>=`로 비교한다.

### E-stop

어느 상태에서든 실제 E-stop 입력 또는 수동 `EMERGENCY_STOP`을 받으면 즉시
`EMERGENCY_STOP + STOP`으로 간다. START와 일반 mode override는 무시된다.
실제 입력이 해제된 뒤 명시적인 `RESET_EMERGENCY`가 와야
`WAIT_START_SIGNAL + STOP`으로 돌아간다. Reset 직후 자동 출발하지 않는다.

### 출발

`start_signal_go=true`가 0.3초 연속 유지되면 `RACING`으로 간다. 시작 mode는
drive policy가 유효하면 `NORMAL_IL`, 아니면 lane fallback이 유효하면
`LANE_FALLBACK`, 둘 다 아니면 `STOP`이다. 짧은 신호는 timer를 충족하지
못하며 false가 되면 확인 시간이 초기화된다. 수동 `START`는 E-stop이 아닐 때
확인 시간을 건너뛴다.

### NORMAL_IL → CONE_DRIVE_RULE

다음 전체 조건이 0.25초 연속 유지되어야 한다.

- `cone_detected=true`
- `cone_count >= 2`
- `cone_confidence >= 0.5`

한 frame의 검출이나 낮은 count/confidence는 전이를 만들지 않는다.

### CONE_DRIVE_RULE 이탈

다음 조건을 모두 만족해야 한다.

- 현재 콘 mode가 1.0초 이상 유지됨
- `cone_exit_ready=true`
- 콘 미검출이 0.7초 이상 연속 유지됨

이탈 시 drive policy가 유효하면 `NORMAL_IL`, 아니면 lane fallback이 유효하면
`LANE_FALLBACK`, 둘 다 아니면 `STOP`이다. 짧은 검출 dropout은 콘 mode를
끝내지 않는다. 이탈 후 1.0초 동안 자동 콘 재진입을 막고, cooldown이 끝난
뒤에도 0.25초 진입 확인을 새로 충족해야 한다.

### NORMAL_IL ↔ LANE_FALLBACK

- drive policy가 무효이고 lane fallback이 유효한 상태가 0.2초 연속되면
  `LANE_FALLBACK`으로 간다.
- lane mode에서 drive policy가 다시 0.4초 연속 유효하면 `NORMAL_IL`로 간다.
- 두 source가 모두 무효이면 `STOP`이다.
- `RACING + STOP`에서 source가 회복되면 확인된 콘, drive IL, lane fallback,
  STOP 순서로 선택한다.

짧은 invalid/recovery 구간에서는 현재 mode를 유지해 단일 입력 흔들림으로
mode가 바뀌지 않게 한다. 최종 driver에는 별도의 source freshness watchdog이
필요하다.

## 5. 수동 override

토픽은 `/mission/override`, 타입은 `std_msgs/msg/String`이다.

| 명령 | 동작 |
|---|---|
| `AUTO` | 현재 MissionState에서 자동 mode 선택으로 복귀 |
| `START` | E-stop이 아니면 즉시 `RACING` 진입하는 one-shot action |
| `NORMAL_IL` | RACING에서 강제, drive policy 무효이면 STOP |
| `CONE_DRIVE_RULE` | RACING 통합시험용 강제 mode |
| `LANE_FALLBACK` | RACING에서 강제, lane 무효이면 STOP |
| `STOP` | MissionState는 유지하고 mode만 STOP |
| `EMERGENCY_STOP` | latch되는 EMERGENCY_STOP 진입 |
| `RESET_EMERGENCY` | 실제 E-stop 해제 후 WAIT+STOP 복귀 |

`NORMAL_IL`, `CONE_DRIVE_RULE`, `LANE_FALLBACK`, `STOP`은 WAIT 상태 자체를
RACING으로 바꾸지 않는다. E-stop 중에는 reset 외 명령이 무시된다. 현재
Observation에는 cone controller validity 필드가 없으므로 수동 콘 mode는
controller가 통합되어 있다는 시험 전제 아래 강제된다.

`START`와 `RESET_EMERGENCY`는 다음 20 Hz update에서 한 번만 소비된다.
`START`는 기존 지속 override를 유지하므로 같은 주기 안에 `START` 다음
`NORMAL_IL`을 보내도 출발 action이 사라지지 않는다. 별도 지속 override가
없으면 기본값 `AUTO`로 출발한다.

## 6. ROS2 통합시험 adapter

노드 이름은 `mission_manager`, update 주기는 고정 20 Hz다. 다음 토픽은 실제
perception 계약이 확정되기 전까지만 사용하는 통합시험 adapter 입력이다.

| 토픽 | 타입 |
|---|---|
| `/mission/override` | `std_msgs/String` |
| `/mission/input/emergency_stop` | `std_msgs/Bool` |
| `/mission/input/start_signal_go` | `std_msgs/Bool` |
| `/mission/input/drive_policy_valid` | `std_msgs/Bool` |
| `/mission/input/lane_fallback_valid` | `std_msgs/Bool` |
| `/mission/input/cone_detected` | `std_msgs/Bool` |
| `/mission/input/cone_exit_ready` | `std_msgs/Bool` |
| `/mission/input/cone_confidence` | `std_msgs/Float32` |
| `/mission/input/cone_count` | `std_msgs/Int32` |

V0.1 wrapper에는 publisher가 없다. 상태 또는 mode가 바뀌면 ROS logger가
다음 형식으로 한 번 출력한다.

안전 규칙상 Mission Manager는 `/xycar_motor`를 발행하지 않으며 두 번째
motor publisher도 만들지 않는다.

```text
[MISSION] state=RACING mode=CONE_DRIVE_RULE
```

설정 주기마다 선택적으로 출력하는 상태도 현재 값만 포함한다.

```text
[MISSION] state=RACING mode=CONE_DRIVE_RULE lap=0 shortcut_used=false override=AUTO
```

매 update마다 출력하지 않으며 전이 원인을 출력하거나 파일에 저장하지 않는다.

## 7. 빌드와 테스트

Mission Manager V0.1은 기존 ROS2 Python 패키지 `track_drive` 안에 통합되어 있다.
별도의 중첩 ROS2 패키지나 symlink는 만들지 않는다.
`track_drive/mission_state.py`의 기존 경기용 상태 구조와도 연결하지 않으며,
새 초안은 `track_drive/mission/` 안에서 독립적으로 동작한다.

```bash
cd ~/xycar_ws
colcon build --symlink-install --packages-select track_drive cone_il
source install/setup.bash
```

순수 코어 테스트:

```bash
cd ~/xycar_ws/src/kookmin_autonomous_competition_teamKAI
python3 -m unittest discover -s test -v
```

ROS2 package 테스트:

```bash
cd ~/xycar_ws
colcon test --packages-select track_drive
colcon test-result --verbose
```

실행:

```bash
source install/setup.bash
ros2 launch track_drive mission_manager_draft.launch.py
```

## 8. 수동 통합시험 명령

일반 IL 준비 후 즉시 START:

```bash
ros2 topic pub --once /mission/input/drive_policy_valid \
  std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /mission/override \
  std_msgs/msg/String "{data: START}"
```

수동 콘, lane fallback, 정지, AUTO 복귀:

```bash
ros2 topic pub --once /mission/override \
  std_msgs/msg/String "{data: CONE_DRIVE_RULE}"
ros2 topic pub --once /mission/input/lane_fallback_valid \
  std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /mission/override \
  std_msgs/msg/String "{data: LANE_FALLBACK}"
ros2 topic pub --once /mission/override \
  std_msgs/msg/String "{data: STOP}"
ros2 topic pub --once /mission/override \
  std_msgs/msg/String "{data: AUTO}"
```

E-stop과 reset:

```bash
ros2 topic pub --once /mission/override \
  std_msgs/msg/String "{data: EMERGENCY_STOP}"
ros2 topic pub --once /mission/input/emergency_stop \
  std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /mission/override \
  std_msgs/msg/String "{data: RESET_EMERGENCY}"
```

## 9. 남은 TODO

- 실제 drive IL output/health adapter 연결
- 실제 rule-based cone controller output/health adapter 연결
- YOLO lane perception을 steering source로 바꾸는 별도 adapter 연결
- source heartbeat와 freshness timeout/watchdog
- `FIXED_OBSTACLE_RULE`, vehicle follow/overtake, route select, shortcut 구현
- 실제 lap count와 shortcut flag 갱신
- 후단 policy selector 및 유일한 final driver 구현

Mission Manager 자체에는 이후에도 `/xycar_motor` publisher를 추가하지 않는다.
