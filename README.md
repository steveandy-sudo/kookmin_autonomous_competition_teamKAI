# Kookmin Autonomous Competition Team KAI

국민대 자율주행대회용 Xycar RULE·콘·차량 회피 통합 저장소다.

실차 실행은 runbook을 기준으로 하고, 구현 내용과 측정 결과는 현황 문서에서
확인한다.

- [실차 RULE·콘·차량 회피 실행](docs/REAL_CAR_RUNBOOK_KO.md)
- [2026-08-07 통합 주행 구현·검증 현황](docs/INTEGRATED_RULE_DRIVE_STATUS_20260807_KO.md)

현재 통합 브랜치: `agent/integrated-rule-drive-tuning`

## 지름길 주행 파라미터

아래 내용은 현재 `gsw`의
`real_sequential_hybrid_drive.launch.py`와 `shortcut_entry_review` 구현을
기준으로 한다. 단위가 다른 값을 한꺼번에 바꾸면 원인을 구분할 수 없으므로
실차 튜닝은 반드시 shadow(`drive_enabled:=false`)와 동일 rosbag에서 한 항목씩
비교한 뒤 적용한다.

현재 지름길 흐름은 다음과 같다.

```text
left_4 >= 0.40을 2 detector 프레임 확인
  -> left_4 미검출 2 detector 프레임 확인(S, 추가 시간 지연 0초)
  -> LR-ASPP로 W1 검색, RULE 유지
  -> W1 lock + 공간 gate + W1 유효 프레임 확인
  -> RULE/W1 조향 혼합
  -> 진입 완료 조건 충족
  -> 기본값은 yellow Xbin RULE로 handoff
```

### 조향 시작 위치와 혼합

공간 gate의 추정 조향 시작 거리는 다음 식으로 정해진다.

```text
entry_speed_cmd = min(현재 RULE speed command, shortcut_entry_speed_command)
estimated_speed_mps = entry_speed_cmd * shortcut_speed_command_to_mps
trigger_distance_m = shortcut_spatial_gate_minimum_distance_m
                   + estimated_speed_mps
                   * shortcut_spatial_gate_response_time_sec
```

차량이 W1/W2 추정 분기점에 접근해 남은 `branch_distance_m`가
`trigger_distance_m` 이하가 되면 gate가 열린다. 즉 **시작거리 값을 크게 하면
더 멀리서 일찍 조향하고, 작게 하면 분기점에 더 가까워진 뒤 늦게 조향한다.**
`minimum_distance`는 분기점을 통과한 뒤 추가로 직진할 거리를 뜻하지 않는다.

| 상위 launch 인자 | 현재 기본값 | 값을 작게 하면 | 값을 크게 하면 |
|---|---:|---|---|
| `shortcut_spatial_gate_minimum_distance_m` | `0.25` m | 분기점에 더 가까워진 뒤 gate가 열려 조향이 늦어진다. | 분기점에서 더 먼 위치에서 gate가 열려 조향이 빨라진다. |
| `shortcut_spatial_gate_response_time_sec` | `0.35` s | 속도 선행 보정이 줄어 고속에서도 조향 시작이 늦어진다. | 속도 선행 보정이 커져 고속일수록 더 일찍 조향한다. |
| `shortcut_speed_command_to_mps` | `0.04` | 추정 속도와 진행거리가 작아져 gate 및 handoff가 늦어진다. | 추정 속도와 진행거리가 커져 gate 및 handoff가 빨라진다. 실제 차속 환산 보정값이므로 임의 조향 gain처럼 쓰지 않는다. |
| `shortcut_spatial_gate_blend_distance_m` | `0.25` m | gate가 열린 뒤 W1 비중이 빠르게 증가한다. | W1 비중이 더 긴 거리 동안 천천히 증가한다. **조향 시작 위치는 바꾸지 않는다.** |
| `shortcut_w1_steering_start_delay_frames` | `4` | gate 뒤 적은 W1 프레임만 보고 빨리 조향한다. `0`이면 프레임 지연이 없다. | W1을 더 오래 확인한 뒤 조향하므로 늦지만 오검출에 강해진다. |
| `shortcut_w1_steering_delay_missing_tolerance_frames` | `2` | W1 한두 번 누락에도 누적 확인값을 빨리 초기화해 시작이 보수적이다. | 간헐적 W1 누락을 더 오래 허용해 조향 시작이 쉬워지지만 불안정한 W1도 통과할 수 있다. |
| `shortcut_w1_path_weight` | `0.60` | 고정 차선 폭 반대편 쪽으로 경로 offset이 커진다. 허용 최솟값은 `0.50`이다. | 중심경로가 W1 자체에 더 가까워지고 고정 횡 offset이 작아진다. `1.0`은 허용되지 않는다. |

현재 경로 식은 W1의 각도와 곡선 형상을 사용하고 고정 폭 `0.31`을 더한다.

```text
target_x = W1_x + (1 - shortcut_w1_path_weight) * 0.31
```

`shortcut_spatial_gate_blend_distance_m:=0.50`은 현재 기본값 `0.25`보다
W1 비중을 약 두 배 긴 거리에서 천천히 올리지만 gate가 열리는 위치는 같다.

### 조향 강제값, 속도와 일시 누락

| 상위 launch 인자 | 현재 기본값 | 값을 작게 하면 | 값을 크게 하면 |
|---|---:|---|---|
| `shortcut_entry_direction_hold_command` | `-30.0` | 음수 크기를 더 키워 `-35`처럼 만들면 더 강한 최소 좌조향을 강제한다. | `-20`, `-10`, `0`처럼 0에 가까워질수록 강제가 약해지고 `0`은 강제를 끈다. 양수는 우조향을 강제하므로 지름길 진입에 사용하지 않는다. |
| `shortcut_entry_speed_command` | `9.0` | 진입 속도 cap이 낮아진다. 동시에 gate 식의 추정 속도도 줄어 조향 시작이 늦어질 수 있다. | 더 높은 RULE 속도를 허용하고 속도 선행거리도 커져 조향 시작이 빨라질 수 있다. RULE 속도보다 높여도 차량을 RULE보다 가속하지는 않는다. |
| `shortcut_w1_steering_hold_sec` | `1.0` s | W1 경로가 끊기면 마지막 조향을 짧게 유지하고 RULE 검색으로 빨리 돌아간다. `0`이면 추가 hold가 없다. | 영상 누락 때 마지막 W1 조향을 더 오래 유지한다. 너무 크면 잘못된 조향도 오래 유지한다. |

주의: 현재 `gsw`의 mux는 유효한 W1 경로가 생기면 RULE/W1 혼합 결과에
`entry_direction_hold_command`를 적용한다. 기본 `-30`에서는 blend가 작아도
최종 후보가 최소 `-30` 좌조향으로 제한될 수 있다. 따라서 blend 효과만
확인하려면 먼저 shadow에서 hold를 `0`으로 비교해야 한다.

### 진입 종료와 handoff

| 상위 launch 인자 | 현재 기본값 | 값을 작게/끄면 | 값을 크게/켜면 |
|---|---:|---|---|
| `shortcut_minimum_entry_progress_m` | `0.50` m | 진입 후 더 짧게 진행해도 handoff가 가능해져 빨리 끝난다. | W1 조향으로 더 멀리 진행한 뒤 handoff한다. |
| `shortcut_pair_track_handoff_required_frames` | `2` | W1/Y1 pair를 적게 확인하고 빨리 handoff한다. | pair를 더 오래 확인하므로 안정적이지만 handoff가 늦어진다. |
| `shortcut_w1_loss_handoff_enabled` | `true` | `false`이면 Y1 확인 뒤 W1 소실을 handoff 조건으로 사용하지 않는다. | `true`이면 최소 진행거리 이후 Y1 확인 + W1 소실로도 handoff할 수 있다. |
| `shortcut_maximum_entry_steering_sec` | `1.5` s | 더 빨리 시간 제한 handoff가 발생한다. `0`이면 이 시간 fallback을 끈다. | W1 조향을 더 오래 허용하고 시간 fallback이 늦어진다. |
| `shortcut_handoff_to_rule` | `true` | `false`이면 semantic 진입 뒤 기존 ShortcutCore cruise/exit 후보를 기다린다. | `true`이면 semantic 진입 완료 뒤 yellow Xbin RULE 후보로 복귀한다. 숫자 크기 파라미터가 아니라 mode 선택이다. |

`minimum_entry_progress_m`를 만족하기 전에는 정렬이나 pair 조건만으로 handoff하지
않는다. 단, `maximum_entry_steering_sec` 시간 fallback은 최소 진행거리와 별개로
작동한다.

### left_4와 안전 timeout

다음 값은 현재 상위 launch 인자가 아니라
`sequential_hybrid_driver`의 파라미터 또는 통합 launch의 고정 연결값이다.
상위 launch 명령에 같은 이름을 임의로 추가해도 변경되지 않는다.

| 내부 파라미터 | 현재값 | 값을 작게 하면 | 값을 크게 하면 |
|---|---:|---|---|
| `shortcut_yolo_min_confidence` | `0.40` | 약한 `left_4`도 인정해 trigger가 쉬워지지만 오검출 위험이 커진다. | 확실한 detection만 인정해 오검출은 줄지만 미검출 가능성이 커진다. |
| `shortcut_yolo_required_frames` | `2` | `left_4`를 빨리 확정하지만 한 프레임 오검출에 약하다. | 여러 프레임을 요구해 안정적이지만 확정이 늦어진다. |
| `shortcut_yolo_absence_frames` | `2` | `left_4` 소실 S를 빨리 선언한다. | 일시 미검출에 강하지만 S와 LR-ASPP 시작이 늦어진다. |
| `shortcut_entry_search_timeout_sec` | `12.0` s | W1을 못 찾으면 검색을 빨리 취소하고 RULE로 돌아간다. | LR-ASPP/W1 검색을 더 오래 유지한다. |
| `shortcut_candidate_timeout_sec` | `0.35` s | 오래된 shortcut 후보를 빨리 거부하지만 주기 지터에도 끊길 수 있다. | 통신 지터에는 강하지만 오래된 조향을 더 오래 유효하게 본다. |
| `shortcut_rearm_absence_sec` | `1.0` s | 지름길 종료 뒤 trigger가 빨리 재무장되어 재진입 위험이 커진다. | 충분히 오래 `left_4`가 없어야 재무장되어 중복 trigger가 줄어든다. |

`shortcut_wait_for_entry_ready`는 현재 `true`를 유지한다. 이를 끄면 W1 공간
gate가 준비되기 전에 shortcut 제어권이 시작될 수 있으므로 튜닝 목적으로
변경하지 않는다.

### 실행 시 값 지정

상위 launch를 직접 실행할 때 기존 검증된 명령 뒤에 다음처럼 값을 붙인다.
아래는 motor를 발행하지 않는 shadow 예시다.

```bash
ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  drive_enabled:=false start_shortcut:=true \
  shortcut_spatial_gate_minimum_distance_m:=0.25 \
  shortcut_spatial_gate_response_time_sec:=0.35 \
  shortcut_spatial_gate_blend_distance_m:=0.50 \
  shortcut_w1_steering_start_delay_frames:=4 \
  shortcut_w1_path_weight:=0.60 \
  shortcut_entry_direction_hold_command:=-30.0 \
  shortcut_entry_speed_command:=9.0
```

`run_space_hybrid_test.sh`는 현재 공간 gate의 세 값
(`minimum_distance`, `response_time`, `blend_distance`)을 환경변수로 전달하지
않으므로 이 세 값은 직접 launch할 때만 바뀐다. 이 스크립트에서 지원하는
지름길 환경변수 예시는 다음과 같다.

```bash
SHORTCUT_W1_STEERING_START_DELAY_FRAMES=4 \
SHORTCUT_W1_STEERING_DELAY_MISSING_TOLERANCE_FRAMES=2 \
SHORTCUT_MINIMUM_ENTRY_PROGRESS_M=0.50 \
SHORTCUT_PAIR_TRACK_HANDOFF_REQUIRED_FRAMES=2 \
SHORTCUT_W1_LOSS_HANDOFF_ENABLED=true \
SHORTCUT_MAXIMUM_ENTRY_STEERING_SEC=1.5 \
SHORTCUT_W1_STEERING_HOLD_SEC=1.0 \
SHORTCUT_ENTRY_DIRECTION_HOLD_COMMAND=-30.0 \
SHORTCUT_ENTRY_SPEED_COMMAND=9.0 \
  bash xycar_ws/src/xycar_map_nav/scripts/run_space_hybrid_test.sh
```

조향이 너무 빠르면 먼저 `shortcut_spatial_gate_minimum_distance_m` 또는
`shortcut_spatial_gate_response_time_sec`를 줄인다. 시작 위치는 맞지만 조향이
급하면 `shortcut_spatial_gate_blend_distance_m`를 키운다. 반대로 조향 시작이
늦으면 minimum/response를 키우고, 시작은 맞지만 W1 반영이 느리면 blend
distance를 줄인다. 한 시험에서 이 두 종류를 동시에 바꾸지 않는다.
