# left_4 이후 W1/Y1 지름길 진입 주행

이 패키지는 `left_4`가 확인된 뒤 두 번째 미검출 detector 메시지인 S부터
지름길 전용 LR-ASPP를 켜고, 사용자가 지정한 흰선 W1과 노란선 Y1을 골라
기존 Stanley/Pure-Pursuit 조향기로 진입하는 구현이다. 평상시 자이카 PC의
Xbin RULE은 교체하지 않는다. S부터 W1을 찾는 동안에도 RULE이 계속 차량을
제어하고, W1이 처음 lock된 뒤에만 지름길 후보가 제어권을 받는다.

현재 구현과 rosbag 검증은 끝났지만 실차 조향값은 아직 승인되지 않았다.
기본 실차 실행값은 계속 `drive_enabled=false`이며, 이 패키지의 어떤 노드도
직접 `/xycar_motor`를 발행하지 않는다. 이 문서와 지름길 진입 구현은
2026-08-11 사용자 요청에 따라 `gsw` 브랜치에 반영한다. 이후 추가 변경은
다시 사용자 승인을 받기 전에는 push하지 않는다.

## 자이카 PC Codex에 그대로 전달할 지시

아래 두 줄은 표현을 고치지 말고 그대로 전달한다.

```text
<현재 자이카pc로컬에서 하이브리드 주행제어기부분구도를 간단하게설명하고
만약 신호등관련부분이 없다면 gsw브런치에서 깃헙리드미읽고 하이브리드주행제어기 관련부분가져와>
```

이 지시를 받은 Codex는 코드를 먼저 덮어쓰지 않는다. 현재 자이카 PC에서
다음을 읽기 전용으로 조사하고, 발견한 실제 구조를 사용자에게 먼저
설명해야 한다.

- 기존 Xbin 모델 경로, 입력과 출력 토픽, 메시지 타입
- Xbin 뒤의 RULE 조향기와 `/hybrid/rule_candidate` 유무
- `left_4` 신호등 인지와 연속 프레임 상태기가 이미 있는지
- 기존 shortcut 후보, 최종 arbitration, SPACE 안전 gate 유무
- `/xycar_motor` publisher의 노드 이름과 개수
- 실행 중인 launch와 로컬 미커밋 변경

흰선과 노란선을 모두 출력할 수 있는 호환 모델이 자이카 PC에 이미 있으면
그 모델을 우선 시험할 수 있다. 이름만 보지 말고 출력 클래스가 실제로
`background/white/yellow`인지 확인한다. 노란 중앙선만 출력하는 Xbin/Row
모델은 W1/Y1 지름길 입력을 대신할 수 없다.

## 절대로 바꾸지 않는 경계

- 정상 구간의 Xbin 모델, 카메라 입력, RULE 토픽과 실차 조향 보정값을
  수정하거나 교체하지 않는다.
- LR-ASPP는 S 전에는 카메라 프레임을 받지 않는다. 모델을 메모리에 미리
  올릴 수는 있지만 추론 gate는 닫혀 있다.
- S는 bag 시간이 아니라 `left_4`가 2개 detector 메시지에서 연속 확인된
  뒤, `left_4`가 없는 두 번째 detector 메시지다.
- S에서는 LR-ASPP 검색만 켠다. W1 ready 전에는 기존 RULE이 계속 제어한다.
- 지름길 진입에서 W1/Y1만 사용하고 W2/Y2는 무시한다.
- 최종 `/xycar_motor` publisher는 정확히 하나여야 한다.
- `xycar_map_nav`, `xycar_rule_drive`, `lane_seg_control`, `track_drive_sve`
  디렉터리를 자이카 PC 버전 위에 통째로 복사하지 않는다.
- 사용자 승인 없이 checkout, merge, stash, reset, clean, 삭제 또는 push를
  하지 않는다.

## 현재 노트북과 자이카 PC의 제어 구도

노트북의 통합 reference는 다음과 같다. 노트북의 정상 perception launch가
LR-ASPP인 것과 별개로, 자이카 PC에서는 이 자리를 기존 Xbin이 계속 맡아야
한다.

```text
정상 구간
카메라 -> 정상 perception(자이카 PC: 기존 Xbin)
       -> 기존 canonical Stanley/Pure-Pursuit
       -> /hybrid/rule_candidate
       -> SequentialHybridDriver
       -> /hybrid_gate/xycar_motor_shadow
       -> 최종 SPACE gate 또는 유일한 driver -> /xycar_motor

신호등
카메라 -> my_rule object YOLO(3 Hz)
       -> TrafficLightController
       -> left_4 2회 확인 + 2회 미검출 S

지름길 진입
S -> /hybrid/shortcut_processing_enabled=true
  -> 압축 카메라 gate -> 지름길 전용 LR-ASPP
  -> white/yellow mask -> W1/Y1 순서 선택기
  -> /shortcut/entry/selected_centerline
  -> 기존 canonical Stanley/Pure-Pursuit(drive_enabled=false)
  -> /shortcut/entry/controller_candidate
  -> shortcut_candidate_mux -> /hybrid/shortcut_candidate
  -> SequentialHybridDriver -> 최종 motor 경로

진입 완료
W1/Y1이 차체 전방축에 3프레임 정렬
  -> 기존 track_drive_sve/ShortcutCore cruise와 T-exit
  -> done -> LR-ASPP 종료 -> 기존 Xbin RULE 복귀
```

Semantic 통합에서는 `shortcut_semantic_entry_enabled:=true` 하나가 다음 두
설정을 자동으로 결합한다.

- `shortcut_start_delay_sec=0.0`: 두 번째 미검출 메시지 S에서 즉시 검색
- `shortcut_wait_for_entry_ready=true`: W1 ready 전까지 RULE 유지

또한 기본 `shortcut_entry_speed_command=4.0`을 S부터 진입 완료까지 공통으로
사용한다. S 직후에는 기존 RULE 조향을 그대로 두고 전진 속도만 최대 4로
낮추며, W1 ready 뒤의 semantic Stanley/Pure-Pursuit도 속도 4 후보를 낸다.
W1/Y1 정렬 후 기존 `ShortcutCore` cruise로 인계되면 이 제한은 풀리고
자이카 PC에 원래 튜닝되어 있던 지름길 주행 속도로 돌아간다.

진입 중앙경로는 흰선 W1 60%, 노란선 Y1 40%로 계산해 조향이 W1을 조금 더
따르도록 했다. 이 비중은 진입 구간에만 적용되며 W1/Y1 선택 규칙이나 기존
`ShortcutCore` 주행에는 영향을 주지 않는다. 실차에서 조정할 때는 다음
상위 launch 인자를 사용한다.

```text
shortcut_entry_w1_path_weight:=0.60
```

안전 범위는 `0.50 <= 값 < 1.0`이고, 값이 커질수록 W1 쪽으로 경로가
이동한다. 첫 실차 shadow 검증에서는 기본 0.60을 유지한다.

명령행에서 진입 속도만 바꿀 수 있다.

```text
shortcut_entry_speed_command:=4.0
```

이 값은 Xbin 정상 RULE 속도와 인계 후 `ShortcutCore` 속도를 수정하지 않는다.

Semantic 플래그와 ready 대기를 따로 조합하지 않게 하여, 검색 중 0 명령이
RULE을 잘못 덮는 구성을 막았다. 기본값은 semantic `false`이므로 기존
shortcut 동작도 그대로 유지된다.

## 저장한 55프레임에서 얻은 규칙

저장 위치는 다음과 같다.

```text
analysis/shortcut_line_annotations
```

파일명 순서로 관측 topology가 네 단계로 나뉜다.

| 저장 순서 | 보이는 수동 라벨 | 의미 |
|---|---|---|
| 1–12 | W1, W2, Y2 | Y1은 실제로 없음 |
| 13–17 | W1, W2, Y1, Y2 | 두 갈래가 모두 보임 |
| 18–27 | W1, W2, Y1 | Y2가 먼저 사라짐 |
| 28–55 | W1, Y1 | 목표 pair만 남음 |

W1은 55/55, Y1은 43/55 프레임에 존재했다. 따라서 초기 노란선 Y2를
Y1처럼 쓰면 사용자 지정과 반대로 진입한다. 구현은 다음 순서를 사용한다.

1. 여러 BEV 높이의 후보와 연결요소/선분을 추출한다. 하단 histogram 하나만
   사용하지 않으므로 멀리 짧게 보이는 W1도 후보가 된다.
2. W1 획득 시 차체 좌측으로 갈라지는 진행 방향을 사용한다. 같은 시기의
   W2는 반대 방향이므로 배제된다.
3. Y1이 확정되기 전에는 W1의 분기 정체성을 계속 확인한다. 위치만 보면
   이전 W1 위치에 가까운 W2로 바뀔 수 있기 때문이다.
4. W1 오른쪽에서 같은 진입 방향을 보이는 노란 후보를 2프레임 확인해
   Y1으로 lock한다. 초기의 lone Y2는 Y1 fallback으로 쓰지 않는다.
5. W1/Y1 lock 뒤에는 두 선이 차체 방향과 나란해지며 기울기가 0 또는
   음수로도 바뀌므로, 획득용 양의 방향 gate를 다시 적용하지 않는다.
   이전 polyline, 상대 순서와 폭의 연속성으로 추적한다.

수동 점의 절대 픽셀이나 rosbag offset은 런타임 입력이 아니다. JSON의
시간은 검증 결과를 사람이 다시 찾는 용도뿐이며, 상태 전이는 관측 순서로만
진행한다.

`/shortcut/entry/ready=true`는 W1과 Y1이 모두 보인 때가 아니라 W1이 처음
lock된 때다. 이는 “W1이 보이기 시작한 부분부터 기존 RULE을 끈다”는 요구를
반영한다. 첫 12개 저장 프레임처럼 Y1이 없을 때에는 W1과 어노테이션에서
얻은 정상화 차폭으로 임시 반대 경계를 만들고 속도를 낮게 유지한다. Y2를
대신 쓰지 않는다. 실제 Y1이 2프레임 확인되면 프레임당 25%씩 실제 W1/Y1
중간 경로로 혼합해 조향 급변을 줄인다.

BEV의 `base_footprint` 값은 현재 실제 `tf2` camera 변환이 아니다. 노트북의
고정 homography와 다음 정적 범위를 사용한 차체 상대 좌표다.

```text
BEV: 640 x 660
forward = (1 - y_normalized) * 1.5 m
left    = (0.5 - x_normalized) * 1.4 m
```

따라서 자이카 PC에서는 절대 좌표 임계값을 복사하지 말고 카메라 YAML과
BEV의 차체 정면축, 좌우 부호, 실제 보이는 후보 방향을 다시 확인한다.

## 어노테이션 재검증 결과

재현 명령은 다음과 같다.

```bash
KAI_REPO=/home/kai/kookmin_autonomous_competition_teamKAI
cd "$KAI_REPO"
source /opt/ros/humble/setup.bash
source xycar_ws/install/setup.bash

ros2 run shortcut_entry_review evaluate_sequence_annotations \
  "$KAI_REPO/analysis/shortcut_line_annotations" \
  --output-dir /tmp/shortcut-entry-sequence-check
```

현재 저장 결과는 다음과 같다.

- W1 선택 55/55, W2 오선택 0
- Y1은 두 번째 확인부터 42프레임 추적, Y2 오선택 0
- Y1이 없던 초기 false Y1 선택 0
- 유효 진입 경로 55/55
- W1 mean-x MAE 0.0138W, Y1 mean-x MAE 0.0131W
- 수동 선분의 관측 높이 전체 41-row MAE: W1 0.0108W(6.89 px),
  Y1 0.0077W(4.90 px)
- 저장 순서 53에서 3프레임 전방 정렬을 만족해 기존 cruise로 인계

전체 결과는
`analysis/shortcut_entry_sequence_20260811/summary.json`과
`frame_results.csv`에 남겨 두었다. 최종 수치는 후보 추출 회귀 테스트 후
생성된 report를 기준으로 한다.

## 기존 흰선+노란선 조향 코드 재사용

지름길용으로 새 조향 공식이나 새 servo 보정표를 만들지 않았다. 기존
`xycar_rule_drive/canonical_stanley_pursuit_driver.py`의 external
`kaiev26_msgs/msg/Centerline` 입력, Stanley/Pure-Pursuit, Xycar 조향 변환,
평활화와 조향 변화율 제한을 그대로 사용한다.

기존 캐노니컬 코드의 기본 색 융합은 설정에 따라 노란선 비중이 1.0이 될
수 있다. 이번 진입에서는 W1과 Y1을 선택기가 먼저 정확히 고르고 두 경계의
중간 `Centerline`을 만들어 external path로 넣는다. 따라서 W1과 Y1이 모두
실제 목표 경로에 반영되며 W2/Y2는 controller 입력에 들어가지 않는다.

Shortcut용 controller 안전 설정은 다음과 같다.

- `drive_enabled=false`
- `external_path_enabled=true`
- `external_path_timeout_sec=0.35`
- `lane_loss_speed_command=0`
- `hold_last_steering_on_lane_loss=false`
- `hold_last_speed_on_lane_loss=false`
- 출력은 `/shortcut/entry/controller_candidate` shadow뿐

`shortcut_candidate_mux`도 `/xycar_motor`를 소유하지 않는다. 진입 중에는
기존 Stanley/Pure-Pursuit 후보를, 정렬 후에는 기존 `ShortcutCore`
cruise/exit 후보를 하나의 `/hybrid/shortcut_candidate` 계약
`[angle, speed, done, phase]`으로 보낸다.

Ready 이후 W1/Y1 또는 후보가 stale이면 0.35초 안에 `[0,0]`으로 안전
정지한다. 교차로 안에서 정상 RULE로 자동 복귀하는 것은 더 위험할 수 있어
자동 복귀하지 않고 reset을 기다린다. 반면 ready 전 W1 검색이 12초간
실패하면 LR-ASPP를 끄고 기존 RULE을 유지한다.

## rosbag을 처음부터 재생하며 확인

다음 명령 하나가 필요한 패키지를 빌드하고, 새 RViz, S-gated 인지/제어
스택, 키보드 rosbag 재생기를 각각 연다.

```bash
bash /home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws/src/shortcut_entry_review/scripts/run_shortcut_entry_sequence_review.sh
```

rosbag은 offset 0부터 시작한다. `left_4` YOLO가 2회 확인된 뒤 두 번째
미검출에서 review gate가 열리므로, LR-ASPP와 W1/Y1 선택기는 S 전 영상을
학습 규칙과 무관하게 lock하지 않는다.

키보드 재생기:

- `Space`: 재생/일시정지
- `a`: 현재 위치에서 0.2초 전으로 이동하고 해당 프레임 표시
- `q`: 재생기 종료

별도 OpenCV 창 두 개와 RViz pane이 함께 표시된다.

- `Shortcut Canonical Model Input`: LR-ASPP 흰색/노란색 캐노니컬 입력
- `Shortcut W1-Y1 Selection and Path`: 모든 후보, 선택 W1/Y1, 중앙 경로
- RViz `left_4 Detection`: S를 만드는 객체 인지 영상
- RViz `Controller Path`: 기존 controller가 받은 경로 marker

이 review launch의 후보 토픽은 `/shortcut/review/candidate`로 격리되고
`/xycar_motor` publisher는 없다.

## 주요 토픽

| 토픽 | 의미 |
|---|---|
| `/hybrid/shortcut_processing_enabled` | 생산 S 이후 LR-ASPP gate |
| `/shortcut/lraspp/white_mask` | 지름길 전용 흰선 mask |
| `/shortcut/lraspp/yellow_mask` | 지름길 전용 노란선 mask |
| `/shortcut/entry/ready` | W1 최초 lock, RULE override 허용 |
| `/shortcut/entry/path_valid` | 현재 진입 경로 유효성 |
| `/shortcut/entry/selected_centerline` | W1/Y1 기반 차체 좌표 경로 |
| `/shortcut/entry/controller_candidate` | 기존 Stanley/Pursuit `[angle,speed]` |
| `/shortcut/entry/cruise_enabled` | 기존 ShortcutCore 인계 gate |
| `/hybrid/shortcut_candidate` | 최종 지름길 후보 `[angle,speed,done,phase]` |
| `/hybrid_gate/xycar_motor_shadow` | 통합 최종 shadow |
| `/xycar_motor` | 유일한 최종 실차 출력만 허용 |

Semantic phase code는 W1-only 11, Y1-locked 12, pair 13, handoff 14다.

## 새 진입 제어기에서 기존 지름길 제어기로 넘어가는 시점

전환 조건은 W1/Y1의 차체 전방축 대비 절대 기울기가 기준 안에 들어온 상태가
3개 semantic 프레임 연속 유지되는 순간이다. 기본 기울기 기준은 `0.22`이며
bag의 초를 하드코딩하지 않는다.

전환 때 터미널에 다음 세 로그가 순서대로 한 번씩 표시된다.

```text
SHORTCUT HANDOFF REQUEST: semantic W1/Y1 entry -> existing ShortcutCore cruise; ros_stamp=... semantic_frame=... condition=forward_alignment_3_frames ...
SHORTCUT HANDOFF GATE: semantic entry finished; waiting for the first fresh existing ShortcutCore cruise candidate
SHORTCUT CONTROL SWITCHED: semantic W1/Y1 entry -> existing ShortcutCore cruise/exit; handoff_delay_ms=... phase=4
```

- 첫 로그: 선택기가 전방 정렬 3프레임을 확정한 영상 timestamp와 S 이후
  semantic 처리 순번
- 두 번째 로그: mux가 기존 제어기 인계 gate를 받은 시점
- 세 번째 로그: 기존 `ShortcutCore`의 첫 fresh 명령을 실제 후보로 사용하기
  시작한 시점과 gate-to-command 지연

`/shortcut/entry/cruise_enabled=true`, semantic phase `4`,
`/shortcut/entry/mux_status`의 `existing ShortcutCore cruise/exit candidate`도
같이 확인할 수 있다. 전환 직후 기존 후보가 stale이면 제어기를 바꾸지 않고
안전 정지하며, 세 번째 `CONTROL SWITCHED` 로그는 fresh 후보를 받은 뒤에만
출력된다.

## 자이카 PC 모델 배치

기준 모델:

```text
파일명: kookmin_lane_lraspp_mbv3s_256x144.pt
크기: 4,021,542 bytes
SHA-256: 45ab4744f5edee1464441f15681e0b77ab94415a1193558b113c2bd619120fe8
입력: RGB, ImageNet normalization, 256x144
출력: 0=background, 1=white, 2=yellow
```

현재 노트북 경로:

```text
/home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws/src/xycar_perception/models/kookmin_lane_lraspp_mbv3s_256x144.pt
```

자이카 PC에서는 기존 동명 파일을 덮어쓰지 않는다. 호환 모델을 별도
절대경로에 두고 다음 상위 launch 인자로 선택한다.

```text
shortcut_lane_model:=/absolute/path/to/white_yellow_model.pt
shortcut_camera_yaml:=/absolute/path/to/xycar_wide_camera.yaml
```

모델 복사 전후에는 `sha256sum`을 비교한다. 다른 호환 모델을 사용하면
이 README의 55프레임 shadow 검증을 새로 수행한다.

## `gsw`에서 자이카 PC로 충돌 없이 가져오기

먼저 현재 상태만 기록한다.

```bash
KAI_REPO=/absolute/path/to/kookmin_autonomous_competition_teamKAI
KAI_WS="$KAI_REPO/xycar_ws"
cd "$KAI_REPO"

git status --short --branch
git rev-parse HEAD
git remote -v
ros2 node list
ros2 topic list -t
ros2 topic info /xycar_motor --verbose
ros2 topic info /hybrid/rule_candidate --verbose
ros2 topic info /hybrid/shortcut_candidate --verbose
```

작업 트리가 더러우면 `gsw`로 checkout하거나 merge하지 않는다. 사용자에게
현재 변경을 보여 주고, 허용된 경우에만 별도 임시 worktree에서 읽기 전용
비교를 한다.

```bash
git fetch origin gsw
git rev-parse origin/gsw
git worktree add --detach /tmp/kai-gsw-review origin/gsw

git diff --name-status HEAD...origin/gsw -- \
  xycar_ws/src/shortcut_entry_review \
  xycar_ws/src/xycar_map_nav \
  xycar_ws/src/xycar_rule_drive \
  xycar_ws/src/track_drive_sve \
  xycar_ws/src/lane_seg_control
```

`/tmp/kai-gsw-review`가 이미 있으면 삭제하지 않는다. 누구의 작업인지 먼저
확인한다.

신규 `shortcut_entry_review` 패키지는 자이카 PC에 같은 경로가 없을 때만
전체 추가할 수 있다. 기존 패키지는 다음 단위로 수동 이식한다.

| 대상 | 이식할 부분 | 반드시 보존할 부분 |
|---|---|---|
| `shortcut_entry_review/` | 신규 패키지와 launch, 테스트 | 같은 이름 로컬 패키지가 있으면 먼저 diff |
| `sequential_hybrid_driver.py` | search/ready 상태와 gate 관련 함수 | 기존 RULE·신호등·콘·회피 arbitration |
| `real_sequential_hybrid_drive.launch.py` | semantic include와 인자 | 기존 Xbin perception/RULE launch |
| `shortcut_candidate_node.py` | semantic 완료 후 cruise 시작 옵션 | 자이카 PC의 튜닝된 `ShortcutCore` |
| `shortcut_core.py` | `start_cruise()` 진입점만 | 기존 cruise/PID/T-exit 값 |
| `canonical_stanley_pursuit_driver.py` | external Centerline 기능이 없을 때 해당 hunk | 실차 steering map과 정상 RULE 파라미터 |
| `lane_seg_control` | 흰/노란 mask와 압축 입력 기능이 없을 때 해당 hunk | 기존 Xbin launch와 정상 토픽 |
| LR-ASPP 모델 | SHA 확인 후 별도 경로 | 기존 모델 파일을 덮어쓰지 않음 |

자이카 PC에 신호등 상태기가 이미 있으면 `gsw` 상태기로 교체하지 않는다.
다음 계약만 기존 상태기에 연결한다.

```text
S -> shortcut_entry_search_active=true
  -> shortcut_processing_enabled=true
  -> 기존 RULE 계속
W1 ready -> shortcut latch start
search timeout -> processing false, RULE 유지
shortcut done -> processing false, RULE 복귀
```

`kaiev26_msgs/msg/Centerline` 계약이 없으면 동일 필드의 메시지를 새로 빌드한
뒤 연결하거나, 자이카 PC의 기존 centerline 메시지로 adapter를 만든다.
메시지 패키지 전체를 덮어쓰지 않는다.

## 자이카 PC 빌드와 검증 순서

```bash
cd "$KAI_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install --packages-select \
  lane_seg_control xycar_rule_drive track_drive_sve \
  shortcut_entry_review xycar_map_nav

source install/setup.bash
colcon test --packages-select \
  shortcut_entry_review xycar_rule_drive track_drive_sve xycar_map_nav
colcon test-result --verbose
```

테스트가 실패하면 실차 단계로 넘어가지 않는다.

### 1. 격리된 semantic shadow

```bash
ros2 launch shortcut_entry_review shortcut_entry_semantic_control.launch.py \
  lane_model:=/absolute/path/to/white_yellow_model.pt \
  camera_yaml:=/absolute/path/to/xycar_wide_camera.yaml \
  processing_enabled_topic:=/shortcut_shadow/enabled \
  candidate_topic:=/shortcut_shadow/candidate \
  default_enabled:=false \
  entry_speed_command:=4.0 \
  show_opencv_windows:=false

ros2 topic pub --once /shortcut_shadow/enabled \
  std_msgs/msg/Bool '{data: true}'
```

`ros2 topic info /xycar_motor --verbose`로 이 launch가 motor publisher를
추가하지 않았는지 확인한다.

### 2. 기존 Xbin과 통합 shadow

기존 Xbin이 이미 `/hybrid/rule_candidate`를 발행한다면 정상 perception과
RULE 노드를 중복 실행하지 않는다.

```bash
ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  drive_enabled:=false \
  gate_arming_required:=false \
  force_rule_only:=true \
  start_perception:=false \
  start_rule:=false \
  start_direct_bev_rule:=false \
  start_shortcut:=true \
  shortcut_semantic_entry_enabled:=true \
  shortcut_lane_model:=/absolute/path/to/white_yellow_model.pt \
  shortcut_camera_yaml:=/absolute/path/to/xycar_wide_camera.yaml \
  shortcut_entry_speed_command:=4.0 \
  shortcut_entry_w1_path_weight:=0.60 \
  shortcut_entry_search_timeout_sec:=12.0
```

Xbin이 canonical image까지만 발행하고 기존 canonical controller를 이 launch가
실행해야 하면 `start_rule:=true`만 사용한다. `start_perception:=false`는
유지한다.

확인 기준:

- S 전 processing gate false, 지름길 LR-ASPP mask 발행 없음
- S 이후 `SHORTCUT_ENTRY_SEARCH_RULE`, 기존 RULE shadow 유지
- S부터 W1/Y1 진입 완료까지 전진 속도 최대 4
- W1 ready 이후에만 `SHORTCUT_W1_ONLY_ENTRY`
- 초기 status에 `synthetic Y1`, Y2 미사용
- 실제 Y1 확인 뒤 `W1/Y1 pair tracked`, W2/Y2 미사용
- stale 시 `[0,0]`, 검색 12초 실패 시 RULE 유지
- 완료 후 processing false와 기존 Xbin RULE 복귀

### 3. 바퀴를 띄운 상태와 저속 실차

먼저 통합 driver는 `drive_enabled:=false`, `gate_arming_required:=true`로
유지하고, 기존 `space_drive_gate` 하나만 `/xycar_motor`를 소유하게 한다.
`space_drive_gate`와 `sequential_hybrid_driver drive_enabled:=true`를 동시에
사용하면 motor publisher가 두 개가 되므로 금지한다.

검증 순서:

1. 단위 테스트
2. 이 rosbag RViz/OpenCV 시각화
3. 격리 semantic shadow
4. 기존 Xbin과 통합 shadow
5. 바퀴를 띄운 상태에서 SPACE gate
6. 기본 command 4.0 또는 승인된 더 낮은 값으로 S→W1 ready
7. 실제 Y1 lock과 W2/Y2 무시
8. 기존 ShortcutCore handoff와 종료 후 RULE 복귀
9. 충분한 로그를 확인한 뒤에만 속도 상향

실차 RUN 직전에는 반드시 다음 publisher가 각각 하나인지 확인한다.

```bash
ros2 topic info /hybrid/shortcut_candidate --verbose
ros2 topic info /xycar_motor --verbose
ros2 node list
```

## 롤백 기록

이식 전후 다음을 남긴다.

- 자이카 PC 이식 전 commit SHA
- 참고한 `gsw` commit SHA
- 실제 변경 파일과 수동 이식한 함수/launch 블록
- 기존 Xbin 모델 경로와 SHA
- shortcut LR-ASPP 모델 경로와 SHA
- 사용한 카메라 YAML
- shadow 및 저속 실행 명령
- 실패 시 원복할 파일과 commit

자이카 PC의 기존 미커밋 변경을 stash하거나 삭제하지 않는다. 자이카 PC에서
통합·튜닝한 후 추가로 GitHub에 push하려면 다시 사용자 승인을 받는다.
