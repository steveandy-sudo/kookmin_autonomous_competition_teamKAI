# left_4 이후 W1/Y1 지름길 진입 주행

이 패키지는 `left_4`가 확인된 뒤 두 번째 미검출 detector 메시지인 S부터
지름길 전용 LR-ASPP를 켜고, 사용자가 지정한 흰선 W1과 노란선 Y1을 골라
기존 Stanley/Pure-Pursuit 조향기로 진입하는 구현이다. 평상시 자이카 PC의
Xbin RULE은 교체하지 않는다. S부터 W1을 찾는 동안에도 RULE이 계속 차량을
제어하고, W1이 처음 lock된 뒤에만 지름길 후보가 제어권을 받는다.

조향 경로의 실제 관측 기하는 W1만 사용한다. Y1은 진입 단계와 기존
`ShortcutCore`로 넘기는 시점을 판단하는 관측값이며, Y1/W2/Y2의 위치나
각도는 진입 경로를 이동하거나 회전시키지 않는다.

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
- S에서는 LR-ASPP 검색만 켠다. 거리 기반 control ready 전에는 기존 RULE이 계속 제어한다.
- 지름길 진입 조향에는 W1만 사용한다. Y1은 단계와 handoff 판단에만
  사용하고 W2/Y2는 조향에서 무시한다.
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
  -> 256x144 canonical white/yellow mask -> W1/W2 분기 기반 W1 lock
  -> W1/W2 분기점 잔여거리 + 현재 차속 기반 공간 조향 gate
  -> Y1 단계/handoff 추적(W1 조향 경로에는 미사용)
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
- `shortcut_wait_for_entry_ready=true`: 거리 기반 control ready 전까지 RULE 유지

`W1_LOCKED`는 분기선 identity를 확보했다는 뜻일 뿐 조향 개입 신호가 아니다.
W1이 멀리 보이더라도 `/shortcut/entry/control_blend=0`인 동안 조향과 속도는
RULE 100%다. W1/W2의 교차점으로 구한 차량 전방 잔여거리가 시작 gate 안에
들어오면 프레임 수가 아니라 **거리(m)** 에 따라 RULE 조향에서 W1 조향으로
smoothstep 혼합한다. 차속이 높을수록 실측 `/vehicle/vesc_state.speed_mps`와
제어 지연을 이용해 시작 거리를 더 멀리 잡는다. 실측값이 잠시 없으면 현재
RULE speed command의 보정값을 사용한다.

속도 4 제한은 제거했다. S 검색 중에는 sequential driver가 기존 RULE 속도를
그대로 내고, 조향 혼합 중 mux도 매 순간 받은 RULE speed 필드를 수정 없이
통과시킨다. 진입 완료 후에는 기존 `ShortcutCore` cruise/exit 후보의 원래
속도를 사용한다.

진입 중앙경로의 각도와 곡선 형상은 100% W1에서 가져온다. W1 자체를
밟지 않도록 고정 차선 폭 `expected_pair_separation_ratio=0.31`의 40%만
횡방향으로 더한다. 현재 `w1_path_weight=0.60`일 때 식은 다음과 같다.

```text
target_x = 0.60 * W1_x + 0.40 * (W1_x + 0.31)
```

Y1 좌표를 위 식에 넣지 않는다. 이 비중은 진입 구간에만 적용되며 W1 선택
규칙이나 기존 `ShortcutCore` 주행에는 영향을 주지 않는다. 실차에서 고정
오프셋 비율을 조정할 때는 다음 상위 launch 인자를 사용한다.

```text
shortcut_entry_w1_path_weight:=0.60
```

안전 범위는 `0.50 <= 값 < 1.0`이고, 값이 커질수록 고정 횡오프셋이
작아진다. 첫 실차 shadow 검증에서는 기본 0.60을 유지한다. 발행되는
`Centerline`의 source는 `shortcut_W1_fixed_lane_offset`, confidence는
`1.0`이다.

거리 기반 조향 시작은 상위 launch에서 조정할 수 있다.

```text
shortcut_entry_full_control_distance_m:=0.15
shortcut_entry_minimum_start_distance_m:=0.55
shortcut_entry_maximum_start_distance_m:=1.20
shortcut_entry_control_latency_sec:=0.25
shortcut_entry_distance_margin_m:=0.08
```

`minimum_start_distance_m`를 키우면 더 일찍, 줄이면 더 늦게 W1 조향이 섞인다.
고속에서는 `control_latency_sec`를 실제 조향 응답 지연에 맞추는 것이 먼저다.
이 값들은 W1 선택/LOCK 조건을 바꾸지 않으며 시간, bag offset, 프레임 번호,
고정 픽셀 좌표를 사용하지 않는다. 조향 혼합이 시작된 뒤에는 인지 흔들림으로
RULE 쪽으로 되돌아가지 않도록 최대 혼합 비율을 latch한다.

Semantic 플래그와 ready 대기를 따로 조합하지 않게 하여, 검색 중 0 명령이
RULE을 잘못 덮는 구성을 막았다. 통합 launch의
`shortcut_semantic_entry_enabled` 기본값은 `true`다. 실제 motor 출력은
별도이며 `drive_enabled` 기본값은 안전상 계속 `false`다. 바퀴를 띄운 검증과
publisher 단일성 확인 뒤 실제 주행할 때만 `drive_enabled:=true`를 명시한다.

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
2. W1 획득 시 바깥 노란선 왼쪽에서 차체 좌측으로 갈라지는 흰 W1과 반대
   방향의 흰 W2가 실제 분기 topology를 이루어야 한다. W2의 짧은 양의
   edge를 W1처럼 보이게 하는 교차는 배제한다.
3. Y1이 확정되기 전에는 W1의 분기 정체성을 계속 확인한다. 위치만 보면
   이전 W1 위치에 가까운 W2로 바뀔 수 있기 때문이다.
4. W1 오른쪽에서 같은 진입 방향을 보이는 노란 후보를 2프레임 확인해
   Y1으로 lock한다. 초기의 lone Y2는 Y1 fallback으로 쓰지 않는다.
5. W1 lock은 픽셀 고정이 아니라 identity lock이다. 정상 후보는 매 프레임
   새 fit으로 갱신한다. `direction_dx_dy < -0.20` 또는 span `< 0.05`인
   흰 후보는 W1 갱신에 사용하지 않고 직전 W1을 유지한다.
6. W1 후보가 사라져도 프레임 수 제한 없이 마지막 W1 fit과 그 고정
   lane-width offset 경로를 유지한다. 기존 2프레임 hold 뒤 3프레임째
   path invalid/safe stop으로 바꾸던 규칙은 W1에 적용하지 않는다. 랜덤
   필터나 시간/rosbag offset 기반 선택도 사용하지 않는다.
7. Y1은 2프레임 확인 후 단계 추적과 W1/Y1 전방 정렬 handoff에만 사용한다.

수동 점의 절대 픽셀이나 rosbag offset은 런타임 입력이 아니다. JSON의
시간은 검증 결과를 사람이 다시 찾는 용도뿐이며, 상태 전이는 관측 순서로만
진행한다.

## 추가 canonical W1/W2 59프레임 반영

버드아이뷰 `256x144` canonical 마스크에 사용자가 직접 그린 W1/W2 59개는
`analysis/canonical_white_line_annotations`에 보존한다. 이 데이터에서는
W1 57개가 모두 양의 `dx/dy`(0.074~0.926), W2 38개가 모두 음의
`dx/dy`(-0.844~-0.117)로 완전히 분리됐다. 이를 다음과 같이 반영했다.

1. 긴 W2는 기존 Canny/Hough와 여러 높이 band track으로 검출한다.
2. Canny가 놓치는 짧은 W1은 채워진 흰 semantic mask에 작은 Hough를 한 번
   더 적용한다. 이 보완 후보는 양의 W1에만 사용해 작은 흰 blob이 W2로
   생기는 것을 막는다.
3. 시작 획득은 W1 양의 분기, W2 음의 본선, 두 선의 실제 관측구간 결합,
   바깥 노란선 안쪽이라는 topology를 모두 만족해야 한다.
4. 획득 뒤에는 W1과 W2를 독립적으로 직전 polyline에 연결한다. W2는 긴
   지지를 우선하여 짧은 마스크 가장자리로 바뀌지 않게 한다.
5. 바깥 노란선이 크게 휘면 전역 직선의 좌우가 국소적으로 뒤집힐 수 있다.
   노란선 기울기가 실제로 큰 프레임에서만 결합 topology가 맞는 W2의
   국소 좌우 예외를 허용한다.

프레임 번호, bag offset, ROS timestamp와 수동 점 좌표는 런타임 규칙에
들어가지 않는다. 동일 마스크와 동일한 직전 관측 상태에는 항상 같은 선이
선택된다.

재현 명령:

```bash
KAI_REPO=/home/kai/kookmin_autonomous_competition_teamKAI
cd "$KAI_REPO"
PYTHONPATH="$KAI_REPO/xycar_ws/src/shortcut_entry_review" \
python3 -m shortcut_entry_review.evaluate_canonical_white_annotations \
  analysis/canonical_white_line_annotations \
  --output-dir analysis/canonical_white_selector_evaluation_20260811
```

관측 선분 전체에서 평균 가로 오차가 canonical 폭의 4% 이하이면 정답으로
계산한 결과는 다음과 같다.

- W1: 4% 선분 오차 기준 55/57 정답, recall/precision 0.965
- W2: 실제 존재 38/38 검출, recall/precision 1.000
- 정답 W1 선 MAE 중앙값 0.0093W, 최대 0.0374W
- 정답 W2 선 MAE 중앙값 0.0126W, 최대 0.0236W

W1 수동 라벨이 비어 있던 연속 2프레임은 직전 W1과 이어지는 양의 흰 조각이
남아 있어 W1 연속 관측으로 유지했다. 이 둘까지 즉시 `None`으로 만드는
골격 gate도 시험했지만, 실제 W1이 화면 안에서 빠르게 이동한 정상 프레임을
함께 누락해 실차 안정성이 더 나빠졌으므로 채택하지 않았다. 상세 결과는
`analysis/canonical_white_selector_evaluation_20260811/summary.json`과
`frame_results.csv`에 저장한다.

버드아이뷰 모델 영상과 최종 W1/W2만 나란히 재생하는 검토 창:

```bash
bash /home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws/src/shortcut_entry_review/scripts/run_canonical_white_selection_review.sh
```

기본 재생 간격은 원본 0.2초 샘플을 rosbag 0.5배속으로 보는 것과 같은
400 ms다. `SPACE`는 정지/재생, `A/D`는 이전/다음 프레임, `F`는 전체화면,
`Q`는 종료다.

`/shortcut/entry/ready=true`는 W1과 Y1이 모두 보인 때가 아니라 W1이 처음
lock된 때다. 이는 “W1이 보이기 시작한 부분부터 기존 RULE을 끈다”는 요구를
반영한다. 첫 12개 저장 프레임처럼 Y1이 없을 때에도 W1 형상에 고정 차선 폭
offset을 더한 경로를 사용한다. Y2를 대신 쓰지 않는다. 실제 Y1이 2프레임
확인되어도 조향 경로는 혼합하지 않으며 Y1은 단계와 handoff 판단에만 쓴다.

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
- W1 mean-x MAE 0.0154W, Y1 mean-x MAE 0.0134W
- 수동 선분의 관측 높이 전체 41-row MAE: W1 0.0135W,
  Y1 0.0080W
- 저장 순서 55에서 3프레임 전방 정렬을 만족해 기존 cruise로 인계

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
수 있다. 이번 진입에서는 선택기가 W1 형상에 고정 차선 폭 offset만 더한
`Centerline`을 먼저 만들어 external path로 넣는다. 따라서 W1만 실제
관측 조향 기하에 반영되고 Y1/W2/Y2는 controller 경로에 들어가지 않는다.

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

W1 mask가 잠시 또는 오래 누락돼도 selector는 마지막 W1 fit을 계속
발행한다. 반면 semantic 파이프라인 전체나 controller 후보 발행 자체가
stale이면 0.35초 안에 `[0,0]`으로 안전 정지한다. 교차로 안에서 정상 RULE로
자동 복귀하는 것은 더 위험할 수 있어 reset을 기다린다. ready 전 W1 검색이
12초간 실패하면 LR-ASPP를 끄고 기존 RULE을 유지한다.

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
| `/shortcut/entry/ready` | W1 identity 최초 lock(조기 인지 허용, 조향 개입 아님) |
| `/shortcut/entry/entry_distance_m` | W1/W2 분기점까지 차량 전방 잔여거리(m) |
| `/shortcut/entry/control_blend` | 거리 기반 RULE→W1 조향 비율 0..1 |
| `/shortcut/entry/control_ready` | 공간 gate 진입 후 메인 driver override 허용 |
| `/shortcut/entry/path_valid` | 현재 진입 경로 유효성 |
| `/shortcut/entry/selected_centerline` | W1 형상 + 고정 차선 폭 offset 차체 좌표 경로 |
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
W1 identity ready -> RULE 계속, 거리 gate 계산
control_ready -> shortcut latch start, RULE→W1 공간 조향 혼합
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
  minimum_start_distance_m:=0.55 \
  full_control_distance_m:=0.15 \
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
  shortcut_entry_w1_path_weight:=0.60 \
  shortcut_entry_minimum_start_distance_m:=0.55 \
  shortcut_entry_full_control_distance_m:=0.15 \
  shortcut_entry_control_latency_sec:=0.25 \
  shortcut_entry_search_timeout_sec:=12.0
```

Xbin이 canonical image까지만 발행하고 기존 canonical controller를 이 launch가
실행해야 하면 `start_rule:=true`만 사용한다. `start_perception:=false`는
유지한다.

확인 기준:

- S 전 processing gate false, 지름길 LR-ASPP mask 발행 없음
- S 이후 `SHORTCUT_ENTRY_SEARCH_RULE`, 기존 RULE shadow 유지
- S 이후에도 RULE 속도 유지(4 제한 없음)
- W1 identity lock만으로는 조향 전환하지 않음
- 거리 기반 `control_ready` 이후에만 `SHORTCUT_W1_ONLY_ENTRY`
- 혼합 중 speed는 RULE 후보 값과 정확히 동일
- 기존 ShortcutCore cruise/exit로 인계된 뒤에도 speed는 RULE 후보와 동일
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
6. RULE 속도를 유지한 채 W1 lock→거리 gate→공간 조향 혼합 확인
7. 실제 Y1 lock과 W2/Y2 무시
8. 기존 ShortcutCore handoff와 종료 후 RULE 복귀
9. 충분한 로그를 확인한 뒤에만 속도 상향

실차 RUN 직전에는 반드시 다음 publisher가 각각 하나인지 확인한다.

```bash
ros2 topic info /hybrid/shortcut_candidate --verbose
ros2 topic info /xycar_motor --verbose
ros2 node list
```

## 새 객체 모델과 미션 로그

통합 launch의 객체 모델 기본값은 아래 파일이다. 예전 모델 파일이 남아
있어도 선택하지 않는다.

```text
xycar_ws/src/study/my_rule/models/no_red_car_best.pt
SHA256 27904321a059ff0290df0c152e9421e7b3792711a121133ee4ac51cf830b79b2
classes = cone, green_4, green_car, null_4, red_4, red_car, yellow_4, left_4
```

`green_car`/`red_car`는 차량 회피용 `car`로 alias하며 `null_4`는 무시한다.
`left_4`는 confidence `0.40` 이상만 인정한다. 3 Hz 객체 detector에서
2프레임 확인한 뒤 2프레임 연속 사라지는 순간 S가 된다.

통합 실행 로그의 `[MISSION]` 순서는 다음과 같다.

```text
left_4 DETECTED 1/2 -> left_4 CONFIRMED 2/2
-> left_4 ABSENT 1/2 -> left_4 ABSENT 2/2 CONFIRMED
-> SHORTCUT ENTRY PERCEPTION START
-> W1 DETECTED/LOCKED
-> SHORTCUT ENTRY STEERING START
-> W1 STEERING(rule/w1/blend/output/RULE_speed, 20 Hz)
-> SHORTCUT HANDOFF REQUEST/GATE
-> SHORTCUT CONTROL SWITCHED(existing ShortcutCore)
-> SHORTCUT FINISHED; returning to canonical RULE
```

W1 lock은 조향 시작이 아니다. `SHORTCUT ENTRY STEERING START`가 차량 기준
거리 gate를 실제로 넘은 시점이며, 이후 각 `W1 STEERING` 줄의 `output`이
발행된 shortcut 조향각이다. `RULE_speed`가 전 구간에서 유지되는지도 같은
줄에서 확인한다.

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
## W1 진입 제어 검토 뷰어

저장된 wide camera, 분기를 보존한 canonical bird's-eye W1, 실제
Stanley/Pure-Pursuit 후보를 한 창에서 보려면 저장소 루트에서 실행한다.

```bash
cd /home/kai/kookmin_autonomous_competition_teamKAI
./run_w1_viewer.sh
```

첫 카메라 프레임이 실제로 들어온 뒤 자동으로 rosbag을 재개하며 0.5배속으로
반복 재생한다. `SPACE`는 일시정지/재생, `F`는 전체화면, `Q`는 종료다.
뷰어와 `shortcut_entry_review` 노드는 `/xycar_motor`를 발행하지 않는다.
종료 시 INT와 TERM 대기를 각각 제한하고, 그래도 남은 이 런처의 bag/stack
자식만 종료한 뒤 회수하므로 launch가 무한 대기하지 않는다.
