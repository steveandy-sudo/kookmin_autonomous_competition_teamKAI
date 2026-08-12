# GSW: 최신 자이카 RULE + 하이브리드 미션 + W1 지름길 진입

이 브랜치는 자이카 기준 `agent/yellow-center-curve-test` 최신 커밋
`978a13f`의 Xbin/곡선 주행 튜닝을 보존하면서, `gsw`의 하이브리드 미션
우선순위와 이번 W1 지름길 진입기를 함께 정합한 실차 이식용 브랜치다.

가져온 내용은 간단히 다음 네 묶음이다.

- 최신 정상 주행: 2.5 m yellow-center Xbin 인지와 직선/곡선 RULE 튜닝
- 하이브리드 우선순위: 신호등 > 지름길 > 라바콘 > 차량 회피 > RULE
- 객체 인지: 새 `no_red_car_best.pt`와 `left_4 >= 0.40` 조건
- 지름길: RULE → left_4 확인/소실 → W1 진입 → 기존 ShortcutCore 주행

W1 알고리즘과 55프레임 검증, 토픽 계약은
[`shortcut_entry_review/README.md`](xycar_ws/src/shortcut_entry_review/README.md)를
함께 읽는다.

## 실제 상태 흐름

```text
기존 Xbin RULE 주행
  -> left_4(conf >= 0.40) 2개 detector 프레임 확인
  -> left_4 2개 detector 프레임 미검출 = S
  -> LR-ASPP/W1 검색 시작, 아직 RULE 조향/속도 100%
  -> W1_LOCKED(identity 확보, 조향 전환 아님)
  -> W1/W2 분기점 거리 + 현재 차속의 공간 gate
  -> RULE angle에서 W1 angle로 거리 기반 혼합, speed는 RULE 그대로
  -> W1/Y1 전방 정렬
  -> 기존 ShortcutCore cruise/exit 조향, speed는 계속 RULE 그대로
  -> 완료 후 기존 Xbin RULE
```

프레임 수로 조향을 섞지 않는다. W1은 멀리서 먼저 lock할 수 있지만 실제
조향은 차량 좌표계의 분기점 잔여거리 안에서만 시작한다. `W1_LOCKED`는
픽셀 고정이 아니라 W1 분기 identity를 선택하고 연속 추적한다는 뜻이다.

전체 모드에서 전진 속도는 현재 `/hybrid/rule_candidate`의 speed 값을
사용한다. 과거의 진입 speed 4 제한은 삭제했다. 진입기와 ShortcutCore는
조향만 제공하며, 모드가 바뀐다는 이유로 속도를 낮추거나 올리지 않는다.

## 모델

객체 모델은 다음 파일만 기본값으로 사용한다.

```text
xycar_ws/src/study/my_rule/models/no_red_car_best.pt
SHA256 27904321a059ff0290df0c152e9421e7b3792711a121133ee4ac51cf830b79b2
classes: cone, green_4, green_car, null_4, red_4, red_car, yellow_4, left_4
```

`green_car`와 `red_car`는 `car`로 alias한다. `null_4`는 미션 입력으로 쓰지
않는다. 예전 `kookmin_objects_best_20260804.pt`가 디스크에 남아 있어도 통합
launch는 사용하지 않는다. 자이카 PC 이식 때 위 SHA가 다른 파일이면 실행을
중단한다.

지름길 진입 때만 다음 LR-ASPP TorchScript를 추가로 사용한다.

```text
xycar_ws/src/xycar_perception/models/kookmin_lane_lraspp_mbv3s_256x144.pt
SHA256 45ab4744f5edee1464441f15681e0b77ab94415a1193558b113c2bd619120fe8
```

wide compressed camera → 256x144 canonical BEV white/yellow mask이며, 흰 분기
보존을 위해 canonical 단일-white fit은 끈다. 평상시 Xbin RULE 파이프라인은
교체하지 않는다.

## 실시간 로그

통합 실행 터미널에서 `[MISSION]`을 보면 다음 전환과 조향을 확인할 수 있다.

```text
[MISSION] left_4 DETECTED ... confirm=1/2
[MISSION] left_4 CONFIRMED ...
[MISSION] left_4 ABSENT ... confirm=1/2
[MISSION] left_4 ABSENT 2/2 CONFIRMED
[MISSION] SHORTCUT ENTRY PERCEPTION START ...
[MISSION] W1 DETECTED/LOCKED ...
[MISSION] SHORTCUT ENTRY STEERING START ...
[MISSION] W1 STEERING: rule=...deg w1=...deg blend=... output=...deg RULE_speed=...
SHORTCUT HANDOFF REQUEST ...
SHORTCUT CONTROL SWITCHED ... existing ShortcutCore ... RULE speed preserved
SHORTCUT FINISHED; returning to canonical RULE
```

W1 조향 로그는 진입 제어 중 20 Hz 후보 주기마다 남는다. `output`이 최종
shortcut 후보 조향각이고 `RULE_speed`가 바뀌지 않았는지 함께 확인한다.

## 자이카 PC에 충돌 없이 가져오기

자이카 PC에 미커밋 변경이 있으면 `git checkout gsw`, 강제 reset, 디렉터리
덮어쓰기를 하지 않는다. 먼저 현재 작업과 publisher를 조사한 뒤 다음 순서로
필요한 hunk만 수동 이식한다.

1. 현재 branch/SHA와 `git status --short`를 기록한다.
2. 현재 Xbin 모델, RULE launch, `/hybrid/rule_candidate`, `/xycar_motor`
   publisher를 확인한다.
3. `origin/gsw`를 fetch하고 이 README 및 상세 README를 읽는다.
4. 최신 자이카 RULE 튜닝은 그대로 두고 `shortcut_entry_review`, 하이브리드
   신호등/left_4 gate, launch 연결만 병합한다.
5. `no_red_car_best.pt`를 위 경로에 배치하고 SHA256을 비교한 뒤 launch의
   기본 객체 모델이 이 파일인지 확인한다. 기존 모델로 fallback하지 않는다.
6. `left_4` threshold 0.40, 확인 2프레임, 소실 2프레임인지 확인한다.
7. shadow 실행에서 speed가 모든 모드에서 RULE 후보와 같은지 확인한다.
8. `/xycar_motor` publisher가 정확히 하나일 때만 `drive_enabled:=true`로
   실차 실행한다.

자이카 PC Codex에 그대로 붙여 넣을 프롬프트:

```text
작업 경로의 미커밋 변경을 절대 삭제하거나 롤백하지 마. 먼저 현재 branch,
SHA, git status와 Xbin RULE/신호등/라바콘/차량회피/지름길 구조, 모델 경로,
/hybrid/rule_candidate 및 /xycar_motor publisher를 읽기 전용으로 설명해.
그 다음 origin/gsw를 fetch하고 루트 README와
xycar_ws/src/shortcut_entry_review/README.md를 모두 읽어. 자이카의 최신
agent/yellow-center-curve-test RULE 튜닝은 보존하고 필요한 hunk만 수동 병합해.
평상시는 기존 Xbin RULE, left_4>=0.40 2프레임 확인 후 2프레임 소실 시
LR-ASPP/W1 검색, 거리 gate 뒤 W1 조향, 이후 기존 ShortcutCore 조향 순서로
연결해. 모든 모드의 speed는 현재 RULE speed를 그대로 유지해.
객체 모델 기본값을 xycar_ws/src/study/my_rule/models/no_red_car_best.pt로
바꾸고 SHA256 27904321a059ff0290df0c152e9421e7b3792711a121133ee4ac51cf830b79b2를
검증해. 기존 객체 모델로 fallback하지 마. [MISSION] left_4 감지/2프레임
소실/W1 lock/진입 조향각/ShortcutCore handoff 로그가 보이게 유지해.
빌드와 테스트, shadow 주행, /xycar_motor 단일 publisher를 확인하고 실제
motor 출력은 내가 승인하기 전 켜지 마. 무엇을 가져왔고 어떤 로컬 hunk를
보존했는지 마지막에 간단히 목록으로 보고해.
```

## 빌드와 실행

```bash
cd /home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to wide_camera xycar_vesc_driver \
  my_rule_msgs my_rule lane_seg_control xycar_rule_drive track_drive_sve \
  shortcut_entry_review xycar_map_nav
source install/setup.bash
```

통합 shadow(모터 미발행):

```bash
ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  drive_enabled:=false force_rule_only:=false start_shortcut:=true \
  shortcut_semantic_entry_enabled:=true speed_command:=18.0
```

실차는 바퀴를 띄운 검증과 publisher 확인 뒤에만 `drive_enabled:=true`로
바꾼다. 조향 진입 거리 튜닝 옵션은 다음과 같다.

```text
shortcut_entry_minimum_start_distance_m:=0.55
shortcut_entry_full_control_distance_m:=0.15
shortcut_entry_control_latency_sec:=0.25
shortcut_entry_distance_margin_m:=0.08
```

검증용 뷰어는 저장 rosbag의 RULE 후보와 새 W1 조향을 함께 표시한다.

```bash
cd /home/kai/kookmin_autonomous_competition_teamKAI
./run_w1_viewer.sh
```

SPACE 일시정지/재생, F 전체화면, Q 종료다. 뷰어와 semantic stack은 직접
`/xycar_motor`를 발행하지 않는다.
