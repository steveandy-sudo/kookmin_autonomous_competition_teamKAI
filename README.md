# GSW 신호등·W1/Y1 지름길 진입 통합

`gsw` 브랜치는 기존 자이카 PC의 Xbin RULE을 정상 구간에 그대로 두고,
`left_4` 이후 지름길 진입 구간에서만 흰선·노란선 semantic 모델을 사용하는
통합 reference다.

상세한 제어 구조, 모델 SHA, rosbag 검증 결과, 자이카 PC 충돌 방지 이식
절차는 다음 문서를 먼저 읽는다.

- [`xycar_ws/src/shortcut_entry_review/README.md`](xycar_ws/src/shortcut_entry_review/README.md)

## 현재 지름길 진입 동작

```text
평상시: 기존 Xbin -> 기존 RULE -> 최종 hybrid selector

left_4 2프레임 확인 후 2프레임 미검출(S)
  -> 기존 RULE 조향 유지, 전진 속도만 기본 최대 4
  -> 지름길 전용 LR-ASPP와 W1 검색 시작

W1 lock
  -> RULE override 허용
  -> W1 60% + Y1 40% 진입 경로
  -> 기존 canonical Stanley/Pure-Pursuit로 조향

W1/Y1 전방 정렬 완료
  -> 기존 ShortcutCore cruise/T-exit로 인계
  -> 자이카 PC에 원래 튜닝된 지름길 속도로 복귀
```

- 진입에는 W1/Y1만 사용하고 W2/Y2는 무시한다.
- Y1이 아직 보이지 않으면 W1과 예상 차폭으로 임시 경로를 만들며 Y2를
  대신 사용하지 않는다.
- 정상 Xbin 모델, 정상 RULE 토픽과 조향 보정값은 바꾸지 않는다.
- semantic stack은 직접 `/xycar_motor`를 발행하지 않는다.
- 최종 `/xycar_motor` publisher는 실차에서 반드시 하나여야 한다.

기본 진입 설정은 다음과 같다.

```text
shortcut_entry_speed_command:=4.0
shortcut_entry_w1_path_weight:=0.60
shortcut_entry_search_timeout_sec:=12.0
```

`shortcut_entry_w1_path_weight`는 `0.50 <= 값 < 1.0` 범위이며 값이 커질수록
흰선 W1 쪽 비중이 커진다. 진입 완료 후 기존 `ShortcutCore`에는 이 비중과
속도 제한이 적용되지 않는다.

새 진입 제어기에서 기존 `ShortcutCore`로 넘어가는 조건은 W1/Y1이 차체
전방축에 3개 semantic 프레임 연속 정렬되는 순간이다. 터미널의
`SHORTCUT HANDOFF REQUEST`는 전환 요청, `SHORTCUT HANDOFF GATE`는 인계
gate, `SHORTCUT CONTROL SWITCHED`는 기존 제어기의 첫 fresh 명령을 실제로
사용하기 시작한 시점을 뜻한다. 상세 필드와 확인 토픽은 위 상세 README의
“새 진입 제어기에서 기존 지름길 제어기로 넘어가는 시점” 절을 따른다.

## 자이카 PC로 가져올 때

자이카 PC의 기존 작업 트리가 더러우면 `gsw`로 checkout하거나 디렉터리를
덮어쓰지 않는다. 먼저 현재 Xbin, RULE, 신호등, shortcut, motor publisher
구조를 읽기 전용으로 조사한 뒤 관련 hunk만 병합한다.

자이카 PC Codex에는 다음 지시를 전달한다.

```text
현재 자이카 PC 로컬의 하이브리드 주행제어기 구조와 미커밋 변경을 먼저 설명해.
신호등/left_4 처리가 없으면 origin/gsw의 README를 읽고 필요한 hunk만 가져와.
기존 Xbin RULE과 조향 보정은 보존하고 S 이후 semantic W1/Y1 진입만 통합해.
진입은 W1 60%·Y1 40%, 속도 4이며 완료 후 기존 ShortcutCore 속도로 복귀해.
실차 전 shadow 검증하고 /xycar_motor publisher가 정확히 하나인지 확인해.
```

## 빌드

```bash
cd xycar_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to \
  wide_camera xycar_vesc_driver my_rule_msgs my_rule lane_seg_control \
  xycar_rule_drive track_drive_sve shortcut_entry_review xycar_map_nav
source install/setup.bash
```

## 통합 shadow 실행

자이카 PC의 기존 Xbin이 `/hybrid/rule_candidate`를 이미 발행한다면 다음처럼
기존 perception/RULE을 중복 실행하지 않는다.

```bash
ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  drive_enabled:=false start_perception:=false start_rule:=false \
  start_shortcut:=true shortcut_semantic_entry_enabled:=true \
  shortcut_entry_speed_command:=4.0 \
  shortcut_entry_w1_path_weight:=0.60
```

실차 RUN 전에 다음을 확인한다.

```bash
ros2 topic info /hybrid/shortcut_candidate --verbose
ros2 topic info /xycar_motor --verbose
ros2 node list
```

rosbag RViz/OpenCV 검증과 충돌 없는 수동 이식 표는
[`shortcut_entry_review/README.md`](xycar_ws/src/shortcut_entry_review/README.md)에
기록되어 있다.
