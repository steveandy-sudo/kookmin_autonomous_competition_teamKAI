# 지름길 주행 개발 기록 (2026-08-12)

## 문서 상태

- 기준 브랜치: `agent/yellow-center-curve-test`
- 복구 기준 커밋: `978a13f6fd5de535f463232b373548103ba24630`
- 상태: 실차 통합 전 실험 단계에서 중단
- 처리: 아래 구현 코드는 기준 커밋으로 롤백했으며 이 문서만 개발 기록으로 남긴다.

이 문서는 삭제된 지름길 주행 실험 코드를 다시 활성화하는 실행 문서가 아니다. 이후 재개할 때 설계 의도, 구현 범위, 검증 결과와 미해결 문제를 확인하기 위한 인수인계 기록이다.

## 목표

기존 XBin 기반 RULE 주행과 조향 보정을 유지하면서 신호등 이후 지름길 구간만 별도의 semantic 진입 제어기로 통과시키는 구조를 시험했다.

1. YOLO 신호등 클래스로 지름길 진입 시점을 결정한다.
2. 진입부에서 LR-ASPP 차선 분할 결과의 `W1`, `W2`, `Y1`, `Y2`를 순서와 위치로 구분한다.
3. `W1` 60%, `Y1` 40%의 목표 경로를 생성한다.
4. semantic 진입이 끝나면 기존 `ShortcutCore`의 CRUISE/EXIT 로직으로 제어권을 넘긴다.
5. 후보 명령은 직접 `/xycar_motor`를 발행하지 않고 기존 하이브리드 선택기와 SPACE 게이트를 거친다.

## 구현했던 구조

제어 흐름은 다음과 같았다.

```text
/wide_camera_mjpeg/image_raw/compressed
  -> my_rule YOLO object detector
  -> traffic-light / shortcut trigger
  -> /hybrid/shortcut_processing_enabled
  -> gated LR-ASPP lane segmentation
  -> W1/W2/Y1/Y2 sequence selector
  -> /shortcut/entry/selected_centerline
  -> Stanley/Pure Pursuit entry controller
  -> semantic/legacy candidate mux
  -> /hybrid/shortcut_candidate
  -> sequential_hybrid_driver
  -> /hybrid_gate/xycar_motor_shadow
  -> SPACE gate
  -> /xycar_motor
```

### 신호등과 트리거

- 네 칸 신호등 클래스 `red_4`, `yellow_4`, `green_4`, `left_4`를 일반 신호 클래스와 분리했다.
- 최종 실험 상태에서는 `green_4`를 2프레임 연속, confidence 0.50 이상 검출하면 지름길 검색을 시작하도록 구성했다.
- 빨강/노랑 정지는 박스 면적 비율 0.025 이상, 2프레임 연속 검출을 기준으로 시험했다.
- 지름길 모드가 활성화된 뒤에는 배경의 신호등 검출이 진행 중인 지름길 미션을 중단하지 않도록 래치했다.
- YOLO에서 허용된 클래스, 개수와 최대 confidence를 `[YOLO 객체]` 로그로 출력하는 기능을 추가했었다.

### Semantic W1/Y1 진입

- 별도 실험 패키지 이름: `shortcut_entry_review`
- 입력 모델: `kookmin_lane_lraspp_mbv3s_256x144.pt`
- 입력 카메라는 처리 활성화 신호가 들어올 때만 별도 인지 노드로 전달했다.
- 흰 경계 후보를 `W1/W2`, 노란 경계 후보를 `Y1/Y2`로 구분하고 이전 프레임과의 연속성을 추적했다.
- 목표 경로 기본 가중치는 `W1=0.60`, `Y1=0.40`이었다.
- Y1을 아직 얻지 못한 경우에는 측정된 차로 폭을 이용한 합성 Y1 경로를 사용했다.
- W1/Y1이 차량 전방축과 3프레임 연속 정렬되면 기존 ShortcutCore로 handoff했다.
- 진입 목표의 우측 이동량을 cm 단위 실행 인자로 받을 수 있게 했고 마지막 기본 시험값은 30 cm였다.

### 기존 ShortcutCore 연동

- 기존 `ShortcutCore`에 `start_cruise()` 진입점을 추가해 semantic 진입 이후 기존 시간 기반 ENTER를 반복하지 않고 CRUISE부터 시작하게 했다.
- 별도 후보 노드는 `[angle, speed, done, phase]` 형식으로 명령을 발행했다.
- semantic 후보와 기존 ShortcutCore 후보를 하나의 `/hybrid/shortcut_candidate`로 합치는 MUX를 사용했다.
- 완료 플래그가 들어오면 지름길 래치를 끝내고 기존 canonical RULE로 복귀하게 했다.

## 주요 시험 파라미터

| 항목 | 시험값 |
| --- | ---: |
| 지름길 진입 속도 command | 4.0 |
| W1 경로 가중치 | 0.60 |
| Y1 경로 가중치 | 0.40 |
| 지름길 목표 우측 보정 | 0.30 m |
| 진입 경로 탐색 제한 시간 | 12.0 s |
| semantic/legacy 후보 timeout | 0.35 s |
| 외부 경로 timeout | 0.35 s |
| 지름길 후보 최대 조향 | +/-42 command |
| 진입 인지 최대 출력률 | 15 Hz |
| 진입 제어/후보 MUX 주기 | 20 Hz |

## 사용했던 주요 토픽

| 토픽 | 의미 |
| --- | --- |
| `/hybrid/shortcut_processing_enabled` | 지름길 인지 처리 활성화 |
| `/shortcut/lraspp/white_mask` | 지름길용 흰 경계 마스크 |
| `/shortcut/lraspp/yellow_mask` | 지름길용 노란 경계 마스크 |
| `/shortcut/entry/selected_centerline` | W1/Y1로 만든 진입 목표 경로 |
| `/shortcut/entry/ready` | 진입 제어 시작 가능 상태 |
| `/shortcut/entry/path_valid` | semantic 진입 경로 유효 여부 |
| `/shortcut/entry/cruise_enabled` | 기존 ShortcutCore handoff 요청 |
| `/shortcut/entry/controller_candidate` | semantic 진입 조향/속도 후보 |
| `/shortcut/legacy_cruise_candidate` | 기존 ShortcutCore 후보 |
| `/shortcut/entry/mux_status` | 후보 선택 또는 safe-stop 이유 |
| `/hybrid/shortcut_candidate` | 하이브리드 선택기로 전달한 최종 지름길 후보 |
| `/hybrid/traffic_light_status` | 신호등 상태와 지름길 트리거 진단 |

`/shortcut/entry/diagnostics` 배열에는 phase, ready, path_valid, 합성 Y1 사용 여부, W1/Y1 위치와 기울기, 후보 개수, W1/W2/Y1/Y2 검출 여부를 기록하도록 구성했었다.

## 검증했던 항목

- W1 첫 검출과 W2 오선택 방지
- Y1 순차 확정과 짧은 Y1 조각 허용
- W1 60% / Y1 40% 경로 생성
- W1 또는 Y1 짧은 소실 시 이전 형상 유지
- 전방 정렬 후 기존 ShortcutCore로 handoff
- 지름길 트리거 2프레임 확인과 재진입 방지
- 빨강/노랑 정지와 초록 해제 상태 머신
- 지름길 활성 중 배경 신호등 무시
- 실행 인자를 통한 지름길 우측 보정값 전달

관련 단위 테스트와 패키지 빌드는 개발 중 통과했지만, 이 구현은 커밋되지 않은 실험 상태였으므로 재사용하려면 현재 브랜치 기준으로 다시 구현하고 전체 테스트를 재수행해야 한다.

## 미해결 문제

실차에서 지름길 모드가 진행 중 간헐적으로 정지한 뒤 다시 움직이는 현상이 있었다. `/xycar_motor` 발행 노드 자체가 사라진다기보다 안전 조건이 속도 0 후보를 계속 발행하는 구조였다.

확인된 정지 경로는 다음과 같다.

1. W1을 허용 프레임보다 오래 놓치면 `path_valid=false`가 되어 `W1 unavailable; safe stop`이 발생했다.
2. W1/Y1 형상으로 유효 경로를 만들지 못하면 `W1/Y1 geometry invalid; safe stop`이 발생했다.
3. semantic 진입 제어 후보가 0.35초 이상 갱신되지 않으면 MUX가 `entry path/controller unavailable; safe stop`을 발행했다.
4. handoff 직후 기존 ShortcutCore 후보가 준비되지 않거나 0.35초 이상 오래되면 `legacy cruise candidate stale; safe stop`이 발생했다.
5. 원본 카메라가 0.35초 이상 오래되면 기존 ShortcutCore 후보 노드가 발행을 멈췄다.
6. 하이브리드 선택기의 지름길 후보가 0.35초 이상 오래되면 `shortcut camera/candidate stale`로 정지했다.
7. SPACE 게이트가 받은 후보 속도가 0 이하이면 최종 상태가 `SELECTOR_STOP`이 되었다.

15 Hz 인지와 여러 단계의 0.35초 timeout이 연속되어 CPU 지연, 짧은 차선 소실 또는 semantic/legacy handoff 순간에 정지와 재출발이 발생할 가능성이 높았다.

## 재개 시 권장 순서

1. 저장된 rosbag에서 W1/W2/Y1/Y2 역할 분류와 `path_valid`를 먼저 재검증한다.
2. semantic 진입과 기존 ShortcutCore를 동시에 실행하되 모터를 발행하지 않는 shadow mode로 후보 연속성을 측정한다.
3. `/shortcut/entry/mux_status`, `/shortcut/entry/status`, `/hybrid_gate/status`의 최초 정지 이유를 같은 시간축으로 기록한다.
4. timeout을 단순히 늘리기 전에 W1 소실, 외부 경로 소실, handoff 공백 중 어느 단계가 최초 원인인지 구분한다.
5. handoff 시 첫 legacy 후보가 준비될 때까지 마지막 유효 semantic 후보를 짧게 유지하는 원자적 전환을 검토한다.
6. 정상 경로의 한두 프레임 소실에는 마지막 유효 경로를 제한 시간 동안 유지하되, 잘못된 경로를 무기한 유지하지 않도록 상한을 둔다.
7. `/xycar_motor` publisher가 SPACE 게이트 하나인지 확인한 후 정지 상태, 저속 순서로 실차 검증한다.

## 롤백으로 제거된 실험 범위

- `shortcut_entry_review` 전체 패키지
- `track_drive_sve/shortcut_candidate_node.py`와 관련 테스트
- `ShortcutCore.start_cruise()` 추가
- `xycar_map_nav`의 지름길/신호등 래치와 상태 머신
- semantic entry launch 연결과 실행 스크립트 인자
- W1/W2/Y1/Y2 및 YOLO 객체 로그 확장
- 지름길 관련 단위 테스트와 패키지 의존성

현재 실행 코드는 원격 `agent/yellow-center-curve-test`의 `978a13f` 상태이며, 위 기능은 이 문서 외에는 저장소에 남아 있지 않다.
