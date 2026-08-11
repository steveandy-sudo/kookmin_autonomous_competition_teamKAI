# 통합 RULE 주행 구현 및 검증 현황 (2026-08-07)

이 문서는 실차 통합 RULE 주행에서 지금까지 반영한 코드, 시험 결과와 현재
기준 실행 방법을 기록한다. 신호등은 아직 제외하며 주행 우선순위는
`CONE > YOLO+LiDAR AVOIDANCE > CANONICAL RULE`이다.

## 현재 기본 구성

실차 기본 경로 생성은 다시 canonical 방식으로 복구했다.

1. 광각 카메라 원본 MJPEG를 한 번 보정한다.
2. 차선 인지 결과를 `/perception/canonical_road_image`로 발행한다.
3. canonical 이미지에서 노란 중앙선을 연결한다.
4. 노란선이 부족하면 오른쪽 흰 경계선을 이용해 목표 경로를 복원한다.
5. 노란선과 흰선이 모두 유효하면 두 경로를 융합한다.
6. Pure Pursuit와 Stanley를 혼합해 `/hybrid/rule_candidate`를 만든다.
7. 통합 선택기가 콘, 차량 회피, RULE 중 하나를 선택한다.
8. SPACE 게이트가 RUN 상태일 때만 `/xycar_motor`로 발행한다.

`best_512.onnx` 마스크를 고정 BEV로 바로 투영하는 direct-BEV 방식도 비교용
코드로 남아 있지만 기본 실차 주행에는 사용하지 않는다. 이 방식은 ONNX
추론 사이의 경로 누락과 유효 경로 생성 실패가 반복되어 실제 주행이
불안정했다.

## 실행 명령

카메라, LiDAR와 VESC를 연결하고 모터 배터리 상태를 확인한 뒤 실행한다.

```bash
cd /home/xytron/kookmin_ty/integrated_rule_drive_latest
./run_integrated_canonical_drive.sh
```

기존에 사용했던 아래 파일도 호환을 위해 canonical 실행기로 바꿨다.

```bash
./run_integrated_best512_drive.sh
```

실행 시 속도, 조향각 기반 감속, Lookahead, Stanley 비율과 gain, 제어점,
지연 예측, 곡선 조향 배수와 좌측 보정을 차례대로 입력한다. 센서와 제어기
확인이 끝나면 `READY`가 한 번 표시된다.

- `SPACE`: 주행 시작
- `SPACE`: 다시 누르면 정지
- `Ctrl+C`: 전체 종료

현재 주요 기본값은 다음과 같다.

| 항목 | 기본값 |
| --- | ---: |
| 곡선 Lookahead | 0.30 m |
| 곡선 Stanley 비율 | 20% |
| Pure Pursuit 제어점 X | -0.08 m |
| Stanley 제어점 X | 0.16 m |
| 곡선 Stanley gain | 1.20 |
| 직선 Stanley 비율 | 90% |
| 직선 Stanley gain | 0.50 |
| 방향 상충 시 Stanley 비율 | 70% |
| 제어 지연 예측 | 0.35 s |
| 좌측 목표 보정 | 12 cm |
| 콘 속도 command | 6.0 |
| 콘 센서 소실 유지 | 0.50 s |
| 최대 조향 command | -42 ~ +42 |

실행에 사용한 모든 값은 `/tmp/xycar_hybrid_run_config.yaml`, 상세 제어 로그는
`/tmp/xycar_hybrid_control_*.log`에 저장된다.

## 경로와 조향 개선

- 조향 smoothing의 현재값 반영 비율을 직선 `0.40`, 곡선 `0.70`으로 올려
  S자 방향 전환 응답을 빠르게 했다.
- 경로 곡률은 한 구간 평균만 보지 않고 설정한 전방 구간을 여러 조각으로
  나눠 가장 큰 heading 변화량도 확인할 수 있게 했다.
- 큰 곡선 조향 명령에 배수를 적용하는 기능을 추가했지만 기본값은 OFF다.
- 조향각 기반 속도 감속은 실행 시 켜거나 끌 수 있다. 기본 동작은
  `|angle|<=20`에서 입력 상한, `20~42`에서 선형 감속, `42`에서 command
  `8`이다.
- 좌측 목표 보정은 실행 시 cm 단위로 변경할 수 있고 현재 기본값은 12cm다.
- PP와 Stanley의 기준점, gain, softening, 혼합 비율과 latency preview를
  모두 실행 시작 시 조정할 수 있다.

## 콘과 차량 회피

- 웨이포인트 기반 RULE/RL 전환은 제거했다.
- 콘 모드는 YOLO 또는 LiDAR 콘이 남아 있는 동안 유지하고, 둘 다 설정 시간
  동안 사라졌을 때만 RULE로 복귀한다.
- 콘 조향각은 실측 관계를 이용해 실제 바퀴 각도에 대응하는 command로
  변환하고 -42~+42에서 제한한다.
- 차량은 YOLO 한 프레임 검출로 회피를 시작할 수 있다.
- 카메라의 노란 중앙선 기준으로 장애물 좌우를 결정하되, 좌우 확정은
  직선 구간에서만 수행한다. 곡선에서 잘못 뒤집히는 판단을 막기 위해서다.
- LiDAR는 장애물 거리와 좌우 빈 공간의 계측값으로 사용한다. 즉시 회피
  모드에서는 엄격한 LiDAR 군집 연관 실패가 진입 자체를 막지 않는다.
- 현재 회피 이동량은 왼쪽 `0.28m`, 오른쪽 `0.31m`, 회피 속도 상한은
  command `8`이다.
- 통합 선택기는 후보 하나가 짧게 누락됐을 때 바로 다른 모드로 튀지 않도록
  마지막 유효 상태와 센서 소실 유지 시간을 사용한다.

## VESC 실차 설정

- ROS 조향 command 0의 의미를 유지하면서 서보 출력에만 `-5` command의
  중앙 trim을 적용했다.
- 가속 slew는 ON/OFF 가능하다.
- 현재 가속 제한은 `0.6m/s^2`로, command 0에서 10까지 약 1.3초에
  올라가도록 이전 `0.3m/s^2`보다 완화했다.
- 배터리 저전압 보호와 VESC telemetry는 유지한다.

## 중앙선 추출 실측

2026-08-07 통합주행 rosbag 세 개에서 canonical 입력과 실제 제어 경로
`/rule_drive/connected_yellow_path`를 비교했다.

| bag | canonical Hz | 유효 경로 Hz | 생성 성공률 |
| --- | ---: | ---: | ---: |
| `rule_tuning_20260807_132533` | 11.81 | 10.11 | 84.9% |
| `rule_tuning_120260807_133830` | 14.66 | 13.90 | 91.7% |
| `rule_tuning_220260807_134142` | 14.96 | 12.60 | 72.6% |

전체 `6,937` canonical 프레임 중 `5,741`프레임에서 경로가 생성돼 전체
성공률은 `82.8%`다. 노란 중앙선이 단독 또는 흰선과 융합돼 사용된 비율은
전체 입력의 약 `76.2%`, 흰선 단독 fallback은 약 `6.6%`다.

유효 경로는 항상 32개 점으로 재표본화된다. 실제 관측 선의 평균 전방
길이는 약 `0.58~0.63m`, 피팅된 제어 경로의 평균 길이는 약
`0.84~0.91m`, 최대 길이는 `1.49m`였다. 최신 bag은 차선 픽셀이 보이는
비율이 `93.3%`였지만 유효 경로는 `71.7%`뿐이었다. 따라서 현재 주요
병목은 차선 픽셀의 완전한 소실보다 짧고 끊긴 선이 연결·피팅 조건을
통과하지 못하는 현상이다.

실시간 처리율은 다음으로 확인한다.

```bash
ros2 topic hz /perception/canonical_road_image
ros2 topic hz /rule_drive/connected_yellow_path
ros2 topic hz /hybrid/rule_candidate
ros2 topic echo /rule_drive/diagnostics
```

## direct-BEV 비교 도구

비교 실험 코드는 기존 canonical을 변경하지 않고 선택적으로 실행할 수 있게
분리했다. `best_512.onnx` 및 다른 차선 모델의 마스크, 고정 BEV와 경로
생성 결과를 rosbag에서 같은 프레임으로 비교할 수 있다.

- 비교 스크립트: `scripts/compare_lane_models_bag.py`
- direct-BEV launch: `lane_seg_control/direct_bev_stanley_pursuit.launch.py`
- 픽셀 경로를 metric Centerline으로 변환하는 노드:
  `lane_seg_control/bev_path_centerline_node.py`

이 코드는 현재 기본 실차 실행 경로가 아니며, 다시 적용하려면 canonical보다
높은 경로 생성 성공률과 안정적인 방향 전환을 먼저 rosbag에서 확인해야 한다.

## 검증과 다음 작업

- `xycar_rule_drive`, `xycar_map_nav` symlink build 통과
- 관련 Python/launch 구문 검사 통과
- 통합 제어 테스트 101개 통과

다음 튜닝은 한 번에 한 항목만 바꾸고 매번 rosbag과 실행 설정을 함께
남긴다.

1. canonical 연결·피팅 실패 구간을 줄여 경로 성공률 95% 이상 확보
2. S자에서 좌우 raw steering과 최종 steering의 부호 전환 시점 검증
3. 속도 3부터 시작해 6, 8, 10 순서로 PP/Stanley 값 조정
4. 고속 차량 회피의 좌우 확정과 복귀 안정성 확인
5. 콘 센서 소실과 RULE 복귀 시 조향·속도 단절 제거
