# 실차 통합 룰베이스 주행 및 튜닝 가이드

## 1. 목적

이 브랜치는 국민대 실차 Xycar에서 다음 기능을 한 실행 흐름으로 시험하기
위한 기준 코드다.

- 카메라 기반 canonical 차선 인지
- Pure Pursuit와 Stanley를 혼합한 룰베이스 조향
- YOLO와 LiDAR를 이용한 장애물 추적 및 회피
- YOLO로 진입을 판단하고 LiDAR 콘 경로를 따르는 라바콘 주행
- SPACE 키로 주행 시작과 정지
- 압축 카메라, 제어 명령, 인지 결과 및 실제 launch 입력값 기록

현재 기본 주행원은 룰베이스다. 강화학습 후보는 실행하지 않으며 우선순위는
다음과 같다.

```text
CONE > YOLO+LiDAR 장애물 회피 > RULE
```

신호등 제어는 아직 본선 주행에 활성화하지 않았다. 충분한 rosbag 검증 후
마지막 단계에서 통합한다.

객체 인식 모델은 다음 파일을 사용한다.

```text
xycar_ws/src/study/my_rule/models/kookmin_objects_best_20260804.pt
SHA256: 875117819232ba81e385ca0b772e4c5dfdb868abb084e95714d95dc084767070
```

## 2. 현재 실차 기본값

| 항목 | 기본값 | 비고 |
|---|---:|---|
| 일반 주행 속도 command | 3.0 | 시작 시 3.0~30.0 입력 |
| 곡선 lookahead distance | 0.50 m | 시작 시 변경 가능 |
| 곡선 Stanley 비율 | 10% | Pure Pursuit는 자동으로 90% |
| 좌측 목표 보정 | 9 cm | BEV 영점 보정 후 기존 실차 궤적을 유지하는 시작값 |
| 콘 주행 속도 command | 6.0 | 일반 주행 속도와 분리 |
| 최대 조향 command | +/-42 | 하드웨어 명령 한계 |
| 룰 명령 주기 | 10 Hz | canonical 입력이 정상일 때 |

곡선 제어의 기본 혼합은 다음과 같다.

```text
steering = 0.90 * pure_pursuit + 0.10 * stanley
```

시작할 때 Stanley 비율을 `S` 퍼센트로 입력하면 스크립트가
`pure_pursuit_weight = 1 - S / 100`으로 변환한다. 직선 판정 구간은
`straight_*` 파라미터를 별도로 사용하므로 위 비율은 주로 곡선 구간의
응답을 조절한다.

LD를 줄이면 가까운 경로점을 보므로 곡선 진입이 빨라지지만 조향 진동이
커질 수 있다. Stanley 비율을 높이면 횡오차 복원이 강해지지만 노이즈와
좌우 반전에도 민감해질 수 있다.

## 3. 주요 파일과 역할

### `real_hybrid_test_sensors.launch.py`

실차 카메라, LiDAR, native ROS 2 VESC 드라이버를 함께 시작한다. IMU는
현재 룰베이스 통합 시험에서 사용하지 않는다.

### `real_sequential_hybrid_drive.launch.py`

아래 노드를 한 launch에서 구성한다.

- `lane_seg_lraspp_inference`: canonical 차선 영상 생성
- `canonical_stanley_pursuit_driver`: 룰베이스 조향 후보 생성
- `my_rule_object_detection_node`: 차량, 콘 등 객체 인지
- `my_rule_cone_node`: 라바콘 경로와 조향 명령 생성
- `sequential_hybrid_driver`: RULE, CONE, AVOIDANCE 중 최종 명령 선택

중요 launch 인자는 다음과 같다.

| 인자 | 의미 |
|---|---|
| `speed_command` | 일반 룰베이스 속도 |
| `lookahead_distance_m` | 곡선 Pure Pursuit LD |
| `pure_pursuit_weight` | PP 혼합 비율, Stanley는 `1-value` |
| `target_left_offset_m` | 목표 경로 좌측 보정 |
| `cone_speed_command` | 콘 주행 전용 속도 |
| `force_rule_only` | RULE을 기본 주행원으로 고정, 기본 `true` |

### 실행 스크립트

- `run_complete_rule_only.sh`: 센서 확인부터 SPACE 주행까지 한 번에 실행
- `run_complete_space_hybrid.sh`: 입력 검증과 센서 준비
- `run_space_hybrid_test.sh`: 제어 launch 실행 및 READY 상태 확인

## 4. 빌드

```bash
cd /path/to/your/xycar_ws
set +u
source /opt/ros/humble/setup.bash

colcon build --packages-select \
  wide_camera \
  xycar_vesc_driver \
  lane_seg_control \
  my_rule_msgs \
  my_rule \
  xycar_rule_drive \
  xycar_map_nav \
  --symlink-install

source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE
```

## 5. 실차 실행

모터 배터리, 비상 정지 담당자, 카메라와 LiDAR 고정을 먼저 확인한다.

```bash
cd /path/to/your/xycar_ws
bash src/xycar_map_nav/scripts/run_complete_rule_only.sh
```

스크립트는 다음 값을 순서대로 묻는다. 단위를 붙이지 않고 숫자만 입력한다.

```text
주행 속도 command: 5
곡선 Lookahead distance(m): 0.7
곡선 Stanley 비율(%): 10
좌측 주행 보정 거리(cm): 9
```

모든 토픽이 준비되면 `READY`가 한 번 출력된다. SPACE를 한 번 누르면
주행하고 다시 누르면 정지한다.

## 6. Pure Pursuit와 Stanley 튜닝 계획

먼저 속도, 좌측 보정, 카메라 위치를 고정하고 LD와 Stanley만 바꾼다.
한 번에 두 개 이상의 조건을 바꾸지 않는다.

1차 권장 실험표:

| 시험 | LD(m) | Stanley(%) | 반복 |
|---|---:|---:|---:|
| A | 0.5 | 0 | 3회 |
| B | 0.5 | 10 | 3회 |
| C | 0.5 | 20 | 3회 |
| D | 0.7 | 10 | 3회 |
| E | 0.7 | 20 | 3회 |
| F | 1.0 | 10 | 3회 |

각 주행에서 다음 값을 비교한다.

- 곡선 진입 시 조향 command 상승 시점
- 최대 조향 command와 +/-42 포화 지속시간
- 곡선 탈출 후 반대 조향 횟수와 진폭
- 좌우 차선 중심 오차
- canonical, RULE 후보, 최종 모터 명령의 Hz와 누락 구간
- 완주 시간과 사람이 개입한 시각

트랙 이탈이나 큰 오실레이션이 발생한 세션은 성공 세션과 분리해서
보존한다. 실패 직전 영상과 명령이 다음 파라미터를 정하는 핵심 자료다.

## 7. 조향각 기반 속도 제어 계획

현재 일반 룰 속도는 시작 시 입력한 고정 command다. 다음 단계에서는 최종
조향 command의 절댓값에 따라 속도를 연속적으로 낮추는 프로파일을 시험한다.

초기 시험 기준:

| `abs(steering)` | 목표 속도 동작 |
|---:|---|
| 0~8 | 입력 상한 유지 |
| 8~18 | 상한에서 중간 속도로 선형 감소 |
| 18~30 | 중간 속도에서 곡선 최저 속도로 감소 |
| 30~42 | 곡선 최저 속도 유지 |

속도 프로파일은 LD와 Stanley 조합을 먼저 확정한 뒤 적용한다. 동시에
바꾸면 조향 개선과 감속 효과를 구분할 수 없다. 가속 복귀에는 별도 rate
limit를 둬 곡선 출구에서 급가속하지 않게 해야 한다.

## 8. 남은 미션별 작업

### 장애물 회피

- YOLO 검출 시각과 LiDAR 추적 시작 시각 비교
- 왼쪽/오른쪽 빈 공간 선택의 정확도 확인
- 일시적인 YOLO 또는 LiDAR 누락 시 직전 회피 상태 유지 검증
- 장애물과 최소 횡방향 여유 거리 측정
- 복귀 시 조향 반전과 정지 반복 제거

### 라바콘

- YOLO 콘 검출 후 1m 진입 조건 검증
- 콘 경로의 단일 경계와 양쪽 경계 전환 검증
- 콘 전용 속도 `6.0`에서 S자 완주 여부 확인
- 콘 종료 후 RULE 복귀가 끊기지 않는지 확인

### 신호등

- 현재 룰베이스 통합 시험에서는 비활성
- 클래스별 confidence와 연속 검출 프레임 수 확정
- 정지선과의 거리 조건 추가
- 녹색 전환 오검출 및 timeout을 shadow mode에서 먼저 검증

## 9. 완료 기준

다음 조건을 만족하면 룰베이스 기본값 후보로 승격한다.

- 동일 조건 3회 연속 완주
- 곡선에서 조향 포화 후 반대 방향 큰 진동이 없음
- 카메라 또는 LiDAR의 짧은 누락으로 차량이 반복 정지하지 않음
- 콘과 장애물 구간 진입, 유지, 복귀가 rosbag에서 명확히 구분됨
- 실행 입력값과 실제 노드 파라미터가 세션에 모두 저장됨
- 속도를 높여도 차선 중심 오차와 최소 장애물 거리가 허용 범위 안에 있음

이 기준을 통과한 뒤 신호등과 조향각 기반 동적 속도를 차례로 합친다.
