# Kookmin Autonomous Competition Team KAI

국민대 자율주행대회용 Xycar Xbin RULE, 지름길, 라바콘, 차량 회피 통합
저장소다.

- [실차 RULE, 콘, 차량 회피 실행](docs/REAL_CAR_RUNBOOK_KO.md)
- [통합 주행 구현 및 검증 현황](docs/INTEGRATED_RULE_DRIVE_STATUS_20260807_KO.md)

현재 실차 튜닝 브랜치: `agent/yellow-center-curve-test`

## 2026-08-14 실차 튜닝 현황

### 현재 구성

- 일반 차선 및 S자 주행: Xbin 노란 중앙선 기반 RULE 제어
- 곡선 제어: Pure Pursuit 80%, Stanley 20%
- 지름길: `left_4` 확인 후 LR-ASPP W1 진입 제어
- 객체 인지 기본 모델: `xycar_ws/src/study/my_rule/models/final.pt`
- 객체 모델 SHA256:
  `0183997ed5e8510045509e7a36bfe69ff3dc8a64840fc822fa8ff8390e90eff5`
- 객체 클래스: `cone`, `green_4`, `green_car`, `left_4`, `null_4`,
  `red_4`, `yellow_4`, `red_car`
- 객체 모델은 차선 Xbin 모델과 독립적으로 동작하므로 일반 차선 경로 생성은
  이번 모델 교체의 영향을 받지 않는다.

### 확인된 주행 결과

다음 순서로 한 단계씩 속도를 높였으며 모두 실차 주행에 성공했다.

| 직선 속도 | 곡선 속도 | 결과 |
|---:|---:|---|
| 20 | 12 | 일반 차선 및 S자 통과 |
| 20 | 14 | S자 통과 |
| 22 | 14 | 통과 |
| 25 | 14 | 통과 |
| 25 | 16 | 통과 |

현재 효과가 확인된 조향값은 다음과 같다.

- 직선 현재 조향 반영 비율: `STEERING_CURRENT_WEIGHT=0.35`
- 곡선 현재 조향 반영 비율: `STEERING_CURVE_CURRENT_WEIGHT=0.80`
- 곡선 속도: `16`
- 경로 품질 저하 시 속도: `12`

직선에서 간헐적으로 직선을 곡선으로 판단하는 현상이 있어 곡선 판정 기준을
`0.16 -> 0.20 -> 0.24 rad/m` 순서로 높이는 시험을 시작했다. `0.24`는
배터리 저전압 때문에 아직 정상 주행 검증이 끝나지 않은 후보값이다.

### 충전 후 재개 명령

충전 후에는 먼저 `/vehicle/vesc_state`의 `fault_code: 0`과 부하 시 전압을
확인한다. 저속 기준 주행을 한 번 통과한 뒤 아래 목표 설정을 시험한다.

```bash
cd /home/xytron/kookmin_ty/yellow_center_curve_test/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source /home/xytron/xycar_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

STRAIGHT_PATH_CURVATURE_THRESHOLD=0.24 \
STEERING_CURRENT_WEIGHT=0.35 \
STEERING_CURVE_CURRENT_WEIGHT=0.80 \
CURVATURE_SPEED_CONTROL_ENABLED=true \
CURVE_SPEED_COMMAND=16 \
DEGRADED_PATH_SPEED_COMMAND=12 \
XYCAR_ENABLE_RVIZ=false \
bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh \
  25 0.30 20 0
```

`0.24`에서 완만한 곡선 진입이 늦어지면 `0.22`, 직선 오판이 계속되면
주행 로그를 확보한 뒤 기준을 다시 조정한다. 곡선을 놓친 상태에서 속도
`25`가 유지되면 즉시 시험을 중단한다.

### VESC 저전압 진단

속도 명령 `25`가 정상 발행되는데 차량이 움찔거리며 출발하지 않는 현상을
확인했다. 진단 당시 상태는 다음과 같았다.

- `/xycar_motor` publisher는 `space_drive_gate` 하나로 정상
- VESC USB 및 `/dev/ttyMOTOR` 연결 정상
- 정지 상태 전압 약 `8.9V`
- 부하 순간 전압 `6.0~7.5V`까지 하락
- VESC `fault_code: 2`, `UNDER_VOLTAGE`
- 전압 보호기가 가속을 제한하거나 모터 출력을 차단

따라서 이 현상은 차선 또는 조향 파라미터 문제가 아니다. 보호 전압을 낮추지
말고 배터리를 완전히 충전한 뒤 배터리 셀, 커넥터, 전원 스위치 및 VESC
전원선을 확인한다. 충전 후에도 부하 전압이 `7.5V` 아래로 반복해서 내려가면
고속 시험을 중단한다.

전압과 fault 상태 확인:

```bash
ros2 topic echo /vehicle/vesc_state
```

### 다음 시험 순서

1. 충전 후 저속 주행으로 VESC 전압과 fault 재확인
2. `25/16`, 곡률 기준 `0.24` 재검증
3. 라바콘 진입, 경로 유지, RULE 복귀를 단독 시험
4. 차량 검출, 좌우 판단, 추월 및 원경로 복귀를 단독 시험
5. 전체 통합 주행과 rosbag 검증

한 번의 시험에서는 파라미터 하나만 변경하고, 성공한 값만 기본 설정 후보로
승격한다.
