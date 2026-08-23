# 주차 내비게이션 설계 근거

## 경기장 관찰

첨부된 현실 경기장은 낮은 회색 판넬과 라바콘으로 구성된 약 7 m 규모의
폐쇄형 free-space 환경이다. 차선은 일부 통로 안내용일 뿐 A/B 주차 자세를
만드는 구속조건은 아니다. 따라서 카메라 차선 추종보다 다음 정보가 우선한다.

1. PGM 정적 구조물
2. 360도 LiDAR가 검출한 판넬·라바콘의 실시간 장애물
3. VESC 이동거리와 IMU yaw로 구성한 연속 odometry
4. 정적 지도에 대한 AMCL pose와 covariance

## 연구 결과를 적용한 부분

2026년 협소공간 주차 연구는 주차 진입점을 먼저 정하고, Hybrid-A*와
Reeds-Shepp 연결, 직사각형/다중원 차체 충돌검사를 계층적으로 사용하는
구조가 계산시간과 주차 성공률을 개선한다고 보고한다.

- [Hierarchical Path Planning for Automatic Parking in Constrained Scenarios via Entry-Point Guidance](https://www.mdpi.com/2075-1702/14/1/112)
- [An Anti-Y-shaped efficient parking path planning method for restricted areas](https://doi.org/10.1016/j.robot.2026.105618)
- [An Efficient Improved Bidirectional Hybrid A* Algorithm for Autonomous Parking in Narrow Parking Slots](https://www.mdpi.com/2076-3417/16/4/1897)

이에 따라 `A_ENTRY`, `B_ENTRY`를 공식 목표와 분리했고, Reeds-Shepp
Hybrid-A*에 후진/방향전환 비용을 설정했다. 검색과 제어 모두 실측 조향의 약한
쪽 곡률 `1.502435 1/m`에서 계산한 최소 회전반경을 올림한 `0.67 m`보다 급한
회전을 요구하지 않는다. 각 주차·출차·복귀 경로의 곡률 변화점과 방향전환점을
별도 goal로 둔 31단계 계층 미션으로 조향 지연도 경로 구조에 반영했다.

2025년 연구는 LiDAR occupancy grid가 시뮬레이션과 실환경 사이 표현 차이를
줄이는 데 효과적임을 보인다. 다만 학습 정책은 데이터 분포 밖 행동을 실차에서
검증하기 어렵다.

- [RL-OGM-Parking: Lidar OGM-Based Hybrid Reinforcement Learning Planner for Autonomous Parking](https://arxiv.org/abs/2502.18846)

따라서 본 구현은 동일한 LiDAR OGM 입력 장점은 사용하지만, 제어권은 검증 가능한
결정론적 planner/controller에 둔다.

동적 장애물이 있는 주차 연구는 시간축을 포함한 Hybrid-A*와 적응형 중간 목표,
해를 찾지 못했을 때 정지를 안전한 기본 동작으로 사용한다.

- [Graph-based Path Planning with Dynamic Obstacle Avoidance for Autonomous Parking](https://arxiv.org/abs/2504.12616)

대회 경기장은 보행자 예측까지 요구하지 않으므로 시간축 planner 전체를 넣는
대신, 10 Hz local costmap 갱신, 1 Hz 전역 재계획과 독립 LiDAR
swept-footprint stop shield를 적용했다. 센서/위치추정/계획 중 하나라도
불확실하면 정지한다.

2025년 Hybrid-A*+NMPC 실차 연구는 Hybrid-A*로 전역 초기 궤적을 만들고
NMPC로 제약을 만족시키는 계층 구조를 검증했다.

- [Automatic parking trajectory planning in narrow spaces based on Hybrid A* and NMPC](https://www.nature.com/articles/s41598-025-85541-x)

현재 Xycar에는 최저 구동 명령과 조향 지연은 측정됐지만 타이어 slip 및 steering
rate 모델 식별이 충분하지 않아 NMPC가 잘못된 모델을 정밀하게 추종할 위험이
있다. 이번 구현은 Nav2 MPPI의 Ackermann 운동모델·full-footprint cost critic과
독립 slew/dwell 제한을 사용한다. 실차 parking bag으로 모델을 식별한 뒤 NMPC를
교체 후보로 둘 수 있다.

## ROS 2 구현 선택

Nav2 Smac Hybrid-A*는 `REEDS_SHEPP`, 실제 최소 회전반경, reverse/change
penalty를 제공한다. MPPI는 Ackermann 운동모델, 전·후진 속도범위, 경로 inversion
정지점과 footprint cost critic을 함께 제공한다. RPP도 cusp 후진 기능은 있지만,
Humble 계열에서 Ackermann 주차 경로가 방향 구간을 건너뛰는 사례가 보고되어
실측 어댑터를 포함한 본 경기 경로에서는 MPPI를 선택했다.

- [Nav2 Smac Hybrid-A*](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)
- [Nav2 MPPI Controller](https://docs.nav2.org/configuration/packages/configuring-mppic.html)
- [Nav2 AMCL](https://docs.nav2.org/configuration/packages/configuring-amcl.html)
- [RPP Ackermann manoeuvre issue](https://github.com/ros-navigation/navigation2/issues/4757)

설정은 `REEDS_SHEPP`, MPPI `motion_model: Ackermann`, `vx_min < 0`,
`enforce_path_inversion: true`이며 custom Behavior Tree에서도 `Spin`, `BackUp`,
`DriveOnHeading` recovery를 제거했다. MPPI가 요구한 곡률이 실측 lookup 범위를
넘으면 어댑터는 실현 가능한 곡률로 포화시키며, 조향 명령이 목표에서 3 command
이내로 정렬될 때까지 traction을 열지 않는다.

## 지도·차체·안전 여유

대회 원본 PGM의 회색 205 픽셀은 원본 `free_thresh: 0.25`에서 FREE가 된다.
Nav2 trinary 변환 규칙을 그대로 검증한 결과 원본에서는 UNKNOWN 셀이 0개가 되어
벽 바깥 회색 영역까지 계획 가능 공간이 된다. 실행용 YAML만 `free_thresh:
0.196`으로 낮춰 회색을 UNKNOWN으로 보존했고, PGM·해상도·origin과 원본 YAML은
변경하지 않았다.

차량 치수의 근거는 이 저장소의 `scripts/generate_kookmin_track.py`와 여기서 생성된
`worlds/kookmin_xycar_track_final.sdf`다. 생성기에는 실차 측정 기반 wheelbase
0.32 m, 차체 0.50 x 0.20 x 0.12 m, 직진 타이어 외폭 0.28 m, 휠 반경 0.06 m,
차량 중심 기준 전·후륜축 `x=+0.16/-0.16 m`, 전륜축 기준 LiDAR
`x=+0.065, z=+0.080 m`가 명시돼 있다. 현재 motor bridge와 기존 실차 제어기도
동일한 wheelbase와 `0.080612 m/s/command` 속도 보정을 사용한다.

생성기 형상에서 전륜축 기준 후단은 약 -0.445 m, LiDAR 전단은 +0.105 m다.
또한 최대 Ackermann 조향 시 안쪽 앞 타이어의 횡방향 외곽이 약 0.171 m까지
나오므로 바깥쪽으로 반올림한 차체 polygon `x=[-0.46, +0.11]`, `y=+-0.18 m`를
사용한다. Nav2 costmap은 물리 여유 45 mm와 지도/위치추정 한 셀 50 mm를 더한
`footprint_padding: 0.095 m`를 쓰고, inflation 반경은 패딩 후 내접반경
0.205 m 이상인 0.21 m다. 실시간 LiDAR shield는 map 오차와 독립인 base frame
점군을 사용하므로 물리 여유 45 mm만 적용한다. 실차의 최소 이동 명령 4는 속도
보정값 기준 약 0.322 m/s다. 연속 명령 4, 20 Hz 제어 네 주기인 반응시간
0.20초, 실차 VESC 드라이버와 같은 감속 1.50 m/s^2에서 계산한 정지거리는 약
0.099 m다. 이보다 긴 0.14 m까지 곡선 swept rectangle을 검사한다.

실차가 명령 4 미만에서 움직이지 않으므로 모든 0이 아닌 Nav2 전진/후진 요청은
각각 `+4`/`-4`로 포화하며 주행 중에는 그 값을 계속 출력한다. 펄스 제어와 어댑터
속도 ramp는 사용하지 않는다. 정지·방향전환·조향정렬·LiDAR 충돌·센서 timeout
때만 즉시 0을 출력한다. 실차 주차 launch는 VESC 내부 acceleration slew를 켜서
연속 목표 명령 4를 유지하면서 물리 가감속을 부드럽게 만든다. 2026-08-23 실차에서
측정한 START/A/B `map -> base_footprint` 좌표를 차량 중심 목표로 변환했으며,
중간 경유점은 기존 31단계의 검증된 기하를 유지한다.

주행 중 새로 생긴 장애물은 `/slam/scan_filtered`에서 local/global obstacle
layer로 동시에 marking한다. 전역 costmap은 10 Hz로 갱신하고 Behavior Tree의
`PipelineSequence`가 검증된 2 Hz로 전역 경로를 다시 계산한다. 차량 회전반경과
footprint를 만족하는 다른 통로가 있으면 Smac Hybrid-A* 경로가 자동으로 바뀐다.
Smac에는 기어 전환비용이
따로 없으므로 검증된 후진·곡률 일관성 regularizer(1.35/1.10/0.20)를 유지한다.
이를 거리 최단에 가깝게 낮춘 실기동에서는 경로 안에 작은 cusp가 반복되어 매번
0.4초 정지했고 20초 동안 첫 단계를 벗어나지 못했다. 따라서 안전한 우회 후보 중
경로 길이뿐 아니라 방향·곡률 일관성까지 포함한 실제 주행시간 최소 경로를 쓴다.
동적 장애물마다 obstacle heuristic을 다시 계산해 이미 막힌 통로의 오래된 비용을
재사용하지 않는다. 관측은 1초 유지하고 LiDAR ray tracing으로 clearing한다.
우회로가 없는 동안에는 LiDAR shield가 모터를 0으로 유지하며, Humble의 정수초
Wait 대신 즉시 재계획하고 0.25초 뒤 미션 재시도로 넘어간다. 제한된 횟수를
넘기면 해당 단계가 실패해 안전 중단으로 넘어간다.

경기 제한 3분은 미션 start 승인 후 첫 목표를 보내는 순간부터 출발지 복귀 완료까지
연속으로 측정한다. 위치추정 일시정지와 장애물 재계획 시간도 경기 시간에 포함한다.
31개 목표 사이의 일반 정지 확인은 0.15초로 줄이고, 방향전환 0.4초 dwell과 조향
정렬 gate는 최종 모터 어댑터에 그대로 남겨 안전성을 유지한다. 실제 주차 성공을
확인하는 A/B 목표에서는 각각 3초간 정지를 유지한다. 남은 시간이 30초가 되면 한글
경고를 출력하고 180초가 되면 진행 중 goal을 취소한 뒤 모터 권한을 닫는다.

B 평행주차의 두 번째 후진 arc와 reverse cusp는 기존 기하 경로보다 x축 양의
방향으로 0.04 m, 다음 전진 보정점은 0.02 m 이동했다. 최종 `B_PARK` Pose는
2026-08-23 실차 측정값으로 보정했다. 이 여유는 MPPI가 앞 단계에 허용오차 범위 내의 얕은 yaw로
도착해도, 연속 명령 4의 0.14 m LiDAR 정지 예상 궤적이 A/B 분리벽과 겹쳐 차량을
영구 정지시키는 상황을 방지하면서 다음 전진 보정 방향도 자연스럽게 유지한다.

## 미션 안전 상태기계

미션 관리자는 시작 pose를 반복 발행한 뒤 AMCL covariance, pose age, 순간 pose
jump가 8회 연속 정상일 때만 READY가 된다. 주행 중 localization이 0.8초 이상
불량하면 현재 Nav2 goal을 취소하고 모터 권한을 닫으며, 회복 후 해당 단계부터
재시도한다. Nav2 성공 뒤에도 현재 AMCL pose가 목표에서 위치 0.12 m, yaw 0.12 rad
안에 있는지 별도로 확인한다. 목표 실패는 단계별 한도만 재시도하고 초과하면
ABORTED로 정지한다.

## 실차에서 반드시 다시 측정할 값

- 공식 Pose의 기준점이 정말 차량 기하 중심인지
- 대회 차량 조립 상태가 Gazebo 기준의 중심→전륜축 0.16 m와 동일한지
- 쉘·LiDAR·최대 조향 타이어가 `x=[-0.46,+0.11], y=+-0.18 m` 안에 드는지
- 바닥 재질과 배터리 전압별 최저 전·후진 명령
- 좌/우 실제 최소 회전반경과 조향 비대칭
- 0.24~0.32 m/s에서 제동거리와 command-to-motion 지연
- LiDAR의 `x/y/yaw`, 판넬 높이에서의 누락률 및 반사 이상치
- AMCL pose covariance와 실제 위치오차의 관계

이 측정 전에는 `drive_enabled:=false`가 유일한 승인 상태다.
