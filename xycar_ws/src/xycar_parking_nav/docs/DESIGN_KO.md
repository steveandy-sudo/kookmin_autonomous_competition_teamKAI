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

이에 따라 `A_ENTRY`, `B_ENTRY`를 공식 목표와 분리했다. 일반 이동에는 전진 전용
Dubins Hybrid-A*, 주차 진입 arc에는 Reeds-Shepp Hybrid-A*와 높은 방향전환
비용을 적용한다. 검색과 제어 모두 실측 조향의 약한 쪽 곡률 `1.502435 1/m`에서
계산한 최소 회전반경을 올림한 `0.67 m`보다 급한 회전을 요구하지 않는다. 각
주차·출차·복귀 경로의 곡률 변화점과 방향전환점을 별도 goal로 둔 31단계 계층
미션으로 조향 지연도 경로 구조에 반영했다.

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
대신, 구간 시작 시 만든 전역 기준경로와 독립 LiDAR swept-footprint stop shield를
적용했다. 주행 중 전역경로를 반복 교체하지 않고 MPPI가 정적 지도상의 경로를
강하게 추종한다. 실시간 LiDAR는 경로 변형이 아니라 비상정지에만 사용한다.
센서/위치추정/계획 중 하나라도
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

Nav2 Smac Hybrid-A*는 `DUBIN`과 `REEDS_SHEPP`, 실제 최소 회전반경,
reverse/change penalty를 제공한다. MPPI는 Ackermann 운동모델, 속도범위, 경로
inversion 정지점과 footprint cost critic을 함께 제공한다. RPP도 cusp 후진 기능은
있지만, Humble 계열에서 Ackermann 주차 경로가 방향 구간을 건너뛰는 사례가
보고되어 실측 어댑터를 포함한 본 경기 경로에서는 MPPI를 선택했다.

- [Nav2 Smac Hybrid-A*](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)
- [Nav2 MPPI Controller](https://docs.nav2.org/configuration/packages/configuring-mppic.html)
- [Nav2 AMCL](https://docs.nav2.org/configuration/packages/configuring-amcl.html)
- [RPP Ackermann manoeuvre issue](https://github.com/ros-navigation/navigation2/issues/4757)

일반 이동 트리는 `DUBIN`과 `vx_min: 0.0`인 `FollowPathForward`를 사용해 계획과
제어 양쪽에서 먼저 후진을 막는다. 일반 전진 경유점의 Nav2 action이
`ABORTED`되면 차량이 이미 그 지점을 통과해 Dubins 재진입이 불가능할 수 있으므로,
그 단계에 한해서 `REEDS_SHEPP`와 양방향 `FollowPath`로 다시 보낸다. 정밀 cusp와
주차 Pose는 이 fallback 대상에서 제외해 기하 설계의 진행 방향을 보존한다.
`allow_reverse: true`인 정밀 정렬 단계는 처음부터
`REEDS_SHEPP`, `vx_min < 0`인 `FollowPath`를 쓰되 경로를 한 번만 계산한다.
최종 A/B 주차 단계는 같은 Reeds-Shepp 전역 경로를 `vx_max: 0.0`인
`FollowPathReverse`로 추종해 전진 명령을 구조적으로 차단한다. 명시적인
`*_ENTRY` 단계는 `DUBIN`과 `FollowPathForward`로 주차 헤딩을 정확히 맞추므로
최종 구간은 직선 후진 하나로 끝난다.
두 custom Behavior Tree 모두 `Spin`, `BackUp`, `DriveOnHeading` recovery를
제거했다. MPPI가 요구한 곡률이 실측 lookup 범위를 넘으면 어댑터는 실현 가능한
곡률로 포화시키며, 조향 명령이 목표에서 3 command 이내로 정렬될 때까지 traction을
열지 않는다.

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
사용한다. 공격적 실차 확인 모드의 Nav2 costmap은 물리 여유와 같은
`footprint_padding: 0.045 m`를 쓰고, inflation 반경은 패딩 후 내접반경보다 큰
0.23 m다. 실시간 LiDAR shield도 map 오차와 독립인 base frame
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

주행 중 `/slam/scan_filtered`는 Nav2 obstacle layer를 변형하지 않고 독립 모터
shield에만 입력된다. 일반 이동 구간의 Smac Hybrid-A* 경로는 정적 지도에서
시작할 때 한 번 계산해 유지한다. 양방향 주차 구간만 실제 도착 오차를 흡수하기
위해 같은 기록 목표까지의 정적 맵 경로를 2 Hz로 다시 연결하고, 전진 경계 경로는
고정한다. 일반 구간에서
남은 yaw만 맞추기 위한 불가능한 Dubins 루프를 반복 요구하던 문제는 막는다.
local/global inflation 반경은 0.23 m다. MPPI의 PathAlign/PathFollow 비용은
18.0/12.0으로 높여 기준경로를 우선한다. 추종이 실패하면 그때 정적 costmap을
지우고 구간 경로를 다시 계산하며, 실제 장애물이 예상 차체궤적에 들어오면
base-frame LiDAR shield가 최종 제동한다.
일반 이동은 우선 Dubins primitive와 전진 전용 controller를 함께 쓴다. 이 action이
실패한 경우에만 0.25초 뒤 같은 일반 경유점을 Reeds-Shepp로 재계획해, 지나친
목표로 후진 복귀할 수 있다. 목표에 도착하면 fallback 상태를 지우므로 다음 일반
단계는 다시 전진 전용이다. 후진 주차 단계의 Reeds-Shepp planner는
`reverse/change/non_straight penalty = 1.0/4.0/1.10`을 사용한다. 높은 change
penalty는 주차 arc 안에 작은 cusp가 반복되어 0.4초 방향전환 정지가 계속 생기는
현상을 억제한다. 주차가 끝난 전진 보정·출차·Start 복귀는 다시 전진 전용 트리로
전환한다.
Nav2의 실시간 obstacle layer는 비활성화해 scan 잡음이 기준경로를 바꾸지 않는다.
LiDAR shield가 실제 예상 충돌 동안 모터를 0으로 유지한다. 경로 추종 action 자체가
실패한 경우에는 costmap을 정리하고 0.25초 뒤 짧은 미션 재시도로 넘어간다. 이 재시도를
모두 사용해도 영구 중단하지 않고 `RECOVERING` 상태에서 2초간 정지한 뒤 같은
단계를 새 costmap으로 다시 시작한다. 우회 경로가 생기면 자동 출발하고, 장애물이
여전히 예상 차체 궤적을 막으면 출력 0을 유지한다. 180초 제한시간과 운전자 중단은
이 자동 복구보다 우선한다.

경기 제한 3분은 미션 start 승인 후 첫 목표를 보내는 순간부터 출발지 복귀 완료까지
연속으로 측정한다. 위치추정 일시정지와 경로 재시도 시간도 경기 시간에 포함한다.
일반 전진 경유점과 중간 후진 원호는 `transit_goal_checker`의 0.42 m 범위에
들어오거나 미션 관리자의 0.45 m 통과 gate를 만족하면 헤딩을 강제하지 않고
별도 hold 없이 다음 단계로 넘긴다. 이 범위를 지나친 뒤 좁은
자세를 다시 맞추려고 전후진을 반복하거나 Dubins 경로가 경기장 전체를 도는
현상을 막는다. 실측 A/B 주차 자세만
재현 맵의 정합 오차를 반영한 0.12 m/0.14 rad checker를 사용하며, 방향전환 0.4초
dwell과 조향 정렬 gate도
최종 모터 어댑터에 남긴다. 실제 주차 성공을 확인하는 A/B 목표에서는 각각 3초간
정지를 유지한다. 남은 시간이 30초가 되면 한글 경고를 출력하고 180초가 되면 진행
중 goal을 취소한 뒤 모터 권한을 닫는다.

조향 정렬 gate는 traction이 이미 0인 최초 출발, 전진/후진 dwell, 장애물·센서
안전정지의 해제 직전에만 적용한다. 같은 방향으로 command 4 주행을 시작한 뒤에는
MPPI가 조향 목표를 바꿔도 속도 명령을 0으로 끊지 않고, 실측 servo rate로 조향을
slew하면서 `+4` 또는 `-4`를 유지한다. 안전 gate의 0 출력은 계속 즉시 적용한다.
다만 일반 경유점 주행 중 Nav2가 보내는 0.40초 미만의 짧은 0은 직전 ±4로 이어
붙여 구동계 deadband를 다시 밟지 않게 한다. 후진 전용 일반 경유점에서는
지속되는 Nav2 0/제자리회전 요청도 마지막 조향을 유지한 -4 저속 후진으로 바꾼다.
A/B 주차점과 운전자 STOP,
정합 상실, LiDAR 충돌, sensor/VESC timeout, 전후진 dwell에는 이 필터를 적용하지
않는다.

B 평행주차는 후진 arc를 통과한 뒤 `B_FORWARD_CORRECTION`에서 차체를
`base=(2.850, 2.400, -0.600)`으로 정밀하게 맞춘다. 실제로 관측된 두 가지 얕은
reverse-cusp 도착 자세 모두에서 이 지점까지의 Dubins 경로는 순수 전진이고,
여기서 기록된 `B_PARK=(2.070, 3.026, -1.503)`까지의 Reeds-Shepp 경로는 추가
cusp가 없는 순수 후진이다. 이전처럼 상단 벽(y 약 4.0) 쪽으로 먼저 후진해 LiDAR
비상정지가 걸리는 경로를 제거하면서, 2026-08-23에 기록한 최종 주차 Pose 자체는
바꾸지 않았다.

B 출차는 기록된 B Pose에서 `base=(2.222,2.809,-pi/4)`로 연결할 때만 짧은
Reeds-Shepp 후진 연결을 허용한다. 이어지는 기준점
`(2.458,2.637,-35deg) -> (2.660,2.427,-60deg)`은 순수 전진 원호다. 기존 출차
점들이 차량을 y=2.19까지 과주행시켜 하단 경계의 LiDAR 비상정지를 일으키던
경로를 정적 지도 안쪽으로 옮겼다.

복귀 마지막 `START_RETURN_ALIGN_2`는 별도의 근접 Pose 대신 기록된 START Pose를
직접 사용한다. `ALIGN_1`의 실제 허용오차 도착 자세에서 약 5 cm 후진 연결 후 하나의
전진 우회전 원호로 START에 도달한다. 기존 ALIGN_2는 차량의 우측 후방에 놓여 전진
컨트롤러가 0 명령만 내면서 남은 30초를 소모했다.

## 미션 안전 상태기계

미션 관리자는 시작 pose를 반복 발행한 뒤 AMCL covariance, pose age, 순간 pose
jump가 6회 연속 정상일 때만 READY가 된다. 자체 제작 맵은 실제 경기장과 약
10~15 cm 정합 오차가 날 수 있으므로 AMCL `sigma_hit=0.20`, beam agreement 거리
0.45 m, 위치 covariance 0.0625, yaw covariance 0.12까지 허용한다. 큰 위치 점프
한도와 base-frame LiDAR 충돌검사는 완화하지 않는다. AMCL은 odometry 이동량이 정확히 0이면
새 laser update를 생략할 수 있으므로, 미션 활성 상태에서 pose 출력이 0.2초간
조용할 때 `/request_nomotion_update`를 최대 4 Hz로 비동기 호출한다. 따라서 출발 전
정차와 A/B의 3초 정지 확인 중에도 독립된 최신 scan matching 표본을 계속 얻는다.
주행 중 localization이 1.0초 이상 불량하면 현재 Nav2 goal을 취소하고 모터 권한을
닫으며, 회복 후 해당 단계부터 재시도한다. Nav2 성공 뒤에도 현재 AMCL pose가
목표에서 위치 0.16 m, yaw 0.18 rad 안에 있는지 별도로 확인한다. 목표 실패는
단계별 한도만 재시도하고 초과하면 ABORTED로 정지한다.

실행 중 미션 매니저는 2초마다 `[단계 진행 03/31]` 형식으로 단계명, 현재/목표
Pose, XY·yaw 오차/허용치, Nav2 goal 상태와 위치추정 상태를 기록한다. 모터
어댑터가 0을 출력할 때에는 `[모터 정지]` 형식으로 단계, 미션 상태, gate 원인,
Ackermann 제약이 적용된 입력 `/cmd_vel_nav`, 명령·scan age, LiDAR 유효점 수,
VESC 전압·고장을 기록한다. 범용 velocity smoother의 축별 감속 과정에서 발생할 수
있는 `v=0, w!=0` 제자리 회전 과도명령은 모터 경로에 들어오지 않는다.
경로 계획 실패와 최종 모터 출력 차단을 별도 원인으로 추적할 수 있다.

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
