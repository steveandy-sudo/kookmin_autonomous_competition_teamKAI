# Xycar LiDAR parking navigation

국민대 Xycar 주차경기의 제공 PGM/YAML 지도를 사용해 다음 미션을 수행하는
ROS 2 Humble 패키지입니다.

```text
Start (1.790, 0.778, -3.038)
  -> A 후진주차 (-0.016, 4.105, 0.021)
  -> B 평행주차 (2.059, 3.186, -1.503)
  -> Start 복귀
```

주행 중 새 지도를 만드는 SLAM 패키지가 아니라, 제공된 정적 지도와 2D
LiDAR를 이용하는 `map_server + AMCL` localization 구조입니다. 전역/주차
일반 이동은 `SmacPlannerHybrid (DUBIN)`과 전진 전용
`MPPIController (Ackermann, vx_min=0)`를 사용합니다. 후진으로 진입해야 하는
주차 arc에만 `REEDS_SHEPP`와 양방향 MPPI를 선택합니다. A/B/Start Pose는
2026-08-23 실차 `map -> base_footprint` 측정값을 차량 중심 기준으로 변환해
사용하고, 조향 서보가 따라갈 수 있도록 곡률 변화점과 전진/후진 cusp를 포함한
31개 단계로 미션을 분할했습니다.

`/slam/odom`, `/slam/scan_filtered`는 기존 차량 인터페이스와 맞춘 topic 이름일
뿐이며, visual SLAM이나 `slam_toolbox` 노드는 실행하지 않습니다.

## 동작 로직과 원리

```text
LiDAR + VESC + IMU
        |
        v
scan_filter + vesc_imu_odom
        |
        v
정적 지도 + AMCL 위치추정
        |
        v
mission_manager (Start -> A -> B -> Start, 31단계)
        |
        v
Hybrid-A* 경로계획 + Ackermann MPPI 경로추종
        |
        v
cmd_vel_adapter (권한·LiDAR·방향전환 검사)
        |
        v
/xycar_motor (+4 / 0 / -4)
```

1. `scan_filter`가 차량 자체와 사용할 수 없는 LiDAR 구간을 제거하고,
   `vesc_imu_odom`이 VESC 이동량과 IMU yaw를 결합해 `/slam/odom`을 만듭니다.
2. AMCL은 새 지도를 만들지 않고 제공된 PGM 지도에서
   `map -> slam_odom` 변환과 `/amcl_pose`를 계산합니다. covariance, 위치·방향
   급변과 시간 유효성을 모두 검사합니다. 완전 정차 후 0.2초 동안 새 Pose가 없으면
   `/request_nomotion_update`를 최대 4 Hz로 호출해 최신 LiDAR로 위치를 다시
   계산합니다. 마지막 정상 Pose는 3초 주차 확인시간보다 긴 4초까지 인정하며,
   그 시간을 넘기거나 위치 품질이 나빠지면 모터 권한을 닫습니다.
3. `mission_manager`는 차량 중심으로 기록한 Start/A/B Pose를 전륜축
   `base_footprint` 기준으로 변환합니다. 첫 SPACE가 미션과 초기 위치를 reset하고
   시작 요청을 저장합니다. AMCL 정상 표본 6개가 연속으로 들어오면 `READY`가 되고
   `RUNNING -> HOLDING`을 반복하며 31단계를 순서대로 실행합니다.
4. 기본 단계는 Dubins Hybrid-A*와 `vx_min=0`인 MPPI로 계획·추종하므로 Start와
   주차 진입점 사이에서 후진하지 않습니다. 단, 일반 전진 경유점을 지나쳐 Dubins
   경로가 실패하면 같은 목표를 Reeds-Shepp와 양방향 MPPI로 한정 재계획해 필요한
   만큼 후진한 뒤 복귀합니다. 정밀 cusp와 주차 Pose에는 이 fallback을 적용하지
   않아 설계된 주차 방향을 유지합니다. `allow_reverse: true`로 표시한 A/B
   후진 진입 arc와 A/B 최종 주차 단계에서만 Reeds-Shepp와 양방향 MPPI를 씁니다.
   주차 경로의 높은 방향전환 비용은 짧은 전진/후진 cusp 반복을 억제합니다.
   B 전진 보정은 최종 주차칸의 남동쪽에서 자세를 맞추고, B 최종 주차는 cusp 없는
   후진 경로로 수행합니다. B 출차 첫 단계는 주차 완료 자세에서 수 cm 후진할 수
   있고, 기준 출차 원호에 합류한 뒤에는 전진합니다. 마지막 출발지 정렬에서는
   불가능한 우측 후방 목표를 만들지 않도록 수 cm 후진 연결 뒤 한 번의 전진 원호로
   기록된 START Pose에 바로 합류합니다.
5. `cmd_vel_adapter`만 최종 모터 명령 권한을 가집니다. 범용 smoother가 선속도와
   각속도를 따로 줄이며 제자리 회전 성분을 만들지 않도록 Ackermann MPPI 원본
   `/cmd_vel_nav`를 직접 사용합니다. 미션 승인, AMCL 정상, 최신 명령과 `/scan`,
   조향 정렬, 방향전환 정지시간, 예상 정지궤적의
   LiDAR 충돌검사를 전부 통과해야 `+4` 또는 `-4`를 냅니다. 하나라도 실패하면
   즉시 `0`을 내고 `/parking_cmd_vel_adapter/status`에 원인을 기록합니다. 최초
   출발·전후진 전환·안전정지 해제 전에는 조향을 먼저 맞추지만, 같은 방향 주행이
   시작된 뒤에는 MPPI 조향 변화 때문에 속도를 `4/0`으로 반복하지 않고 `±4`를
   유지한 채 조향만 설정된 물리 속도로 따라갑니다.
6. 두 번째 SPACE, `Q`, `ESC` 또는 abort 서비스는 Nav2 목표를 취소하고
   `/parking/drive_authorized`를 닫습니다. 키보드 터미널에는 미션 단계와
   `stale_pose`, LiDAR 충돌, 조향 대기 같은 정지 이유가 한글로 표시됩니다.

미션 매니저는 주행 중 2초마다 `[단계 진행 03/31]` 형식으로 현재 단계, 주행
모드, 현재/목표 Pose, XY·yaw 오차와 완료 허용치, Nav2 목표 활성 여부를
기록합니다. 모터 출력이 0이면 어댑터가 `[모터 정지]` 형식으로 현재 단계와 미션
상태, 정지 이유, 원래의 `/cmd_vel`, 명령·scan 나이, 유효 LiDAR 점 수, VESC
전압·고장을 기록합니다. 따라서 미션 경로 실패와 최종 모터 안전 gate를 같은
시각의 로그로 구분할 수 있습니다.

## 안전 기본값

- `drive_enabled:=false`가 기본값이며 `/xycar_motor`로 실차 명령을 보내지
  않습니다.
- AMCL covariance, 갱신시각, pose jump를 모두 통과해야 모터 권한이 열립니다.
- 다른 장소에서 재현한 맵과 실제 구조물의 약 10~15 cm 정합 오차를 고려해 AMCL
  likelihood 폭과 beam-skip 허용폭을 넓혔습니다. Nav2의 계획용 footprint는
  전륜축 주변 0.34 x 0.20 m로 축소해 맵 벽과 한두 셀 겹쳐도 경로 탐색을 시작합니다.
  모터 어댑터의 실차 footprint는 0.57 x 0.36 m 그대로입니다.
- `/cmd_vel` 또는 `/scan`이 stale이면 즉시 정지합니다.
- 곡선 주행 중 차량 전체 polygon의 예상 정지 궤적과 LiDAR 점을 독립적으로
  충돌 검사합니다.
- 기본 31단계 고정 미션의 일반 이동 구간은 시작 때 전역 경로를 한 번 계산해
  기준경로로 고정합니다. 주행 중에는
  정적 지도와 AMCL을 기준으로 MPPI가 이 경로를 강하게 추종합니다. LiDAR obstacle
  layer는 비활성화하며, LiDAR는 모터 직전 예상 차체궤적의 비상정지에만 사용합니다.
  양방향 정렬과 주차 구간도 시작 자세에서 정적 맵 경로를 한 번 계산해 고정합니다.
  주차 중 2 Hz 재계산으로 cusp가 움직이며 전진/후진 명령이 뒤집히는 것을 막습니다.
  경로 추종 자체가 실패한 경우에만 costmap을
  지우고 해당 구간을 다시 계산합니다. 일반 이동 단계는 먼저 전진 경로만 선택하고,
  그 Nav2 action이 실패한 경우에만 현재 경유점에 전진·후진 후보를 평가합니다.
  명시된 후진 주차 단계는 처음부터 전진·후진 후보를 평가합니다.
  RViz에서 새로 캡처한 웨이포인트 미션은 일반점도 모두 `allow_reverse: true`로
  생성되어 처음부터 전진·후진 후보를 함께 평가합니다.
  정적 지도 costmap은 0 m padding과 0.10 m inflation을 사용합니다.
  실제 장애물이 가까운 전방 예상 궤적을 막으면 독립 LiDAR shield가 즉시 제동합니다.
  전진 중 발생한 충돌 정지는 0.40초 정지·조향 중앙 정렬 후 약 0.62초(공칭 약
  0.20 m)만 직선 후진하고, Nav2 목표를 취소해 같은 단계를 새로 계산합니다.
  후방 LiDAR 시야가 없으므로 이 복구는 위치추정·scan·VESC·주행권한이 모두
  정상일 때만 시간 제한으로 실행하며, 후진 중 장애물 정지에는 중첩 적용하지
  않습니다. Nav2의 짧은 재시도가 모두 실패해도 영구
  중단하지 않고 0.25초 뒤 같은 단계부터
  costmap을 새로 계산해 재시도합니다. 장애물이 계속 앞을 막으면 이 짧은 후진과
  재계산을 냉각시간을 두고 반복하며, 운전자 중단 때만 끝냅니다.
- 일반 전진 goal이 `ABORTED`되면 0.25초 뒤 해당 goal에 한해서 후진 허용
  fallback을 실행합니다. 이후 같은 단계를 재시도하는 동안 fallback을 유지하고,
  도착하면 다음 단계는 다시 전진 전용으로 돌아갑니다.
- 자동 시작 시 Nav2 lifecycle 전체 활성화 전의 목표 거부는 주행 실패 횟수로
  세지 않으며, 0.25초마다 확인하되 10초 안에 준비되지 않으면 안전 중단합니다.
- 시작 명령부터 경과시간과 180초 기준 남은 시간은 계속 기록하지만
  `mission_time_limit_enforced: false`이므로 3분을 넘어도 주행을 계속합니다.
  일반 웨이포인트는 목표 반경 35 cm 또는 진행 방향 통과선(최대 60 cm 이내,
  횡오차 45 cm 이내)을 넘으면 다음 단계로 진행하고 헤딩은 요구하지 않습니다.
  A/B 주차 Pose만 16 cm/0.18 rad로 정확하게 확인합니다.
- 전진/후진이 바뀔 때 0.4초 정지하고 조향을 미리 정렬합니다.
- 실차가 명령 4 미만에서 움직이지 않는 특성을 반영해 모든 0이 아닌 주행 요청을
  전진 `+4` 또는 후진 `-4`로 계속 출력합니다. 센서·권한·충돌 조건이 깨지거나
  방향을 바꾸는 안전 정지 구간에서만 `0`을 출력합니다.
- 실측 조향 lookup의 약한 쪽을 기준으로 최소 회전반경을 0.67 m로 제한합니다.
- Ackermann 차량이 수행할 수 없는 rotate-in-place 명령은 거부합니다.
- Nav2 복구 Behavior Tree에서도 spin 동작을 제거했습니다.

## 지도와 좌표

`maps/parking_map_original.yaml`은 대회 원본 값입니다. 원본의
`free_thresh: 0.25`는 회색 205 픽셀을 FREE로 해석하므로, 실행용
`maps/parking_map.yaml`은 `free_thresh: 0.196`을 사용해 회색을 UNKNOWN으로
보존합니다. `resolution`과 `origin`은 원본 그대로입니다.

도면의 십자는 차량 기하 중심이지만 실차의 `base_footprint`는 전륜 중심입니다.
`config/parking_mission.yaml`의 `base_from_reference_x_m: 0.16`이 모든 차량 중심
Pose를 TF 기준 Pose로 변환합니다. 이 값은 국민대 Gazebo 생성기의 전·후륜
위치 `x=+0.16/-0.16 m`와 일치합니다.

차량 형상도 같은 생성기의 실차 측정값을 사용합니다. Wheelbase `0.32 m`, 중앙
차체 `0.50 x 0.20 m`, 직진 타이어 외폭 `0.28 m`, 휠 반경 `0.06 m`, 전륜축
기준 LiDAR `x=+0.065 m`입니다. 차체·LiDAR·최대 조향 타이어를 모두 감싸도록
planning footprint는 전륜축 기준 `x=[-0.46,+0.11]`, `y=+-0.18 m`로 바깥쪽
반올림했습니다. 대회에 투입되는 차체가 해당 Gazebo 모델과 같은 조립 상태인지는
현장에서 마지막으로 확인해야 합니다.

## 설치와 빌드

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  python3-opencv python3-pil python3-yaml

cd /home/xytron/parking_ws/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  xycar_msgs xycar_vesc_driver xycar_parking_nav
source install/setup.bash
```

## 맵-LiDAR 자동 정합과 RViz 웨이포인트 주행

`codex/slam-gazebo-controller` 브랜치의 `route_scan_matching.py`와
`route_scan_localizer.py`를 가져와 현재 AMCL 스택에 연결했습니다. 점유맵의
벽까지 거리장을 만든 뒤, 기록 경로를 0.40 m 간격으로 훑으며 좌우 0.30 m와
yaw ±20도를 탐색합니다. 벽 0.15 m 이내 LiDAR 점 비율이 45% 이상이고 평균
벽 거리가 0.22 m 이하이며, 2 m 이상 떨어진 두 후보의 점수 차가 0.06 이상인
경우만 후보로 인정합니다. 같은 후보가 4회 연속 확인되면 `/initialpose`를
AMCL에 보내고, 2초 뒤 `map -> base_footprint` 오차가 0.75 m/20도 이내일 때
`/parking/route_localization/ready=true`가 됩니다. 반복되는 벽에서 어느 위치인지
구별할 수 없으면 `AMBIGUOUS` 상태로 남고 모터 주행을 시작하지 않습니다.

주차 차량은 항상 `WP_00`에 배치하므로 초기 정합 후보는 전역경로의 첫 2 m로
제한합니다. 마지막 웨이포인트가 시작점 근처로 돌아오는 열린 경로에서 시작과
끝을 서로 다른 후보로 계산해 `AMBIGUOUS`가 되는 문제를 막기 위한 설정입니다.
초기 pose를 받은 뒤에는 `route_scan_localizer`가 아니라 AMCL이
`/slam/scan_filtered`의 모든 새 LiDAR 스캔을 정적 PGM 지도와 계속 대조하며
`map -> slam_odom`을 갱신합니다. 원본 브랜치의 `slam_toolbox localization`을
그대로 사용하려면 PGM/YAML 외에 매핑 당시 저장한 `.posegraph`와 `.data`가
반드시 필요합니다.

### 1. 웨이포인트 찍기

모터 출력이 없는 캡처 launch를 실행합니다.

```bash
cd /home/xytron/parking_ws/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch xycar_parking_nav parking_waypoint_capture.launch.py \
  waypoints_yaml:=$HOME/parking_waypoints.yaml \
  mission_output_yaml:=$HOME/parking_waypoint_mission.yaml
```

웨이포인트 번호는 `0`부터 RViz에 크게 표시됩니다. 흰색/초록 화살표는
전진 헤딩이고, 후진으로 지정한 번호와 구간은 주황색 `R` 및 반대 방향 화살표로
표시됩니다. 예를 들어 4번에서 5번, 8번에서 9번으로 후진하려면 기존 점을
다시 찍지 않고 캡처 launch를 다음처럼 다시 실행하면 YAML과 주행 미션이
즉시 재생성됩니다.

```bash
ros2 launch xycar_parking_nav parking_waypoint_capture.launch.py \
  waypoints_yaml:=$HOME/parking_waypoints.yaml \
  mission_output_yaml:=$HOME/parking_waypoint_mission.yaml \
  reverse_waypoint_ranges:="4-5,8-9"
```

`4-6`은 `4 -> 5 -> 6` 두 구간을 모두 후진한다는 뜻입니다. 범위 시작점부터
다음 구간의 반대 방향 헤딩을 경로 생성에 사용하고, 이후 범위 안의 목표는
`reverse_only: true`로 생성됩니다. 일반 웨이포인트는 위치 0.42 m 안 또는
통과 반경 0.45 m 안이면 도착으로 인정하며 헤딩 오차는 강제하지 않습니다.
A/B 주차점만 위치 0.12 m·헤딩 0.14 rad의 정밀 Nav2 조건과 미션 관리자의
최종 정합 검증을 함께 적용합니다. 번호를 바꾸려면 캡처 launch만
새 범위로 다시 실행한 뒤 종료하면 되며 웨이포인트 좌표는 유지됩니다.
캡처가 이미 실행 중이라면 재시작 없이 다음 명령으로 즉시 바꿀 수도 있습니다.

```bash
ros2 param set /parking_waypoint_route reverse_waypoint_ranges "4-5,8-9"
```

RViz 상단 `Publish Point`를 선택하고 다음 순서로 클릭합니다.

1. 차량을 실제로 놓을 시작 위치(실차는 두 번째 점 방향을 바라보게 배치)
2. 실제 주행 순서의 일반 경유점
3. A 주차 시 파란색 `A_PARK` 박스 안 한 점
4. A 출차 뒤 일반 경유점
5. B 주차 시 주황색 `B_PARK` 박스 안 한 점
6. 이후 복귀 경유점과 마지막 정지 위치

클릭할 때마다 점유맵에서 A* 경로를 다시 만들고 초록색
`/parking/waypoint_route`로 표시합니다. 첫 점은 초기 위치이며 실제 주행 목표는
두 번째 점부터 시작합니다. 곡선은 직선보다 촘촘하게 찍어야 합니다. 되돌리기와
전체 초기화는 다음 서비스입니다.
기존 실측 위치는 `/parking/reference_locations`에 항상 표시됩니다. 자홍색
`PREVIOUS_START`는 `(1.790, 0.778, -3.038)`, 파란색 `A_PARK`는
`(-0.016, 4.105, 0.021)`, 주황색 `B_PARK`는
`(2.059, 3.186, -1.503)`이며, 각 위치는 방향 화살표와 0.80 x 0.50 m 박스로
보입니다. A/B 중심 45 cm 안에서 클릭한 점은 생성 시 해당 실측 중심과 헤딩으로
자동 스냅되어 `parking_goal/precise_goal/allow_reverse`가 모두 켜집니다. 그 외
점은 헤딩을 정확히 맞추지 않아도 되는 일반 통과점입니다. 다만 이름이
`*_FORWARD_CUSP`는 전진 전용 정밀점, `*_REVERSE_ALIGN`은 후진 전용 정밀점,
`*_ENTRY`는 주차칸 헤딩을 정확히 맞추는 전진 전용 진입점으로 생성됩니다.
그 다음 A/B 최종 구간은
`FollowPathReverse(vx_max=0)`로 후진만 수행합니다. 이름을 쓰지 않은 기존 경로는
바로 전 점의 헤딩만 주차칸 바깥쪽으로 유도하는 완화된 동작을 유지합니다.
단, B 평행주차 직전 점은 긴 후진 진입의 오버슈트를 막기 위해 자동으로 정밀
전진 정렬점이 됩니다. 정렬 완료 후 별도 대기 없이 B_PARK의 후진 전용 경로로
넘어가며, 주차 허용오차에 들어가기 전 Nav2의 0속도·제자리회전 요청은 마지막
조향을 유지한 `-4` 후진 원호로 실행합니다.
열린 경로의 마지막 클릭점도 일반 통과 허용범위를 사용한 뒤 정지합니다.
A/B 주차점은 Nav2 도착 응답만으로 완료하지 않고, 그 순간 경로 LiDAR 정합과
AMCL의 최신성·공분산 조건이 모두 정상일 때만 도착으로 확정합니다. 정지 확인 중
정합이 풀려도 다음 단계로 넘어가지 않으며, 정합이 회복되면 같은 목표를 다시
전송해 재검증합니다.

```bash
ros2 service call /parking_waypoint_route/undo_waypoint std_srvs/srv/Trigger '{}'
ros2 service call /parking_waypoint_route/clear_waypoints std_srvs/srv/Trigger '{}'
```

순환 경로라면 캡처와 주행 명령 양쪽에 `closed_route:=true`를 동일하게
추가합니다. 점을 모두 찍으면 launch를 `Ctrl+C`로 종료합니다.

### 2. 정합 확인과 웨이포인트 주행

마지막 실차 조정본은 `config/parking_waypoints.yaml`과
`config/parking_waypoint_mission.yaml`에 함께 저장되어 있으므로 별도 경로를
지정하지 않으면 이 스냅샷을 사용합니다. `$HOME`에서 새로 캡처한 경로를 시험할
때만 아래처럼 `waypoints_yaml`과 `mission_config`를 명시합니다.

먼저 실차 출력을 끈 shadow 모드로 확인합니다.

```bash
ros2 launch xycar_parking_nav parking_waypoint_drive.launch.py \
  drive_enabled:=false \
  waypoints_yaml:=$HOME/parking_waypoints.yaml \
  mission_config:=$HOME/parking_waypoint_mission.yaml
```

```bash
ros2 topic echo /parking/route_localization/status
ros2 topic echo /parking/route_localization/ready
ros2 topic echo /parking/mission_state
```

상태가 `MATCHING 1/4 ... -> SETTLING ... -> READY` 순서가 되고 RViz의 자홍색
`Route LiDAR Best Pose`, AMCL Pose, 실제 LiDAR 벽이 같은 위치인지 확인합니다.
`AMBIGUOUS`면 차량을 고유한 모서리가 보이는 시작점으로 옮기거나 경로의 첫 구간을
더 특징적인 위치에 찍습니다.

정합을 확인한 뒤 launch를 종료하고 실제 모터를 허용합니다.

```bash
ros2 launch xycar_parking_nav parking_waypoint_drive.launch.py \
  drive_enabled:=true \
  waypoints_yaml:=$HOME/parking_waypoints.yaml \
  mission_config:=$HOME/parking_waypoint_mission.yaml
```

다른 터미널에서 키보드 노드를 실행하고, 정합 상태와 미션 상태가 모두 `READY`가
된 뒤 SPACE를 누릅니다.

```bash
ros2 run xycar_parking_nav parking_keyboard_control
```

캡처한 일반 웨이포인트도 처음부터 Reeds-Shepp 전후진 경로를 허용하므로 Nav2가
현재 자세에서 더 짧고 실행 가능한 방향을 선택합니다. A/B 박스 안에서 찍은 점은
별도 YAML 수정 없이 정밀 후진주차 목표가 됩니다. LiDAR obstacle layer는
비활성이고, 앞쪽 라이다는 모터 직전 예상 차체 궤적의 비상정지에만 사용합니다.

현재 A 진입은 기존 `WP_03=(0.996, 4.685)` 한 점이나 양방향 정렬 제어를 쓰지
않습니다. 실차 로그에서 얻은 cusp `A_FORWARD_CUSP=(1.119, 4.391, 0.912)`까지
전진하고, `A_REVERSE_ALIGN=(0.534, 4.116, 0.022)`까지 후진한 뒤
`A_ENTRY=(1.034, 4.127, 0.021)`까지 전진하고 `A_PARK`까지 직선 후진합니다.
각 단계는 `전진 -> 후진 -> 전진 -> 후진`으로 고정되어 한 단계 안에서 Nav2가
기어를 반복해서 뒤집을 수 없습니다. 이 이름과 yaw는 웨이포인트를 다시 생성해도
같은 방향 규칙을 보존합니다.

## 빠른 실행 명령

실행 중인 이전 주차 launch가 있으면 먼저 `Ctrl+C`로 종료합니다. LiDAR, IMU,
VESC를 주차 launch가 모두 시작하는 표준 실차 명령은 다음과 같습니다. 아래 명령은
실제 모터 출력을 허용하므로 첫 SPACE 뒤 AMCL이 안정되면 차량이 움직입니다.

터미널 1:

```bash
cd /home/xytron/parking_ws/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch xycar_parking_nav parking_real.launch.py \
  drive_enabled:=true \
  autostart_mission:=false
```

LiDAR를 다른 터미널에서 이미 lifecycle `active` 상태로 실행 중인 경우에만
중복 포트를 피하도록 `start_lidar:=false`를 추가합니다.

터미널 2:

```bash
cd /home/xytron/parking_ws/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run xycar_parking_nav parking_keyboard_control
```

- 첫 SPACE: 미션/초기 위치 reset 후 AMCL 준비 시 자동 출발
- 두 번째 SPACE: Nav2 목표 취소, 모터 권한 해제 및 즉시 정지
- 다시 SPACE: 처음부터 재시작
- `Q` 또는 `ESC`: 정지 후 키보드 프로그램 종료

키보드 프로그램을 쓰지 않을 때의 서비스 명령은 다음과 같습니다.

```bash
ros2 service call /parking_mission_manager/reset std_srvs/srv/Trigger '{}'
ros2 service call /parking_mission_manager/start std_srvs/srv/Trigger '{}'
ros2 service call /parking_mission_manager/abort std_srvs/srv/Trigger '{}'
```

실차 출력 없이 전체 로직만 확인하려면 터미널 1의 launch에서
`drive_enabled:=false`를 사용합니다. 상태와 정지 원인은 다음 명령으로 봅니다.

```bash
ros2 topic echo /parking/mission_state
ros2 topic echo /parking_cmd_vel_adapter/status
ros2 topic hz /amcl_pose
```

현재 `/slam/scan_filtered`의 필터 조건은 유한한 거리값 중 `0.20 <= range <=
6.00 m`인 점만 통과시키고 나머지는 `NaN`으로 바꾸는 것입니다. 각도 제외구간은
없습니다. 모터 어댑터의 독립 충돌검사는 이 결과를 `base_footprint`로 변환한 뒤
차량 자체 반사 구간 `x=[-0.46,+0.11] m`, `|y| <= 0.18 m`를 제거합니다. scan
나이가 0.35초 이하이고 자체 반사 제거 후 점이 60개 이상이어야 주행을 허용하며,
차체 footprint에 0.045 m 여유를 둔 0.14~0.80 m 예상 정지궤적을 검사합니다.
RViz에는 `/parking/mission_visualization`의 녹색 전체 기준경로, 노란색 현재 목표,
파란색 `A_PARK` 박스, 주황색 `B_PARK` 박스가 표시됩니다. 모든 31개 목표 Pose는
기존 `/parking/mission_goals`의 청록색 화살표로 함께 표시됩니다.

## 오프라인 검증

지도 및 목표 차체 충돌검사:

```bash
ros2 run xycar_parking_nav validate_map
```

Nav2와 독립된 Hybrid-A* 전체 미션 검증:

```bash
PYTHONPATH=src/xycar_parking_nav \
python3 src/xycar_parking_nav/scripts/plan_mission_offline.py \
  --package src/xycar_parking_nav
```

단위 테스트:

```bash
python3 -m pytest -q src/xycar_parking_nav/test
```

실차와 Gazebo 없이 제공 PGM을 직접 레이캐스팅해 AMCL/Nav2/미션/안전 어댑터
전체를 시험할 수도 있습니다. 시뮬레이터는 `/cmd_vel`을 직접 사용하지 않고 실제
어댑터가 만든 `/parking/xycar_motor_shadow`를 실측 조향·속도 lookup으로 다시
해석하므로 조향 slew, 방향전환 dwell, LiDAR stop shield까지 시험합니다. 이
launch는 `/xycar_motor`를 발행하지 않습니다.

```bash
ros2 launch xycar_parking_nav parking_sim.launch.py \
  autostart_mission:=true enable_rviz:=true
```

진행 상태와 `elapsed`/`remaining` 시간은 `/parking/mission_state`, 정확한 모의 궤적은
`/parking/sim_ground_truth_path`에서 확인합니다.

2026-08-24 최종 기준경로와 독립 LiDAR 비상정지를 함께 켠 통합 실행에서
31/31단계를 159.0초에 완료했습니다. 180초 제한 대비 21.0초가 남았으며 A/B 주차,
B 출차, START 복귀까지 같은 실행에서 확인했습니다.

## 실차 실행 순서

먼저 모터 출력이 없는 shadow 모드로 실행합니다.

```bash
ros2 launch xycar_parking_nav parking_real.launch.py \
  drive_enabled:=false \
  autostart_mission:=false
```

이 통합 launch는 기본적으로 `parking_preflight`를 함께 실행합니다. 실행 직후
터미널에서 `/dev/ttyLIDAR`, `/dev/ttyIMU`, `/dev/ttyMOTOR` 장치와 다음 입력을
자동 확인합니다.

```text
/scan                    LiDAR 원본
/imu                     IMU
/vehicle/vesc_state      VESC 텔레메트리
/slam/scan_filtered      주차용 LiDAR 필터
/slam/odom               VESC+IMU 오도메트리
/amcl_pose               지도 위치추정
map <- slam_odom <- base_footprint <- laser_frame TF
```

정상이면 `========== 모든 주차 센서·TF 정상 ==========`과 미션 시작 명령을
출력합니다. 20초 안에 준비되지 않으면 `[실패]` 항목별 원인과 조치 방법을
출력하며, 이때는 미션을 시작하지 않습니다. 점검 시간은
`preflight_timeout_sec:=30.0`처럼 바꿀 수 있고, 외부 점검기를 사용하는 경우에만
`enable_preflight:=false`로 끌 수 있습니다.

다음 항목을 확인합니다.

```bash
ros2 topic hz /scan
ros2 topic hz /slam/scan_filtered
ros2 topic hz /slam/odom
ros2 topic echo /amcl_pose --once
ros2 topic echo /parking/mission_state
ros2 run tf2_ros tf2_echo map slam_odom
ros2 run tf2_ros tf2_echo slam_odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_frame
```

RViz에서 지도 위 LiDAR 점이 벽과 일치하고 mission state가 `READY`가 되면
미션을 시작합니다. Shadow 모드에서는 계산된 명령이
`/parking/xycar_motor_shadow`에만 나옵니다.

별도 터미널에서 SPACE 시작/정지 조작기를 실행할 수 있습니다. 첫 SPACE는
위치추정을 reset한 뒤 미션 시작 요청을 저장하고, 다음 SPACE는 즉시 abort하여
모터 주행 권한을 닫습니다. 같은 터미널에 현재 단계와 위치추정·LiDAR·Nav2 등
정지 이유가 한글로 표시됩니다. `Q` 또는 `ESC`는 안전 정지 후 종료합니다.

```bash
ros2 run xycar_parking_nav parking_keyboard_control
```

```bash
ros2 service call /parking_mission_manager/start std_srvs/srv/Trigger '{}'
```

중지 및 초기화:

```bash
ros2 service call /parking_mission_manager/abort std_srvs/srv/Trigger '{}'
ros2 service call /parking_mission_manager/reset std_srvs/srv/Trigger '{}'
```

바퀴를 띄운 시험, 최저속 전·후진 시험, 장애물 회피·최종 정지 시험을 모두 통과한 후에만
실차 출력을 켭니다.

```bash
ros2 launch xycar_parking_nav parking_real.launch.py \
  drive_enabled:=true \
  autostart_mission:=false
```

`drive_enabled:=true`는 VESC 드라이버와 최종 어댑터를 동시에 활성화하지만,
미션 start 서비스와 localization gate가 열리기 전에는 계속 정지 명령만 냅니다.

센서와 VESC 드라이버를 다른 launch에서 이미 실행 중이면 중복 포트/TF를 피하도록
하드웨어 드라이버가 포함되지 않은 launch를 사용합니다.

```bash
ros2 launch xycar_parking_nav parking_navigation.launch.py \
  start_odometry:=true start_scan_filter:=true \
  drive_enabled:=false
```

## 주요 설정

- `config/nav2_parking.yaml`: AMCL, Hybrid-A*, Ackermann MPPI, costmap과 footprint
- `config/parking_mission.yaml`: 공식 Pose와 31단계 곡률/cusp/정지 구간
- `config/mission_manager.yaml`: localization gate, 재시도, 최종 Pose 허용오차
- `config/cmd_vel_adapter.yaml`: 실측 조향/속도 map과 LiDAR 독립 안전영역
- `config/vesc_imu_odom.yaml`: 타코미터 거리 및 IMU gyro yaw 보정
- `behavior_trees/ackermann_bidirectional_precise_navigate_to_pose.xml`: 경로를 한 번 계산하는 정밀 양방향 정렬
- `behavior_trees/ackermann_reverse_parking_navigate_to_pose.xml`: A/B 최종 구간의 후진 전용 추종
- `behavior_trees/ackermann_reverse_transit_navigate_to_pose.xml`: 전진 경로 실패 시에만 쓰는 후진 허용 복구

실차 주차 launch에서는 주차 어댑터가 주행 중 `-4` 또는 `+4`를 연속 출력하고,
정지 조건에서만 `0`을 출력합니다. 모터 명령을 0에서 4 사이로 천천히 올리면 차가
움직이지 않으므로 어댑터의 명령값 ramp는 사용하지 않습니다. 대신 VESC 드라이버의
`acceleration_slew_enabled`를 켜서 목표 명령은 4로 유지하면서 실제 가감속만
부드럽게 제한합니다. 일반 경유점 주행 중 들어오는 짧은 Nav2 `0` 명령은 최대
0.40초 동안 직전 `+4/-4`로 이어 붙여 출발 토크가 끊기는 현상을 막습니다. 명령 4의
실측 환산속도에서는 최대 약 0.13 m이며, A/B 주차점·최종 정밀점에는 적용하지
않습니다. SPACE 정지, 정합 상실, LiDAR 충돌, sensor/VESC timeout과 전후진 전환은
이 유지시간을 무시하고 즉시 `0`을 출력합니다.

설계 근거와 실차 튜닝 항목은 `docs/DESIGN_KO.md`에 정리되어 있습니다.
