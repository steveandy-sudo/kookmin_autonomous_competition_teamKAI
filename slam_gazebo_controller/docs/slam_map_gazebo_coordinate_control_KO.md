# SLAM 맵 Gazebo 좌표 제어 가이드

## 만들어진 구성

ROS bag으로 만든 `glass_balanced` 점유 지도를 Gazebo Harmonic 월드로 변환했다.

- 지도 크기: 450 × 317 px, 0.05 m/px
- 현재 월드 크기: 22.50 × 15.85 m
- 차량: 기존 simulation 브랜치의 `xycar_ackermann`
- 차량 제원: 차체 0.55 × 0.30 × 0.25 m, 휠베이스 0.32 m,
  윤거 0.265 m, 바퀴 반지름 0.06 m
- 기본 차량 위치: x=8.649, y=7.420, yaw=-163.43°
- 도로 전체 폭: simulation 생성기 기준 고정 1.632 m
- 안전벽: 한 바퀴 기준 경로 좌우 0.816 m에 배치
- 유리 구간: 현재 SLAM 맵 화면 기준 오른쪽과 오른쪽 위에 배치
- 라바콘 rule 구간: 맵 오른쪽 아래, 콘 사이 통로 폭 0.85 m

주요 파일:

- `xycar_ws/src/xycar_gazebo_bridge/worlds/slam_glass_balanced.sdf`
- `xycar_ws/src/xycar_gazebo_bridge/maps/slam_glass_balanced/world_metadata.json`
- `xycar_ws/src/xycar_gazebo_bridge/launch/slam_map_gazebo.launch.py`
- `xycar_ws/src/xycar_gazebo_bridge/xycar_gazebo_bridge/sim_control_gui.py`

## Windows에서 실행

PowerShell에서 simulation 작업트리로 이동해 다음을 실행한다.

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\start_slam_gazebo.ps1
```

이 명령은 WSL의 ROS 2 패키지를 빌드한 뒤 Gazebo 3D 화면과 좌표 제어
창을 함께 연다. 이미 빌드했다면 다음처럼 시간을 줄일 수 있다.

```powershell
.\scripts\start_slam_gazebo.ps1 -NoBuild
```

3D 화면 없이 센서와 제어만 시험하려면:

```powershell
.\scripts\start_slam_gazebo.ps1 -Headless -NoCoordinateGui -NoBuild
```

현재 WSL에는 Gazebo Harmonic 8.14.0과 Humble용 Harmonic 브리지가 설치되어
있다. 다른 PC에서 처음 구성할 때는 WSL 터미널에서 다음을 실행한다.

```bash
cd <simulation 작업트리>
./scripts/install_gazebo_harmonic_wsl.sh
```

## 좌표 제어 창 사용법

좌표계는 Gazebo/ROS 표준을 사용한다.

- x: 지도 오른쪽, 단위 m
- y: 지도 위쪽, 단위 m
- z: 높이, 단위 m
- yaw: 반시계 방향 회전, 단위 degree
- yaw 0°는 +x, 90°는 +y 방향

지도에서 위치를 클릭한 뒤 적용 대상을 선택한다.

1. `차량/엔티티`를 선택하면 클릭 위치가 차량 x, y에 들어간다.
2. yaw와 z를 입력하고 `차량/엔티티 이동`을 누른다.
3. `장애물`을 선택하면 클릭 위치가 장애물 x, y에 들어간다.
4. 사람, 상자, 의자, 테이블, 카메라 폴, 라바콘, 유리 패널을
   생성·이동·삭제한다.
5. 청록색 선은 유리 구간, 주황 점선은 라바콘 rule 구간이다.

`slam_map`도 x, y, yaw로 통째로 움직일 수 있다. `map scale`은 클릭 좌표
환산에 사용되며, 실제 월드 크기 변경은 아래의 월드 재생성이 필요하다.

## 터미널에서 직접 좌표 제어

WSL 터미널에서:

```bash
cd <simulation 작업트리>/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

차량 이동:

```bash
ros2 run xycar_gazebo_bridge xycar_sim_control \
  set-pose xycar_ackermann --x 8.2 --y 7.1 --z 0.05 --yaw-deg -150
```

사람 생성과 이동:

```bash
ros2 run xycar_gazebo_bridge xycar_sim_control \
  spawn person_01 person --x 6.0 --y 4.0 --yaw-deg 90

ros2 run xycar_gazebo_bridge xycar_sim_control \
  move person_01 --x 6.5 --y 4.2 --z 0.85 --yaw-deg 90
```

LiDAR가 보지 못하는 임시 유리 패널:

```bash
ros2 run xycar_gazebo_bridge xycar_sim_control \
  spawn glass_test glass_panel --x 2.5 --y 1.0 \
  --yaw-deg 90 --visual-only
```

삭제, 일시정지, 초기화:

```bash
ros2 run xycar_gazebo_bridge xycar_sim_control remove person_01
ros2 run xycar_gazebo_bridge xycar_sim_control pause
ros2 run xycar_gazebo_bridge xycar_sim_control resume
ros2 run xycar_gazebo_bridge xycar_sim_control reset-world
```

## 유리벽 모델

현재 추정한 기본값은 다음과 같다.

| 구간 | 중심 x | 중심 y | yaw | 길이 |
|---|---:|---:|---:|---:|
| 오른쪽 세로 | 14.07 | 3.35 | 90° | 6.70 m |
| 오른쪽 위 가로 | 10.40 | 8.25 | 0° | 5.60 m |

전체 높이 0.90 m의 투명 패널에는 LiDAR 충돌 형상을 넣지 않았고, 바퀴가
코스를 벗어나지 않도록 바닥에 높이 0.04 m의 턱만 넣었다. 따라서 실제
유리처럼 `/scan`에는 벽이 빠질 수 있지만 차량은 경계를 넘기 어렵다.

이 좌표와 길이는 사진·SLAM 맵으로 추정한 값이다. 현장에서는 각 유리
구간의 양 끝점 두 개만 측정하면 정확히 보정할 수 있다.

## 오른쪽 아래 라바콘 rule 구간

도로 폭은 저장소의 `scripts/generate_kookmin_track.py`에 정의된
`ROAD_W=1.632 m`를 사용한다. 이 값은 전체 포장 노면 폭이다. 같은 생성기의
흰선 안쪽 폭은 `0.800 m`, 실제 흰선 중심 간격은 `0.824 m`이므로 세 값을
같은 의미로 사용하면 안 된다. 라바콘 크기는 기존 월드의 기준 물체와 같은
`0.18 × 0.18 × 0.36 m`, 콘 사이 주행 통로는 기존 `hwj` LiDAR rule 설정의
`0.85 m`, 진행 방향 간격은 `0.48 m`다.

맵의 라바콘 구간은 다음 순서로 구성했다.

```text
직선 진입 → 꼬불꼬불한 라바콘 통로 → 짧은 직선 → 위쪽으로 회전하는 곡선
```

| 항목 | 맵 좌표 |
|---|---|
| rule 진입 게이트 | x=5.95, y=-0.16, yaw=0° |
| rule 이탈 게이트 | x=13.04, y=1.35, yaw=90° |
| SLAM의 단순 기준선 | (5.50, -0.16) → (11.70, -0.30) |

개별 라바콘 위치는 고정 SLAM 경로에 넣지 않는다. SLAM 위치추정 노드는
계속 실행하지만, 진입 후에는 SLAM 기준선의 조향 명령을 사용하지 않고
LiDAR 라바콘 중간 경로가 조향과 속도를 담당한다.

```text
SLAM_PATH_FOLLOW
  → 콘 경로 3프레임 연속 확인
CONE_RULE
  → 콘 종료 + 차선 4프레임 복구
SLAM_PATH_REJOIN
```

관련 설정은
`xycar_ws/src/xycar_hybrid_drive/config/cone_rule_zone_slam_map.yaml`에 있다.
콘은 이동될 수 있으므로 정적 SLAM 장애물로 저장하지 않는 것이 중요하다.

## 여러 맵 좌표를 잇는 실차 전역경로

`xycar_map_nav` 패키지는 waypoint YAML의 좌표를 순서대로 읽고, 저장된
occupancy map에서 각 좌표 쌍마다 A* 경로를 만든 뒤 하나의
`/map_nav/global_path`로 연결한다. 경로는 `map -> base_footprint` TF로
현재 위치를 읽어 Pure Pursuit으로 추종한다.

각 waypoint의 `controller_to_next`가 그 점에서 다음 점까지의 제어권이다.

| 값 | 동작 |
|---|---|
| `global_path` | SLAM 전역경로 추종 |
| `cone_rule` | 전역 조향을 멈추고 LiDAR 라바콘 rule 명령만 전달 |
| `dynamic_vehicle_rule` | 전역경로에 rule 기반 좌우 회피 오프셋을 적용 |

동적차량 rule은 저장소의 기존
`/yolo_obstacle/stable_counts=[front,left,right,behind]` 계약을 사용하고,
`NORMAL → AVOID_RIGHT → PASS_LEFT → RETURN_CENTER` 상태 전이를 미터
좌표계에서 실행한다. 기본 오프셋은 좌우 `0.28 m`, 제한 속도 명령은 `4`다.
전방 LiDAR `0.38 m` 이내는 모든 모드보다 우선하여 정지한다.

RViz의 `Publish Point`로 실제 맵 좌표를 진행 순서대로 저장할 수 있다.

```bash
ros2 launch xycar_map_nav real_waypoint_nav.launch.py \
  drive_enabled:=false \
  map_yaml:=/absolute/path/to/map.yaml \
  capture_output_yaml:=/absolute/path/to/my_waypoints.yaml
```

좌표를 찍은 후 저장된 YAML에서 라바콘과 동적차량 구간의
`controller_to_next`만 바꾼다. 라바콘용 `xycar_hybrid_drive`는
`drive_enabled:=false`로 실행해야 하며, 최종 `/xycar_motor` 발행자는
`xycar_map_nav` 하나만 남겨야 한다. 예시 좌표와 전체 절차는
`xycar_ws/src/xycar_map_nav/README.md`에 있다.

## 실제 크기에 맞춰 월드 재생성

SLAM 경로 길이는 약 43.55 m이고 도면 중심선은 49.2 m이므로 단순 비율은
약 1.1298이다. 다만 이 차이가 전부 지도 축척 오차라고 확정할 수 없으므로
기본 월드는 SLAM 좌표를 그대로 보존한 `scale=1.0`으로 만들었다.

도면 길이를 우선하는 비교 월드는 다음처럼 재생성한다.

```bash
cd <simulation 작업트리>/xycar_ws/src/xycar_gazebo_bridge
PYTHONPATH=. python3 -m xycar_gazebo_bridge.slam_world_generator \
  --map-yaml maps/slam_glass_balanced/slam_glass_balanced.yaml \
  --source-world ../../../worlds/kookmin_xycar_track_final.sdf \
  --output-world worlds/slam_glass_balanced.sdf \
  --output-texture maps/slam_glass_balanced/slam_glass_balanced_texture.png \
  --texture-uri file://maps/slam_glass_balanced/slam_glass_balanced_texture.png \
  --route-csv maps/slam_glass_balanced/one_lap_path.csv \
  --metadata maps/slam_glass_balanced/world_metadata.json \
  --wall-mode route --road-half-width 0.816 \
  --glass-layout upper_right --cone-layout bottom_right_rule \
  --map-scale 1.1298
```

재생성 후 다시 `colcon build --packages-up-to xycar_gazebo_bridge
--symlink-install`을 실행한다.

## 본선 전 다른 장소에서 시험하는 순서

1. 기본 월드를 실행하고 GUI로 차량 시작점과 방향을 여러 위치로 바꾼다.
2. 장애물과 `--visual-only` 유리 패널을 추가해 `/scan` 누락 상황을 만든다.
3. Gazebo의 `/scan`, `/imu`, odometry를 ROS 2로 받아 같은 SLAM 설정을 실행한다.
4. 한 바퀴 후 시작점 재방문 시 루프 폐쇄, 벽 이중상, 위치 점프를 기록한다.
5. 유리 구간 포함/제외 조건의 지도와 궤적 오차를 비교한다.
6. 실제 임시 장소에서는 바닥에 비닐 테이프나 낮은 폼보드로 같은 폭의
   폐곡선을 만들고, 느린 속도부터 같은 순서로 rosbag을 기록한다.
7. 본선 현장에서는 유리 양 끝점, 출발점, 직선 한 구간의 실제 길이만 먼저
   측정해 scale과 유리 좌표를 최종 보정한다.

자동 검증은 다음 명령으로 다시 실행할 수 있다.

```bash
./scripts/smoke_test_slam_gazebo.sh
```

이 검증은 SDF 문법, 월드 로딩, 차량 좌표 이동, 장애물 생성·이동·삭제
서비스를 실제 Gazebo 서버에서 확인한다.
