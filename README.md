# 국민대학교 자율주행 경진대회 Xycar 통합 주행

ROS 2 Humble 기반 실차 통합 주행 코드다. `main`은 `/home/xytron/xycar_ws`에서 사용하는 대회용 최종본이며, 2026-08-23 기준 차선 주행과 전체 미션을 포함한 3바퀴 완주를 확인했다. 첫 출발과 2·3바퀴 지름길 선택을 관리하는 대회용 랩 정책도 포함한다.

- 실차 워크스페이스: `/home/xytron/xycar_ws`
- ROS 도메인: `7`
- 차선 인지: direct Xbin, 최대 `20 Hz`
- 제어 우선순위: `TRAFFIC > SHORTCUT > CONE > YOLO+LiDAR AVOIDANCE > RULE`
- 최종 모터 출력: `space_drive_gate`가 `/xycar_motor`를 단독 발행

## 현재 구조

```text
광각 카메라
  ├─ Xbin 노란 중앙선 모델 ──> direct 미터 경로 ──> RULE 후보
  ├─ YOLO 객체 모델 ─────────> 신호등·차량·라바콘 판단
  └─ LR-ASPP 지름길 모델 ────> W1 진입 후보

LiDAR
  ├─ 라바콘 좌우 경계 ───────> CONE 후보
  └─ 차량 거리·빈 공간 ─────> AVOIDANCE 후보

후보 명령 ──> sequential_hybrid_driver
              ├─ RaceLapPolicy: 3바퀴 신호·지름길 정책
              └─ 우선순위·미션 상태 선택
                       │
                       v
              space_drive_gate ──> VESC
```

주요 패키지는 다음과 같다.

| 패키지 | 역할 |
| --- | --- |
| `lane_seg_control` | direct Xbin 차선 인지와 미터 경로 생성 |
| `xycar_rule_drive` | Pure Pursuit + Stanley 차선 추종 |
| `study/my_rule` | YOLO 객체 인지와 LiDAR 라바콘 경로 생성 |
| `shortcut_entry_review` | W1 기반 지름길 진입 인지 |
| `xycar_map_nav` | 신호등·지름길·라바콘·회피 우선순위와 최종 선택 |
| `xycar_vesc_driver` | 모터 출력, 전압 및 VESC fault 감시 |

## 차선 주행

기본 인지 백엔드는 `direct_xbin`이다.

1. `/wide_camera_mjpeg/image_raw/compressed`를 왜곡 보정한다.
2. `kookmin_far_centerline_xbin_512x288.pt`가 노란 중앙선을 추론한다.
3. 전체 BEV 영상을 제어 입력으로 사용하지 않고 중앙선 점을 직접 미터 좌표로 변환한다.
4. `/perception/xbin_direct_centerline`의 `x=전방`, `y=왼쪽` 경로를 RULE 제어기에 전달한다.
5. RULE 제어기는 경로를 직선·곡선·DEGRADED로 분류하고 Pure Pursuit과 Stanley를 혼합해 조향한다.

현재 direct 경로는 전방 최대 `2.5 m`, 좌우 약 `+/-0.7 m`를 사용한다. 경로가 `0.50초` 이상 갱신되지 않으면 stale 경로로 처리한다.

기준 주행 속도는 직선 `25`, 일반 곡선과 S자 내부 `12`, 짧거나 기억된 DEGRADED 경로 `15`다. S자 진입 가드는 지름길 또는 동적 장애물 구간 이후 곡선이 확인되거나 가드 이동거리가 `8.0 m`에 도달하면 속도를 `11`로 제한한다. `red_car` 회피 직후 좌회전은 별도 상한 `13`을 즉시 사용한다.

## 미션 동작

### 신호등과 지름길

- 최초 출발 전에는 `red_4`와 `yellow_4`가 정지 조건이고, `green_4`가 2프레임 확인되면 출발한다.
- 첫 출발 이후에는 경기 중 다시 검출되는 빨간불과 노란불로 차량을 세우지 않는다.
- `green_4`와 `left_4`가 흔들려 검출되더라도, 신호등이 사라지기 직전의 마지막 유효 YOLO 클래스로 최종 방향을 확정한다.
- 최종값이 `green_4`이면 지름길 준비를 취소하고 Xbin RULE 직진으로 복귀한다.
- 최종값이 `left_4`이면 LR-ASPP로 W1을 찾고 지름길 진입 후보를 활성화한다.
- OpenCV HSV 방향 판별은 현재 비활성화되어 있으며 YOLO 클래스만 사용한다.
- 지름길 진입과 통과 중에는 차량 회피가 주행권을 빼앗지 않도록 억제한다.

3바퀴 정책은 SLAM이나 odometry 대신 S자 구간 handoff와 다음 신호등 세션을 조합해 랩을 구분한다.

1. 1바퀴는 신호 방향과 관계없이 직진한다.
2. S자 handoff가 다음 랩을 준비하고, 다음 새로운 신호등 세션에서 2바퀴로 전환한다.
3. 2바퀴는 YOLO의 최종 직진·좌회전 판단을 따른다.
4. 2바퀴에서 지름길 통과를 완료하면 3바퀴는 강제로 직진한다.
5. 2바퀴가 직진했거나 지름길에 실패했다면 3바퀴도 YOLO 판단을 사용한다.

같은 신호등이 여러 프레임 보이더라도 한 세션으로 계산하며, 검출이 2프레임 사라져야 다음 신호 세션으로 재무장한다.

### 라바콘

- 중앙 영역에서 YOLO 콘을 처음 확인하면 RULE 조향을 유지하면서 속도 상한을 `15`로 낮춘다.
- 카메라와 연결된 LiDAR 콘 군집의 전방 거리가 `1.15 m` 이내이면 속도 상한을 `8`로 낮춘다.
- 실제 CONE 조향권 진입 거리 `0.95 m`는 감속 기준과 분리되어 있다.
- LiDAR에서 좌우 경계를 분리하고 두 경계의 중앙에 주행 경로를 만든다.
- 라바콘 주행 속도 기본값은 `8`이다.
- 탈출 후 S자 진입 전까지 단일 라바콘 때문에 CONE 모드가 다시 켜지지 않도록 재진입을 억제한다.

### 차량 회피

- YOLO 차량 bbox와 LiDAR 거리·빈 공간을 결합한다.
- 장애물이 오른쪽이면 왼쪽, 왼쪽이면 오른쪽으로 회피한다.
- 좌우 이동량 기본값은 각각 `0.13 m`, 이동 변화율은 `0.65 m/s`, 속도 상한은 `20`이다.
- 회피 중 반대쪽 판단이 2프레임 확인되면 회피 방향을 다시 선택하고, 재선택 구간은 `1.30 m/s`로 오프셋을 전환한다.
- `AVOID_RIGHT`는 오른쪽 오프셋이 자리 잡을 때까지 속도를 `14`로 제한하고, 처음 `0.35초` 동안 남아 있는 반대 조향을 억제한다.
- 동적 장애물 `green_car`가 S자 진입 가드 중 2프레임 다시 보이면 가드를 해제하고 즉시 회피를 재개한다.
- 장애물 소실 후 중앙선 횡오차가 `0.08 m` 이내로 3프레임 확인되어야 RULE 중앙 추종으로 복귀한다.
- `red_car` 회피 직후 좌회전에서는 전역 S자 보호를 바꾸지 않고 해당 구간에만 방향 전환 지연을 완화한다.

## 새 자이카 설치와 빌드

Ubuntu 22.04, ROS 2 Humble과 자이카 USB 장치 별칭(`/dev/ttyMOTOR`, `/dev/ttyLIDAR`)이 준비된 실차 PC에서 다음 순서로 설치한다. 모델 파일은 저장소에 포함되어 있으므로 별도로 다운로드하지 않는다.

오늘 완주 설정의 실차 재현 조건은 다음과 같다.

- 광각 카메라 장착 각도는 팀 기준 위치 `1`로 고정한다.
- 카메라는 `/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0`로 인식되어야 한다.
- VESC와 LiDAR는 각각 `/dev/ttyMOTOR`, `/dev/ttyLIDAR` 별칭으로 인식되어야 한다.
- ROS 도메인은 `7`을 사용하며, 다른 워크스페이스 overlay를 함께 source하지 않는다.
- 모터 배터리와 VESC fault 상태가 정상일 때만 실차 주행을 시작한다.

```bash
cd /home/xytron
git clone --branch main --single-branch \
  https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  xycar_ws

cd /home/xytron/xycar_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash src/xycar_map_nav/scripts/build_xycar_only.sh
```

기존 `/home/xytron/xycar_ws`가 있는 차량에서는 새 clone으로 덮어쓰지 말고 별도 경로에서 먼저 빌드와 센서 점검을 수행한다. 다른 ROS 워크스페이스 overlay가 섞이지 않도록 실행에는 전용 `install_xycar_only`를 사용한다.

```bash
cd /home/xytron/xycar_ws
bash src/xycar_map_nav/scripts/build_xycar_only.sh
```

## 통합 주행

아래는 대회용 최종 기준 명령이다. direct Xbin 인지와 3바퀴 정책을 사용하며, 직선 `25`, 곡선/S자 `12`, DEGRADED `15`, S자 진입 `11`, `red_car` 이후 좌회전 `13`, 라바콘 `8`, 전체 상한 `25`를 적용한다.

```bash
cd /home/xytron/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source install_xycar_only/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

XYCAR_RULE_PERCEPTION_BACKEND=direct_xbin \
OVERALL_SPEED_LIMIT_COMMAND=25 \
CURVATURE_SPEED_CONTROL_ENABLED=true \
CURVE_SPEED_COMMAND=12 \
DEGRADED_PATH_SPEED_COMMAND=15 \
S_CURVE_ENTRY_SPEED_CAP_COMMAND=11 \
S_CURVE_ENTRY_RED_CAR_SPEED_CAP_COMMAND=13 \
CONE_APPROACH_CONFIRMED_DISTANCE_M=1.15 \
CURVE_STEERING_MULTIPLIER=1.0 \
VEHICLE_LEFT_OFFSET_M=0.13 \
VEHICLE_RIGHT_OFFSET_M=0.13 \
XYCAR_ENABLE_RVIZ=false \
bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh \
  25 0.30 20 0
```

실행 중 고급 조향 파라미터는 터미널 질문에서 확인하거나 변경할 수 있다. 모든 센서가 정상이고 `READY`가 표시된 뒤 `Space`를 한 번 누르면 출발하며, 다시 누르면 정지한다. `Ctrl+C`는 전체 스택을 종료한다.

주행 시작 전에 반드시 다음을 확인한다.

- `/dev/ttyMOTOR`, `/dev/ttyLIDAR`, 카메라 장치가 존재하는지 확인한다.
- 기존 주행 노드가 남아 있지 않은지 확인한다.
- `/xycar_motor` publisher가 `space_drive_gate` 하나인지 확인한다.
- VESC 저전압 또는 fault 로그가 있으면 주행하지 않는다.

## 주행 기록

rosbag은 저장소 밖의 `/home/xytron/rosbags/integrated_drive`에 저장한다.

```bash
cd /home/xytron/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source install_xycar_only/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

BAG_DIR=/home/xytron/rosbags/integrated_drive
BAG_NAME="integrated_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BAG_DIR"

ros2 bag record \
  --compression-mode file \
  --compression-format zstd \
  -o "$BAG_DIR/$BAG_NAME" \
  /wide_camera_mjpeg/image_raw/compressed \
  /perception/xbin_direct_centerline \
  /perception/canonical_road_image \
  /lane_seg/yellow_centerline_mask \
  /lane_seg/white_boundary_mask \
  /my_rule/object_detections \
  /my_rule/object_detection/debug_image \
  /scan \
  /my_rule/cone_clusters \
  /my_rule/cone_cmd \
  /rule_drive/connected_yellow_path \
  /rule_drive/diagnostics \
  /hybrid/rule_candidate \
  /hybrid/avoidance_lateral_offset \
  /hybrid/shortcut_candidate \
  /hybrid/traffic_light_status \
  /hybrid_gate/mode \
  /hybrid_gate/status \
  /hybrid_gate/xycar_motor_shadow \
  /vehicle/vesc_state \
  /xycar_motor \
  /rosout
```

## 현재 검증 상태

- 2026-08-23 실차 통합 주행 3바퀴 완주
- direct Xbin 경로, RULE, 신호등, 지름길, 라바콘 및 차량 회피 통합 확인
- 첫 바퀴 직진, 2·3바퀴 지름길 선택 정책을 단위 테스트로 검증
- 관련 pytest `320개` 통과
- `install_xycar_only` 독립 빌드 성공
- 최종 `/xycar_motor` publisher 1개 구조 유지

실행 시 적용된 모든 값은 `/tmp/xycar_hybrid_run_config.yaml`에 저장되고, 상세 제어 로그는 `/tmp/xycar_hybrid_control_*.log`에 기록된다.
