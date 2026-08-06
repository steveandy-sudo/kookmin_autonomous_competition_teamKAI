# CAD v4 룰 기반 임무 시뮬레이션

이 환경은 국민대 CAD v4 트랙에서 모델 주행을 제외하고 룰 기반 차선
주행, 콘 주행, YOLO-LiDAR 차량 회피를 조정하기 위한 Gazebo Harmonic
환경이다.

## 배치

| 물체 | 위치 및 동작 |
|---|---|
| 저속 이동차량 | 상단 직선 `x=-7.70~-4.60 m`, `y=5.59 m`를 `0.16 m/s`로 왕복 |
| 고정차량 | 하단 직선 `(1.30, -4.56) m` |
| 신호등 | 상단 직선 `(-3.90, 4.94) m`, 녹색 점등 |
| 콘 | 좌측 직선 `y=2.85~-1.65 m`, 0.45 m 간격, 총 22개 |

콘 양쪽 경계의 중심 간격은 0.78 m다. 실제 콘 밑판을 포함한 주행 가능
폭과 `cone_control.yaml`의 허용 통로 폭 `0.68~0.98 m`를 함께 만족한다.

배치는
`xycar_ws/src/xycar_map_nav/config/sim_mission_layout.yaml`에서 바꾼다.
수정 후 다음 명령으로 월드를 다시 생성한다.

```bash
python3 scripts/generate_kookmin_mission_world.py
gz sdf -k worlds/kookmin_xycar_track_cad_v4_missions.sdf
```

## 실행

```bash
cd ~/xycar_kookmin_gazebo_track/xycar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select my_rule_msgs my_rule xycar_rule_drive xycar_map_nav
source install/setup.bash

ros2 launch xycar_map_nav sim_rule_missions.launch.py \
  headless:=gui enable_rviz:=true drive_enabled:=false
```

`drive_enabled:=false`가 기본값이라 최종 명령은
`/hybrid_gate/xycar_motor_shadow`에만 나온다. 화면과 검출 결과를 확인한 뒤
Gazebo 차량을 실제로 움직일 때만 `drive_enabled:=true`로 바꾼다.

## 검출 모드

기본 실행은 Gazebo 물체의 실제 위치를 320x256 검출 상자로 투영한다.
이 모드는 YOLO 오검출과 무관하게 콘·회피 제어 파라미터를 먼저 조정하기
위한 것이다.

- 정답 검출: `/my_rule/sim_ground_truth_detections`, 약 10 Hz
- 콘 명령: `/my_rule/cone_cmd`
- 최종 모드: `/hybrid_gate/mode`
- 회피 오프셋: `/hybrid/avoidance_lateral_offset`
- 최종 시험 명령: `/hybrid_gate/xycar_motor_shadow`

학습한 실제 YOLO 모델까지 포함해 시험하려면 정답 검출을 끄고 다음처럼
실행한다.

```bash
ros2 launch xycar_map_nav sim_rule_missions.launch.py \
  headless:=gui enable_rviz:=true drive_enabled:=false \
  start_ground_truth:=false start_yolo:=true \
  object_detections_topic:=/my_rule/object_detections
```

두 검출 노드를 같은 토픽에 동시에 켜면 결과가 섞이므로 사용하지 않는다.

## 구간별 정지 시험

자이카를 콘 입구에 정지시켜 `CONE_RULE` 전환과 라이다 경로를 확인한다.

```bash
gz service -s /world/kookmin_xycar_track/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "xycar_ackermann", position: {x: -8.83, y: 3.55, z: 0.10}, orientation: {z: -0.70710678, w: 0.70710678}'
```

고정차량 회피는 다음 위치에서 확인한다.

```bash
gz service -s /world/kookmin_xycar_track/set_pose \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name: "xycar_ackermann", position: {x: 0.20, y: -4.56, z: 0.10}, orientation: {w: 1.0}'
```

정상 동작 시 콘 구간은 `/hybrid_gate/mode=CONE_RULE`, 고정차량 앞은
`YOLO_LIDAR_AVOIDANCE`가 된다. 정지 시험에서는 최종 명령을 적용하지
않으므로 물체와의 거리, 오프셋, 조향 명령을 안전하게 먼저 비교할 수 있다.

## 현재 검증값

- SDF 구문 검사 통과
- LR-ASPP 처리시간 약 12~25 ms, 7 Hz 출력
- 정답 검출 약 9.9~10.0 Hz
- 이동차량 왕복 전환 확인
- 콘 카메라 게이트, LiDAR 통로 생성, `CONE_RULE` 전환 확인
- 이동·고정차량 검출과 `AVOID_LEFT`, `+0.33 m` 회피 오프셋 확인

정답 검출로 제어 파라미터를 먼저 안정화하고, 그다음 실제 YOLO 모드,
마지막으로 동일한 룰 설정을 실차에 적용하는 순서로 사용한다.
