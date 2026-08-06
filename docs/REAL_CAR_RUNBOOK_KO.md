# 실차 RULE·콘·차량 회피 실행

이 문서만 실차 실행 기준으로 사용한다. 기본 주행은 RULE이며 우선순위는
`콘 > 차량 회피 > RULE`이다. 신호등은 차량 회피 대상에서 제외된다.

현재 기준값은 다음과 같다.

- RULE 속도: 처음에는 command `3`
- 곡선 Lookahead: `0.30 m`
- 조향 혼합: Pure Pursuit `80%` + Stanley `20%`
- 실차 좌측 보정: `9 cm`
- 콘 속도: command `6`
- 차량 회피: YOLO 차량 감지와 LiDAR 거리를 함께 사용

## 1. 실차 코드 업데이트

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller

git remote get-url teamkai >/dev/null 2>&1 || \
  git remote add teamkai https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI.git
git fetch teamkai

if git show-ref --verify --quiet refs/heads/agent/integrated-rule-drive-tuning; then
  git switch agent/integrated-rule-drive-tuning
else
  git switch -c agent/integrated-rule-drive-tuning \
    --track teamkai/agent/integrated-rule-drive-tuning
fi

git pull --ff-only teamkai agent/integrated-rule-drive-tuning
```

로그인이 풀렸다면 먼저 다음 명령을 실행한다.

```bash
gh auth login
gh auth setup-git
```

## 2. 빌드

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  kaiev26_msgs \
  wide_camera \
  xycar_lidar \
  xycar_vesc_driver \
  xycar_perception \
  lane_seg_control \
  my_rule_msgs \
  my_rule \
  xycar_rule_drive \
  xycar_map_nav

source install/setup.bash
```

## 3. 장치 확인

카메라, LiDAR, VESC USB와 모터 배터리를 연결한 뒤 확인한다.

```bash
ls -l /dev/v4l/by-id/
readlink -f /dev/ttyLIDAR
readlink -f /dev/ttyMOTOR
```

세 항목 중 하나라도 경로가 나오지 않으면 주행하지 않는다.

## 4. 통합 주행 실행

바퀴를 들거나 즉시 정지할 수 있는 상태에서 처음 실행한다.

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh \
  3 1 rule speed100 0.3 20 9
```

스크립트가 카메라, LiDAR, VESC, 차선 인지, RULE 후보, 객체 감지와 주행
선택기를 자동으로 확인한다. `READY`가 출력되기 전에는 출발하지 않는다.

- `SPACE`: 주행 시작
- `SPACE`: 다시 누르면 즉시 정지
- `Ctrl+C`: 전체 실행 종료

인자의 순서는 다음과 같다.

```text
속도  시작WP  모드  프로필  Lookahead(m)  Stanley(%)  좌측보정(cm)
3     1       rule  speed100 0.3           20          9
```

기본값을 대화식으로 입력하려면 다음 명령을 사용한다.

```bash
bash src/xycar_map_nav/scripts/run_complete_rule_only.sh 3
```

## 5. 정지 상태 점검

통합 스크립트에서 `READY`가 나온 뒤, 아직 `SPACE`를 누르지 않은 상태에서
다른 터미널을 열어 확인한다.

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7

ros2 topic hz /wide_camera_mjpeg/image_raw/compressed
ros2 topic hz /scan
ros2 topic hz /perception/canonical_road_image
ros2 topic echo /hybrid/rule_candidate --once
ros2 topic echo /my_rule/object_detections --once
ros2 topic echo /hybrid_gate/status --once
```

`/hybrid/rule_candidate`가 없거나 상태에 `CANDIDATE_STALE`이 나오면 주행하지
말고 제어 로그를 확인한다.

```bash
tail -n 100 "$(ls -t /tmp/xycar_hybrid_control_*.log | head -n 1)"
tail -n 100 "$(ls -t /tmp/xycar_hybrid_sensors_*.log | head -n 1)"
```

## 6. 실차 시험 순서

한 단계가 안정적으로 정지까지 되는 것을 확인한 뒤 다음 단계로 진행한다.

1. 직선과 곡선 RULE 주행: 장애물 없이 command `3`
2. 콘 주행: 콘 구간 진입과 RULE 복귀 확인
3. 고정 차량 회피: 차량만 피하고 신호등은 피하지 않는지 확인
4. 천천히 움직이는 차량 회피: 진입·나란히 주행·원래 차선 복귀 확인
5. 전체 통합 주행

속도는 먼저 `3`, 다음 `6`, 마지막 `8` 순으로 올린다. 속도만 바꾸고
Lookahead `0.3`, Stanley `20`, 좌측 보정 `9`는 그대로 유지한다.

```bash
# command 6
bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh \
  6 1 rule speed100 0.3 20 9

# command 8
bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh \
  8 1 rule speed100 0.3 20 9
```

## 7. 이전 실행이 남아 있을 때

먼저 실행 중인 통합 터미널에서 `Ctrl+C`를 누른다. 그래도 노드가 남으면
아래처럼 이 통합 주행 노드만 종료한다.

```bash
pkill -INT -f 'real_hybrid_test_sensors.launch.py'
pkill -INT -f 'real_sequential_hybrid_drive.launch.py'
pkill -INT -f 'space_drive_gate'
sleep 2
ros2 node list
```

센서와 통합 제어 노드가 사라진 것을 확인한 뒤 4절 명령을 다시 실행한다.
