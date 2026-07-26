# 실차 IMU yaw 회전각 검증

이 시험은 모터 명령을 발행하지 않는다. 차량을 평평한 바닥에 놓고 차체 전체를
손으로 돌려 실제 각도와 `/imu` quaternion yaw 및 Z축 각속도를 비교한다.

## 실행

터미널 1에서 IMU만 실행한다.

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

ros2 launch xycar_imu xycar_imu.launch.py
```

터미널 2에서 측정기를 실행한다.

```bash
cd /home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

SESSION="imu_yaw_$(date +%Y%m%d_%H%M%S)"
ros2 run xycar_map_nav imu_yaw_calibrator --ros-args \
  -p session_name:="$SESSION"
```

## 키와 순서

위에서 내려다봤을 때 왼쪽/반시계 방향이 ROS 양의 yaw다.

1. 최초 방향에서 2초 정지한 뒤 `0`을 누른다.
2. 왼쪽 90도로 돌리고 2초 정지한 뒤 `1`을 누른다.
3. 처음 방향으로 돌아와 2초 정지한 뒤 `c`를 누른다.
4. 오른쪽 90도로 돌리고 2초 정지한 뒤 `2`를 누른다.
5. 처음 방향으로 돌아와 2초 정지한 뒤 `c`를 누른다.
6. `0`으로 기준을 다시 잡고 왼쪽 180도에서 `3`을 누른다.
7. 처음 방향으로 돌아와 `c`를 누른다.
8. `0`으로 기준을 다시 잡고 왼쪽으로 한 바퀴 돌아 `5`를 누른다.
9. `q`를 눌러 저장하고 종료한다.

오른쪽 180도와 한 바퀴도 측정할 때는 각각 `4`, `6`을 사용한다.

## 결과

결과는 다음 위치에 자동 저장된다.

```text
~/xycar_test_results/imu_yaw/<session>/
├── imu_samples.csv
├── imu_markers.csv
└── imu_yaw_report.json
```

권장 통과 기준은 90도 오차 3도 이하, 180/360도 오차 5도 이하,
최대 메시지 간격 0.5초 미만이다. `suggested_yaw_scale`은 IMU 측정 회전량에
곱할 후보값이며, 실제 odom 설정에 적용하기 전에 모든 방향 측정을 함께
검토한다.

## 2026-07-26 드라이버 수정

최초 실측에서 90/180/360도 차체 회전이 quaternion yaw에 반영되지 않는
문제를 확인했다. 원인은 `transforms3d.euler2quat()`의 `(w, x, y, z)`
반환값을 ROS 메시지의 `(x, y, z, w)`에 순서 변환 없이 넣은 것이었다.
드라이버와 3D 표시기를 명시적인 순서 변환 함수로 수정했다. 수정 전 측정
결과는 원인 확인 자료로 보존하고, 패키지 재빌드 후 같은 절차를 다시 수행한다.

## 2026-07-26 재측정 결론

재측정 세션은 다음 경로에 보존한다.

```text
/home/xytron/xycar_test_results/imu_yaw/imu_yaw_fixed_20260726_170031
```

quaternion yaw는 실제 90/180/360도 회전 중 거의 변하지 않아 SLAM heading
소스로 사용할 수 없었다. 반면 정지 상태 Z축 자이로 바이어스
`+0.030 rad/s`를 뺀 뒤 적분하면 다음 결과를 얻었다.

| 회전 | 실측 gyro Z 적분 | ROS 부호 변환 후 오차 |
|---|---:|---:|
| 좌 90도 | -90.81도 | +0.81도 |
| 우 90도 | +89.87도 | +0.13도 |
| 좌 180도 | -182.55도 | +2.55도 |
| 좌 360도 | -362.44도 | +2.44도 |

따라서 실차 VESC+IMU odom은 `gyro_z`를 사용하며, 좌회전을 ROS 양의
yaw로 만들기 위해 부호 `-1`을 적용한다. 제자리 회전 네 번의 최소제곱
배율은 `0.9922`였지만, 실제 복도 재방문에서 물리적 720도 회전에 748도가
누적된 결과를 반영해 production 배율은 `0.955`로 조정했다. 정지 중 양자화
잡음은 `0.015 rad/s` 데드밴드로 억제한다.
