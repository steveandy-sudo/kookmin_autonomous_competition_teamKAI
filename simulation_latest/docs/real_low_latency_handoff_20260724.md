# 2026-07-24 실차 저지연 인지·제어 인수인계

이 문서는 실차 ASUS PC의 Codex/Copilot이 `simulation` 브랜치를 받은 뒤
저지연 인지와 룰베이스 또는 학습 정책을 재현하고, 측정 결과를 남기기 위한
실행 기준이다. 실차에서 Gazebo는 실행하지 않는다.

## 1. 해결하려는 문제

2026-07-24 실차 rosbag에서 확인한 기존 지연은 다음과 같았다.

```text
압축 카메라 원본 age                 약 30 ms
canonical 도착 age                  약 157~164 ms
학습 정책 추론                       평균 약 45 ms
정책 명령 시점의 영상 age            약 249 ms
카메라 촬영부터 yaw 반응              약 414 ms
```

기존 인지 경로는 다음 작업을 수행했다.

```text
MJPEG 30 Hz
  -> 외부 rectifier가 1280x1024 전체 프레임을 30 Hz 보정
  -> 큰 raw Image를 ROS로 전달
  -> LR-ASPP가 7 Hz 프레임 선택
  -> source/white/yellow ROS 메시지 3개 발행
  -> ApproximateTimeSynchronizer
  -> 세 메시지를 다시 OpenCV 배열로 변환
  -> color/white/yellow perspective warp 3회
  -> canonical
```

큰 raw 영상 전달, 사용하지 않을 23 Hz 프레임 보정, 중간 메시지 직렬화,
동기화 대기, 제어에 필요 없는 color BEV 생성이 포함돼 있었다.

## 2. 새 저지연 경로

새 실차 launch는 다음 경로를 사용한다.

```text
/wide_camera_mjpeg/image_raw/compressed, 30 Hz
  -> KeepLast(1)로 최신 프레임만 보관
  -> 7 Hz마다 가장 최신 JPEG 한 장만 decode 및 rectify
  -> LR-ASPP MobileNetV3-Small, 256x144
  -> NumPy 메모리 안에서 white/yellow mask를 바로 BEV로 변환
  -> 같은 프로세스에서 canonical 256x144 생성
  -> /perception/canonical_road_image
  -> 룰베이스 또는 camera-only 정책
```

활성 제어 경로에는 `lane_seg_canonical_adapter`가 뜨지 않는다. 중간
`/lane_seg/source_image`, white mask, yellow mask도 구독자가 없으면 만들거나
발행하지 않는다.

적용된 최적화:

1. 선택된 7 Hz 프레임만 JPEG decode와 fisheye 보정을 수행한다.
2. 1280x1024 raw ROS Image 전달을 제어 경로에서 제거했다.
3. LR-ASPP와 canonical 변환을 한 프로세스로 합쳤다.
4. source/white/yellow 3토픽과 ApproximateTimeSynchronizer를 제거했다.
5. 제어에 필요 없는 color BEV perspective warp를 제거했다.
6. 고정 BEV valid mask를 캐시한다.
7. 이미 분류된 마스크를 BGR, HSV, CLAHE로 다시 처리하던 단계를 제거했다.
8. 디버그 영상, 중간 영상, component count는 구독하거나 명시적으로 켠
   경우에만 계산한다.
9. 정책 입력 디버그 영상도 구독자가 있을 때만 만든다.
10. canonical 입력과 모터 명령 큐를 `KeepLast(1)`로 제한한다.
11. 인지·정책 CPU thread 수를 명시해 과도한 thread 생성과 경쟁을 막는다.
12. 오래된 입력은 비싼 추론 전에 폐기한다.

기존 adapter와 직접 경로는 같은 canonical 렌더 함수를 공유한다.
자동 테스트는 BEV white/yellow/valid와 최종 canonical RGB가 픽셀 단위로
동일함을 확인한다.

## 3. 이 PC에서 확인한 성능

본선 실차 rosbag `track_run_02_20260715_143636`의 압축 카메라를 7 Hz로
재생해 측정했다.

| 단계 | p50 | p95 |
|---|---:|---:|
| JPEG decode + fisheye rectify | 8.5 ms | 10.1 ms |
| LR-ASPP 전처리·추론 | 7.3 ms | 9.6 ms |
| BEV + canonical | 4.9 ms | 6.3 ms |
| 인지 callback 전체 | 21.0 ms | 24.3 ms |

58개 연속 canonical 프레임이 생성됐고 누락·stale drop은 없었다. 기존
공용 canonical 변환을 그대로 썼을 때 같은 PC의 callback p50은 29.9 ms였다.

이 수치는 개발 PC 결과다. 실차 ASUS Ryzen 5에서 아래 도구로 다시 측정해야
하며, 실차 결과를 측정하기 전에는 동일한 21 ms라고 가정하지 않는다.

## 4. 실차 Codex가 먼저 할 일

기존 `/home/xytron/kookmin_ty`에는 미커밋 변경이 있었으므로 자동으로
`reset --hard`, `checkout --`, `stash`를 실행하지 않는다. 새 clone을 사용한다.

```bash
cd /home/xytron
git clone --branch simulation \
  git@github.com:steveandy-sudo/kookmin_autonomous_competition_teamKAI.git \
  kookmin_ty_low_latency_20260724
cd /home/xytron/kookmin_ty_low_latency_20260724
git status -sb
git log -1 --oneline
```

빌드:

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
cd /home/xytron/kookmin_ty_low_latency_20260724

colcon build --packages-up-to \
  xycar_perception lane_seg_control xycar_rule_drive xycar_rl \
  xycar_final_drive \
  --symlink-install

source install/setup.bash
```

## 5. 카메라 노드 조건

차량의 기존 카메라 실행 절차로 MJPEG 카메라만 먼저 시작한다. 이 저장소는
카메라 하드웨어 드라이버와 VESC/ROS1 bridge를 대신 시작하지 않는다.

```bash
export ROS_DOMAIN_ID=7
source /opt/ros/humble/setup.bash
source /home/xytron/kookmin_ty_low_latency_20260724/install/setup.bash

ros2 topic info /wide_camera_mjpeg/image_raw/compressed -v
ros2 topic hz /wide_camera_mjpeg/image_raw/compressed
ros2 node list | grep -E 'wide_camera|rectifier'
```

압축 원본은 약 25~30 Hz여야 한다. 새 경로는 저장소의
`wide_camera_fisheye_1280x1024_20260708.yaml`과 `balance=0.3`을 내부에서
정확히 한 번 적용한다.

외부 `/wide_camera_rectifier`는 새 제어 경로에 필요 없다. 같은 카메라
보정을 30 Hz로 계속 수행하면 CPU만 경쟁한다. 먼저 프로세스를 확인하고
기존 카메라 터미널에서 rectifier만 `Ctrl+C`로 종료한다.

```bash
pgrep -af 'rectif|wide_camera'
```

반드시 `/wide_camera_mjpeg`와 압축 토픽은 남겨야 한다. rectifier를 끄면서
MJPEG 카메라까지 같이 종료됐다면 카메라 실행 구성을 분리한 뒤 진행한다.

## 6. 인지만 shadow 확인

```bash
ros2 launch lane_seg_control \
  lane_seg_lraspp_low_latency_real.launch.py \
  cpu_threads:=4 \
  max_output_rate_hz:=7.0
```

별도 터미널:

```bash
ros2 node list | grep -E 'lane_seg|rectifier'
ros2 topic hz /perception/canonical_road_image
```

정상 노드:

```text
/lane_seg_lraspp_inference
```

정상 제어 경로에는 다음 노드가 없어야 한다.

```text
/lane_seg_canonical_adapter
/wide_camera_rectifier
```

canonical은 `256x144 bgr8`, 약 `6.5~7.2 Hz`여야 한다. RGB 색상 계약은
배경 `(36,36,36)`, 흰선 `(255,255,255)`, 노란선 `(0,220,255)`다.

RViz와 영상 debug는 확인할 때만 켠다. 실제 지연 측정 및 주행에서는 영상
구독자를 닫아 디버그 직렬화와 렌더 부하를 제거한다.

## 7. 룰베이스 통합 shadow

기존 perception 터미널을 종료한 뒤 통합 launch 하나만 실행한다.

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=rule \
  drive_enabled:=false \
  perception_cpu_threads:=4
```

확인:

```bash
ros2 topic hz /perception/canonical_road_image
ros2 topic hz /xycar_motor_shadow
ros2 topic echo /xycar_motor_shadow
```

shadow 조향 부호, 곡선 진입 시점, 명령 공백을 확인한 뒤에만 바퀴를 띄워
저속 시험한다. 처음부터 `drive_enabled:=true`를 실행하지 않는다.

## 8. 학습 정책 통합 shadow

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=rl \
  drive_enabled:=false \
  device:=cpu \
  perception_cpu_threads:=4 \
  policy_cpu_threads:=4
```

확인:

```bash
ros2 topic hz /rl/policy_motor_shadow
ros2 topic echo /rl/policy_debug
ros2 topic echo /rl/policy_status
```

정책 입력은 canonical source-driven 7 Hz다. 정책에 두 번째 7 Hz rate limit를
걸지 않는다. `max_inference_rate_hz=0`이 정상 기본값이다.

## 9. 30초 지연 측정

인지 또는 통합 shadow가 실행 중일 때:

```bash
cd /home/xytron/kookmin_ty_low_latency_20260724
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 scripts/measure_live_pipeline_latency.py \
  --duration 30 \
  --output ~/kookmin_latency_20260724_threads4.json
```

도구가 출력하는 주요 값:

```text
canonical_rate.hz
policy_command_rate.hz
lane.decode_rectify_ms.p50/p95
lane.model_ms.p50/p95
lane.canonical_ms.p50/p95
lane.callback_total_ms.p50/p95
lane.source_age_ms.p50/p95
policy.inference_ms.p50/p95
policy.image_age_at_command_ms.p50/p95
```

`lane.replaced_input_count`는 30 Hz 중 7 Hz만 선택하므로 증가하는 것이
정상이다. `lane.stale_input_count`와 `policy.stale_frame_count`는 정상 live
주행에서 계속 증가하면 안 된다.

첫 shadow 통과 기준:

- canonical과 정책 출력이 각각 6.5~7.2 Hz
- 인지 callback p95가 80 ms 미만
- 정책 inference p95가 80 ms 미만
- 정책 명령 시점 영상 age p95가 200 ms 미만
- 250 ms 이상 명령 공백이 반복되지 않음
- stale frame 누적이 지속 증가하지 않음

## 10. 실차 CPU thread 선택

실차 Ryzen 5에서 `2`, `4`, `6` thread를 각각 30초 shadow 측정한다. 한 번에
여러 stack을 띄우지 않는다.

```bash
ros2 launch xycar_final_drive final_real_stack.launch.py \
  driver_mode:=rl drive_enabled:=false device:=cpu \
  perception_cpu_threads:=2 policy_cpu_threads:=2
```

같은 방법으로 `4`, `6`을 측정한다. 평균만 보지 말고
`callback_total_ms.p95`, `policy.inference_ms.p95`, canonical Hz를 비교한다.
기본값 4는 시작점일 뿐이며 실차 측정에서 p95가 가장 낮은 조합을 선택한다.

확인할 시스템 상태:

```bash
nproc
lscpu | grep -E 'Model name|CPU\\(s\\)|Thread|Core'
cpupower frequency-info
sensors
```

성능 governor 변경, CPU affinity, process priority 변경에는 관리자 권한과
현장 검증이 필요하다. 실차 Codex는 측정 결과 없이 자동 적용하지 않는다.

## 11. 주행 전 실차에서 추가로 해야 할 일

코드 밖에서 남는 지연은 다음과 같다.

1. 카메라 exposure와 MJPEG 발행 주기가 흔들리지 않는지 확인
2. CPU thermal throttling과 governor 확인
3. ROS1 dynamic bridge와 VESC 명령 주기 확인
4. `/xycar_motor` 명령부터 실제 앞바퀴 또는 IMU yaw 반응까지 재측정
5. 룰베이스와 정책을 각각 별도 bag으로 기록
6. 물리 비상 정지와 단일 motor publisher 확인

기록 토픽:

```text
/wide_camera_mjpeg/image_raw/compressed
/perception/canonical_road_image
/lane_seg/diagnostics
/xycar_motor
/xycar_motor_shadow
/rl/policy_motor_shadow
/rl/policy_debug
/rl/policy_status
/rule_drive/diagnostics
/imu
/vehicle/vesc_state
```

## 12. 실차 Codex 작업 규칙

실차 Codex는 다음 순서를 지킨다.

1. 기존 clone의 미커밋 변경을 삭제하지 않고 새 clone을 만든다.
2. 모터와 Gazebo를 실행하지 않은 상태에서 빌드와 인지 smoke test를 한다.
3. 외부 rectifier만 제거하고 MJPEG 카메라가 살아 있는지 재확인한다.
4. `2/4/6` CPU thread shadow 측정 결과 JSON을 남긴다.
5. canonical과 정책이 모두 6.5 Hz 이상일 때만 룰 또는 정책 shadow를 본다.
6. 자동으로 `drive_enabled:=true`를 실행하지 않는다.
7. 사람이 비상 정지를 준비한 상태에서 바퀴 공중 시험부터 진행한다.
8. 속도 4 직선, 속도 4 단일 곡선, 속도 6 순서로 올린다.
9. 결과 bag, JSON, 실제 commit hash를 한 보고서에 기록한다.
10. 실차에서 필요한 추가 수정은 별도 브랜치와 커밋으로 올린다.

## 13. 호환 경로

압축 직접 입력에 문제가 생겼을 때만 기존 rectified raw 입력으로 확인한다.

```bash
ros2 launch lane_seg_control lane_seg_lraspp_canonical_only.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  use_compressed_image:=false \
  enable_rectify:=false \
  direct_canonical_enabled:=true \
  max_output_rate_hz:=7.0
```

완전한 기존 2노드 adapter 비교:

```bash
ros2 launch lane_seg_control lane_seg_lraspp_canonical_only.launch.py \
  image_topic:=/wide_camera/rect/image_raw \
  use_compressed_image:=false \
  enable_rectify:=false \
  direct_canonical_enabled:=false \
  publish_intermediate_topics:=true \
  max_output_rate_hz:=7.0
```

호환 경로는 원인 비교용이다. 최종 실차 기본 경로는
`lane_seg_lraspp_low_latency_real.launch.py`다.
