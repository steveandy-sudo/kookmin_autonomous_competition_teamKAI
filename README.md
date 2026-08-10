# GSW 신호등·지름길 통합 주행

`gsw` 브랜치는 국민대 자율주행대회 실차 스택에 다음 기능을 통합한 전용
브랜치다.

- 4구 신호등 YOLO 클래스 보존: `red_4`, `yellow_4`, `green_4`, `left_4`
- 신호등 판단을 통합 선택기의 최상위 권한으로 적용
- `left_4` 확인과 소실을 이용한 지름길 자동 진입
- 기존 `ShortcutCore`의 진입, 차선 추종, 노란 T 검출, 탈출 주행 재사용
- 명령행에서 지름길 시작 지연시간 변경
- SPACE 키 기반 실차 안전 게이트

현재 최종 권한 순서는 다음과 같다.

```text
TRAFFIC_LIGHT(STOP/SHORTCUT)
  > CONE_RULE
  > YOLO_LIDAR_AVOIDANCE
  > RULE
```

`green_4`와 `left_4` 접근 대기 상태는 차를 직접 조향하지 않고, 하위 RULE
주행을 계속 허용한다. `red_4`/`yellow_4` 정지와 활성화된 지름길 주행만
최상위 권한으로 최종 명령을 덮어쓴다.

## 신호등 동작

객체 모델 경로:

```text
xycar_ws/src/study/my_rule/models/kookmin_objects_best_20260804.pt
```

모델에는 다음 원본 클래스가 들어 있다.

```text
cone, green_3, green_4, green_car, left_4, null_4,
red_3, red_4, red_car, yellow_3, yellow_4
```

4구 신호등 클래스는 다른 이름으로 합치지 않고 그대로
`/my_rule/object_detections`에 발행한다. `null_4`는 제어에 사용하지 않아
필터에서 제외한다.

### red_4 / yellow_4

- confidence `0.50` 이상
- bbox 면적 / 전체 영상 면적 `0.025` 이상
- 2개 detector frame 연속 충족 시 정지 latch
- 한 프레임 누락으로 바로 출발하지 않음

1280x1024 영상에서 `0.025`는 약 32,768 px²이고 정사각형 환산 시 약
181x181 px이다. 실제 제공 샘플 `322x105 px`는 약 `0.025898`이므로 이
기준을 통과한다.

### green_4

- confidence와 bbox 조건을 2개 detector frame 연속 충족하면 정지 latch 해제
- 정지 상태가 아니면 기존 RULE/콘/회피 중재를 그대로 계속함

### left_4

1. confidence `0.50` 이상으로 2개 detector frame 연속 검출
2. `left_4`가 보이는 동안 기존 RULE 주행으로 직진 접근
3. 첫 미검출 frame부터 `shortcut_start_delay_sec` 타이머 시작
4. 2개 detector frame 연속 미검출로 소실 확인
5. 설정 시간이 경과하면 지름길 후보 노드를 활성화
6. 지름길 완료 신호 뒤 RULE 중재로 복귀

단발 검출이나 단발 누락으로는 지름길을 시작하지 않는다.

## 지름길 주행 단계

기존 `track_drive_sve/shortcut_core.py`를 다음 순서로 실행한다.

```text
ENTER -> CRUISE -> EXIT -> DONE
```

- `ENTER`: 고정 좌회전, speed 1, 3.2초
- `CRUISE`: 흰색/노란색 차선 기반 지름길 차선 추종
- 진입 후 첫 5초는 입구 노란 무늬의 T자 오검출을 무시
- 노란 T자 검출 시 `EXIT` 고정 좌회전, speed 10, 2.7초
- `DONE`: 지름길 권한 해제 후 RULE 복귀

코어의 고정 조향값 `-100`은 통합 실차 안전 범위에서 `-42`로 제한된다.
카메라 또는 지름길 후보 명령이 `0.35초`보다 오래되면 차량을 정지시킨다.

## 준비

기본 환경은 Ubuntu 22.04와 ROS 2 Humble이다. 다음 장치를 먼저 확인한다.

- 광각 카메라: 기본
  `/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0`
- LiDAR: `/scan` 발행
- VESC: `/dev/ttyMOTOR`
- 모터 배터리와 비상 정지 수단

객체 YOLO Python 의존성:

```bash
python3 -m pip install -r xycar_ws/src/study/my_rule/requirements.txt
```

## 빌드

저장소를 받은 뒤 다음 명령을 실행한다.

```bash
cd xycar_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --packages-up-to \
  wide_camera xycar_vesc_driver my_rule_msgs my_rule \
  track_drive_sve xycar_map_nav

source install/setup.bash
```

## 권장 실차 실행

센서와 VESC를 연결한 상태에서 `xycar_ws` 디렉터리에서 실행한다.

```bash
SHORTCUT_START_DELAY_SEC=0.75 \
  ./src/xycar_map_nav/scripts/run_complete_rule_only.sh
```

스크립트가 주행 속도와 RULE 제어 파라미터를 순서대로 질문한다. 값을 입력하지
않고 Enter를 누르면 기본값을 사용한다. 모든 센서와 제어 토픽이 준비되면
다음 안내가 나온다.

```text
Press SPACE once to RUN. Press SPACE again to STOP.
```

- 첫 SPACE: 주행 시작
- 두 번째 SPACE: 즉시 정지
- Ctrl+C: 전체 센서·제어 프로세스 종료

SPACE를 누르기 전에는 `left_4` 상태와 지름길 타이머도 진행하지 않는다.

## 지름길 시작 지연 튜닝

기본값은 `0.75초`다. 지름길 좌회전이 늦으면 `0.50`, 너무 빠르면 `1.00`
등으로 바꾼다.

```bash
SHORTCUT_START_DELAY_SEC=0.50 \
  ./src/xycar_map_nav/scripts/run_complete_rule_only.sh
```

다섯 번째 위치 인자로도 지정할 수 있다.

```bash
./src/xycar_map_nav/scripts/run_complete_space_hybrid.sh \
  16 0.3 20 12 0.50
```

직접 launch할 때는 다음 인자를 사용한다.

```bash
ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  shortcut_start_delay_sec:=0.50
```

이 값은 **지름길 주행이 시작되기 전 직진 접근 시간**이다. `ENTER`의 고정
좌회전 시간 3.2초와는 다른 설정이다.

## 주요 튜닝 파일

```text
xycar_ws/src/xycar_map_nav/config/sequential_hybrid_real.yaml
xycar_ws/src/study/my_rule/config/object_detection.yaml
xycar_ws/src/track_drive_sve/track_drive_sve/config.py
```

주요 파라미터:

| 파라미터 | 기본값 | 의미 |
| --- | ---: | --- |
| `traffic_light_min_confidence` | 0.50 | 4구 신호등 최소 confidence |
| `traffic_light_stop_min_box_area_ratio` | 0.025 | 정지 신호 근접 bbox 기준 |
| `traffic_light_go_min_box_area_ratio` | 0.025 | green 해제 bbox 기준 |
| `traffic_light_stop_required_frames` | 2 | red/yellow 연속 검출 수 |
| `traffic_light_go_required_frames` | 2 | green 연속 검출 수 |
| `shortcut_yolo_required_frames` | 2 | left_4 연속 검출 수 |
| `shortcut_yolo_absence_frames` | 2 | left_4 연속 미검출 수 |
| `shortcut_start_delay_sec` | 0.75 | 첫 미검출 후 직진 접근 시간 |
| `shortcut_candidate_timeout_sec` | 0.35 | 지름길 후보 stale 정지 기준 |

## 상태 확인

다른 터미널에서 다음 토픽을 확인할 수 있다.

```bash
source /opt/ros/humble/setup.bash
source xycar_ws/install/setup.bash

ros2 topic echo /my_rule/object_detections
ros2 topic echo /hybrid/traffic_light_status
ros2 topic echo /hybrid/shortcut_status
ros2 topic echo /hybrid_gate/status
ros2 topic echo /hybrid_gate/mode
```

주행할 때 입력한 값은 다음 파일에 기록된다.

```text
/tmp/xycar_hybrid_run_config.yaml
```

## 시험 권장 순서

1. 바퀴를 띄운 상태에서 `red_4`, `yellow_4`, `green_4` 정지/해제를 확인한다.
2. `left_4`를 2회 연속 보여준 뒤 가리고 상태가 `LEFT_4_APPROACH`에서
   `SHORTCUT_ENTER`로 바뀌는지 확인한다.
3. 저속 실차 시험은 `SHORTCUT_START_DELAY_SEC=0.75`부터 시작한다.
4. 좌회전 시작 위치가 늦으면 `0.10~0.25초` 단위로 줄인다.
5. 지름길 CRUISE 차선 추종과 노란 T EXIT를 별도로 확인한다.
6. 한 번에 여러 파라미터를 바꾸지 않는다.

## 테스트

```bash
cd xycar_ws
source /opt/ros/humble/setup.bash
colcon test --packages-select my_rule track_drive_sve xycar_map_nav
colcon test-result --verbose
```

구현 세부 파일 목록은
[`xycar_ws/src/xycar_map_nav/README.md`](xycar_ws/src/xycar_map_nav/README.md)를
참고한다.
