# 2026-07-28 수정 경로 실차 주행

저장 지도에서 웨이포인트를 수정한 뒤 전역경로 주행 중 기록한 압축 ROS 2
bag이다. 지도 이미지와 최종 웨이포인트를 함께 보존해 당시 경로를 다시
확인할 수 있게 했다.

## 기록 정보

- 기록 시각: 2026-07-28 18:24:34 ~ 18:27:13 KST
- 길이: 159.189초
- 메시지: 56,615개
- 압축 bag 크기: 4.4 MiB
- storage: SQLite3
- compression: zstd file
- 웨이포인트: 11개, `closed: true`

최종 수정 좌표:

- 세 번째 `clicked_02`: `(0.1982, -0.3442)`
- 네 번째 `clicked_03`: `(1.0712, -0.8731)`

## 지도 점유공간 규칙

이 지도는 `map.pgm`의 픽셀값을 다음과 같이 사용한다.

- `254`: 확인된 자유공간
- `205`: 미확인 공간
- `0`: 점유공간

`map.yaml`의 `free_thresh`는 `0.19`다. 따라서 `205` 회색 영역은
자유공간이 아니며, `unknown_is_occupied: true`인 전역경로 계획에서는
통과할 수 없다. 새 지도를 저장하거나 교체할 때도 이 값을 유지한다.

## 기록 토픽

| 토픽 | 메시지 수 | 평균 주기 |
| --- | ---: | ---: |
| `/imu` | 5,667 | 35.599 Hz |
| `/scan` | 1,532 | 9.624 Hz |
| `/slam/scan_filtered` | 1,501 | 9.429 Hz |
| `/vehicle/vesc_state` | 7,772 | 48.822 Hz |
| `/vehicle/system_telemetry` | 155 | 0.974 Hz |
| `/odom` | 7,772 | 48.822 Hz |
| `/slam/odom` | 7,772 | 48.822 Hz |
| `/xycar_motor` | 2,976 | 18.695 Hz |
| `/map_nav/xycar_motor_shadow` | 2,976 | 18.695 Hz |
| `/map_nav/control_mode` | 2,976 | 18.695 Hz |
| `/tf` | 15,515 | 97.463 Hz |
| `/tf_static` | 1 | 정적 |

요청 목록에 있었던 `/map_nav/global_path`, route-localization 진단 및
`/diagnostics`는 기록 시작 시 활성 메시지가 없어 이 bag에는 들어 있지
않다. 경로는 동봉한 `waypoints_pure_pursuit.yaml`과 `map.yaml/.pgm`으로
재구성한다.

## 확인

```bash
source /opt/ros/humble/setup.bash

BAG_DIR=data/real_waypoint_runs/2026-07-28/edited_route_20260728_182433
ros2 bag info "$BAG_DIR"
ros2 bag play "$BAG_DIR" --clock
```

실차 모터를 연결한 상태에서는 bag 재생 전에 `/xycar_motor` remap 또는 VESC
`drive_enabled=false`를 적용해야 한다.

## SHA256

```text
ad1748917156f7fe3ad886bfcd93c9e2b0ad263ffb455f7dd5aed35b4b89fc35  edited_route_20260728_182433_0.db3.zstd
52364f68aaa9eb36e5e73afcd9ed22ee30e6ed6042c3983eb3433360ba766f6e  map.pgm
833ba5851978c9d30c736224deb9f8831dfa3388dd16f7c7ade68d03161cb971  map.yaml
0aaa8bd1395775d7bcf642d2baea8d2fc6112f12190e3aafebec4bfc84cf2d19  metadata.yaml
88621bff35e8428cb3a83ebfe15f4222c041d1d2a457a6223422920abad58e86  waypoints_pure_pursuit.yaml
```
