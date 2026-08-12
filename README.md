# Kookmin Autonomous Competition Team KAI

국민대 자율주행대회용 Xycar RULE·콘·차량 회피 통합 저장소다.

실차 실행은 runbook을 기준으로 하고, 구현 내용과 측정 결과는 현황 문서에서
확인한다.

- [실차 RULE·콘·차량 회피 실행](docs/REAL_CAR_RUNBOOK_KO.md)
- [2026-08-07 통합 주행 구현·검증 현황](docs/INTEGRATED_RULE_DRIVE_STATUS_20260807_KO.md)

현재 Jetson 대회 브랜치: `jetson_final`

이 브랜치는 저장소 자체가 ROS 2 작업공간이 되도록 구성한다. Jetson에서는
저장소를 `~/xycar_ws`에 두며 ROS 패키지는 `~/xycar_ws/src` 아래에 있다.

빌드 후 대회 스택의 단일 진입점은 다음과 같다.

```bash
cd ~/xycar_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xycar_final_drive final.launch.py
```

모터 안전 게이트와 하드웨어 검증 절차는
[`xycar_final_drive` README](src/xycar_final_drive/README.md)를 따른다.

## Jetson 이식 시 변경한 내용

`jetson_final` 브랜치는 검증 브랜치
`agent/yellow-center-curve-test`의 최신 상태를 Jetson Orin NX 실차용으로
정리한 브랜치다.

- 저장소 자체를 ROS 2 작업공간으로 바꿨다. 기존의 중첩
  `저장소/xycar_ws/src`를 `~/xycar_ws/src`로 이동하고 내부 상대경로,
  스크립트와 `.gitignore`를 함께 수정했다.
- [`final.launch.py`](src/xycar_final_drive/launch/final.launch.py)를 추가해
  카메라, LiDAR, ROS 2 VESC, GPU 인지와 하이브리드 주행 노드를 한 명령으로
  실행하도록 만들었다.
- LR-ASPP 인지 노드에 `device` 파라미터를 추가했다. 모델, warm-up tensor와
  입력 tensor를 지정 장치로 옮기며, `cuda`를 요청했는데 CUDA를 사용할 수
  없으면 조용히 CPU로 바꾸지 않고 오류로 중단한다.
- `final.launch.py`에서는 LR-ASPP와 RL 정책의 기본 장치를 `cuda`로 지정했다.
- Jetson aarch64에 사용자 prefix로 설치한 YDLidar SDK 1.2.7의 실제
  `ydlidar_sdk` 라이브러리를 찾고 연결하도록 LiDAR CMake 설정을 수정했다.
- 최종 모터 경로는 호스트의 native ROS 2 VESC 드라이버다. 현재 대회
  실행 경로에는 ROS 1 컨테이너나 `ros1_bridge`가 없다.
- 판단 publisher와 VESC 출력을 각각 막는 이중 안전 게이트를 추가했다.
  두 게이트는 모두 기본 `false`다.
- 모델과 보정 파일은 절대 사용자 경로가 아니라 ROS 패키지 share 경로로
  찾도록 구성했다.

## 빌드 및 실행 명령

Jetson CUDA PyTorch가 설치된 전용 Python 환경과 YDLidar SDK prefix를
사용해 빌드한다.

```bash
cd ~/xycar_ws
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/kookmin_ty/venvs/xycar-jp621/bin/activate
```

```bash
export CMAKE_PREFIX_PATH="$HOME/kookmin_ty/ydlidar-sdk-v1.2.7-install${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
```

```bash
python -m colcon build --packages-up-to xycar_final_drive --symlink-install
```

```bash
source install/setup.bash
```

대회 스택 실행 명령은 다음 하나다.

```bash
ros2 launch xycar_final_drive final.launch.py
```

이 기본 명령은 모든 장치와 인지·판단 노드를 시작하지만 모터 명령 발행과
VESC 비영점 출력은 모두 차단한다. 장치가 없는 개발 환경에서 GPU 경로만
검증할 때는 다음과 같이 하드웨어 시작을 끈다.

```bash
ros2 launch xycar_final_drive final.launch.py start_camera:=false start_lidar:=false start_vesc:=false command_drive_enabled:=false vesc_drive_enabled:=false device:=cuda
```

## 코드 내부 작동 흐름

```text
wide_camera
  /wide_camera_mjpeg/image_raw/compressed
    → LR-ASPP 차선 분할 + 어안 보정 + canonical 변환 (CUDA)
      /perception/canonical_road_image
        → hybrid_drive
          1순위: LiDAR cone rule
          2순위: canonical lane curve rule
          3순위: learned model straight
            → /xycar_motor_shadow (항상 확인용)
            → /xycar_motor (command_drive_enabled=true일 때만)
              → native ROS 2 xycar_vesc_driver
                → /dev/ttyMOTOR (vesc_drive_enabled=true일 때만 비영점 출력)
```

`driver_mode`는 `hybrid`, `rule`, `rl` 중 하나만 실행한다. 기본 `hybrid`는
다음 우선순위를 사용한다.

1. `/scan`에서 유효한 콘 통로가 검출되면 LiDAR cone rule을 사용한다.
2. canonical 경로가 곡선 진입 조건을 만족하면 Stanley/Pure Pursuit rule을
   사용한다.
3. 그 외 직선 구간에서는 카메라+속도 temporal learned policy를 사용한다.

인지 입력 또는 정책 출력이 timeout을 넘으면 오래된 결과를 정상 명령으로
계속 사용하지 않는다. VESC는 별도로 명령 watchdog, telemetry, firmware,
저전압과 fault를 검사한다.

## 현재 대회 launch 주요 파라미터

아래 값은 별도 인자를 주지 않았을 때 `final.launch.py`가 실제로 사용하는
기본값이다.

### 실행 및 안전

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `start_camera` | `true` | wide camera 시작 |
| `start_lidar` | `true` | YDLidar 시작 |
| `start_vesc` | `true` | native ROS 2 VESC 시작 |
| `start_drive_stack` | `true` | 인지·판단 스택 시작 |
| `start_perception` | `true` | LR-ASPP 인지 시작 |
| `driver_mode` | `hybrid` | `hybrid`, `rule`, `rl` 중 선택 |
| `device` | `cuda` | LR-ASPP와 learned policy 실행 장치 |
| `command_drive_enabled` | `false` | `/xycar_motor` 발행 허용 |
| `vesc_drive_enabled` | `false` | VESC 비영점 출력 허용 |

### 카메라 및 인지

| 파라미터 | 기본값 |
|---|---:|
| `camera_device` | `/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0` |
| `camera_width` × `camera_height` | `1280 × 1024` |
| `camera_fps` | `30` |
| LR-ASPP 모델 | `kookmin_lane_lraspp_mbv3s_256x144.pt` |
| 모델 입력 | `256 × 144` |
| canonical 출력 | `256 × 144` |
| 흰색/노란색 confidence | `0.50 / 0.50` |
| `max_output_rate_hz` | `15.0` |
| `max_input_age_sec` | `0.35` |
| `pipeline_qos_depth` | `1` |
| `perception_cpu_threads` | `4` |
| 어안 보정 파일 | `wide_camera_fisheye_1280x1024_20260708.yaml` |
| `rect_balance` | `0.3` |
| canonical 실제 범위 | 횡방향 `1.4 m`, 전방 `1.5 m` |

### 하이브리드 판단

| 파라미터 | 기본값 |
|---|---:|
| learned policy | `straight_speed25_recovery_v3_20260805/camera_speed_td3_bc_best.pth` |
| policy 입력 | `160 × 90` |
| `policy_cpu_threads` | `4` |
| `model_speed_cap` | `20.0` |
| `cone_speed_cap` | `9.5` |
| model timeout | `0.35 s` |
| canonical timeout | `0.50 s` |
| 곡선 진입/이탈 curvature | `0.16 / 0.10 1/m` |
| 곡선 진입/이탈 path angle | `0.12 / 0.11 rad` |
| 곡선 모드 최소 유지 | `1.2 s` |
| cone 진입 confidence | `0.35` |
| cone 진입 확인 | `3 frames` |
| cone 유효 거리 | `0.18..1.6 m` |
| 예상 cone corridor 폭 | `0.85 m` (`0.68..0.98 m`) |
| LiDAR→rear axle / wheelbase | `0.42 / 0.33 m` |

rule 실차 프로파일은 직선 `18`, 곡선 `16`, degraded path `12`, lane loss
`4`의 속도 명령과 `10 Hz` 명령률을 사용한다. `model_speed_cap=20.0`은
learned-policy 직선 속도에 적용되고 `cone_speed_cap=9.5`는 cone 모드에
적용된다. 이 두 cap이 모든 모드의 전역 속도 제한이라는 뜻은 아니다.

### LiDAR 및 VESC

| 파라미터 | 기본값 |
|---|---:|
| LiDAR port / baud | `/dev/ttyLIDAR` / `512000` |
| LiDAR frequency / frame | `10 Hz` / `laser_frame` |
| LiDAR angle / range | `-180..180°` / `0.1..16.0 m` |
| VESC port / baud | `/dev/ttyMOTOR` / `115200` |
| expected firmware | `2.18` |
| VESC control / telemetry | `20 / 50 Hz` |
| command / telemetry timeout | `0.5 / 0.25 s` |
| steering trim | `-5.0 command` |
| acceleration / deceleration limit | `0.6 / 1.5 m/s²` |
| low-voltage limit / stop / recovery | `7.5 / 6.0 / 8.0 V` |
| fault recovery stable time | `3.0 s` |

파라미터를 바꿀 때는 launch 인자로 명시하고, 실차에서 검증한 값만 코드
기본값으로 반영한다. 장치가 연결되지 않은 상태에서는 보정값이나 udev
식별값을 추측해 수정하지 않는다.

## Jetson 실차 하드웨어 연결 체크리스트

이 절차는 새 Jetson에 카메라, LiDAR, VESC를 처음 연결할 때 사용한다.
장치는 카메라 → LiDAR → VESC 순서로 하나씩 연결하고, 각 단계가 통과한
뒤에만 다음 장치로 넘어간다. 명령도 한 번에 하나씩 실행하고 결과를
기록한다.

### 0. 절대 안전 조건

- 차량 바퀴를 지면에서 띄우거나 구동축을 물리적으로 안전하게 만든다.
- 비상 정지 담당자와 모터 전원 차단 방법을 먼저 정한다.
- 처음에는 VESC 구동 전원을 끄고 USB 통신 장치부터 식별한다.
- USB 케이블과 Jetson 포트에 카메라, LiDAR, VESC 라벨을 붙인다.
- 장치명, VID, PID, serial을 확인하기 전에는 udev 규칙을 만들지 않는다.
- `/dev/ttyUSB0`처럼 연결 순서에 따라 바뀌는 이름을 실차 설정에 고정하지
  않는다.
- `command_drive_enabled`와 `vesc_drive_enabled`는 모두 `false`로 시작한다.
- 카메라 보정값, LiDAR 방향, 조향 부호가 확인되기 전에는 차량을 움직이지
  않는다.
- JetPack, L4T, CUDA, 커널, 부트로더를 업그레이드하지 않는다.

### 1. 터미널 환경 준비

```bash
cd ~/xycar_ws
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
source install/setup.bash
```

새 터미널을 열 때마다 위 세 명령을 순서대로 실행한다.

### 2. 장치 연결 전 기준 상태

아무 센서도 연결하지 않은 상태에서 다음 결과를 저장한다.

```bash
lsusb
```

```bash
find /dev/v4l/by-id /dev/serial/by-id -maxdepth 1 -type l -ls 2>/dev/null
```

```bash
id teamkai
```

`teamkai`가 `video`와 `dialout` 그룹에 속하는지 확인한다. 그룹 추가가
필요하면 시스템 변경 승인을 받은 후에만 다음 명령을 사용하고, 적용을 위해
로그아웃 후 다시 로그인한다.

```bash
sudo usermod -aG dialout,video teamkai
```

### 3. 카메라 단독 검증

카메라만 연결하고 `lsusb` 및 다음 명령으로 실제 장치 경로를 확인한다.

```bash
v4l2-ctl --list-devices
```

기본 launch는 다음 장치를 기대한다.

```text
/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0
```

확인된 경로가 다르면 실제 경로를 `device` 인자로 전달한다. 예시는 실제
경로를 확인한 뒤 사용한다.

```bash
ros2 launch wide_camera wide_camera.launch.py
```

다른 터미널에서 토픽과 주기를 하나씩 확인한다.

```bash
ros2 topic info /wide_camera_mjpeg/image_raw/compressed -v
```

```bash
ros2 topic hz /wide_camera_mjpeg/image_raw/compressed
```

통과 조건:

- 1280×1024 MJPEG 영상이 약 30 Hz로 들어온다.
- 프레임이 지속해서 증가하고 timestamp가 현재 시각과 맞는다.
- 좌우 반전, 상하 반전, 과도한 노출, 프레임 드롭이 없다.
- 저장소의 어안 보정값이 실제 카메라 및 장착 방향과 일치한다.

### 4. LiDAR 단독 검증

카메라 검증이 끝난 뒤 LiDAR를 연결한다. 새로 나타난 시리얼 장치의
속성을 확인한다. 아래의 장치명은 실제 발견된 값으로 바꾼다.

```bash
udevadm info --query=property --name=/dev/ttyUSB0
```

VID, PID, serial을 기록한 뒤에만 영구 이름 `/dev/ttyLIDAR`를 만드는 udev
규칙을 작성한다. 기본 설정은 다음과 같다.

- port: `/dev/ttyLIDAR`
- baud rate: `512000`
- frame: `laser_frame`
- frequency: `10.0 Hz`
- angle: `-180°..180°`
- range: `0.1..16.0 m`

영구 이름과 권한이 확인된 뒤 단독 실행한다.

```bash
ros2 launch xycar_lidar xycar_lidar.launch.py
```

```bash
ros2 topic hz /scan
```

```bash
ros2 topic echo /scan --once
```

통과 조건:

- `/scan`이 약 10 Hz로 끊기지 않는다.
- 가까운 장애물의 실제 방향과 스캔 각도 방향이 일치한다.
- 거리값, `frame_id`, timestamp가 정상이다.
- 연결을 끊었을 때 노드가 오래된 스캔을 정상 데이터처럼 계속 발행하지
  않는다.

### 5. VESC 단독 검증

바퀴를 띄우고 비상 정지를 준비한 상태에서 VESC USB를 연결한다. 새로
나타난 시리얼 장치의 VID, PID, serial을 확인한 후에만 `/dev/ttyMOTOR`
udev 규칙을 만든다.

VESC 기본 조건:

- port: `/dev/ttyMOTOR`
- baud rate: `115200`
- expected firmware: `2.18`
- command watchdog: `0.5 s`
- `drive_enabled=false`
- acceleration slew enabled

처음에는 반드시 출력 비활성 상태로 실행한다.

```bash
ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py drive_enabled:=false
```

```bash
ros2 topic hz /vehicle/vesc_state
```

```bash
ros2 topic echo /diagnostics
```

통과 조건:

- firmware와 telemetry가 정상적으로 확인된다.
- 배터리 전압이 정상 범위이고 VESC fault가 없다.
- `drive_enabled=false`에서 비영점 모터 출력이 절대로 적용되지 않는다.
- 통신이 끊기면 watchdog이 정지 상태를 유지한다.

### 6. 전체 스택 Shadow 검증

세 장치의 단독 검증이 끝난 뒤 기본 대회 명령을 실행한다. 기본값에서는
인지·판단은 실행되지만 실제 모터 출력은 이중으로 차단된다.

```bash
ros2 launch xycar_final_drive final.launch.py
```

다른 터미널에서 한 항목씩 확인한다.

```bash
ros2 topic info /xycar_motor -v
```

```bash
ros2 topic echo /xycar_motor_shadow
```

```bash
ros2 topic hz /perception/canonical_road_image
```

```bash
ros2 topic hz /scan
```

```bash
ros2 topic hz /vehicle/vesc_state
```

통과 조건:

- LR-ASPP 인지와 하이브리드 정책 로그가 모두 `device=cuda`다.
- 카메라는 약 30 Hz, LiDAR는 약 10 Hz다.
- canonical 인지는 현재 구성에서 약 12~15 Hz를 유지한다.
- `/xycar_motor`의 최종 명령 publisher가 둘 이상 존재하지 않는다.
- 조향 후보의 좌우 부호가 차량 기준과 일치한다.
- 센서 timestamp가 오래되면 유효한 주행 명령을 계속 내보내지 않는다.

### 7. 바퀴를 띄운 구동 검증

먼저 최종 명령 publisher만 켜고 VESC 출력은 계속 막는다.

```bash
ros2 launch xycar_final_drive final.launch.py command_drive_enabled:=true vesc_drive_enabled:=false
```

이 상태에서 `/xycar_motor`의 publisher 수, 속도 제한, 조향 방향, 조향 영점을
검증한다. 모든 항목이 통과한 뒤에만 바퀴를 띄운 상태에서 두 번째 게이트를
연다.

```bash
ros2 launch xycar_final_drive final.launch.py command_drive_enabled:=true vesc_drive_enabled:=true
```

확인 항목:

- 비상 정지와 전원 차단이 즉시 작동한다.
- 전진 명령에서 바퀴 회전 방향이 정확하다.
- 좌·우 조향 방향과 `steering_center_trim_command`가 정확하다.
- 명령이 갑자기 바뀌어도 acceleration slew가 적용된다.
- 명령 publisher 또는 센서 노드를 정지했을 때 0.5초 watchdog으로 출력이
  0이 된다.
- VESC fault, 저전압 또는 telemetry 단절 시 비영점 출력을 차단한다.

### 8. 즉시 중단해야 하는 조건

다음 중 하나라도 발생하면 두 안전 게이트를 열지 않는다.

- 장치 VID, PID 또는 serial을 식별하지 못했다.
- 임시 장치명과 `/dev/ttyLIDAR`, `/dev/ttyMOTOR`의 연결 대상이 불일치한다.
- 권한 오류를 `sudo` 실행으로 임시 우회해야 한다.
- CUDA 초기화가 실패하거나 CPU로 의도치 않게 바뀐다.
- 카메라 보정, LiDAR 방향 또는 조향 부호가 맞지 않는다.
- `/xycar_motor`에 최종 publisher가 둘 이상 있다.
- VESC firmware, 전압, telemetry 또는 fault 상태가 예상과 다르다.
- 센서가 끊겼는데 속도 0이 보장되지 않는다.
- 차량을 손으로 제어할 비상 정지 담당자가 없다.

### 9. 현장에서 기록할 장치 정보

| 장치 | 실제 포트 | 영구 이름 | VID | PID | serial | 결과 |
|---|---|---|---|---|---|---|
| 카메라 | 미확인 | `/dev/v4l/by-id/...` | 미확인 | 미확인 | 미확인 | 대기 |
| LiDAR | 미확인 | `/dev/ttyLIDAR` | 미확인 | 미확인 | 미확인 | 대기 |
| VESC | 미확인 | `/dev/ttyMOTOR` | 미확인 | 미확인 | 미확인 | 대기 |

udev 규칙, 그룹 권한 또는 설정 파일을 변경할 때는 실제 장치 정보와 변경
내용, 원상 복구 방법을 먼저 확인하고 적용한다.
