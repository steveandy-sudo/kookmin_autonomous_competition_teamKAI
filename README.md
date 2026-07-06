# Kookmin Xycar Sim-to-Real Track

Gazebo Sim에서 국민대학교 Xycar 자율주행 트랙을 최대한 비슷하게 재현하고,
실차 Xycar 주행 코드와 맞춰 rule-based 주행, 데이터 수집, 모방학습, Offline RL까지 이어가기 위한 작업 저장소입니다.

현재 저장소의 1차 목표는 **트랙 맵 안정화와 실차 조건 정리**입니다.

## 현재 상태

- 최종 Gazebo 월드: `worlds/kookmin_xycar_track_final.sdf`
- Gazebo에서 다시 저장한 작업본: `worlds/kookmin_xycar_track_gz.sdf`
- 흰색 도로 경계선: 개별 Gazebo 객체로 분리되어 Entity Tree에서 이동/조정 가능
- 노란색 중앙 점선: 개별 Gazebo 객체로 분리되어 이동/조정 가능
- 차량: `xycar_ackermann` 모델이 월드 안에 미리 스폰됨
- 실차 코드 참고본: `xycar_ws/src`
- 실차-시뮬레이션 보정 문서: `docs/sim_to_real_vehicle_calibration.md`

## 실행

저장소 루트에서 실행합니다.

```bash
export GZ_SIM_RESOURCE_PATH=$PWD:${GZ_SIM_RESOURCE_PATH}
gz sim worlds/kookmin_xycar_track_final.sdf
```

Gazebo GUI가 안 뜨거나 빈 화면이면 기존 `gz sim` 프로세스가 남아 있는지 먼저 확인합니다.

```bash
pkill -f "gz sim" || true
export GZ_SIM_RESOURCE_PATH=$PWD:${GZ_SIM_RESOURCE_PATH}
gz sim worlds/kookmin_xycar_track_final.sdf
```

`libEGL warning: egl: failed to create dri2 screen` 경고는 그래픽 드라이버/렌더링 경고입니다.
창이 열리고 월드가 보이면 이 경고 자체는 치명적이지 않습니다.

## 맵 기준

트랙은 CAD/DXF에서 가져온 차선 데이터를 바탕으로 만들었습니다.
사용자가 Gazebo에서 직접 조정한 최종 위치를 보존하기 위해 `final.sdf`를 기준 파일로 둡니다.

- 도로 바닥: 실제 도로처럼 회색 톤
- 흰색 경계선 두께: `0.024 m`
- 노란색 중앙선 두께: `0.024 m`
- 노란색 중앙 점선: `0.30 m` 길이, `0.30 m` 간격 기준
- 흰색 경계선 안쪽 기준 도로 폭: 최종 저장본 기준 대략 `0.76~0.84 m`
- 전체 기준 크기: 설계도 외곽 `20.150 m x 11.350 m`를 기준으로 정합
- S자 곡선: CAD에서 추출한 흰색 라인의 형태를 유지한 뒤 Gazebo 객체로 분리

주의: `scripts/generate_kookmin_track.py`를 다시 실행하면 Gazebo에서 수동으로 저장한 차선 위치가 덮어써질 수 있습니다.
최종 맵을 수정할 때는 먼저 `worlds/kookmin_xycar_track_final.sdf`를 백업한 뒤 진행합니다.

## 차량 모델

월드에는 `xycar_ackermann`이 포함되어 있습니다.
현재 값은 실차를 완전히 보정한 값이 아니라 rule-based 주행과 센서 파이프라인을 먼저 붙이기 위한 시작점입니다.

- 차체 크기: 약 `0.52 m x 0.20 m x 0.12 m`
- 바퀴 포함 외폭: 약 `0.26 m`
- wheelbase: `0.32 m`
- wheel separation: `0.225 m`
- wheel radius: `0.06 m`
- steering limit: `0.5 rad`
- max velocity: `1.5 m/s`

실차와 맞추기 위한 측정 항목과 ROS2 확인 명령은
`docs/sim_to_real_vehicle_calibration.md`에 정리되어 있습니다.

## 실차 코드에서 확인한 ROS2 기준

`xycar_ws/src`에 복사된 실차 주행 코드를 기준으로 정리한 내용입니다.

- 카메라 토픽: `/image_raw`
- 라이다 토픽: `/scan`
- 초음파 토픽: `xycar_ultrasonic`
- 모터 토픽 후보: `/xycar_motor` 또는 `xycar_motor`
- 현재 예제 코드 대부분의 모터 메시지: `std_msgs/msg/Float32MultiArray`
- 모터 데이터 순서: `[angle, speed]`
- 일부 예전/시뮬 코드의 모터 메시지: `xycar_msgs/msg/XycarMotor`

실차에서 제일 먼저 확인할 명령입니다.

```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash

ros2 topic list -t
ros2 topic info /image_raw
ros2 topic info /scan
ros2 topic info /xycar_motor
ros2 topic info xycar_motor
```

## 주요 파일 구조

```text
.
├── worlds/
│   ├── kookmin_xycar_track_final.sdf      # 최종 Gazebo Sim 월드
│   └── kookmin_xycar_track_gz.sdf         # Gazebo 저장 작업본
├── media/materials/textures/              # 트랙/차선 텍스처
├── scripts/
│   ├── generate_kookmin_track.py          # 맵 생성 스크립트
│   └── blueprint_alignment_report.py      # 설계도 정합 검증 스크립트
├── docs/
│   └── sim_to_real_vehicle_calibration.md # 실차-시뮬레이션 보정 문서
├── xycar_ws/src/                          # 실차 ROS2 주행 코드 참고본
├── preview_topdown.png                    # 트랙 미리보기
└── alignment/fit_report.txt               # 설계도/생성맵 정합 리포트
```

## 다음 단계

1. 실차에서 모터 토픽 타입과 `[angle, speed]` 스케일 확인
2. 시뮬레이션 차량의 wheelbase, 조향 한계, 속도 스케일을 실차에 맞게 보정
3. Gazebo 카메라와 라이다 위치를 실차 장착 위치에 맞게 조정
4. rule-based 차선 주행 노드 작성
5. 주행 로그와 이미지/라이다 데이터를 수집
6. Behavioral Cloning 학습 후 Offline RL과 supervisor 구조로 확장
