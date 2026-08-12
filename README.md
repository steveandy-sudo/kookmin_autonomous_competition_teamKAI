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
