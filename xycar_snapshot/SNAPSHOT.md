# Xycar reference snapshot

이 디렉터리는 기존 `state_machine` 구현에 바로 병합하기 위한 코드가 아니라,
실차 콘 제어 계약과 LiDAR `/scan` 계약을 확인하기 위한 읽기 전용 참고 스냅샷이다.

## Snapshot metadata

- 복사 날짜: `2026-07-23T14:38:44+09:00`
- 대상 Git 저장소: `https://github.com/steveandy-sudo/kookmin_autonomous_competition_teamKAI`
- 대상 브랜치: `xycar_reference`
- 대상 기준 브랜치: `main`
- 대상 기준 commit SHA: `cce47d5c731a5fb83f4859e5b22c597268a4d9d2`
- ROS 2 배포판: `Humble` (`/opt/ros/humble` 설치 확인, 복사 셸의 `ROS_DISTRO`는 unset)
- ROS_DOMAIN_ID: `unset` (별도 설정이 없다면 ROS 2 기본값 `0`)
- ROS_NAMESPACE: `unset`

## Original source provenance

### my_rule

- 원본 경로: `/home/xytron/xycar_ws/src/study/my_rule`
- Git 저장소: 확인 불가 — 원본 디렉터리와 상위 `xycar_ws`가 Git work tree가 아님
- 브랜치: `N/A`
- commit SHA: `N/A`
- 원본 저장소 dirty 여부: 확인 불가
- `git status --short` 파일명 목록: `N/A`

### xycar_lidar

- 원본 경로: `/home/xytron/xycar_ws/src/xycar_device/xycar_lidar`
- Git 저장소: 확인 불가 — 드라이버 패키지와 상위 `xycar_ws`가 Git work tree가 아님
- 브랜치: `N/A`
- commit SHA: `N/A`
- 원본 저장소 dirty 여부: 확인 불가
- `git status --short` 파일명 목록: `N/A`
- 참고: 패키지 아래 `YDLidar-SDK/`는 별도의 중첩 Git 저장소지만 이번 요청 범위에서 제외했다.

## Vehicle cone launch command

실행 중인 `my_rule` 프로세스와 셸 기록에서는 실제 사용 명령을 확인하지 못했다.
패키지의 `docs/vehicle_test_readiness.md`에 기록된 실차용 명령은 다음과 같다.

```bash
ros2 launch my_rule my_rule_vehicle.launch.py \
  start_yolo:=true model:=/절대/경로/best.pt
```

따라서 위 명령은 **문서 기준이며 런타임 확인 값은 아니다**.

## Copied scope

```text
xycar_snapshot/
├── SNAPSHOT.md
├── my_rule/
│   ├── cone_node.py
│   ├── rule_driver.py
│   ├── rule_params.yaml
│   ├── launch/
│   ├── setup.py
│   └── package.xml
└── xycar_lidar/
    ├── src/
    ├── launch/
    ├── params/
    ├── package.xml
    └── CMakeLists.txt
```

원본 `xycar_lidar` 패키지는 설정 디렉터리 이름으로 `config/`가 아니라
`params/`를 사용하므로 원래 이름을 보존했다.

## Integration warning

- `rule_driver.py`는 계약 확인용 참고 파일이다.
- 기존 motor publisher를 포함하므로 현재 Mission Manager/Final Driver 구조에 그대로 사용하지 않는다.
- `cone_node.py`의 `/my_rule/cone_cmd` 출력 계약과 confidence/stale/진입·이탈 조건만 추출하여 통합한다.
- 이 브랜치는 참고용으로 유지하고 `state_machine` 브랜치에 통째로 병합하지 않는다.
- 복사 과정에서 원본 코드, 설정, Git 상태, ROS 노드 및 하드웨어는 변경하지 않았다.
