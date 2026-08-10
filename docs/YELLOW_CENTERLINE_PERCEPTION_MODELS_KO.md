# 노란 중앙선 인지 모델 변경 및 통합 주행 비교

## 1. 목적

`agent/yellow-center-curve-test` 브랜치에는 기존 LR-ASPP 차선 분할 모델과
별도로 노란 중앙선을 직접 예측하는 두 후보 모델이 추가되었다.

- Row centerline: `kookmin_yellow_row_centerline_256x144.pt`
- Xbin far centerline: `kookmin_far_centerline_xbin_512x288.pt`

두 모델은 기존 통합 주행의 Pure Pursuit, Stanley, 라바콘, 객체 인지,
LiDAR 회피 및 최종 선택기 로직을 바꾸지 않는다. 모델 출력을 기존과
동일한 canonical 도로 영상으로 변환하여 RULE 제어기에 전달한다.

## 2. 공통 처리 흐름

```text
/wide_camera_mjpeg/image_raw/compressed
  -> 최신 프레임 선택
  -> JPEG 디코딩 및 fisheye 보정
  -> 중앙선 모델 추론
  -> 호환용 white/yellow mask 생성
  -> 실차 BEV 변환
  -> /perception/canonical_road_image
  -> Pure Pursuit + Stanley
  -> /hybrid/rule_candidate
  -> 콘/회피/선택기
```

모델 비교 시 최종 속도는 단순 신경망 추론 횟수가 아니라
`/hybrid/rule_candidate`에 새 조향 판단이 발행되는 빈도로 측정했다.

## 3. 기존 LR-ASPP와의 차이

### 기존 LR-ASPP

- 입력: RGB `256x144`
- 출력: 배경, 흰 경계선, 노란 중앙선의 픽셀 단위 semantic segmentation
- 흰선과 노란선을 모두 출력한다.
- 기본 canonical 전방 범위는 1.5m이다.
- 흰선 sliding-window fitting 및 노란선 기준 좌우 분리 기능을 사용할 수
  있다.

### 새 중앙선 모델의 공통점

- 노란 중앙선 위치를 직접 예측한다.
- 런타임 호환 mask의 흰선은 비어 있고 노란 중앙선만 그린다.
- 기존 canonical 색상과 토픽 형식을 유지하므로 RULE 제어기를 그대로
  사용할 수 있다.
- 노란선이 보이지 않을 때 LR-ASPP의 흰선 기반 fallback을 기대할 수 없다.
- 흰선은 학습 target 확장에 사용될 수 있지만 런타임 입력이나 출력에는
  포함되지 않는다.

## 4. Row centerline 모델

### 구조

- 입력: `256x144`
- 48개의 비균일 image row에서 정규화된 중앙선 x 좌표와 visibility를
  예측한다.
- 32개 anchor는 먼 영역, 16개 anchor는 가까운 영역을 담당한다.
- visibility가 0.50 이상인 좌표만 호환용 노란 mask에 그린다.
- 학습 label 내부의 노란 점선 간격은 연결하지만 첫 label 위나 마지막
  label 아래로 임의 외삽하지 않는다.
- canonical 전방 범위: 1.5m

### 장점

- 입력이 작고 출력 구조가 단순해 응답 속도가 빠르다.
- 전체 통합 주행에서 설정된 15Hz 제한을 모두 채웠다.
- 고속 주행에서 조향 판단 갱신 간격을 짧게 유지하기 유리하다.

### 한계

- 한 feature 오류가 여러 row 좌표에 동시에 영향을 줄 가능성이 있다.
- 노란 중앙선이 짧거나 visibility가 낮으면 출력점이 빠르게 줄어든다.
- 흰 경계선을 직접 제공하지 않는다.

## 5. Xbin far centerline 모델

### 구조

- 입력: `512x288`
- 72개 image row마다 128개의 x-bin 또는 `no line`을 분류한다.
- 공간 feature를 입력의 1/4 해상도로 유지한다.
- 파라미터 수: 200,794
- held-out 기준 line-presence recall 98.9%, visible row 4px 이내 92.2%,
  평균 visible-row 오차 1.91px이다.
- 확장 프로필의 canonical 전방 범위: 2.5m

### 2.5m BEV 방식

실측된 1.5m homography 자체를 새로 계산하지 않는다. 기존 projective
plane을 높이 1100px BEV로 아래 방향 이동하여 2.5m까지 평면 연장한다.
따라서 먼 곡선을 먼저 볼 수 있지만 평평한 노면 가정이 맞는지 RViz와
shadow 주행으로 확인해야 한다.

### 장점

- 각 row를 독립적인 x-bin 분류로 처리해 한 feature 오류가 전체 경로를
  자유롭게 이동시키는 현상을 줄인다.
- 2.5m 선행 중앙선을 이용해 S자 반대 곡선을 더 일찍 확인할 수 있다.

### 한계

- 큰 입력과 확장 BEV 처리 때문에 Row 모델보다 최종 조향 Hz가 낮다.
- 2.5m 영역은 실측 캘리브레이션이 아니라 평면 연장 결과다.
- 흰 경계선을 직접 제공하지 않는다.

## 6. 실측 성능 비교

### 공통 조건

- PC: 실차 Xycar Mini PC CPU
- ROS 2: Humble
- 입력 rosbag: `lane_model_compare_20260807_141414`
- 입력 카메라: 약 29.82Hz MJPEG
- Torch CPU threads: 4
- 모터 발행 및 RViz: 비활성화

### 단독 perception + RULE 제어기

출력 제한을 30Hz로 열고 최종 `/hybrid/rule_candidate`를 측정했다.

| 모델 | 전방 범위 | 최종 조향 판단 |
| --- | ---: | ---: |
| Row centerline | 1.5m | 약 20.7Hz |
| Xbin far centerline | 2.5m | 약 11.9Hz |

### 전체 통합 주행 shadow

라바콘, 객체 YOLO, LiDAR 회피 및 최종 선택기를 모두 실행하고 통합 주행의
인지 제한을 15Hz로 유지했다. 입력은 `rule_tuning_220260807_134142` bag의
압축 카메라와 `/scan`만 재생했다.

| 모델 | 최종 조향 판단 | 선택기 상태 | 오류 |
| --- | ---: | --- | --- |
| Row centerline | 15.0Hz | `RUNNING / RULE` | 없음 |
| Xbin far centerline | 약 9.8Hz | `RUNNING / RULE` | 없음 |

Row 모델은 통합 제한 15Hz를 모두 사용했다. Xbin 확장형은 전체 스택에서
약 9.8Hz로 낮아졌다. 하지만 현재 실차 속도에서는 Xbin의 추가 선행 인지
거리가 프레임 간 이동 거리 증가보다 충분히 컸으므로, 통합 주행 기본값은
Xbin 2.5m로 변경했다.

## 7. 통합 주행 모델 선택

통합 실행 스크립트는 다음 환경변수를 지원한다.

- `XYCAR_LANE_PERCEPTION_LAUNCH`: 사용할 perception launch
- `XYCAR_CANONICAL_FORWARD_RANGE_M`: canonical 전방 범위 수동 지정
- `XYCAR_PERCEPTION_MAX_OUTPUT_RATE_HZ`: perception 출력 상한

선택된 launch, 전방 범위, 최대 Hz는 시작 터미널과
`/tmp/xycar_hybrid_run_config.yaml`에 기록된다.

### 기본값: Xbin 2.5m 모델

```bash
cd /home/xytron/kookmin_ty/yellow_center_curve_test/xycar_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

export XYCAR_RULE_PERCEPTION_BACKEND=canonical
export XYCAR_PERCEPTION_MAX_OUTPUT_RATE_HZ=15.0
export XYCAR_ENABLE_RVIZ=true

bash src/xycar_map_nav/scripts/run_complete_space_hybrid.sh
```

모델 관련 환경변수를 생략하면 Xbin 확장 launch와 전방 범위 2.5m가 자동
적용된다.

### Row 1.5m 비교 실행

```bash
export XYCAR_LANE_PERCEPTION_LAUNCH=lane_seg_row_centerline_low_latency_real.launch.py
```

Row launch를 선택하면 전방 범위 1.5m가 자동 적용된다. 나머지 명령은
기본 Xbin 실행과 동일하다.

## 8. 검증 결과와 실차 시험 순서

- Row/Xbin 모델 단위 테스트: 10개 통과
- 통합 스크립트 관련 테스트: 10개 통과
- `xycar_map_nav`, `lane_seg_control`, `xycar_rule_drive` 빌드 성공
- 두 통합 shadow 실행에서 launch 및 추론 오류 없음

실차에서는 다음 순서를 사용한다.

1. Xbin 모델, 속도 0, RViz로 노란 canonical과 조향 방향 확인
2. Xbin 모델, 속도 6~8로 S자 진입 및 반대 조향 시점 확인
3. 필요하면 동일 위치와 속도에서 Row 1.5m를 비교
4. 먼 중앙선이 안정적인 것을 확인한 뒤 저속 실제 주행
5. 모델별 rosbag을 별도로 저장해 조향 전환 시점과 경로 유효율 비교

현재 측정은 rosbag shadow 검증이다. 실제 바퀴를 구동한 결과는 아니므로
최종 모델 선택은 같은 트랙, 같은 속도에서 진행한 실차 비교로 확정한다.
