# 반복 문 복도 SLAM 점프 진단과 운용 방법

## 사용 데이터

```text
/home/xytron/xycar_test_bags/
  gyro_slam_20260726_171622_recovered_20260726_1730
  gyro_slam_20260726_175337
```

첫 bag은 630.28초, 546.6 MiB, 173,702개 메시지이며 SQLite 무결성 검사와
재생 시험을 통과했다. 최신 bag은 400.72초이며 `/scan` 9.66 Hz, `/imu`
35.74 Hz, `/slam/odom` 48.71 Hz, VESC telemetry 48.80 Hz로 센서 주기
손실은 없었다.

## 원인

첫 bag의 `map -> slam_odom`은 한 번에 최대 6.72 m와 10.2도 변했다. 최신
bag도 196~227초에 보정이 집중됐고 최대 순간 이동은 3.01 m였다.

최신 bag의 LiDAR 스캔 시퀀스를 비교하면 문제 구간은 50~142초의 여러 위치와
동시에 `0.8~0.95`의 높은 유사도를 보인다. 반복되는 20 cm 문 홈은 단일
스캔이나 짧은 구간만으로 절대 위치를 구별할 수 없다. Karto correlation
matcher가 현재 문을 앞뒤의 다른 문으로 연결하면서 새 각도와 공간을 만들었다.

## 동일 bag 재처리

| 처리 방식 | 최대 순간 map 보정 | 175~235초 큰 보정 |
|---|---:|---:|
| 설치돼 있던 strict matcher | 3.01 m | 266개 TF 샘플 |
| 20 cm 간격 bounded matcher | 2.54 m | 107개 TF 샘플 |
| VESC+IMU odom guarded | 0.00 m | 0개 |

matcher 탐색 폭과 응답 임계값만 조정해서는 2 m 이상의 오인식을 제거하지
못했다. 아래 역할 분리는 진단 실험으로 사용했다.

- VESC tachometer: 종방향 이동량
- IMU gyro Z: 회전량
- LiDAR: occupancy map과 pose graph scan
- 반복 복도: VESC+IMU odom만 사용한 guarded 재처리
- 고유한 모서리/시작 표식: loop closure 후보를 구별하기 위한 기준

이 guarded 방식은 지도 점프는 막았지만 LiDAR의 정상적인 횡방향·각도 보정까지
사라져 실제 지도 형상이 더 나빠졌다. 따라서 production
`real_mapping.launch.py`는 다시 공식 Karto tuning의
`slam_toolbox_mapping.yaml`을 기본으로 사용한다. 실험용 bounded/global
matcher와 RF2O map은 기본 launch에 포함하지 않는다.

## 저전압 정지

최신 bag에서 VESC `fault_code=2 (UNDER_VOLTAGE)`는 257.77~260.74초에
발생했다. 기존 드라이버는 이후 출력을 계속 latch했고, odom 속도도 약
258초부터 132초 동안 0이었다. 최초 정합 오류는 196초에 시작했으므로
저전압 정지와 map 오정합은 별개다.

드라이버는 이제 `UNDER_VOLTAGE` fault가 사라진 뒤 입력 전압이 8.0 V
이상으로 연속 3초 유지될 때 자동 복구한다. 다른 VESC fault는 계속 수동
점검과 clear가 필요하다.

## 실차 폐합 절차

기존 망가진 pose graph를 이어서 쓰지 않는다. 빈 맵과 새 bag으로 시작하고,
반복 복도에서는 속도 명령 2~3을 사용한다. 첫 바퀴에서 기존 벽과 현재
LaserScan이 겹치는지 확인한 뒤 같은 방향으로 2~3바퀴 재관측한다. 반복 통과는
고유한 모서리와 폭 변화 구간을 포함해야 하며, 이중 벽이나 0.20 m 이상의
위치 점프가 생기면 즉시 중단한다.

한 바퀴 뒤 고유한 시작 모서리에 같은 방향으로 정확히 정지한다. 자동
loop closure가 성공하면 시작 부근 벽과 스캔이 한 줄로 겹치고 다음 반복
주행에서도 같은 모양을 유지해야 한다.

반복되는 문 하나만 기준으로 폐합하면 안 된다. 시작점에는 LiDAR에서 보이는
비대칭 L자 표식이나 기존 고유 모서리가 필요하다. 800개 이상 노드가 있는
그래프에서 단일 마지막 노드를 억지로 옮긴 manual closure는 최적화 정지와
잘못된 공간 생성을 유발했으므로 production 복구 절차로 사용하지 않는다.
