# VESC tachometer 5 m 실측 보정 (2026-07-26)

## 적용값

5.00 m 직선 주행 5회의 tachometer 증가량 중앙값을 사용했다.

```yaml
meters_per_tachometer_count: 0.002527806
```

기존 임시값 `0.002167316 m/count`보다 `16.63%` 큰 값이다.

## 측정 결과

| Run | Tachometer counts | m/count | 시간 (s) | 최소 전압 (V) | 최대 motor current (A) | IMU yaw 변화 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 01 | 2001 | 0.002498751 | 29.18 | 6.10 | 26.08 | +5.76 deg |
| 02 | 1922 | 0.002601457 | 29.17 | 6.10 | 27.40 | +5.95 deg |
| 03 | 1985 | 0.002518892 | 29.33 | 6.10 | 26.46 | +3.17 deg |
| 04 | 1978 | 0.002527806 | 25.15 | 6.10 | 26.77 | +3.86 deg |
| 05 | 1944 | 0.002572016 | 26.20 | 6.10 | 26.38 | +4.45 deg |

- 증가량 중앙값: `1978 counts`
- 환산값 평균: `0.002543784 m/count`
- 환산값 중앙값: `0.002527806 m/count`
- 표준편차: `0.000041911 m/count`
- 변동계수(CV): `1.65%`
- 범위: `0.002498751` ~ `0.002601457 m/count`
- 모든 측정 구간의 VESC fault code: `0`

## 원본 rosbag

원본은 크기가 커서 저장소에 복사하지 않고 실차 PC에 보존한다.

```text
/home/xytron/xycar_test_bags/5m_calibration_20260726_153331
/home/xytron/xycar_test_bags/5m_calibration_run02to05_20260726_155211
```

첫 번째 bag에 RUN 01의 START/END marker 2개가 있고, 두 번째 bag에
RUN 02~05의 marker 8개가 있다.

## 계산 방법

각 RUN의 START 및 END marker와 가장 가까운 `/vehicle/vesc_state` 메시지에서
누적 `displacement` 차이를 읽고 다음 식으로 계산했다.

```text
meters_per_tachometer_count = 5.00 m / abs(end_count - start_count)
```

왕복 방향에 따른 구동계 유격과 일부 이상값의 영향을 줄이기 위해 평균 대신
5회 중앙값을 production 설정에 반영했다.

## 제한 사항

- 모든 주행에서 부하 시 전압이 `6.10 V`까지 내려갔다. 이는 가속 성능과
  주행 시간에는 영향을 주지만, 실측 거리와 tachometer 누적 count의 직접
  비율은 계산에 사용할 수 있다.
- IMU yaw가 각 구간에서 `3.17`~`5.95 deg` 변해 완전한 직선은 아니었다.
- 배터리 정비 후 같은 5 m 구간을 양방향으로 반복하면 바닥, 경사, 조향
  편향에 따른 체계적 오차를 추가로 확인할 수 있다.

## 적용 위치

```text
xycar_ws/src/xycar_map_nav/config/vesc_imu_odom_real.yaml
xycar_ws/src/xycar_map_nav/xycar_map_nav/vesc_imu_odom_node.py
```
