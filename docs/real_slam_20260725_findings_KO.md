# 2026-07-25 실차 SLAM 진단 결과

## 결론

`new_site_02` pose graph는 차량이 실제 시작 장소로 돌아왔지만 SLAM상의
마지막 노드가 시작 노드에서 `12.57 m` 떨어져 있었다. 이 상태에서 시작 노드
또는 마지막 추정 좌표로 이어 매핑하는 방법은 기존 오차를 닫지 못했고,
동일 공간을 새 공간처럼 추가해 지도를 악화시켰다.

다음 파일은 사용하지 않는다.

```text
~/xycar_maps/new_site_02_corrected/
```

원본은 덮어쓰지 않았으며 다음 위치에 보존되어 있다.

```text
~/xycar_maps/new_site_02/
```

## 원본 pose graph 측정

| 항목 | 값 |
|---|---:|
| 노드 수 | 2,391 |
| 첫 노드 | `(0.0000, 0.0000) m` |
| 마지막 노드 | `(1.1535, 12.5180) m` |
| 시작-종료 거리 | `12.5711 m` |
| 마지막 진행 방향 추정 | 약 `-0.107 rad` |

원본 파일:

| 파일 | 크기 | SHA-256 |
|---|---:|---|
| `map.pgm` | 717,308 bytes | `62b8768597e51210089a8f1cb24bce624649eeb0c2291ecdb1cacaba24bf1696` |
| `map.posegraph` | 47,844,546 bytes | `027d27c455d53e00ce5c8718c1edcac20ed23a3b6409c766520999d159266013` |
| `map.data` | 37,622,396 bytes | `a08fe7cb7f2aadad671e7204fa4020d126462e025666313376dc9b45bb792189` |

## 실패한 복구 방법

### 첫 노드에서 이어 매핑

`map_start_at_dock:=true`로 현재 스캔을 첫 노드에 붙인 뒤 같은 구간을 다시
주행했다. 기존 마지막 노드의 누적 오차는 남은 채 새 주행이 별도 공간처럼
추가되어 중복 벽과 새로운 빈 공간이 생겼다.

### 마지막 추정 좌표에서 이어 매핑

다음 값으로 재개했다.

```text
map_start_pose = [1.1535, 12.5180, -0.107]
loop_search_maximum_distance = 15.0
```

현재 스캔이 실제 시작점이 아니라 pose graph의 오차가 난 끝부분에 붙었다.
동일 장소를 반복 주행해도 정합되지 않았고 지도가 계속 변형됐다. 큰
`loop_search_maximum_distance`는 형태가 비슷한 긴 복도에서 잘못된 후보를
늘릴 위험도 있다.

### 단일 노드 수동 이동

마지막 노드 하나를 시작 좌표로 이동해 수동 최적화를 호출했지만 새로운
loop constraint가 추가되지 않아 기존 순차 제약이 노드를 원래 오차 위치로
되돌렸다. 큰 누적 오차는 단일 노드 이동으로 해결되지 않았다.

## 가장 가능성 높은 원인

- 실차 엔코더 odom이 없어 모터 명령으로 병진 거리를 적분했다.
- 바닥 부하, 배터리, 미끄러짐을 command odom이 측정하지 못했다.
- 긴 직선 복도의 LiDAR 형상이 반복되어 scan matching이 진행 방향 오차를
  강하게 구속하지 못했다.
- 시작점으로 돌아왔을 때 pose graph 오차가 기본 loop 검색 거리 `5 m`보다
  커져 첫 노드가 loop 후보에 포함되지 않았다.

IMU 상대 yaw는 회전 오차를 줄이지만 병진 누적 오차를 직접 보정하지 못한다.

## 확보된 재처리 데이터

복구 시험은 다음 rosbag에 저장됐다.

```text
~/xycar_test_bags/new_site_02_recovery_20260725_175347/
```

| 항목 | 값 |
|---|---:|
| 지속 시간 | 183.068 s |
| 전체 크기 | 286.8 MiB |
| `/scan` | 1,769 messages |
| `/imu` | 6,543 messages |
| `/xycar_motor` | 3,662 messages |
| `/slam/odom` | 9,154 messages |
| `/tf` | 18,261 messages |
| `/map` | 353 messages |

기존 잘못된 `/map`, `/slam/odom`, `/tf`를 재사용하지 않고 `/scan`, `/imu`,
`/xycar_motor`만 격리된 ROS 도메인에서 재생해 새 지도를 처음부터 만들어야
한다. 재처리 결과를 검증하기 전에는 navigation용 지도로 채택하지 않는다.

## 다음 작업

1. `ROS_DOMAIN_ID=77`처럼 실차와 격리된 도메인에서 bag을 재생한다.
2. 기존 pose graph를 로드하지 않고 새 mapping 세션을 시작한다.
3. 원시 `/scan`, `/imu`, `/xycar_motor`만 사용해 odom과 지도를 다시 계산한다.
4. 시작-종료 오차, 이중 벽 두께, 복도 폭을 수치로 비교한다.
5. 결과가 불량하면 엔코더 odom을 추가한 뒤 새 rosbag을 기록한다.
6. 검증 전에는 `new_site_02_corrected`를 덮어쓰거나 주행에 사용하지 않는다.

큰 rosbag은 Git에 직접 넣지 않는다. 실패한 임시 지도는 재현과 비교를 위해
다음 디렉터리에 보존했다.

```text
artifacts/real_slam/20260725_new_site_02_corrected_failed/
```

`map.posegraph`와 `map.data`는 저장소 크기를 줄이기 위해 gzip으로 압축했다.
이 결과는 진단 전용이며 localization 또는 실제 주행에 사용하지 않는다.
