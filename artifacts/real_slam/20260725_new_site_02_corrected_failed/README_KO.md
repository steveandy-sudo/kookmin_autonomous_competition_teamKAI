# 2026-07-25 임시 SLAM 지도

이 디렉터리는 `new_site_02`의 정합을 복구하려다 실패한 임시 결과를
재현·분석하기 위해 보존한 것이다.

**이 지도는 localization 또는 실제 주행에 사용하지 않는다.**

## 내용

| 파일 | 용도 |
|---|---|
| `map.yaml` | occupancy map 메타데이터 |
| `map.pgm` | occupancy grid |
| `map_preview.png` | 빠른 육안 확인용 미리보기 |
| `map.posegraph.gz` | 압축된 slam_toolbox pose graph |
| `map.data.gz` | 압축된 slam_toolbox serialized data |
| `SHA256SUMS` | 저장소 파일 체크섬 |

GitHub 저장소 크기를 줄이기 위해 `map.posegraph`와 `map.data`만 gzip으로
압축했다. 원본 파일로 복원하려면 별도 작업 디렉터리에서 실행한다.

```bash
cp -a artifacts/real_slam/20260725_new_site_02_corrected_failed \
  /tmp/new_site_02_corrected_failed

cd /tmp/new_site_02_corrected_failed
gzip -dk map.posegraph.gz
gzip -dk map.data.gz
```

복원된 원본 파일의 SHA-256은 다음과 같다.

```text
map.posegraph  7b0bc46e79bf12be4b8168ba897b271cca265eb5f0c34cf2d9516679643cb6c8
map.data       7132583e9687f7f1d02bb3b475ac786959dd0dabf91310c9b0657d2e60d41907
map.pgm        a1213efbf6cec59e5cd6c73ae251bf7ccec330101cca12438517be64d3461f7e
map.yaml       4c31422c412f3861d7ba4045cb85be1b502baae9fe6860438adfa65e74b275f6
map_preview    9760bc54abfe2c334c2760d55013f94d678c8b553f4d93701eda2dea6c70ec8a
```

실패 원인과 원본 지도 보존 상태는
[`docs/real_slam_20260725_findings_KO.md`](../../../docs/real_slam_20260725_findings_KO.md)
에서 확인한다.
