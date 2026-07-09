# IL Troubleshooting

## xycar_msgs missing

증상:

```text
Unknown package 'xycar_msgs'
ModuleNotFoundError: No module named 'xycar_msgs'
```

확인:

```bash
ros2 interface show xycar_msgs/msg/XycarMotor
```

해결:

- 기본값은 `motor_msg_type:=float32_multi_array`입니다.
- 사용하는 주행 코드가 `xycar_msgs/msg/XycarMotor`를 publish할 때만 `xycar_msgs`가 필요합니다.

## `/il/mission_label`이 publish되지 않음

증상:

```bash
ros2 topic echo /il/mission_label
```

가 아무것도 출력하지 않습니다.

해결:

고정 label을 publish합니다.

```bash
ros2 topic pub /il/mission_label std_msgs/msg/String "{data: general_drive}" --rate 5
```

## `samples.csv`를 못 찾음

해결:

```bash
find /tmp/il_test -name "samples.csv"
find ~/xycar_ws/datasets/il -name "samples.csv"
```

`SAMPLES` 변수가 비어 있는 상태로 `head "$SAMPLES"`를 실행하지 마세요.

```bash
SAMPLES=$(find /tmp/il_test -name "samples.csv" | tail -n 1)
echo "$SAMPLES"
head -5 "$SAMPLES"
```

## `val.csv` 또는 `test.csv`가 비어 있음

원인:

- session이 하나뿐일 수 있습니다.
- builder split은 random frame 기반이 아니라 session 기반입니다.

해결:

- 여러 session을 수집하세요.
- 같은 구간이라도 시간, 조명, 주행 안정성, recovery 상황을 나눠 여러 session으로 만드세요.

## 이미지가 저장되지 않음

확인:

```bash
ros2 topic list
ros2 topic info /image_raw -v
```

해결:

- recorder의 `camera_front_topic`이 실제 카메라 topic과 같은지 확인합니다.
- `cv_bridge`와 OpenCV가 설치되어 있는지 확인합니다.
- output directory 권한을 확인합니다.
- recorder log에서 `waiting for: front_image`가 계속 나오는지 봅니다.

## scan rows가 0임

실제 `/scan` 토픽이 publish되고 있는지 확인하세요.

```bash
ros2 topic info /scan -v
```

drive/cone/overtake launch는 기본적으로 `save_scan_npz=true`입니다.

## `pkg_resources DeprecationWarning`

`ros2 run`으로 script를 실행할 때 setuptools wrapper에서 보일 수 있는 non-fatal warning입니다. 실행 자체가 성공하면 당장 막는 문제는 아닙니다.

## `tests_require` warning

과거 `setup.py`에 `tests_require`가 있으면 setuptools 경고가 날 수 있습니다. 현재 코드에서는 제거되어야 합니다. 다시 보이면 `setup.py`가 최신인지 확인하세요.

## `/xycar_motor` publisher가 여러 개임

위험합니다. 최종 차량 제어 publisher는 하나만 있어야 합니다.

확인:

```bash
ros2 topic info /xycar_motor -v
```

해결:

- 중복 motor publisher를 종료합니다.
- `il_data_tools` recorder는 `/xycar_motor`를 publish하지 않습니다.

## recorder가 sample을 저장하지 않음

log에 다음이 반복되는지 확인합니다.

```text
waiting for: front_image, motor, mission_label
```

필요한 입력:

- camera image
- motor command
- mission label

drive recorder는 기본적으로 `idle`과 `bad_data`를 제외합니다. label이 허용 목록에 없으면 sample이 저장되지 않습니다.
