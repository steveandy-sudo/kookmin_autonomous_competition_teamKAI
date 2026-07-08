# IL-Rulebase Interface 설계 문서

이 문서는 Team K.A.I. ROS2 Humble Xycar 프로젝트에서 imitation learning 정책 코드와 rule-based 미션 매니저/최종 모터 제어 코드 사이의 역할 분리와 호출 인터페이스를 정의한다.

## 핵심 원칙

학습 모델은 조향각만 예측한다.

```text
learned policy output = steering angle only
```

학습 모델이 하지 않는 일:

- 속도 결정
- 정지/출발 결정
- 신호등 판단
- 보행자 정지 판단
- 차량 추월 시작/종료 판단
- 미션 전환 판단
- 긴급 정지 판단
- `/xycar_motor` publish

rule-based 코드가 결정하는 일:

- traffic light stop/go
- pedestrian stop
- vehicle detection trigger
- mission transition
- speed
- emergency stop
- final `/xycar_motor` command

최종 시스템에서 `/xycar_motor`를 publish하는 노드는 반드시 하나만 있어야 한다. IL 코드는 `/xycar_motor`를 publish하지 않고, rule-based 최종 제어 노드가 IL 예측 조향각을 참고해 최종 명령을 만든다.

## 전체 구조

```text
camera image
    |
    v
rule-based mission manager
    |
    | selects policy: drive / cone / overtake
    | decides speed, stop/go, safety, mission transition
    v
ILPolicyManager.predict(...)
    |
    | returns steer_deg only
    v
rule-based final motor controller
    |
    | publishes final XycarMotor(angle, speed)
    v
/xycar_motor
```

## 학습 모델 파일

세 개의 TorchScript 모델을 사용한다.

| 정책 이름 | 모델 파일 | 사용 미션 |
| --- | --- | --- |
| `drive` | `drive_policy_scripted.pt` | `general_drive`, `lane_drive`, `hill_drive`, `shortcut` |
| `cone` | `cone_policy_scripted.pt` | `cone_drive` |
| `overtake` | `overtake_policy_scripted.pt` | `vehicle_overtake` |

활성 policy 선택은 rule-based 미션 매니저가 담당한다.

예:

```text
lane_drive       -> drive
hill_drive       -> drive
shortcut         -> drive
cone_drive       -> cone
vehicle_overtake -> overtake
```

## Python 인터페이스

IL 쪽은 다음 Python 인터페이스를 제공한다.

```python
class ILPolicyManager:
    def load_models(
        self,
        drive_model_path: str | None = None,
        cone_model_path: str | None = None,
        overtake_model_path: str | None = None,
        device: str = "auto",
        image_width: int = 160,
        image_height: int = 90,
        roi_top_ratio: float = 0.45,
        max_steer_deg: float = 100.0,
    ) -> None:
        ...

    def predict(
        self,
        policy_name: str,
        image_bgr,
        phase: float | None = None,
    ) -> tuple[float | None, dict]:
        ...
```

### `load_models(...)`

역할:

- TorchScript 모델을 로드한다.
- 로드 실패 시 전체 시스템을 crash시키지 않는다.
- 어떤 policy가 사용 가능한지 내부 상태로 저장한다.

권장 동작:

```text
모델 로드 성공 -> 해당 policy 사용 가능
모델 파일 없음 -> 해당 policy disabled, warning 저장
모델 로드 실패 -> 해당 policy disabled, warning 저장
```

### `predict(policy_name, image_bgr, phase=None)`

역할:

- rule-based 코드가 선택한 policy에 대해 조향각을 예측한다.
- 반환값은 `steer_deg`와 `debug_info`이다.
- `/xycar_motor`를 publish하지 않는다.

반환 형식:

```python
steer_deg, debug_info = il_manager.predict(
    policy_name="drive",
    image_bgr=image_bgr,
)
```

`steer_deg`:

```text
float | None
```

`debug_info` 예시:

```python
{
    "ok": True,
    "policy_name": "drive",
    "model_loaded": True,
    "warning": None,
    "steer_deg": -3.2,
    "phase": None,
}
```

모델이 없거나 예측할 수 없는 경우:

```python
{
    "ok": False,
    "policy_name": "drive",
    "model_loaded": False,
    "warning": "model_missing",
    "steer_deg": None,
    "phase": None,
}
```

이 경우 `steer_deg`는 `None`이어야 하며, rule-based 코드는 기존 rule-based 조향 또는 안전 정지 로직으로 fallback한다.

## Policy 이름

허용되는 policy 이름:

```text
drive
cone
overtake
```

그 외 이름이 들어오면 crash하지 않고 다음처럼 반환한다.

```python
None, {
    "ok": False,
    "policy_name": policy_name,
    "warning": "unknown_policy",
}
```

## Overtake phase 규칙

`overtake` policy는 `phase`가 필요하다.

```text
phase range: 0.0 ~ 1.0
```

의미:

| phase 범위 | 의미 |
| --- | --- |
| `0.0 ~ 0.2` | 추월 시작, 차선 이탈/회피 시작 |
| `0.2 ~ 0.6` | 추월 진행, 대상 차량 통과 |
| `0.6 ~ 1.0` | 복귀 |

rule-based 미션 매니저가 phase를 계산해서 넘긴다.

예:

```python
elapsed = now_sec - overtake_start_sec
phase = elapsed / overtake_duration_sec
phase = max(0.0, min(1.0, phase))
```

`policy_name == "overtake"`인데 `phase is None`이면 예측하지 않는다.

```python
None, {
    "ok": False,
    "policy_name": "overtake",
    "warning": "phase_required",
    "phase": None,
}
```

phase가 범위를 벗어나면 내부에서 clamp한다.

```text
phase < 0.0 -> 0.0
phase > 1.0 -> 1.0
```

## 이미지 입력 규칙

입력 이미지는 OpenCV 형식의 BGR 이미지이다.

```text
image_bgr: numpy.ndarray, H x W x 3, BGR
```

ILPolicyManager 내부 전처리:

```text
1. 하단 ROI crop
2. 160x90 resize
3. BGR -> RGB
4. 0~1 정규화
5. CHW 변환
6. batch dimension 추가
```

기본값:

```text
image_width: 160
image_height: 90
roi_top_ratio: 0.45
```

모델 학습과 실시간 추론의 전처리는 반드시 같아야 한다.

## Rule-based 코드 예시

아래 예시는 rule-based 미션 매니저가 ILPolicyManager를 사용하는 방식이다.

```python
from team_kai_il.il_policy_manager import ILPolicyManager


class RuleBasedMissionManager:
    def __init__(self):
        self.il_manager = ILPolicyManager()
        self.il_manager.load_models(
            drive_model_path="/home/xytron/xycar_ws/models/il/drive_policy_scripted.pt",
            cone_model_path="/home/xytron/xycar_ws/models/il/cone_policy_scripted.pt",
            overtake_model_path="/home/xytron/xycar_ws/models/il/overtake_policy_scripted.pt",
            device="auto",
            image_width=160,
            image_height=90,
            roi_top_ratio=0.45,
            max_steer_deg=100.0,
        )

    def control_once(self, image_bgr, mission_state, safety_state):
        if safety_state.must_stop:
            return 0.0, 0.0, {"mode": "safety_stop"}

        policy_name = self.select_policy(mission_state)
        speed = self.rule_based_speed(mission_state, safety_state)

        phase = None
        if policy_name == "overtake":
            phase = self.compute_overtake_phase(mission_state)

        steer_deg, debug_info = self.il_manager.predict(
            policy_name=policy_name,
            image_bgr=image_bgr,
            phase=phase,
        )

        if steer_deg is None:
            steer_deg = self.rule_based_steer_fallback(mission_state, safety_state)
            debug_info["fallback"] = "rule_based_steer"

        return steer_deg, speed, debug_info

    def select_policy(self, mission_state):
        if mission_state.name in ("general_drive", "lane_drive", "hill_drive", "shortcut"):
            return "drive"
        if mission_state.name == "cone_drive":
            return "cone"
        if mission_state.name == "vehicle_overtake":
            return "overtake"
        return "drive"
```

최종 `/xycar_motor` publish는 rule-based 최종 제어 노드에서만 수행한다.

```python
msg = XycarMotor()
msg.angle = steer_deg
msg.speed = speed
motor_pub.publish(msg)
```

ILPolicyManager 내부에서는 위 publish를 하지 않는다.

## 선택적 ROS debug topics

IL 또는 rule-based 쪽에서 디버깅 편의를 위해 아래 topic을 publish할 수 있다.

```text
/il/selected_policy
/il/predicted_steer
/il/overtake_phase
```

권장 메시지 타입:

| topic | type | 의미 |
| --- | --- | --- |
| `/il/selected_policy` | `std_msgs/msg/String` | 현재 선택된 policy 이름 |
| `/il/predicted_steer` | `std_msgs/msg/Float32` | IL이 예측한 조향각 degree |
| `/il/overtake_phase` | `std_msgs/msg/Float32` | 추월 phase, 0.0~1.0 |

주의:

```text
debug topic은 상태 확인용이다.
debug topic이 /xycar_motor를 대체하지 않는다.
IL debug publisher가 있어도 /xycar_motor publisher는 만들지 않는다.
```

## Fallback 규칙

IL 예측을 쓰면 안 되는 경우:

- 모델 파일이 없음
- TorchScript 로드 실패
- 이미지가 없음
- policy 이름이 잘못됨
- `overtake`인데 phase가 없음
- 추론 중 exception 발생

이때는 반드시:

```text
steer_deg = None
debug_info["ok"] = False
debug_info["warning"] = reason
```

로 반환한다.

rule-based 코드는 이 상태를 보고 기존 조향 로직 또는 안전 정지 로직으로 fallback한다.

## 책임 분리 요약

| 항목 | ILPolicyManager | Rule-based mission manager |
| --- | --- | --- |
| 이미지 전처리 | 담당 | 입력 이미지 전달 |
| 모델 로드 | 담당 | 경로 제공 |
| policy 선택 | 하지 않음 | 담당 |
| 조향 예측 | 담당 | 결과 사용 |
| 속도 결정 | 하지 않음 | 담당 |
| stop/go 결정 | 하지 않음 | 담당 |
| 미션 전환 | 하지 않음 | 담당 |
| 긴급 정지 | 하지 않음 | 담당 |
| `/xycar_motor` publish | 하지 않음 | 담당 |

## 구현 순서 권장안

1. `ILPolicyManager`를 독립 Python 클래스로 구현한다.
2. 더미 TorchScript 모델 또는 없는 모델 경로로 crash 없이 fallback되는지 확인한다.
3. `drive`, `cone`, `overtake` policy 선택을 rule-based 코드에서 호출만 하도록 연결한다.
4. `/xycar_motor` publisher가 하나뿐인지 확인한다.
5. debug topic은 선택적으로 추가한다.
6. 실제 주행 전에는 `steer_deg is None` fallback이 항상 안전하게 동작하는지 dry-run으로 검증한다.
