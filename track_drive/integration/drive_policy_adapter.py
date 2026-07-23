import math
from collections.abc import Sequence


MINIMUM_POLICY_DEBUG_VALUES = 4
FINAL_STEERING_INDEX = 2
MINIMUM_STEERING_ANGLE = -42.0
MAXIMUM_STEERING_ANGLE = 42.0


def policy_debug_values_are_valid(
    values: Sequence[float],
) -> bool:
    """최신 성공 추론에 정상 범위의 최종 조향 후보가 있는지 확인한다."""

    try:
        if isinstance(values, (str, bytes)):
            return False
        if len(values) < MINIMUM_POLICY_DEBUG_VALUES:
            return False
        final_steering = float(values[FINAL_STEERING_INDEX])
    except (IndexError, TypeError, ValueError):
        return False
    return (
        math.isfinite(final_steering)
        and MINIMUM_STEERING_ANGLE
        <= final_steering
        <= MAXIMUM_STEERING_ANGLE
    )


def drive_policy_source_is_fresh(
    *,
    now_sec: float,
    last_receive_sec: float | None,
    timeout_sec: float,
    source_values_valid: bool,
) -> bool:
    """마지막 성공 추론 debug가 아직 최신인지 판단한다."""

    if not source_values_valid or last_receive_sec is None:
        return False
    if not all(
        math.isfinite(value)
        for value in (now_sec, last_receive_sec, timeout_sec)
    ):
        return False
    if timeout_sec < 0.0:
        return False

    age_sec = now_sec - last_receive_sec
    return 0.0 <= age_sec <= timeout_sec
