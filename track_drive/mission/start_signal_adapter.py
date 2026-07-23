import math

from .states import StartSignal


_START_SIGNAL_BY_SOURCE_STATE = {
    "RED": StartSignal.RED,
    "YELLOW": StartSignal.YELLOW,
    "GREEN": StartSignal.GREEN,
    "BLUE": StartSignal.GREEN,
    "NONE": StartSignal.UNKNOWN,
    "LEFT": StartSignal.UNKNOWN,
    "UNKNOWN": StartSignal.UNKNOWN,
}


def normalize_start_signal_state(
    raw_state: str,
) -> tuple[StartSignal, bool]:
    """신호등 노드의 상태 문자열을 Mission Manager 입력으로 바꾼다."""

    if not isinstance(raw_state, str):
        return StartSignal.UNKNOWN, False

    normalized = raw_state.strip().upper()
    signal = _START_SIGNAL_BY_SOURCE_STATE.get(normalized)
    if signal is None:
        return StartSignal.UNKNOWN, False
    return signal, True


def start_signal_source_is_fresh(
    *,
    now_sec: float,
    last_receive_sec: float | None,
    timeout_sec: float,
    source_state_valid: bool,
) -> bool:
    """마지막 신호등 상태가 아직 사용할 수 있는 최신 결과인지 판단한다."""

    if not source_state_valid or last_receive_sec is None:
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
