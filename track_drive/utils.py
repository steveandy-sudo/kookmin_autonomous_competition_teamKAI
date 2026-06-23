"""공통 유틸리티 함수 모음."""

from __future__ import annotations

import math
from typing import Optional


def clamp(value: float, minimum: float, maximum: float) -> float:
    """값을 지정한 최소/최대 범위로 제한한다."""
    # 값을 최소/최대 범위 안으로 제한한다.
    return max(minimum, min(maximum, value))


def safe_float(value: object, default: float = 0.0) -> float:
    """숫자 변환 실패 또는 비정상 값(NaN/Inf)일 때 기본값을 반환한다."""
    # 입력값을 float로 바꾸고 실패하면 기본값을 반환한다.
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default

    if math.isnan(result) or math.isinf(result):
        return default
    return result


def get_logger_or_none(node: object) -> Optional[object]:
    """노드에서 로거를 안전하게 가져온다."""
    # 노드에서 logger를 안전하게 가져온다.
    if node is None:
        return None

    get_logger = getattr(node, 'get_logger', None)
    if callable(get_logger):
        return get_logger()
    return None


def log_debug(node: object, message: str) -> None:
    """디버그 로그를 안전하게 출력한다."""
    # logger가 있으면 debug 로그를 출력한다.
    logger = get_logger_or_none(node)
    if logger:
        logger.debug(message)


def log_info(node: object, message: str) -> None:
    """정보 로그를 안전하게 출력한다."""
    # logger가 있으면 info 로그를 출력한다.
    logger = get_logger_or_none(node)
    if logger:
        logger.info(message)
