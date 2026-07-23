import math
from collections.abc import Sequence
from typing import Any


def centerline_values_are_valid(
    *,
    points: Sequence[Any],
    confidence: float,
    minimum_point_count: int,
    minimum_confidence: float,
) -> bool:
    """Validate the numeric fields needed from a Centerline message."""

    try:
        point_count = len(points)
        confidence_value = float(confidence)
        minimum_points = int(minimum_point_count)
        confidence_threshold = float(minimum_confidence)
    except (TypeError, ValueError):
        return False

    if minimum_points < 1 or point_count < minimum_points:
        return False
    if not all(
        math.isfinite(value)
        for value in (confidence_value, confidence_threshold)
    ):
        return False
    if not 0.0 <= confidence_threshold <= 1.0:
        return False
    if not confidence_threshold <= confidence_value <= 1.0:
        return False

    for point in points:
        try:
            coordinates = (
                float(point.x),
                float(point.y),
                float(point.z),
            )
        except (AttributeError, TypeError, ValueError):
            return False
        if not all(math.isfinite(value) for value in coordinates):
            return False

    return True


def lane_fallback_source_is_fresh(
    *,
    now_sec: float,
    last_receive_sec: float | None,
    timeout_sec: float,
    source_values_valid: bool,
) -> bool:
    """Return whether the last valid Centerline is still fresh."""

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
    if age_sec < 0.0:
        return False
    return age_sec <= timeout_sec or math.isclose(
        age_sec,
        timeout_sec,
        rel_tol=1e-9,
        abs_tol=1e-9,
    )
