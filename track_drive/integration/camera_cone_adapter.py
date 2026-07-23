import math
from collections.abc import Sequence
from typing import Any


def detector_output_is_valid(
    *,
    class_scores: Sequence[float],
    raw_shape: str,
    expected_class_count: int,
) -> bool:
    """Return whether the current frame produced a usable YOLO output."""

    try:
        class_count = int(expected_class_count)
        if class_count < 1 or not str(raw_shape).strip():
            return False
        if isinstance(class_scores, (str, bytes)):
            return False
        if len(class_scores) < class_count:
            return False
        scores = tuple(
            float(class_scores[index]) for index in range(class_count)
        )
    except (IndexError, TypeError, ValueError):
        return False

    return all(
        math.isfinite(score) and 0.0 <= score <= 1.0
        for score in scores
    )


def count_camera_cones(
    *,
    detections: Sequence[Any],
    cone_class_ids: Sequence[int],
    minimum_confidence: float,
) -> int | None:
    """Count cone detections, returning None for malformed source data."""

    try:
        if isinstance(detections, (str, bytes)):
            return None
        allowed_ids = {int(class_id) for class_id in cone_class_ids}
        threshold = float(minimum_confidence)
    except (TypeError, ValueError):
        return None

    if not allowed_ids:
        return None
    if not math.isfinite(threshold) or not 0.0 <= threshold <= 1.0:
        return None

    count = 0
    for detection in detections:
        try:
            class_id = int(detection.class_id)
            score = float(detection.score)
        except (AttributeError, TypeError, ValueError):
            return None
        if not math.isfinite(score) or not 0.0 <= score <= 1.0:
            return None
        if class_id in allowed_ids and score >= threshold:
            count += 1

    return count


def camera_cone_source_is_fresh(
    *,
    now_sec: float,
    last_receive_sec: float | None,
    timeout_sec: float,
    source_count: int,
) -> bool:
    """Return whether a non-negative source count is still fresh."""

    if last_receive_sec is None:
        return False
    try:
        count = int(source_count)
    except (TypeError, ValueError):
        return False
    if count < 0:
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
