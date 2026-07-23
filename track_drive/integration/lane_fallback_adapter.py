import math
from collections.abc import Sequence
from typing import Any


WHITE_ROAD_SEGMENT_TYPES = frozenset({1, 2})
YELLOW_ROAD_SEGMENT_TYPES = frozenset({3, 4})


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


def lane_path_evidence_is_valid(
    *,
    segments: Sequence[Any],
    minimum_point_count: int,
    minimum_confidence: float,
) -> bool:
    """Accept a yellow centerline or a pair of white lane boundaries."""

    valid_white_count = 0
    for segment in segments:
        try:
            segment_type = int(segment.type)
            points = segment.points
            confidence = float(segment.confidence)
        except (AttributeError, TypeError, ValueError):
            continue

        if segment_type not in (
            WHITE_ROAD_SEGMENT_TYPES | YELLOW_ROAD_SEGMENT_TYPES
        ):
            continue
        if not centerline_values_are_valid(
            points=points,
            confidence=confidence,
            minimum_point_count=minimum_point_count,
            minimum_confidence=minimum_confidence,
        ):
            continue

        if segment_type in YELLOW_ROAD_SEGMENT_TYPES:
            return True
        valid_white_count += 1

    return valid_white_count >= 2


def lane_fallback_command_values_are_valid(
    *,
    steering_angle_deg: float,
    command_valid: bool,
    max_steering_angle_deg: float,
) -> bool:
    """Validate the physical steering candidate produced by the controller."""

    try:
        steering = float(steering_angle_deg)
        angle_limit = float(max_steering_angle_deg)
    except (TypeError, ValueError):
        return False
    if not command_valid:
        return False
    if not all(math.isfinite(value) for value in (steering, angle_limit)):
        return False
    if angle_limit <= 0.0:
        return False
    return abs(steering) <= angle_limit


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
