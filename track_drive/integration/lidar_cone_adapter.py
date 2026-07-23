"""Validate the vehicle-tested ``my_rule/cone_node`` output contract."""

from collections.abc import Sequence
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ConeCommand:
    """Parsed ``/my_rule/cone_cmd`` payload.

    The numeric steering and speed candidates stay outside Mission Manager.
    This adapter reads them only to validate the three-element source
    contract; Mission Manager receives availability booleans only.
    """

    angle: float
    speed: float
    confidence: float


@dataclass(frozen=True)
class LidarConeStatus:
    """Semantic LiDAR cone evidence used by Mission Manager."""

    source_valid: bool
    path_ready: bool
    present: bool


def parse_cone_command(values: Sequence[float]) -> ConeCommand | None:
    """Parse ``[angle, speed, confidence]`` or reject malformed data."""

    if isinstance(values, (str, bytes)) or len(values) < 3:
        return None
    try:
        angle, speed, confidence = (
            float(values[0]),
            float(values[1]),
            float(values[2]),
        )
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (angle, speed, confidence)):
        return None
    if speed < 0.0 or not 0.0 <= confidence <= 1.0:
        return None
    return ConeCommand(
        angle=angle,
        speed=speed,
        confidence=confidence,
    )


def source_sample_is_fresh(
    *,
    now_sec: float,
    last_receive_sec: float | None,
    timeout_sec: float,
) -> bool:
    """Return whether a source sample is usable at ``now_sec``."""

    if last_receive_sec is None:
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


def evaluate_lidar_cone_status(
    *,
    now_sec: float,
    command: ConeCommand | None,
    command_receive_sec: float | None,
    cluster_count: int | None,
    cluster_receive_sec: float | None,
    timeout_sec: float,
    path_ready_confidence: float,
    presence_confidence: float,
    presence_min_clusters: int,
) -> LidarConeStatus:
    """Derive entry readiness and continued cone presence.

    A ready path is deliberately stricter than continued presence. This
    mirrors the vehicle-tested consumer: confidence >= 0.35 can enter the
    section, while a weaker recovery command or two fresh clusters can keep
    the current cone section from exiting too early.
    """

    numeric_config = (
        timeout_sec,
        path_ready_confidence,
        presence_confidence,
    )
    if not all(math.isfinite(value) for value in numeric_config):
        return LidarConeStatus(False, False, False)
    if (
        timeout_sec < 0.0
        or not 0.0 <= presence_confidence <= path_ready_confidence <= 1.0
        or presence_min_clusters < 1
        or cluster_count is None
        or cluster_count < 0
    ):
        return LidarConeStatus(False, False, False)

    command_fresh = command is not None and source_sample_is_fresh(
        now_sec=now_sec,
        last_receive_sec=command_receive_sec,
        timeout_sec=timeout_sec,
    )
    clusters_fresh = source_sample_is_fresh(
        now_sec=now_sec,
        last_receive_sec=cluster_receive_sec,
        timeout_sec=timeout_sec,
    )
    source_valid = command_fresh and clusters_fresh
    if not source_valid:
        return LidarConeStatus(False, False, False)

    path_ready = command.confidence >= path_ready_confidence
    present = (
        command.confidence > presence_confidence
        or cluster_count >= presence_min_clusters
    )
    return LidarConeStatus(
        source_valid=True,
        path_ready=path_ready,
        present=present,
    )
