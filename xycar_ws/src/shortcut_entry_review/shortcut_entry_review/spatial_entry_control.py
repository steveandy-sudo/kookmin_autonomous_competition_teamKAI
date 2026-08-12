"""Distance-based activation helpers for W1 shortcut-entry control.

The helpers are deliberately stateless and deterministic.  Runtime nodes may
latch the largest blend already reached, but neither timestamps nor frame
counts participate in the transition.
"""

from __future__ import annotations

import math


def dynamic_blend_start_distance_m(
    *,
    speed_mps: float,
    full_control_distance_m: float,
    minimum_start_distance_m: float,
    maximum_start_distance_m: float,
    control_latency_sec: float,
    distance_margin_m: float,
) -> float:
    """Return a speed-aware, bounded distance at which W1 starts steering."""
    full = max(0.0, float(full_control_distance_m))
    minimum = max(full + 1.0e-3, float(minimum_start_distance_m))
    maximum = max(minimum, float(maximum_start_distance_m))
    preview = (
        full
        + abs(float(speed_mps)) * max(0.0, float(control_latency_sec))
        + max(0.0, float(distance_margin_m))
    )
    return float(min(max(preview, minimum), maximum))


def distance_blend_ratio(
    *,
    remaining_distance_m: float,
    start_distance_m: float,
    full_control_distance_m: float,
) -> float:
    """Map remaining metres to a smooth RULE->W1 blend in ``[0, 1]``."""
    distance = float(remaining_distance_m)
    start = float(start_distance_m)
    full = float(full_control_distance_m)
    if not math.isfinite(distance) or start <= full:
        return 0.0
    linear = min(max((start - distance) / (start - full), 0.0), 1.0)
    # Smoothstep avoids a steering-rate discontinuity at both spatial gates.
    return float(linear * linear * (3.0 - 2.0 * linear))


def blend_rule_and_w1_command(
    *,
    rule_angle: float,
    rule_speed: float,
    w1_angle: float,
    blend_ratio: float,
) -> tuple[float, float]:
    """Blend steering only; preserve the current RULE speed exactly."""
    ratio = min(max(float(blend_ratio), 0.0), 1.0)
    angle = (1.0 - ratio) * float(rule_angle) + ratio * float(w1_angle)
    return float(angle), float(rule_speed)
