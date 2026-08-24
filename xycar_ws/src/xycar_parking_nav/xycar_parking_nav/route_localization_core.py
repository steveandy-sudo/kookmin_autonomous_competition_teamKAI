"""Pure helpers for route-constrained localization."""

from __future__ import annotations

import math


def route_prefix(
    points: list[tuple[float, float]],
    maximum_distance_m: float,
) -> list[tuple[float, float]]:
    """Return an interpolated route prefix for start-area localization."""
    if len(points) < 2 or maximum_distance_m <= 0.0:
        return list(points)

    remaining = float(maximum_distance_m)
    prefix = [points[0]]
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1.0e-9:
            continue
        if length <= remaining:
            prefix.append(end)
            remaining -= length
            if remaining <= 1.0e-9:
                break
            continue
        ratio = remaining / length
        prefix.append((start[0] + ratio * dx, start[1] + ratio * dy))
        break
    return prefix if len(prefix) >= 2 else list(points[:2])
