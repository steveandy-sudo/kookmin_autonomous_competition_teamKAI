"""Pure waypoint document and parking mission conversion helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class RouteWaypoint:
    name: str
    x: float
    y: float
    yaw: float | None = None

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("waypoint name must not be empty")
        if not math.isfinite(self.x) or not math.isfinite(self.y):
            raise ValueError("waypoint coordinates must be finite")
        if self.yaw is not None and not math.isfinite(self.yaw):
            raise ValueError("waypoint yaw must be finite when provided")


@dataclass(frozen=True)
class ReferenceLocation:
    name: str
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("reference location name must not be empty")
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("reference location pose must be finite")


def parse_reverse_waypoint_ranges(value: str | Iterable[str]) -> tuple[tuple[int, int], ...]:
    """Parse inclusive waypoint ranges such as ``4-5,8-9``.

    A range ``4-6`` means that the segments 4->5 and 5->6 are driven in
    reverse.  Bounds are checked later, once the number of captured points is
    known.
    """

    tokens = value.split(",") if isinstance(value, str) else list(value)
    ranges: list[tuple[int, int]] = []
    for raw_token in tokens:
        token = str(raw_token).strip()
        if not token:
            continue
        separator = "-" if "-" in token else ":" if ":" in token else None
        if separator is None:
            raise ValueError(
                f"reverse waypoint range '{token}' must use START-END"
            )
        parts = [part.strip() for part in token.split(separator)]
        if len(parts) != 2 or not all(part.isdigit() for part in parts):
            raise ValueError(
                f"reverse waypoint range '{token}' must use non-negative integers"
            )
        start, end = (int(parts[0]), int(parts[1]))
        if start >= end:
            raise ValueError(
                f"reverse waypoint range '{token}' must satisfy START < END"
            )
        ranges.append((start, end))
    return tuple(ranges)


def normalize_reverse_waypoint_ranges(
    ranges: Iterable[tuple[int, int]], waypoint_count: int
) -> tuple[tuple[int, int], ...]:
    """Validate ranges and merge ranges that share reverse segments."""

    count = int(waypoint_count)
    normalized = sorted((int(start), int(end)) for start, end in ranges)
    merged: list[tuple[int, int]] = []
    for start, end in normalized:
        if start < 0 or start >= end or end >= count:
            raise ValueError(
                f"reverse waypoint range {start}-{end} is outside 0-{max(0, count - 1)}"
            )
        if merged and start <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
        else:
            merged.append((start, end))
    return tuple(merged)


def reverse_range_text(ranges: Iterable[tuple[int, int]]) -> str:
    return ",".join(f"{start}-{end}" for start, end in ranges)


def reference_locations_from_mission(document: dict) -> list[ReferenceLocation]:
    """Read the previously surveyed START and A/B parking centers."""

    mission = document["mission"]
    initial = mission["initial_pose"]
    selected = {
        "PREVIOUS_START": ReferenceLocation(
            name="PREVIOUS_START",
            x=float(initial["x"]),
            y=float(initial["y"]),
            yaw=float(initial["yaw"]),
        )
    }
    for item in mission.get("steps", []):
        name = str(item.get("name", ""))
        if name not in {"A_PARK", "B_PARK"}:
            continue
        selected[name] = ReferenceLocation(
            name=name,
            x=float(item["x"]),
            y=float(item["y"]),
            yaw=float(item["yaw"]),
        )
    missing = {"PREVIOUS_START", "A_PARK", "B_PARK"} - set(selected)
    if missing:
        raise ValueError(
            "reference mission is missing: " + ", ".join(sorted(missing))
        )
    return [
        selected["PREVIOUS_START"],
        selected["A_PARK"],
        selected["B_PARK"],
    ]


def route_waypoints_from_items(items: Iterable[dict]) -> list[RouteWaypoint]:
    waypoints = [
        RouteWaypoint(
            name=str(item.get("name", f"wp_{index:02d}")),
            x=float(item["x"]),
            y=float(item["y"]),
            yaw=(float(item["yaw"]) if item.get("yaw") is not None else None),
        )
        for index, item in enumerate(items)
    ]
    names = [waypoint.name for waypoint in waypoints]
    if len(names) != len(set(names)):
        raise ValueError("waypoint names must be unique")
    return waypoints


def waypoint_headings(
    waypoints: list[RouteWaypoint],
    *,
    closed: bool,
    reverse_ranges: Iterable[tuple[int, int]] = (),
) -> list[float]:
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    headings: list[float] = []
    for index, waypoint in enumerate(waypoints):
        if waypoint.yaw is not None:
            headings.append(waypoint.yaw)
            continue
        if index + 1 < len(waypoints):
            following = waypoints[index + 1]
            start = waypoint
        elif closed:
            following = waypoints[0]
            start = waypoint
        else:
            following = waypoint
            start = waypoints[index - 1]
        dx = following.x - start.x
        dy = following.y - start.y
        if math.hypot(dx, dy) < 1.0e-6:
            raise ValueError("consecutive waypoints must not overlap")
        headings.append(math.atan2(dy, dx))
    normalized_ranges = normalize_reverse_waypoint_ranges(
        reverse_ranges, len(waypoints)
    )
    # A reverse range records the vehicle body heading, not the direction of
    # travel.  Point the car opposite each segment so backing from START to END
    # follows the clicked waypoint order.
    for start_index, end_index in normalized_ranges:
        for index in range(start_index, end_index + 1):
            if index < end_index:
                start = waypoints[index]
                following = waypoints[index + 1]
            else:
                start = waypoints[index - 1]
                following = waypoints[index]
            dx = following.x - start.x
            dy = following.y - start.y
            if math.hypot(dx, dy) < 1.0e-6:
                raise ValueError("consecutive waypoints must not overlap")
            headings[index] = math.atan2(
                math.sin(math.atan2(dy, dx) + math.pi),
                math.cos(math.atan2(dy, dx) + math.pi),
            )
    return headings


def waypoint_document(
    waypoints: list[RouteWaypoint],
    *,
    frame_id: str,
    closed: bool,
    reverse_ranges: Iterable[tuple[int, int]] = (),
) -> dict:
    normalized_ranges = normalize_reverse_waypoint_ranges(
        reverse_ranges, len(waypoints)
    )
    reverse_origins = {
        index
        for start, end in normalized_ranges
        for index in range(start, end)
    }
    return {
        "frame_id": str(frame_id),
        "closed": bool(closed),
        "reverse_ranges": [
            f"{start}-{end}" for start, end in normalized_ranges
        ],
        "waypoints": [
            {
                "name": waypoint.name,
                "x": round(waypoint.x, 4),
                "y": round(waypoint.y, 4),
                "controller_to_next": "global_path",
                "travel_direction_to_next": (
                    "reverse" if index in reverse_origins else "forward"
                ),
                **(
                    {"yaw": round(waypoint.yaw, 6)}
                    if waypoint.yaw is not None
                    else {}
                ),
            }
            for index, waypoint in enumerate(waypoints)
        ],
    }


def parking_mission_document(
    waypoints: list[RouteWaypoint],
    *,
    frame_id: str,
    closed: bool,
    reference_locations: Iterable[ReferenceLocation] = (),
    parking_snap_radius_m: float = 0.45,
    reverse_ranges: Iterable[tuple[int, int]] = (),
) -> dict:
    """Convert clicked base-frame path points into a Nav2 mission document."""

    snap_radius = float(parking_snap_radius_m)
    if not math.isfinite(snap_radius) or snap_radius <= 0.0:
        raise ValueError("parking snap radius must be finite and positive")
    parking_references = [
        reference
        for reference in reference_locations
        if reference.name in {"A_PARK", "B_PARK"}
    ]
    used_parking_references: set[str] = set()
    normalized_ranges = normalize_reverse_waypoint_ranges(
        reverse_ranges, len(waypoints)
    )
    headings = waypoint_headings(
        waypoints, closed=closed, reverse_ranges=normalized_ranges
    )
    reverse_destinations = {
        index
        for start, end in normalized_ranges
        for index in range(start + 1, end + 1)
    }
    reverse_boundaries = {
        start for start, _end in normalized_ranges if start > 0
    }
    reverse_range_indices = {
        index
        for start, end in normalized_ranges
        for index in range(start, end + 1)
    }
    steps = []
    for index in range(1, len(waypoints)):
        waypoint = waypoints[index]
        candidates = [
            reference
            for reference in parking_references
            if reference.name not in used_parking_references
            # A route may cross a parking box while traversing an explicitly
            # requested reverse range. Those numbered points are pass gates;
            # the parking snap is deferred to the first point after the range.
            and index not in reverse_range_indices
            and math.hypot(waypoint.x - reference.x, waypoint.y - reference.y)
            <= snap_radius
        ]
        parking_reference = min(
            candidates,
            key=lambda reference: math.hypot(
                waypoint.x - reference.x, waypoint.y - reference.y
            ),
            default=None,
        )
        if parking_reference is not None:
            used_parking_references.add(parking_reference.name)
            entry = waypoints[index - 1]
            target_from_entry_x = parking_reference.x - entry.x
            target_from_entry_y = parking_reference.y - entry.y
            parking_is_reverse = (
                math.cos(parking_reference.yaw) * target_from_entry_x
                + math.sin(parking_reference.yaw) * target_from_entry_y
                < 0.0
            )
            # An explicitly named *_ENTRY point is a direction boundary: the
            # car must reach it facing the surveyed bay heading before the
            # reverse-only parking controller starts.  Older/generic routes
            # keep the loose outward-facing transit gate so B parking and
            # existing captures are not silently made more restrictive.
            if steps:
                explicit_entry = entry.name.upper().endswith("_ENTRY")
                entry_yaw = (
                    parking_reference.yaw
                    if explicit_entry
                    else math.atan2(
                        entry.y - parking_reference.y,
                        entry.x - parking_reference.x,
                    )
                    if parking_is_reverse
                    else math.atan2(
                        parking_reference.y - entry.y,
                        parking_reference.x - entry.x,
                    )
                )
                steps[-1]["yaw"] = entry_yaw
                steps[-1]["maximum_retries"] = max(
                    int(steps[-1].get("maximum_retries", 2)),
                    3,
                )
                if explicit_entry:
                    steps[-1]["hold_sec"] = max(
                        float(steps[-1].get("hold_sec", 0.0)),
                        0.2,
                    )
                    steps[-1]["precise_goal"] = True
                    steps[-1]["allow_reverse"] = False
                elif (
                    parking_reference.name.upper() == "B_PARK"
                ):
                    # Parallel-parking entry must be reached on its measured
                    # centerline before the long reverse insertion. Accepting
                    # this point with the loose transit radius starts B_PARK
                    # about 0.4 m early and lets the reverse-only controller
                    # pass the bay before its lateral error has converged.
                    # Hand straight over to B_PARK once the entry pose is
                    # aligned; no additional stationary hold is needed.  An
                    # explicitly configured reverse range takes precedence,
                    # including when it crosses the B_PARK snap point.
                    steps[-1]["hold_sec"] = 0.0
                    steps[-1]["precise_goal"] = True
                    if (
                        parking_is_reverse
                        and not steps[-1].get("reverse_only", False)
                    ):
                        steps[-1]["allow_reverse"] = False
            parking_step = {
                "name": parking_reference.name,
                "x": round(parking_reference.x, 4),
                "y": round(parking_reference.y, 4),
                "yaw": parking_reference.yaw,
                "hold_sec": 1.0,
                "parking_goal": True,
                "precise_goal": True,
                "allow_reverse": parking_is_reverse,
                "maximum_retries": 3,
            }
            if parking_is_reverse:
                parking_step["reverse_only"] = True
            steps.append(parking_step)
            continue
        step = {
            "name": waypoint.name,
            "x": round(waypoint.x, 4),
            "y": round(waypoint.y, 4),
            "yaw": headings[index],
            "hold_sec": 0.0,
            # Captured waypoint missions are fully bidirectional.  Nav2 may
            # select a short reverse connection immediately instead of
            # waiting for a forward-only plan to fail first.
            "allow_reverse": True,
            "maximum_retries": 2,
        }
        if waypoint.name.upper().endswith("_ALIGN_IN"):
            # This is the one exact maneuvering pose before a straight entry.
            # It is intentionally bidirectional so an Ackermann path can turn
            # the car onto the parking-bay centerline without rotate-in-place.
            step["hold_sec"] = 0.2
            step["precise_goal"] = True
            step["maximum_retries"] = 3
        if waypoint.name.upper().endswith("_FORWARD_CUSP"):
            # A fixed forward-only end pose replaces an unstable internal
            # Reeds-Shepp cusp selected by the local controller.
            step["hold_sec"] = 0.2
            step["precise_goal"] = True
            step["allow_reverse"] = False
            step["maximum_retries"] = 3
        if waypoint.name.upper().endswith("_REVERSE_ALIGN"):
            # The following alignment arc is explicitly reverse-only.  This
            # prevents MPPI from changing gear repeatedly near the cusp.
            step["hold_sec"] = 0.2
            step["precise_goal"] = True
            step["allow_reverse"] = True
            step["reverse_only"] = True
            step["maximum_retries"] = 3
        if index in reverse_boundaries:
            # Reach the first point of a requested reverse range with the body
            # already facing opposite the following segment. It remains a
            # loose transit gate; only surveyed parking goals are strict.
            step["allow_reverse"] = True
        if index in reverse_destinations:
            # Every target inside START-END is followed using the reverse-only
            # controller, but is accepted with the normal transit radius.
            # A/B parking goals below retain their strict XY/yaw checks.
            step["allow_reverse"] = True
            step["reverse_only"] = True
        steps.append(step)
    if closed:
        first = waypoints[0]
        steps.append(
            {
                "name": f"{first.name}_RETURN",
                "x": round(first.x, 4),
                "y": round(first.y, 4),
                "yaw": headings[0],
                "hold_sec": 1.0,
                "precise_goal": True,
                "allow_reverse": True,
                "maximum_retries": 3,
            }
        )
    elif steps:
        # Stop at the open route's last transit gate, but do not require its
        # recorded click or heading exactly. Surveyed A/B parking goals are the
        # only automatically strict points in a generic captured route.
        steps[-1]["hold_sec"] = max(float(steps[-1].get("hold_sec", 0.0)), 1.0)
    first = waypoints[0]
    return {
        "mission": {
            "frame_id": str(frame_id),
            # Clicked points represent base_footprint directly, unlike the
            # competition-space rectangle-center survey coordinates.
            "base_from_reference_x_m": 0.0,
            "reverse_waypoint_ranges": [
                f"{start}-{end}" for start, end in normalized_ranges
            ],
            "initial_pose": {
                "name": first.name,
                "x": round(first.x, 4),
                "y": round(first.y, 4),
                "yaw": headings[0],
            },
            "steps": steps,
        }
    }
