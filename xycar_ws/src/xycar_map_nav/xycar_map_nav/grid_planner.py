"""Occupancy-grid transforms and segment-by-segment A* planning."""

from __future__ import annotations

from dataclasses import dataclass
import csv
import bisect
import heapq
import math
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
import yaml


Cell = tuple[int, int]
Point = tuple[float, float]


@dataclass(frozen=True)
class MapGrid:
    free: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    source_yaml: Path | None = None

    @property
    def height(self) -> int:
        return int(self.free.shape[0])

    @property
    def width(self) -> int:
        return int(self.free.shape[1])

    def contains(self, cell: Cell) -> bool:
        row, column = cell
        return 0 <= row < self.height and 0 <= column < self.width

    def world_to_cell(self, x: float, y: float) -> Cell:
        dx = float(x) - self.origin_x
        dy = float(y) - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        column = int(math.floor(local_x / self.resolution))
        bottom_row = int(math.floor(local_y / self.resolution))
        return self.height - 1 - bottom_row, column

    def cell_to_world(self, cell: Cell) -> Point:
        row, column = cell
        local_x = (float(column) + 0.5) * self.resolution
        bottom_row = self.height - 1 - int(row)
        local_y = (float(bottom_row) + 0.5) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )


@dataclass(frozen=True)
class PlannedRoute:
    points: tuple[Point, ...]
    segment_indices: tuple[int, ...]
    snapped_waypoints: tuple[Point, ...]


def load_path_csv(
    path_csv: str | Path,
    *,
    spacing_m: float,
    closed: bool,
) -> tuple[Point, ...]:
    source = Path(path_csv).expanduser().resolve()
    with source.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    points = [
        (float(row["x"]), float(row["y"]))
        for row in rows
    ]
    if (
        closed
        and len(points) > 1
        and math.hypot(
            points[-1][0] - points[0][0],
            points[-1][1] - points[0][1],
        )
        < 1.0e-6
    ):
        points.pop()
    if len(points) < 2:
        raise ValueError(f"path CSV has fewer than two points: {source}")

    spacing = max(0.02, float(spacing_m))
    sampled = [points[0]]
    accumulated = 0.0
    previous = points[0]
    for point in points[1:]:
        accumulated += math.hypot(
            point[0] - previous[0],
            point[1] - previous[1],
        )
        previous = point
        if accumulated >= spacing:
            sampled.append(point)
            accumulated = 0.0
    if not closed and sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return tuple(sampled)


def load_map_grid(
    map_yaml: str | Path,
    *,
    inflation_radius_m: float,
    unknown_is_occupied: bool = True,
    ignore_occupancy: bool = False,
) -> MapGrid:
    yaml_path = Path(map_yaml).expanduser().resolve()
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = Path(str(data["image"]))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"cannot read occupancy image: {image_path}")

    negate = bool(int(data.get("negate", 0)))
    normalized = image.astype(np.float32) / 255.0
    occupied_probability = normalized if negate else 1.0 - normalized
    occupied_threshold = float(data.get("occupied_thresh", 0.65))
    free_threshold = float(data.get("free_thresh", 0.25))
    if ignore_occupancy:
        blocked = np.zeros(image.shape, dtype=bool)
    elif unknown_is_occupied:
        blocked = occupied_probability > free_threshold
    else:
        blocked = occupied_probability >= occupied_threshold

    resolution = float(data["resolution"])
    radius_pixels = max(
        0, int(math.ceil(float(inflation_radius_m) / resolution))
    )
    if radius_pixels:
        diameter = radius_pixels * 2 + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (diameter, diameter)
        )
        blocked = cv2.dilate(blocked.astype(np.uint8), kernel) > 0

    origin = [float(value) for value in data["origin"]]
    return MapGrid(
        free=np.logical_not(blocked),
        resolution=resolution,
        origin_x=origin[0],
        origin_y=origin[1],
        origin_yaw=origin[2],
        source_yaml=yaml_path,
    )


def nearest_free_cell(
    grid: MapGrid,
    cell: Cell,
    *,
    maximum_radius_m: float,
) -> Cell:
    if grid.contains(cell) and bool(grid.free[cell]):
        return cell
    radius_cells = int(math.ceil(maximum_radius_m / grid.resolution))
    candidates = []
    for row in range(cell[0] - radius_cells, cell[0] + radius_cells + 1):
        for column in range(
            cell[1] - radius_cells, cell[1] + radius_cells + 1
        ):
            candidate = (row, column)
            if not grid.contains(candidate) or not bool(grid.free[candidate]):
                continue
            distance = math.hypot(row - cell[0], column - cell[1])
            if distance <= radius_cells:
                candidates.append((distance, candidate))
    if not candidates:
        raise ValueError(
            f"no free map cell within {maximum_radius_m:.2f} m of {cell}"
        )
    return min(candidates, key=lambda item: item[0])[1]


_NEIGHBORS = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (1, 1, math.sqrt(2.0)),
)


def astar(
    grid: MapGrid,
    start: Cell,
    goal: Cell,
    *,
    clearance_cost_weight: float = 0.0,
    clearance_cost_decay_m: float = 0.35,
) -> list[Cell]:
    if not grid.contains(start) or not bool(grid.free[start]):
        raise ValueError(f"A* start is occupied or outside map: {start}")
    if not grid.contains(goal) or not bool(grid.free[goal]):
        raise ValueError(f"A* goal is occupied or outside map: {goal}")
    if start == goal:
        return [start]

    frontier: list[tuple[float, float, Cell]] = [(0.0, 0.0, start)]
    cost = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    visited: set[Cell] = set()
    clearance_weight = max(0.0, float(clearance_cost_weight))
    clearance_decay = max(grid.resolution, float(clearance_cost_decay_m))
    clearance_m = None
    if clearance_weight > 0.0:
        clearance_m = (
            cv2.distanceTransform(
                grid.free.astype(np.uint8),
                cv2.DIST_L2,
                5,
            )
            * grid.resolution
        )
    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            break
        for delta_row, delta_column, step in _NEIGHBORS:
            neighbor = (
                current[0] + delta_row,
                current[1] + delta_column,
            )
            if not grid.contains(neighbor) or not bool(grid.free[neighbor]):
                continue
            if delta_row and delta_column:
                side_a = (current[0] + delta_row, current[1])
                side_b = (current[0], current[1] + delta_column)
                if not bool(grid.free[side_a]) or not bool(grid.free[side_b]):
                    continue
            clearance_penalty = 0.0
            if clearance_m is not None:
                clearance_penalty = clearance_weight * math.exp(
                    -float(clearance_m[neighbor]) / clearance_decay
                )
            candidate_cost = current_cost + step * (
                1.0 + clearance_penalty
            )
            if candidate_cost >= cost.get(neighbor, float("inf")):
                continue
            cost[neighbor] = candidate_cost
            parent[neighbor] = current
            heuristic = math.hypot(
                goal[0] - neighbor[0], goal[1] - neighbor[1]
            )
            heapq.heappush(
                frontier,
                (candidate_cost + heuristic, candidate_cost, neighbor),
            )
    if goal not in parent:
        raise ValueError(f"no collision-free map path from {start} to {goal}")

    path = [goal]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path


def _line_cells(start: Cell, end: Cell) -> list[Cell]:
    row0, column0 = start
    row1, column1 = end
    delta_column = abs(column1 - column0)
    delta_row = -abs(row1 - row0)
    step_column = 1 if column0 < column1 else -1
    step_row = 1 if row0 < row1 else -1
    error = delta_column + delta_row
    cells = []
    while True:
        cells.append((row0, column0))
        if row0 == row1 and column0 == column1:
            return cells
        doubled = 2 * error
        if doubled >= delta_row:
            error += delta_row
            column0 += step_column
        if doubled <= delta_column:
            error += delta_column
            row0 += step_row


def line_is_free(grid: MapGrid, start: Cell, end: Cell) -> bool:
    return all(
        grid.contains(cell) and bool(grid.free[cell])
        for cell in _line_cells(start, end)
    )


def simplify_cells(grid: MapGrid, path: Sequence[Cell]) -> list[Cell]:
    if len(path) <= 2:
        return list(path)
    simplified = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while candidate > anchor + 1:
            if line_is_free(grid, path[anchor], path[candidate]):
                break
            candidate -= 1
        simplified.append(path[candidate])
        anchor = candidate
    return simplified


def resample_polyline(
    points: Sequence[Point], spacing_m: float
) -> list[Point]:
    if not points:
        return []
    if len(points) == 1:
        return [points[0]]
    spacing = max(0.02, float(spacing_m))
    output = [points[0]]
    for first, second in zip(points, points[1:]):
        length = math.hypot(second[0] - first[0], second[1] - first[1])
        steps = max(1, int(math.ceil(length / spacing)))
        for index in range(1, steps + 1):
            ratio = index / steps
            output.append(
                (
                    first[0] + (second[0] - first[0]) * ratio,
                    first[1] + (second[1] - first[1]) * ratio,
                )
            )
    return output


def smooth_path_points(
    grid: MapGrid,
    points: Sequence[Point],
    *,
    closed: bool,
    data_weight: float,
    smooth_weight: float,
    iterations: int,
    anchor_indices: Sequence[int] = (),
    anchor_data_weight: float = 0.02,
    maximum_deviation_m: float = -1.0,
) -> list[Point]:
    """Smooth a resampled path without crossing inflated occupied cells."""
    if len(points) < 3 or iterations <= 0 or smooth_weight <= 0.0:
        return list(points)
    original = np.asarray(points, dtype=np.float64)
    smoothed = original.copy()
    data_gain = max(0.0, float(data_weight))
    smooth_gain = max(0.0, float(smooth_weight))
    anchors = {int(index) % len(points) for index in anchor_indices}
    maximum_deviation = float(maximum_deviation_m)

    for _ in range(int(iterations)):
        candidate = smoothed.copy()
        indices = range(len(points)) if closed else range(1, len(points) - 1)
        for index in indices:
            before = (index - 1) % len(points)
            after = (index + 1) % len(points)
            point_data_gain = (
                max(data_gain, float(anchor_data_weight))
                if index in anchors
                else data_gain
            )
            candidate[index] += point_data_gain * (
                original[index] - smoothed[index]
            ) + smooth_gain * (
                smoothed[before] + smoothed[after] - 2.0 * smoothed[index]
            )
        if maximum_deviation >= 0.0:
            displacement = candidate - original
            distances = np.linalg.norm(displacement, axis=1)
            outside = distances > maximum_deviation
            if np.any(outside):
                candidate[outside] = original[outside] + (
                    displacement[outside]
                    * (
                        maximum_deviation
                        / distances[outside, np.newaxis]
                    )
                )

        valid = True
        segment_count = len(candidate) if closed else len(candidate) - 1
        cells = [
            grid.world_to_cell(float(point[0]), float(point[1]))
            for point in candidate
        ]
        if any(
            not grid.contains(cell) or not bool(grid.free[cell])
            for cell in cells
        ):
            valid = False
        if valid:
            for index in range(segment_count):
                following = (index + 1) % len(candidate)
                if not line_is_free(grid, cells[index], cells[following]):
                    valid = False
                    break
        if not valid:
            break
        smoothed = candidate
    return [
        (float(point[0]), float(point[1]))
        for point in smoothed
    ]


def plan_waypoint_route(
    grid: MapGrid,
    waypoints: Sequence[Point],
    *,
    closed: bool,
    path_spacing_m: float,
    waypoint_snap_radius_m: float,
    path_smoothing_enabled: bool = True,
    path_smoothing_data_weight: float = 0.0005,
    path_smoothing_weight: float = 0.45,
    path_smoothing_iterations: int = 2500,
    path_smoothing_anchor_weight: float = 0.02,
    path_smoothing_maximum_deviation_m: float = -1.0,
    clearance_cost_weight: float = 3.0,
    clearance_cost_decay_m: float = 0.35,
) -> PlannedRoute:
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    cells = [
        nearest_free_cell(
            grid,
            grid.world_to_cell(point[0], point[1]),
            maximum_radius_m=waypoint_snap_radius_m,
        )
        for point in waypoints
    ]
    snapped = [grid.cell_to_world(cell) for cell in cells]
    pair_count = len(cells) if closed else len(cells) - 1
    route_points: list[Point] = []
    segment_indices: list[int] = []
    for segment_index in range(pair_count):
        start = cells[segment_index]
        goal = cells[(segment_index + 1) % len(cells)]
        cell_path = astar(
            grid,
            start,
            goal,
            clearance_cost_weight=clearance_cost_weight,
            clearance_cost_decay_m=clearance_cost_decay_m,
        )
        if clearance_cost_weight <= 0.0:
            cell_path = simplify_cells(grid, cell_path)
        world_path = resample_polyline(
            [grid.cell_to_world(cell) for cell in cell_path],
            path_spacing_m,
        )
        if route_points and world_path:
            # The shared waypoint belongs to the segment that starts there,
            # matching controller_to_next semantics exactly at the gate.
            segment_indices[-1] = segment_index
            world_path = world_path[1:]
        route_points.extend(world_path)
        segment_indices.extend([segment_index] * len(world_path))
    if (
        closed
        and len(route_points) > 1
        and math.hypot(
            route_points[-1][0] - route_points[0][0],
            route_points[-1][1] - route_points[0][1],
        )
        < 1.0e-6
    ):
        route_points.pop()
        segment_indices.pop()
    if len(route_points) < 2:
        raise ValueError("planned route contains fewer than two points")
    if path_smoothing_enabled:
        anchor_indices = {
            0,
            *(
                index
                for index in range(1, len(segment_indices))
                if segment_indices[index] != segment_indices[index - 1]
            ),
        }
        route_points = smooth_path_points(
            grid,
            route_points,
            closed=closed,
            data_weight=path_smoothing_data_weight,
            smooth_weight=path_smoothing_weight,
            iterations=path_smoothing_iterations,
            anchor_indices=tuple(sorted(anchor_indices)),
            anchor_data_weight=path_smoothing_anchor_weight,
            maximum_deviation_m=path_smoothing_maximum_deviation_m,
        )
        boundaries = [0]
        for segment_index in range(1, pair_count):
            original_boundary = segment_indices.index(segment_index)
            search_start = max(
                boundaries[-1] + 1, original_boundary - 30
            )
            search_stop = min(
                len(route_points), original_boundary + 31
            )
            boundary = min(
                range(search_start, search_stop),
                key=lambda index: (
                    route_points[index][0] - snapped[segment_index][0]
                ) ** 2
                + (
                    route_points[index][1] - snapped[segment_index][1]
                ) ** 2,
            )
            boundaries.append(boundary)
        segment_indices = [
            bisect.bisect_right(boundaries, index) - 1
            for index in range(len(route_points))
        ]
    return PlannedRoute(
        points=tuple(route_points),
        segment_indices=tuple(segment_indices),
        snapped_waypoints=tuple(snapped),
    )
