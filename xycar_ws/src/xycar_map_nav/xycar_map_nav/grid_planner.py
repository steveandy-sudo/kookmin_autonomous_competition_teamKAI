"""Occupancy-grid transforms and segment-by-segment A* planning."""

from __future__ import annotations

from dataclasses import dataclass
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


def load_map_grid(
    map_yaml: str | Path,
    *,
    inflation_radius_m: float,
    unknown_is_occupied: bool = True,
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
    if unknown_is_occupied:
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


def astar(grid: MapGrid, start: Cell, goal: Cell) -> list[Cell]:
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
            candidate_cost = current_cost + step
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


def resample_polyline(points: Sequence[Point], spacing_m: float) -> list[Point]:
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


def plan_waypoint_route(
    grid: MapGrid,
    waypoints: Sequence[Point],
    *,
    closed: bool,
    path_spacing_m: float,
    waypoint_snap_radius_m: float,
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
        cell_path = simplify_cells(grid, astar(grid, start, goal))
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
    return PlannedRoute(
        points=tuple(route_points),
        segment_indices=tuple(segment_indices),
        snapped_waypoints=tuple(snapped),
    )
