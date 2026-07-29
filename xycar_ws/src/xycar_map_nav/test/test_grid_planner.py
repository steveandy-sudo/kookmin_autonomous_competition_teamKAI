import math
from pathlib import Path

import cv2
import numpy as np

from xycar_map_nav.grid_planner import (
    MapGrid,
    astar,
    line_is_free,
    load_map_grid,
    load_path_csv,
    plan_waypoint_route,
    smooth_path_points,
)


def make_grid() -> MapGrid:
    free = np.ones((30, 40), dtype=bool)
    free[:, 20] = False
    free[13:17, 20] = True
    return MapGrid(
        free=free,
        resolution=0.1,
        origin_x=-1.0,
        origin_y=-1.5,
        origin_yaw=0.0,
    )


def test_world_cell_round_trip_with_rotated_origin():
    grid = MapGrid(
        free=np.ones((20, 20), dtype=bool),
        resolution=0.1,
        origin_x=2.0,
        origin_y=-1.0,
        origin_yaw=math.pi * 0.5,
    )
    cell = (7, 11)
    assert grid.world_to_cell(*grid.cell_to_world(cell)) == cell


def test_astar_uses_gap_and_prevents_corner_cutting():
    grid = make_grid()
    start = (5, 5)
    goal = (5, 35)
    path = astar(grid, start, goal)
    assert path[0] == start
    assert path[-1] == goal
    assert any(13 <= row <= 16 and column == 20 for row, column in path)
    assert not line_is_free(grid, start, goal)


def test_trinary_gray_pixel_is_blocked_when_unknown_is_occupied(tmp_path):
    image_path = tmp_path / "map.pgm"
    cv2.imwrite(
        str(image_path),
        np.asarray([[0, 205, 254]], dtype=np.uint8),
    )
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        "image: map.pgm\n"
        "mode: trinary\n"
        "resolution: 0.05\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )

    grid = load_map_grid(
        yaml_path,
        inflation_radius_m=0.0,
        unknown_is_occupied=True,
    )

    assert grid.free.tolist() == [[False, False, True]]


def test_waypoints_are_planned_segment_by_segment():
    grid = make_grid()
    waypoints = [
        grid.cell_to_world((5, 5)),
        grid.cell_to_world((15, 24)),
        grid.cell_to_world((24, 35)),
    ]
    route = plan_waypoint_route(
        grid,
        waypoints,
        closed=False,
        path_spacing_m=0.1,
        waypoint_snap_radius_m=0.2,
    )
    assert len(route.points) == len(route.segment_indices)
    assert set(route.segment_indices) == {0, 1}
    assert route.snapped_waypoints == tuple(waypoints)
    middle_index = min(
        range(len(route.points)),
        key=lambda index: (
            route.points[index][0] - waypoints[1][0]
        ) ** 2
        + (route.points[index][1] - waypoints[1][1]) ** 2,
    )
    assert route.segment_indices[middle_index] == 1


def test_path_smoothing_reduces_corner_and_stays_collision_free():
    grid = MapGrid(
        free=np.ones((80, 80), dtype=bool),
        resolution=0.05,
        origin_x=-2.0,
        origin_y=-2.0,
        origin_yaw=0.0,
    )
    points = (
        [(index * 0.05, 0.0) for index in range(21)]
        + [(1.0, index * 0.05) for index in range(1, 21)]
    )
    smoothed = smooth_path_points(
        grid,
        points,
        closed=False,
        data_weight=0.12,
        smooth_weight=0.35,
        iterations=80,
    )
    assert len(smoothed) == len(points)
    assert smoothed[20][0] < points[20][0]
    assert smoothed[20][1] > points[20][1]
    cells = [grid.world_to_cell(*point) for point in smoothed]
    assert all(bool(grid.free[cell]) for cell in cells)
    assert all(
        line_is_free(grid, first, second)
        for first, second in zip(cells, cells[1:])
    )


def test_path_smoothing_respects_maximum_deviation():
    grid = MapGrid(
        free=np.ones((80, 80), dtype=bool),
        resolution=0.05,
        origin_x=-2.0,
        origin_y=-2.0,
        origin_yaw=0.0,
    )
    points = (
        [(index * 0.05, 0.0) for index in range(21)]
        + [(1.0, index * 0.05) for index in range(1, 21)]
    )
    smoothed = smooth_path_points(
        grid,
        points,
        closed=False,
        data_weight=0.0,
        smooth_weight=0.45,
        iterations=300,
        maximum_deviation_m=0.10,
    )
    assert max(
        math.hypot(
            smoothed[index][0] - points[index][0],
            smoothed[index][1] - points[index][1],
        )
        for index in range(len(points))
    ) <= 0.100001


def test_route_smoothing_preserves_segment_labels():
    grid = make_grid()
    waypoints = [
        grid.cell_to_world((5, 5)),
        grid.cell_to_world((15, 24)),
        grid.cell_to_world((24, 35)),
    ]
    route = plan_waypoint_route(
        grid,
        waypoints,
        closed=False,
        path_spacing_m=0.1,
        waypoint_snap_radius_m=0.2,
        path_smoothing_enabled=True,
    )
    assert len(route.points) == len(route.segment_indices)
    assert set(route.segment_indices) == {0, 1}


def test_example_route_file_exists():
    package = Path(__file__).resolve().parents[1]
    assert (
        package / "config" / "slam_glass_balanced_example_waypoints.yaml"
    ).is_file()


def test_load_path_csv_resamples_and_removes_closed_duplicate(tmp_path):
    path = tmp_path / "path.csv"
    path.write_text(
        "x,y\n"
        "0.0,0.0\n"
        "0.05,0.0\n"
        "0.10,0.0\n"
        "0.20,0.0\n"
        "0.0,0.0\n",
        encoding="utf-8",
    )
    points = load_path_csv(path, spacing_m=0.09, closed=True)
    assert points == ((0.0, 0.0), (0.1, 0.0), (0.2, 0.0))
