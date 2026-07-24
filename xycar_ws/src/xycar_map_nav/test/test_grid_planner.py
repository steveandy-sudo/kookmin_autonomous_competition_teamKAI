import math
from pathlib import Path

import numpy as np

from xycar_map_nav.grid_planner import (
    MapGrid,
    astar,
    line_is_free,
    plan_waypoint_route,
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


def test_example_route_file_exists():
    package = Path(__file__).resolve().parents[1]
    assert (
        package / "config" / "slam_glass_balanced_example_waypoints.yaml"
    ).is_file()
