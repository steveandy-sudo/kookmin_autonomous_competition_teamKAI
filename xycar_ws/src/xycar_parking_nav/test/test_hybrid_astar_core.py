import math
from pathlib import Path

from xycar_parking_nav.command_core import Footprint
from xycar_parking_nav.hybrid_astar_core import HybridAStarConfig, HybridAStarPlanner
from xycar_parking_nav.map_core import FREE, OCCUPIED, OccupancyMap, load_occupancy_map
from xycar_parking_nav.mission_core import Pose2D


PACKAGE = Path(__file__).resolve().parents[1]


def planner():
    return HybridAStarPlanner(
        load_occupancy_map(PACKAGE / "maps" / "parking_map.yaml"),
        Footprint(-0.46, 0.11, 0.18),
        HybridAStarConfig(maximum_expansions=100000),
    )


def test_a_entry_to_bay_requires_reverse():
    result = planner().plan(Pose2D(1.01, 4.2, 0.0), Pose2D(0.16, 4.2, 0.0))
    assert result.success, result.reason
    assert any(point.direction < 0 for point in result.points)


def test_recorded_a_entry_to_parking_is_one_reverse_only_leg():
    result = planner().plan(
        Pose2D(1.034, 4.127, 0.021),
        Pose2D(-0.016, 4.105, 0.021),
    )

    assert result.success, (result.reason, result.expansions)
    moving_directions = {
        point.direction for point in result.points if point.direction != 0
    }
    assert moving_directions == {-1}
    assert result.direction_changes == 0


def test_b_entry_to_parallel_goal_is_kinematically_feasible():
    result = planner().plan(
        Pose2D(2.75, 2.39, -math.pi / 2.0),
        Pose2D(2.10, 3.14, -math.pi / 2.0),
    )
    assert result.success, (result.reason, result.expansions)
    assert any(point.direction < 0 for point in result.points)


def test_new_obstacle_uses_shorter_open_detour():
    width, height, resolution = 50, 35, 0.20
    cells = [FREE] * (width * height)
    for grid_x in range(width):
        cells[grid_x] = OCCUPIED
        cells[(height - 1) * width + grid_x] = OCCUPIED
    for grid_y in range(height):
        cells[grid_y * width] = OCCUPIED
        cells[grid_y * width + width - 1] = OCCUPIED

    # A newly marked one-metre-wide obstacle blocks the direct corridor.  The
    # upper passage is much shorter than circling below it.
    for grid_y in range(10, 16):
        for grid_x in range(22, 27):
            cells[grid_y * width + grid_x] = OCCUPIED

    occupancy_map = OccupancyMap(
        width, height, resolution, 0.0, 0.0, 0.0, tuple(cells)
    )
    fast_planner = HybridAStarPlanner(
        occupancy_map,
        Footprint(-0.46, 0.11, 0.18),
        HybridAStarConfig(
            primitive_length_m=0.30,
            collision_sample_step_m=0.05,
            xy_quantization_m=0.20,
            maximum_expansions=50000,
        ),
    )

    result = fast_planner.plan(Pose2D(1.5, 3.0, 0.0), Pose2D(8.5, 3.0, 0.0))

    assert result.success, (result.reason, result.expansions)
    assert max(point.pose.y for point in result.points) > 3.45
    assert min(point.pose.y for point in result.points) >= 3.0
    assert result.direction_changes == 0

    # Prove the lower passage is also feasible, then compare its forced route
    # with the chosen upper route rather than merely accepting the first path.
    lower_only_cells = list(cells)
    for grid_y in range(16, height - 1):
        for grid_x in range(22, 27):
            lower_only_cells[grid_y * width + grid_x] = OCCUPIED
    lower_only_map = OccupancyMap(
        width, height, resolution, 0.0, 0.0, 0.0, tuple(lower_only_cells)
    )
    lower_only_result = HybridAStarPlanner(
        lower_only_map,
        Footprint(-0.46, 0.11, 0.18),
        fast_planner.config,
    ).plan(Pose2D(1.5, 3.0, 0.0), Pose2D(8.5, 3.0, 0.0))

    assert lower_only_result.success, lower_only_result.reason
    assert min(point.pose.y for point in lower_only_result.points) < 2.0
    assert result.cost < lower_only_result.cost
