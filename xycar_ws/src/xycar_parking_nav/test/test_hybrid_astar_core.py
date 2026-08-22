import math
from pathlib import Path

from xycar_parking_nav.command_core import Footprint
from xycar_parking_nav.hybrid_astar_core import HybridAStarConfig, HybridAStarPlanner
from xycar_parking_nav.map_core import load_occupancy_map
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


def test_b_entry_to_parallel_goal_is_kinematically_feasible():
    result = planner().plan(
        Pose2D(2.75, 2.39, -math.pi / 2.0),
        Pose2D(2.10, 3.14, -math.pi / 2.0),
    )
    assert result.success, (result.reason, result.expansions)
    assert any(point.direction < 0 for point in result.points)
