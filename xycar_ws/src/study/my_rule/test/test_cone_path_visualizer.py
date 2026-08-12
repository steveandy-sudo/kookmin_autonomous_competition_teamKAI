import math

import pytest

from my_rule.cone_path_visualizer_node import (
    fov_outline_points,
    sector_triangle_points,
    select_lookahead_index,
    transform_planar_points,
)


def test_rear_axle_path_is_transformed_into_laser_frame():
    transformed = transform_planar_points(
        [(0.42, 0.0), (1.42, -0.2)],
        "rear_axle",
        "laser_frame",
        0.42,
    )
    assert transformed == pytest.approx([(0.0, 0.0), (1.0, -0.2)])


def test_base_footprint_path_uses_rear_axle_transform():
    transformed = transform_planar_points(
        [(0.42, 0.0), (1.42, -0.2)],
        "base_footprint",
        "laser_frame",
        0.42,
    )
    assert transformed == pytest.approx([(0.0, 0.0), (1.0, -0.2)])


def test_unknown_path_frame_is_not_silently_mislabeled():
    assert transform_planar_points(
        [(1.0, 0.0)],
        "map",
        "laser_frame",
        0.42,
    ) is None


def test_lookahead_selection_matches_controller_distance_rule():
    index, lookahead = select_lookahead_index(
        [(0.42, 0.0), (0.80, 0.0), (1.20, 0.10)],
        minimum_m=0.7,
        maximum_m=1.45,
        length_scale=0.12,
    )
    assert lookahead == pytest.approx(0.7)
    assert index == 1


def test_fov_outline_uses_requested_range_and_angles():
    outline = fov_outline_points(2.2, -94.0, 94.0, samples=3)
    assert outline[0] == (0.0, 0.0)
    assert outline[-1] == (0.0, 0.0)
    assert math.hypot(*outline[1]) == pytest.approx(2.2)
    assert math.degrees(math.atan2(outline[1][1], outline[1][0])) == pytest.approx(
        -94.0
    )


def test_sector_fill_respects_inner_range_and_seed_angles():
    triangles = sector_triangle_points(0.18, 2.2, 15.0, 90.0, samples=3)
    assert len(triangles) == 12
    radii = [math.hypot(*point) for point in triangles]
    assert min(radii) == pytest.approx(0.18)
    assert max(radii) == pytest.approx(2.2)
    angles = [math.degrees(math.atan2(y, x)) for x, y in triangles]
    assert min(angles) == pytest.approx(15.0)
    assert max(angles) == pytest.approx(90.0)
