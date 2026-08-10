import numpy as np

from my_control.sitl_bypass_path import (
    RememberedObstacle,
    SitlBypassConfig,
    SitlBypassPathPlanner,
    blend_to_opposite_lane,
    bypass_transition_window,
)


def straight_path(length_m: float = 3.0) -> np.ndarray:
    return np.column_stack(
        (np.linspace(0.0, length_m, 121), np.zeros(121))
    )


def test_sitl_longitudinal_windows_scale_only_with_xycar_length() -> None:
    config = SitlBypassConfig(vehicle_length_m=0.55)
    scale = 0.55 / 2.473
    approach, hold = bypass_transition_window(2.0, config)
    assert abs(approach - (2.0 - 6.0 * scale)) < 1.0e-9
    assert abs(hold - (2.0 - 1.2 * scale)) < 1.0e-9
    assert abs(config.post_obstacle_margin_m - 1.5 * scale) < 1.0e-9
    assert abs(config.return_distance_m - 5.0 * scale) < 1.0e-9


def test_obstacle_on_left_blends_to_right_lane_and_returns() -> None:
    config = SitlBypassConfig()
    path = blend_to_opposite_lane(
        straight_path(),
        RememberedObstacle(
            x=1.0,
            y=0.1,
            length=0.55,
            width=0.28,
            bypass_side=-1.0,
        ),
        config,
    )
    assert path[0, 1] == 0.0
    assert np.min(path[:, 1]) < -0.39
    assert abs(path[-1, 1]) < 1.0e-6


def test_obstacle_on_right_blends_to_left_lane() -> None:
    path = blend_to_opposite_lane(
        straight_path(),
        RememberedObstacle(
            x=1.0,
            y=-0.1,
            length=0.55,
            width=0.28,
            bypass_side=1.0,
        ),
        SitlBypassConfig(),
    )
    assert np.max(path[:, 1]) > 0.39


def test_planner_keeps_obstacle_through_dropout_until_scaled_clearance() -> None:
    config = SitlBypassConfig()
    planner = SitlBypassPathPlanner(config)
    planner.observe(
        active=True,
        observation_valid=True,
        bypass_side=-1.0,
        obstacle_x=0.8,
        obstacle_y=0.1,
        obstacle_length=0.55,
        obstacle_width=0.28,
    )
    planner.observe(
        active=True,
        observation_valid=False,
        bypass_side=-1.0,
        obstacle_x=float("inf"),
        obstacle_y=0.0,
        obstacle_length=0.55,
        obstacle_width=0.28,
    )
    assert planner.active
    planner.advance(0.8 + 0.55 + config.memory_clearance_m - 0.01)
    assert planner.active
    planner.advance(0.02)
    assert not planner.active


def test_inactive_request_resets_remembered_path() -> None:
    planner = SitlBypassPathPlanner(SitlBypassConfig())
    planner.observe(
        active=True,
        observation_valid=True,
        bypass_side=1.0,
        obstacle_x=0.8,
        obstacle_y=-0.1,
        obstacle_length=0.55,
        obstacle_width=0.28,
    )
    planner.observe(
        active=False,
        observation_valid=False,
        bypass_side=1.0,
        obstacle_x=0.0,
        obstacle_y=0.0,
        obstacle_length=0.55,
        obstacle_width=0.28,
    )
    assert not planner.active
