from xycar_rule_drive.post_red_turn_exit import PostRedTurnExitBypass
from xycar_rule_drive.post_red_turn_exit import PostRedTurnExitConfig
from xycar_rule_drive.post_red_turn_exit import PostRedTurnExitEvent


def observe(
    bypass: PostRedTurnExitBypass,
    *,
    now: float,
    window_active: bool = True,
    raw_command: float,
    curvature: float,
    points: int = 20,
    confidence: float = 0.8,
) -> tuple[bool, PostRedTurnExitEvent]:
    return bypass.begin_cycle(
        now=now,
        window_active=window_active,
        new_path_frame=True,
        path_valid=True,
        path_point_count=points,
        path_confidence=confidence,
        raw_command=raw_command,
        curve_detection_per_m=curvature,
    )


def test_bypass_requires_left_turn_then_confirmed_exit() -> None:
    bypass = PostRedTurnExitBypass(PostRedTurnExitConfig())

    assert observe(
        bypass,
        now=1.0,
        raw_command=-24.0,
        curvature=0.45,
    ) == (False, PostRedTurnExitEvent.NONE)
    assert observe(
        bypass,
        now=1.1,
        raw_command=-26.0,
        curvature=0.40,
    ) == (False, PostRedTurnExitEvent.LEFT_TURN_CONFIRMED)

    assert observe(
        bypass,
        now=1.2,
        raw_command=-3.0,
        curvature=0.18,
    ) == (False, PostRedTurnExitEvent.NONE)
    assert observe(
        bypass,
        now=1.3,
        raw_command=8.0,
        curvature=0.10,
    ) == (True, PostRedTurnExitEvent.BYPASS_STARTED)
    assert bypass.state().bypass_active


def test_bypass_finishes_after_three_centered_output_frames() -> None:
    bypass = PostRedTurnExitBypass(PostRedTurnExitConfig())
    observe(bypass, now=1.0, raw_command=-24.0, curvature=0.45)
    observe(bypass, now=1.1, raw_command=-26.0, curvature=0.40)
    observe(bypass, now=1.2, raw_command=-3.0, curvature=0.18)
    observe(bypass, now=1.3, raw_command=8.0, curvature=0.10)

    for command in (4.0, -3.0):
        assert (
            bypass.observe_output(
                new_path_frame=True,
                path_valid=True,
                output_command=command,
            )
            == PostRedTurnExitEvent.NONE
        )
    assert (
        bypass.observe_output(
            new_path_frame=True,
            path_valid=True,
            output_command=2.0,
        )
        == PostRedTurnExitEvent.STRAIGHT_CONFIRMED
    )
    assert bypass.state().completed
    assert not bypass.state().bypass_active


def test_invalid_path_quality_cannot_arm_bypass() -> None:
    bypass = PostRedTurnExitBypass(PostRedTurnExitConfig())

    for index in range(5):
        assert observe(
            bypass,
            now=float(index),
            raw_command=-42.0,
            curvature=0.5,
            points=4,
        ) == (False, PostRedTurnExitEvent.NONE)

    assert not bypass.state().left_turn_confirmed


def test_window_end_resets_and_global_curves_remain_untouched() -> None:
    bypass = PostRedTurnExitBypass(PostRedTurnExitConfig())
    observe(bypass, now=1.0, raw_command=-24.0, curvature=0.45)
    observe(bypass, now=1.1, raw_command=-26.0, curvature=0.40)

    assert bypass.begin_cycle(
        now=1.2,
        window_active=False,
        new_path_frame=True,
        path_valid=True,
        path_point_count=20,
        path_confidence=0.8,
        raw_command=30.0,
        curve_detection_per_m=-0.4,
    ) == (False, PostRedTurnExitEvent.RESET)
    assert not bypass.state().window_active


def test_bypass_timeout_restores_normal_transition_processing() -> None:
    bypass = PostRedTurnExitBypass(PostRedTurnExitConfig())
    observe(bypass, now=1.0, raw_command=-24.0, curvature=0.45)
    observe(bypass, now=1.1, raw_command=-26.0, curvature=0.40)
    observe(bypass, now=1.2, raw_command=-3.0, curvature=0.18)
    observe(bypass, now=1.3, raw_command=8.0, curvature=0.10)

    assert bypass.begin_cycle(
        now=1.81,
        window_active=True,
        new_path_frame=False,
        path_valid=True,
        path_point_count=20,
        path_confidence=0.8,
        raw_command=12.0,
        curve_detection_per_m=0.1,
    ) == (False, PostRedTurnExitEvent.BYPASS_TIMEOUT)
    assert bypass.state().completed
