import pytest

from xycar_map_nav.s_curve_entry_guard import green_car_avoidance_completed
from xycar_map_nav.s_curve_entry_guard import SCurveEntryEvent
from xycar_map_nav.s_curve_entry_guard import SCurveEntryGuard
from xycar_map_nav.s_curve_entry_guard import SCurveEntryGuardConfig
from xycar_map_nav.s_curve_entry_guard import SCurveEntryTrigger


def update(
    guard: SCurveEntryGuard,
    *,
    angle: float,
    speed: float,
    vehicle_speed: float = 0.8,
    dt: float = 0.25,
) -> SCurveEntryEvent:
    return guard.update(
        dt_sec=dt,
        vehicle_speed_mps=vehicle_speed,
        vehicle_speed_fresh=True,
        rule_angle_command=angle,
        rule_speed_command=speed,
        rule_command_fresh=True,
    )


def test_only_completed_green_car_avoidance_is_a_trigger() -> None:
    assert green_car_avoidance_completed(
        previous_controls_vehicle=True,
        previous_target_class_name="green_car",
        controls_vehicle=False,
    )
    assert not green_car_avoidance_completed(
        previous_controls_vehicle=True,
        previous_target_class_name="red_car",
        controls_vehicle=False,
    )
    assert not green_car_avoidance_completed(
        previous_controls_vehicle=False,
        previous_target_class_name="green_car",
        controls_vehicle=False,
    )
    assert not green_car_avoidance_completed(
        previous_controls_vehicle=True,
        previous_target_class_name="green_car",
        controls_vehicle=True,
    )


@pytest.mark.parametrize(
    "trigger",
    [
        SCurveEntryTrigger.SHORTCUT_EXIT,
        SCurveEntryTrigger.GREEN_CAR_EXIT,
    ],
)
def test_both_explicit_mission_exits_start_command_11_guard(trigger) -> None:
    guard = SCurveEntryGuard(SCurveEntryGuardConfig())

    assert guard.start(trigger) == SCurveEntryEvent.STARTED
    angle, speed = guard.limit_command(angle_command=2.0, speed_command=25.0)

    assert angle == 2.0
    assert speed == 11.0
    assert guard.state().trigger == trigger


def test_no_trigger_leaves_an_ordinary_corner_unchanged() -> None:
    guard = SCurveEntryGuard(SCurveEntryGuardConfig())

    angle, speed = guard.limit_command(
        angle_command=-28.0,
        speed_command=25.0,
    )

    assert (angle, speed) == (-28.0, 25.0)


def test_shortcut_residual_left_does_not_release_before_straight() -> None:
    guard = SCurveEntryGuard(SCurveEntryGuardConfig())
    guard.start(SCurveEntryTrigger.SHORTCUT_EXIT)

    for _ in range(8):
        assert update(guard, angle=-20.0, speed=11.0) == SCurveEntryEvent.NONE

    state = guard.state()
    assert state.distance_m == pytest.approx(1.6)
    assert not state.straight_ready
    assert state.active


@pytest.mark.parametrize("right_correction", [2.4, 17.2, 22.8])
def test_center_and_left_offset_right_corrections_are_preserved(
    right_correction: float,
) -> None:
    guard = SCurveEntryGuard(SCurveEntryGuardConfig())
    guard.start(SCurveEntryTrigger.GREEN_CAR_EXIT)

    # Three centered command-25 frames establish the post-mission straight.
    for _ in range(3):
        update(guard, angle=1.0, speed=25.0)
    assert guard.state().straight_ready

    early_angle, early_speed = guard.limit_command(
        angle_command=right_correction,
        speed_command=25.0,
    )
    assert (early_angle, early_speed) == (right_correction, 11.0)

    while guard.state().distance_m < 1.50:
        update(guard, angle=1.0, speed=25.0)
    angle, speed = guard.limit_command(
        angle_command=right_correction,
        speed_command=25.0,
    )

    assert angle == right_correction
    assert speed == 11.0


def test_normal_rule_curve_takes_over_after_three_left_frames() -> None:
    guard = SCurveEntryGuard(SCurveEntryGuardConfig())
    guard.start(SCurveEntryTrigger.GREEN_CAR_EXIT)

    while guard.state().distance_m < 1.50:
        update(guard, angle=0.0, speed=25.0)
    assert guard.state().straight_ready

    assert update(guard, angle=-10.0, speed=11.0) == SCurveEntryEvent.NONE
    assert update(guard, angle=-13.0, speed=11.0) == SCurveEntryEvent.NONE
    assert (
        update(guard, angle=-18.0, speed=11.0)
        == SCurveEntryEvent.CURVE_HANDOFF
    )
    assert not guard.state().active

    # Once normal curve mode owns the car, this feature no longer modifies it.
    assert guard.limit_command(
        angle_command=-25.0,
        speed_command=11.0,
    ) == (-25.0, 11.0)


def test_guard_never_raises_a_lower_safety_speed() -> None:
    guard = SCurveEntryGuard(SCurveEntryGuardConfig())
    guard.start(SCurveEntryTrigger.SHORTCUT_EXIT)

    assert guard.limit_command(
        angle_command=0.0,
        speed_command=8.0,
    ) == (0.0, 8.0)
    assert guard.limit_command(
        angle_command=0.0,
        speed_command=0.0,
    ) == (0.0, 0.0)


def test_overdue_guard_stays_slow_instead_of_reaccelerating() -> None:
    guard = SCurveEntryGuard(
        SCurveEntryGuardConfig(overdue_distance_m=1.0)
    )
    guard.start(SCurveEntryTrigger.GREEN_CAR_EXIT)

    for _ in range(6):
        update(guard, angle=0.0, speed=25.0)

    assert guard.state().overdue
    assert guard.state().active
    assert guard.limit_command(
        angle_command=0.0,
        speed_command=25.0,
    ) == (0.0, 11.0)
