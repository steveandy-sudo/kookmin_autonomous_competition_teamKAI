import math

import pytest

from xycar_map_nav.localization_guard import (
    compose_planar,
    LocalizationJumpGuard,
    PlanarTransform,
)


def make_guard() -> LocalizationJumpGuard:
    return LocalizationJumpGuard(
        maximum_translation_jump_m=0.20,
        maximum_yaw_jump_rad=math.radians(5.0),
        fault_after_sec=0.30,
    )


def test_disabled_guard_tracks_raw_transform():
    guard = make_guard()
    first = PlanarTransform(0.0, 0.0, 0.0)
    second = PlanarTransform(2.0, -1.0, 0.5)

    guard.update(first, now_sec=0.0, enabled=False)
    result = guard.update(second, now_sec=0.1, enabled=False)

    assert result.state == "DISABLED"
    assert result.transform == second


def test_small_localization_corrections_are_accepted():
    guard = make_guard()
    guard.update(
        PlanarTransform(1.0, 2.0, 0.1),
        now_sec=0.0,
        enabled=True,
    )

    result = guard.update(
        PlanarTransform(1.05, 1.98, 0.12),
        now_sec=0.1,
        enabled=True,
    )

    assert result.state == "TRACKING"
    assert result.transform.x == pytest.approx(1.05)
    assert result.translation_residual_m == pytest.approx(
        math.hypot(0.05, -0.02)
    )


def test_one_frame_jump_is_held_then_normal_tracking_recovers():
    guard = make_guard()
    trusted = PlanarTransform(1.0, 2.0, 0.1)
    guard.update(trusted, now_sec=0.0, enabled=True)

    outlier = guard.update(
        PlanarTransform(2.2, 2.0, 0.1),
        now_sec=0.1,
        enabled=True,
    )
    recovered = guard.update(
        PlanarTransform(1.02, 2.01, 0.11),
        now_sec=0.2,
        enabled=True,
    )

    assert outlier.state == "HOLDING"
    assert outlier.transform == trusted
    assert recovered.state == "TRACKING"
    assert not recovered.faulted


def test_persistent_jump_latches_fault_until_reset():
    guard = make_guard()
    trusted = PlanarTransform(1.0, 2.0, 0.1)
    bad = PlanarTransform(2.2, 2.0, 0.1)
    guard.update(trusted, now_sec=0.0, enabled=True)
    guard.update(bad, now_sec=0.1, enabled=True)

    fault = guard.update(bad, now_sec=0.41, enabled=True)
    returned = guard.update(trusted, now_sec=0.5, enabled=True)

    assert fault.state == "FAULT"
    assert fault.transform == trusted
    assert returned.state == "FAULT"

    guard.reset()
    reset = guard.update(bad, now_sec=0.6, enabled=True)
    assert reset.state == "TRACKING"
    assert reset.transform == bad


def test_compose_planar_uses_trusted_map_to_odom_transform():
    map_to_odom = PlanarTransform(1.0, 2.0, math.pi / 2.0)
    odom_to_base = PlanarTransform(2.0, 0.5, -math.pi / 4.0)

    pose = compose_planar(map_to_odom, odom_to_base)

    assert pose.x == pytest.approx(0.5)
    assert pose.y == pytest.approx(4.0)
    assert pose.yaw == pytest.approx(math.pi / 4.0)


def test_jump_limit_uses_induced_vehicle_displacement():
    guard = make_guard()
    local_pose = PlanarTransform(10.0, 0.0, 0.0)
    guard.update(
        PlanarTransform(0.0, 0.0, 0.0),
        now_sec=0.0,
        enabled=True,
        child_to_base=local_pose,
    )

    result = guard.update(
        PlanarTransform(0.0, 0.0, math.radians(4.0)),
        now_sec=0.1,
        enabled=True,
        child_to_base=local_pose,
    )

    assert result.state == "HOLDING"
    assert result.translation_residual_m == pytest.approx(
        2.0 * 10.0 * math.sin(math.radians(2.0))
    )
