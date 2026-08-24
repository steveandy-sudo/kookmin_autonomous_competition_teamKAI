import math

import pytest

from xycar_parking_nav.mission_core import (
    ForwardProgressWatchdog,
    LocalizationGate,
    LocalizationGateConfig,
    Pose2D,
    assess_transit_waypoint_pass,
    assess_mission_time,
    direct_reverse_parking_command,
    is_reverse_fallback_candidate,
    mission_steps_from_dicts,
    pose_error,
    reference_pose_to_base,
)


def covariance(xy=0.01, yaw=0.02):
    result = [0.0] * 36
    result[0] = xy
    result[7] = xy
    result[35] = yaw
    return result


@pytest.mark.parametrize(
    "pose, expected",
    [
        (Pose2D(0.0, 4.2, 0.0), Pose2D(0.16, 4.2, 0.0)),
        (Pose2D(2.1, 3.3, -math.pi / 2.0), Pose2D(2.1, 3.14, -math.pi / 2.0)),
        (Pose2D(1.8, 0.9, math.pi), Pose2D(1.64, 0.9, math.pi)),
    ],
)
def test_reference_pose_is_shifted_along_heading(pose, expected):
    actual = reference_pose_to_base(pose, 0.16)
    assert actual.x == pytest.approx(expected.x)
    assert actual.y == pytest.approx(expected.y)
    assert pose_error(actual, expected) == pytest.approx((0.0, 0.0))


def test_direct_reverse_parking_tracks_a_goal_straight_behind():
    command = direct_reverse_parking_command(
        current=Pose2D(2.126, 2.018, -1.549),
        target=Pose2D(2.059, 3.186, -1.503),
        speed_mps=0.322448,
        position_tolerance_m=0.16,
        yaw_tolerance_rad=0.18,
        heading_gain=0.8,
        maximum_curvature=1.2,
    )

    assert command.linear_x == pytest.approx(-0.322448)
    assert command.angular_z > 0.0
    assert command.distance_m == pytest.approx(1.17, abs=0.01)
    assert not command.reached


def test_direct_reverse_parking_stops_only_inside_pose_tolerances():
    command = direct_reverse_parking_command(
        current=Pose2D(2.08, 3.10, -1.55),
        target=Pose2D(2.059, 3.186, -1.503),
        speed_mps=0.322448,
        position_tolerance_m=0.16,
        yaw_tolerance_rad=0.18,
        heading_gain=0.8,
        maximum_curvature=1.2,
    )

    assert command.reached
    assert command.linear_x == 0.0
    assert command.angular_z == 0.0


def test_localization_requires_consecutive_good_samples():
    gate = LocalizationGate(
        LocalizationGateConfig(required_stable_samples=3)
    )
    pose = Pose2D(1.0, 2.0, 0.2)
    assert not gate.update(
        pose=pose, covariance=covariance(), stamp_sec=1.0, now_sec=1.01
    ).ready
    assert not gate.update(
        pose=Pose2D(1.01, 2.0, 0.2),
        covariance=covariance(),
        stamp_sec=1.1,
        now_sec=1.11,
    ).ready
    assert gate.update(
        pose=Pose2D(1.02, 2.0, 0.2),
        covariance=covariance(),
        stamp_sec=1.2,
        now_sec=1.21,
    ).ready


def test_localization_jump_resets_stability():
    gate = LocalizationGate(
        LocalizationGateConfig(required_stable_samples=2)
    )
    gate.update(
        pose=Pose2D(0.0, 0.0, 0.0),
        covariance=covariance(),
        stamp_sec=1.0,
        now_sec=1.0,
    )
    assessment = gate.update(
        pose=Pose2D(1.0, 0.0, 0.0),
        covariance=covariance(),
        stamp_sec=1.1,
        now_sec=1.1,
    )
    assert assessment.reason == "position_jump"
    assert assessment.stable_samples == 0


def test_localization_age_fails_closed():
    gate = LocalizationGate(
        LocalizationGateConfig(required_stable_samples=1)
    )
    assert gate.update(
        pose=Pose2D(0.0, 0.0, 0.0),
        covariance=covariance(),
        stamp_sec=1.0,
        now_sec=1.0,
    ).ready
    assert gate.age_assessment(5.1).reason == "stale_pose"
    assert not gate.age_assessment(5.1).ready


def test_mission_parser_rejects_duplicate_names():
    with pytest.raises(ValueError, match="unique"):
        mission_steps_from_dicts(
            [
                {"name": "A", "x": 0, "y": 0, "yaw": 0},
                {"name": "A", "x": 1, "y": 0, "yaw": 0},
            ]
        )


def test_mission_parser_defaults_to_forward_and_marks_parking_reverse():
    steps = mission_steps_from_dicts(
        [
            {"name": "TRANSIT", "x": 0, "y": 0, "yaw": 0},
            {
                "name": "PARK_REVERSE",
                "x": 1,
                "y": 0,
                "yaw": 0,
                "allow_reverse": True,
                "precise_goal": True,
            },
        ]
    )
    assert steps[0].allow_reverse is False
    assert steps[0].precise_goal is False
    assert steps[1].allow_reverse is True
    assert steps[1].precise_goal is True


def test_reverse_only_step_requires_reverse_and_is_preserved():
    step = mission_steps_from_dicts(
        [
            {
                "name": "A_REVERSE_ALIGN",
                "x": 0.5,
                "y": 4.1,
                "yaw": 0.02,
                "allow_reverse": True,
                "reverse_only": True,
            }
        ]
    )[0]

    assert step.allow_reverse is True
    assert step.reverse_only is True

    with pytest.raises(ValueError, match="requires allow_reverse"):
        mission_steps_from_dicts(
            [
                {
                    "name": "INVALID_REVERSE_ONLY",
                    "x": 0.0,
                    "y": 0.0,
                    "yaw": 0.0,
                    "reverse_only": True,
                }
            ]
        )


def test_reverse_fallback_is_limited_to_ordinary_forward_transit():
    steps = mission_steps_from_dicts(
        [
            {"name": "TRANSIT", "x": 0, "y": 0, "yaw": 0},
            {
                "name": "PRECISE",
                "x": 1,
                "y": 0,
                "yaw": 0,
                "precise_goal": True,
            },
            {
                "name": "REVERSE",
                "x": 2,
                "y": 0,
                "yaw": 0,
                "allow_reverse": True,
            },
            {
                "name": "PARK",
                "x": 3,
                "y": 0,
                "yaw": 0,
                "parking_goal": True,
            },
        ]
    )

    assert is_reverse_fallback_candidate(steps[0])
    assert not is_reverse_fallback_candidate(steps[1])
    assert not is_reverse_fallback_candidate(steps[2])
    assert not is_reverse_fallback_candidate(steps[3])


def test_forward_progress_watchdog_requests_reverse_after_stall():
    watchdog = ForwardProgressWatchdog(
        timeout_sec=3.0,
        minimum_improvement_m=0.08,
    )

    assert not watchdog.update(distance_m=2.0, now_sec=10.0)
    assert not watchdog.update(distance_m=1.95, now_sec=12.9)
    assert watchdog.update(distance_m=1.94, now_sec=13.0)

    watchdog.reset()
    assert not watchdog.update(distance_m=2.0, now_sec=20.0)
    assert not watchdog.update(distance_m=1.90, now_sec=22.9)
    assert not watchdog.update(distance_m=1.85, now_sec=25.8)
    assert watchdog.update(distance_m=1.84, now_sec=25.9)


def test_transit_waypoint_accepts_radius_or_crossed_gate_but_not_wide_miss():
    previous = Pose2D(0.0, 0.0, 0.0)
    target = Pose2D(1.0, 0.0, 0.0)

    radius = assess_transit_waypoint_pass(
        current=Pose2D(0.70, 0.10, 1.0),
        previous=previous,
        target=target,
        radius_m=0.35,
        maximum_miss_distance_m=0.60,
        lateral_tolerance_m=0.45,
    )
    crossed = assess_transit_waypoint_pass(
        current=Pose2D(1.20, 0.30, -1.0),
        previous=previous,
        target=target,
        radius_m=0.35,
        maximum_miss_distance_m=0.60,
        lateral_tolerance_m=0.45,
    )
    wide = assess_transit_waypoint_pass(
        current=Pose2D(1.20, 0.55, 0.0),
        previous=previous,
        target=target,
        radius_m=0.35,
        maximum_miss_distance_m=0.60,
        lateral_tolerance_m=0.45,
    )

    assert radius.passed and radius.reason == "radius"
    assert crossed.passed and crossed.reason == "crossed_gate"
    assert not wide.passed


def test_three_minute_clock_warns_expires_and_freezes_at_finish():
    normal = assess_mission_time(
        started_at_sec=100.0,
        now_sec=249.9,
        limit_sec=180.0,
        warning_remaining_sec=30.0,
    )
    warning = assess_mission_time(
        started_at_sec=100.0,
        now_sec=250.0,
        limit_sec=180.0,
        warning_remaining_sec=30.0,
    )
    expired = assess_mission_time(
        started_at_sec=100.0,
        now_sec=280.0,
        limit_sec=180.0,
        warning_remaining_sec=30.0,
    )
    frozen = assess_mission_time(
        started_at_sec=100.0,
        now_sec=400.0,
        limit_sec=180.0,
        warning_remaining_sec=30.0,
        finished_at_sec=212.9,
    )

    assert normal.elapsed_sec == pytest.approx(149.9)
    assert not normal.warning and not normal.expired
    assert warning.warning and not warning.expired
    assert expired.expired and expired.remaining_sec == 0.0
    assert frozen.elapsed_sec == pytest.approx(112.9)
    assert frozen.remaining_sec == pytest.approx(67.1)
    assert not frozen.expired
