import pytest

from xycar_map_nav.sequential_hybrid_driver import (
    cone_lane_handoff_command,
    cone_recovery_target,
)


def test_cone_recovery_holds_last_command_before_timeout():
    angle, speed, phase = cone_recovery_target(
        last_angle_deg=20.0,
        last_speed_command=8.0,
        age_sec=0.7,
        hold_timeout_sec=0.8,
        creep_timeout_sec=1.8,
        creep_speed_command=8.0,
        minimum_speed_command=3.0,
        final_steering_retention=0.70,
    )
    assert angle == pytest.approx(20.0)
    assert speed == pytest.approx(8.0)
    assert phase == "hold_last_cone"


def test_cone_recovery_reduces_steering_during_creep():
    angle, speed, phase = cone_recovery_target(
        last_angle_deg=20.0,
        last_speed_command=6.0,
        age_sec=1.3,
        hold_timeout_sec=0.8,
        creep_timeout_sec=1.8,
        creep_speed_command=8.0,
        minimum_speed_command=3.0,
        final_steering_retention=0.70,
    )
    assert angle == pytest.approx(17.0)
    assert speed == pytest.approx(8.0)
    assert phase == "cone_exit_creep"


def test_cone_recovery_stops_after_creep_without_lane():
    assert cone_recovery_target(
        last_angle_deg=20.0,
        last_speed_command=8.0,
        age_sec=1.81,
        hold_timeout_sec=0.8,
        creep_timeout_sec=1.8,
        creep_speed_command=8.0,
        minimum_speed_command=3.0,
        final_steering_retention=0.70,
    ) == (0.0, 0.0, "waiting_lane_recovery")


def test_cone_handoff_uses_smoothstep_and_speed_cap():
    angle, speed, progress = cone_lane_handoff_command(
        from_angle_command=30.0,
        from_speed_command=8.0,
        lane_angle_command=-10.0,
        lane_speed_command=18.0,
        elapsed_sec=0.325,
        duration_sec=0.65,
        maximum_speed_command=8.0,
        minimum_speed_command=3.0,
    )
    assert progress == pytest.approx(0.5)
    assert angle == pytest.approx(10.0)
    assert speed == pytest.approx(8.0)
