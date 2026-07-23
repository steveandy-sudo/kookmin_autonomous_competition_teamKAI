import math
import unittest

from track_drive.final_driver.lane_fallback_input import (
    LaneFallbackDriveCandidate,
    LaneFallbackInputConfig,
    LaneFallbackInputSelector,
    select_fresh_lane_fallback_candidate,
)
from track_drive.mission.mission_types import MissionDecision
from track_drive.mission.states import ControlMode, MissionState


def lane_decision(**overrides):
    values = {
        "mission_state": MissionState.LANE_DRIVING,
        "control_mode": ControlMode.LANE_FALLBACK,
        "selected_source": "lane_fallback",
        "speed_profile": "fallback",
        "stop_required": False,
    }
    values.update(overrides)
    return MissionDecision(**values)


def fresh_select(**overrides):
    values = {
        "decision": lane_decision(),
        "steering_angle_deg": 12.0,
        "command_valid": True,
        "command_receive_sec": 9.9,
        "now_sec": 10.0,
        "config": LaneFallbackInputConfig(fallback_speed=2.0),
    }
    values.update(overrides)
    return select_fresh_lane_fallback_candidate(**values)


class FreshLaneFallbackCandidateTest(unittest.TestCase):
    def test_uses_physical_angle_and_configured_fallback_speed(self):
        candidate = fresh_select()

        self.assertEqual(
            candidate,
            LaneFallbackDriveCandidate(
                steering_angle_deg=12.0,
                requested_speed=2.0,
            ),
        )

    def test_accepts_physical_angle_and_freshness_boundaries(self):
        candidate = fresh_select(
            steering_angle_deg=-26.0,
            command_receive_sec=9.8,
        )

        self.assertEqual(candidate.steering_angle_deg, -26.0)

    def test_rejects_invalid_stale_or_out_of_range_command(self):
        self.assertIsNone(fresh_select(command_valid=False))
        self.assertIsNone(fresh_select(command_receive_sec=9.79))
        self.assertIsNone(fresh_select(steering_angle_deg=26.01))
        self.assertIsNone(fresh_select(steering_angle_deg=math.nan))

    def test_requires_exact_lane_fallback_decision(self):
        self.assertIsNone(
            fresh_select(
                decision=lane_decision(stop_required=True),
            )
        )
        self.assertIsNone(
            fresh_select(
                decision=lane_decision(selected_source="drive_il"),
            )
        )

    def test_fallback_speed_has_no_numeric_default(self):
        with self.assertRaises(TypeError):
            LaneFallbackInputConfig()
        with self.assertRaises(ValueError):
            LaneFallbackInputConfig(fallback_speed=math.nan)


class LaneFallbackInputSelectorTest(unittest.TestCase):
    def setUp(self):
        self.selector = LaneFallbackInputSelector(
            LaneFallbackInputConfig(fallback_speed=2.0)
        )

    def select(self, **overrides):
        values = {
            "decision": lane_decision(),
            "steering_angle_deg": 12.0,
            "command_valid": True,
            "command_receive_sec": 9.9,
            "now_sec": 10.0,
        }
        values.update(overrides)
        return self.selector.select(**values)

    def test_invalid_command_holds_last_steering_and_fallback_speed(self):
        fresh = self.select()
        held = self.select(
            steering_angle_deg=0.0,
            command_valid=False,
            command_receive_sec=10.0,
        )

        self.assertFalse(fresh.steering_held)
        self.assertEqual(
            held,
            LaneFallbackDriveCandidate(
                steering_angle_deg=12.0,
                requested_speed=2.0,
                steering_held=True,
            ),
        )

    def test_selector_has_no_second_grace_timer(self):
        self.select()

        held = self.select(now_sec=10_000.0)

        self.assertTrue(held.steering_held)
        self.assertEqual(held.steering_angle_deg, 12.0)

    def test_first_invalid_command_has_nothing_to_hold(self):
        candidate = self.select(
            command_valid=False,
        )

        self.assertIsNone(candidate)

    def test_recovery_updates_immediately_without_rate_limiting(self):
        self.select(steering_angle_deg=-26.0)

        recovered = self.select(
            steering_angle_deg=26.0,
            command_receive_sec=10.0,
        )

        self.assertFalse(recovered.steering_held)
        self.assertEqual(recovered.steering_angle_deg, 26.0)

    def test_source_change_clears_old_lane_steering(self):
        self.select()
        il_decision = MissionDecision(
            mission_state=MissionState.LANE_DRIVING,
            control_mode=ControlMode.NORMAL_IL,
            selected_source="drive_il",
            speed_profile="normal",
            stop_required=False,
        )

        self.select(decision=il_decision)
        reentered_without_source = self.select(
            steering_angle_deg=None,
            command_valid=False,
            command_receive_sec=None,
            now_sec=11.0,
        )

        self.assertIsNone(reentered_without_source)
        self.assertIsNone(self.selector.last_valid_steering_angle_deg)

    def test_stop_clears_old_lane_steering(self):
        self.select()

        stopped = self.select(
            decision=lane_decision(stop_required=True),
        )

        self.assertIsNone(stopped)
        self.assertIsNone(self.selector.last_valid_steering_angle_deg)


if __name__ == "__main__":
    unittest.main()
