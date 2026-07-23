import math
import unittest

from track_drive.final_driver.normal_il_input import (
    NormalIlDriveCandidate,
    NormalIlInputConfig,
    NormalIlInputSelector,
    select_fresh_normal_il_candidate,
)
from track_drive.mission.mission_types import MissionDecision
from track_drive.mission.states import ControlMode, MissionState


def il_decision(**overrides):
    values = {
        "mission_state": MissionState.LANE_DRIVING,
        "control_mode": ControlMode.NORMAL_IL,
        "selected_source": "drive_il",
        "speed_profile": "normal",
        "stop_required": False,
    }
    values.update(overrides)
    return MissionDecision(**values)


def fresh_select(**overrides):
    values = {
        "decision": il_decision(),
        "debug_values": [0.2, 13.0, 11.0, 4.0],
        "debug_receive_sec": 9.9,
        "now_sec": 10.0,
        "config": NormalIlInputConfig(fallback_speed=2.0),
    }
    values.update(overrides)
    return select_fresh_normal_il_candidate(**values)


class FreshNormalIlCandidateTest(unittest.TestCase):
    def test_uses_data_two_and_data_three_without_recalculation(self):
        candidate = fresh_select()

        self.assertEqual(
            candidate,
            NormalIlDriveCandidate(
                steering_command=11.0,
                requested_speed=4.0,
            ),
        )

    def test_does_not_apply_a_numeric_speed_range_or_clamp(self):
        candidate = fresh_select(
            debug_values=[0.0, 0.0, 5.0, 123.0],
        )

        self.assertEqual(candidate.requested_speed, 123.0)

    def test_rejects_only_non_finite_speed_at_numeric_output_boundary(self):
        self.assertIsNone(
            fresh_select(debug_values=[0.0, 0.0, 5.0, math.nan])
        )
        self.assertIsNone(
            fresh_select(debug_values=[0.0, 0.0, 5.0, math.inf])
        )

    def test_rejects_stale_or_invalid_steering_source(self):
        self.assertIsNone(fresh_select(debug_receive_sec=9.49))
        self.assertIsNone(
            fresh_select(debug_values=[0.0, 0.0, 42.01, 4.0])
        )
        self.assertIsNone(
            fresh_select(debug_values=[0.0, 0.0, math.nan, 4.0])
        )

    def test_requires_normal_il_source_and_normal_speed_profile(self):
        self.assertIsNone(
            fresh_select(
                decision=il_decision(speed_profile="fallback"),
            )
        )
        self.assertIsNone(
            fresh_select(
                decision=il_decision(stop_required=True),
            )
        )

    def test_fallback_speed_is_required_and_must_be_finite(self):
        with self.assertRaises(TypeError):
            NormalIlInputConfig()
        with self.assertRaises(ValueError):
            NormalIlInputConfig(fallback_speed=math.nan)


class NormalIlInputSelectorTest(unittest.TestCase):
    def setUp(self):
        self.selector = NormalIlInputSelector(
            NormalIlInputConfig(fallback_speed=2.0)
        )

    def select(self, **overrides):
        values = {
            "decision": il_decision(),
            "debug_values": [0.2, 13.0, 11.0, 4.0],
            "debug_receive_sec": 9.9,
            "now_sec": 10.0,
        }
        values.update(overrides)
        return self.selector.select(**values)

    def test_fallback_profile_holds_last_steering_at_fallback_speed(self):
        fresh = self.select()
        held = self.select(
            decision=il_decision(speed_profile="fallback"),
            debug_values=None,
            debug_receive_sec=None,
            now_sec=10.2,
        )

        self.assertFalse(fresh.steering_held)
        self.assertEqual(
            held,
            NormalIlDriveCandidate(
                steering_command=11.0,
                requested_speed=2.0,
                steering_held=True,
            ),
        )

    def test_stale_data_uses_fallback_even_before_decision_catches_up(self):
        self.select()

        held = self.select(now_sec=20.0)

        self.assertTrue(held.steering_held)
        self.assertEqual(held.steering_command, 11.0)
        self.assertEqual(held.requested_speed, 2.0)

    def test_first_invalid_sample_has_nothing_to_hold(self):
        candidate = self.select(
            decision=il_decision(speed_profile="fallback"),
            debug_values=None,
            debug_receive_sec=None,
        )

        self.assertIsNone(candidate)

    def test_recovery_updates_immediately_without_rate_limiting(self):
        self.select(debug_values=[0.0, 0.0, -42.0, 3.0])

        recovered = self.select(
            debug_values=[0.0, 0.0, 42.0, 5.0],
            debug_receive_sec=10.0,
        )

        self.assertFalse(recovered.steering_held)
        self.assertEqual(recovered.steering_command, 42.0)
        self.assertEqual(recovered.requested_speed, 5.0)

    def test_source_change_clears_old_il_steering(self):
        self.select()
        lane_decision = MissionDecision(
            mission_state=MissionState.LANE_DRIVING,
            control_mode=ControlMode.LANE_FALLBACK,
            selected_source="lane_fallback",
            speed_profile="fallback",
            stop_required=False,
        )

        self.select(decision=lane_decision)
        reentered_without_source = self.select(
            decision=il_decision(speed_profile="fallback"),
            debug_values=None,
            debug_receive_sec=None,
            now_sec=11.0,
        )

        self.assertIsNone(reentered_without_source)
        self.assertIsNone(self.selector.last_valid_steering_command)

    def test_stop_clears_old_il_steering(self):
        self.select()

        stopped = self.select(
            decision=il_decision(stop_required=True),
        )

        self.assertIsNone(stopped)
        self.assertIsNone(self.selector.last_valid_steering_command)


if __name__ == "__main__":
    unittest.main()
