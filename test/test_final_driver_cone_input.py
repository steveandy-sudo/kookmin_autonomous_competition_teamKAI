import math
import unittest

from track_drive.final_driver.cone_input import (
    ConeDriveCandidate,
    ConeInputConfig,
    ConeInputSelector,
    select_cone_drive_candidate,
)
from track_drive.mission.mission_types import MissionDecision
from track_drive.mission.states import ControlMode, MissionState


def cone_decision(**overrides):
    values = {
        "mission_state": MissionState.CONE_SECTION,
        "control_mode": ControlMode.CONE_DRIVE_RULE,
        "selected_source": "cone_rule",
        "speed_profile": "cone",
        "stop_required": False,
    }
    values.update(overrides)
    return MissionDecision(**values)


def select(**overrides):
    values = {
        "decision": cone_decision(),
        "command_values": [12.0, 17.0, 0.75],
        "command_receive_sec": 9.9,
        "now_sec": 10.0,
    }
    values.update(overrides)
    return select_cone_drive_candidate(**values)


class FinalDriverConeInputTest(unittest.TestCase):
    def test_uses_data_one_as_requested_speed_without_recalculation(self):
        candidate = select()

        self.assertEqual(
            candidate,
            ConeDriveCandidate(
                steering_angle_deg=12.0,
                requested_speed=17.0,
                confidence=0.75,
            ),
        )

    def test_caps_only_speed_above_vehicle_tested_maximum(self):
        candidate = select(command_values=[3.0, 25.0, 0.8])

        self.assertEqual(candidate.requested_speed, 21.0)

    def test_rejects_zero_or_below_minimum_speed_instead_of_accelerating(self):
        self.assertIsNone(select(command_values=[0.0, 0.0, 0.0]))
        self.assertIsNone(select(command_values=[3.0, 9.4, 0.8]))

    def test_rejects_stale_low_confidence_or_out_of_range_steering(self):
        self.assertIsNone(select(command_receive_sec=9.79))
        self.assertIsNone(
            select(command_values=[3.0, 17.0, 0.2])
        )
        self.assertIsNone(
            select(command_values=[26.1, 17.0, 0.8])
        )
        self.assertIsNone(
            select(command_values=[math.nan, 17.0, 0.8])
        )

    def test_accepts_source_and_range_boundaries(self):
        candidate = select(
            command_values=[-26.0, 9.5, 0.2001],
            command_receive_sec=9.8,
        )

        self.assertEqual(candidate.steering_angle_deg, -26.0)
        self.assertEqual(candidate.requested_speed, 9.5)

    def test_stop_has_priority_over_valid_cone_command(self):
        self.assertIsNone(
            select(decision=cone_decision(stop_required=True))
        )

    def test_rejects_candidate_when_mission_selects_another_source(self):
        normal_decision = MissionDecision(
            mission_state=MissionState.LANE_DRIVING,
            control_mode=ControlMode.NORMAL_IL,
            selected_source="drive_il",
            speed_profile="normal",
            stop_required=False,
        )

        self.assertIsNone(select(decision=normal_decision))

    def test_configuration_rejects_unsafe_ranges(self):
        with self.assertRaises(ValueError):
            ConeInputConfig(minimum_requested_speed=22.0)
        with self.assertRaises(ValueError):
            ConeInputConfig(source_timeout_sec=-0.1)


class ConeInputSelectorTest(unittest.TestCase):
    def setUp(self):
        self.selector = ConeInputSelector()

    def select(self, **overrides):
        values = {
            "decision": cone_decision(),
            "command_values": [12.0, 17.0, 0.75],
            "command_receive_sec": 9.9,
            "now_sec": 10.0,
        }
        values.update(overrides)
        return self.selector.select(**values)

    def test_stale_input_holds_last_steering_at_minimum_cone_speed(self):
        fresh = self.select()
        held = self.select(now_sec=20.0)

        self.assertFalse(fresh.steering_held)
        self.assertEqual(
            held,
            ConeDriveCandidate(
                steering_angle_deg=12.0,
                requested_speed=9.5,
                confidence=0.0,
                steering_held=True,
            ),
        )

    def test_hold_has_no_automatic_timeout(self):
        self.select()

        held = self.select(now_sec=10_000.0)

        self.assertTrue(held.steering_held)
        self.assertEqual(held.steering_angle_deg, 12.0)
        self.assertEqual(held.requested_speed, 9.5)

    def test_invalid_input_cannot_replace_last_valid_steering(self):
        self.select()

        held = self.select(
            command_values=[-25.0, 0.0, 0.0],
            command_receive_sec=10.0,
        )

        self.assertTrue(held.steering_held)
        self.assertEqual(held.steering_angle_deg, 12.0)

    def test_first_invalid_input_produces_no_candidate(self):
        candidate = self.select(
            command_values=[0.0, 0.0, 0.0],
            command_receive_sec=10.0,
        )

        self.assertIsNone(candidate)
        self.assertIsNone(self.selector.last_valid_steering_angle_deg)

    def test_new_valid_command_replaces_memory_without_rate_limiting(self):
        self.select(command_values=[-26.0, 17.0, 0.8])

        changed = self.select(
            command_values=[26.0, 18.0, 0.8],
            command_receive_sec=10.0,
        )

        self.assertFalse(changed.steering_held)
        self.assertEqual(changed.steering_angle_deg, 26.0)
        self.assertEqual(changed.requested_speed, 18.0)

    def test_leaving_cone_mode_clears_old_steering(self):
        self.select()
        normal_decision = MissionDecision(
            mission_state=MissionState.LANE_DRIVING,
            control_mode=ControlMode.NORMAL_IL,
            selected_source="drive_il",
            speed_profile="normal",
            stop_required=False,
        )

        self.select(decision=normal_decision)
        reentered_without_source = self.select(
            command_values=None,
            command_receive_sec=None,
            now_sec=11.0,
        )

        self.assertIsNone(reentered_without_source)
        self.assertIsNone(self.selector.last_valid_steering_angle_deg)

    def test_stop_decision_clears_old_steering(self):
        self.select()

        stopped = self.select(
            decision=cone_decision(stop_required=True),
        )

        self.assertIsNone(stopped)
        self.assertIsNone(self.selector.last_valid_steering_angle_deg)


if __name__ == "__main__":
    unittest.main()
