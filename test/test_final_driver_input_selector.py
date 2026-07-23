import unittest

from track_drive.final_driver.input_selector import (
    DriveInputSelection,
    DriveInputSelector,
    DriveInputSelectorConfig,
    DriveInputSnapshot,
    SteeringUnit,
)
from track_drive.mission.mission_types import MissionDecision
from track_drive.mission.states import ControlMode, MissionState


def decision_for(mode):
    contracts = {
        ControlMode.NORMAL_IL: (
            MissionState.LANE_DRIVING,
            "drive_il",
            "normal",
        ),
        ControlMode.LANE_FALLBACK: (
            MissionState.LANE_DRIVING,
            "lane_fallback",
            "fallback",
        ),
        ControlMode.CONE_DRIVE_RULE: (
            MissionState.CONE_SECTION,
            "cone_rule",
            "cone",
        ),
        ControlMode.STOP: (
            MissionState.LANE_DRIVING,
            "none",
            "stop",
        ),
    }
    mission_state, source, speed_profile = contracts[mode]
    return MissionDecision(
        mission_state=mission_state,
        control_mode=mode,
        selected_source=source,
        speed_profile=speed_profile,
        stop_required=mode is ControlMode.STOP,
    )


def all_inputs(**overrides):
    values = {
        "now_sec": 10.0,
        "il_debug_values": [0.1, 8.0, 7.0, 4.0],
        "il_debug_receive_sec": 9.9,
        "lane_steering_angle_deg": -12.0,
        "lane_command_valid": True,
        "lane_command_receive_sec": 9.9,
        "cone_command_values": [20.0, 17.0, 0.8],
        "cone_command_receive_sec": 9.9,
    }
    values.update(overrides)
    return DriveInputSnapshot(**values)


class DriveInputSelectionContractTest(unittest.TestCase):
    def test_invalid_selection_has_explicit_neutral_contract(self):
        selection = DriveInputSelection.invalid("drive_il")

        self.assertFalse(selection.valid)
        self.assertEqual(selection.steering_value, 0.0)
        self.assertIs(selection.steering_unit, SteeringUnit.NONE)
        self.assertEqual(selection.requested_speed, 0.0)
        self.assertEqual(selection.selected_source, "drive_il")

    def test_rejects_invalid_selection_with_non_neutral_values(self):
        with self.assertRaises(ValueError):
            DriveInputSelection(
                steering_value=1.0,
                steering_unit=SteeringUnit.NONE,
                requested_speed=0.0,
                selected_source="none",
                steering_held=False,
                valid=False,
            )


class DriveInputSelectorTest(unittest.TestCase):
    def setUp(self):
        self.selector = DriveInputSelector(
            DriveInputSelectorConfig(fallback_speed=2.0)
        )

    def test_normal_il_keeps_xycar_command_unit(self):
        selected = self.selector.select(
            decision=decision_for(ControlMode.NORMAL_IL),
            inputs=all_inputs(),
        )

        self.assertEqual(
            selected,
            DriveInputSelection(
                steering_value=7.0,
                steering_unit=SteeringUnit.XYCAR_COMMAND,
                requested_speed=4.0,
                selected_source="drive_il",
                steering_held=False,
                valid=True,
            ),
        )

    def test_lane_fallback_keeps_physical_degree_unit(self):
        selected = self.selector.select(
            decision=decision_for(ControlMode.LANE_FALLBACK),
            inputs=all_inputs(),
        )

        self.assertTrue(selected.valid)
        self.assertEqual(selected.steering_value, -12.0)
        self.assertIs(
            selected.steering_unit,
            SteeringUnit.PHYSICAL_DEG,
        )
        self.assertEqual(selected.requested_speed, 2.0)
        self.assertEqual(selected.selected_source, "lane_fallback")

    def test_cone_keeps_physical_degree_and_source_requested_speed(self):
        selected = self.selector.select(
            decision=decision_for(ControlMode.CONE_DRIVE_RULE),
            inputs=all_inputs(),
        )

        self.assertTrue(selected.valid)
        self.assertEqual(selected.steering_value, 20.0)
        self.assertIs(
            selected.steering_unit,
            SteeringUnit.PHYSICAL_DEG,
        )
        self.assertEqual(selected.requested_speed, 17.0)
        self.assertEqual(selected.selected_source, "cone_rule")

    def test_only_mission_selected_source_can_win(self):
        selected = self.selector.select(
            decision=decision_for(ControlMode.CONE_DRIVE_RULE),
            inputs=all_inputs(
                il_debug_values=[0.0, 0.0, 40.0, 100.0],
                lane_steering_angle_deg=25.0,
                cone_command_values=[3.0, 11.0, 0.7],
            ),
        )

        self.assertEqual(selected.selected_source, "cone_rule")
        self.assertEqual(selected.steering_value, 3.0)
        self.assertEqual(selected.requested_speed, 11.0)

    def test_stop_returns_no_driving_candidate(self):
        selected = self.selector.select(
            decision=decision_for(ControlMode.STOP),
            inputs=all_inputs(),
        )

        self.assertEqual(selected, DriveInputSelection.invalid())

    def test_missing_selected_candidate_is_explicitly_invalid(self):
        selected = self.selector.select(
            decision=decision_for(ControlMode.NORMAL_IL),
            inputs=all_inputs(
                il_debug_values=None,
                il_debug_receive_sec=None,
            ),
        )

        self.assertFalse(selected.valid)
        self.assertEqual(selected.selected_source, "drive_il")
        self.assertIs(selected.steering_unit, SteeringUnit.NONE)

    def test_future_unimplemented_source_is_invalid_without_crashing(self):
        future_decision = MissionDecision(
            mission_state=MissionState.FIXED_OBSTACLE_SECTION,
            control_mode=ControlMode.FIXED_OBSTACLE_RULE,
            selected_source="fixed_obstacle_rule",
            speed_profile="obstacle",
            stop_required=False,
        )

        selected = self.selector.select(
            decision=future_decision,
            inputs=all_inputs(),
        )

        self.assertFalse(selected.valid)
        self.assertEqual(
            selected.selected_source,
            "fixed_obstacle_rule",
        )

    def test_one_fallback_speed_is_shared_by_il_and_lane(self):
        self.selector.select(
            decision=decision_for(ControlMode.NORMAL_IL),
            inputs=all_inputs(),
        )
        il_fallback_decision = MissionDecision(
            mission_state=MissionState.LANE_DRIVING,
            control_mode=ControlMode.NORMAL_IL,
            selected_source="drive_il",
            speed_profile="fallback",
            stop_required=False,
        )
        il_held = self.selector.select(
            decision=il_fallback_decision,
            inputs=all_inputs(
                il_debug_values=None,
                il_debug_receive_sec=None,
            ),
        )
        lane = self.selector.select(
            decision=decision_for(ControlMode.LANE_FALLBACK),
            inputs=all_inputs(),
        )

        self.assertEqual(il_held.requested_speed, 2.0)
        self.assertEqual(lane.requested_speed, 2.0)

    def test_steering_memory_never_crosses_between_sources(self):
        self.selector.select(
            decision=decision_for(ControlMode.NORMAL_IL),
            inputs=all_inputs(),
        )

        lane_missing = self.selector.select(
            decision=decision_for(ControlMode.LANE_FALLBACK),
            inputs=all_inputs(
                lane_steering_angle_deg=None,
                lane_command_valid=False,
                lane_command_receive_sec=None,
            ),
        )

        self.assertFalse(lane_missing.valid)
        self.assertEqual(lane_missing.selected_source, "lane_fallback")

    def test_stop_clears_source_memory_before_reentry(self):
        self.selector.select(
            decision=decision_for(ControlMode.CONE_DRIVE_RULE),
            inputs=all_inputs(),
        )
        self.selector.select(
            decision=decision_for(ControlMode.STOP),
            inputs=all_inputs(),
        )

        reentered = self.selector.select(
            decision=decision_for(ControlMode.CONE_DRIVE_RULE),
            inputs=all_inputs(
                cone_command_values=None,
                cone_command_receive_sec=None,
            ),
        )

        self.assertFalse(reentered.valid)
        self.assertEqual(reentered.selected_source, "cone_rule")

    def test_fallback_speed_has_no_default(self):
        with self.assertRaises(TypeError):
            DriveInputSelectorConfig()


if __name__ == "__main__":
    unittest.main()
