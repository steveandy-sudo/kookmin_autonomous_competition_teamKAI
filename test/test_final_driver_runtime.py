import math
import unittest

from track_drive.final_driver.input_selector import DriveInputSelectorConfig
from track_drive.final_driver.runtime import (
    FinalDriverRuntime,
    FinalDriverRuntimeConfig,
)
from track_drive.mission.mission_types import MissionDecision
from track_drive.mission.states import ControlMode, MissionState


def decision(
    *,
    mode: ControlMode,
    source: str,
    speed_profile: str,
    stop_required: bool = False,
) -> MissionDecision:
    return MissionDecision(
        mission_state=MissionState.LANE_DRIVING,
        control_mode=mode,
        selected_source=source,
        speed_profile=speed_profile,
        stop_required=stop_required,
    )


class FinalDriverRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.runtime = FinalDriverRuntime(
            FinalDriverRuntimeConfig(
                input_selector=DriveInputSelectorConfig(
                    fallback_speed=10.0
                ),
                decision_timeout_sec=0.2,
            )
        )

    def test_normal_il_is_selected_directly(self):
        self.runtime.update_il(
            [0.0, 0.0, 12.5, 18.0],
            receive_sec=1.0,
        )
        self.runtime.update_decision(
            decision(
                mode=ControlMode.NORMAL_IL,
                source="drive_il",
                speed_profile="normal",
            ),
            receive_sec=1.0,
        )

        command = self.runtime.command(now_sec=1.01)

        self.assertTrue(command.valid)
        self.assertEqual(command.steering_command, 12.5)
        self.assertEqual(command.requested_speed, 18.0)
        self.assertEqual(command.selected_source, "drive_il")

    def test_lane_physical_angle_is_converted(self):
        self.runtime.update_lane(
            steering_angle_deg=10.0,
            command_valid=True,
            receive_sec=1.0,
        )
        self.runtime.update_decision(
            decision(
                mode=ControlMode.LANE_FALLBACK,
                source="lane_fallback",
                speed_profile="fallback",
            ),
            receive_sec=1.0,
        )

        command = self.runtime.command(now_sec=1.01)

        self.assertTrue(command.valid)
        self.assertEqual(command.steering_command, 20.0)
        self.assertEqual(command.requested_speed, 10.0)

    def test_cone_speed_and_physical_angle_are_selected(self):
        self.runtime.update_cone(
            [4.0, 12.0, 0.8],
            receive_sec=1.0,
        )
        self.runtime.update_decision(
            MissionDecision(
                mission_state=MissionState.CONE_SECTION,
                control_mode=ControlMode.CONE_DRIVE_RULE,
                selected_source="cone_rule",
                speed_profile="cone",
                stop_required=False,
            ),
            receive_sec=1.0,
        )

        command = self.runtime.command(now_sec=1.01)

        self.assertTrue(command.valid)
        self.assertEqual(command.steering_command, 10.0)
        self.assertEqual(command.requested_speed, 12.0)

    def test_stop_decision_returns_neutral_invalid_command(self):
        self.runtime.update_decision(
            decision(
                mode=ControlMode.STOP,
                source="none",
                speed_profile="stop",
                stop_required=True,
            ),
            receive_sec=1.0,
        )

        command = self.runtime.command(now_sec=1.01)

        self.assertFalse(command.valid)
        self.assertEqual(command.steering_command, 0.0)
        self.assertEqual(command.requested_speed, 0.0)

    def test_stale_decision_stops_even_when_source_is_fresh(self):
        self.runtime.update_il(
            [0.0, 0.0, 12.5, 18.0],
            receive_sec=1.19,
        )
        self.runtime.update_decision(
            decision(
                mode=ControlMode.NORMAL_IL,
                source="drive_il",
                speed_profile="normal",
            ),
            receive_sec=1.0,
        )

        command = self.runtime.command(now_sec=1.21)

        self.assertFalse(command.valid)

    def test_invalid_receive_time_is_rejected(self):
        with self.assertRaises(ValueError):
            self.runtime.update_il(
                [0.0, 0.0, 0.0, 0.0],
                receive_sec=math.nan,
            )


if __name__ == "__main__":
    unittest.main()
