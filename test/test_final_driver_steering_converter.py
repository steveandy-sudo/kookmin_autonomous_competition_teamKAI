import math
import unittest

from track_drive.final_driver.input_selector import (
    DriveInputSelection,
    SteeringUnit,
)
from track_drive.final_driver.steering_converter import (
    FinalDriveCommandCandidate,
    SteeringCalibration,
    convert_drive_input,
    physical_deg_to_xycar_command,
)


def valid_selection(**overrides):
    values = {
        "steering_value": 13.0,
        "steering_unit": SteeringUnit.PHYSICAL_DEG,
        "requested_speed": 17.0,
        "selected_source": "cone_rule",
        "steering_held": False,
        "valid": True,
    }
    values.update(overrides)
    return DriveInputSelection(**values)


class PhysicalSteeringConversionTest(unittest.TestCase):
    def test_matches_vehicle_tested_calibration_points(self):
        expected = {
            0.0: 0.0,
            4.0: 10.0,
            10.0: 20.0,
            16.0: 30.0,
            26.0: 42.0,
        }

        for angle, command in expected.items():
            with self.subTest(angle=angle):
                self.assertEqual(
                    physical_deg_to_xycar_command(angle),
                    command,
                )
                self.assertEqual(
                    physical_deg_to_xycar_command(-angle),
                    -command,
                )

    def test_linearly_interpolates_between_calibration_points(self):
        expected = {
            2.0: 5.0,
            7.0: 15.0,
            13.0: 25.0,
            21.0: 36.0,
        }

        for angle, command in expected.items():
            with self.subTest(angle=angle):
                self.assertAlmostEqual(
                    physical_deg_to_xycar_command(angle),
                    command,
                )

    def test_clamps_physical_angle_to_last_calibration_point(self):
        self.assertEqual(
            physical_deg_to_xycar_command(100.0),
            42.0,
        )
        self.assertEqual(
            physical_deg_to_xycar_command(-100.0),
            -42.0,
        )

    def test_configurable_sign_can_match_vehicle_direction(self):
        calibration = SteeringCalibration(
            physical_steering_sign=-1.0,
        )

        self.assertEqual(
            physical_deg_to_xycar_command(10.0, calibration),
            -20.0,
        )

    def test_rejects_nonfinite_angle_or_invalid_table(self):
        with self.assertRaises(ValueError):
            physical_deg_to_xycar_command(math.nan)
        with self.assertRaises(ValueError):
            SteeringCalibration(
                physical_angle_deg=(0.0, 10.0, 5.0),
                xycar_command=(0.0, 20.0, 30.0),
            )
        with self.assertRaises(ValueError):
            SteeringCalibration(physical_steering_sign=0.0)


class DriveInputConversionTest(unittest.TestCase):
    def test_il_xycar_command_passes_through_unchanged(self):
        converted = convert_drive_input(
            valid_selection(
                steering_value=37.5,
                steering_unit=SteeringUnit.XYCAR_COMMAND,
                requested_speed=4.0,
                selected_source="drive_il",
            )
        )

        self.assertEqual(converted.steering_command, 37.5)
        self.assertEqual(converted.requested_speed, 4.0)

    def test_physical_angle_is_converted_and_metadata_is_preserved(self):
        converted = convert_drive_input(
            valid_selection(steering_held=True)
        )

        self.assertEqual(
            converted,
            FinalDriveCommandCandidate(
                steering_command=25.0,
                requested_speed=17.0,
                selected_source="cone_rule",
                steering_held=True,
                valid=True,
            ),
        )

    def test_invalid_input_remains_explicitly_invalid(self):
        converted = convert_drive_input(
            DriveInputSelection.invalid("lane_fallback")
        )

        self.assertEqual(
            converted,
            FinalDriveCommandCandidate.invalid("lane_fallback"),
        )

    def test_no_rate_limit_between_consecutive_conversions(self):
        left = convert_drive_input(
            valid_selection(steering_value=-26.0)
        )
        right = convert_drive_input(
            valid_selection(steering_value=26.0)
        )

        self.assertEqual(left.steering_command, -42.0)
        self.assertEqual(right.steering_command, 42.0)


if __name__ == "__main__":
    unittest.main()
