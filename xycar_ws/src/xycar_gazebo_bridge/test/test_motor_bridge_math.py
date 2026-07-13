import unittest

from xycar_gazebo_bridge.xycar_motor_bridge import interpolate_clamped


COMMANDS = [-42.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 42.0]
CURVATURES = [
    1.366747,
    0.922781,
    0.552809,
    0.194230,
    0.0,
    -0.556883,
    -0.959829,
    -1.369323,
    -1.860716,
]


class MotorBridgeMathTest(unittest.TestCase):
    def test_profile_points_are_preserved(self):
        for command, curvature in zip(COMMANDS, CURVATURES):
            self.assertEqual(
                interpolate_clamped(command, COMMANDS, CURVATURES),
                curvature,
            )

    def test_intermediate_command_is_linearly_interpolated(self):
        expected = (CURVATURES[5] + CURVATURES[6]) / 2.0
        actual = interpolate_clamped(15.0, COMMANDS, CURVATURES)
        self.assertAlmostEqual(actual, expected)

    def test_unmeasured_commands_are_clamped_to_measured_range(self):
        self.assertEqual(
            interpolate_clamped(-100.0, COMMANDS, CURVATURES),
            CURVATURES[0],
        )
        self.assertEqual(
            interpolate_clamped(100.0, COMMANDS, CURVATURES),
            CURVATURES[-1],
        )

    def test_42_endpoints_extend_the_measured_20_to_30_slopes(self):
        negative_42 = CURVATURES[1] + 1.2 * (CURVATURES[1] - CURVATURES[2])
        positive_42 = CURVATURES[7] + 1.2 * (CURVATURES[7] - CURVATURES[6])
        self.assertAlmostEqual(CURVATURES[0], negative_42, places=6)
        self.assertAlmostEqual(CURVATURES[-1], positive_42, places=6)

    def test_invalid_lookup_table_is_rejected(self):
        invalid_tables = [
            ([0.0], [0.0]),
            ([0.0, 1.0], [0.0]),
            ([0.0, 0.0], [0.0, 1.0]),
        ]
        for inputs, outputs in invalid_tables:
            with self.assertRaises(ValueError):
                interpolate_clamped(0.0, inputs, outputs)


if __name__ == "__main__":
    unittest.main()
