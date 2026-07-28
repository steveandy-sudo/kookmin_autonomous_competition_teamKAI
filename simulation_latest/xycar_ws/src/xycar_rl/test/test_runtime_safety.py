import math
import unittest

import numpy as np
from sensor_msgs.msg import LaserScan

from xycar_rl.policy_runtime_node import front_obstacle_distance
from xycar_rl.policy_runtime_node import shift_canonical_image_for_target


class RuntimeSafetyTest(unittest.TestCase):
    def test_front_distance_ignores_rear_obstacle(self):
        scan = LaserScan()
        scan.angle_min = -math.pi
        scan.angle_increment = math.pi / 2.0
        scan.range_min = 0.1
        scan.range_max = 12.0
        scan.ranges = [0.2, 3.0, 1.2, 4.0, 0.3]
        distance = front_obstacle_distance(scan, math.radians(25.0))
        self.assertAlmostEqual(distance, 1.2)

    def test_positive_target_offset_shifts_canonical_features_left(self):
        image = np.full((3, 14, 3), 36, dtype=np.uint8)
        image[:, 7, :] = 255

        shifted = shift_canonical_image_for_target(
            image,
            target_lateral_offset_m=0.10,
            lateral_range_m=1.4,
            background_gray=36,
        )

        self.assertTrue(np.all(shifted[:, 6, :] == 255))
        self.assertTrue(np.all(shifted[:, -1, :] == 36))

    def test_zero_target_offset_preserves_canonical_image(self):
        image = np.arange(42, dtype=np.uint8).reshape(3, 14)

        shifted = shift_canonical_image_for_target(
            image,
            target_lateral_offset_m=0.0,
            lateral_range_m=1.4,
            background_gray=36,
        )

        np.testing.assert_array_equal(shifted, image)
        self.assertIsNot(shifted, image)


if __name__ == "__main__":
    unittest.main()
