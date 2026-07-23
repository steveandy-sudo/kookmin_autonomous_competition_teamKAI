import unittest

from track_drive.traffic_light_signal import (
    detection_signal_flags,
    resolve_signal_state,
)


class DetectionSignalFlagsTest(unittest.TestCase):
    def test_class_four_is_red_only(self):
        flags = detection_signal_flags(
            class_id=4,
            score=0.9,
            valid=True,
            red_color=False,
            red_ids={4},
            yellow_ids={5},
            green_ids={1},
            left_ids={2},
            red_threshold=0.55,
            yellow_threshold=0.35,
            left_threshold=0.28,
        )

        self.assertEqual(flags, (True, False, False, False))

    def test_class_five_is_yellow_even_with_red_color_pixels(self):
        flags = detection_signal_flags(
            class_id=5,
            score=0.9,
            valid=True,
            red_color=True,
            red_ids={4},
            yellow_ids={5},
            green_ids={1},
            left_ids={2},
            red_threshold=0.55,
            yellow_threshold=0.35,
            left_threshold=0.28,
        )

        self.assertEqual(flags, (False, False, False, True))

    def test_invalid_or_low_confidence_yellow_is_not_accepted(self):
        common = {
            "class_id": 5,
            "red_color": False,
            "red_ids": {4},
            "yellow_ids": {5},
            "green_ids": {1},
            "left_ids": {2},
            "red_threshold": 0.55,
            "yellow_threshold": 0.35,
            "left_threshold": 0.28,
        }

        self.assertEqual(
            detection_signal_flags(
                score=0.34,
                valid=True,
                **common,
            ),
            (False, False, False, False),
        )
        self.assertEqual(
            detection_signal_flags(
                score=0.9,
                valid=False,
                **common,
            ),
            (False, False, False, False),
        )


class ResolveSignalStateTest(unittest.TestCase):
    def test_clean_red_yellow_and_green_states_are_distinct(self):
        cases = (
            ((True, False, False, False), "red"),
            ((False, True, False, False), "green"),
            ((False, False, False, True), "yellow"),
        )

        for flags, expected in cases:
            with self.subTest(expected=expected):
                red, green, left, yellow = flags
                self.assertEqual(
                    resolve_signal_state(
                        red=red,
                        green=green,
                        left=left,
                        yellow=yellow,
                        best_label="unknown",
                    ),
                    expected,
                )

    def test_conflicting_primary_signals_are_unknown(self):
        conflicts = (
            (True, True, False),
            (True, False, True),
            (False, True, True),
            (True, True, True),
        )

        for red, green, yellow in conflicts:
            with self.subTest(
                red=red,
                green=green,
                yellow=yellow,
            ):
                self.assertEqual(
                    resolve_signal_state(
                        red=red,
                        green=green,
                        left=False,
                        yellow=yellow,
                        best_label="red",
                    ),
                    "unknown",
                )


if __name__ == "__main__":
    unittest.main()
