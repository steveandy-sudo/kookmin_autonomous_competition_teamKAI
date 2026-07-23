import math
import unittest

from track_drive.mission.start_signal_adapter import (
    normalize_start_signal_state,
    start_signal_source_is_fresh,
)
from track_drive.mission.states import StartSignal


class StartSignalNormalizationTest(unittest.TestCase):
    def test_known_detector_states_are_normalized(self):
        cases = (
            ("red", StartSignal.RED),
            (" YELLOW ", StartSignal.YELLOW),
            ("green", StartSignal.GREEN),
            ("BLUE", StartSignal.GREEN),
            ("none", StartSignal.UNKNOWN),
            ("left", StartSignal.UNKNOWN),
            ("unknown", StartSignal.UNKNOWN),
        )

        for raw_state, expected in cases:
            with self.subTest(raw_state=raw_state):
                signal, valid = normalize_start_signal_state(raw_state)
                self.assertIs(signal, expected)
                self.assertTrue(valid)

    def test_go_and_unknown_strings_are_invalid(self):
        for raw_state in ("GO", "", "vehicle", None):
            with self.subTest(raw_state=raw_state):
                signal, valid = normalize_start_signal_state(raw_state)
                self.assertIs(signal, StartSignal.UNKNOWN)
                self.assertFalse(valid)


class StartSignalFreshnessTest(unittest.TestCase):
    def test_freshness_includes_timeout_boundary(self):
        self.assertTrue(
            start_signal_source_is_fresh(
                now_sec=10.5,
                last_receive_sec=10.0,
                timeout_sec=0.5,
                source_state_valid=True,
            )
        )
        self.assertFalse(
            start_signal_source_is_fresh(
                now_sec=10.500001,
                last_receive_sec=10.0,
                timeout_sec=0.5,
                source_state_valid=True,
            )
        )

    def test_missing_invalid_or_reversed_source_is_not_fresh(self):
        cases = (
            (10.0, None, 0.5, True),
            (10.0, 10.0, 0.5, False),
            (9.0, 10.0, 0.5, True),
            (10.0, 10.0, -0.1, True),
            (math.inf, 10.0, 0.5, True),
        )

        for now_sec, last_receive_sec, timeout_sec, valid in cases:
            with self.subTest(
                now_sec=now_sec,
                last_receive_sec=last_receive_sec,
                timeout_sec=timeout_sec,
                valid=valid,
            ):
                self.assertFalse(
                    start_signal_source_is_fresh(
                        now_sec=now_sec,
                        last_receive_sec=last_receive_sec,
                        timeout_sec=timeout_sec,
                        source_state_valid=valid,
                    )
                )


if __name__ == "__main__":
    unittest.main()
