from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

from il_data_tools.common_recorder_node import BadDataPreroll, rate_limit_allows
from il_data_tools.recorder_state import (
    DiskSpaceGuard,
    LabelLatch,
    wait_for_thread_shutdown,
)


class ReliabilityTests(unittest.TestCase):
    class Sample:
        def __init__(self, timestamp_ns):
            self.timestamp_ns = timestamp_ns

    def test_stop_discards_exact_three_second_preroll(self):
        buffer = BadDataPreroll(window_ns=3_000_000_000)
        released = []
        for second in range(1, 7):
            released += buffer.push(self.Sample(second * 1_000_000_000))
        safe, discarded = buffer.stop(6_500_000_000)
        released += safe
        self.assertEqual([sample.timestamp_ns for sample in released], [1_000_000_000, 2_000_000_000, 3_000_000_000])
        self.assertEqual(discarded, 3)
        self.assertEqual(len(buffer), 0)

    def test_stop_keeps_only_samples_older_than_the_bad_window(self):
        buffer = BadDataPreroll(window_ns=3_000_000_000)
        released = []
        for tenth in range(50, 101):
            released += buffer.push(self.Sample(tenth * 100_000_000))

        safe, discarded = buffer.stop(10_000_000_000)
        released += safe

        self.assertTrue(released)
        self.assertLess(released[-1].timestamp_ns, 7_000_000_000)
        self.assertEqual(discarded, 31)
        self.assertTrue(all(sample.timestamp_ns < 7_000_000_000 for sample in released))

    def test_rate_limit_accepts_small_nominal_timestamp_jitter(self):
        self.assertTrue(rate_limit_allows(91_000_000, 0, 10.0))
        self.assertFalse(rate_limit_allows(89_000_000, 0, 10.0))
        self.assertTrue(rate_limit_allows(1, None, 10.0))
        self.assertTrue(rate_limit_allows(1, 0, 0.0))

    def test_label_is_latched_until_changed(self):
        latch = LabelLatch("general_drive")
        self.assertEqual(latch.active, "general_drive")
        latch.update("recovery", 100)
        self.assertEqual(latch.active, "recovery")
        self.assertTrue(latch.ever_received)
        self.assertEqual(latch.change_count, 1)
        latch.update("recovery", 200)
        self.assertEqual(latch.change_count, 1)
        self.assertEqual(latch.last_timestamp_ns, 200)

    def test_disk_guard_uses_cached_periodic_check(self):
        guard = DiskSpaceGuard(min_free_gb=10.0, period_sec=5.0)
        usage = mock.Mock(free=20 * 1024 ** 3)
        with tempfile.TemporaryDirectory() as directory, mock.patch(
            "il_data_tools.recorder_state.shutil.disk_usage", return_value=usage
        ) as disk_usage:
            self.assertTrue(guard.available(Path(directory), now=10.0))
            self.assertTrue(guard.available(Path(directory), now=12.0))
            self.assertEqual(disk_usage.call_count, 1)

    def test_disk_guard_stops_below_threshold(self):
        guard = DiskSpaceGuard(min_free_gb=10.0, period_sec=5.0)
        usage = mock.Mock(free=9 * 1024 ** 3)
        with tempfile.TemporaryDirectory() as directory, mock.patch(
            "il_data_tools.recorder_state.shutil.disk_usage", return_value=usage
        ):
            self.assertFalse(guard.available(Path(directory), now=10.0))

    def test_writer_shutdown_waits_past_warning_threshold(self):
        warnings = []
        worker = threading.Thread(target=lambda: time.sleep(0.05), daemon=True)
        worker.start()
        elapsed = wait_for_thread_shutdown(
            worker,
            warning_after_sec=0.01,
            on_warning=warnings.append,
            poll_sec=0.01,
        )
        self.assertFalse(worker.is_alive())
        self.assertGreaterEqual(elapsed, 0.04)
        self.assertEqual(len(warnings), 1)


if __name__ == "__main__":
    unittest.main()
