import math
import unittest

from track_drive.integration.lidar_cone_adapter import (
    ConeCommand,
    evaluate_lidar_cone_status,
    parse_cone_command,
    source_sample_is_fresh,
)


def evaluate(**overrides):
    parameters = {
        "now_sec": 10.0,
        "command": ConeCommand(2.0, 12.0, 0.6),
        "command_receive_sec": 9.9,
        "cluster_count": 4,
        "cluster_receive_sec": 9.9,
        "timeout_sec": 0.2,
        "path_ready_confidence": 0.35,
        "presence_confidence": 0.2,
        "presence_min_clusters": 2,
    }
    parameters.update(overrides)
    return evaluate_lidar_cone_status(**parameters)


class ConeCommandContractTest(unittest.TestCase):
    def test_parses_vehicle_tested_three_value_contract(self):
        command = parse_cone_command([2.5, 17.0, 0.75])

        self.assertEqual(command, ConeCommand(2.5, 17.0, 0.75))

    def test_rejects_short_non_finite_or_out_of_range_payload(self):
        self.assertIsNone(parse_cone_command([1.0, 2.0]))
        self.assertIsNone(parse_cone_command([math.nan, 2.0, 0.5]))
        self.assertIsNone(parse_cone_command([1.0, -1.0, 0.5]))
        self.assertIsNone(parse_cone_command([1.0, 2.0, 1.1]))


class LidarConeAdapterTest(unittest.TestCase):
    def test_ready_path_is_valid_and_present(self):
        status = evaluate()

        self.assertTrue(status.source_valid)
        self.assertTrue(status.path_ready)
        self.assertTrue(status.present)

    def test_weak_recovery_command_can_preserve_presence_without_entry(self):
        status = evaluate(
            command=ConeCommand(2.0, 9.5, 0.21),
            cluster_count=0,
        )

        self.assertTrue(status.source_valid)
        self.assertFalse(status.path_ready)
        self.assertTrue(status.present)

    def test_two_clusters_preserve_presence_without_ready_path(self):
        status = evaluate(
            command=ConeCommand(0.0, 0.0, 0.0),
            cluster_count=2,
        )

        self.assertTrue(status.source_valid)
        self.assertFalse(status.path_ready)
        self.assertTrue(status.present)

    def test_valid_absence_requires_fresh_zero_evidence_from_both_topics(self):
        status = evaluate(
            command=ConeCommand(0.0, 0.0, 0.0),
            cluster_count=0,
        )

        self.assertTrue(status.source_valid)
        self.assertFalse(status.path_ready)
        self.assertFalse(status.present)

    def test_stale_command_or_clusters_make_source_invalid(self):
        stale_command = evaluate(command_receive_sec=9.79)
        stale_clusters = evaluate(cluster_receive_sec=9.79)

        for status in (stale_command, stale_clusters):
            self.assertFalse(status.source_valid)
            self.assertFalse(status.path_ready)
            self.assertFalse(status.present)

    def test_malformed_command_or_cluster_count_make_source_invalid(self):
        malformed_command = evaluate(command=None)
        malformed_clusters = evaluate(cluster_count=None)

        self.assertFalse(malformed_command.source_valid)
        self.assertFalse(malformed_clusters.source_valid)

    def test_accepts_sample_at_timeout_boundary(self):
        self.assertTrue(
            source_sample_is_fresh(
                now_sec=10.2,
                last_receive_sec=10.0,
                timeout_sec=0.2,
            )
        )

    def test_rejects_missing_stale_or_future_sample(self):
        self.assertFalse(
            source_sample_is_fresh(
                now_sec=10.0,
                last_receive_sec=None,
                timeout_sec=0.5,
            )
        )
        self.assertFalse(
            source_sample_is_fresh(
                now_sec=10.3,
                last_receive_sec=10.0,
                timeout_sec=0.2,
            )
        )
        self.assertFalse(
            source_sample_is_fresh(
                now_sec=10.0,
                last_receive_sec=10.1,
                timeout_sec=0.2,
            )
        )


if __name__ == "__main__":
    unittest.main()
