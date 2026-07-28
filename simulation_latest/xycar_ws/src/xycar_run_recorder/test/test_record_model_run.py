import tempfile
import unittest
from pathlib import Path

import yaml

from xycar_run_recorder.record_model_run import choose_topics
from xycar_run_recorder.record_model_run import missing_required_groups
from xycar_run_recorder.record_model_run import write_rate_summary
from xycar_run_recorder.canonical_compressor import PERCEPTION_STREAMS
from xycar_run_recorder.canonical_compressor import perception_streams


class RecordModelRunTest(unittest.TestCase):
    def test_intermediate_perception_streams_are_opt_in(self):
        self.assertEqual(perception_streams(False), ())
        self.assertEqual(perception_streams(True), PERCEPTION_STREAMS)

    def test_topic_selection_excludes_uncompressed_images(self):
        topics = {
            "/wide_camera_mjpeg/image_raw/compressed": (
                "sensor_msgs/msg/CompressedImage"
            ),
            "/perception/canonical_road_image": "sensor_msgs/msg/Image",
            "/recording/canonical_road_image/compressed": (
                "sensor_msgs/msg/CompressedImage"
            ),
            "/scan": "sensor_msgs/msg/LaserScan",
            "/xycar_ultrasonic": "std_msgs/msg/Int32MultiArray",
            "/imu": "sensor_msgs/msg/Imu",
            "/xycar_motor": "std_msgs/msg/Float32MultiArray",
        }

        selected = dict(choose_topics(topics))

        self.assertIn("/wide_camera_mjpeg/image_raw/compressed", selected)
        self.assertIn("/recording/canonical_road_image/compressed", selected)
        self.assertNotIn("/perception/canonical_road_image", selected)
        self.assertNotIn("/scan", selected)
        self.assertNotIn("/xycar_ultrasonic", selected)

    def test_required_groups_report_missing_vesc(self):
        topics = {
            "/wide_camera_mjpeg/image_raw/compressed": (
                "sensor_msgs/msg/CompressedImage"
            ),
            "/imu": "sensor_msgs/msg/Imu",
            "/xycar_motor": "std_msgs/msg/Float32MultiArray",
        }

        missing = missing_required_groups(
            topics,
            require_vesc=True,
        )

        self.assertEqual(missing, ["VESC_TELEMETRY"])

    def test_motor_command_is_deferred_and_not_required(self):
        topics = {
            "/wide_camera_mjpeg/image_raw/compressed": (
                "sensor_msgs/msg/CompressedImage"
            ),
            "/imu": "sensor_msgs/msg/Imu",
            "/vehicle/vesc_state": "xycar_msgs/msg/XycarVescState",
        }

        selected = dict(choose_topics(topics))
        missing = missing_required_groups(
            topics,
            require_vesc=True,
        )

        self.assertEqual(
            selected["/xycar_motor"],
            "<runtime-discovery>",
        )
        self.assertEqual(missing, [])

    def test_hybrid_topics_are_available_for_runtime_discovery(self):
        selected = dict(choose_topics({}))

        self.assertEqual(
            selected["/hybrid/mode"],
            "<runtime-discovery>",
        )
        self.assertEqual(
            selected["/hybrid/debug"],
            "<runtime-discovery>",
        )

    def test_rate_summary_uses_bag_duration(self):
        metadata = {
            "rosbag2_bagfile_information": {
                "duration": {"nanoseconds": 2_000_000_000},
                "topics_with_message_count": [
                    {
                        "topic_metadata": {
                            "name": "/imu",
                            "type": "sensor_msgs/msg/Imu",
                        },
                        "message_count": 200,
                    }
                ],
            }
        }
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            bag = session / "bag"
            bag.mkdir()
            (bag / "metadata.yaml").write_text(
                yaml.safe_dump(metadata),
                encoding="utf-8",
            )

            rows = write_rate_summary(bag, session)

            self.assertAlmostEqual(rows[0]["mean_hz"], 100.0)
            self.assertEqual(rows[0]["status"], "OK")

    def test_rate_summary_marks_empty_topics(self):
        metadata = {
            "rosbag2_bagfile_information": {
                "duration": {"nanoseconds": 2_000_000_000},
                "topics_with_message_count": [
                    {
                        "topic_metadata": {
                            "name": "/unused_sensor",
                            "type": "sensor_msgs/msg/LaserScan",
                        },
                        "message_count": 0,
                    }
                ],
            }
        }
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            bag = session / "bag"
            bag.mkdir()
            (bag / "metadata.yaml").write_text(
                yaml.safe_dump(metadata),
                encoding="utf-8",
            )

            rows = write_rate_summary(bag, session)

            self.assertEqual(rows[0]["status"], "EMPTY")


if __name__ == "__main__":
    unittest.main()
