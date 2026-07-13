from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


class SimulationContractTests(unittest.TestCase):
    def test_sim_recorder_uses_gazebo_clock_and_real_vehicle_topics(self):
        source = (ROOT / "launch" / "record_sim_drive_dataset.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('SetParameter(name="use_sim_time", value=True)', source)
        self.assertIn('"camera_front_topic": "/image_raw"', source)
        self.assertIn('"scan_topic": "/scan"', source)
        self.assertIn('"motor_topic": "/xycar_motor"', source)
        self.assertIn('"motor_msg_type": "float32_multi_array"', source)
        self.assertIn('default_value="50000"', source)
        self.assertIn('SetParameter(name="max_samples", value=max_samples)', source)
        recorder_source = (ROOT / "il_data_tools" / "common_recorder_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('name="il-recorder-shutdown"', recorder_source)

    def test_full_collection_launch_stops_everything_with_the_recorder(self):
        source = (ROOT / "launch" / "collect_sim_drive_dataset.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('default_value="50000"', source)
        self.assertIn("OnProcessExit(", source)
        self.assertIn("target_action=recorder", source)
        self.assertIn("event=Shutdown(", source)

    def test_recorder_does_not_shutdown_an_already_closed_context(self):
        source = (ROOT / "il_data_tools" / "common_recorder_node.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("if rclpy.ok():\n            rclpy.shutdown()", source)

    def test_runtime_preprocessing_matches_training_contract(self):
        scripts = ROOT / "scripts"
        sys.path.insert(0, str(scripts))
        try:
            from image_preprocessing import preprocess_bgr_image as train_image
            from policy_dataset import load_lidar_tensor
            from il_data_tools.runtime_preprocessing import (
                preprocess_bgr_image as runtime_image,
                preprocess_lidar_ranges,
            )

            image = np.random.default_rng(42).integers(
                0,
                256,
                size=(1024, 1280, 3),
                dtype=np.uint8,
            )
            np.testing.assert_allclose(
                runtime_image(image, 160, 90),
                train_image(image, 160, 90),
                rtol=0.0,
                atol=0.0,
            )

            ranges = np.linspace(0.1, 12.0, 505, dtype=np.float32)
            ranges[20] = np.inf
            ranges[40] = np.nan
            with tempfile.NamedTemporaryFile(suffix=".npz") as handle:
                np.savez(
                    handle.name,
                    ranges=ranges,
                    range_min=np.float32(0.1),
                    range_max=np.float32(12.0),
                )
                training_lidar = load_lidar_tensor(handle.name, 360).numpy()
            np.testing.assert_allclose(
                preprocess_lidar_ranges(ranges, 0.1, 12.0, 360),
                training_lidar,
                rtol=0.0,
                atol=0.0,
            )
        finally:
            sys.path.remove(str(scripts))


if __name__ == "__main__":
    unittest.main()
