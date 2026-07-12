import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))


class PolicyContractTests(unittest.TestCase):
    def test_pilotnet_preserves_spatial_features(self):
        try:
            import torch
            from policy_models import PilotNetEncoder
        except ImportError:
            self.skipTest("torch is not installed")
        encoder = PilotNetEncoder()
        output = encoder(torch.zeros(2, 3, 90, 160))
        self.assertEqual(tuple(output.shape), (2, 64 * 2 * 4))

    def test_augmentation_does_not_shift_image(self):
        source = (SCRIPTS / "policy_dataset.py").read_text(encoding="utf-8")
        self.assertNotIn("random_shift_crop", source)
        self.assertIn("steer_norm = -steer_norm", source)

    def test_phase_benchmark_uses_batch_column_shape(self):
        source = (SCRIPTS / "benchmark_policy_model.py").read_text(encoding="utf-8")
        self.assertIn("torch.zeros(1, 1", source)

    def test_reference_camera_crop_preserves_16_by_9(self):
        try:
            import numpy as np
            from image_preprocessing import crop_to_target_aspect
        except ImportError:
            self.skipTest("opencv/numpy are not installed")
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        roi = crop_to_target_aspect(image, 160, 90)
        self.assertEqual(tuple(roi.shape), (360, 640, 3))
        # Encode row index to verify the documented y=80:440 crop.
        image[:, :, 0] = np.arange(480, dtype=np.uint16)[:, None] % 256
        roi = crop_to_target_aspect(image, 160, 90)
        self.assertEqual(int(roi[0, 0, 0]), 80)
        self.assertEqual(int(roi[-1, 0, 0]), 439 % 256)


if __name__ == "__main__":
    unittest.main()
