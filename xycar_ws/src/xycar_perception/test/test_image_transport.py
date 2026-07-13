import unittest

import cv2
import numpy as np

from xycar_perception.camera_perception_node import decode_compressed_image


class ImageTransportTest(unittest.TestCase):
    def test_jpeg_is_decoded_as_bgr_image(self):
        source = np.zeros((24, 32, 3), dtype=np.uint8)
        source[:, :, 1] = 180
        encoded_ok, encoded = cv2.imencode(".jpg", source)
        self.assertTrue(encoded_ok)

        decoded = decode_compressed_image(encoded.tobytes())

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.shape, source.shape)
        self.assertGreater(float(decoded[:, :, 1].mean()), 150.0)

    def test_empty_payload_is_rejected(self):
        self.assertIsNone(decode_compressed_image(b""))

    def test_invalid_payload_is_rejected(self):
        self.assertIsNone(decode_compressed_image(b"not-an-image"))


if __name__ == "__main__":
    unittest.main()
