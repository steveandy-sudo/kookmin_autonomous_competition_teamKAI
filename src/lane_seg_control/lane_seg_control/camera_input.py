"""Low-latency compressed camera decoding and calibration utilities."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml


def decode_compressed_bgr(
    data: bytes | bytearray | memoryview,
) -> np.ndarray | None:
    encoded = np.frombuffer(data, dtype=np.uint8)
    if encoded.size == 0:
        return None
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def scale_camera_matrix(
    matrix: np.ndarray,
    calibration_size: tuple[int, int] | None,
    image_size: tuple[int, int],
) -> np.ndarray:
    scaled = np.asarray(matrix, dtype=np.float64).reshape(3, 3).copy()
    if calibration_size is None or calibration_size == image_size:
        return scaled
    calibration_width, calibration_height = calibration_size
    image_width, image_height = image_size
    if calibration_width <= 0 or calibration_height <= 0:
        return scaled
    scale_x = image_width / float(calibration_width)
    scale_y = image_height / float(calibration_height)
    scaled[0, :3] *= scale_x
    scaled[1, :3] *= scale_y
    return scaled


class CameraRectifier:
    """Cache OpenCV rectification maps for one calibration and image size."""

    def __init__(self, calibration_path: str | Path, balance: float) -> None:
        path = Path(calibration_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"camera calibration not found: {path}")
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)

        if "camera_matrix" in data:
            matrix_data = data["camera_matrix"]["data"]
        elif "K" in data:
            matrix_data = data["K"]
        else:
            raise ValueError(f"camera_matrix or K is missing from {path}")
        if "distortion_coefficients" in data:
            distortion_data = data["distortion_coefficients"]["data"]
        else:
            distortion_data = data.get("D", [0.0, 0.0, 0.0, 0.0])

        self.path = path
        self.matrix = np.asarray(
            matrix_data, dtype=np.float64
        ).reshape(3, 3)
        self.distortion = np.asarray(
            distortion_data, dtype=np.float64
        ).reshape(-1)
        self.distortion_model = str(
            data.get("distortion_model", "fisheye")
        ).lower()
        calibration_width = int(data.get("image_width") or 0)
        calibration_height = int(data.get("image_height") or 0)
        self.calibration_size = (
            (calibration_width, calibration_height)
            if calibration_width > 0 and calibration_height > 0
            else None
        )
        self.balance = float(balance)
        self.rectified_matrix: np.ndarray | None = None
        self.map1: np.ndarray | None = None
        self.map2: np.ndarray | None = None
        self.map_size: tuple[int, int] | None = None
        self.scaled_maps: dict[
            tuple[int, int, int, int],
            tuple[np.ndarray, np.ndarray],
        ] = {}

    def rectification_parameters(
        self,
        width: int,
        height: int,
    ) -> tuple[
        tuple[int, int],
        np.ndarray,
        np.ndarray,
        np.ndarray,
    ]:
        size = (int(width), int(height))
        camera_matrix = scale_camera_matrix(
            self.matrix,
            self.calibration_size,
            size,
        )
        rotation = np.eye(3, dtype=np.float64)
        if (
            "fisheye" in self.distortion_model
            or "equidistant" in self.distortion_model
        ):
            distortion = np.zeros((4, 1), dtype=np.float64)
            count = min(4, self.distortion.size)
            distortion[:count, 0] = self.distortion[:count]
            rectified_matrix = (
                cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    camera_matrix,
                    distortion,
                    size,
                    rotation,
                    balance=self.balance,
                    new_size=size,
                )
            )
        else:
            distortion = self.distortion
            rectified_matrix, _ = cv2.getOptimalNewCameraMatrix(
                camera_matrix,
                distortion,
                size,
                alpha=self.balance,
                newImgSize=size,
            )
        return size, camera_matrix, distortion, rectified_matrix

    def build_maps(self, width: int, height: int) -> None:
        size, camera_matrix, distortion, rectified_matrix = (
            self.rectification_parameters(width, height)
        )
        rotation = np.eye(3, dtype=np.float64)
        if (
            "fisheye" in self.distortion_model
            or "equidistant" in self.distortion_model
        ):
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                camera_matrix,
                distortion,
                rotation,
                rectified_matrix,
                size,
                cv2.CV_32FC1,
            )
        else:
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                camera_matrix,
                distortion,
                rotation,
                rectified_matrix,
                size,
                cv2.CV_32FC1,
            )
        self.rectified_matrix = rectified_matrix
        self.map_size = size

    def build_scaled_maps(
        self,
        source_width: int,
        source_height: int,
        output_width: int,
        output_height: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        source_size, camera_matrix, distortion, rectified_matrix = (
            self.rectification_parameters(source_width, source_height)
        )
        output_size = (int(output_width), int(output_height))
        if output_size[0] <= 0 or output_size[1] <= 0:
            raise ValueError(f"invalid rectified output size: {output_size}")

        scaled_rectified_matrix = rectified_matrix.copy()
        scaled_rectified_matrix[0, :3] *= (
            output_size[0] / float(source_size[0])
        )
        scaled_rectified_matrix[1, :3] *= (
            output_size[1] / float(source_size[1])
        )
        rotation = np.eye(3, dtype=np.float64)
        if (
            "fisheye" in self.distortion_model
            or "equidistant" in self.distortion_model
        ):
            return cv2.fisheye.initUndistortRectifyMap(
                camera_matrix,
                distortion,
                rotation,
                scaled_rectified_matrix,
                output_size,
                cv2.CV_32FC1,
            )
        return cv2.initUndistortRectifyMap(
            camera_matrix,
            distortion,
            rotation,
            scaled_rectified_matrix,
            output_size,
            cv2.CV_32FC1,
        )

    def rectify(self, image: np.ndarray) -> np.ndarray:
        height, width = image.shape[:2]
        if self.map1 is None or self.map2 is None or self.map_size != (
            width,
            height,
        ):
            self.build_maps(width, height)
        return cv2.remap(
            image,
            self.map1,
            self.map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def rectify_to_size(
        self,
        image: np.ndarray,
        output_width: int,
        output_height: int,
    ) -> np.ndarray:
        """Rectify directly into a smaller model frame with one remap."""
        source_height, source_width = image.shape[:2]
        key = (
            int(source_width),
            int(source_height),
            int(output_width),
            int(output_height),
        )
        maps = self.scaled_maps.get(key)
        if maps is None:
            maps = self.build_scaled_maps(*key)
            self.scaled_maps[key] = maps
        return cv2.remap(
            image,
            maps[0],
            maps[1],
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
