"""Lightweight projection and box association for planar LiDAR clusters."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import yaml


Point2 = tuple[float, float]
ImageBox = tuple[float, float, float, float]


def load_lidar_camera_extrinsic(
    path_text: str | Path,
) -> tuple[np.ndarray, np.ndarray]:
    """Load the laser-to-rectified-camera transform from a calibration YAML."""
    path = Path(path_text).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"LiDAR-camera extrinsic not found: {path}")
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    transform = data.get("T_camera_lidar", {})
    try:
        rotation = np.asarray(
            transform["R_row_major"], dtype=np.float64
        ).reshape(3, 3)
        translation = np.asarray(
            transform["t_xyz"], dtype=np.float64
        ).reshape(3)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f"invalid T_camera_lidar in extrinsic YAML: {path}"
        ) from exc
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError(f"non-finite LiDAR-camera transform in {path}")
    return rotation, translation


def project_laser_xy(
    points_xy: np.ndarray,
    rotation_camera_laser: np.ndarray,
    translation_camera_laser: np.ndarray,
    camera_matrix: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Project planar laser-frame points into a rectified camera image."""
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    if points.size == 0:
        return np.empty((0, 2), dtype=np.float64), np.empty(0, dtype=bool)
    points_3d = np.column_stack((points, np.zeros(points.shape[0])))
    camera = (
        np.asarray(rotation_camera_laser, dtype=np.float64).reshape(3, 3)
        @ points_3d.T
    ).T + np.asarray(translation_camera_laser, dtype=np.float64).reshape(1, 3)
    depth = camera[:, 2]
    valid = np.isfinite(camera).all(axis=1) & (depth > 1e-4)
    pixels = np.full((points.shape[0], 2), np.nan, dtype=np.float64)
    if np.any(valid):
        normalized = camera[valid, :2] / depth[valid, np.newaxis]
        matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        pixels[valid, 0] = normalized[:, 0] * matrix[0, 0] + matrix[0, 2]
        pixels[valid, 1] = normalized[:, 1] * matrix[1, 1] + matrix[1, 2]
    return pixels, valid


def associate_lidar_clusters_with_boxes(
    clusters: Iterable[Point2],
    boxes: Sequence[ImageBox],
    *,
    rotation_camera_laser: np.ndarray,
    translation_camera_laser: np.ndarray,
    camera_matrix: np.ndarray,
    image_width: int,
    image_height: int,
    padding_ratio: float = 0.20,
    minimum_padding_px: float = 8.0,
    match_vertical: bool = True,
) -> list[Point2]:
    """Keep the nearest projected LiDAR cluster for each padded YOLO box.

    A planar LiDAR has no object-height measurement.  Callers may therefore
    match only the horizontal camera bearing while retaining full 3-D
    extrinsic projection for left/right alignment.  Selecting at most one
    cluster per box prevents a large nearby cone box from also admitting
    farther cones that lie behind it in the camera view.
    """
    points = [(float(x), float(y)) for x, y in clusters]
    if not points or not boxes or image_width <= 0 or image_height <= 0:
        return []
    pixels, valid = project_laser_xy(
        np.asarray(points, dtype=np.float64),
        rotation_camera_laser,
        translation_camera_laser,
        camera_matrix,
    )
    ratio = max(0.0, float(padding_ratio))
    minimum = max(0.0, float(minimum_padding_px))
    expanded_boxes: list[ImageBox] = []
    for xmin, ymin, xmax, ymax in boxes:
        width = max(0.0, float(xmax) - float(xmin))
        height = max(0.0, float(ymax) - float(ymin))
        pad_x = max(minimum, width * ratio)
        pad_y = max(minimum, height * ratio)
        expanded_boxes.append(
            (
                max(0.0, float(xmin) - pad_x),
                max(0.0, float(ymin) - pad_y),
                min(float(image_width), float(xmax) + pad_x),
                min(float(image_height), float(ymax) + pad_y),
            )
        )

    candidate_indices: list[int] = []
    for index in range(len(points)):
        if not bool(valid[index]):
            continue
        u, v = pixels[index]
        if not (0.0 <= u < image_width and 0.0 <= v < image_height):
            continue
        candidate_indices.append(index)

    selected_indices: set[int] = set()
    for xmin, ymin, xmax, ymax in expanded_boxes:
        matches = [
            index
            for index in candidate_indices
            if (
                xmin <= pixels[index, 0] <= xmax
                and (
                    not match_vertical
                    or ymin <= pixels[index, 1] <= ymax
                )
            )
        ]
        if not matches:
            continue
        nearest = min(
            matches,
            key=lambda index: (
                float(np.hypot(points[index][0], points[index][1])),
                index,
            ),
        )
        selected_indices.add(nearest)

    return [
        point
        for index, point in enumerate(points)
        if index in selected_indices
    ]
