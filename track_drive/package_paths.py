"""Compatibility imports for the single model-path implementation."""

from .model_paths import (
    default_cone_model_path,
    default_yolo_model_path,
    package_model_path,
    package_share_path,
)


__all__ = [
    "default_cone_model_path",
    "default_yolo_model_path",
    "package_model_path",
    "package_share_path",
]
