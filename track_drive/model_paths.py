"""Resolve the optional traffic-light model from the ROS2 package share."""

from pathlib import Path


PACKAGE_NAME = "track_drive"


def package_share_path() -> Path:
    """Return the installed share path, or the source root in unit tests."""

    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(PACKAGE_NAME))
    except Exception:
        return Path(__file__).resolve().parents[1]


def package_model_path(filename: str) -> str:
    """Return a model path under ``assets/models``."""

    return str(package_share_path() / "assets" / "models" / filename)


def default_yolo_model_path() -> str:
    """Return the optional traffic-light detector model path."""

    return package_model_path("final.onnx")
