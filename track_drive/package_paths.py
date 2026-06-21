from pathlib import Path


PACKAGE_NAME = 'track_drive'


def package_share_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(PACKAGE_NAME))
    except Exception:
        return Path(__file__).resolve().parents[1]


def package_model_path(filename: str) -> str:
    return str(package_share_path() / 'assets' / 'models' / filename)


def default_cone_model_path() -> str:
    return package_model_path('cone_bc_scripted_4.pt')


def default_yolo_model_path() -> str:
    return package_model_path('final.onnx')
