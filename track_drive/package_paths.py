from pathlib import Path


PACKAGE_NAME = 'track_drive'


def package_share_path() -> Path:
    # 현재 패키지의 share 경로를 찾아 반환한다.
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(PACKAGE_NAME))
    except Exception:
        return Path(__file__).resolve().parents[1]


def package_model_path(filename: str) -> str:
    # 패키지 내부 모델 파일의 절대 경로를 만든다.
    return str(package_share_path() / 'assets' / 'models' / filename)


def default_cone_model_path() -> str:
    # 기본 콘 AI TorchScript 모델 경로를 반환한다.
    return package_model_path('cone_bc_scripted_5.pt')


def default_yolo_model_path() -> str:
    # 기본 YOLO ONNX 모델 경로를 반환한다.
    return package_model_path('final.onnx')
