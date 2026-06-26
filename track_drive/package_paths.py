from pathlib import Path


PACKAGE_NAME = 'track_drive'


# 설명: track_drive 패키지의 공유 디렉터리 경로를 찾는다.
def package_share_path() -> Path:
    # 현재 패키지의 share 경로를 찾아 반환한다.
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(PACKAGE_NAME))
    except Exception:
        return Path(__file__).resolve().parents[1]


# 설명: 패키지 공유 디렉터리 안의 모델 파일 경로를 만든다.
def package_model_path(filename: str) -> str:
    # 패키지 내부 모델 파일의 절대 경로를 만든다.
    return str(package_share_path() / 'assets' / 'models' / filename)


# 설명: 기본 콘 주행 AI 모델 파일 경로를 반환한다.
def default_cone_model_path() -> str:
    # 기본 콘 AI TorchScript 모델 경로를 반환한다.
    return package_model_path('cone_bc_scripted_5.pt')


# 설명: 기본 YOLO 모델 파일 경로를 반환한다.
def default_yolo_model_path() -> str:
    # 기본 YOLO ONNX 모델 경로를 반환한다.
    return package_model_path('final.onnx')
