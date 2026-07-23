# 설치된 ROS2 패키지 share 경로에서 모델 파일 위치를 찾는 유틸리티이다.
# V0.2 runtime은 final.onnx를 사용하며 cone model 함수는 레거시 source 호환용이다.
from pathlib import Path


PACKAGE_NAME = 'track_drive'


# 빌드 후 설치된 share/track_drive 경로를 기준으로 모델 파일을 찾기 위한 유틸리티 함수들이다.
# colcon build 후 install/share 영역에 설치된 track_drive 패키지 경로를 찾는다.
def package_share_path() -> Path:
    # 현재 패키지의 share 경로를 찾아 반환한다.
    # ROS2 환경에서는 ament index를 사용하고, 직접 실행 환경에서는 현재 파일 위치를 fallback으로 사용한다.
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(PACKAGE_NAME))
    except Exception:
        return Path(__file__).resolve().parents[1]


# launch나 노드에서 모델 파일명을 넘기면 설치 위치의 assets/models 경로로 변환한다.
# 모델 파일명을 받아 패키지 내부 assets/models의 절대 경로로 변환한다.
def package_model_path(filename: str) -> str:
    # 패키지 내부 모델 파일의 절대 경로를 만든다.
    return str(package_share_path() / 'assets' / 'models' / filename)


# source tree에 보관된 레거시 CNN 조향 모델의 기본 경로를 반환한다.
# 이 모델은 V0.2 설치 대상이 아니다.
def default_cone_model_path() -> str:
    return package_model_path('cone_bc_scripted_5.pt')


# 기본 객체 인식 ONNX 모델 경로를 반환한다.
def default_yolo_model_path() -> str:
    return package_model_path('final.onnx')
