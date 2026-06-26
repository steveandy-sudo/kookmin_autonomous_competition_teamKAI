# 설치된 ROS2 패키지 share 경로에서 모델 파일 위치를 찾는 유틸리티이다.
# 제출 폴더가 빌드된 뒤에도 CNN 조향 모델과 ONNX 객체 인식 모델을 안정적으로 참조하기 위해 사용한다.
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


# 기본 CNN 조향 모델 경로를 반환한다.
def default_cone_model_path() -> str:
    # 기본 CNN 조향 TorchScript 모델 경로를 반환한다.
    return package_model_path('cnn_steering_model.pt')


# 기본 객체 인식 ONNX 모델 경로를 반환한다.
def default_yolo_model_path() -> str:
    # 기본 미션 객체 인식 ONNX 모델 경로를 반환한다.
    return package_model_path('object_detector_model.onnx')
