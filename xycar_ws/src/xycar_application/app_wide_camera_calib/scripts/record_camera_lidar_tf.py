#!/usr/bin/env python3

import argparse
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import yaml


HOME = Path.home()
WORKSPACE_SETUP = HOME / "xycar_ws/install/setup.bash"

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
INTRINSIC_YAML = PACKAGE_ROOT / "config/wide_camera_fisheye_1280x1024.yaml"
EXTRINSIC_YAML = PACKAGE_ROOT / "config/lidar_camera_extrinsic_final_safe.yaml"

CAMERA_TOPIC = "/wide_camera_mjpeg/image_raw/compressed"
CAMERA_INFO_TOPIC = "/wide_camera/rect/camera_info"
RECTIFIED_TOPIC = "/wide_camera/rect/image_raw"
SCAN_TOPIC = "/scan"

EXPECTED_WIDTH = 1280
EXPECTED_HEIGHT = 1024

BEV_AND_LANE_CONFIG = {
    "bev": {
        "width": 640,
        "height": 220,
        "src_top_y_ratio": 0.520,
        "src_bottom_y_ratio": 0.625,
        "src_top_left_x_ratio": 0.357,
        "src_top_right_x_ratio": 0.777,
        "src_bottom_left_x_ratio": 0.170,
        "src_bottom_right_x_ratio": 0.965,
        "dst_margin_ratio": 0.10,
        "lateral_m_per_px": 0.0015234375,
        "forward_m_per_px": 0.005,
    },
    "lane_color_threshold": {
        "yellow_h_min": 14,
        "yellow_h_max": 45,
        "yellow_sat_min": 60,
        "yellow_val_min": 80,
        "white_sat_max": 120,
        "white_val_min": 145,
    },
}


def ros_command(command: str):
    return [
        "bash",
        "-lc",
        (
            "source /opt/ros/humble/setup.bash && "
            f"source {shlex.quote(str(WORKSPACE_SETUP))} && "
            f"exec {command}"
        ),
    ]


def start_process(command: str) -> subprocess.Popen:
    return subprocess.Popen(
        ros_command(command),
        start_new_session=True,
    )


def run_ros(command: str, timeout_sec: float = 8.0) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            ros_command(command),
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(
            args=command,
            returncode=124,
            stdout="",
            stderr="timeout",
        )


def stop_process(
    process: Optional[subprocess.Popen],
    name: str,
    timeout_sec: float = 10.0,
) -> None:
    if process is None or process.poll() is not None:
        return

    print(f"[종료 중] {name}")

    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)


def sanitize_name(name: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9_-]+", "_", name).strip("_")
    return clean if clean else "camera_lidar_tf"


def topic_exists(topic: str) -> bool:
    result = run_ros("ros2 topic list", timeout_sec=5.0)
    return topic in result.stdout.splitlines()


def wait_for_topic(topic: str, timeout_sec: float) -> bool:
    start = time.time()

    while time.time() - start < timeout_sec:
        if topic_exists(topic):
            print(f"[OK] {topic}")
            return True

        print(f"[대기] {topic}")
        time.sleep(1.0)

    return False


def check_camera_resolution():
    result = run_ros(
        (
            "timeout 7 ros2 topic echo --once "
            "--qos-reliability best_effort "
            f"{CAMERA_INFO_TOPIC}"
        ),
        timeout_sec=9.0,
    )

    width_match = re.search(r"width:\s*(\d+)", result.stdout)
    height_match = re.search(r"height:\s*(\d+)", result.stdout)

    width = int(width_match.group(1)) if width_match else None
    height = int(height_match.group(1)) if height_match else None

    if width == EXPECTED_WIDTH and height == EXPECTED_HEIGHT:
        print(f"[OK] Camera resolution: {width}x{height}")
    else:
        print(
            "[경고] Camera resolution 확인값: "
            f"{width}x{height}, 기대값: "
            f"{EXPECTED_WIDTH}x{EXPECTED_HEIGHT}"
        )

    return width, height


def rotation_matrix_to_quaternion(matrix) -> Tuple[float, float, float, float]:
    r00, r01, r02 = matrix[0]
    r10, r11, r12 = matrix[1]
    r20, r21, r22 = matrix[2]

    trace = r00 + r11 + r22

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r21 - r12) / scale
        qy = (r02 - r20) / scale
        qz = (r10 - r01) / scale

    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / scale
        qx = 0.25 * scale
        qy = (r01 + r10) / scale
        qz = (r02 + r20) / scale

    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / scale
        qx = (r01 + r10) / scale
        qy = 0.25 * scale
        qz = (r12 + r21) / scale

    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / scale
        qx = (r02 + r20) / scale
        qy = (r12 + r21) / scale
        qz = 0.25 * scale

    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)

    return (
        qx / norm,
        qy / norm,
        qz / norm,
        qw / norm,
    )


def load_extrinsic():
    with EXTRINSIC_YAML.open("r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    camera_frame = data["camera_frame"]
    lidar_frame = data["lidar_frame"]

    flat_rotation = data["T_camera_lidar"]["R_row_major"]
    translation = data["T_camera_lidar"]["t_xyz"]

    rotation = [
        flat_rotation[0:3],
        flat_rotation[3:6],
        flat_rotation[6:9],
    ]

    quaternion = rotation_matrix_to_quaternion(rotation)

    return data, camera_frame, lidar_frame, translation, quaternion


def transform_exists(camera_frame: str, lidar_frame: str) -> bool:
    result = run_ros(
        (
            "timeout 5 ros2 run tf2_ros tf2_echo "
            f"{shlex.quote(camera_frame)} "
            f"{shlex.quote(lidar_frame)}"
        ),
        timeout_sec=7.0,
    )

    output = result.stdout + result.stderr
    return "Translation:" in output or "At time" in output


def make_static_tf_command(
    camera_frame: str,
    lidar_frame: str,
    translation,
    quaternion,
) -> str:
    tx, ty, tz = translation
    qx, qy, qz, qw = quaternion

    return " ".join([
        "ros2 run tf2_ros static_transform_publisher",
        f"--x {tx:.15f}",
        f"--y {ty:.15f}",
        f"--z {tz:.15f}",
        f"--qx {qx:.15f}",
        f"--qy {qy:.15f}",
        f"--qz {qz:.15f}",
        f"--qw {qw:.15f}",
        f"--frame-id {shlex.quote(camera_frame)}",
        f"--child-frame-id {shlex.quote(lidar_frame)}",
    ])


def write_qos_file() -> Path:
    qos_path = HOME / ".cache/xycar_camera_lidar_tf_record_qos.yaml"
    qos_path.parent.mkdir(parents=True, exist_ok=True)

    qos_path.write_text(
        f"""
{CAMERA_TOPIC}:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile

{CAMERA_INFO_TOPIC}:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile

{RECTIFIED_TOPIC}:
  history: keep_last
  depth: 5
  reliability: best_effort
  durability: volatile

{SCAN_TOPIC}:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile

/tf:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: volatile

/tf_static:
  history: keep_last
  depth: 10
  reliability: reliable
  durability: transient_local
""".strip() + "\n",
        encoding="utf-8",
    )

    return qos_path


def save_capture_config(
    bag_path: Path,
    session: str,
    topics,
    width,
    height,
    extrinsic_data,
) -> None:
    if not bag_path.exists():
        print(f"[경고] bag 폴더가 없습니다: {bag_path}")
        return

    for source in (INTRINSIC_YAML, EXTRINSIC_YAML):
        if source.exists():
            shutil.copy2(source, bag_path / source.name)

    rule_params = (
        HOME
        / "xycar_ws/src/study/my_rule/config/rule_params.yaml"
    )

    if rule_params.exists():
        shutil.copy2(
            rule_params,
            bag_path / "rule_params_capture_snapshot.yaml",
        )

    intrinsic_data = None
    if INTRINSIC_YAML.exists():
        with INTRINSIC_YAML.open("r", encoding="utf-8") as file:
            intrinsic_data = yaml.safe_load(file)

    capture_config = {
        "recording": {
            "mode": "camera_lidar_tf",
            "session": session,
            "recorded_at": datetime.now().isoformat(timespec="seconds"),
            "bag_path": str(bag_path),
            "topics": list(topics),
        },
        "camera_validation": {
            "received_width": width,
            "received_height": height,
            "expected_width": EXPECTED_WIDTH,
            "expected_height": EXPECTED_HEIGHT,
        },
        "calibration": {
            "intrinsic_file": INTRINSIC_YAML.name,
            "extrinsic_file": EXTRINSIC_YAML.name,
            "intrinsic_used_for_rectification": True,
            "extrinsic_used_for_static_tf": True,
            "intrinsic": intrinsic_data,
            "extrinsic": extrinsic_data,
        },
        **BEV_AND_LANE_CONFIG,
    }

    with (bag_path / "capture_config.yaml").open(
        "w",
        encoding="utf-8",
    ) as file:
        yaml.safe_dump(
            capture_config,
            file,
            sort_keys=False,
            allow_unicode=True,
        )

    print(f"[OK] 설정 스냅샷 저장: {bag_path / 'capture_config.yaml'}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="1280x1024 카메라 + LiDAR + calibrated TF ROS2 bag 기록"
    )
    parser.add_argument(
        "--session",
        default="camera_lidar_tf_dataset",
        help="촬영 이름",
    )
    parser.add_argument(
        "--device",
        default="/dev/video2",
        help="카메라 장치",
    )
    parser.add_argument(
        "--record-rectified",
        action="store_true",
        help="비압축 rectified 영상도 기록",
    )
    parser.add_argument(
        "--skip-tf",
        action="store_true",
        help="TF가 이미 발행 중인 경우 새 TF 실행 생략",
    )
    args = parser.parse_args()

    if not WORKSPACE_SETUP.exists():
        raise FileNotFoundError(
            f"워크스페이스 setup 파일이 없습니다: {WORKSPACE_SETUP}"
        )

    if not INTRINSIC_YAML.exists():
        raise FileNotFoundError(
            f"내부 캘브 파일이 없습니다: {INTRINSIC_YAML}"
        )

    if not EXTRINSIC_YAML.exists():
        raise FileNotFoundError(
            f"외부 캘브 파일이 없습니다: {EXTRINSIC_YAML}"
        )

    (
        extrinsic_data,
        camera_frame,
        lidar_frame,
        translation,
        quaternion,
    ) = load_extrinsic()

    session = sanitize_name(args.session)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    bag_root = HOME / "rosbags/yolo_dataset/camera_lidar_tf"
    bag_root.mkdir(parents=True, exist_ok=True)

    bag_path = bag_root / f"{session}_{timestamp}"
    qos_path = write_qos_file()

    topics = [
        CAMERA_TOPIC,
        CAMERA_INFO_TOPIC,
        SCAN_TOPIC,
        "/tf",
        "/tf_static",
    ]

    if args.record_rectified:
        topics.append(RECTIFIED_TOPIC)

    camera_process = None
    lidar_process = None
    tf_process = None
    bag_process = None

    width = None
    height = None

    try:
        if topic_exists(CAMERA_TOPIC):
            print("[재사용] 카메라 토픽이 이미 실행 중입니다.")
        else:
            camera_command = (
                "ros2 launch app_wide_camera_calib "
                "wide_camera_rectified.launch.py "
                f"device:={shlex.quote(args.device)}"
            )

            print("[실행] Wide camera + fisheye rectification")
            camera_process = start_process(camera_command)

        if topic_exists(SCAN_TOPIC):
            print("[재사용] /scan이 이미 실행 중입니다.")
        else:
            lidar_command = (
                "ros2 launch xycar_lidar "
                "xycar_lidar.launch.py"
            )

            print("[실행] Xycar LiDAR")
            lidar_process = start_process(lidar_command)

        if not wait_for_topic(CAMERA_TOPIC, 25.0):
            raise RuntimeError(f"{CAMERA_TOPIC}이 생성되지 않았습니다.")

        if not wait_for_topic(CAMERA_INFO_TOPIC, 15.0):
            raise RuntimeError(f"{CAMERA_INFO_TOPIC}이 생성되지 않았습니다.")

        if not wait_for_topic(SCAN_TOPIC, 25.0):
            raise RuntimeError(
                "/scan이 생성되지 않았습니다. "
                "/dev/ttyLIDAR와 라이다 launch를 확인하세요."
            )

        width, height = check_camera_resolution()

        if transform_exists(camera_frame, lidar_frame):
            print(
                "[재사용] TF가 이미 존재합니다: "
                f"{camera_frame} <- {lidar_frame}"
            )

        elif args.skip_tf:
            print("[경고] --skip-tf가 지정되어 TF를 새로 발행하지 않습니다.")

        else:
            tf_command = make_static_tf_command(
                camera_frame,
                lidar_frame,
                translation,
                quaternion,
            )

            print(
                "[실행] 외부 캘브 static TF: "
                f"{camera_frame} <- {lidar_frame}"
            )

            tf_process = start_process(tf_command)
            time.sleep(2.0)

            if tf_process.poll() is not None:
                raise RuntimeError("static TF publisher가 종료되었습니다.")

            if not transform_exists(camera_frame, lidar_frame):
                raise RuntimeError(
                    "외부 캘브 TF를 확인하지 못했습니다: "
                    f"{camera_frame} <- {lidar_frame}"
                )

            print("[OK] LiDAR-Camera calibrated TF")

        bag_command = " ".join([
            "ros2 bag record",
            "--storage sqlite3",
            f"--output {shlex.quote(str(bag_path))}",
            "--qos-profile-overrides-path",
            shlex.quote(str(qos_path)),
            *[shlex.quote(topic) for topic in topics],
        ])

        print("\n================================================")
        print("카메라 + 라이다 + TF ROSBAG 기록 시작")
        print(f"세션 이름 : {session}")
        print(f"저장 경로 : {bag_path}")
        print(f"내부 캘브 : {INTRINSIC_YAML}")
        print(f"외부 캘브 : {EXTRINSIC_YAML}")
        print(f"TF        : {camera_frame} <- {lidar_frame}")
        print(f"기록 토픽 : {', '.join(topics)}")
        print("종료 방법 : Ctrl+C")
        print("모터 실행 : 없음")
        print("================================================\n")

        bag_process = start_process(bag_command)

        while True:
            if camera_process is not None and camera_process.poll() is not None:
                raise RuntimeError("카메라 프로세스가 종료되었습니다.")

            if lidar_process is not None and lidar_process.poll() is not None:
                raise RuntimeError("라이다 프로세스가 종료되었습니다.")

            if tf_process is not None and tf_process.poll() is not None:
                raise RuntimeError("TF 프로세스가 종료되었습니다.")

            if bag_process.poll() is not None:
                raise RuntimeError("rosbag 프로세스가 종료되었습니다.")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nCtrl+C 입력됨. 저장을 정상 종료합니다.")

    finally:
        stop_process(bag_process, "rosbag")
        stop_process(tf_process, "static TF")
        stop_process(lidar_process, "LiDAR")
        stop_process(camera_process, "wide camera")

        save_capture_config(
            bag_path,
            session,
            topics,
            width,
            height,
            extrinsic_data,
        )

        print(f"\n최종 저장 경로: {bag_path}")


if __name__ == "__main__":
    main()
