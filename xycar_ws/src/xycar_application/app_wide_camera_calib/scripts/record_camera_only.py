#!/usr/bin/env python3

import argparse
import os
import re
import shlex
import shutil
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import yaml


HOME = Path.home()
WORKSPACE_SETUP = HOME / "xycar_ws/install/setup.bash"

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
INTRINSIC_YAML = PACKAGE_ROOT / "config/wide_camera_fisheye_1280x1024.yaml"
EXTRINSIC_YAML = PACKAGE_ROOT / "config/lidar_camera_extrinsic_final_safe.yaml"

CAMERA_TOPIC = "/wide_camera_mjpeg/image_raw/compressed"
CAMERA_INFO_TOPIC = "/wide_camera/rect/camera_info"
RECTIFIED_TOPIC = "/wide_camera/rect/image_raw"

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
    return clean if clean else "camera_dataset"


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


def write_qos_file() -> Path:
    qos_path = HOME / ".cache/xycar_camera_record_qos.yaml"
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
            "mode": "camera_only",
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
            "intrinsic_used_for_rectification": True,
            "extrinsic_file": EXTRINSIC_YAML.name,
            "extrinsic_used_in_camera_only_recording": False,
            "intrinsic": intrinsic_data,
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
        description="1280x1024 Wide camera 실행 및 ROS2 bag 기록"
    )
    parser.add_argument(
        "--session",
        default="camera_dataset",
        help="촬영 이름. 예: traffic_light_01",
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
    args = parser.parse_args()

    if not WORKSPACE_SETUP.exists():
        raise FileNotFoundError(
            f"워크스페이스 setup 파일이 없습니다: {WORKSPACE_SETUP}"
        )

    if not INTRINSIC_YAML.exists():
        raise FileNotFoundError(
            f"내부 캘브 파일이 없습니다: {INTRINSIC_YAML}"
        )

    session = sanitize_name(args.session)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    bag_root = HOME / "rosbags/yolo_dataset/camera_only"
    bag_root.mkdir(parents=True, exist_ok=True)

    bag_path = bag_root / f"{session}_{timestamp}"
    qos_path = write_qos_file()

    topics = [
        CAMERA_TOPIC,
        CAMERA_INFO_TOPIC,
    ]

    if args.record_rectified:
        topics.append(RECTIFIED_TOPIC)

    camera_process = None
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

        if not wait_for_topic(CAMERA_TOPIC, 25.0):
            raise RuntimeError(f"{CAMERA_TOPIC}이 생성되지 않았습니다.")

        if not wait_for_topic(CAMERA_INFO_TOPIC, 15.0):
            raise RuntimeError(f"{CAMERA_INFO_TOPIC}이 생성되지 않았습니다.")

        width, height = check_camera_resolution()

        bag_command = " ".join([
            "ros2 bag record",
            "--storage sqlite3",
            f"--output {shlex.quote(str(bag_path))}",
            "--qos-profile-overrides-path",
            shlex.quote(str(qos_path)),
            *[shlex.quote(topic) for topic in topics],
        ])

        print("\n==============================================")
        print("카메라 전용 ROSBAG 기록 시작")
        print(f"세션 이름 : {session}")
        print(f"저장 경로 : {bag_path}")
        print(f"내부 캘브 : {INTRINSIC_YAML}")
        print(f"기록 토픽 : {', '.join(topics)}")
        print("종료 방법 : Ctrl+C")
        print("모터 실행 : 없음")
        print("==============================================\n")

        bag_process = start_process(bag_command)

        while True:
            if camera_process is not None and camera_process.poll() is not None:
                raise RuntimeError("카메라 프로세스가 종료되었습니다.")

            if bag_process.poll() is not None:
                raise RuntimeError("rosbag 프로세스가 종료되었습니다.")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nCtrl+C 입력됨. 저장을 정상 종료합니다.")

    finally:
        stop_process(bag_process, "rosbag")
        stop_process(camera_process, "wide camera")

        save_capture_config(
            bag_path,
            session,
            topics,
            width,
            height,
        )

        print(f"\n최종 저장 경로: {bag_path}")


if __name__ == "__main__":
    main()
