from __future__ import annotations

import argparse
import csv
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import time
from typing import TextIO

import yaml


DEFAULT_OUTPUT_ROOT = Path.home() / "rosbags" / "model_gap"
UNCOMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/Image"
COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"

EXACT_TOPICS = (
    "/recording/canonical_road_image/compressed",
    "/recording/canonical_white_mask/compressed",
    "/recording/canonical_yellow_mask/compressed",
    "/recording/bev_before_canonical/compressed",
    "/recording/lane_seg_white_mask/compressed",
    "/recording/lane_seg_yellow_mask/compressed",
    "/wide_camera/rect/camera_info",
    "/imu",
    "/imu/raw_data",
    "/imu/data",
    "/odom",
    "/tf",
    "/tf_static",
    "/xycar_motor",
    "/rl/policy_motor_shadow",
    "/rl/policy_debug",
    "/rl/policy_status",
    "/rl/action_applied",
    "/il/policy_motor_shadow",
    "/il/policy_debug",
    "/il/policy_status",
    "/xycar_motor_shadow",
    "/rule_drive/diagnostics",
    "/rule_drive/connected_yellow_path",
    "/hybrid/mode",
    "/hybrid/debug",
    "/hybrid/cone_clusters",
    "/hybrid/cone_path",
    "/perception/centerline",
    "/perception/road_segments",
    "/vehicle/vesc_state",
    "/vehicle/system_telemetry",
    "/joint_states",
    "/diagnostics",
    "/parameter_events",
    "/rosout",
)

COMPRESSED_CAMERA_CANDIDATES = (
    "/wide_camera_mjpeg/image_raw/compressed",
    "/image_raw/compressed",
)

DEFERRED_DRIVING_TOPICS = (
    "/xycar_motor",
    "/rl/policy_motor_shadow",
    "/rl/policy_debug",
    "/rl/policy_status",
    "/rl/action_applied",
    "/il/policy_motor_shadow",
    "/il/policy_debug",
    "/il/policy_status",
    "/xycar_motor_shadow",
    "/rule_drive/diagnostics",
    "/rule_drive/connected_yellow_path",
    "/hybrid/mode",
    "/hybrid/debug",
    "/hybrid/cone_clusters",
    "/hybrid/cone_path",
)

REQUIRED_CANDIDATES = {
    "CAMERA": COMPRESSED_CAMERA_CANDIDATES,
    "IMU": ("/imu", "/imu/data"),
    "VESC_TELEMETRY": ("/vehicle/vesc_state",),
}

MIN_EXPECTED_HZ = {
    "/wide_camera_mjpeg/image_raw/compressed": 20.0,
    "/imu": 30.0,
    "/vehicle/vesc_state": 20.0,
    "/recording/canonical_road_image/compressed": 3.0,
    "/rl/policy_motor_shadow": 3.0,
    "/hybrid/debug": 3.0,
    "/xycar_motor": 3.0,
}

PARAMETER_NODES = (
    "/rl_policy_inference",
    "/canonical_stanley_pursuit_driver",
    "/xycar_hybrid_drive",
    "/lane_seg_lraspp_inference",
    "/lane_seg_canonical_adapter",
    "/xycar_camera_perception",
    "/wide_camera_mjpeg",
    "/wide_camera_rectifier",
    "/ebimu_serial_publisher",
    "/imu_node",
    "/xycar_lidar_node",
    "/vehicle_recording_telemetry",
    "/canonical_recording_compressor",
)


def run_command(
    command: list[str],
    *,
    timeout: float = 10.0,
) -> subprocess.CompletedProcess:
    return subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
        check=False,
    )


def safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_.-")
    return cleaned or "model_run"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def get_topics() -> dict[str, str]:
    result = run_command(
        ["ros2", "topic", "list", "-t", "--no-daemon"],
    )
    if result.returncode != 0:
        raise RuntimeError(f"failed to list ROS topics: {result.stderr.strip()}")
    topics: dict[str, str] = {}
    pattern = re.compile(r"^(\S+)\s+\[(.+)\]$")
    for line in result.stdout.splitlines():
        match = pattern.match(line.strip())
        if match:
            topics[match.group(1)] = match.group(2)
    return topics


def wait_for_topic_discovery(
    expected_topics: set[str],
    *,
    candidate_groups: tuple[tuple[str, ...], ...] = (),
    timeout_sec: float = 10.0,
) -> dict[str, str]:
    deadline = time.monotonic() + timeout_sec
    available: dict[str, str] = {}
    while time.monotonic() < deadline:
        available = get_topics()
        candidates_ready = all(
            any(candidate in available for candidate in candidates)
            for candidates in candidate_groups
        )
        if expected_topics.issubset(available) and candidates_ready:
            return available
        time.sleep(0.5)
    return available


def choose_topics(
    available: dict[str, str],
    *,
    include_vesc: bool = True,
) -> list[tuple[str, str]]:
    selected: dict[str, str] = {}
    for candidate in COMPRESSED_CAMERA_CANDIDATES:
        if available.get(candidate) == COMPRESSED_IMAGE_TYPE:
            selected[candidate] = available[candidate]
            break
    for topic in EXACT_TOPICS:
        if topic == "/vehicle/vesc_state" and not include_vesc:
            continue
        topic_type = available.get(topic)
        if topic_type and topic_type != UNCOMPRESSED_IMAGE_TYPE:
            selected[topic] = topic_type
    for topic in DEFERRED_DRIVING_TOPICS:
        selected.setdefault(
            topic,
            available.get(topic, "<runtime-discovery>"),
        )
    return sorted(selected.items())


def missing_required_groups(
    available: dict[str, str],
    *,
    require_vesc: bool,
) -> list[str]:
    required = dict(REQUIRED_CANDIDATES)
    if not require_vesc:
        required.pop("VESC_TELEMETRY", None)
    return [
        name
        for name, candidates in required.items()
        if not any(candidate in available for candidate in candidates)
    ]


def topic_has_message(topic: str, timeout_sec: float) -> bool:
    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", topic, "--once"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_sec,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return False
    return result.returncode == 0


def infer_running_model_path() -> Path | None:
    result = run_command(["ps", "-eo", "args"], timeout=5.0)
    match = re.search(r"checkpoint_path:=(?:\"([^\"]+)\"|'([^']+)'|(\S+))", result.stdout)
    if not match:
        return None
    value = next(group for group in match.groups() if group)
    path = Path(value).expanduser()
    return path.resolve() if path.exists() else path


def stop_process(process: subprocess.Popen, timeout_sec: float = 30.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)


def interrupt_main(_signum, _frame) -> None:
    raise KeyboardInterrupt


def start_support_process(
    command: list[str],
    log_path: Path,
) -> tuple[subprocess.Popen, TextIO]:
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    return process, log_file


def capture_text(command: list[str], output_path: Path) -> None:
    try:
        result = run_command(command, timeout=15.0)
        content = result.stdout
        if result.stderr:
            content += "\nSTDERR\n" + result.stderr
    except (OSError, subprocess.TimeoutExpired) as exc:
        content = f"capture failed: {exc}\n"
    output_path.write_text(content, encoding="utf-8")


def capture_runtime_context(session_path: Path, repository: Path) -> None:
    diagnostics = session_path / "diagnostics"
    diagnostics.mkdir(parents=True, exist_ok=True)
    capture_text(
        ["ros2", "node", "list", "--no-daemon"],
        diagnostics / "ros2_nodes.txt",
    )
    capture_text(
        ["ros2", "topic", "list", "-t", "--no-daemon"],
        diagnostics / "ros2_topics.txt",
    )
    capture_text(
        ["ps", "-eo", "pid,etimes,args"],
        diagnostics / "processes.txt",
    )
    capture_text(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        diagnostics / "git_head.txt",
    )
    capture_text(
        ["git", "-C", str(repository), "status", "-sb"],
        diagnostics / "git_status.txt",
    )
    capture_text(
        ["docker", "ps", "--format", "{{.Names}}\t{{.Status}}"],
        diagnostics / "docker_status.txt",
    )
    capture_text(
        ["bash", "-lc", "lsusb; echo; ls -l /dev/serial/by-* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null"],
        diagnostics / "usb_devices.txt",
    )

    nodes = set(
        run_command(
            ["ros2", "node", "list", "--no-daemon"],
            timeout=5.0,
        ).stdout.splitlines()
    )
    for node in PARAMETER_NODES:
        if node not in nodes:
            continue
        filename = safe_name(node.lstrip("/")) + ".yaml"
        capture_text(
            ["ros2", "param", "dump", node],
            diagnostics / filename,
        )


def write_rate_summary(bag_path: Path, session_path: Path) -> list[dict]:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return []
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    information = metadata["rosbag2_bagfile_information"]
    duration_sec = (
        float(information.get("duration", {}).get("nanoseconds", 0))
        / 1.0e9
    )
    rows = []
    for entry in information.get("topics_with_message_count", []):
        topic_metadata = entry["topic_metadata"]
        topic = topic_metadata["name"]
        count = int(entry.get("message_count", 0))
        mean_hz = count / duration_sec if duration_sec > 0.0 else 0.0
        minimum = MIN_EXPECTED_HZ.get(topic)
        status = (
            "EMPTY"
            if count == 0
            else "LOW"
            if minimum is not None and mean_hz < minimum
            else "OK"
        )
        rows.append(
            {
                "topic": topic,
                "type": topic_metadata["type"],
                "message_count": count,
                "bag_duration_sec": duration_sec,
                "mean_hz": mean_hz,
                "minimum_expected_hz": minimum,
                "status": status,
            }
        )
    rows.sort(key=lambda row: row["topic"])
    with (session_path / "topic_rates.csv").open(
        "w",
        newline="",
        encoding="utf-8",
    ) as output:
        writer = csv.DictWriter(
            output,
            fieldnames=(
                "topic",
                "type",
                "message_count",
                "bag_duration_sec",
                "mean_hz",
                "minimum_expected_hz",
                "status",
            ),
        )
        writer.writeheader()
        writer.writerows(rows)
    return rows


def session_size_bytes(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record a compact real-vehicle model run with compressed camera "
            "images, commands, IMU, VESC, and runtime metadata."
        )
    )
    parser.add_argument("--session", required=True)
    parser.add_argument(
        "--driver",
        choices=("rl", "rule", "hybrid", "keyboard", "manual", "unknown"),
        default="rl",
    )
    parser.add_argument("--model-path", default="")
    parser.add_argument("--model-label", default="")
    parser.add_argument("--notes", default="")
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
    )
    parser.add_argument(
        "--repository",
        default="/home/xytron/kookmin_ty/simulation_latest",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to record; zero records until Ctrl+C.",
    )
    parser.add_argument("--allow-missing", action="store_true")
    parser.add_argument("--no-vesc", action="store_true")
    parser.add_argument("--no-canonical-compression", action="store_true")
    parser.add_argument(
        "--include-perception-intermediates",
        action="store_true",
        help=(
            "Also compress masks and pre-canonical BEV. This adds perception "
            "work and should stay off for latency measurements."
        ),
    )
    parser.add_argument(
        "--compression",
        choices=("file", "none"),
        default="file",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv=None) -> None:
    signal.signal(signal.SIGTERM, interrupt_main)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, interrupt_main)
    args = make_parser().parse_args(argv)
    output_root = Path(args.output_root).expanduser().resolve()
    repository = Path(args.repository).expanduser().resolve()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_path = output_root / f"{safe_name(args.session)}_{timestamp}"
    bag_path = session_path / "bag"
    session_path.mkdir(parents=True, exist_ok=False)

    support_processes: list[subprocess.Popen] = []
    support_logs: list[TextIO] = []
    try:
        telemetry_command = [
            "ros2",
            "run",
            "xycar_run_recorder",
            "vehicle_telemetry_bridge",
        ]
        process, log_file = start_support_process(
            telemetry_command,
            session_path / "telemetry_bridge.log",
        )
        support_processes.append(process)
        support_logs.append(log_file)

        if not args.no_canonical_compression:
            compressor_command = [
                "ros2",
                "run",
                "xycar_run_recorder",
                "canonical_compressor",
            ]
            if args.include_perception_intermediates:
                compressor_command.extend(
                    [
                        "--ros-args",
                        "-p",
                        "include_intermediate_streams:=true",
                    ]
                )
            process, log_file = start_support_process(
                compressor_command,
                session_path / "canonical_compressor.log",
            )
            support_processes.append(process)
            support_logs.append(log_file)

        support_topics = {"/vehicle/system_telemetry"}
        if not args.no_vesc:
            support_topics.add("/vehicle/vesc_state")
        if not args.no_canonical_compression:
            support_topics.add(
                "/recording/canonical_road_image/compressed"
            )
        available = wait_for_topic_discovery(
            support_topics,
            candidate_groups=(
                COMPRESSED_CAMERA_CANDIDATES,
                REQUIRED_CANDIDATES["IMU"],
            ),
        )
        selected = choose_topics(
            available,
            include_vesc=not args.no_vesc,
        )
        missing = missing_required_groups(
            available,
            require_vesc=not args.no_vesc,
        )

        required_topics = []
        for group, candidates in REQUIRED_CANDIDATES.items():
            if group == "VESC_TELEMETRY" and args.no_vesc:
                continue
            selected_candidate = next(
                (candidate for candidate in candidates if candidate in available),
                None,
            )
            if selected_candidate is not None:
                required_topics.append(selected_candidate)
        no_messages = []
        for topic in required_topics:
            timeout_sec = (
                10.0 if topic == "/vehicle/vesc_state" else 5.0
            )
            if not topic_has_message(topic, timeout_sec):
                no_messages.append(topic)
        if no_messages:
            missing.extend(f"NO_MESSAGES:{topic}" for topic in no_messages)

        model_path = (
            Path(args.model_path).expanduser().resolve()
            if args.model_path
            else infer_running_model_path()
        )
        model_metadata = None
        if model_path is not None:
            model_metadata = {
                "path": str(model_path),
                "exists": model_path.exists(),
                "size_bytes": model_path.stat().st_size if model_path.exists() else None,
                "sha256": sha256_file(model_path) if model_path.exists() else None,
            }

        manifest = {
            "schema_version": 1,
            "session": args.session,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "driver": args.driver,
            "model_label": args.model_label,
            "model": model_metadata,
            "notes": args.notes,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "ros_namespace": os.environ.get("ROS_NAMESPACE", ""),
            "selected_topics": [
                {"name": topic, "type": topic_type}
                for topic, topic_type in selected
            ],
            "missing_requirements": sorted(set(missing)),
            "image_contract": (
                "Only sensor_msgs/CompressedImage is recorded. "
                "No sensor_msgs/Image topic is selected."
            ),
            "perception_intermediates_enabled": (
                args.include_perception_intermediates
            ),
            "steering_contract": (
                "/xycar_motor contains commanded steering and speed; "
                "the vehicle has no measured front-wheel steering feedback."
            ),
            "actual_motion_contract": (
                "/odom is preferred for independent m/s when available; "
                "/vehicle/vesc_state contains raw motor ERPM and m/s "
                "converted with the recorded speed_to_erpm parameters."
            ),
        }
        (session_path / "manifest.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

        print("\n===== MODEL RUN RECORDING PREFLIGHT =====")
        print(f"session: {session_path}")
        print(f"driver:  {args.driver}")
        if model_path is not None:
            model_display = str(model_path)
        elif args.model_label:
            model_display = args.model_label
        elif args.driver == "rule":
            model_display = "not applicable (rule driver)"
        else:
            model_display = "not detected"
        print(f"model:   {model_display}")
        print("topics:")
        for topic, topic_type in selected:
            print(f"  {topic:52s} [{topic_type}]")
        if missing:
            print("missing:")
            for item in sorted(set(missing)):
                print(f"  - {item}")
        if missing and not args.allow_missing:
            raise RuntimeError(
                "required sensor data is missing; start the compressed "
                "camera, IMU, and VESC motor bridge, or use "
                "--allow-missing intentionally"
            )
        if not selected:
            raise RuntimeError("no recordable topics were found")
        if args.dry_run:
            print("DRY RUN: rosbag recording was not started.")
            return

        capture_runtime_context(session_path, repository)
        command = [
            "ros2",
            "bag",
            "record",
            "--storage",
            "sqlite3",
            "--max-cache-size",
            str(64 * 1024 * 1024),
            "--output",
            str(bag_path),
        ]
        if args.compression == "file":
            command.extend(
                [
                    "--compression-mode",
                    "file",
                    "--compression-format",
                    "zstd",
                    "--compression-threads",
                    "1",
                    "--compression-queue-size",
                    "2",
                ]
            )
        command.extend(topic for topic, _ in selected)
        (session_path / "record_command.txt").write_text(
            " ".join(shlex.quote(part) for part in command) + "\n",
            encoding="utf-8",
        )

        print("\n===== RECORDING =====")
        print("Stop with Ctrl+C once. Wait for metadata and zstd finalization.")
        bag_process = subprocess.Popen(command, start_new_session=True)
        started = time.monotonic()
        try:
            while bag_process.poll() is None:
                if args.duration > 0.0 and time.monotonic() - started >= args.duration:
                    break
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\nStopping rosbag cleanly...")
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        finally:
            stop_process(bag_process, timeout_sec=60.0)

        capture_runtime_context(session_path, repository)
        rates = write_rate_summary(bag_path, session_path)
        capture_text(
            ["ros2", "bag", "info", str(bag_path)],
            session_path / "rosbag_info.txt",
        )
        manifest["finished_at"] = datetime.now().isoformat(timespec="seconds")
        manifest["recording_exit_code"] = bag_process.returncode
        manifest["session_size_bytes"] = session_size_bytes(session_path)
        manifest["topic_rates"] = rates
        (session_path / "manifest.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

        print("\n===== COMPLETE =====")
        print(f"session: {session_path}")
        print(f"bag:     {bag_path}")
        print(f"rates:   {session_path / 'topic_rates.csv'}")
        print(f"model:   {session_path / 'manifest.json'}")
    finally:
        for process in reversed(support_processes):
            stop_process(process, timeout_sec=5.0)
        for log_file in support_logs:
            log_file.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"\n[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
