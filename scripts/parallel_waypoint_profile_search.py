#!/usr/bin/env python3
"""Run isolated Gazebo waypoint-controller profiles in parallel."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time


DEFAULT_PROFILES = [
    {
        "name": "baseline_c15_min7",
        "cruise_speed_command": 15.0,
        "minimum_speed_command": 7.0,
        "maximum_lateral_accel_mps2": 2.0,
        "speed_acceleration_rate_command_per_sec": 25.0,
        "speed_deceleration_rate_command_per_sec": 40.0,
        "speed_curvature_preview_m": -1.0,
        "straight_stanley_gain": 0.45,
        "straight_stanley_heading_gain": 0.55,
        "straight_steering_rate_command_per_sec": 90.0,
        "straight_steering_filter_sec": 0.16,
        "curve_controller": "stanley",
    },
    {
        "name": "stanley_c17_min7",
        "cruise_speed_command": 17.0,
        "minimum_speed_command": 7.0,
        "maximum_lateral_accel_mps2": 2.0,
        "speed_acceleration_rate_command_per_sec": 25.0,
        "speed_deceleration_rate_command_per_sec": 40.0,
        "speed_curvature_preview_m": -1.0,
        "straight_stanley_gain": 0.45,
        "straight_stanley_heading_gain": 0.55,
        "straight_steering_rate_command_per_sec": 90.0,
        "straight_steering_filter_sec": 0.16,
        "curve_controller": "stanley",
    },
    {
        "name": "stanley_c19_min7",
        "cruise_speed_command": 19.0,
        "minimum_speed_command": 7.0,
        "maximum_lateral_accel_mps2": 2.0,
        "speed_acceleration_rate_command_per_sec": 25.0,
        "speed_deceleration_rate_command_per_sec": 40.0,
        "speed_curvature_preview_m": -1.0,
        "straight_stanley_gain": 0.45,
        "straight_stanley_heading_gain": 0.55,
        "straight_steering_rate_command_per_sec": 90.0,
        "straight_steering_filter_sec": 0.16,
        "curve_controller": "stanley",
    },
    {
        "name": "stanley_c21_min7",
        "cruise_speed_command": 21.0,
        "minimum_speed_command": 7.0,
        "maximum_lateral_accel_mps2": 2.0,
        "speed_acceleration_rate_command_per_sec": 25.0,
        "speed_deceleration_rate_command_per_sec": 40.0,
        "speed_curvature_preview_m": -1.0,
        "straight_stanley_gain": 0.45,
        "straight_stanley_heading_gain": 0.55,
        "straight_steering_rate_command_per_sec": 90.0,
        "straight_steering_filter_sec": 0.16,
        "curve_controller": "stanley",
    },
    {
        "name": "stanley_c17_min8",
        "cruise_speed_command": 17.0,
        "minimum_speed_command": 8.0,
        "maximum_lateral_accel_mps2": 2.0,
        "speed_acceleration_rate_command_per_sec": 25.0,
        "speed_deceleration_rate_command_per_sec": 40.0,
        "speed_curvature_preview_m": -1.0,
        "straight_stanley_gain": 0.45,
        "straight_stanley_heading_gain": 0.55,
        "straight_steering_rate_command_per_sec": 90.0,
        "straight_steering_filter_sec": 0.16,
        "curve_controller": "stanley",
    },
    {
        "name": "stanley_c19_min8",
        "cruise_speed_command": 19.0,
        "minimum_speed_command": 8.0,
        "maximum_lateral_accel_mps2": 2.0,
        "speed_acceleration_rate_command_per_sec": 25.0,
        "speed_deceleration_rate_command_per_sec": 40.0,
        "speed_curvature_preview_m": -1.0,
        "straight_stanley_gain": 0.45,
        "straight_stanley_heading_gain": 0.55,
        "straight_steering_rate_command_per_sec": 90.0,
        "straight_steering_filter_sec": 0.16,
        "curve_controller": "stanley",
    },
    {
        "name": "stanley_c21_min8",
        "cruise_speed_command": 21.0,
        "minimum_speed_command": 8.0,
        "maximum_lateral_accel_mps2": 2.0,
        "speed_acceleration_rate_command_per_sec": 25.0,
        "speed_deceleration_rate_command_per_sec": 40.0,
        "speed_curvature_preview_m": -1.0,
        "straight_stanley_gain": 0.45,
        "straight_stanley_heading_gain": 0.55,
        "straight_steering_rate_command_per_sec": 90.0,
        "straight_steering_filter_sec": 0.16,
        "curve_controller": "stanley",
    },
    {
        "name": "stanley_c23_min8",
        "cruise_speed_command": 23.0,
        "minimum_speed_command": 8.0,
        "maximum_lateral_accel_mps2": 2.0,
        "speed_acceleration_rate_command_per_sec": 25.0,
        "speed_deceleration_rate_command_per_sec": 40.0,
        "speed_curvature_preview_m": -1.0,
        "straight_stanley_gain": 0.45,
        "straight_stanley_heading_gain": 0.55,
        "straight_steering_rate_command_per_sec": 90.0,
        "straight_steering_filter_sec": 0.16,
        "curve_controller": "stanley",
    },
]


COMMON_LAUNCH_ARGUMENTS = {
    "headless": "true",
    "enable_rviz": "false",
    "drive_enabled": "true",
    "drive_start_delay_sec": 3.0,
    "fixed_speed_command": -1.0,
    "speed_alignment_cross_track_soft_m": 0.05,
    "speed_alignment_cross_track_hard_m": 0.30,
    "speed_alignment_heading_soft_rad": 0.08,
    "speed_alignment_heading_hard_rad": 0.45,
    "path_heading_preview_m": 0.0,
    "path_curvature_preview_m": 0.10,
    "reposition_vehicle": "true",
    "start_x": -2.725,
    "start_y": 2.4456,
    "start_yaw_deg": -173.257623,
}


def stop_process_group(process: subprocess.Popen, timeout_sec: float) -> None:
    if process.poll() is not None:
        return
    for sig, wait_sec in (
        (signal.SIGINT, timeout_sec),
        (signal.SIGTERM, 4.0),
        (signal.SIGKILL, 2.0),
    ):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=wait_sec)
            return
        except subprocess.TimeoutExpired:
            continue


def run_profile(
    *,
    profile: dict,
    profile_index: int,
    project_root: Path,
    output_root: Path,
    domain_base: int,
    monitor_timeout_sec: float,
) -> dict:
    name = str(profile["name"])
    output_dir = output_root / name
    output_dir.mkdir(parents=True, exist_ok=True)
    result_json = output_dir / "result.json"
    domain_id = domain_base + profile_index
    partition = f"xycar_profile_{os.getpid()}_{domain_id}"
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    environment["ROS_LOCALHOST_ONLY"] = "1"
    environment["GZ_PARTITION"] = partition
    environment["IGN_PARTITION"] = partition

    monitor_command = [
        "python3",
        str(project_root / "scripts" / "monitor_waypoint_profile.py"),
        "--output-json",
        str(result_json),
        "--timeout-sec",
        str(monitor_timeout_sec),
    ]
    launch_arguments = {
        **COMMON_LAUNCH_ARGUMENTS,
        "project_root": str(project_root),
        **{key: value for key, value in profile.items() if key != "name"},
    }
    launch_command = [
        "ros2",
        "launch",
        "xycar_map_nav",
        "sim_custom_track_waypoint_nav.launch.py",
        *[
            f"{key}:={value}"
            for key, value in launch_arguments.items()
        ],
    ]
    (output_dir / "profile.json").write_text(
        json.dumps(profile, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    started = time.monotonic()
    with (
        (output_dir / "monitor.log").open("w", encoding="utf-8") as monitor_log,
        (output_dir / "launch.log").open("w", encoding="utf-8") as launch_log,
    ):
        monitor = subprocess.Popen(
            monitor_command,
            cwd=project_root,
            env=environment,
            stdout=monitor_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        launch = subprocess.Popen(
            launch_command,
            cwd=project_root,
            env=environment,
            stdout=launch_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            monitor.wait(timeout=monitor_timeout_sec + 30.0)
        except subprocess.TimeoutExpired:
            stop_process_group(monitor, 2.0)
        finally:
            stop_process_group(launch, 12.0)
            stop_process_group(monitor, 2.0)

    if result_json.exists():
        result = json.loads(result_json.read_text(encoding="utf-8"))
    else:
        result = {
            "reason": "monitor_failed",
            "success": False,
        }
    result.update(
        {
            "profile": name,
            "profile_parameters": profile,
            "ros_domain_id": domain_id,
            "gz_partition": partition,
            "wall_time_sec": time.monotonic() - started,
        }
    )
    result_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return result


def ranking_key(result: dict) -> tuple:
    speed = result.get("speed_command", {}).get("mean", 0.0)
    reversals = result.get("steering_command", {}).get(
        "large_straight_reversals",
        999,
    )
    cte = result.get("cross_track_error_m", {}).get("p95_abs", 99.0)
    return (
        0 if result.get("success") else 1,
        reversals,
        abs(float(speed) - 17.0),
        cte,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--project-root",
        default=str(Path(__file__).resolve().parents[1]),
    )
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--domain-base", type=int, default=40)
    parser.add_argument("--timeout-sec", type=float, default=70.0)
    parser.add_argument("--profiles-json", default="")
    parser.add_argument("--output-dir", default="")
    options = parser.parse_args()

    project_root = Path(options.project_root).expanduser().resolve()
    if options.profiles_json:
        profiles = json.loads(
            Path(options.profiles_json).read_text(encoding="utf-8")
        )
    else:
        profiles = DEFAULT_PROFILES
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = (
        Path(options.output_dir).expanduser().resolve()
        if options.output_dir
        else project_root / "analysis" / f"parallel_profile_search_{timestamp}"
    )
    output_root.mkdir(parents=True, exist_ok=True)
    print(
        f"Starting {len(profiles)} profiles with "
        f"{options.workers} parallel Gazebo instances",
        flush=True,
    )
    results = []
    with ThreadPoolExecutor(max_workers=max(1, options.workers)) as pool:
        futures = {
            pool.submit(
                run_profile,
                profile=profile,
                profile_index=index,
                project_root=project_root,
                output_root=output_root,
                domain_base=options.domain_base,
                monitor_timeout_sec=options.timeout_sec,
            ): profile["name"]
            for index, profile in enumerate(profiles)
        }
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            speed = result.get("speed_command", {}).get("mean", 0.0)
            cte = result.get("cross_track_error_m", {}).get(
                "p95_abs",
                0.0,
            )
            reversals = result.get("steering_command", {}).get(
                "large_straight_reversals",
                0,
            )
            print(
                f"[{result['profile']}] {result['reason']} "
                f"mean_speed={speed:.2f} p95_cte={cte:.3f} "
                f"large_reversals={reversals}",
                flush=True,
            )

    ranked = sorted(results, key=ranking_key)
    summary = {
        "target_mean_speed_command": 17.0,
        "workers": options.workers,
        "profiles": len(profiles),
        "ranking": ranked,
    }
    summary_path = output_root / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"Results: {summary_path}", flush=True)
    if ranked:
        print(
            f"Best profile: {ranked[0]['profile']} "
            f"({ranked[0]['reason']})",
            flush=True,
        )


if __name__ == "__main__":
    main()
