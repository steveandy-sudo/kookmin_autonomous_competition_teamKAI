import argparse
import csv
import math
from collections import defaultdict
from statistics import mean, stdev
from typing import Dict, Iterable, List, Optional


def _to_float(row: Dict[str, str], key: str) -> Optional[float]:
    value = row.get(key, "")
    if value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _mean(values: Iterable[Optional[float]]) -> Optional[float]:
    clean = [float(value) for value in values if value is not None]
    if not clean:
        return None
    return mean(clean)


def _std(values: Iterable[Optional[float]]) -> Optional[float]:
    clean = [float(value) for value in values if value is not None]
    if len(clean) < 2:
        return None
    return stdev(clean)


def _fmt(value: Optional[float], digits: int = 4) -> str:
    if value is None or math.isnan(value):
        return "-"
    return f"{value:.{digits}f}"


def _steady_samples(rows: List[Dict[str, str]]) -> List[Dict[str, str]]:
    if not rows:
        return []
    max_elapsed = max(_to_float(row, "phase_elapsed") or 0.0 for row in rows)
    cutoff = max_elapsed * 0.5
    return [row for row in rows if (_to_float(row, "phase_elapsed") or 0.0) >= cutoff]


def _best_yaw_rate(row: Dict[str, str]) -> Optional[float]:
    odom_yaw_rate = _to_float(row, "odom_yaw_rate_rad_s")
    if odom_yaw_rate is not None:
        return odom_yaw_rate
    return _to_float(row, "yaw_rate_z_rad_s")


def analyze(csv_path: str, wheelbase_m: float) -> None:
    with open(csv_path, newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    groups: Dict[str, List[Dict[str, str]]] = defaultdict(list)
    for row in rows:
        groups[row.get("phase", "")].append(row)

    print(f"log: {csv_path}")
    print("")
    print("Response events")
    found_event = False
    for row in rows:
        event = row.get("event", "")
        if event:
            found_event = True
            print(
                f"- {row.get('phase')} at {row.get('phase_elapsed')}s: {event}"
            )
    if not found_event:
        print("- no auto/manual response event was recorded")

    print("")
    print("Speed command summary")
    print("phase, speed_cmd, mean_speed_mps, speed_gain_mps_per_cmd, speed_std")
    for phase_name, phase_rows in groups.items():
        if not phase_name.startswith("speed_cmd_"):
            continue
        samples = _steady_samples(phase_rows)
        speed_cmd = abs(_to_float(samples[-1], "target_speed_cmd") or 0.0) if samples else 0.0
        speeds = [_to_float(row, "estimated_speed_mps") for row in samples]
        mean_speed = _mean(speeds)
        gain = mean_speed / speed_cmd if mean_speed is not None and speed_cmd > 1e-6 else None
        print(
            f"{phase_name}, {_fmt(speed_cmd, 2)}, {_fmt(mean_speed)}, "
            f"{_fmt(gain)}, {_fmt(_std(speeds))}"
        )

    print("")
    print("Turn radius summary")
    print(
        "phase, angle_cmd, speed_cmd, mean_speed_mps, mean_yaw_rate_rad_s, "
        "radius_m, inferred_steering_rad, steering_gain_rad_per_cmd"
    )
    for phase_name, phase_rows in groups.items():
        if not phase_name.startswith("turn_angle_"):
            continue
        samples = _steady_samples(phase_rows)
        if not samples:
            continue
        angle_cmd = abs(_to_float(samples[-1], "target_angle_cmd") or 0.0)
        speed_cmd = abs(_to_float(samples[-1], "target_speed_cmd") or 0.0)
        mean_speed = _mean(_to_float(row, "estimated_speed_mps") for row in samples)
        mean_yaw_rate = _mean(
            abs(yaw_rate) for yaw_rate in (_best_yaw_rate(row) for row in samples)
            if yaw_rate is not None
        )
        radius = None
        steering_rad = None
        steering_gain = None
        if mean_speed is not None and mean_yaw_rate is not None and mean_yaw_rate > 1e-6:
            radius = abs(mean_speed / mean_yaw_rate)
            steering_rad = math.atan2(wheelbase_m, radius)
            if angle_cmd > 1e-6:
                steering_gain = steering_rad / angle_cmd
        print(
            f"{phase_name}, {_fmt(angle_cmd, 2)}, {_fmt(speed_cmd, 2)}, "
            f"{_fmt(mean_speed)}, {_fmt(mean_yaw_rate)}, {_fmt(radius)}, "
            f"{_fmt(steering_rad)}, {_fmt(steering_gain, 6)}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze Xycar dynamics measurement CSV logs."
    )
    parser.add_argument("--csv", required=True, help="CSV file from dynamics_test_runner.")
    parser.add_argument(
        "--wheelbase-m",
        type=float,
        default=0.32,
        help="Measured wheelbase in meters.",
    )
    args = parser.parse_args()
    analyze(args.csv, args.wheelbase_m)


if __name__ == "__main__":
    main()
