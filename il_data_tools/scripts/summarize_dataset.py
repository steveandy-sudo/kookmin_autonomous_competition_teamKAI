#!/usr/bin/env python3

import argparse
import csv
import math
from collections import Counter
from pathlib import Path
from statistics import mean, pstdev
from typing import Iterable, List


def as_float(value: str):
    try:
        number = float(value)
        if math.isfinite(number):
            return number
    except (TypeError, ValueError):
        return None
    return None


def describe(name: str, values: Iterable[float]) -> None:
    cleaned = [value for value in values if value is not None]
    if not cleaned:
        print(f"{name}: no numeric values")
        return
    print(
        f"{name}: min={min(cleaned):.4f}, max={max(cleaned):.4f}, "
        f"mean={mean(cleaned):.4f}, std={pstdev(cleaned):.4f}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize an il_data_tools dataset.")
    parser.add_argument("dataset_dir", type=Path)
    args = parser.parse_args()

    dataset_dir = args.dataset_dir.expanduser().resolve()
    samples_csv = dataset_dir / "samples.csv"
    if not samples_csv.exists():
        raise SystemExit(f"samples.csv not found: {samples_csv}")

    with samples_csv.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    timestamps: List[int] = []
    angles = []
    speeds = []
    missing_images = 0
    labels = Counter()
    for row in rows:
        try:
            timestamps.append(int(row.get("timestamp_ns") or "0"))
        except ValueError:
            pass
        labels[row.get("mission_label") or "unknown"] += 1
        angles.append(as_float(row.get("motor_angle", "")))
        speeds.append(as_float(row.get("motor_speed", "")))
        image_rel = row.get("image_front") or ""
        if not image_rel or not (dataset_dir / image_rel).exists():
            missing_images += 1

    moving = sum(1 for speed in speeds if speed is not None and abs(speed) > 1e-6)
    stopped = sum(1 for speed in speeds if speed is not None and abs(speed) <= 1e-6)
    gaps = []
    for left, right in zip(timestamps, timestamps[1:]):
        gaps.append((right - left) / 1e9)

    print(f"Dataset: {dataset_dir}")
    print(f"Total samples: {len(rows)}")
    print("Label counts:")
    for label, count in labels.most_common():
        print(f"  {label}: {count}")
    describe("Angle", angles)
    describe("Speed", speeds)
    print(f"Stopped samples: {stopped}")
    print(f"Moving samples: {moving}")
    print(f"Missing front image count: {missing_images}")
    if len(timestamps) >= 2:
        duration = (max(timestamps) - min(timestamps)) / 1e9
        rate = (len(timestamps) - 1) / duration if duration > 0 else 0.0
        print(f"Sample rate estimate: {rate:.3f} Hz over {duration:.3f} sec")
    else:
        print("Sample rate estimate: unavailable")
    print("Top 10 largest time gaps:")
    for gap in sorted(gaps, reverse=True)[:10]:
        print(f"  {gap:.4f} sec")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
