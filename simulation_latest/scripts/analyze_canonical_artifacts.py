#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable, Optional

import cv2
import numpy as np


def line_centers(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    centers = []
    for row in range(mask.shape[0]):
        columns = np.flatnonzero(mask[row])
        if columns.size:
            rows.append(row)
            centers.append(float(np.median(columns)))
    return np.asarray(rows, dtype=np.float32), np.asarray(centers, dtype=np.float32)


def median_x(mask: np.ndarray) -> Optional[float]:
    _, columns = np.nonzero(mask)
    return float(np.median(columns)) if columns.size else None


def curve_excursion(mask: np.ndarray) -> Optional[float]:
    rows, centers = line_centers(mask)
    if rows.size < 12 or float(np.ptp(rows)) < 30.0:
        return None
    coefficients = np.polyfit(rows, centers, 1)
    residuals = np.abs(centers - np.polyval(coefficients, rows))
    return float(np.percentile(residuals, 95))


def percentile(values: Iterable[float], level: float) -> Optional[float]:
    array = np.asarray(list(values), dtype=np.float32)
    return float(np.percentile(array, level)) if array.size else None


def masks(image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    white = np.all(image >= 235, axis=2)
    blue, green, red = cv2.split(image)
    yellow = (
        (blue <= 80)
        & (green >= 150)
        & (red >= 180)
        & ((red.astype(np.int16) - blue.astype(np.int16)) >= 100)
    )
    return white, yellow


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure line loss, abrupt jumps, and curvature in canonical PNGs."
    )
    parser.add_argument("image_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    files = sorted(args.image_dir.expanduser().resolve().glob("*.png"))
    if not files:
        raise SystemExit(f"no PNG images found below {args.image_dir}")

    missing = {"left_white": 0, "right_white": 0, "yellow": 0}
    curve_values = {"left_white": [], "right_white": [], "yellow": []}
    relative_positions = []
    relative_jumps = []
    previous_relative = None
    high_curve_frames = []
    abrupt_jump_frames = []

    for index, path in enumerate(files):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"failed to read {path}")
        white, yellow = masks(image)
        columns = np.arange(image.shape[1])[None, :]
        line_masks = {
            "left_white": white & (columns < image.shape[1] // 2),
            "right_white": white & (columns >= image.shape[1] // 2),
            "yellow": yellow,
        }
        medians = {}
        frame_curve = 0.0
        for name, line_mask in line_masks.items():
            center = median_x(line_mask)
            medians[name] = center
            if center is None:
                missing[name] += 1
            excursion = curve_excursion(line_mask)
            if excursion is not None:
                curve_values[name].append(excursion)
                frame_curve = max(frame_curve, excursion)
        if frame_curve >= 8.0:
            high_curve_frames.append(
                {"index": index, "image": path.name, "excursion_px": frame_curve}
            )

        if all(medians[name] is not None for name in line_masks):
            corridor_mid = 0.5 * (
                float(medians["left_white"]) + float(medians["right_white"])
            )
            relative = float(medians["yellow"]) - corridor_mid
            relative_positions.append(relative)
            if previous_relative is not None:
                jump = abs(relative - previous_relative)
                relative_jumps.append(jump)
                if jump >= 8.0:
                    abrupt_jump_frames.append(
                        {"index": index, "image": path.name, "jump_px": jump}
                    )
            previous_relative = relative
        else:
            previous_relative = None

    count = len(files)
    report = {
        "image_dir": str(args.image_dir.expanduser().resolve()),
        "frame_count": count,
        "missing_ratios": {
            name: value / count for name, value in missing.items()
        },
        "curve_excursion_px": {
            name: {
                "p50": percentile(values, 50),
                "p90": percentile(values, 90),
                "p99": percentile(values, 99),
                "max": max(values) if values else None,
            }
            for name, values in curve_values.items()
        },
        "relative_center_jump_px": {
            "p50": percentile(relative_jumps, 50),
            "p90": percentile(relative_jumps, 90),
            "p99": percentile(relative_jumps, 99),
            "max": max(relative_jumps) if relative_jumps else None,
            "abrupt_threshold_px": 8.0,
            "abrupt_count": len(abrupt_jump_frames),
        },
        "high_curve_threshold_px": 8.0,
        "high_curve_count": len(high_curve_frames),
        "high_curve_examples": high_curve_frames[:30],
        "abrupt_jump_examples": abrupt_jump_frames[:30],
        "calibrated_simulation_profile": {
            "event_start_probability_per_camera_frame": 0.018,
            "duration_frames": [3, 9],
            "center_jump_px": [7.0, 22.0],
            "white_bend_px": [18.0, 58.0],
            "events": [
                "center_jump",
                "white_bend",
                "yellow_dropout",
                "white_dropout",
            ],
            "label_policy": "keep clean rule-based steering command",
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
