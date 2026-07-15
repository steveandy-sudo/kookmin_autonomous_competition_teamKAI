#!/usr/bin/env python3
"""Build ordered real-camera/canonical comparison frames for inspection."""

from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path

import cv2
import numpy as np


def fit_image(
    image: np.ndarray,
    width: int,
    height: int,
    interpolation: int,
) -> np.ndarray:
    scale = min(width / image.shape[1], height / image.shape[0])
    resized_width = max(1, int(round(image.shape[1] * scale)))
    resized_height = max(1, int(round(image.shape[0] * scale)))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=interpolation,
    )
    canvas = np.full((height, width, 3), 28, dtype=np.uint8)
    x = (width - resized_width) // 2
    y = (height - resized_height) // 2
    canvas[y:y + resized_height, x:x + resized_width] = resized
    return canvas


def make_pair(
    source: np.ndarray,
    canonical: np.ndarray,
    frame_index: int,
    time_sec: float,
    timestamp_ns: str,
) -> np.ndarray:
    view_width = 480
    view_height = 384
    header_height = 48
    gap = 8
    output = np.full(
        (header_height + view_height, view_width * 2 + gap, 3),
        20,
        dtype=np.uint8,
    )
    output[header_height:, :view_width] = fit_image(
        source, view_width, view_height, cv2.INTER_AREA
    )
    output[header_height:, view_width + gap:] = fit_image(
        canonical, view_width, view_height, cv2.INTER_NEAREST
    )
    cv2.putText(
        output,
        "REAL RECTIFIED CAMERA",
        (12, 31),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        "CANONICAL 1.5m MODEL INPUT",
        (view_width + gap + 12, 31),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 230, 255),
        1,
        cv2.LINE_AA,
    )
    status = f"idx={frame_index:04d}  t={time_sec:6.2f}s  stamp={timestamp_ns}"
    text_size, _ = cv2.getTextSize(
        status, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1
    )
    cv2.putText(
        output,
        status,
        (output.shape[1] - text_size[0] - 10, output.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Defaults to SESSION/debug/paired.",
    )
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    session = args.session.expanduser().resolve()
    debug_csv = session / "frame_debug.csv"
    if not debug_csv.is_file():
        raise RuntimeError(f"missing frame_debug.csv: {debug_csv}")
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else session / "debug" / "paired"
    )
    if output_dir.exists():
        if not args.overwrite and any(output_dir.iterdir()):
            raise RuntimeError(f"output directory is not empty: {output_dir}")
        if args.overwrite:
            shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with debug_csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("frame_debug.csv contains no frames")

    index_path = output_dir.parent / "paired_index.csv"
    with index_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "frame_index",
                "timestamp_ns",
                "time_sec",
                "paired_image_path",
            ],
        )
        writer.writeheader()
        for output_index, row in enumerate(rows):
            timestamp_ns = row["timestamp_ns"]
            source = cv2.imread(str(session / row["source_path"]))
            canonical = cv2.imread(str(session / row["canonical_path"]))
            if source is None or canonical is None:
                raise RuntimeError(f"missing source/canonical frame: {timestamp_ns}")
            paired = make_pair(
                source,
                canonical,
                output_index,
                float(row["time_sec"]),
                timestamp_ns,
            )
            filename = f"{output_index:06d}.jpg"
            path = output_dir / filename
            if not cv2.imwrite(
                str(path),
                paired,
                [int(cv2.IMWRITE_JPEG_QUALITY), 92],
            ):
                raise RuntimeError(f"failed to write {path}")
            writer.writerow(
                {
                    "frame_index": output_index,
                    "timestamp_ns": timestamp_ns,
                    "time_sec": row["time_sec"],
                    "paired_image_path": str(path.relative_to(session)),
                }
            )
            if (output_index + 1) % 200 == 0:
                print(f"wrote {output_index + 1}/{len(rows)} paired frames")

    print(f"wrote {len(rows)} paired frames: {output_dir}")
    print(f"index: {index_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
