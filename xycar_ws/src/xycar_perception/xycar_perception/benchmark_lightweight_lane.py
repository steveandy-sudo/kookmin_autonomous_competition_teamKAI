from __future__ import annotations

import argparse
from pathlib import Path
import statistics
import time

import cv2
import numpy as np

from xycar_perception.lightweight_lane_segmenter import LightweightLaneSegmenter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark the lightweight semantic lane wrapper."
    )
    parser.add_argument("--model", required=True)
    parser.add_argument("--source", default="")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--input-width", type=int, default=256)
    parser.add_argument("--input-height", type=int, default=144)
    parser.add_argument("--cpu-threads", type=int, default=1)
    parser.add_argument(
        "--restore-source-size",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Resize masks back to the source image instead of keeping 256x144 masks.",
    )
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--iterations", type=int, default=100)
    return parser.parse_args()


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round((len(ordered) - 1) * fraction)))
    return ordered[index]


def main() -> None:
    args = parse_args()
    if args.source:
        image = cv2.imread(str(Path(args.source).expanduser()))
        if image is None:
            raise FileNotFoundError(f"failed to read image: {args.source}")
    else:
        image = np.zeros((1024, 1280, 3), dtype=np.uint8)

    segmenter = LightweightLaneSegmenter(
        args.model,
        device=args.device,
        input_width=args.input_width,
        input_height=args.input_height,
        cpu_threads=args.cpu_threads,
    )
    predict = (
        segmenter.predict
        if args.restore_source_size
        else segmenter.predict_low_resolution
    )
    for _ in range(max(0, args.warmup)):
        predict(image)

    durations_ms: list[float] = []
    for _ in range(max(1, args.iterations)):
        started = time.perf_counter()
        predict(image)
        durations_ms.append((time.perf_counter() - started) * 1000.0)

    mean_ms = statistics.fmean(durations_ms)
    print(f"model={segmenter.model_path}")
    print(
        f"device={segmenter.device} input={args.input_width}x{args.input_height} "
        f"cpu_threads={args.cpu_threads} source={image.shape[1]}x{image.shape[0]} "
        f"restore_source_size={args.restore_source_size}"
    )
    print(
        "latency_ms "
        f"mean={mean_ms:.2f} median={statistics.median(durations_ms):.2f} "
        f"p95={percentile(durations_ms, 0.95):.2f} max={max(durations_ms):.2f}"
    )
    print(f"throughput_fps={1000.0 / mean_ms:.2f}")


if __name__ == "__main__":
    main()
