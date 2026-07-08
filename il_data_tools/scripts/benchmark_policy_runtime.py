#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Benchmark TorchScript policy runtime.")
    parser.add_argument("--model", required=True)
    parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="auto")
    parser.add_argument("--iters", type=int, default=500)
    parser.add_argument("--warmup", type=int, default=50)
    parser.add_argument("--use-phase", action="store_true")
    parser.add_argument("--input-width", type=int, default=160)
    parser.add_argument("--input-height", type=int, default=90)
    parser.add_argument("--output-json", default="")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    import torch

    device = choose_device(args.device, torch)
    if args.device == "cuda" and device.type != "cuda":
        print("WARN CUDA requested but unavailable; falling back to CPU.")
    model_path = Path(args.model).expanduser().resolve()
    model = torch.jit.load(str(model_path), map_location=device)
    model.eval()
    image = torch.zeros(1, 3, args.input_height, args.input_width, device=device)
    phase = torch.zeros(1, 1, device=device)

    with torch.no_grad():
        for _ in range(args.warmup):
            run_model(model, image, phase, args.use_phase)
        sync(device, torch)
        latencies = []
        for _ in range(args.iters):
            started = time.perf_counter()
            run_model(model, image, phase, args.use_phase)
            sync(device, torch)
            latencies.append((time.perf_counter() - started) * 1000.0)

    p50 = percentile(latencies, 50)
    p95 = percentile(latencies, 95)
    avg = statistics.fmean(latencies) if latencies else 0.0
    result = {
        "model": str(model_path),
        "device": str(device),
        "iters": args.iters,
        "warmup": args.warmup,
        "use_phase": args.use_phase,
        "avg_latency_ms": avg,
        "p50_latency_ms": p50,
        "p95_latency_ms": p95,
        "max_latency_ms": max(latencies) if latencies else 0.0,
        "fps": 1000.0 / avg if avg > 0 else 0.0,
        "guidance": latency_guidance(p95),
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print_guidance()
    if args.output_json:
        output_path = Path(args.output_json).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )


def run_model(model, image, phase, use_phase: bool):
    if use_phase:
        return model(image, phase)
    return model(image)


def sync(device, torch_module) -> None:
    if device.type == "cuda":
        torch_module.cuda.synchronize()


def choose_device(name: str, torch_module):
    if name == "auto":
        return torch_module.device("cuda" if torch_module.cuda.is_available() else "cpu")
    if name == "cuda" and not torch_module.cuda.is_available():
        return torch_module.device("cpu")
    return torch_module.device(name)


def percentile(values, percent: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round((percent / 100.0) * (len(ordered) - 1)))
    return ordered[max(0, min(len(ordered) - 1, index))]


def latency_guidance(p95: float) -> str:
    if p95 < 20:
        return "excellent"
    if p95 < 35:
        return "good"
    if p95 < 50:
        return "acceptable"
    if p95 < 80:
        return "risky"
    return "too_slow"


def print_guidance() -> None:
    print("Guidance: <20 ms excellent, 20~35 ms good, 35~50 ms acceptable,")
    print("          50~80 ms risky, >80 ms too slow for real-time driving.")


if __name__ == "__main__":
    main()
