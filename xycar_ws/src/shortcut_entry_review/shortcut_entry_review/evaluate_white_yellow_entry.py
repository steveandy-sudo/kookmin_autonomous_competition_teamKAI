#!/usr/bin/env python3
"""Export S-window shadow montages for the white-left/yellow-right selector."""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage

from lane_seg_control.bag_montage_exporter import load_rectification
from lane_seg_control.canonical_adapter_node import (
    build_bev_geometry,
    warp_semantic_masks,
)
from lane_seg_control.lraspp_inference_node import (
    masks_from_probabilities,
    prepare_model_input,
)

from .white_yellow_entry_core import (
    WhiteYellowEntrySelector,
    render_entry_debug,
)


SOURCE_TOPIC = "/wide_camera_mjpeg/image_raw/compressed"
PANEL_SIZE = (640, 480)


def positive_float(value: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return number


def build_parser() -> argparse.ArgumentParser:
    perception_share = Path(get_package_share_directory("xycar_perception"))
    parser = argparse.ArgumentParser(
        description=(
            "Apply LR-ASPP after S and evaluate a shortcut corridor bounded "
            "by the nearest white line left of the yellow line."
        )
    )
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--source-topic", default=SOURCE_TOPIC)
    parser.add_argument("--start-offset-sec", type=float, required=True)
    parser.add_argument("--duration-sec", type=positive_float, default=12.0)
    parser.add_argument("--sample-hz", type=positive_float, default=5.0)
    parser.add_argument(
        "--model-path",
        type=Path,
        default=(
            perception_share
            / "models"
            / "kookmin_lane_lraspp_mbv3s_256x144.pt"
        ),
    )
    parser.add_argument(
        "--camera-yaml",
        type=Path,
        default=(
            perception_share
            / "config"
            / "wide_camera_fisheye_1280x1024_20260708.yaml"
        ),
    )
    parser.add_argument("--rect-balance", type=float, default=0.3)
    parser.add_argument("--cpu-threads", type=int, default=4)
    parser.add_argument("--white-confidence", type=float, default=0.50)
    parser.add_argument("--yellow-confidence", type=float, default=0.50)
    return parser


def source_overlay(
    image: np.ndarray,
    white_mask: np.ndarray,
    yellow_mask: np.ndarray,
) -> np.ndarray:
    overlay = np.zeros_like(image)
    overlay[white_mask > 0] = (255, 255, 255)
    overlay[yellow_mask > 0] = (0, 220, 255)
    selected = (white_mask > 0) | (yellow_mask > 0)
    output = image.copy()
    blended = cv2.addWeighted(image, 0.55, overlay, 0.45, 0.0)
    output[selected] = blended[selected]
    return output


def panel(image: np.ndarray, label: str) -> np.ndarray:
    resized = cv2.resize(image, PANEL_SIZE, interpolation=cv2.INTER_AREA)
    cv2.rectangle(resized, (0, 0), (PANEL_SIZE[0], 34), (16, 16, 16), -1)
    cv2.putText(
        resized,
        label,
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (240, 240, 240),
        2,
        cv2.LINE_AA,
    )
    return resized


def make_montage(
    rectified_overlay: np.ndarray,
    bev_overlay: np.ndarray,
    entry_debug: np.ndarray,
    *,
    offset_sec: float,
    s_offset_sec: float,
) -> np.ndarray:
    body = np.hstack(
        [
            panel(rectified_overlay, "LR-ASPP SOURCE OVERLAY"),
            panel(bev_overlay, "CALIBRATED BEV"),
            panel(entry_debug, "WHITE-LEFT + YELLOW-RIGHT ENTRY PATH"),
        ]
    )
    banner = np.full((42, body.shape[1], 3), 14, dtype=np.uint8)
    cv2.putText(
        banner,
        f"bag={offset_sec:.3f}s | S={s_offset_sec:.3f}s | S+{offset_sec - s_offset_sec:+.3f}s",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        (240, 240, 240),
        2,
        cv2.LINE_AA,
    )
    return np.vstack([banner, body])


def evaluate(args: argparse.Namespace) -> dict:
    import torch

    bag_path = args.bag_path.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    model_path = args.model_path.expanduser().resolve()
    camera_yaml = args.camera_yaml.expanduser().resolve()
    if not bag_path.is_dir():
        raise FileNotFoundError(f"rosbag directory not found: {bag_path}")
    if not model_path.is_file():
        raise FileNotFoundError(f"lane model not found: {model_path}")
    if not camera_yaml.is_file():
        raise FileNotFoundError(f"camera calibration not found: {camera_yaml}")
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    montage_dir = output_dir / "montages"
    montage_dir.mkdir()

    torch.set_num_threads(max(1, int(args.cpu_threads)))
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass
    cv2.setNumThreads(1)
    model = torch.jit.load(str(model_path), map_location="cpu").eval()
    model = torch.jit.optimize_for_inference(model)
    with torch.inference_mode():
        warmup = model(torch.zeros((1, 3, 144, 256), dtype=torch.float32))
    if tuple(warmup.shape[1:]) != (3, 144, 256):
        raise RuntimeError(f"unexpected model output shape: {tuple(warmup.shape)}")

    image_size, _, map1, map2 = load_rectification(
        camera_yaml, float(args.rect_balance)
    )
    geometry = build_bev_geometry(
        image_size[0],
        image_size[1],
        source_ratios=(
            0.442578,
            0.480781,
            0.688281,
            0.480781,
            0.919141,
            0.614189,
            0.190625,
            0.614189,
        ),
        destination_ratios=(0.205714, 0.794286, 0.0, 0.666666667),
        bev_width=640,
        bev_height=660,
    )
    selector = WhiteYellowEntrySelector()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    expected = "sensor_msgs/msg/CompressedImage"
    if topics.get(args.source_topic) != expected:
        raise RuntimeError(
            f"{args.source_topic} must have type {expected}; "
            f"bag has {topics.get(args.source_topic)!r}"
        )

    period_ns = int(round(1_000_000_000.0 / float(args.sample_hz)))
    start_ns = int(round(float(args.start_offset_sec) * 1_000_000_000.0))
    duration_ns = int(round(float(args.duration_sec) * 1_000_000_000.0))
    first_stamp_ns: int | None = None
    next_sample_ns: int | None = None
    rows: list[dict] = []
    source_messages = 0
    valid_frames = 0
    started = time.perf_counter()

    while reader.has_next():
        topic, serialized, storage_stamp_ns = reader.read_next()
        if topic != args.source_topic:
            continue
        source_messages += 1
        message = deserialize_message(serialized, CompressedImage)
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns <= 0:
            stamp_ns = int(storage_stamp_ns)
        if first_stamp_ns is None:
            first_stamp_ns = stamp_ns
            next_sample_ns = first_stamp_ns + start_ns
        offset_ns = stamp_ns - first_stamp_ns
        if offset_ns < start_ns:
            continue
        if offset_ns > start_ns + duration_ns:
            break
        if next_sample_ns is None or stamp_ns < next_sample_ns:
            continue
        while next_sample_ns <= stamp_ns:
            next_sample_ns += period_ns

        raw = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if raw is None:
            continue
        if (raw.shape[1], raw.shape[0]) != image_size:
            raise RuntimeError(
                f"camera frame is {raw.shape[1]}x{raw.shape[0]}, "
                f"expected {image_size[0]}x{image_size[1]}"
            )
        rectified = cv2.remap(
            raw,
            map1,
            map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        model_input = prepare_model_input(rectified, 256, 144)
        with torch.inference_mode():
            logits = model(torch.from_numpy(model_input))
            probabilities = torch.softmax(logits, dim=1)[0].cpu().numpy()
        white_small, yellow_small = masks_from_probabilities(
            probabilities,
            white_class_id=1,
            yellow_class_id=2,
            white_confidence=float(args.white_confidence),
            yellow_confidence=float(args.yellow_confidence),
        )
        white = cv2.resize(white_small, image_size, interpolation=cv2.INTER_NEAREST)
        yellow = cv2.resize(yellow_small, image_size, interpolation=cv2.INTER_NEAREST)
        bev_image, bev_white, bev_yellow, _ = warp_semantic_masks(
            rectified,
            white,
            yellow,
            geometry,
            valid_lateral_margin_px=0,
            valid_erode_px=0,
            clip_to_source_polygon=False,
        )
        result = selector.process(bev_white, bev_yellow)
        if result.valid:
            valid_frames += 1
        rectified_debug = source_overlay(rectified, white, yellow)
        bev_debug = source_overlay(bev_image, bev_white, bev_yellow)
        entry_debug = render_entry_debug(bev_white, bev_yellow, result)
        offset_sec = offset_ns / 1_000_000_000.0
        montage = make_montage(
            rectified_debug,
            bev_debug,
            entry_debug,
            offset_sec=offset_sec,
            s_offset_sec=float(args.start_offset_sec),
        )
        index = len(rows)
        filename = f"frame_{index:04d}_{offset_sec:08.3f}s.jpg"
        cv2.imwrite(
            str(montage_dir / filename),
            montage,
            [cv2.IMWRITE_JPEG_QUALITY, 92],
        )
        rows.append(
            {
                "frame_index": index,
                "timestamp_ns": stamp_ns,
                "offset_sec": offset_sec,
                "relative_to_s_sec": offset_sec - float(args.start_offset_sec),
                "valid": int(result.valid),
                "reason": result.reason,
                "white_base_x": result.white.base_x,
                "yellow_base_x": result.yellow.base_x,
                "white_centers": len(result.white.centers),
                "yellow_centers": len(result.yellow.centers),
                "white_rmse_px": result.white.rmse_px,
                "yellow_rmse_px": result.yellow.rmse_px,
                "median_separation_px": result.median_separation_px,
                "target_lateral_px": result.target_lateral_px,
                "filename": filename,
            }
        )

    manifest = output_dir / "manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)
    summary = {
        "bag_path": str(bag_path),
        "source_topic": str(args.source_topic),
        "model_path": str(model_path),
        "start_offset_sec": float(args.start_offset_sec),
        "duration_sec": float(args.duration_sec),
        "sample_hz": float(args.sample_hz),
        "source_messages_scanned": source_messages,
        "sampled_frames": len(rows),
        "valid_frames": valid_frames,
        "valid_ratio": valid_frames / len(rows) if rows else 0.0,
        "boundary_contract": "white-left + yellow-right",
        "normal_rule_backend": "existing Xbin, unchanged",
        "elapsed_processing_sec": time.perf_counter() - started,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> None:
    summary = evaluate(build_parser().parse_args())
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
