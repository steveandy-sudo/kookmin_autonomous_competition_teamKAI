#!/usr/bin/env python3
"""Compare every distinct packaged lane model on identical rosbag frames."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import time

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

from lane_seg_control.camera_input import CameraRectifier
from lane_seg_control.lane_seg_path_core import (
    PathParams,
    TemporalState,
    process_lane_path,
)
from lane_seg_control.lane_seg_inference_node import merge_instance_masks
from xycar_perception.lightweight_lane_segmenter import LightweightLaneSegmenter


CAMERA_TOPIC = "/wide_camera_mjpeg/image_raw/compressed"
WHITE = (255, 255, 255)
YELLOW = (0, 220, 255)


@dataclass(frozen=True)
class ModelSpec:
    label: str
    path: Path
    kind: str
    image_size: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=6)
    parser.add_argument("--white-confidence", type=float, default=0.20)
    parser.add_argument("--yellow-confidence", type=float, default=0.40)
    parser.add_argument("--cpu-threads", type=int, default=4)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--bev-config", type=Path, required=True)
    parser.add_argument("--path-config", type=Path, required=True)
    parser.add_argument("--models-root", type=Path, required=True)
    parser.add_argument(
        "--only",
        default="",
        help="Comma-separated model label fragments to benchmark.",
    )
    return parser.parse_args()


def open_reader(path: Path):
    metadata = path / "metadata.yaml"
    compressed = (
        metadata.is_file()
        and "compression_mode: MESSAGE" in metadata.read_text(encoding="utf-8")
    )
    reader = (
        rosbag2_py.SequentialCompressionReader()
        if compressed
        else rosbag2_py.SequentialReader()
    )
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[CAMERA_TOPIC]))
    return reader


def read_camera_frames(
    path: Path,
    sample_count: int,
) -> list[tuple[int, np.ndarray]]:
    reader = open_reader(path)
    stamps: list[int] = []
    while reader.has_next():
        topic, _serialized, stamp_ns = reader.read_next()
        if topic == CAMERA_TOPIC:
            stamps.append(int(stamp_ns))
    if not stamps:
        return []

    count = max(1, min(int(sample_count), len(stamps)))
    fractions = np.linspace(0.08, 0.92, count)
    selected_indices = {
        int(round(value * (len(stamps) - 1))) for value in fractions
    }
    reader = open_reader(path)
    frames: list[tuple[int, np.ndarray]] = []
    camera_index = 0
    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        if topic != CAMERA_TOPIC:
            continue
        if camera_index not in selected_indices:
            camera_index += 1
            continue
        message = deserialize_message(serialized, CompressedImage)
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is not None:
            frames.append((int(stamp_ns), image))
        camera_index += 1
    return frames


def load_bev(path: Path) -> tuple[np.ndarray, tuple[int, int]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    source = np.asarray(data["source_points_px"], dtype=np.float32)
    destination = np.asarray(data["destination_points"], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(source, destination)
    return matrix, (int(data["bev_width"]), int(data["bev_height"]))


def load_path_params(path: Path) -> PathParams:
    import yaml

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return PathParams.from_dict(data)


def roles_from_names(names) -> dict[int, str]:
    items = names.items() if isinstance(names, dict) else enumerate(names)
    roles: dict[int, str] = {}
    for class_id, name in items:
        normalized = str(name).lower().replace("-", "_")
        if "white" in normalized:
            roles[int(class_id)] = "white"
        elif "yellow" in normalized:
            roles[int(class_id)] = "yellow"
    return roles


class YoloRunner:
    def __init__(self, spec: ModelSpec, args: argparse.Namespace) -> None:
        import torch

        torch.set_num_threads(max(1, int(args.cpu_threads)))
        self.spec = spec
        self.model = YOLO(str(spec.path), task="segment")
        self.roles = roles_from_names(self.model.names)
        self.white_confidence = float(args.white_confidence)
        self.yellow_confidence = float(args.yellow_confidence)

    def predict(self, image: np.ndarray):
        started = time.perf_counter()
        result = self.model.predict(
            source=image,
            task="segment",
            imgsz=self.spec.image_size,
            conf=min(self.white_confidence, self.yellow_confidence),
            iou=0.60,
            max_det=30,
            device="cpu",
            retina_masks=True,
            verbose=False,
        )[0]
        if result.masks is None or result.boxes is None:
            masks = np.empty((0, *image.shape[:2]), dtype=np.float32)
            classes = np.empty((0,), dtype=np.float32)
            confidences = np.empty((0,), dtype=np.float32)
        else:
            masks = result.masks.data.detach().cpu().numpy()
            classes = result.boxes.cls.detach().cpu().numpy()
            confidences = result.boxes.conf.detach().cpu().numpy()
        white, yellow, white_count, yellow_count = merge_instance_masks(
            masks,
            classes,
            confidences,
            image.shape[:2],
            self.roles,
            confidence_threshold=self.white_confidence,
            role_confidence_thresholds={
                "white": self.white_confidence,
                "yellow": self.yellow_confidence,
            },
            mask_threshold=0.5,
        )
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return white, yellow, white_count, yellow_count, elapsed_ms


class LrasppRunner:
    def __init__(self, spec: ModelSpec, args: argparse.Namespace) -> None:
        self.spec = spec
        self.model = LightweightLaneSegmenter(
            str(spec.path),
            device="cpu",
            input_width=256,
            input_height=144,
            cpu_threads=args.cpu_threads,
        )

    def predict(self, image: np.ndarray):
        started = time.perf_counter()
        white, yellow, _ = self.model.predict(image)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        return white, yellow, int(np.any(white)), int(np.any(yellow)), elapsed_ms


def overlay_masks(image: np.ndarray, white: np.ndarray, yellow: np.ndarray):
    colors = np.zeros_like(image)
    colors[white > 0] = WHITE
    colors[yellow > 0] = YELLOW
    return cv2.addWeighted(image, 0.68, colors, 0.55, 0.0)


def draw_path(bev: np.ndarray, white: np.ndarray, yellow: np.ndarray, result):
    panel = overlay_masks(bev, white, yellow)
    if result.smooth_path.shape[0] >= 2:
        points = np.rint(result.smooth_path).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(panel, [points], False, (255, 0, 255), 5, cv2.LINE_AA)
    return panel


def titled(image: np.ndarray, title: str, subtitle: str = "") -> np.ndarray:
    width = 400
    resized = cv2.resize(image, (width, 320), interpolation=cv2.INTER_AREA)
    header = np.full((54, width, 3), 24, dtype=np.uint8)
    cv2.putText(header, title, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(header, subtitle, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (190, 220, 255), 1, cv2.LINE_AA)
    return np.vstack((header, resized))


def model_specs(root: Path) -> list[ModelSpec]:
    return [
        ModelSpec("Legacy YOLO best_512", root / "lane_seg_control/models/best_512.onnx", "yolo", 512),
        ModelSpec("YOLO11n 256", root / "xycar_perception/models/kookmin_lane_yolo11n_256.onnx", "yolo", 256),
        ModelSpec("YOLO11n 512", root / "xycar_perception/models/kookmin_lane_yolo11n_512.onnx", "yolo", 512),
        ModelSpec("YOLO26n 256", root / "xycar_perception/models/kookmin_lane_yolo26n_256.onnx", "yolo", 256),
        ModelSpec("LR-ASPP 256x144", root / "lane_seg_control/models/kookmin_lane_lraspp_mbv3s_256x144.pt", "lraspp", 256),
    ]


def main() -> None:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    specs = model_specs(args.models_root.resolve())
    only = [value.strip().lower() for value in args.only.split(",") if value.strip()]
    if only:
        specs = [
            spec
            for spec in specs
            if any(fragment in spec.label.lower() for fragment in only)
        ]
    if not specs:
        raise RuntimeError(f"no model matched --only={args.only!r}")
    missing = [str(spec.path) for spec in specs if not spec.path.is_file()]
    if missing:
        raise FileNotFoundError("missing model files: " + ", ".join(missing))

    frames = read_camera_frames(args.bag.resolve(), args.frames)
    if not frames:
        raise RuntimeError(f"no {CAMERA_TOPIC} frames in {args.bag}")
    rectifier = CameraRectifier(args.calibration, balance=0.3)
    bev_matrix, bev_size = load_bev(args.bev_config)
    path_params = load_path_params(args.path_config)
    runners = [
        LrasppRunner(spec, args) if spec.kind == "lraspp" else YoloRunner(spec, args)
        for spec in specs
    ]

    summary: dict[str, list[dict[str, float | str | int]]] = {
        spec.label: [] for spec in specs
    }
    overview_rows = []
    first_stamp = frames[0][0]
    for frame_index, (stamp_ns, raw) in enumerate(frames):
        rectified = rectifier.rectify(raw)
        rows = []
        source_panel = titled(rectified, "RECTIFIED SOURCE", f"sample={frame_index + 1} t={(stamp_ns - first_stamp) / 1e9:.2f}s")
        for spec, runner in zip(specs, runners):
            white, yellow, white_count, yellow_count, elapsed_ms = runner.predict(rectified)
            white_bev = cv2.warpPerspective(white, bev_matrix, bev_size, flags=cv2.INTER_NEAREST)
            yellow_bev = cv2.warpPerspective(yellow, bev_matrix, bev_size, flags=cv2.INTER_NEAREST)
            color_bev = cv2.warpPerspective(rectified, bev_matrix, bev_size, flags=cv2.INTER_LINEAR)
            result = process_lane_path(
                color_bev,
                white_bev,
                yellow_bev,
                path_params,
                TemporalState(),
                stamp_ns / 1e9,
                "auto",
            )
            camera_panel = titled(
                overlay_masks(rectified, white, yellow),
                spec.label,
                f"camera W/Y={white_count}/{yellow_count} infer={elapsed_ms:.1f}ms",
            )
            bev_panel = titled(
                draw_path(color_bev, white_bev, yellow_bev, result),
                "BEV + DIRECT PATH",
                f"{result.path_source} conf={result.confidence:.2f} points={result.smooth_path.shape[0]}",
            )
            rows.append(np.hstack((camera_panel, bev_panel)))
            summary[spec.label].append(
                {
                    "sample": frame_index + 1,
                    "stamp_ns": stamp_ns,
                    "inference_ms": round(elapsed_ms, 3),
                    "white_pixels": int(np.count_nonzero(white)),
                    "yellow_pixels": int(np.count_nonzero(yellow)),
                    "bev_white_pixels": int(np.count_nonzero(white_bev)),
                    "bev_yellow_pixels": int(np.count_nonzero(yellow_bev)),
                    "path_source": result.path_source,
                    "path_confidence": round(float(result.confidence), 4),
                    "path_points": int(result.smooth_path.shape[0]),
                }
            )
        sheet = np.vstack((np.hstack((source_panel, source_panel)), *rows))
        output = args.output / f"lane_models_sample_{frame_index + 1:02d}.jpg"
        cv2.imwrite(str(output), sheet, [cv2.IMWRITE_JPEG_QUALITY, 92])
        overview_rows.append(cv2.resize(sheet, (800, 500), interpolation=cv2.INTER_AREA))

    cv2.imwrite(
        str(args.output / "lane_models_overview.jpg"),
        np.vstack(overview_rows),
        [cv2.IMWRITE_JPEG_QUALITY, 90],
    )
    (args.output / "metrics.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    print(args.output.resolve())


if __name__ == "__main__":
    main()
