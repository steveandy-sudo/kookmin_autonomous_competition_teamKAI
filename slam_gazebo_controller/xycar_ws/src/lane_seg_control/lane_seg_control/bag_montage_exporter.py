#!/usr/bin/env python3
"""Export synchronized LR-ASPP perception montages directly from a rosbag."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

from lane_seg_control.canonical_adapter_node import (
    build_bev_geometry,
    warp_semantic_masks,
)
from lane_seg_control.lraspp_inference_node import (
    masks_from_probabilities,
    prepare_model_input,
)
from lane_seg_control.white_lane_fitter import (
    compose_fitted_canonical,
    fit_white_lane_boundaries,
    fit_yellow_centerline_reference,
    render_white_lane_fit_debug,
)
from xycar_perception.canonical_road import (
    CanonicalRoadStages,
    make_canonical_road_image_from_masks,
)


SOURCE_TOPIC = "/wide_camera_mjpeg/image_raw/compressed"
PANEL_WIDTH = 640
PANEL_HEIGHT = 512
LABEL_HEIGHT = 36
ANNOTATION_HEIGHT = 44


def sha256sum(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_motor_commands(
    bag_path: Path,
    topic_name: str,
) -> tuple[np.ndarray, np.ndarray]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topics = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    expected_type = "std_msgs/msg/Float32MultiArray"
    if topics.get(topic_name) != expected_type:
        raise RuntimeError(
            f"{topic_name} must have type {expected_type}; "
            f"bag has {topics.get(topic_name)!r}"
        )
    stamps: list[int] = []
    commands: list[tuple[float, float]] = []
    while reader.has_next():
        topic, serialized, storage_timestamp = reader.read_next()
        if topic != topic_name:
            continue
        message = deserialize_message(serialized, Float32MultiArray)
        if len(message.data) < 2:
            continue
        stamps.append(int(storage_timestamp))
        commands.append((float(message.data[0]), float(message.data[1])))
    return (
        np.asarray(stamps, dtype=np.int64),
        np.asarray(commands, dtype=np.float32).reshape(-1, 2),
    )


def nearest_motor_command(
    timestamp_ns: int,
    stamps: np.ndarray,
    commands: np.ndarray,
    tolerance_ns: int,
) -> tuple[float, float, float] | None:
    if stamps.size == 0:
        return None
    index = int(np.searchsorted(stamps, int(timestamp_ns)))
    candidates = [
        candidate
        for candidate in (index - 1, index)
        if 0 <= candidate < stamps.size
    ]
    nearest = min(
        candidates,
        key=lambda candidate: abs(int(stamps[candidate]) - timestamp_ns),
    )
    delta_ns = int(stamps[nearest]) - int(timestamp_ns)
    if abs(delta_ns) > max(0, int(tolerance_ns)):
        return None
    return (
        float(commands[nearest, 0]),
        float(commands[nearest, 1]),
        delta_ns / 1_000_000.0,
    )


def load_rectification(
    calibration_path: Path, balance: float
) -> tuple[tuple[int, int], np.ndarray, np.ndarray, np.ndarray]:
    with calibration_path.open("r", encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    size = (
        int(calibration["image_width"]),
        int(calibration["image_height"]),
    )
    camera_matrix = np.asarray(
        calibration["camera_matrix"]["data"], dtype=np.float64
    ).reshape(3, 3)
    distortion = np.asarray(
        calibration["distortion_coefficients"]["data"], dtype=np.float64
    ).reshape(4, 1)
    new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        camera_matrix,
        distortion,
        size,
        np.eye(3),
        balance=float(balance),
        new_size=size,
        fov_scale=1.0,
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        camera_matrix,
        distortion,
        np.eye(3),
        new_camera_matrix,
        size,
        cv2.CV_16SC2,
    )
    return size, new_camera_matrix, map1, map2


def make_bev_overlay(
    bev_image: np.ndarray, white: np.ndarray, yellow: np.ndarray
) -> np.ndarray:
    overlay = np.zeros_like(bev_image)
    overlay[white > 0] = (255, 255, 255)
    overlay[yellow > 0] = (0, 220, 255)
    selected = (white > 0) | (yellow > 0)
    output = bev_image.copy()
    blended = cv2.addWeighted(bev_image, 0.55, overlay, 0.45, 0.0)
    output[selected] = blended[selected]
    return output


def make_panel(
    image: np.ndarray, label: str, *, nearest: bool = False
) -> np.ndarray:
    canvas = np.full((PANEL_HEIGHT, PANEL_WIDTH, 3), 24, dtype=np.uint8)
    available_height = PANEL_HEIGHT - LABEL_HEIGHT
    scale = min(
        PANEL_WIDTH / float(image.shape[1]),
        available_height / float(image.shape[0]),
    )
    output_size = (
        max(1, int(round(image.shape[1] * scale))),
        max(1, int(round(image.shape[0] * scale))),
    )
    interpolation = cv2.INTER_NEAREST if nearest else cv2.INTER_AREA
    resized = cv2.resize(image, output_size, interpolation=interpolation)
    x = (PANEL_WIDTH - resized.shape[1]) // 2
    y = LABEL_HEIGHT + (available_height - resized.shape[0]) // 2
    canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
    cv2.putText(
        canvas,
        label,
        (12, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (235, 235, 235),
        2,
        cv2.LINE_AA,
    )
    return canvas


def make_montage(
    rectified: np.ndarray,
    bev: np.ndarray,
    canonical: np.ndarray,
    canonical_fit_debug: np.ndarray | None = None,
    annotation: str | None = None,
) -> np.ndarray:
    panels = [
        make_panel(rectified, "RECTIFIED SOURCE"),
        make_panel(bev, "BEV BEFORE CANONICAL (RAW)"),
        make_panel(canonical, "CANONICAL WHITE FIT (RAW YELLOW)", nearest=True),
    ]
    if canonical_fit_debug is not None:
        panels.append(
            make_panel(
                canonical_fit_debug,
                "CANONICAL SLIDING-WINDOW DEBUG",
                nearest=True,
            )
        )
    montage = np.hstack(panels)
    if annotation:
        banner = np.full(
            (ANNOTATION_HEIGHT, montage.shape[1], 3),
            16,
            dtype=np.uint8,
        )
        cv2.putText(
            banner,
            annotation,
            (12, 29),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.66,
            (240, 240, 240),
            2,
            cv2.LINE_AA,
        )
        montage = np.vstack([banner, montage])
    return montage


def build_argument_parser() -> argparse.ArgumentParser:
    lane_share = Path(get_package_share_directory("lane_seg_control"))
    camera_share = Path(get_package_share_directory("xycar_perception"))
    rl_share = Path(get_package_share_directory("xycar_rl"))
    parser = argparse.ArgumentParser(
        description=(
            "Sample compressed camera frames from a rosbag and export "
            "rectified/BEV/canonical LR-ASPP montages."
        )
    )
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--start-offset-sec", type=float, default=0.0)
    parser.add_argument("--source-topic", default=SOURCE_TOPIC)
    parser.add_argument("--motor-topic", default="/xycar_motor")
    parser.add_argument("--motor-sync-tolerance-sec", type=float, default=0.10)
    parser.add_argument(
        "--model-path",
        type=Path,
        default=lane_share / "models" / "kookmin_lane_lraspp_mbv3s_256x144.pt",
    )
    parser.add_argument(
        "--camera-yaml",
        type=Path,
        default=(
            camera_share / "config" / "wide_camera_fisheye_1280x1024.yaml"
        ),
    )
    parser.add_argument("--balance", type=float, default=0.3)
    parser.add_argument("--white-confidence", type=float, default=0.5)
    parser.add_argument("--yellow-confidence", type=float, default=0.5)
    parser.add_argument("--cpu-threads", type=int, default=4)
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--white-fit-window-count", type=int, default=9)
    parser.add_argument("--white-fit-margin-px", type=int, default=24)
    parser.add_argument("--white-fit-min-pixels", type=int, default=4)
    parser.add_argument("--white-fit-min-centers", type=int, default=2)
    parser.add_argument("--white-fit-min-span-px", type=int, default=8)
    parser.add_argument("--white-fit-residual-px", type=float, default=6.0)
    parser.add_argument("--white-fit-line-width-px", type=int, default=5)
    parser.add_argument("--disable-yellow-divider", action="store_true")
    parser.add_argument("--yellow-divider-min-pixels", type=int, default=3)
    parser.add_argument("--yellow-divider-residual-px", type=float, default=6.0)
    parser.add_argument("--yellow-divider-line-width-px", type=int, default=5)
    parser.add_argument(
        "--rl-checkpoint",
        type=Path,
        default=(
            rl_share
            / "models"
            / "high_speed_td3_bc_focus_v3_20260717"
            / "camera_speed_td3_bc_epoch_042.pth"
        ),
    )
    parser.add_argument("--rl-input-width", type=int, default=160)
    parser.add_argument("--rl-input-height", type=int, default=90)
    parser.add_argument("--rl-max-steering-command", type=float, default=42.0)
    return parser


def export_montages(args: argparse.Namespace) -> Path:
    import torch

    from il_data_tools.runtime_preprocessing import preprocess_bgr_image
    from xycar_rl.camera_speed_models import denormalize_speed_command
    from xycar_rl.policy_loader import load_camera_speed_policy

    bag_path = args.bag_path.expanduser().resolve()
    model_path = args.model_path.expanduser().resolve()
    rl_checkpoint = args.rl_checkpoint.expanduser().resolve()
    camera_yaml = args.camera_yaml.expanduser().resolve()
    if not bag_path.is_dir():
        raise FileNotFoundError(f"rosbag directory not found: {bag_path}")
    if not model_path.is_file():
        raise FileNotFoundError(f"TorchScript model not found: {model_path}")
    if not camera_yaml.is_file():
        raise FileNotFoundError(f"camera calibration not found: {camera_yaml}")
    if not rl_checkpoint.is_file():
        raise FileNotFoundError(f"RL checkpoint not found: {rl_checkpoint}")
    if args.sample_hz <= 0.0:
        raise ValueError("sample_hz must be positive")

    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else bag_path.parent / f"{bag_path.name}_lraspp_montages_10hz"
    )
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    image_dir = output_dir / "montages"
    image_dir.mkdir(exist_ok=True)

    cv2.setNumThreads(1)
    torch.set_num_threads(max(1, int(args.cpu_threads)))
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass
    model = torch.jit.load(str(model_path), map_location="cpu").eval()
    model = torch.jit.optimize_for_inference(model)
    warmup = torch.zeros((1, 3, 144, 256), dtype=torch.float32)
    with torch.inference_mode():
        output = model(warmup)
    if not isinstance(output, torch.Tensor) or tuple(output.shape[1:]) != (
        3,
        144,
        256,
    ):
        raise RuntimeError(f"unexpected model output shape: {tuple(output.shape)}")
    rl_policy, rl_payload = load_camera_speed_policy(
        rl_checkpoint,
        device="cpu",
    )
    rl_policy.reset()
    rl_min_speed = float(rl_payload.get("min_speed_command", 4.0))
    rl_max_speed = float(rl_payload.get("max_speed_command", 12.0))
    rl_epoch = int(rl_payload.get("epoch", -1))
    motor_stamps, motor_commands = read_motor_commands(
        bag_path,
        args.motor_topic,
    )
    motor_tolerance_ns = int(
        max(0.0, float(args.motor_sync_tolerance_sec)) * 1_000_000_000
    )

    image_size, new_camera_matrix, map1, map2 = load_rectification(
        camera_yaml, args.balance
    )
    geometry = build_bev_geometry(
        image_size[0],
        image_size[1],
        source_ratios=(
            472.0 / 1280.0,
            494.0 / 1024.0,
            906.0 / 1280.0,
            486.0 / 1024.0,
            1272.0 / 1280.0,
            612.0 / 1024.0,
            46.0 / 1280.0,
            622.0 / 1024.0,
        ),
        destination_ratios=(
            80.0 / 640.0,
            560.0 / 640.0,
            0.0,
            479.0 / 660.0,
        ),
        bev_width=640,
        bev_height=660,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    expected_type = "sensor_msgs/msg/CompressedImage"
    if topics.get(args.source_topic) != expected_type:
        raise RuntimeError(
            f"{args.source_topic} must have type {expected_type}; "
            f"bag has {topics.get(args.source_topic)!r}"
        )

    period_ns = int(round(1_000_000_000.0 / float(args.sample_hz)))
    next_sample_ns: int | None = None
    source_first_stamp_ns: int | None = None
    frame_index = 0
    source_messages = 0
    motor_matches = 0
    first_stamp_ns: int | None = None
    last_stamp_ns: int | None = None
    started = time.perf_counter()
    manifest_path = output_dir / "manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as manifest:
        writer = csv.writer(manifest)
        writer.writerow(
            [
                "frame_index",
                "timestamp_ns",
                "filename",
                "bev_white_pixels",
                "bev_yellow_pixels",
                "raw_canonical_white_pixels",
                "fitted_canonical_white_pixels",
                "canonical_yellow_pixels",
                "left_degree",
                "left_centers",
                "left_rmse_px",
                "right_degree",
                "right_centers",
                "right_rmse_px",
                "yellow_divider_reference_pixels",
                "yellow_divider_valid",
                "yellow_component_count",
                "yellow_divider_rmse_px",
                "camera_storage_timestamp_ns",
                "recorded_steering_command",
                "recorded_speed_command",
                "motor_time_delta_ms",
                "rl_steering_norm",
                "rl_steering_command",
                "rl_speed_command",
            ]
        )
        while reader.has_next():
            topic, serialized, storage_timestamp = reader.read_next()
            if topic != args.source_topic:
                continue
            source_messages += 1
            message = deserialize_message(serialized, CompressedImage)
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            if stamp_ns <= 0:
                stamp_ns = int(storage_timestamp)
            if source_first_stamp_ns is None:
                source_first_stamp_ns = stamp_ns
                next_sample_ns = source_first_stamp_ns + int(
                    max(0.0, float(args.start_offset_sec)) * 1_000_000_000
                )
            if next_sample_ns is None:
                next_sample_ns = stamp_ns
            if stamp_ns < next_sample_ns:
                continue
            while next_sample_ns <= stamp_ns:
                next_sample_ns += period_ns

            encoded = np.frombuffer(message.data, dtype=np.uint8)
            raw = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if raw is None:
                print(f"warning: JPEG decode failed at {stamp_ns}", flush=True)
                continue
            if (raw.shape[1], raw.shape[0]) != image_size:
                raise RuntimeError(
                    f"camera frame is {raw.shape[1]}x{raw.shape[0]}, "
                    f"calibration is {image_size[0]}x{image_size[1]}"
                )
            rectified = cv2.remap(
                raw,
                map1,
                map2,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )

            model_input = prepare_model_input(rectified, 256, 144)
            tensor = torch.from_numpy(model_input)
            with torch.inference_mode():
                logits = model(tensor)
                probabilities = torch.softmax(logits, dim=1)[0].cpu().numpy()
            white_small, yellow_small = masks_from_probabilities(
                probabilities,
                white_class_id=1,
                yellow_class_id=2,
                white_confidence=args.white_confidence,
                yellow_confidence=args.yellow_confidence,
            )
            white = cv2.resize(
                white_small, image_size, interpolation=cv2.INTER_NEAREST
            )
            yellow = cv2.resize(
                yellow_small, image_size, interpolation=cv2.INTER_NEAREST
            )
            bev_image, bev_white, bev_yellow, valid = warp_semantic_masks(
                rectified,
                white,
                yellow,
                geometry,
                valid_lateral_margin_px=0,
                valid_erode_px=0,
                clip_to_source_polygon=False,
            )
            stages = make_canonical_road_image_from_masks(
                bev_white,
                bev_yellow,
                valid_mask=valid,
                lateral_m_per_px=1.4 / 640.0,
                forward_m_per_px=1.5 / 660.0,
                lateral_range_m=1.4,
                forward_range_m=1.5,
                output_width=256,
                output_height=144,
                background_gray=36,
                line_width_px=5,
                min_component_area_px=1,
                white_max_component_thickness_px=0.0,
                yellow_max_component_thickness_px=0.0,
                geometry_filter_enabled=False,
                preserve_white_mask=True,
                top_ignore_m=0.0,
                bottom_ignore_m=0.0,
                return_stages=True,
            )
            if not isinstance(stages, CanonicalRoadStages):
                raise RuntimeError("canonical stage output was not returned")
            raw_stages = stages
            yellow_before_fit = raw_stages.yellow_mask.copy()
            yellow_reference = None
            if not args.disable_yellow_divider:
                yellow_reference = fit_yellow_centerline_reference(
                    raw_stages.yellow_mask,
                    min_pixels=args.yellow_divider_min_pixels,
                    residual_threshold_px=args.yellow_divider_residual_px,
                    line_width_px=args.yellow_divider_line_width_px,
                )
            divider = (
                yellow_reference.x_by_y
                if yellow_reference is not None and yellow_reference.valid
                else None
            )
            white_fit = fit_white_lane_boundaries(
                raw_stages.white_mask,
                window_count=args.white_fit_window_count,
                window_margin_px=args.white_fit_margin_px,
                min_pixels_per_window=args.white_fit_min_pixels,
                min_centers=args.white_fit_min_centers,
                min_span_px=args.white_fit_min_span_px,
                residual_threshold_px=args.white_fit_residual_px,
                line_width_px=args.white_fit_line_width_px,
                divider_x_by_y=divider,
            )
            output_yellow = raw_stages.yellow_mask
            fitted_road, fitted_white = compose_fitted_canonical(
                white_fit.mask,
                output_yellow,
                raw_stages.valid_mask,
                background_gray=36,
            )
            if not np.array_equal(yellow_before_fit, raw_stages.yellow_mask):
                raise RuntimeError("canonical yellow mask changed during white fit")
            canonical_fit_debug = render_white_lane_fit_debug(
                raw_stages.road_image,
                raw_stages.white_mask,
                raw_stages.yellow_mask,
                white_fit,
                yellow_reference,
            )
            rl_image = preprocess_bgr_image(
                fitted_road,
                int(args.rl_input_width),
                int(args.rl_input_height),
            )
            rl_action = rl_policy({"image": rl_image})
            rl_steering_norm = float(rl_action[0])
            rl_steering_command = (
                rl_steering_norm * float(args.rl_max_steering_command)
            )
            rl_speed_command = denormalize_speed_command(
                float(rl_action[1]),
                rl_min_speed,
                rl_max_speed,
            )
            motor = nearest_motor_command(
                int(storage_timestamp),
                motor_stamps,
                motor_commands,
                motor_tolerance_ns,
            )
            if motor is None:
                recorded_steering = float("nan")
                recorded_speed = float("nan")
                motor_delta_ms = float("nan")
                recorded_text = "REC_CMD steer=N/A speed=N/A"
            else:
                recorded_steering, recorded_speed, motor_delta_ms = motor
                motor_matches += 1
                recorded_text = (
                    f"REC_CMD steer={recorded_steering:+.2f} "
                    f"speed={recorded_speed:.2f} dt={motor_delta_ms:+.1f}ms"
                )
            annotation = (
                f"t={stamp_ns} | {recorded_text} | "
                f"RL Focus-v3 E{rl_epoch} raw steer="
                f"{rl_steering_command:+.2f} "
                f"(norm={rl_steering_norm:+.3f}) "
                f"speed={rl_speed_command:.2f}"
            )
            montage = make_montage(
                rectified,
                make_bev_overlay(bev_image, bev_white, bev_yellow),
                fitted_road,
                canonical_fit_debug,
                annotation,
            )
            filename = f"frame_{frame_index:06d}_{stamp_ns}.jpg"
            saved = cv2.imwrite(
                str(image_dir / filename),
                montage,
                [cv2.IMWRITE_JPEG_QUALITY, int(args.jpeg_quality)],
            )
            if not saved:
                raise RuntimeError(f"failed to save montage: {filename}")
            writer.writerow(
                [
                    frame_index,
                    stamp_ns,
                    filename,
                    int(np.count_nonzero(bev_white)),
                    int(np.count_nonzero(bev_yellow)),
                    int(np.count_nonzero(raw_stages.white_mask)),
                    int(np.count_nonzero(fitted_white)),
                    int(np.count_nonzero(raw_stages.yellow_mask)),
                    white_fit.left.degree,
                    int(white_fit.left.centers.shape[0]),
                    white_fit.left.rmse_px,
                    white_fit.right.degree,
                    int(white_fit.right.centers.shape[0]),
                    white_fit.right.rmse_px,
                    (
                        int(np.count_nonzero(yellow_reference.mask))
                        if yellow_reference is not None
                        and yellow_reference.valid
                        else 0
                    ),
                    int(
                        yellow_reference is not None
                        and yellow_reference.valid
                    ),
                    (
                        yellow_reference.component_count
                        if yellow_reference is not None
                        else 0
                    ),
                    (
                        yellow_reference.rmse_px
                        if yellow_reference is not None
                        else float("inf")
                    ),
                    int(storage_timestamp),
                    recorded_steering,
                    recorded_speed,
                    motor_delta_ms,
                    rl_steering_norm,
                    rl_steering_command,
                    rl_speed_command,
                ]
            )
            first_stamp_ns = stamp_ns if first_stamp_ns is None else first_stamp_ns
            last_stamp_ns = stamp_ns
            frame_index += 1
            if frame_index % 100 == 0:
                elapsed = time.perf_counter() - started
                print(
                    f"exported {frame_index} montages "
                    f"({frame_index / max(elapsed, 1e-6):.1f} frames/s)",
                    flush=True,
                )
            if args.max_frames > 0 and frame_index >= args.max_frames:
                break

    elapsed = time.perf_counter() - started
    summary = {
        "bag_path": str(bag_path),
        "source_topic": args.source_topic,
        "source_messages_scanned": source_messages,
        "sample_hz": float(args.sample_hz),
        "start_offset_sec": float(args.start_offset_sec),
        "montage_count": frame_index,
        "first_timestamp_ns": first_stamp_ns,
        "last_timestamp_ns": last_stamp_ns,
        "elapsed_seconds": elapsed,
        "processing_frames_per_second": frame_index / max(elapsed, 1e-6),
        "model_path": str(model_path),
        "model_sha256": sha256sum(model_path),
        "model_input": "RGB ImageNet normalized 256x144",
        "class_mapping": {"0": "background", "1": "white", "2": "yellow"},
        "white_confidence": float(args.white_confidence),
        "yellow_confidence": float(args.yellow_confidence),
        "camera_yaml": str(camera_yaml),
        "rectification_balance": float(args.balance),
        "rectified_camera_matrix": new_camera_matrix.tolist(),
        "bev_size": [640, 660],
        "canonical_size": [256, 144],
        "canonical_white_fit": {
            "stage": "after canonical conversion",
            "window_count": int(args.white_fit_window_count),
            "margin_px": int(args.white_fit_margin_px),
            "min_pixels": int(args.white_fit_min_pixels),
            "min_centers": int(args.white_fit_min_centers),
            "min_span_px": int(args.white_fit_min_span_px),
            "residual_px": float(args.white_fit_residual_px),
            "line_width_px": int(args.white_fit_line_width_px),
            "side_classifier": "yellow divider, single-lane fallback",
        },
        "canonical_yellow_divider": {
            "enabled": not bool(args.disable_yellow_divider),
            "model": "robust straight x(y), extended to full height",
            "usage": "internal white-lane side classification only",
            "canonical_yellow_output": "original fragmented mask",
            "min_pixels": int(args.yellow_divider_min_pixels),
            "residual_px": float(args.yellow_divider_residual_px),
            "line_width_px": int(args.yellow_divider_line_width_px),
        },
        "recorded_motor_command": {
            "topic": args.motor_topic,
            "messages": int(motor_stamps.size),
            "matched_frames": int(motor_matches),
            "sync_tolerance_sec": float(args.motor_sync_tolerance_sec),
            "note": "command value, not measured physical steering feedback",
        },
        "rl_prediction": {
            "policy_kind": "camera_speed_td3_bc",
            "checkpoint": str(rl_checkpoint),
            "checkpoint_sha256": sha256sum(rl_checkpoint),
            "epoch": rl_epoch,
            "temporal_frames": int(rl_policy.temporal_frames),
            "input_size": [int(args.rl_input_width), int(args.rl_input_height)],
            "max_steering_command": float(args.rl_max_steering_command),
            "min_speed_command": rl_min_speed,
            "max_speed_command": rl_max_speed,
            "output": "raw actor prediction before runtime stabilizer",
        },
        "montage_size": [
            PANEL_WIDTH * 4,
            PANEL_HEIGHT + ANNOTATION_HEIGHT,
        ],
    }
    with (output_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=True)
        stream.write("\n")
    print(
        f"complete: {frame_index} montages in {elapsed:.1f}s -> {output_dir}",
        flush=True,
    )
    return output_dir


def main() -> None:
    args = build_argument_parser().parse_args()
    export_montages(args)


if __name__ == "__main__":
    main()
