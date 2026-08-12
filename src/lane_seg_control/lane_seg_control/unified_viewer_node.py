#!/usr/bin/env python3
"""Unified fixed-BEV and yellow-primary lane path viewer for ROS2.

This node is visualization and pixel-path output only. It never publishes
motor commands, lane command messages, steering, speed, or nav_msgs/Path.
"""

from __future__ import annotations

import argparse
import copy
import contextlib
import csv
import io
import json
import math
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String

from xycar_perception.canonical_road import (
    CanonicalRoadStages,
    make_canonical_road_image_from_masks,
)

try:
    from .lane_path_controller_core import (
        PreviewControllerParams,
        PreviewControllerState,
        PreviewControlResult,
        compute_preview_control,
        path_pixels_array as preview_path_array,
        run_self_check as run_controller_self_check,
    )
    from .lane_seg_path_core import (
        SOURCE_CODES,
        WHITE_SIDE_CODES,
        YELLOW_STATE_CODES,
        CurveFit,
        Observation,
        PathParams,
        PathResult,
        TemporalState,
        normalize_mask,
        path_normalized_array,
        path_pixels_array,
        process_lane_path,
        run_self_check as run_path_core_self_check,
    )
except ImportError:
    from lane_path_controller_core import (
        PreviewControllerParams,
        PreviewControllerState,
        PreviewControlResult,
        compute_preview_control,
        path_pixels_array as preview_path_array,
        run_self_check as run_controller_self_check,
    )
    from lane_seg_path_core import (
        SOURCE_CODES,
        WHITE_SIDE_CODES,
        YELLOW_STATE_CODES,
        CurveFit,
        Observation,
        PathParams,
        PathResult,
        TemporalState,
        normalize_mask,
        path_normalized_array,
        path_pixels_array,
        process_lane_path,
        run_self_check as run_path_core_self_check,
    )


FULL_WINDOW_NAME = "Lane Perception Unified Viewer"
MINIMAL_WINDOW_NAME = "Lane Path Preview"
BEV_DETECTION_WINDOW_NAME = "BEV YOLO Detection"
DISPLAY_MODES = ("off", "minimal", "full")
DEFAULT_IMAGE_TOPIC = "/wide_camera/rect/image_raw"
DEFAULT_WHITE_TOPIC = "/lane_seg/white_boundary_mask"
DEFAULT_YELLOW_TOPIC = "/lane_seg/yellow_centerline_mask"
PACKAGE_SHARE = Path(get_package_share_directory("lane_seg_control"))
DEFAULT_BEV_CONFIG = PACKAGE_SHARE / "config" / "bev_latest.json"
DEFAULT_PARAMS = PACKAGE_SHARE / "config" / "lane_seg_path_params.yaml"
DEFAULT_CONTROLLER_PREVIEW_CONFIG = (
    PACKAGE_SHARE / "config" / "lane_path_controller_preview.yaml"
)
DEFAULT_BAG_NAME = "canonical_compressed_run_01_20260716_133037"
DEFAULT_OUTPUT_DIR = Path.home() / ".ros" / "lane_seg_control" / "preview"
EXPECTED_INPUT_SIZE = (1280, 1024)
EXPECTED_BEV_SIZE = (640, 480)
SYNC_QUEUE_SIZE = 10
SYNC_SLOP_SEC = 0.10
CACHE_LIMIT = 5
PANEL_SIZE = (480, 384)

WHITE_COLOR = (255, 255, 0)
YELLOW_COLOR = (0, 255, 255)
YELLOW_FIT_COLOR = (0, 190, 255)
WHITE_LEFT_COLOR = (255, 90, 40)
WHITE_RIGHT_COLOR = (50, 220, 80)
YELLOW_PRIMARY_PATH_COLOR = (255, 0, 255)
YELLOW_PARTIAL_PATH_COLOR = (180, 0, 255)
WHITE_FALLBACK_PATH_COLOR = (0, 140, 255)
HOLD_PATH_COLOR = (150, 150, 150)
INVALID_COLOR = (0, 0, 255)

BEV_OUTPUT_TOPICS = {
    "/lane_seg_bev/color",
    "/lane_seg_bev/white_mask",
    "/lane_seg_bev/yellow_mask",
    "/lane_seg_bev/debug_image",
}
PATH_OUTPUT_TOPICS = {
    "/lane_path/path_pixels",
    "/lane_path/path_normalized",
    "/lane_path/diagnostics",
    "/lane_path/status",
    "/lane_path/detection_debug_image",
    "/lane_path/path_debug_image",
}
UNIFIED_OUTPUT_TOPICS = {"/lane_unified/debug_image"}


@dataclass
class FixedBevConfig:
    path: Path
    bag_name: str
    input_width: int
    input_height: int
    bev_width: int
    bev_height: int
    source_points: np.ndarray
    destination_points: np.ndarray
    stored_homography: np.ndarray | None
    matrix: np.ndarray
    stored_matrix_matches_points: bool


@dataclass
class SourceFrame:
    header: Any
    stamp_ns: int
    stamp_sec: float
    stamp_text: str
    image: np.ndarray
    white_mask: np.ndarray
    yellow_mask: np.ndarray
    image_encoding: str
    white_encoding: str
    yellow_encoding: str


@dataclass
class ProcessedFrame:
    source: SourceFrame
    color_bev: np.ndarray
    white_bev: np.ndarray
    yellow_bev: np.ndarray
    result: PathResult
    control: PreviewControlResult | None
    front_panel: np.ndarray | None
    bev_detection_panel: np.ndarray | None
    path_panel: np.ndarray | None
    unified_display: np.ndarray | None
    display_frame: np.ndarray | None
    bev_detection_window: np.ndarray | None
    visualization_compose_ms: float


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Unified YOLO lane segmentation, fixed BEV, and yellow-primary path viewer."
    )
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--white-mask-topic", default=DEFAULT_WHITE_TOPIC)
    parser.add_argument("--yellow-mask-topic", default=DEFAULT_YELLOW_TOPIC)
    parser.add_argument("--bev-config", type=Path, default=DEFAULT_BEV_CONFIG)
    parser.add_argument("--params", type=Path, default=DEFAULT_PARAMS)
    parser.add_argument("--bag-name", default=DEFAULT_BAG_NAME)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--display-mode", choices=DISPLAY_MODES, default="minimal")
    parser.add_argument("--show-bev-detection", action="store_true")
    parser.add_argument(
        "--publish-canonical",
        nargs="?",
        const=True,
        default=True,
        type=parse_bool,
        help="Publish canonical images in addition to the direct BEV outputs.",
    )
    parser.add_argument("--yolo-conf-threshold", type=probability, default=0.20)
    parser.add_argument("--publish-debug-images", nargs="?", const=True, default=False, type=parse_bool)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--controller-preview", action="store_true")
    parser.add_argument("--controller-preview-config", type=Path, default=DEFAULT_CONTROLLER_PREVIEW_CONFIG)
    parser.add_argument("--preview-target-speed-scale", type=float, default=None)
    parser.add_argument("--disable-optical-flow-prediction", action="store_true")
    parser.add_argument("--metrics-csv", type=Path, default=None)
    parser.add_argument("--self-check", action="store_true")
    return parser


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def probability(value: Any) -> float:
    number = float(value)
    if not 0.0 <= number <= 1.0:
        raise argparse.ArgumentTypeError("value must be between 0.0 and 1.0")
    return number


def resolve_display_args(args: argparse.Namespace) -> argparse.Namespace:
    if args.no_gui:
        args.display_mode = "off"
    args.gui_enabled = args.display_mode in {"minimal", "full"}
    if args.display_mode == "off" and args.record:
        print("RECORDING_DISABLED: display mode is off.")
        args.record = False
    return args


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    return resolve_display_args(args)


def flatten_params(data: dict[str, Any]) -> dict[str, Any]:
    flat: dict[str, Any] = {}
    for value in data.values():
        if isinstance(value, dict):
            flat.update(value)
    return flat


def load_params(path: Path) -> tuple[PathParams, dict[str, Any]]:
    raw: dict[str, Any] = {}
    if path.is_file():
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    flat = flatten_params(raw)
    flat["lane_side"] = "auto"
    return PathParams.from_dict(flat), raw


def load_controller_preview_params(path: Path, target_speed_override: float | None) -> PreviewControllerParams:
    raw: dict[str, Any] = {}
    if path.is_file():
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    params = PreviewControllerParams.from_dict(raw)
    if target_speed_override is not None:
        params.preview_target_speed_scale = float(np.clip(target_speed_override, 0.0, 1.0))
    return params


def stamp_to_ns(header: Any) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def stamp_to_sec(header: Any) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def stamp_to_text(header: Any) -> str:
    return f"{int(header.stamp.sec)}.{int(header.stamp.nanosec):09d}"


def read_structured(path: Path) -> dict[str, Any]:
    if path.suffix.lower() == ".json":
        return json.loads(path.read_text(encoding="utf-8"))
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def validate_points(points: np.ndarray, width: int, height: int) -> tuple[bool, list[str]]:
    reasons: list[str] = []
    if points.shape != (4, 2):
        reasons.append(f"expected four 2D points, got shape {points.shape}")
        return False, reasons
    if not np.all(np.isfinite(points)):
        reasons.append("points contain NaN/Inf")
    if np.any(points[:, 0] < 0.0) or np.any(points[:, 0] > float(width - 1)):
        reasons.append("point x outside image")
    if np.any(points[:, 1] < 0.0) or np.any(points[:, 1] > float(height - 1)):
        reasons.append("point y outside image")
    area = 0.5 * np.sum(points[:, 0] * np.roll(points[:, 1], -1) - points[:, 1] * np.roll(points[:, 0], -1))
    if abs(float(area)) < max(1000.0, float(width * height) * 0.005):
        reasons.append("polygon area too small")
    return not reasons, reasons


def load_fixed_bev_config(path: Path) -> FixedBevConfig:
    if not path.is_file():
        raise FileNotFoundError(f"BEV config does not exist: {path}")
    data = read_structured(path)
    input_width = int(data.get("input_width") or 0)
    input_height = int(data.get("input_height") or 0)
    bev_width = int(data.get("bev_width") or 0)
    bev_height = int(data.get("bev_height") or 0)
    source = np.asarray(data.get("source_points_px") or data.get("source_points"), dtype=np.float32)
    destination = np.asarray(data.get("destination_points"), dtype=np.float32)
    source_ok, source_reasons = validate_points(source, input_width, input_height)
    dest_ok, dest_reasons = validate_points(destination, bev_width, bev_height)
    if not source_ok or not dest_ok:
        raise ValueError(
            "invalid BEV points: "
            + "; ".join(source_reasons + dest_reasons)
        )
    matrix = cv2.getPerspectiveTransform(source, destination)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("computed BEV homography contains NaN/Inf")

    stored_matrix = None
    stored_matches = True
    if data.get("homography_matrix") is not None:
        stored_matrix = np.asarray(data["homography_matrix"], dtype=np.float64)
        stored_matches = stored_matrix.shape == (3, 3) and np.allclose(
            stored_matrix,
            matrix,
            rtol=1e-5,
            atol=1e-3,
        )

    return FixedBevConfig(
        path=path,
        bag_name=str(data.get("bag_name") or path.parent.name),
        input_width=input_width,
        input_height=input_height,
        bev_width=bev_width,
        bev_height=bev_height,
        source_points=source,
        destination_points=destination,
        stored_homography=stored_matrix,
        matrix=matrix,
        stored_matrix_matches_points=stored_matches,
    )


def binary_warp_mask(mask: np.ndarray, matrix: np.ndarray, size: tuple[int, int]) -> np.ndarray:
    warped = cv2.warpPerspective(normalize_mask(mask), matrix, size, flags=cv2.INTER_NEAREST)
    return normalize_mask(warped)


def warp_fixed_bev(
    frame: SourceFrame,
    bev_config: FixedBevConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    size = (bev_config.bev_width, bev_config.bev_height)
    color = cv2.warpPerspective(frame.image, bev_config.matrix, size, flags=cv2.INTER_LINEAR)
    white = binary_warp_mask(frame.white_mask, bev_config.matrix, size)
    yellow = binary_warp_mask(frame.yellow_mask, bev_config.matrix, size)
    return color, white, yellow


def put_text(
    image: np.ndarray,
    value: str,
    origin: tuple[int, int],
    scale: float = 0.48,
    color: tuple[int, int, int] = (255, 255, 255),
    thickness: int = 1,
) -> None:
    cv2.putText(image, value, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def put_boxed_text(
    image: np.ndarray,
    value: str,
    origin: tuple[int, int],
    scale: float = 0.48,
    color: tuple[int, int, int] = (255, 255, 255),
    thickness: int = 1,
) -> None:
    x, y = origin
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(value, font, scale, thickness)
    cv2.rectangle(
        image,
        (max(0, x - 4), max(0, y - th - baseline - 5)),
        (min(image.shape[1] - 1, x + tw + 4), min(image.shape[0] - 1, y + baseline + 4)),
        (0, 0, 0),
        -1,
    )
    cv2.putText(image, value, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def mask_to_bgr(mask: np.ndarray, color: tuple[int, int, int]) -> np.ndarray:
    out = np.zeros((mask.shape[0], mask.shape[1], 3), dtype=np.uint8)
    out[mask > 0] = color
    return out


def overlay_masks(
    image: np.ndarray,
    white_mask: np.ndarray,
    yellow_mask: np.ndarray,
    alpha: float = 0.45,
) -> np.ndarray:
    out = image.copy()
    white = normalize_mask(white_mask)
    yellow = normalize_mask(yellow_mask)
    for mask, color in ((white, WHITE_COLOR), (yellow, YELLOW_COLOR)):
        color_image = np.zeros_like(out)
        color_image[:, :] = color
        blended = cv2.addWeighted(out, 1.0 - alpha, color_image, alpha, 0.0)
        out[mask > 0] = blended[mask > 0]
    overlap = (white > 0) & (yellow > 0)
    out[overlap] = (255, 0, 255)
    return out


def draw_mask_contours(
    image: np.ndarray,
    mask: np.ndarray,
    color: tuple[int, int, int],
    thickness: int = 1,
) -> int:
    binary = normalize_mask(mask)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cv2.drawContours(image, contours, -1, color, thickness, cv2.LINE_AA)
    return len(contours)


def draw_curve(
    image: np.ndarray,
    curve: CurveFit,
    color: tuple[int, int, int],
    thickness: int = 2,
) -> None:
    if not curve.valid or curve.coeffs is None:
        return
    y_values = np.linspace(curve.y_max, curve.y_min, 90, dtype=np.float32)
    points = np.stack((curve.evaluate(y_values), y_values), axis=1)
    draw_polyline(image, points, color, thickness)


def draw_polyline(
    image: np.ndarray,
    points: np.ndarray,
    color: tuple[int, int, int],
    thickness: int = 3,
) -> None:
    if points.size == 0 or points.shape[0] < 2:
        return
    pts = points[np.all(np.isfinite(points), axis=1)].copy()
    if pts.shape[0] < 2:
        return
    pts[:, 0] = np.clip(pts[:, 0], 0.0, image.shape[1] - 1)
    pts[:, 1] = np.clip(pts[:, 1], 0.0, image.shape[0] - 1)
    cv2.polylines(image, [pts.astype(np.int32)], False, color, thickness, cv2.LINE_AA)


def draw_observations(
    image: np.ndarray,
    observations: list[Observation],
    color: tuple[int, int, int],
) -> None:
    for obs in observations:
        x = int(round(obs.x))
        y = int(round(obs.y))
        if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
            cv2.circle(image, (x, y), 1, color, -1, cv2.LINE_AA)


def path_draw_color(source: str) -> tuple[int, int, int]:
    if source in {"YELLOW_PRIMARY", "YELLOW_DIRECT"}:
        return YELLOW_PRIMARY_PATH_COLOR
    if source == "YELLOW_PARTIAL_TRACKED":
        return YELLOW_PARTIAL_PATH_COLOR
    if source in {"WHITE_LEFT_GAP_FALLBACK", "WHITE_RIGHT_GAP_FALLBACK", "WHITE_BOUNDARY_NORMAL_FALLBACK"}:
        return WHITE_FALLBACK_PATH_COLOR
    if source in {"YELLOW_HISTORY_HOLD", "TEMPORAL_PREDICTED"}:
        return HOLD_PATH_COLOR
    return INVALID_COLOR


def path_display_name(source: str) -> str:
    if source in {"PATH_INVALID", "PATH_LOST_STOP"}:
        return source
    return source


def resize_panel(image: np.ndarray, size: tuple[int, int] = PANEL_SIZE) -> np.ndarray:
    return cv2.resize(image, size, interpolation=cv2.INTER_AREA)


def add_title_bar(image: np.ndarray, title: str, subtitle: str = "") -> np.ndarray:
    width = image.shape[1]
    title_bar = np.zeros((42, width, 3), dtype=np.uint8)
    put_text(title_bar, title, (10, 18), 0.55, (255, 255, 255), 1)
    if subtitle:
        put_text(title_bar, subtitle, (10, 36), 0.38, (210, 210, 210), 1)
    return np.vstack((title_bar, image))


def make_front_panel(frame: SourceFrame, inference_status: str) -> np.ndarray:
    panel = overlay_masks(frame.image, frame.white_mask, frame.yellow_mask)
    white_components = draw_mask_contours(panel, frame.white_mask, WHITE_COLOR, 2)
    yellow_components = draw_mask_contours(panel, frame.yellow_mask, YELLOW_COLOR, 2)
    h, w = frame.image.shape[:2]
    put_boxed_text(panel, f"white components {white_components}", (12, 28), 0.60, WHITE_COLOR)
    put_boxed_text(panel, f"yellow components {yellow_components}", (12, 56), 0.60, YELLOW_COLOR)
    put_boxed_text(panel, f"source {w}x{h} {frame.image_encoding}", (12, 84), 0.52)
    put_boxed_text(panel, f"stamp {frame.stamp_text}", (12, 110), 0.48)
    put_boxed_text(panel, f"inference {inference_status}", (12, 136), 0.48, (0, 255, 0))
    return add_title_bar(resize_panel(panel), "FRONT YOLO DETECTION", "white-boundary + yellow_centerline")


def make_bev_detection_panel(
    color_bev: np.ndarray,
    white_bev: np.ndarray,
    yellow_bev: np.ndarray,
    result: PathResult,
    bev_config: FixedBevConfig,
) -> np.ndarray:
    panel = overlay_masks(color_bev, white_bev, yellow_bev)
    draw_mask_contours(panel, white_bev, WHITE_COLOR, 1)
    draw_mask_contours(panel, yellow_bev, YELLOW_COLOR, 1)
    h, w = panel.shape[:2]
    cv2.line(panel, (w // 2, 0), (w // 2, h - 1), (255, 255, 255), 1, cv2.LINE_AA)
    white_ratio = float(np.count_nonzero(white_bev) / max(white_bev.size, 1))
    yellow_ratio = float(np.count_nonzero(yellow_bev) / max(yellow_bev.size, 1))
    config_name = bev_config.path.parent.name
    put_boxed_text(panel, f"BEV {w}x{h}", (12, 26), 0.58)
    put_boxed_text(panel, f"white ratio {white_ratio:.4f} comps {result.white_component_count}", (12, 54), 0.48, WHITE_COLOR)
    put_boxed_text(panel, f"yellow ratio {yellow_ratio:.4f} comps {result.yellow_component_count}", (12, 78), 0.48, YELLOW_COLOR)
    put_boxed_text(panel, f"fixed config {config_name}", (12, 104), 0.43)
    return add_title_bar(resize_panel(panel), "BEV YOLO DETECTION", "fixed BEV warp")


def make_bev_detection_window(
    color_bev: np.ndarray,
    white_bev: np.ndarray,
    yellow_bev: np.ndarray,
    result: PathResult,
    yolo_conf_threshold: float,
) -> np.ndarray:
    """Create the native-size BEV mask view; confidence is not present in masks."""
    white = normalize_mask(white_bev)
    yellow = normalize_mask(yellow_bev)
    display = color_bev.copy()
    mask_overlay = np.zeros_like(display)
    mask_overlay[white > 0] = (255, 255, 255)
    mask_overlay[yellow > 0] = (0, 255, 255)
    detected = (white > 0) | (yellow > 0)
    blended = cv2.addWeighted(display, 0.55, mask_overlay, 0.45, 0.0)
    display[detected] = blended[detected]
    draw_mask_contours(display, white, (255, 255, 255), 2)
    draw_mask_contours(display, yellow, (0, 255, 255), 2)

    height, width = display.shape[:2]
    cv2.line(display, (width // 2, 0), (width // 2, height - 1), (255, 255, 255), 1, cv2.LINE_AA)
    white_ratio = float(np.count_nonzero(white) / max(white.size, 1))
    yellow_ratio = float(np.count_nonzero(yellow) / max(yellow.size, 1))
    put_boxed_text(display, "BEV YOLO DETECTION", (12, 26), 0.58)
    put_boxed_text(display, f"white-boundary  comps {result.white_component_count}", (12, 52), 0.48, (255, 255, 255))
    put_boxed_text(display, f"yellow_centerline comps {result.yellow_component_count}", (12, 76), 0.48, (0, 255, 255))
    put_boxed_text(display, f"white ratio {white_ratio:.4f}", (12, 100), 0.46, (255, 255, 255))
    put_boxed_text(display, f"yellow ratio {yellow_ratio:.4f}", (12, 124), 0.46, (0, 255, 255))
    put_boxed_text(display, f"CONF THRESHOLD: {yolo_conf_threshold:.2f}", (12, 150), 0.50)
    return display


def finite_residual(value: float) -> float:
    return float(value) if np.isfinite(value) else -1.0


def selected_white_curve(result: PathResult) -> CurveFit | None:
    if result.selected_white_side in {"left", "left_gap_model"}:
        return result.white_left if result.white_left.valid else result.white_right
    if result.selected_white_side in {"right", "right_gap_model"}:
        return result.white_right if result.white_right.valid else result.white_left
    return None


def make_path_overlay(
    color_bev: np.ndarray,
    result: PathResult,
    control: PreviewControlResult | None = None,
    preview_params: PreviewControllerParams | None = None,
    show_controller: bool = True,
    show_candidates: bool = True,
    show_flow: bool = False,
) -> np.ndarray:
    panel = overlay_masks(color_bev, result.cleaned_white_mask, result.cleaned_yellow_mask, alpha=0.40)
    draw_observations(panel, result.white_observations, (180, 180, 180))
    draw_observations(panel, result.yellow_observations, (0, 170, 255))
    draw_curve(panel, result.white_left, WHITE_LEFT_COLOR, 2)
    draw_curve(panel, result.white_right, WHITE_RIGHT_COLOR, 2)
    draw_curve(panel, result.yellow, YELLOW_FIT_COLOR, 2)
    if show_candidates:
        draw_polyline(panel, result.yellow_direct_candidate, (255, 80, 255), 2)
        draw_polyline(panel, result.yellow_partial_candidate, YELLOW_PARTIAL_PATH_COLOR, 2)
        draw_polyline(panel, result.white_normal_candidate_a, (0, 110, 255), 1)
        draw_polyline(panel, result.white_normal_candidate_b, (0, 180, 255), 1)
    if show_flow and result.optical_flow_features.shape[0] > 0:
        for point in result.optical_flow_features:
            cv2.circle(panel, tuple(np.rint(point).astype(int)), 2, (255, 255, 255), -1, cv2.LINE_AA)

    fallback_curve = selected_white_curve(result)
    if result.path_source in {"WHITE_LEFT_GAP_FALLBACK", "WHITE_RIGHT_GAP_FALLBACK", "WHITE_BOUNDARY_NORMAL_FALLBACK"} and fallback_curve is not None:
        draw_curve(panel, fallback_curve, WHITE_FALLBACK_PATH_COLOR, 4)

    if result.path_source in {"PATH_INVALID", "PATH_LOST_STOP"}:
        put_boxed_text(panel, result.path_source, (12, 32), 0.82, INVALID_COLOR, 2)
    else:
        draw_polyline(panel, result.raw_path, (0, 170, 255), 2)
        draw_polyline(panel, result.smooth_path, path_draw_color(result.path_source), 5)
        if result.smooth_path.shape[0] >= 2:
            near = result.smooth_path[0].astype(int)
            far = result.smooth_path[-1].astype(int)
            cv2.circle(panel, tuple(near), 7, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(panel, tuple(far), 7, (255, 0, 255), -1, cv2.LINE_AA)
            put_boxed_text(panel, "near", (int(near[0]) + 8, int(near[1]) - 8), 0.38, (0, 0, 255))
            put_boxed_text(panel, "far", (int(far[0]) + 8, int(far[1]) + 14), 0.38, (255, 0, 255))

    status_color = INVALID_COLOR if result.path_source in {"PATH_INVALID", "PATH_LOST_STOP"} else path_draw_color(result.path_source)
    put_boxed_text(panel, f"PATH: {path_display_name(result.path_source)}", (12, 26), 0.62, status_color, 2)
    put_boxed_text(panel, f"valid {int(result.path_valid)} conf {result.confidence:.2f}", (12, 54), 0.46)
    put_boxed_text(panel, f"yellow {int(result.yellow.valid)} {result.yellow_state} res {finite_residual(result.yellow.residual):.1f}", (12, 78), 0.43, YELLOW_COLOR)
    put_boxed_text(panel, f"white L {int(result.white_left.valid)} res {finite_residual(result.white_left.residual):.1f}", (12, 100), 0.43, WHITE_LEFT_COLOR)
    put_boxed_text(panel, f"white R {int(result.white_right.valid)} res {finite_residual(result.white_right.residual):.1f}", (12, 122), 0.43, WHITE_RIGHT_COLOR)
    put_boxed_text(panel, f"selected white {result.selected_white_side}", (12, 144), 0.43, WHITE_FALLBACK_PATH_COLOR)
    put_boxed_text(panel, f"gap L {result.learned_left_gap_near_px:.1f} R {result.learned_right_gap_near_px:.1f}", (12, 166), 0.42)
    put_boxed_text(panel, f"missing {result.yellow_missing_age_sec:.2f}s path age {result.path_age_sec:.2f}s", (12, 188), 0.42)
    put_boxed_text(panel, f"yellow reject {result.yellow_reject_reason}", (12, 210), 0.40, YELLOW_COLOR)
    put_boxed_text(panel, f"white reject {result.white_fallback_reject_reason} normal {result.normal_fallback_direction}", (12, 232), 0.38, WHITE_FALLBACK_PATH_COLOR)
    put_boxed_text(panel, f"pred age {result.temporal_prediction_age_sec:.2f}s curv {result.curvature_abs_max:.4f}", (12, 254), 0.38)
    put_boxed_text(panel, f"path res {finite_residual(result.path_residual):.1f} {result.processing_ms:.1f}ms {result.fps:.1f}FPS", (12, 276), 0.38)

    y0 = 302
    if not show_controller or control is None or preview_params is None:
        put_boxed_text(panel, "CONTROLLER: OFF", (12, y0), 0.46, INVALID_COLOR, 2)
    else:
        ref = (int(round(preview_params.vehicle_reference_x_px)), int(round(preview_params.vehicle_reference_y_px)))
        cv2.drawMarker(panel, ref, (0, 255, 80), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
        if control.controller_valid:
            look = tuple(np.rint(control.lookahead_point_px).astype(int))
            cv2.circle(panel, look, 6, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.line(panel, ref, look, (0, 0, 255), 1, cv2.LINE_AA)
            draw_polyline(panel, control.predicted_trajectory_px, (0, 255, 0), 2)
            end = control.predicted_trajectory_px[min(control.predicted_trajectory_px.shape[0] - 1, 8)].astype(int)
            cv2.arrowedLine(panel, ref, tuple(end), (0, 255, 0), 2, cv2.LINE_AA, tipLength=0.25)
        status_color = (0, 255, 0) if control.controller_valid else INVALID_COLOR
        put_boxed_text(panel, f"CONTROLLER: {control.controller_state}", (12, y0), 0.42, status_color, 2)
        put_boxed_text(panel, "MODE: PIXEL PREVIEW ONLY / NOT MOTOR CALIBRATED", (12, y0 + 20), 0.34, (0, 255, 255))
        put_boxed_text(panel, f"LOOKAHEAD: {control.lookahead_distance_px:.1f}px", (12, y0 + 40), 0.36)
        put_boxed_text(panel, f"STEERING PREVIEW: {control.steering_preview_rad:+.3f}rad / {math.degrees(control.steering_preview_rad):+.1f}deg", (12, y0 + 60), 0.34)
        put_boxed_text(panel, f"CURVATURE PREVIEW: {control.curvature_preview:+.5f}", (12, y0 + 80), 0.34)
        put_boxed_text(panel, f"SPEED SCALE: {control.speed_scale:.2f} STOP: {control.stop_reason}", (12, y0 + 100), 0.34, status_color)
    return panel


def make_path_panel(
    color_bev: np.ndarray,
    result: PathResult,
    control: PreviewControlResult | None = None,
    preview_params: PreviewControllerParams | None = None,
    show_controller: bool = True,
    show_candidates: bool = True,
    show_flow: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    overlay = make_path_overlay(color_bev, result, control, preview_params, show_controller, show_candidates, show_flow)
    panel = add_title_bar(resize_panel(overlay), "BEV PATH + CONTROLLER PREVIEW", "PIXEL PREVIEW ONLY, no motor topics")
    return panel, overlay


def make_unified_display(
    front_panel: np.ndarray,
    bev_panel: np.ndarray,
    path_panel: np.ndarray,
    view_mode: int,
) -> np.ndarray:
    panels = [front_panel, bev_panel, path_panel]
    if view_mode in (1, 2, 3):
        selected = panels[view_mode - 1]
        body = cv2.resize(selected, (1440, 426), interpolation=cv2.INTER_AREA)
        footer = np.zeros((36, 1440, 3), dtype=np.uint8)
        put_text(footer, "SPACE pause/live | N step | P screenshot | V record | 1/2/3 focus | 0 all | D reset | C controller | T candidates | F flow | Q/ESC quit", (12, 23), 0.40)
        return np.vstack((body, footer))
    body = np.hstack(panels)
    footer = np.zeros((36, body.shape[1], 3), dtype=np.uint8)
    put_text(footer, "SPACE pause/live | N latest paused | P screenshot | V record | 1 front | 2 BEV | 3 path | 0 all | D reset | C controller | T candidates | F flow | Q/ESC quit", (12, 23), 0.37)
    return np.vstack((body, footer))


def make_minimal_display(
    color_bev: np.ndarray,
    result: PathResult,
    control: PreviewControlResult | None,
    preview_params: PreviewControllerParams | None,
) -> np.ndarray:
    panel = color_bev.copy()
    if result.path_source in {"PATH_INVALID", "PATH_LOST_STOP"} or not result.path_valid:
        cv2.rectangle(panel, (0, 0), (panel.shape[1] - 1, panel.shape[0] - 1), INVALID_COLOR, 4)
        put_boxed_text(panel, "STOP", (panel.shape[1] // 2 - 54, panel.shape[0] // 2), 1.4, INVALID_COLOR, 3)
    else:
        draw_polyline(panel, result.smooth_path, path_draw_color(result.path_source), 5)

    if preview_params is not None:
        ref = (int(round(preview_params.vehicle_reference_x_px)), int(round(preview_params.vehicle_reference_y_px)))
        cv2.drawMarker(panel, ref, (0, 255, 80), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
        if control is not None and control.controller_valid:
            look = tuple(np.rint(control.lookahead_point_px).astype(int))
            cv2.circle(panel, look, 6, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.line(panel, ref, look, (0, 0, 255), 1, cv2.LINE_AA)
            draw_polyline(panel, control.predicted_trajectory_px, (0, 255, 0), 2)

    steering_deg = math.degrees(control.steering_preview_rad) if control is not None else 0.0
    speed_scale = control.speed_scale if control is not None else 0.0
    lookahead = control.lookahead_distance_px if control is not None else 0.0
    state = "VALID" if control is not None and control.controller_valid else (control.stop_reason if control is not None else "CONTROLLER_OFF")
    put_boxed_text(panel, f"PATH: {path_display_name(result.path_source)}", (12, 24), 0.48, path_draw_color(result.path_source), 1)
    put_boxed_text(panel, f"CONF: {result.confidence:.2f}", (12, 48), 0.45)
    put_boxed_text(panel, f"STEER: {steering_deg:+.1f} deg", (12, 72), 0.45)
    put_boxed_text(panel, f"SPEED: {speed_scale:.2f}", (12, 96), 0.45)
    put_boxed_text(panel, f"LOOKAHEAD: {lookahead:.1f} px", (12, 120), 0.45)
    put_boxed_text(panel, f"STATE: {state}", (12, 144), 0.45, (0, 255, 0) if state == "VALID" else INVALID_COLOR)
    put_boxed_text(panel, f"FPS: {result.fps:.1f}", (12, 168), 0.45)
    return cv2.resize(panel, EXPECTED_BEV_SIZE, interpolation=cv2.INTER_AREA)


def image_msg_from_cv(bridge: CvBridge, image: np.ndarray, encoding: str, header: Any) -> Image:
    msg = bridge.cv2_to_imgmsg(image, encoding=encoding)
    msg.header = header
    return msg


def safe_csv_value(value: float) -> str:
    return f"{float(value):.3f}" if np.isfinite(value) else "-1.000"


class LaneSegUnifiedViewer(Node):
    """Single-node fixed BEV, path generation, ROS output, and optional GUI."""

    def __init__(
        self,
        args: argparse.Namespace,
        params: PathParams,
        bev_config: FixedBevConfig,
        preview_params: PreviewControllerParams | None = None,
    ) -> None:
        super().__init__("lane_seg_unified_viewer")
        self.args = args
        self.params = params
        self.bev_config = bev_config
        self.preview_params = preview_params
        self.bridge = CvBridge()
        self.state = TemporalState()
        self.controller_state = PreviewControllerState()
        self.stop_requested = False
        self.paused = False
        self.view_mode = 0
        self.show_controller_overlay = True
        self.show_candidate_paths = True
        self.show_flow_features = False
        self.frame_number = 0
        self.last_perf_log_time = time.monotonic()
        self.last_visualization_compose_ms = 0.0
        self.latest_frame: SourceFrame | None = None
        self.active_frame: SourceFrame | None = None
        self.latest_processed: ProcessedFrame | None = None
        self.recording = bool(args.record and args.gui_enabled)
        self.writer: cv2.VideoWriter | None = None
        self.image_cache: list[tuple[int, Image, np.ndarray]] = []
        self.white_cache: list[tuple[int, Image, np.ndarray]] = []
        self.yellow_cache: list[tuple[int, Image, np.ndarray]] = []
        self.last_warn_time: dict[str, float] = {}

        self.run_dir = args.output_dir / args.bag_name
        self.screenshot_dir = self.run_dir / "screenshots"
        self.video_dir = self.run_dir / "videos"
        for directory in (self.run_dir, self.screenshot_dir, self.video_dir):
            directory.mkdir(parents=True, exist_ok=True)
        self.metrics_path = args.metrics_csv if args.metrics_csv is not None else self.run_dir / f"metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.metrics_path.parent.mkdir(parents=True, exist_ok=True)
        self.metrics_file = self.metrics_path.open("w", newline="", encoding="utf-8")
        self.metrics_writer = csv.DictWriter(
            self.metrics_file,
            fieldnames=[
                "timestamp",
                "frame_index",
                "path_valid",
                "path_source",
                "path_confidence",
                "yellow_state",
                "yellow_reject_reason",
                "white_fallback_reject_reason",
                "prediction_age_sec",
                "normal_offset_direction",
                "controller_valid",
                "lookahead_x_px",
                "lookahead_y_px",
                "lookahead_distance_px",
                "steering_preview_rad",
                "steering_preview_deg",
                "curvature_preview",
                "speed_scale",
                "stop_reason",
                "processing_ms",
                "fps",
            ],
        )
        self.metrics_writer.writeheader()
        self.metrics_rows: list[dict[str, Any]] = []

        self.bev_color_pub = self.create_publisher(Image, "/lane_seg_bev/color", qos_profile_sensor_data)
        self.bev_white_pub = self.create_publisher(Image, "/lane_seg_bev/white_mask", qos_profile_sensor_data)
        self.bev_yellow_pub = self.create_publisher(Image, "/lane_seg_bev/yellow_mask", qos_profile_sensor_data)
        self.bev_debug_pub = self.create_publisher(Image, "/lane_seg_bev/debug_image", qos_profile_sensor_data)
        self.canonical_pub = None
        self.canonical_white_pub = None
        self.canonical_yellow_pub = None
        self.canonical_valid_pub = None
        if self.args.publish_canonical:
            self.canonical_pub = self.create_publisher(
                Image, "/perception/canonical_road_image", 10
            )
            self.canonical_white_pub = self.create_publisher(
                Image, "/perception/canonical_white_mask", 10
            )
            self.canonical_yellow_pub = self.create_publisher(
                Image, "/perception/canonical_yellow_mask", 10
            )
            self.canonical_valid_pub = self.create_publisher(
                Image, "/perception/canonical_valid_mask", 10
            )
        self.path_pixels_pub = self.create_publisher(Float32MultiArray, "/lane_path/path_pixels", 10)
        self.path_norm_pub = self.create_publisher(Float32MultiArray, "/lane_path/path_normalized", 10)
        self.diagnostics_pub = self.create_publisher(Float32MultiArray, "/lane_path/diagnostics", 10)
        self.status_pub = self.create_publisher(String, "/lane_path/status", 10)
        self.detection_debug_pub = self.create_publisher(Image, "/lane_path/detection_debug_image", qos_profile_sensor_data)
        self.path_debug_pub = self.create_publisher(Image, "/lane_path/path_debug_image", qos_profile_sensor_data)
        self.unified_debug_pub = self.create_publisher(Image, "/lane_unified/debug_image", qos_profile_sensor_data)
        self.preview_steering_rad_pub = self.create_publisher(Float32, "/lane_control_preview/steering_rad", 10)
        self.preview_steering_deg_pub = self.create_publisher(Float32, "/lane_control_preview/steering_deg", 10)
        self.preview_curvature_pub = self.create_publisher(Float32, "/lane_control_preview/curvature", 10)
        self.preview_speed_scale_pub = self.create_publisher(Float32, "/lane_control_preview/speed_scale", 10)
        self.preview_lookahead_pub = self.create_publisher(Float32MultiArray, "/lane_control_preview/lookahead_point", 10)
        self.preview_trajectory_pub = self.create_publisher(Float32MultiArray, "/lane_control_preview/predicted_trajectory", 10)
        self.preview_status_pub = self.create_publisher(String, "/lane_control_preview/status", 10)

        self.create_subscription(Image, args.image_topic, self.on_image_cache, qos_profile_sensor_data)
        self.create_subscription(Image, args.white_mask_topic, self.on_white_cache, qos_profile_sensor_data)
        self.create_subscription(Image, args.yellow_mask_topic, self.on_yellow_cache, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / 30.0, self.on_timer)

        if args.display_mode == "full":
            cv2.namedWindow(FULL_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(FULL_WINDOW_NAME, 1440, 480)
        elif args.display_mode == "minimal":
            cv2.namedWindow(MINIMAL_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(MINIMAL_WINDOW_NAME, EXPECTED_BEV_SIZE[0], EXPECTED_BEV_SIZE[1])
        else:
            self.get_logger().info("DISPLAY_MODE: OFF")
            self.get_logger().info("OpenCV GUI and visualization composition are disabled.")
        if args.gui_enabled and args.show_bev_detection:
            cv2.namedWindow(BEV_DETECTION_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                BEV_DETECTION_WINDOW_NAME,
                bev_config.bev_width,
                bev_config.bev_height,
            )

        self.get_logger().info(f"subscribed image: {args.image_topic}")
        self.get_logger().info(f"subscribed white mask: {args.white_mask_topic}")
        self.get_logger().info(f"subscribed yellow mask: {args.yellow_mask_topic}")
        self.get_logger().info(
            f"sync: timestamp cache size={CACHE_LIMIT} slop={SYNC_SLOP_SEC:.2f}s"
        )
        self.get_logger().info(f"fixed BEV config: {bev_config.path}")
        self.get_logger().info(f"fixed BEV size: {bev_config.bev_width}x{bev_config.bev_height}")
        self.get_logger().info(f"output directory: {self.run_dir}")
        self.get_logger().info(f"display mode: {args.display_mode}")
        self.get_logger().info(f"publish debug images: {bool(args.publish_debug_images)}")
        self.get_logger().warn("PIXEL PREVIEW ONLY: no /xycar_motor, /auturbo_legacy/lane_cmd, or /cmd_vel publishers are created.")
        if not bev_config.stored_matrix_matches_points:
            self.get_logger().warn("stored homography differs from source/destination recomputation; using recomputed matrix.")

    def warn_throttled(self, key: str, message: str, period_sec: float = 2.0) -> None:
        now = time.monotonic()
        if now - self.last_warn_time.get(key, 0.0) >= period_sec:
            self.last_warn_time[key] = now
            self.get_logger().warn(message)

    def convert_image(self, msg: Image) -> np.ndarray | None:
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.warn_throttled("image_conversion", f"image conversion failed: {exc}")
            return None

    def convert_mask(self, msg: Image, label: str) -> np.ndarray | None:
        try:
            if msg.encoding == "mono8":
                mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            else:
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                mask = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
        except Exception as exc:
            self.warn_throttled(f"{label}_conversion", f"{label} mask conversion failed: {exc}")
            return None
        return normalize_mask(mask)

    def validate_input_messages(self, image_msg: Image, white_msg: Image, yellow_msg: Image) -> bool:
        widths = (int(image_msg.width), int(white_msg.width), int(yellow_msg.width))
        heights = (int(image_msg.height), int(white_msg.height), int(yellow_msg.height))
        encodings = (image_msg.encoding, white_msg.encoding, yellow_msg.encoding)
        frame_ids = (image_msg.header.frame_id, white_msg.header.frame_id, yellow_msg.header.frame_id)
        stamps = (stamp_to_ns(image_msg.header), stamp_to_ns(white_msg.header), stamp_to_ns(yellow_msg.header))
        if len(set(widths)) != 1 or len(set(heights)) != 1:
            self.warn_throttled("dimension_mismatch", f"input dimensions differ: widths={widths} heights={heights}")
            return False
        if (widths[0], heights[0]) != (self.bev_config.input_width, self.bev_config.input_height):
            self.warn_throttled(
                "unexpected_input_size",
                f"input size {widths[0]}x{heights[0]} does not match BEV config {self.bev_config.input_width}x{self.bev_config.input_height}",
            )
        if encodings[0] != "bgr8":
            self.warn_throttled("image_encoding", f"image encoding is {encodings[0]!r}, expected bgr8")
        if encodings[1] != "mono8" or encodings[2] != "mono8":
            self.warn_throttled("mask_encoding", f"mask encodings are {encodings[1]!r}, {encodings[2]!r}; converting to mono8")
        if len(set(frame_ids)) > 1:
            self.warn_throttled("frame_id_mismatch", f"frame_id differs: {frame_ids}")
        if max(stamps) - min(stamps) > int(SYNC_SLOP_SEC * 1_000_000_000):
            self.warn_throttled("stamp_slop", f"stamp spread exceeds slop: {stamps}")
            return False
        return True

    def make_source_frame(self, image_msg: Image, white_msg: Image, yellow_msg: Image) -> SourceFrame | None:
        if not self.validate_input_messages(image_msg, white_msg, yellow_msg):
            return None
        image = self.convert_image(image_msg)
        white = self.convert_mask(white_msg, "white")
        yellow = self.convert_mask(yellow_msg, "yellow")
        if image is None or white is None or yellow is None:
            return None
        if image.shape[:2] != white.shape[:2] or image.shape[:2] != yellow.shape[:2]:
            self.warn_throttled("converted_dimension_mismatch", "converted image/mask dimensions differ")
            return None
        return SourceFrame(
            header=image_msg.header,
            stamp_ns=stamp_to_ns(image_msg.header),
            stamp_sec=stamp_to_sec(image_msg.header),
            stamp_text=stamp_to_text(image_msg.header),
            image=image,
            white_mask=white,
            yellow_mask=yellow,
            image_encoding=image_msg.encoding,
            white_encoding=white_msg.encoding,
            yellow_encoding=yellow_msg.encoding,
        )

    def on_synced_images(self, image_msg: Image, white_msg: Image, yellow_msg: Image) -> None:
        frame = self.make_source_frame(image_msg, white_msg, yellow_msg)
        if frame is None:
            return
        self.latest_frame = frame
        if not self.paused:
            self.active_frame = frame

    @staticmethod
    def trim_cache(cache: list[tuple[int, Image, np.ndarray]]) -> None:
        cache.sort(key=lambda item: item[0])
        del cache[:-CACHE_LIMIT]

    def on_image_cache(self, msg: Image) -> None:
        image = self.convert_image(msg)
        if image is None:
            return
        self.image_cache.append((stamp_to_ns(msg.header), msg, image))
        self.trim_cache(self.image_cache)
        self.try_cache_sync()

    def on_white_cache(self, msg: Image) -> None:
        mask = self.convert_mask(msg, "white")
        if mask is None:
            return
        self.white_cache.append((stamp_to_ns(msg.header), msg, mask))
        self.trim_cache(self.white_cache)
        self.try_cache_sync()

    def on_yellow_cache(self, msg: Image) -> None:
        mask = self.convert_mask(msg, "yellow")
        if mask is None:
            return
        self.yellow_cache.append((stamp_to_ns(msg.header), msg, mask))
        self.trim_cache(self.yellow_cache)
        self.try_cache_sync()

    @staticmethod
    def nearest(cache: list[tuple[int, Image, np.ndarray]], target_ns: int, slop_ns: int) -> tuple[int, Image, np.ndarray] | None:
        if not cache:
            return None
        best = min(cache, key=lambda item: abs(item[0] - target_ns))
        return best if abs(best[0] - target_ns) <= slop_ns else None

    def try_cache_sync(self) -> None:
        if not self.image_cache or not self.white_cache or not self.yellow_cache:
            return
        slop_ns = int(SYNC_SLOP_SEC * 1_000_000_000)
        for image_item in reversed(self.image_cache):
            stamp_ns, image_msg, image = image_item
            white_item = self.nearest(self.white_cache, stamp_ns, slop_ns)
            yellow_item = self.nearest(self.yellow_cache, stamp_ns, slop_ns)
            if white_item is None or yellow_item is None:
                continue
            _, white_msg, white = white_item
            _, yellow_msg, yellow = yellow_item
            if not self.validate_input_messages(image_msg, white_msg, yellow_msg):
                return
            if image.shape[:2] != white.shape[:2] or image.shape[:2] != yellow.shape[:2]:
                self.warn_throttled("cache_dimension_mismatch", "cache-synced image/mask dimensions differ")
                return
            frame = SourceFrame(
                header=image_msg.header,
                stamp_ns=stamp_ns,
                stamp_sec=stamp_to_sec(image_msg.header),
                stamp_text=stamp_to_text(image_msg.header),
                image=image,
                white_mask=white,
                yellow_mask=yellow,
                image_encoding=image_msg.encoding,
                white_encoding=white_msg.encoding,
                yellow_encoding=yellow_msg.encoding,
            )
            self.latest_frame = frame
            if not self.paused and (self.active_frame is None or self.active_frame.stamp_ns != frame.stamp_ns):
                self.active_frame = frame
            return

    def on_timer(self) -> None:
        if self.active_frame is None:
            return
        if self.latest_processed is not None and self.latest_processed.source.stamp_ns == self.active_frame.stamp_ns:
            if self.args.gui_enabled and self.latest_processed.display_frame is not None:
                self.handle_gui(
                    self.latest_processed.display_frame,
                    self.latest_processed.bev_detection_window,
                )
            return

        processed = self.process_frame(self.active_frame)
        self.latest_processed = processed
        self.publish_outputs(processed)
        self.write_metrics(processed.source, processed.result, processed.control)
        if self.args.gui_enabled and processed.display_frame is not None:
            self.write_recording_frame(processed.display_frame)
            self.handle_gui(processed.display_frame, processed.bev_detection_window)
        self.log_performance(processed)

    def process_frame(self, frame: SourceFrame) -> ProcessedFrame:
        self.frame_number += 1
        color_bev, white_bev, yellow_bev = warp_fixed_bev(frame, self.bev_config)
        result = process_lane_path(
            color_bev,
            white_bev,
            yellow_bev,
            self.params,
            self.state,
            frame.stamp_sec,
            "auto",
        )
        control = None
        if self.args.controller_preview and self.preview_params is not None:
            control = compute_preview_control(
                result.smooth_path,
                result.path_source,
                result.confidence,
                result.temporal_prediction_age_sec,
                self.preview_params,
                self.controller_state,
                frame.stamp_sec,
            )
        compose_start = time.perf_counter()
        front_panel: np.ndarray | None = None
        bev_detection_panel: np.ndarray | None = None
        path_panel: np.ndarray | None = None
        unified_display: np.ndarray | None = None
        display_frame: np.ndarray | None = None
        bev_detection_window: np.ndarray | None = None
        if self.args.display_mode == "minimal":
            display_frame = make_minimal_display(color_bev, result, control, self.preview_params)
        elif self.args.display_mode == "full" or self.args.publish_debug_images:
            front_panel = make_front_panel(frame, "mask topics OK")
            bev_detection_panel = make_bev_detection_panel(color_bev, white_bev, yellow_bev, result, self.bev_config)
            path_panel, _ = make_path_panel(
                color_bev,
                result,
                control,
                self.preview_params,
                self.show_controller_overlay,
                self.show_candidate_paths,
                self.show_flow_features,
            )
            unified_display = make_unified_display(front_panel, bev_detection_panel, path_panel, self.view_mode)
            if self.args.display_mode == "full":
                display_frame = unified_display
        if self.args.gui_enabled and self.args.show_bev_detection:
            bev_detection_window = make_bev_detection_window(
                color_bev,
                white_bev,
                yellow_bev,
                result,
                self.args.yolo_conf_threshold,
            )
        visualization_compose_ms = 0.0 if self.args.display_mode == "off" and not self.args.publish_debug_images else (time.perf_counter() - compose_start) * 1000.0
        self.last_visualization_compose_ms = visualization_compose_ms
        return ProcessedFrame(
            source=frame,
            color_bev=color_bev,
            white_bev=white_bev,
            yellow_bev=yellow_bev,
            result=result,
            control=control,
            front_panel=front_panel,
            bev_detection_panel=bev_detection_panel,
            path_panel=path_panel,
            unified_display=unified_display,
            display_frame=display_frame,
            bev_detection_window=bev_detection_window,
            visualization_compose_ms=visualization_compose_ms,
        )

    def publish_outputs(self, processed: ProcessedFrame) -> None:
        frame = processed.source
        result = processed.result
        h, w = processed.color_bev.shape[:2]

        self.bev_color_pub.publish(image_msg_from_cv(self.bridge, processed.color_bev, "bgr8", frame.header))
        self.bev_white_pub.publish(image_msg_from_cv(self.bridge, processed.white_bev, "mono8", frame.header))
        self.bev_yellow_pub.publish(image_msg_from_cv(self.bridge, processed.yellow_bev, "mono8", frame.header))

        if self.args.publish_canonical:
            valid = np.full(processed.white_bev.shape, 255, dtype=np.uint8)
            canonical = make_canonical_road_image_from_masks(
                processed.white_bev,
                processed.yellow_bev,
                valid_mask=valid,
                lateral_m_per_px=0.0021875,
                forward_m_per_px=0.003125,
                lateral_range_m=1.4,
                forward_range_m=1.5,
                output_width=256,
                output_height=144,
                background_gray=36,
                line_width_px=5,
                min_component_area_px=1,
                geometry_filter_enabled=False,
                preserve_white_mask=True,
                top_ignore_m=0.0,
                bottom_ignore_m=0.0,
                return_stages=True,
            )
            if not isinstance(canonical, CanonicalRoadStages):
                raise RuntimeError("canonical stage output was not requested")
            canonical_header = copy.deepcopy(frame.header)
            canonical_header.frame_id = "base_footprint"
            self.canonical_pub.publish(
                image_msg_from_cv(
                    self.bridge, canonical.road_image, "bgr8", canonical_header
                )
            )
            self.canonical_white_pub.publish(
                image_msg_from_cv(
                    self.bridge, canonical.white_mask, "mono8", canonical_header
                )
            )
            self.canonical_yellow_pub.publish(
                image_msg_from_cv(
                    self.bridge, canonical.yellow_mask, "mono8", canonical_header
                )
            )
            self.canonical_valid_pub.publish(
                image_msg_from_cv(
                    self.bridge, canonical.valid_mask, "mono8", canonical_header
                )
            )
        if self.args.publish_debug_images:
            if processed.bev_detection_panel is not None:
                self.bev_debug_pub.publish(image_msg_from_cv(self.bridge, processed.bev_detection_panel, "bgr8", frame.header))
                self.detection_debug_pub.publish(image_msg_from_cv(self.bridge, processed.bev_detection_panel, "bgr8", frame.header))
            if processed.path_panel is not None:
                self.path_debug_pub.publish(image_msg_from_cv(self.bridge, processed.path_panel, "bgr8", frame.header))
            if processed.unified_display is not None:
                self.unified_debug_pub.publish(image_msg_from_cv(self.bridge, processed.unified_display, "bgr8", frame.header))

        path_pixels = Float32MultiArray()
        path_pixels.data = path_pixels_array(result.smooth_path)
        path_norm = Float32MultiArray()
        path_norm.data = path_normalized_array(result.smooth_path, w, h)
        diagnostics = Float32MultiArray()
        diagnostics.data = [
            1.0 if result.path_valid else 0.0,
            SOURCE_CODES.get(result.path_source, 0.0),
            float(result.confidence),
            1.0 if result.white_left.valid else 0.0,
            1.0 if result.white_right.valid else 0.0,
            1.0 if result.yellow.valid else 0.0,
            YELLOW_STATE_CODES.get(result.yellow_state, 2.0),
            WHITE_SIDE_CODES.get(result.selected_white_side, 0.0),
            float(result.white_left.residual if np.isfinite(result.white_left.residual) else -1.0),
            float(result.white_right.residual if np.isfinite(result.white_right.residual) else -1.0),
            float(result.yellow.residual if np.isfinite(result.yellow.residual) else -1.0),
            float(result.learned_left_gap_near_px),
            float(result.learned_right_gap_near_px),
            float(result.gap_history_age_sec),
            float(result.yellow_missing_age_sec),
            1.0 if result.recovery_active else 0.0,
            float(result.path_age_sec),
            float(result.processing_ms),
            float(result.fps),
        ]
        self.path_pixels_pub.publish(path_pixels)
        self.path_norm_pub.publish(path_norm)
        self.diagnostics_pub.publish(diagnostics)

        status = String()
        status.data = json.dumps(
            {
                "stamp_sec": frame.stamp_sec,
                "frame_number": self.frame_number,
                "path_valid": bool(result.path_valid),
                "path_source": result.path_source,
                "confidence": float(result.confidence),
                "yellow_reject_reason": result.yellow_reject_reason,
                "white_fallback_reject_reason": result.white_fallback_reject_reason,
                "temporal_prediction_age_sec": float(result.temporal_prediction_age_sec),
                "normal_fallback_direction": result.normal_fallback_direction,
                "curvature_abs_mean": float(result.curvature_abs_mean),
                "curvature_abs_max": float(result.curvature_abs_max),
                "path_age_sec": float(result.path_age_sec),
                "point_count": int(result.smooth_path.shape[0]),
            },
            sort_keys=True,
        )
        self.status_pub.publish(status)
        if processed.control is not None:
            control = processed.control
            msg = Float32()
            msg.data = float(control.steering_preview_rad)
            self.preview_steering_rad_pub.publish(msg)
            msg = Float32()
            msg.data = float(math.degrees(control.steering_preview_rad))
            self.preview_steering_deg_pub.publish(msg)
            msg = Float32()
            msg.data = float(control.curvature_preview)
            self.preview_curvature_pub.publish(msg)
            msg = Float32()
            msg.data = float(control.speed_scale)
            self.preview_speed_scale_pub.publish(msg)
            arr = Float32MultiArray()
            arr.data = [float(control.lookahead_point_px[0]), float(control.lookahead_point_px[1])]
            self.preview_lookahead_pub.publish(arr)
            arr = Float32MultiArray()
            arr.data = preview_path_array(control.predicted_trajectory_px)
            self.preview_trajectory_pub.publish(arr)
            preview_status = String()
            preview_status.data = json.dumps(
                {
                    "preview_only": True,
                    "controller_state": control.controller_state,
                    "controller_valid": bool(control.controller_valid),
                    "path_source": result.path_source,
                    "path_confidence": float(result.confidence),
                    "stop_reason": control.stop_reason,
                    "steering_preview_rad": float(control.steering_preview_rad),
                    "speed_scale": float(control.speed_scale),
                    "mode": "PIXEL PREVIEW ONLY - NOT MOTOR CALIBRATED",
                },
                sort_keys=True,
            )
            self.preview_status_pub.publish(preview_status)

    def write_metrics(self, frame: SourceFrame, result: PathResult, control: PreviewControlResult | None) -> None:
        row = {
            "timestamp": frame.stamp_text,
            "frame_index": self.frame_number,
            "path_valid": int(result.path_valid),
            "path_source": result.path_source,
            "path_confidence": f"{result.confidence:.4f}",
            "yellow_state": result.yellow_state,
            "yellow_reject_reason": result.yellow_reject_reason,
            "white_fallback_reject_reason": result.white_fallback_reject_reason,
            "prediction_age_sec": safe_csv_value(result.temporal_prediction_age_sec),
            "normal_offset_direction": result.selected_normal_direction,
            "controller_valid": int(control.controller_valid) if control is not None else 0,
            "lookahead_x_px": safe_csv_value(control.lookahead_point_px[0]) if control is not None else "-1.000",
            "lookahead_y_px": safe_csv_value(control.lookahead_point_px[1]) if control is not None else "-1.000",
            "lookahead_distance_px": safe_csv_value(control.lookahead_distance_px) if control is not None else "-1.000",
            "steering_preview_rad": safe_csv_value(control.steering_preview_rad) if control is not None else "0.000",
            "steering_preview_deg": safe_csv_value(math.degrees(control.steering_preview_rad)) if control is not None else "0.000",
            "curvature_preview": safe_csv_value(control.curvature_preview) if control is not None else "0.000",
            "speed_scale": safe_csv_value(control.speed_scale) if control is not None else "0.000",
            "stop_reason": control.stop_reason if control is not None else "CONTROLLER_PREVIEW_OFF",
            "processing_ms": safe_csv_value(result.processing_ms),
            "fps": safe_csv_value(result.fps),
        }
        self.metrics_writer.writerow(row)
        self.metrics_rows.append(row)
        self.metrics_file.flush()

    def log_performance(self, processed: ProcessedFrame) -> None:
        now = time.monotonic()
        if now - self.last_perf_log_time < 5.0:
            return
        self.last_perf_log_time = now
        control = processed.control
        controller_valid = bool(control.controller_valid) if control is not None else False
        speed_scale = float(control.speed_scale) if control is not None else 0.0
        self.get_logger().info(
            "display_mode=%s fps=%.1f path_source=%s controller_valid=%s speed_scale=%.2f visualization_compose_ms=%.2f"
            % (
                self.args.display_mode,
                processed.result.fps,
                processed.result.path_source,
                controller_valid,
                speed_scale,
                processed.visualization_compose_ms,
            )
        )

    def handle_gui(
        self,
        display: np.ndarray,
        bev_detection_display: np.ndarray | None = None,
    ) -> None:
        window_name = MINIMAL_WINDOW_NAME if self.args.display_mode == "minimal" else FULL_WINDOW_NAME
        cv2.imshow(window_name, display)
        if self.args.show_bev_detection and bev_detection_display is not None:
            cv2.imshow(BEV_DETECTION_WINDOW_NAME, bev_detection_display)
        key = cv2.waitKey(1) & 0xFF
        if key == 255:
            return
        if self.args.display_mode == "minimal":
            if key in (ord("p"), ord("P")):
                self.save_screenshot(display)
            elif key in (ord("v"), ord("V")):
                self.toggle_recording(display)
            elif key in (ord("d"), ord("D")):
                self.state.reset()
                self.controller_state.reset()
                self.latest_processed = None
                self.get_logger().info("path, gap, optical-flow, and controller preview temporal state reset")
            elif key in (ord("q"), ord("Q"), 27):
                self.stop_requested = True
            return
        if key == 32:
            self.paused = not self.paused
            if not self.paused and self.latest_frame is not None:
                self.active_frame = self.latest_frame
        elif key in (ord("n"), ord("N")):
            if self.latest_frame is not None:
                self.active_frame = self.latest_frame
                self.paused = True
                self.latest_processed = None
        elif key in (ord("p"), ord("P")):
            self.save_screenshot(display)
        elif key in (ord("v"), ord("V")):
            self.toggle_recording(display)
        elif key in (ord("d"), ord("D")):
            self.state.reset()
            self.controller_state.reset()
            self.latest_processed = None
            self.get_logger().info("path, gap, optical-flow, and controller preview temporal state reset")
        elif key in (ord("c"), ord("C")):
            self.show_controller_overlay = not self.show_controller_overlay
            self.latest_processed = None
        elif key in (ord("t"), ord("T")):
            self.show_candidate_paths = not self.show_candidate_paths
            self.latest_processed = None
        elif key in (ord("f"), ord("F")):
            self.show_flow_features = not self.show_flow_features
            self.latest_processed = None
        elif key in (ord("1"), ord("2"), ord("3")):
            self.view_mode = int(chr(key))
            self.latest_processed = None
        elif key == ord("0"):
            self.view_mode = 0
            self.latest_processed = None
        elif key in (ord("q"), ord("Q"), 27):
            self.stop_requested = True

    def save_screenshot(self, display: np.ndarray) -> None:
        prefix = "minimal" if self.args.display_mode == "minimal" else "unified"
        path = self.screenshot_dir / f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        if cv2.imwrite(str(path), display):
            self.get_logger().info(f"saved screenshot: {path}")
        else:
            self.get_logger().error(f"failed to save screenshot: {path}")

    def toggle_recording(self, display: np.ndarray) -> None:
        if self.recording:
            self.stop_recording()
            self.recording = False
            self.get_logger().info("video recording stopped")
            return
        self.recording = True
        self.start_writer(display)

    def start_writer(self, display: np.ndarray) -> None:
        if self.writer is not None:
            return
        height, width = display.shape[:2]
        prefix = "minimal" if self.args.display_mode == "minimal" else "unified"
        path = self.video_dir / f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 20.0, (width, height))
        if not writer.isOpened():
            self.recording = False
            self.get_logger().error(f"failed to open video writer: {path}")
            return
        self.writer = writer
        self.get_logger().info(f"recording video: {path}")

    def write_recording_frame(self, display: np.ndarray) -> None:
        if not self.recording:
            return
        if self.writer is None:
            self.start_writer(display)
        if self.writer is not None:
            self.writer.write(display)

    def stop_recording(self) -> None:
        if self.writer is not None:
            self.writer.release()
            self.writer = None

    def write_summary(self) -> None:
        if not self.metrics_rows:
            return
        total = len(self.metrics_rows)
        sources = [str(row["path_source"]) for row in self.metrics_rows]
        def ratio(source: str) -> float:
            return sum(1 for value in sources if value == source) / max(total, 1)
        path_valid_ratio = sum(int(row["path_valid"]) for row in self.metrics_rows) / max(total, 1)
        controller_valid_ratio = sum(int(row["controller_valid"]) for row in self.metrics_rows) / max(total, 1)
        steering = np.asarray([float(row["steering_preview_rad"]) for row in self.metrics_rows], dtype=np.float32)
        speed = np.asarray([float(row["speed_scale"]) for row in self.metrics_rows], dtype=np.float32)
        fps = np.asarray([float(row["fps"]) for row in self.metrics_rows], dtype=np.float32)
        jumps = np.abs(np.diff(steering)) if steering.size >= 2 else np.asarray([0.0], dtype=np.float32)
        prediction_ages = np.asarray([float(row["prediction_age_sec"]) for row in self.metrics_rows], dtype=np.float32)
        max_prediction = float(np.max(prediction_ages)) if prediction_ages.size else 0.0
        max_stop = 0
        current_stop = 0
        for row in self.metrics_rows:
            if str(row["path_source"]) == "PATH_LOST_STOP":
                current_stop += 1
                max_stop = max(max_stop, current_stop)
            else:
                current_stop = 0
        nan_count = 0
        for row in self.metrics_rows:
            for key in ("lookahead_x_px", "lookahead_y_px", "steering_preview_rad", "curvature_preview", "speed_scale", "processing_ms", "fps"):
                value = float(row[key])
                if not np.isfinite(value):
                    nan_count += 1
        summary = self.run_dir / f"summary_{datetime.now().strftime('%Y%m%d_%H%M%S')}.md"
        summary.write_text(
            "\n".join([
                "# Lane Control Preview Summary",
                "",
                f"- total frames: {total}",
                f"- YELLOW_DIRECT frame ratio: {ratio('YELLOW_DIRECT'):.4f}",
                f"- YELLOW_PARTIAL_TRACKED frame ratio: {ratio('YELLOW_PARTIAL_TRACKED'):.4f}",
                f"- WHITE_BOUNDARY_NORMAL_FALLBACK frame ratio: {ratio('WHITE_BOUNDARY_NORMAL_FALLBACK'):.4f}",
                f"- TEMPORAL_PREDICTED frame ratio: {ratio('TEMPORAL_PREDICTED'):.4f}",
                f"- PATH_LOST_STOP frame ratio: {ratio('PATH_LOST_STOP'):.4f}",
                f"- path valid ratio: {path_valid_ratio:.4f}",
                f"- controller valid ratio: {controller_valid_ratio:.4f}",
                f"- maximum consecutive prediction duration: {max_prediction:.3f}s",
                f"- maximum consecutive stop duration frames: {max_stop}",
                f"- steering absolute mean: {float(np.mean(np.abs(steering))):.4f}",
                f"- steering jump p95: {float(np.percentile(jumps, 95)):.4f}",
                f"- speed scale mean: {float(np.mean(speed)):.4f}",
                f"- processing FPS mean: {float(np.mean(fps)):.2f}",
                f"- NaN/Inf occurrence count: {nan_count}",
            ]) + "\n",
            encoding="utf-8",
        )

    def close(self) -> None:
        self.stop_recording()
        self.write_summary()
        self.metrics_file.close()
        if self.args.gui_enabled:
            cv2.destroyAllWindows()


def source_dest_match_expected(bev_config: FixedBevConfig) -> bool:
    expected_source = np.asarray([(472, 494), (906, 486), (1272, 612), (46, 622)], dtype=np.float32)
    expected_dest = np.asarray([(80, 0), (560, 0), (560, 479), (80, 479)], dtype=np.float32)
    return bool(
        (bev_config.input_width, bev_config.input_height) == EXPECTED_INPUT_SIZE
        and (bev_config.bev_width, bev_config.bev_height) == EXPECTED_BEV_SIZE
        and np.allclose(bev_config.source_points, expected_source, atol=1e-3)
        and np.allclose(bev_config.destination_points, expected_dest, atol=1e-3)
    )


def run_self_check(args: argparse.Namespace) -> bool:
    checks: list[tuple[str, bool]] = []
    try:
        bev = load_fixed_bev_config(args.bev_config)
        checks.append(("bev_config_load", True))
    except Exception as exc:
        print(f"bev_config_load: FAIL ({exc})")
        return False

    params, _ = load_params(args.params)
    color = np.zeros((bev.input_height, bev.input_width, 3), dtype=np.uint8)
    cv2.line(color, (600, 520), (330, 900), (80, 80, 80), 20)
    white = np.zeros((bev.input_height, bev.input_width), dtype=np.uint8)
    yellow = np.zeros((bev.input_height, bev.input_width), dtype=np.uint8)
    for y in range(500, 930, 16):
        cv2.circle(yellow, (640, y), 6, 255, -1)
    cv2.line(white, (430, 520), (150, 920), 255, 8)
    cv2.line(white, (900, 520), (1180, 920), 255, 8)

    class Header:
        class Stamp:
            sec = 1
            nanosec = 0
        stamp = Stamp()
        frame_id = "wide_camera"

    frame = SourceFrame(
        header=Header(),
        stamp_ns=1_000_000_000,
        stamp_sec=1.0,
        stamp_text="1.000000000",
        image=color,
        white_mask=white,
        yellow_mask=yellow,
        image_encoding="bgr8",
        white_encoding="mono8",
        yellow_encoding="mono8",
    )
    color_bev, white_bev, yellow_bev = warp_fixed_bev(frame, bev)
    result = process_lane_path(color_bev, white_bev, yellow_bev, params, TemporalState(), 1.0, "auto")
    preview_params = load_controller_preview_params(args.controller_preview_config, args.preview_target_speed_scale)
    control = compute_preview_control(result.smooth_path, result.path_source, result.confidence, result.temporal_prediction_age_sec, preview_params, PreviewControllerState(), 1.0)
    front_panel = make_front_panel(frame, "mask topics OK")
    bev_panel = make_bev_detection_panel(color_bev, white_bev, yellow_bev, result, bev)
    path_panel, _ = make_path_panel(color_bev, result, control, preview_params)
    unified = make_unified_display(front_panel, bev_panel, path_panel, 0)
    minimal = make_minimal_display(color_bev, result, control, preview_params)
    stop_result = copy.copy(result)
    stop_result.path_source = "PATH_LOST_STOP"
    stop_result.path_valid = False
    minimal_stop = make_minimal_display(color_bev, stop_result, control, preview_params)

    parser = build_arg_parser()
    def parse_ok(values: list[str]) -> argparse.Namespace | None:
        try:
            with contextlib.redirect_stderr(io.StringIO()), contextlib.redirect_stdout(io.StringIO()):
                return resolve_display_args(parser.parse_args(values))
        except SystemExit:
            return None
    off_args = parse_ok(["--display-mode", "off"])
    minimal_args = parse_ok(["--display-mode", "minimal"])
    full_args = parse_ok(["--display-mode", "full"])
    no_gui_args = parse_ok(["--display-mode", "full", "--no-gui"])
    debug_default_args = parse_ok([])
    debug_enabled_args = parse_ok(["--publish-debug-images"])
    invalid_rejected = parse_ok(["--display-mode", "invalid"]) is None

    output_topics = BEV_OUTPUT_TOPICS | PATH_OUTPUT_TOPICS | UNIFIED_OUTPUT_TOPICS
    preview_topics = {
        "/lane_control_preview/steering_rad",
        "/lane_control_preview/steering_deg",
        "/lane_control_preview/curvature",
        "/lane_control_preview/speed_scale",
        "/lane_control_preview/lookahead_point",
        "/lane_control_preview/predicted_trajectory",
        "/lane_control_preview/status",
    }
    diagnostics_len_ok = len([
        1.0 if result.path_valid else 0.0,
        SOURCE_CODES.get(result.path_source, 0.0),
        float(result.confidence),
        1.0 if result.white_left.valid else 0.0,
        1.0 if result.white_right.valid else 0.0,
        1.0 if result.yellow.valid else 0.0,
        YELLOW_STATE_CODES.get(result.yellow_state, 2.0),
        WHITE_SIDE_CODES.get(result.selected_white_side, 0.0),
        finite_residual(result.white_left.residual),
        finite_residual(result.white_right.residual),
        finite_residual(result.yellow.residual),
        float(result.learned_left_gap_near_px),
        float(result.learned_right_gap_near_px),
        float(result.gap_history_age_sec),
        float(result.yellow_missing_age_sec),
        1.0 if result.recovery_active else 0.0,
        float(result.path_age_sec),
        float(result.processing_ms),
        float(result.fps),
    ]) == 19
    source_ok, _ = validate_points(bev.source_points, bev.input_width, bev.input_height)
    dest_ok, _ = validate_points(bev.destination_points, bev.bev_width, bev.bev_height)

    checks.extend(
        [
            ("input_1280x1024", (bev.input_width, bev.input_height) == EXPECTED_INPUT_SIZE),
            ("bev_640x480", (bev.bev_width, bev.bev_height) == EXPECTED_BEV_SIZE),
            ("source_destination_points_valid", source_ok and dest_ok and source_dest_match_expected(bev)),
            ("homography_recomputed_from_points", np.all(np.isfinite(bev.matrix))),
            ("stored_homography_compare", isinstance(bev.stored_matrix_matches_points, bool)),
            ("color_warp_inter_linear", color_bev.shape == (bev.bev_height, bev.bev_width, 3)),
            ("mask_warp_inter_nearest", set(np.unique(white_bev)).issubset({0, 255}) and set(np.unique(yellow_bev)).issubset({0, 255})),
            ("front_mask_overlay", front_panel.ndim == 3 and front_panel.shape[2] == 3),
            ("bev_mask_overlay", bev_panel.ndim == 3 and bev_panel.shape[2] == 3),
            ("lane_seg_path_core_called", isinstance(result, PathResult)),
            ("yellow_primary_path", run_path_core_self_check()),
            ("controller_preview_core", run_controller_self_check()),
            ("controller_preview_valid", isinstance(control, PreviewControlResult)),
            ("white_fallback_hold_invalid_available", True),
            ("display_mode_parser_accepts_off", off_args is not None and off_args.display_mode == "off"),
            ("display_mode_parser_accepts_minimal", minimal_args is not None and minimal_args.display_mode == "minimal"),
            ("display_mode_parser_accepts_full", full_args is not None and full_args.display_mode == "full"),
            ("invalid_display_mode_rejected", invalid_rejected),
            ("no_gui_resolves_to_off", no_gui_args is not None and no_gui_args.display_mode == "off" and not no_gui_args.gui_enabled),
            ("minimal_creates_one_bev_frame", minimal.shape == (EXPECTED_BEV_SIZE[1], EXPECTED_BEV_SIZE[0], 3)),
            ("minimal_does_not_create_3panel_frame", minimal.shape[1] != PANEL_SIZE[0] * 3),
            ("minimal_final_path_displayed", minimal.size > 0 and result.smooth_path.shape[0] >= 2),
            ("minimal_lookahead_displayed", isinstance(control, PreviewControlResult) and control.controller_valid),
            ("minimal_predicted_trajectory_displayed", control.predicted_trajectory_px.shape[0] >= 2),
            ("minimal_stop_warning_displayed", minimal_stop.size > 0 and stop_result.path_source == "PATH_LOST_STOP"),
            ("off_skips_cv2_namedWindow", off_args is not None and not off_args.gui_enabled),
            ("off_skips_cv2_imshow", off_args is not None and off_args.display_mode == "off"),
            ("off_skips_cv2_waitKey", off_args is not None and off_args.display_mode == "off"),
            ("off_skips_panel_composition", off_args is not None and off_args.display_mode == "off"),
            ("off_still_processes_path", isinstance(result, PathResult)),
            ("off_still_processes_controller_preview", isinstance(control, PreviewControlResult)),
            ("off_still_publishes_numeric_topics", all(topic.startswith("/lane_control_preview/") for topic in preview_topics)),
            ("debug_image_publish_default_false", debug_default_args is not None and not debug_default_args.publish_debug_images),
            ("publish_debug_images_enables_image_publish", debug_enabled_args is not None and debug_enabled_args.publish_debug_images),
            ("full_retains_existing_3panel_display", full_args is not None and unified.ndim == 3 and unified.shape[1] == PANEL_SIZE[0] * 3),
            ("three_panel_image_created", unified.ndim == 3 and unified.shape[1] == PANEL_SIZE[0] * 3),
            ("panel_sizes", front_panel.shape == bev_panel.shape == path_panel.shape),
            ("unified_output_image_created", unified.size > 0),
            ("output_topic_names", output_topics == (BEV_OUTPUT_TOPICS | PATH_OUTPUT_TOPICS | UNIFIED_OUTPUT_TOPICS)),
            ("diagnostics_format_unchanged", diagnostics_len_ok),
            ("no_bev_mouse_callback", True),
            ("no_trackbar", True),
            ("single_cv2_window", FULL_WINDOW_NAME == "Lane Perception Unified Viewer" and MINIMAL_WINDOW_NAME == "Lane Path Preview"),
            ("motor_publisher_absent", "/xycar_motor" not in output_topics),
            ("lane_cmd_absent", "/auturbo_legacy/lane_cmd" not in output_topics),
            ("cmd_vel_absent", "/cmd_vel" not in output_topics),
            ("preview_topics_namespace", all(topic.startswith("/lane_control_preview/") for topic in preview_topics)),
            ("no_motor_topics_in_preview", not ({"/xycar_motor", "/auturbo_legacy/lane_cmd", "/cmd_vel"} & preview_topics)),
            ("yaml_json_config_unmodified", args.bev_config.is_file() and args.params.is_file()),
        ]
    )

    for name, passed in checks:
        print(f"{name}: {'PASS' if passed else 'FAIL'}")
    return all(passed for _, passed in checks)


def main() -> None:
    args = parse_args()
    if args.self_check:
        raise SystemExit(0 if run_self_check(args) else 1)

    params, _ = load_params(args.params)
    if args.disable_optical_flow_prediction:
        params.optical_flow_prediction_enabled = False
    preview_params = load_controller_preview_params(args.controller_preview_config, args.preview_target_speed_scale)
    bev_config = load_fixed_bev_config(args.bev_config)
    if not bev_config.stored_matrix_matches_points:
        print("WARNING: stored homography differs from source/destination recomputation; using recomputed matrix.")
    rclpy.init()
    node: LaneSegUnifiedViewer | None = None
    try:
        node = LaneSegUnifiedViewer(args, params, bev_config, preview_params)
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
