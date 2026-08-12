#!/usr/bin/env python3
"""Direct W1/W2 line annotation on the branch-preserving canonical image."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosbag2_interfaces.srv import Pause, Resume
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image

from lane_seg_control.canonical_adapter_node import build_bev_geometry
from lane_seg_control.white_lane_fitter import fit_yellow_centerline_reference
from shortcut_entry_review.sequence_entry_core import extract_line_hypotheses


WINDOW = "KAI Canonical W1/W2 Manual Line Annotation"
CANVAS_WIDTH = 1440
CANVAS_HEIGHT = 820
TEXT = (235, 239, 245)
MUTED = (148, 158, 174)
LINE_COLORS = {
    "W1": (255, 0, 255),
    "W2": (255, 145, 30),
}
BEV_WIDTH = 640
BEV_HEIGHT = 660
CANONICAL_WIDTH = 256
CANONICAL_HEIGHT = 144
SOURCE_RATIOS = (
    0.442578,
    0.480781,
    0.688281,
    0.480781,
    0.919141,
    0.614189,
    0.190625,
    0.614189,
)
DESTINATION_RATIOS = (0.205714, 0.794286, 0.0, 0.666666667)


def message_stamp_ns(message) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


def fit_manual_line(
    points_px: list[tuple[float, float]],
    *,
    image_width: int,
    image_height: int,
) -> dict | None:
    """Fit one deterministic straight x(y) line over the clicked y range."""
    if len(points_px) < 2:
        return None
    points = np.asarray(points_px, dtype=np.float64)
    x_values = points[:, 0]
    y_values = points[:, 1]
    if float(np.ptp(y_values)) < 1.0:
        return None
    coefficients = np.polyfit(y_values, x_values, 1)
    fitted_x = np.polyval(coefficients, y_values)
    rmse_px = float(np.sqrt(np.mean(np.square(x_values - fitted_x))))
    y_min = float(y_values.min())
    y_max = float(y_values.max())
    x_min_y = float(np.polyval(coefficients, y_min))
    x_max_y = float(np.polyval(coefficients, y_max))
    normalized_coefficients = (
        float(coefficients[0]) * float(image_height) / float(image_width),
        float(coefficients[1]) / float(image_width),
    )
    return {
        "coefficients_x_px_from_y_px": coefficients.tolist(),
        "coefficients_x_ratio_from_y_ratio": list(normalized_coefficients),
        "direction_dx_dy": normalized_coefficients[0],
        "canonical_display_angle_deg": math.degrees(
            math.atan(float(coefficients[0]))
        ),
        "rmse_px": rmse_px,
        "observed_y_span_ratio": (y_max - y_min) / float(image_height),
        "segment_px": [[x_min_y, y_min], [x_max_y, y_max]],
    }


class CanonicalWhiteAnnotator(Node):
    """Synchronize canonical topics and collect zero, one, or two white lines."""

    def __init__(self) -> None:
        super().__init__("kai_canonical_white_annotator")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.bridge = CvBridge()
        self.frames: OrderedDict[int, dict] = OrderedDict()
        self.live_frame: dict | None = None
        self.frozen_frame: dict | None = None
        self.bev_geometry = None
        self.bev_geometry_input_size: tuple[int, int] | None = None
        self.paused = False
        self.sample_interval_sec = max(
            0.05,
            float(os.environ.get("KAI_ANNOTATION_INTERVAL_SEC", "0.2")),
        )
        self.sample_interval_ns = int(round(self.sample_interval_sec * 1.0e9))
        self.next_sample_stamp_ns: int | None = None
        self.sample_number = 0
        self.active_label = "W1"
        self.points: dict[str, list[tuple[float, float]]] = {
            "W1": [],
            "W2": [],
        }
        self.canonical_display_rect: tuple[int, int, int, int] | None = None
        self.status = (
            f"AUTO sampling every {self.sample_interval_sec:.2f}s - waiting for frame"
        )
        self.clock_ns: int | None = None
        self.first_clock_ns: int | None = None
        self.saved_path: Path | None = None
        default_output = (
            "/home/kai/kookmin_autonomous_competition_teamKAI/"
            "analysis/canonical_white_line_annotations"
        )
        self.output_dir = Path(
            os.environ.get("KAI_CANONICAL_ANNOTATION_DIR", default_output)
        ).expanduser()

        self._subscriptions = [
            self.create_subscription(
                Image,
                "/lane_seg/source_image",
                self.on_source_image,
                qos,
            ),
            self.create_subscription(
                Image,
                "/perception/canonical_road_image",
                self.on_canonical,
                qos,
            ),
            self.create_subscription(
                Image,
                "/perception/canonical_white_mask",
                self.on_white,
                qos,
            ),
            self.create_subscription(
                Image,
                "/perception/canonical_yellow_mask",
                self.on_yellow,
                qos,
            ),
            self.create_subscription(Clock, "/clock", self.on_clock, qos),
        ]
        self.pause_client = self.create_client(Pause, "/rosbag2_player/pause")
        self.resume_client = self.create_client(Resume, "/rosbag2_player/resume")

    def frame_slot(self, stamp_ns: int) -> dict:
        slot = self.frames.setdefault(int(stamp_ns), {"stamp_ns": int(stamp_ns)})
        self.frames.move_to_end(int(stamp_ns))
        while len(self.frames) > 24:
            self.frames.popitem(last=False)
        return slot

    def refresh_live_frame(self, stamp_ns: int) -> None:
        slot = self.frames.get(int(stamp_ns))
        if slot is None:
            return
        if not all(
            name in slot
            for name in ("bev_camera", "canonical", "white", "yellow")
        ):
            return
        self.live_frame = slot
        self.maybe_freeze_sample(slot)

    def ensure_bev_geometry(self, width: int, height: int):
        input_size = (int(width), int(height))
        if self.bev_geometry is not None and input_size == self.bev_geometry_input_size:
            return self.bev_geometry
        self.bev_geometry = build_bev_geometry(
            width,
            height,
            source_ratios=SOURCE_RATIOS,
            destination_ratios=DESTINATION_RATIOS,
            bev_width=BEV_WIDTH,
            bev_height=BEV_HEIGHT,
        )
        self.bev_geometry_input_size = input_size
        return self.bev_geometry

    def on_source_image(self, message: Image) -> None:
        try:
            source = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().warning(f"source image conversion failed: {error}")
            return
        geometry = self.ensure_bev_geometry(source.shape[1], source.shape[0])
        bev = cv2.warpPerspective(
            source,
            geometry.matrix,
            (BEV_WIDTH, BEV_HEIGHT),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(36, 36, 36),
        )
        canonical_bev = cv2.resize(
            bev,
            (CANONICAL_WIDTH, CANONICAL_HEIGHT),
            interpolation=cv2.INTER_AREA,
        )
        stamp_ns = message_stamp_ns(message)
        self.frame_slot(stamp_ns)["bev_camera"] = canonical_bev
        self.refresh_live_frame(stamp_ns)

    def receive_image(self, message: Image, name: str, encoding: str) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding=encoding)
        except Exception as error:
            self.get_logger().warning(f"{name} conversion failed: {error}")
            return
        stamp_ns = message_stamp_ns(message)
        self.frame_slot(stamp_ns)[name] = image
        self.refresh_live_frame(stamp_ns)

    def on_canonical(self, message: Image) -> None:
        self.receive_image(message, "canonical", "bgr8")

    def on_white(self, message: Image) -> None:
        self.receive_image(message, "white", "mono8")

    def on_yellow(self, message: Image) -> None:
        self.receive_image(message, "yellow", "mono8")

    def on_clock(self, message: Clock) -> None:
        current = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
        if self.clock_ns is not None and current < self.clock_ns - 500_000_000:
            self.first_clock_ns = current
        if self.first_clock_ns is None:
            self.first_clock_ns = current
        self.clock_ns = current

    def bag_offset(self) -> float:
        if self.clock_ns is None or self.first_clock_ns is None:
            return 0.0
        return max(0.0, (self.clock_ns - self.first_clock_ns) / 1.0e9)

    def active_frame(self) -> dict | None:
        return self.frozen_frame if self.paused else self.live_frame

    @staticmethod
    def copy_frame(frame: dict) -> dict:
        return {
            key: value.copy() if isinstance(value, np.ndarray) else value
            for key, value in frame.items()
        }

    def reset_points(self) -> None:
        self.points = {"W1": [], "W2": []}
        self.active_label = "W1"
        self.saved_path = None

    def maybe_freeze_sample(self, frame: dict) -> None:
        if self.paused:
            return
        stamp_ns = int(frame["stamp_ns"])
        if (
            self.next_sample_stamp_ns is not None
            and stamp_ns < self.next_sample_stamp_ns
        ):
            return
        if not self.pause_client.service_is_ready():
            self.status = "Waiting for rosbag pause service"
            return
        self.pause_client.call_async(Pause.Request())
        self.frozen_frame = self.copy_frame(frame)
        self.frozen_frame["bag_offset_sec"] = self.bag_offset()
        self.paused = True
        self.sample_number += 1
        self.frozen_frame["sample_number"] = self.sample_number
        self.reset_points()
        self.status = (
            f"AUTO SAMPLE {self.sample_number} - label W1/W2, then S or N"
        )

    def advance_to_next_sample(self, prefix: str) -> None:
        if not self.paused or self.frozen_frame is None:
            self.status = "No frozen automatic sample"
            return
        if not self.resume_client.service_is_ready():
            self.status = "rosbag resume service is not ready"
            return
        current_stamp_ns = int(self.frozen_frame["stamp_ns"])
        self.next_sample_stamp_ns = current_stamp_ns + self.sample_interval_ns
        self.resume_client.call_async(Resume.Request())
        self.paused = False
        self.frozen_frame = None
        self.reset_points()
        self.status = (
            f"{prefix} | seeking +{self.sample_interval_sec:.2f}s next sample"
        )

    def skip(self) -> None:
        self.advance_to_next_sample("SKIPPED")

    def select_label(self, label: str) -> None:
        self.active_label = label
        self.status = f"{label} selected - click at least two points along the line"

    def undo(self) -> None:
        if self.points[self.active_label]:
            self.points[self.active_label].pop()
            self.status = f"{self.active_label}: last point removed"
        else:
            self.status = f"{self.active_label}: no point to undo"

    def clear_active(self) -> None:
        self.points[self.active_label] = []
        self.status = f"{self.active_label}: cleared"

    def on_mouse(self, event: int, x: int, y: int, _flags: int) -> None:
        if event == cv2.EVENT_RBUTTONDOWN:
            self.undo()
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        frame = self.active_frame()
        rect = self.canonical_display_rect
        if not self.paused or self.frozen_frame is None:
            self.status = "Wait for the next automatic sample"
            return
        if frame is None or rect is None:
            return
        left, top, shown_width, shown_height = rect
        if not (left <= x < left + shown_width and top <= y < top + shown_height):
            self.status = "Click inside the canonical image on the right"
            return
        source_height, source_width = frame["canonical"].shape[:2]
        point_x = (float(x - left) + 0.5) * source_width / shown_width - 0.5
        point_y = (float(y - top) + 0.5) * source_height / shown_height - 0.5
        point_x = float(np.clip(point_x, 0.0, source_width - 1.0))
        point_y = float(np.clip(point_y, 0.0, source_height - 1.0))
        self.points[self.active_label].append((point_x, point_y))
        count = len(self.points[self.active_label])
        self.status = f"{self.active_label}: point {count} - continue or select the other line"

    @staticmethod
    def place_aspect(canvas, image, box, interpolation):
        x, y, width, height = box
        cv2.rectangle(canvas, (x, y), (x + width, y + height), (29, 34, 43), -1)
        if image is None:
            return None
        scale = min(width / image.shape[1], height / image.shape[0])
        shown_width = max(1, int(round(image.shape[1] * scale)))
        shown_height = max(1, int(round(image.shape[0] * scale)))
        resized = cv2.resize(
            image,
            (shown_width, shown_height),
            interpolation=interpolation,
        )
        left = x + (width - shown_width) // 2
        top = y + (height - shown_height) // 2
        canvas[top : top + shown_height, left : left + shown_width] = resized
        return left, top, shown_width, shown_height

    def annotated_canonical(self, frame: dict) -> np.ndarray:
        output = frame["canonical"].copy()
        height, width = output.shape[:2]
        for label in ("W1", "W2"):
            color = LINE_COLORS[label]
            points = self.points[label]
            for point in points:
                cv2.circle(
                    output,
                    tuple(np.rint(point).astype(np.int32)),
                    2,
                    color,
                    -1,
                    cv2.LINE_AA,
                )
            fit = fit_manual_line(
                points,
                image_width=width,
                image_height=height,
            )
            if fit is None:
                continue
            segment = np.rint(np.asarray(fit["segment_px"])).astype(np.int32)
            segment[:, 0] = np.clip(segment[:, 0], 0, width - 1)
            segment[:, 1] = np.clip(segment[:, 1], 0, height - 1)
            cv2.line(
                output,
                tuple(segment[0]),
                tuple(segment[1]),
                color,
                2,
                cv2.LINE_AA,
            )
            label_x = int(np.clip(segment[0, 0] + 3, 0, width - 26))
            label_y = int(np.clip(segment[0, 1], 10, height - 2))
            cv2.putText(
                output,
                label,
                (label_x, label_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.34,
                color,
                1,
                cv2.LINE_AA,
            )
        return output

    @staticmethod
    def model_overlay_bev(frame: dict) -> np.ndarray:
        """Blend branch-preserving LR-ASPP classes onto the color camera BEV."""
        output = frame["bev_camera"].copy()
        for mask_name, color in (
            ("white", np.asarray((255, 255, 255), dtype=np.float32)),
            ("yellow", np.asarray((0, 220, 255), dtype=np.float32)),
        ):
            selected = frame[mask_name] > 0
            if not np.any(selected):
                continue
            blended = (
                output[selected].astype(np.float32) * 0.25 + color * 0.75
            )
            output[selected] = np.clip(blended, 0, 255).astype(np.uint8)
        return output

    @staticmethod
    def candidate_document(candidate, yellow_reference, width, height) -> dict:
        document = asdict(candidate)
        if yellow_reference is None or not yellow_reference.valid:
            document["right_of_yellow_fraction"] = None
            document["mean_x_minus_yellow_px"] = None
            return document
        rows = np.linspace(
            candidate.minimum_y_ratio,
            candidate.maximum_y_ratio,
            21,
        )
        row_indices = np.clip(
            np.rint(rows * height).astype(np.int32), 0, height - 1
        )
        white_x = np.asarray(
            [candidate.x_ratio_at(float(row)) * width for row in rows]
        )
        yellow_x = yellow_reference.x_by_y[row_indices]
        delta = white_x - yellow_x
        document["right_of_yellow_fraction"] = float(np.mean(delta >= 0.0))
        document["mean_x_minus_yellow_px"] = float(np.mean(delta))
        return document

    def build_document(self, frame: dict) -> dict:
        canonical = frame["canonical"]
        height, width = canonical.shape[:2]
        line_records = []
        for label in ("W1", "W2"):
            points = self.points[label]
            fit = fit_manual_line(
                points,
                image_width=width,
                image_height=height,
            )
            line_records.append(
                {
                    "label": label,
                    "detected": fit is not None,
                    "points_px": [list(point) for point in points],
                    "points_normalized": [
                        [point[0] / width, point[1] / height] for point in points
                    ],
                    "fit": fit,
                }
            )

        yellow_reference = fit_yellow_centerline_reference(
            frame["yellow"],
            min_pixels=3,
            residual_threshold_px=6.0,
            line_width_px=3,
        )
        candidates = extract_line_hypotheses(frame["white"], "white")
        candidate_records = [
            self.candidate_document(item, yellow_reference, width, height)
            for item in candidates
        ]
        yellow_document = {
            "valid": bool(yellow_reference.valid),
            "coefficients_x_px_from_y_px": (
                yellow_reference.coefficients.tolist()
                if yellow_reference.valid
                else None
            ),
            "component_count": int(yellow_reference.component_count),
            "pixel_count": int(yellow_reference.pixel_count),
            "rmse_px": (
                float(yellow_reference.rmse_px)
                if math.isfinite(float(yellow_reference.rmse_px))
                else None
            ),
        }
        return {
            "schema_version": 1,
            "contract": "manual W1/W2 straight lines on raw 256x144 canonical",
            "timestamp_ns": int(frame["stamp_ns"]),
            "timestamp_sec": float(frame["stamp_ns"]) / 1.0e9,
            "bag_offset_sec": frame.get("bag_offset_sec"),
            "sample_number": frame.get("sample_number"),
            "sampling_interval_sec": self.sample_interval_sec,
            "image_width": int(width),
            "image_height": int(height),
            "none": not any(record["detected"] for record in line_records),
            "lines": line_records,
            "yellow_reference": yellow_document,
            "automatic_white_candidates": candidate_records,
        }

    def save(self, *, force_none: bool = False) -> None:
        if not self.paused or self.frozen_frame is None:
            self.status = "Wait for the next automatic sample"
            return
        if force_none:
            self.points = {"W1": [], "W2": []}
        invalid = [
            label
            for label, points in self.points.items()
            if len(points) == 1
            or (
                len(points) >= 2
                and fit_manual_line(
                    points,
                    image_width=self.frozen_frame["canonical"].shape[1],
                    image_height=self.frozen_frame["canonical"].shape[0],
                )
                is None
            )
        ]
        if invalid:
            self.status = (
                f"SAVE FAILED: {', '.join(invalid)} needs >=2 points at different y"
            )
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp_ns = int(self.frozen_frame["stamp_ns"])
        stem = f"canonical_white_{stamp_ns}"
        index = 2
        while (self.output_dir / f"{stem}.json").exists():
            stem = f"canonical_white_{stamp_ns}_{index:02d}"
            index += 1
        document = self.build_document(self.frozen_frame)
        json_path = self.output_dir / f"{stem}.json"
        json_path.write_text(
            json.dumps(document, indent=2, ensure_ascii=False, allow_nan=False)
            + "\n",
            encoding="utf-8",
        )
        cv2.imwrite(
            str(self.output_dir / f"{stem}_canonical.png"),
            self.frozen_frame["canonical"],
        )
        cv2.imwrite(
            str(self.output_dir / f"{stem}_white.png"),
            self.frozen_frame["white"],
        )
        cv2.imwrite(
            str(self.output_dir / f"{stem}_yellow.png"),
            self.frozen_frame["yellow"],
        )
        cv2.imwrite(
            str(self.output_dir / f"{stem}_annotated.png"),
            self.annotated_canonical(self.frozen_frame),
        )
        cv2.imwrite(
            str(self.output_dir / f"{stem}_bev_camera.png"),
            self.frozen_frame["bev_camera"],
        )
        cv2.imwrite(
            str(self.output_dir / f"{stem}_bev_model_overlay.png"),
            self.model_overlay_bev(self.frozen_frame),
        )
        self.saved_path = json_path
        detected = [
            record["label"] for record in document["lines"] if record["detected"]
        ]
        selection = "+".join(detected) if detected else "NONE"
        self.advance_to_next_sample(
            f"SAVED [{selection}]: {json_path.name}"
        )

    def render(self) -> np.ndarray:
        canvas = np.full((CANVAS_HEIGHT, CANVAS_WIDTH, 3), (17, 20, 26), np.uint8)
        frame = self.active_frame()
        camera = self.model_overlay_bev(frame) if frame is not None else None
        canonical = self.annotated_canonical(frame) if frame is not None else None

        cv2.putText(
            canvas,
            "LR-ASPP MODEL OVERLAY ON BIRD'S-EYE CAMERA",
            (20, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            TEXT,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "CLICK CANONICAL: MANUAL W1 / W2 STRAIGHT FIT",
            (600, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            TEXT,
            2,
            cv2.LINE_AA,
        )
        self.place_aspect(canvas, camera, (10, 54, 560, 600), cv2.INTER_AREA)
        self.canonical_display_rect = self.place_aspect(
            canvas,
            canonical,
            (590, 54, 840, 600),
            cv2.INTER_NEAREST,
        )
        if self.canonical_display_rect is not None:
            left, top, shown_width, shown_height = self.canonical_display_rect
            cv2.rectangle(
                canvas,
                (left, top),
                (left + shown_width - 1, top + shown_height - 1),
                LINE_COLORS[self.active_label],
                3,
            )
        else:
            cv2.putText(
                canvas,
                "Waiting for synchronized canonical/masks ...",
                (700, 170),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                MUTED,
                1,
                cv2.LINE_AA,
            )

        state = (
            f"AUTO SAMPLE {self.sample_number} / FROZEN"
            if self.paused
            else "AUTO SEEK / PLAYING 0.5x"
        )
        counts = f"W1 points={len(self.points['W1'])}   W2 points={len(self.points['W2'])}"
        cv2.putText(
            canvas,
            f"{state}   active={self.active_label}   {counts}",
            (20, 690),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.64,
            LINE_COLORS[self.active_label],
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"AUTO INTERVAL {self.sample_interval_sec:.2f}s | 1 W1 | 2 W2 | LEFT add | RIGHT/Z undo | C/R clear",
            (20, 730),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            TEXT,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "S save + next | N save NONE + next | K skip without saving + next | Q quit",
            (20, 760),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.54,
            TEXT,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            self.status,
            (20, 796),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.49,
            (80, 225, 250) if self.paused else (100, 230, 145),
            1,
            cv2.LINE_AA,
        )
        if frame is not None:
            stamp_text = f"stamp {int(frame['stamp_ns']) / 1.0e9:.9f}  bag {self.bag_offset():.2f}s"
            cv2.putText(
                canvas,
                stamp_text,
                (1010, 690),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                MUTED,
                1,
                cv2.LINE_AA,
            )
        return canvas


def main() -> None:
    cv2.setNumThreads(1)
    rclpy.init()
    node = CanonicalWhiteAnnotator()
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, CANVAS_WIDTH, CANVAS_HEIGHT)
    cv2.setMouseCallback(
        WINDOW,
        lambda event, x, y, flags, _parameter: node.on_mouse(event, x, y, flags),
    )
    next_render = 0.0
    print("Canonical W1/W2 manual annotator ready", flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()
            if now >= next_render:
                cv2.imshow(WINDOW, node.render())
                next_render = now + 0.05
            key = cv2.waitKey(1) & 0xFF
            if key == ord("1"):
                node.select_label("W1")
            elif key == ord("2"):
                node.select_label("W2")
            elif key in (ord("z"), ord("Z"), 8, 127):
                node.undo()
            elif key in (ord("c"), ord("C")):
                node.clear_active()
            elif key in (ord("r"), ord("R")):
                node.reset_points()
                node.status = "W1/W2 points cleared"
            elif key in (ord("s"), ord("S")):
                node.save()
            elif key in (ord("n"), ord("N")):
                node.save(force_none=True)
            elif key in (ord("k"), ord("K")):
                node.skip()
            elif key in (ord("q"), ord("Q"), 27):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
