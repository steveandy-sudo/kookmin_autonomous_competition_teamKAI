#!/usr/bin/env python3
"""Review the learned deterministic W1/W2 selection on saved BEV frames."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .sequence_entry_core import LineHypothesis, SequenceAwareEntrySelector


WINDOW = "Canonical W1-W2 Selection Review"


def _draw_line(
    image: np.ndarray,
    line: LineHypothesis,
    color: tuple[int, int, int],
    label: str,
) -> None:
    height, width = image.shape[:2]
    y1 = int(round(line.minimum_y_ratio * height))
    y2 = int(round(line.maximum_y_ratio * height))
    x1 = int(round(line.x_ratio_at(line.minimum_y_ratio) * width))
    x2 = int(round(line.x_ratio_at(line.maximum_y_ratio) * width))
    cv2.line(image, (x1, y1), (x2, y2), color, 3, cv2.LINE_AA)
    cv2.putText(
        image,
        label,
        (max(0, min(width - 35, (x1 + x2) // 2 + 4)),
         max(18, min(height - 3, (y1 + y2) // 2))),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )


def _panel(
    document: dict,
    directory: Path,
    selector: SequenceAwareEntrySelector,
    total_frames: int,
) -> np.ndarray:
    stamp = int(document["timestamp_ns"])
    stem = directory / f"canonical_white_{stamp}"
    white = cv2.imread(f"{stem}_white.png", cv2.IMREAD_GRAYSCALE)
    yellow = cv2.imread(f"{stem}_yellow.png", cv2.IMREAD_GRAYSCALE)
    camera = cv2.imread(f"{stem}_bev_model_overlay.png", cv2.IMREAD_COLOR)
    if camera is None:
        camera = cv2.imread(f"{stem}_bev_camera.png", cv2.IMREAD_COLOR)
    if white is None or yellow is None or camera is None:
        raise FileNotFoundError(f"saved images missing for {stem.name}")
    result = selector.process(white, yellow)

    canonical = np.full((*white.shape, 3), 22, dtype=np.uint8)
    canonical[white > 0] = (235, 235, 235)
    canonical[yellow > 0] = (0, 220, 255)
    if result.w1 is not None:
        _draw_line(canonical, result.w1, (255, 0, 255), "W1")
    if result.w2 is not None:
        _draw_line(canonical, result.w2, (255, 150, 40), "W2")

    camera = cv2.resize(camera, (640, 360), interpolation=cv2.INTER_AREA)
    canonical = cv2.resize(
        canonical, (640, 360), interpolation=cv2.INTER_NEAREST
    )
    canvas = np.full((440, 1280, 3), (24, 20, 19), dtype=np.uint8)
    canvas[52:412, :640] = camera
    canvas[52:412, 640:] = canonical
    cv2.putText(
        canvas,
        "BEV CAMERA + LANE MODEL",
        (18, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (245, 245, 245),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "SELECTED CANONICAL: W1 / W2",
        (660, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (245, 245, 245),
        2,
        cv2.LINE_AA,
    )
    w1_slope = result.w1.direction_dx_dy if result.w1 else float("nan")
    w2_slope = result.w2.direction_dx_dy if result.w2 else float("nan")
    status = (
        f"sample {int(document['sample_number']):02d}/{total_frames:02d}   "
        f"W1 slope {w1_slope:+.3f}   W2 slope {w2_slope:+.3f}   "
        "SPACE pause/play   A/D step   Q quit"
    )
    cv2.putText(
        canvas,
        status,
        (18, 432),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (210, 220, 230),
        1,
        cv2.LINE_AA,
    )
    return canvas


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("annotation_dir", type=Path)
    parser.add_argument("--interval-ms", type=int, default=400)
    arguments = parser.parse_args()
    documents = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in arguments.annotation_dir.glob("canonical_white_*.json")
    ]
    if not documents:
        raise FileNotFoundError(
            f"no canonical_white_*.json in {arguments.annotation_dir}"
        )
    documents.sort(key=lambda item: int(item["sample_number"]))

    selector = SequenceAwareEntrySelector()
    panels = [
        _panel(document, arguments.annotation_dir, selector, len(documents))
        for document in documents
    ]
    index = 0
    paused = False
    fullscreen = False
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1280, 440)
    try:
        while True:
            frame = panels[index].copy()
            if paused:
                cv2.putText(
                    frame,
                    "PAUSED",
                    (1120, 34),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.68,
                    (0, 230, 255),
                    2,
                    cv2.LINE_AA,
                )
            cv2.imshow(WINDOW, frame)
            key = cv2.waitKey(30 if paused else max(30, arguments.interval_ms))
            key &= 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                paused = not paused
            elif key in (ord("a"), ord("A")):
                index = max(0, index - 1)
                paused = True
            elif key in (ord("d"), ord("D")):
                index = min(len(panels) - 1, index + 1)
                paused = True
            elif key in (ord("f"), ord("F")):
                fullscreen = not fullscreen
                cv2.setWindowProperty(
                    WINDOW,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL,
                )
            elif not paused:
                index = (index + 1) % len(panels)
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
