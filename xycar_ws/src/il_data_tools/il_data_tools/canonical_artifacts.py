from __future__ import annotations

from dataclasses import dataclass
import random
from typing import Dict, Optional, Tuple

import numpy as np


BACKGROUND_GRAY = 36
WHITE_BGR = (255, 255, 255)
YELLOW_BGR = (0, 220, 255)


@dataclass
class ArtifactEvent:
    kind: str
    remaining_frames: int
    parameters: Dict[str, float | str]


def canonical_masks(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("canonical image must be BGR with three channels")
    white = np.all(image >= 235, axis=2)
    blue, green, red = cv_channels(image)
    yellow = (
        (blue <= 80)
        & (green >= 150)
        & (red >= 180)
        & ((red.astype(np.int16) - blue.astype(np.int16)) >= 100)
    )
    return white, yellow


def cv_channels(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    return image[:, :, 0], image[:, :, 1], image[:, :, 2]


def _paint_background(
    image: np.ndarray,
    mask: np.ndarray,
    background_gray: int,
) -> None:
    image[mask] = (background_gray, background_gray, background_gray)


def _translated_mask(mask: np.ndarray, row_offsets: np.ndarray) -> np.ndarray:
    height, width = mask.shape
    if row_offsets.shape != (height,):
        raise ValueError("row_offsets must contain one value per image row")
    ys, xs = np.nonzero(mask)
    shifted_x = np.rint(xs + row_offsets[ys]).astype(np.int32)
    valid = (shifted_x >= 0) & (shifted_x < width)
    output = np.zeros_like(mask)
    output[ys[valid], shifted_x[valid]] = True
    return output


def apply_center_jump(
    image: np.ndarray,
    offset_px: float,
    background_gray: int = BACKGROUND_GRAY,
) -> np.ndarray:
    output = image.copy()
    _, yellow = canonical_masks(output)
    if not np.any(yellow):
        return output
    _paint_background(output, yellow, background_gray)
    offsets = np.full(output.shape[0], float(offset_px), dtype=np.float32)
    shifted = _translated_mask(yellow, offsets)
    output[shifted] = YELLOW_BGR
    return output


def apply_white_bend(
    image: np.ndarray,
    side: str,
    amplitude_px: float,
    exponent: float = 1.8,
    background_gray: int = BACKGROUND_GRAY,
) -> np.ndarray:
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    output = image.copy()
    white, _ = canonical_masks(output)
    columns = np.arange(output.shape[1])[None, :]
    selected = white & (
        columns < output.shape[1] // 2
        if side == "left"
        else columns >= output.shape[1] // 2
    )
    if not np.any(selected):
        return output
    _paint_background(output, selected, background_gray)
    normalized_rows = np.linspace(0.0, 1.0, output.shape[0], dtype=np.float32)
    offsets = float(amplitude_px) * np.power(normalized_rows, float(exponent))
    shifted = _translated_mask(selected, offsets)
    output[shifted] = WHITE_BGR
    return output


def apply_line_dropout(
    image: np.ndarray,
    line: str,
    side: str = "left",
    start_row_ratio: float = 0.0,
    end_row_ratio: float = 1.0,
    background_gray: int = BACKGROUND_GRAY,
) -> np.ndarray:
    if line not in {"white", "yellow"}:
        raise ValueError("line must be white or yellow")
    output = image.copy()
    white, yellow = canonical_masks(output)
    selected = yellow if line == "yellow" else white
    if line == "white":
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        columns = np.arange(output.shape[1])[None, :]
        selected &= (
            columns < output.shape[1] // 2
            if side == "left"
            else columns >= output.shape[1] // 2
        )
    start = int(round(np.clip(start_row_ratio, 0.0, 1.0) * output.shape[0]))
    end = int(round(np.clip(end_row_ratio, 0.0, 1.0) * output.shape[0]))
    row_mask = np.zeros(output.shape[:2], dtype=bool)
    row_mask[min(start, end) : max(start, end)] = True
    _paint_background(output, selected & row_mask, background_gray)
    return output


class CanonicalArtifactAugmenter:
    """Inject short real-camera-like lane artifacts without changing labels."""

    EVENT_KINDS = (
        "center_jump",
        "white_bend",
        "yellow_dropout",
        "white_dropout",
    )

    def __init__(
        self,
        seed: int,
        event_start_probability: float = 0.018,
        min_duration_frames: int = 3,
        max_duration_frames: int = 9,
        center_jump_min_px: float = 7.0,
        center_jump_max_px: float = 22.0,
        white_bend_min_px: float = 18.0,
        white_bend_max_px: float = 58.0,
        background_gray: int = BACKGROUND_GRAY,
    ) -> None:
        if not 0.0 <= event_start_probability <= 1.0:
            raise ValueError("event_start_probability must be in [0, 1]")
        if min_duration_frames <= 0 or max_duration_frames < min_duration_frames:
            raise ValueError("invalid artifact duration range")
        self.rng = random.Random(int(seed))
        self.event_start_probability = float(event_start_probability)
        self.min_duration_frames = int(min_duration_frames)
        self.max_duration_frames = int(max_duration_frames)
        self.center_jump_min_px = float(center_jump_min_px)
        self.center_jump_max_px = float(center_jump_max_px)
        self.white_bend_min_px = float(white_bend_min_px)
        self.white_bend_max_px = float(white_bend_max_px)
        self.background_gray = int(background_gray)
        self.current_event: Optional[ArtifactEvent] = None
        self.last_started_event: Optional[ArtifactEvent] = None

    def _signed_uniform(self, low: float, high: float) -> float:
        magnitude = self.rng.uniform(low, high)
        return magnitude if self.rng.random() < 0.5 else -magnitude

    def _new_event(self) -> ArtifactEvent:
        kind = self.rng.choices(
            self.EVENT_KINDS,
            weights=(0.32, 0.36, 0.18, 0.14),
            k=1,
        )[0]
        duration = self.rng.randint(
            self.min_duration_frames,
            self.max_duration_frames,
        )
        parameters: Dict[str, float | str] = {}
        if kind == "center_jump":
            parameters["offset_px"] = self._signed_uniform(
                self.center_jump_min_px,
                self.center_jump_max_px,
            )
        elif kind == "white_bend":
            side = self.rng.choice(("left", "right"))
            inward_sign = 1.0 if side == "left" else -1.0
            if self.rng.random() >= 0.75:
                inward_sign *= -1.0
            parameters.update(
                {
                    "side": side,
                    "amplitude_px": inward_sign
                    * self.rng.uniform(
                        self.white_bend_min_px,
                        self.white_bend_max_px,
                    ),
                    "exponent": self.rng.uniform(1.5, 2.2),
                }
            )
        elif kind == "yellow_dropout":
            start = self.rng.uniform(0.0, 0.45)
            parameters.update(
                {
                    "start_row_ratio": start,
                    "end_row_ratio": self.rng.uniform(max(0.55, start), 1.0),
                }
            )
        else:
            start = self.rng.uniform(0.0, 0.40)
            parameters.update(
                {
                    "side": self.rng.choice(("left", "right")),
                    "start_row_ratio": start,
                    "end_row_ratio": self.rng.uniform(max(0.60, start), 1.0),
                }
            )
        return ArtifactEvent(kind, duration, parameters)

    def start_event(
        self,
        kind: str,
        duration_frames: int,
        **parameters: float | str,
    ) -> None:
        if kind not in self.EVENT_KINDS:
            raise ValueError(f"unknown canonical artifact: {kind}")
        if duration_frames <= 0:
            raise ValueError("duration_frames must be positive")
        self.current_event = ArtifactEvent(kind, duration_frames, dict(parameters))
        self.last_started_event = self.current_event

    def _apply_event(self, image: np.ndarray, event: ArtifactEvent) -> np.ndarray:
        values = event.parameters
        if event.kind == "center_jump":
            return apply_center_jump(
                image,
                float(values["offset_px"]),
                self.background_gray,
            )
        if event.kind == "white_bend":
            return apply_white_bend(
                image,
                str(values["side"]),
                float(values["amplitude_px"]),
                float(values["exponent"]),
                self.background_gray,
            )
        if event.kind == "yellow_dropout":
            return apply_line_dropout(
                image,
                "yellow",
                start_row_ratio=float(values["start_row_ratio"]),
                end_row_ratio=float(values["end_row_ratio"]),
                background_gray=self.background_gray,
            )
        return apply_line_dropout(
            image,
            "white",
            side=str(values["side"]),
            start_row_ratio=float(values["start_row_ratio"]),
            end_row_ratio=float(values["end_row_ratio"]),
            background_gray=self.background_gray,
        )

    def process(self, image: np.ndarray) -> Tuple[np.ndarray, Optional[ArtifactEvent], bool]:
        started = False
        if self.current_event is None and self.rng.random() < self.event_start_probability:
            self.current_event = self._new_event()
            self.last_started_event = self.current_event
            started = True
        event = self.current_event
        if event is None:
            return image.copy(), None, False
        output = self._apply_event(image, event)
        event.remaining_frames -= 1
        if event.remaining_frames <= 0:
            self.current_event = None
        return output, event, started
