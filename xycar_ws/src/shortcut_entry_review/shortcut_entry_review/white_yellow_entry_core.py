"""Select a shortcut-entry corridor bounded by white-left and yellow-right."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class EntryLaneConfig:
    window_count: int = 18
    white_margin_px: int = 70
    yellow_margin_px: int = 85
    minimum_pixels_per_window: int = 6
    minimum_centers: int = 4
    minimum_span_ratio: float = 0.18
    yellow_minimum_centers: int = 2
    yellow_minimum_span_ratio: float = 0.03
    maximum_fit_extrapolation_ratio: float = 0.14
    minimum_separation_ratio: float = 0.07
    maximum_separation_ratio: float = 0.48
    peak_threshold_ratio: float = 0.20
    peak_cluster_gap_px: int = 18
    polynomial_degree: int = 2
    maximum_fit_rmse_px: float = 42.0
    sample_count: int = 32

    def __post_init__(self) -> None:
        if int(self.window_count) < 3:
            raise ValueError("window_count must be at least 3")
        if int(self.minimum_pixels_per_window) < 1:
            raise ValueError("minimum_pixels_per_window must be positive")
        if int(self.minimum_centers) < 2:
            raise ValueError("minimum_centers must be at least 2")
        if int(self.yellow_minimum_centers) < 2:
            raise ValueError("yellow_minimum_centers must be at least 2")
        if not 0.0 < float(self.minimum_span_ratio) <= 1.0:
            raise ValueError("minimum_span_ratio must be in (0, 1]")
        if not 0.0 < float(self.yellow_minimum_span_ratio) <= 1.0:
            raise ValueError("yellow_minimum_span_ratio must be in (0, 1]")
        if not 0.0 <= float(self.maximum_fit_extrapolation_ratio) <= 0.5:
            raise ValueError("maximum_fit_extrapolation_ratio must be in [0, 0.5]")
        if not 0.0 < float(self.minimum_separation_ratio):
            raise ValueError("minimum_separation_ratio must be positive")
        if float(self.maximum_separation_ratio) <= float(
            self.minimum_separation_ratio
        ):
            raise ValueError("maximum separation must exceed minimum separation")


@dataclass(frozen=True)
class BoundaryFit:
    valid: bool
    base_x: float | None
    centers: tuple[tuple[float, float], ...]
    coefficients: tuple[float, ...]
    rmse_px: float
    span_px: float

    def x_at(self, rows_y: np.ndarray) -> np.ndarray:
        if not self.valid or not self.coefficients:
            return np.full(rows_y.shape, np.nan, dtype=np.float32)
        return np.polyval(np.asarray(self.coefficients), rows_y)


@dataclass(frozen=True)
class EntryLaneResult:
    valid: bool
    reason: str
    white: BoundaryFit
    yellow: BoundaryFit
    path_pixels: tuple[tuple[float, float], ...]
    median_separation_px: float
    target_lateral_px: float


def normalize_mask(mask: np.ndarray) -> np.ndarray:
    if mask is None or mask.ndim != 2:
        raise ValueError("lane mask must be a non-empty mono image")
    return np.where(mask > 0, 255, 0).astype(np.uint8)


def histogram_peaks(mask: np.ndarray, config: EntryLaneConfig) -> list[int]:
    height = mask.shape[0]
    histogram = np.count_nonzero(mask[height * 55 // 100 :, :], axis=0)
    maximum = int(histogram.max(initial=0))
    if maximum <= 0:
        return []
    threshold = max(2, int(round(maximum * config.peak_threshold_ratio)))
    indices = np.flatnonzero(histogram >= threshold)
    if indices.size == 0:
        return []
    clusters: list[list[int]] = [[int(indices[0])]]
    for value in indices[1:]:
        if int(value) - clusters[-1][-1] <= int(config.peak_cluster_gap_px):
            clusters[-1].append(int(value))
        else:
            clusters.append([int(value)])
    return [int(round(float(np.mean(cluster)))) for cluster in clusters]


def choose_entry_bases(
    white_mask: np.ndarray,
    yellow_mask: np.ndarray,
    config: EntryLaneConfig,
    *,
    previous_white_base: float | None = None,
    previous_yellow_base: float | None = None,
) -> tuple[int | None, int | None]:
    width = white_mask.shape[1]
    white_peaks = histogram_peaks(white_mask, config)
    yellow_peaks = histogram_peaks(yellow_mask, config)
    if not white_peaks or not yellow_peaks:
        return None, None

    center = width * 0.5
    if previous_yellow_base is None:
        yellow = min(yellow_peaks, key=lambda value: abs(value - center))
    else:
        yellow = min(
            yellow_peaks,
            key=lambda value: (
                0.8 * abs(value - previous_yellow_base)
                + 0.2 * abs(value - center)
            ),
        )
    minimum_gap = width * float(config.minimum_separation_ratio)
    maximum_gap = width * float(config.maximum_separation_ratio)
    candidates = [
        value
        for value in white_peaks
        if minimum_gap <= yellow - value <= maximum_gap
    ]
    if not candidates:
        return None, int(yellow)
    if previous_white_base is None:
        white = max(candidates)
    else:
        white = min(candidates, key=lambda value: abs(value - previous_white_base))
    return int(white), int(yellow)


def sliding_window_centers(
    mask: np.ndarray,
    base_x: int,
    *,
    window_count: int,
    margin_px: int,
    minimum_pixels: int,
) -> tuple[tuple[float, float], ...]:
    height, width = mask.shape
    nonzero_y, nonzero_x = np.nonzero(mask)
    current_x = float(base_x)
    centers: list[tuple[float, float]] = []
    window_height = max(1, height // int(window_count))
    for index in range(int(window_count)):
        bottom = height - index * window_height
        top = max(0, height - (index + 1) * window_height)
        selected = (
            (nonzero_y >= top)
            & (nonzero_y < bottom)
            & (nonzero_x >= current_x - margin_px)
            & (nonzero_x <= current_x + margin_px)
        )
        if int(np.count_nonzero(selected)) < int(minimum_pixels):
            continue
        center_x = float(np.mean(nonzero_x[selected]))
        center_y = float(np.mean(nonzero_y[selected]))
        current_x = float(np.clip(center_x, 0.0, width - 1.0))
        centers.append((current_x, center_y))
    return tuple(centers)


def fit_boundary(
    centers: tuple[tuple[float, float], ...],
    *,
    base_x: float | None,
    image_height: int,
    config: EntryLaneConfig,
    minimum_centers: int | None = None,
    minimum_span_ratio: float | None = None,
) -> BoundaryFit:
    required_centers = int(
        config.minimum_centers if minimum_centers is None else minimum_centers
    )
    required_span_ratio = float(
        config.minimum_span_ratio
        if minimum_span_ratio is None
        else minimum_span_ratio
    )
    if len(centers) < required_centers:
        return BoundaryFit(False, base_x, centers, (), float("inf"), 0.0)
    points = np.asarray(centers, dtype=np.float64)
    xs = points[:, 0]
    ys = points[:, 1]
    span = float(ys.max() - ys.min())
    if span < image_height * required_span_ratio:
        return BoundaryFit(False, base_x, centers, (), float("inf"), span)
    degree = min(int(config.polynomial_degree), len(centers) - 1)
    coefficients = np.polyfit(ys, xs, degree)
    residuals = xs - np.polyval(coefficients, ys)
    rmse = float(np.sqrt(np.mean(np.square(residuals))))
    valid = bool(np.isfinite(rmse) and rmse <= config.maximum_fit_rmse_px)
    return BoundaryFit(
        valid,
        base_x,
        centers,
        tuple(float(value) for value in coefficients),
        rmse,
        span,
    )


class WhiteYellowEntrySelector:
    """Track the nearest white boundary left of the yellow shortcut boundary."""

    def __init__(self, config: EntryLaneConfig | None = None) -> None:
        self.config = config or EntryLaneConfig()
        self.reset()

    def reset(self) -> None:
        self.previous_white_base: float | None = None
        self.previous_yellow_base: float | None = None

    def process(
        self,
        white_mask: np.ndarray,
        yellow_mask: np.ndarray,
    ) -> EntryLaneResult:
        white = normalize_mask(white_mask)
        yellow = normalize_mask(yellow_mask)
        if white.shape != yellow.shape:
            raise ValueError("white and yellow masks must have the same shape")
        height, width = white.shape
        white_base, yellow_base = choose_entry_bases(
            white,
            yellow,
            self.config,
            previous_white_base=self.previous_white_base,
            previous_yellow_base=self.previous_yellow_base,
        )
        empty = BoundaryFit(False, None, (), (), float("inf"), 0.0)
        if yellow_base is None:
            return EntryLaneResult(
                False, "yellow boundary missing", empty, empty, (), 0.0, 0.0
            )
        if white_base is None:
            return EntryLaneResult(
                False,
                "white boundary left of yellow missing",
                empty,
                BoundaryFit(False, yellow_base, (), (), float("inf"), 0.0),
                (),
                0.0,
                0.0,
            )

        white_centers = sliding_window_centers(
            white,
            white_base,
            window_count=self.config.window_count,
            margin_px=self.config.white_margin_px,
            minimum_pixels=self.config.minimum_pixels_per_window,
        )
        yellow_centers = sliding_window_centers(
            yellow,
            yellow_base,
            window_count=self.config.window_count,
            margin_px=self.config.yellow_margin_px,
            minimum_pixels=self.config.minimum_pixels_per_window,
        )
        white_fit = fit_boundary(
            white_centers,
            base_x=white_base,
            image_height=height,
            config=self.config,
        )
        yellow_fit = fit_boundary(
            yellow_centers,
            base_x=yellow_base,
            image_height=height,
            config=self.config,
            minimum_centers=self.config.yellow_minimum_centers,
            minimum_span_ratio=self.config.yellow_minimum_span_ratio,
        )
        if not white_fit.valid:
            return EntryLaneResult(
                False, "white boundary fit invalid", white_fit, yellow_fit, (), 0.0, 0.0
            )
        if not yellow_fit.valid:
            return EntryLaneResult(
                False, "yellow boundary fit invalid", white_fit, yellow_fit, (), 0.0, 0.0
            )

        white_rows = np.asarray([point[1] for point in white_fit.centers])
        yellow_rows = np.asarray([point[1] for point in yellow_fit.centers])
        # The competition-course yellow boundary is dashed. A detected dash can
        # provide a sound local line fit without overlapping the full white-line
        # support, so extend both fits only by a tightly bounded distance before
        # looking for their common control interval.
        extension = height * float(self.config.maximum_fit_extrapolation_ratio)
        top = max(
            0.0,
            float(white_rows.min()) - extension,
            float(yellow_rows.min()) - extension,
        )
        bottom = min(
            float(height - 1),
            float(white_rows.max()) + extension,
            float(yellow_rows.max()) + extension,
        )
        minimum_span = height * float(self.config.minimum_span_ratio)
        if bottom - top < minimum_span:
            return EntryLaneResult(
                False,
                "white/yellow common span too short",
                white_fit,
                yellow_fit,
                (),
                0.0,
                0.0,
            )
        rows = np.linspace(bottom, top, int(self.config.sample_count))
        white_x = white_fit.x_at(rows)
        yellow_x = yellow_fit.x_at(rows)
        separations = yellow_x - white_x
        minimum_gap = width * float(self.config.minimum_separation_ratio)
        maximum_gap = width * float(self.config.maximum_separation_ratio)
        valid_width = (separations >= minimum_gap) & (separations <= maximum_gap)
        if float(np.mean(valid_width)) < 0.80:
            return EntryLaneResult(
                False,
                "white/yellow ordering or width invalid",
                white_fit,
                yellow_fit,
                (),
                float(np.median(separations)),
                0.0,
            )
        midpoint_x = 0.5 * (white_x + yellow_x)
        path = tuple(
            (float(x), float(y)) for x, y in zip(midpoint_x, rows)
        )
        target_index = min(len(path) - 1, max(0, len(path) // 3))
        target_lateral = float(path[target_index][0] - width * 0.5)
        self.previous_white_base = float(white_base)
        self.previous_yellow_base = float(yellow_base)
        return EntryLaneResult(
            True,
            "white-left/yellow-right entry corridor",
            white_fit,
            yellow_fit,
            path,
            float(np.median(separations)),
            target_lateral,
        )


def render_entry_debug(
    white_mask: np.ndarray,
    yellow_mask: np.ndarray,
    result: EntryLaneResult,
) -> np.ndarray:
    white = normalize_mask(white_mask)
    yellow = normalize_mask(yellow_mask)
    output = np.full((white.shape[0], white.shape[1], 3), 28, dtype=np.uint8)
    output[white > 0] = (255, 255, 255)
    output[yellow > 0] = (0, 220, 255)
    for fit, color in ((result.white, (255, 120, 30)), (result.yellow, (0, 120, 255))):
        for x, y in fit.centers:
            cv2.circle(output, (int(round(x)), int(round(y))), 5, color, -1)
    for x, y in result.path_pixels:
        cv2.circle(output, (int(round(x)), int(round(y))), 4, (255, 0, 255), -1)
    status_color = (50, 220, 80) if result.valid else (40, 40, 255)
    cv2.putText(
        output,
        ("VALID " if result.valid else "INVALID ") + result.reason,
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        status_color,
        2,
        cv2.LINE_AA,
    )
    if result.valid:
        cv2.putText(
            output,
            (
                f"W-left / Y-right width={result.median_separation_px:.1f}px "
                f"target_dx={result.target_lateral_px:+.1f}px"
            ),
            (12, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
    return output
