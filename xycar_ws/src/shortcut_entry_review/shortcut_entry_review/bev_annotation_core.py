"""Pure state and calibration-tolerant descriptors for BEV line clicks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


COLORS = ("white", "yellow")


@dataclass(frozen=True)
class AnnotatedLine:
    label: str
    color: str
    points_px: tuple[tuple[float, float], ...]


class BevAnnotationSession:
    """Collect at most two ordered polylines for each semantic color."""

    def __init__(self, *, lines_per_color: int = 2) -> None:
        if int(lines_per_color) < 1:
            raise ValueError("lines_per_color must be positive")
        self.lines_per_color = int(lines_per_color)
        self.reset()

    def reset(self) -> None:
        self.lines: dict[str, list[AnnotatedLine]] = {
            color: [] for color in COLORS
        }
        self.current_color: str | None = None
        self.current_points: list[tuple[float, float]] = []

    def begin(self, color: str) -> str:
        normalized = str(color).strip().lower()
        if normalized not in COLORS:
            raise ValueError(f"unsupported annotation color: {color}")
        if self.current_color is not None:
            self.finish(allow_empty=True)
        if len(self.lines[normalized]) >= self.lines_per_color:
            raise ValueError(
                f"{normalized} already has {self.lines_per_color} lines"
            )
        self.current_color = normalized
        self.current_points = []
        return self.current_label

    @property
    def current_label(self) -> str:
        if self.current_color is None:
            return ""
        prefix = "W" if self.current_color == "white" else "Y"
        return f"{prefix}{len(self.lines[self.current_color]) + 1}"

    def add_point(self, x_px: float, y_px: float) -> str:
        if self.current_color is None:
            raise ValueError("press w or y before clicking a point")
        self.current_points.append((float(x_px), float(y_px)))
        return self.current_label

    def undo(self) -> bool:
        if not self.current_points:
            return False
        self.current_points.pop()
        return True

    def finish(self, *, allow_empty: bool = False) -> AnnotatedLine:
        if self.current_color is None:
            raise ValueError("no line is being annotated")
        if not self.current_points and not allow_empty:
            raise ValueError("a line needs at least two published points")
        if len(self.current_points) == 1:
            raise ValueError("a line needs at least two published points")
        line = AnnotatedLine(
            label=self.current_label,
            color=self.current_color,
            points_px=tuple(self.current_points),
        )
        self.lines[self.current_color].append(line)
        self.current_color = None
        self.current_points = []
        return line

    def completed(self) -> bool:
        return all(
            len(self.lines[color]) == self.lines_per_color for color in COLORS
        ) and self.current_color is None

    def all_lines(self, *, include_current: bool = True) -> list[AnnotatedLine]:
        output = [*self.lines["white"], *self.lines["yellow"]]
        if include_current and self.current_color and self.current_points:
            output.append(
                AnnotatedLine(
                    label=self.current_label,
                    color=self.current_color,
                    points_px=tuple(self.current_points),
                )
            )
        return output

    def to_document(
        self,
        *,
        image_width: int,
        image_height: int,
        bag_offset_sec: float | None,
        timestamp_ns: int,
    ) -> dict:
        if not self.completed():
            raise ValueError("W1, W2, Y1 and Y2 must all be completed")
        width = float(image_width)
        height = float(image_height)
        records = []
        for color in COLORS:
            detected_lines = [
                line for line in self.lines[color] if line.points_px
            ]
            ordered = sorted(
                detected_lines,
                key=lambda line: float(
                    np.mean([point[0] for point in line.points_px])
                ),
            )
            ranks = {line.label: rank for rank, line in enumerate(ordered)}
            for line in self.lines[color]:
                if not line.points_px:
                    records.append(
                        {
                            "label": line.label,
                            "color": line.color,
                            "detected": False,
                            "left_to_right_rank_within_color": None,
                            "points_px": [],
                            "points_normalized": [],
                            "fit_x_from_y_coefficients": None,
                            "direction_dx_dy": None,
                            "curvature": None,
                            "mean_lateral_ratio": None,
                            "observed_y_span_ratio": 0.0,
                        }
                    )
                    continue
                normalized = np.asarray(
                    [
                        (point[0] / width, point[1] / height)
                        for point in line.points_px
                    ],
                    dtype=np.float64,
                )
                y_values = normalized[:, 1]
                x_values = normalized[:, 0]
                degree = min(2, len(line.points_px) - 1)
                coefficients = np.polyfit(y_values, x_values, degree)
                median_y = float(np.median(y_values))
                derivative = np.polyder(coefficients)
                direction_dx_dy = float(np.polyval(derivative, median_y))
                curvature = (
                    float(2.0 * coefficients[0]) if degree == 2 else 0.0
                )
                records.append(
                    {
                        "label": line.label,
                        "color": line.color,
                        "detected": True,
                        "left_to_right_rank_within_color": ranks[line.label],
                        "points_px": [list(point) for point in line.points_px],
                        "points_normalized": normalized.tolist(),
                        "fit_x_from_y_coefficients": coefficients.tolist(),
                        "direction_dx_dy": direction_dx_dy,
                        "curvature": curvature,
                        "mean_lateral_ratio": float(np.mean(x_values)),
                        "observed_y_span_ratio": float(
                            y_values.max() - y_values.min()
                        ),
                    }
                )
        return {
            "schema_version": 1,
            "contract": "two white lines and two yellow lines in one BEV frame",
            "image_width": int(image_width),
            "image_height": int(image_height),
            "bag_offset_sec": bag_offset_sec,
            "timestamp_ns": int(timestamp_ns),
            "matching_policy": (
                "match color, left-to-right rank, direction, and curvature; "
                "do not reuse absolute pixels across PCs"
            ),
            "lines": records,
            "selected_white": None,
            "selected_yellow": None,
        }
