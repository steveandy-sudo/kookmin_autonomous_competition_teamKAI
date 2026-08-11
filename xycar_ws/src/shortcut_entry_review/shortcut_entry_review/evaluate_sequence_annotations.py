#!/usr/bin/env python3
"""Replay saved W1/W2/Y1/Y2 supervision through the sequence selector.

The evaluator uses the filename order only to reproduce the observed approach.
Bag offsets and ROS timestamps are copied into the report for traceability but
are never passed to :class:`SequenceAwareEntrySelector`.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .sequence_entry_core import (
    EntrySequencePhase,
    LineHypothesis,
    SequenceAwareEntrySelector,
)


def _line_map(annotation: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(item["label"]): item for item in annotation["lines"]}


def _mean_x(item: dict[str, Any]) -> float | None:
    if not bool(item.get("detected", False)):
        return None
    return float(item["mean_lateral_ratio"])


def _observed_span_line_mae(
    item: dict[str, Any],
    selected: LineHypothesis | None,
    *,
    sample_count: int = 41,
) -> float | None:
    """Compare x=f(y) over only the manually observed part of a line."""
    if selected is None or not bool(item.get("detected", False)):
        return None
    points = np.asarray(item["points_normalized"], dtype=np.float64)
    coefficients = np.asarray(
        item["fit_x_from_y_coefficients"], dtype=np.float64
    )
    rows = np.linspace(
        float(np.min(points[:, 1])),
        float(np.max(points[:, 1])),
        int(sample_count),
    )
    expected_x = np.polyval(coefficients, rows)
    selected_x = np.asarray(
        [selected.x_ratio_at(float(row)) for row in rows],
        dtype=np.float64,
    )
    return float(np.mean(np.abs(selected_x - expected_x)))


def evaluate(
    annotation_dir: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    annotation_paths = sorted(annotation_dir.glob("shortcut_lines_*.json"))
    if not annotation_paths:
        raise FileNotFoundError(
            f"no shortcut_lines_*.json in {annotation_dir}"
        )

    selector = SequenceAwareEntrySelector()
    rows: list[dict[str, Any]] = []
    w1_errors: list[float] = []
    y1_errors: list[float] = []
    w1_line_errors: list[float] = []
    y1_line_errors: list[float] = []
    w2_choices = 0
    y2_choices = 0
    y1_false_positives = 0

    for sequence_index, json_path in enumerate(annotation_paths, start=1):
        stem = json_path.stem
        white_path = annotation_dir / f"{stem}_white.png"
        yellow_path = annotation_dir / f"{stem}_yellow.png"
        white = cv2.imread(str(white_path), cv2.IMREAD_GRAYSCALE)
        yellow = cv2.imread(str(yellow_path), cv2.IMREAD_GRAYSCALE)
        if white is None or yellow is None:
            raise FileNotFoundError(
                f"missing saved masks for {json_path.name}: "
                f"{white_path.name}, {yellow_path.name}"
            )
        annotation = json.loads(json_path.read_text(encoding="utf-8"))
        lines = _line_map(annotation)
        result = selector.process(white, yellow)

        gt_w1 = _mean_x(lines["W1"])
        gt_w2 = _mean_x(lines["W2"])
        gt_y1 = _mean_x(lines["Y1"])
        gt_y2 = _mean_x(lines["Y2"])
        selected_w1 = result.w1.mean_x_ratio if result.w1 is not None else None
        selected_y1 = result.y1.mean_x_ratio if result.y1 is not None else None
        w1_error = (
            abs(float(selected_w1) - float(gt_w1))
            if selected_w1 is not None and gt_w1 is not None
            else None
        )
        y1_error = (
            abs(float(selected_y1) - float(gt_y1))
            if selected_y1 is not None and gt_y1 is not None
            else None
        )
        w1_line_error = _observed_span_line_mae(lines["W1"], result.w1)
        y1_line_error = _observed_span_line_mae(lines["Y1"], result.y1)
        if w1_error is not None:
            w1_errors.append(w1_error)
        if y1_error is not None:
            y1_errors.append(y1_error)
        if w1_line_error is not None:
            w1_line_errors.append(w1_line_error)
        if y1_line_error is not None:
            y1_line_errors.append(y1_line_error)
        if (
            selected_w1 is not None
            and gt_w1 is not None
            and gt_w2 is not None
            and abs(float(selected_w1) - gt_w2)
            < abs(float(selected_w1) - gt_w1)
        ):
            w2_choices += 1
        if (
            selected_y1 is not None
            and gt_y1 is not None
            and gt_y2 is not None
            and abs(float(selected_y1) - gt_y2)
            < abs(float(selected_y1) - gt_y1)
        ):
            y2_choices += 1
        if selected_y1 is not None and gt_y1 is None:
            y1_false_positives += 1

        rows.append(
            {
                "sequence_index": sequence_index,
                "sample": stem.removeprefix("shortcut_lines_"),
                "bag_offset_sec_trace_only": float(
                    annotation["bag_offset_sec"]
                ),
                "timestamp_ns_trace_only": int(annotation["timestamp_ns"]),
                "phase": result.phase.name,
                "ready": int(result.ready),
                "path_valid": int(result.path_valid),
                "synthetic_y1": int(result.used_synthetic_y1),
                "gt_w1_mean_x_ratio": gt_w1,
                "selected_w1_mean_x_ratio": selected_w1,
                "w1_abs_error_ratio": w1_error,
                "w1_observed_span_line_mae_ratio": w1_line_error,
                "gt_y1_mean_x_ratio": gt_y1,
                "selected_y1_mean_x_ratio": selected_y1,
                "y1_abs_error_ratio": y1_error,
                "y1_observed_span_line_mae_ratio": y1_line_error,
                "white_candidate_count": len(result.white_candidates),
                "yellow_candidate_count": len(result.yellow_candidates),
                "status": result.reason,
            }
        )

    def first_index(phase: EntrySequencePhase) -> int | None:
        return next(
            (
                int(row["sequence_index"])
                for row in rows
                if row["phase"] == phase.name
            ),
            None,
        )

    summary: dict[str, Any] = {
        "annotation_dir": str(annotation_dir.resolve()),
        "frame_count": len(rows),
        "runtime_uses_bag_offset_or_timestamp": False,
        "ordering_basis": "saved observation order only",
        "ground_truth": {
            "w1_present_frames": sum(
                _mean_x(
                    _line_map(
                        json.loads(path.read_text(encoding="utf-8"))
                    )["W1"]
                )
                is not None
                for path in annotation_paths
            ),
            "y1_present_frames": sum(
                _mean_x(
                    _line_map(
                        json.loads(path.read_text(encoding="utf-8"))
                    )["Y1"]
                )
                is not None
                for path in annotation_paths
            ),
        },
        "selection": {
            "w1_selected_frames": sum(
                row["selected_w1_mean_x_ratio"] is not None for row in rows
            ),
            "y1_selected_frames": sum(
                row["selected_y1_mean_x_ratio"] is not None for row in rows
            ),
            "valid_path_frames": sum(int(row["path_valid"]) for row in rows),
            "w1_mean_x_mae_ratio": float(np.mean(w1_errors)),
            "w1_mean_x_max_error_ratio": float(np.max(w1_errors)),
            "y1_mean_x_mae_ratio": float(np.mean(y1_errors)),
            "y1_mean_x_max_error_ratio": float(np.max(y1_errors)),
            "w1_observed_span_line_mae_ratio": float(
                np.mean(w1_line_errors)
            ),
            "w1_observed_span_line_median_error_ratio": float(
                np.median(w1_line_errors)
            ),
            "w1_observed_span_line_p90_error_ratio": float(
                np.quantile(w1_line_errors, 0.90)
            ),
            "w1_observed_span_line_max_error_ratio": float(
                np.max(w1_line_errors)
            ),
            "y1_observed_span_line_mae_ratio": float(
                np.mean(y1_line_errors)
            ),
            "y1_observed_span_line_median_error_ratio": float(
                np.median(y1_line_errors)
            ),
            "y1_observed_span_line_p90_error_ratio": float(
                np.quantile(y1_line_errors, 0.90)
            ),
            "y1_observed_span_line_max_error_ratio": float(
                np.max(y1_line_errors)
            ),
            "w2_selected_instead_of_w1_frames": w2_choices,
            "y2_selected_instead_of_y1_frames": y2_choices,
            "y1_false_positive_before_annotated_appearance_frames": (
                y1_false_positives
            ),
        },
        "first_phase_sequence_index": {
            phase.name: first_index(phase) for phase in EntrySequencePhase
        },
    }
    return summary, rows


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Validate W1/Y1 sequence selection against saved annotations"
        )
    )
    parser.add_argument("annotation_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    summary, rows = evaluate(args.annotation_dir)
    if args.output_dir is not None:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        with (args.output_dir / "frame_results.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=list(rows[0]),
                lineterminator="\n",
            )
            writer.writeheader()
            writer.writerows(rows)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
