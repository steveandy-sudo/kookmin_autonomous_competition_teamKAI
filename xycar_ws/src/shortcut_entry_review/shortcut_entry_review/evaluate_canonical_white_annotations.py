#!/usr/bin/env python3
"""Evaluate W1/W2 selection on the manually fitted canonical masks."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from .sequence_entry_core import LineHypothesis, SequenceAwareEntrySelector


def _manual_line(
    document: dict[str, Any], label: str
) -> tuple[float, float, float, float] | None:
    record = next(
        item for item in document["lines"] if item["label"] == label
    )
    if not bool(record["detected"]):
        return None
    slope, intercept = record["fit"][
        "coefficients_x_ratio_from_y_ratio"
    ]
    points = np.asarray(record["points_normalized"], dtype=np.float64)
    return (
        float(slope),
        float(intercept),
        float(np.min(points[:, 1])),
        float(np.max(points[:, 1])),
    )


def _line_error(
    selected: LineHypothesis | None,
    expected: tuple[float, float, float, float] | None,
) -> float | None:
    if selected is None or expected is None:
        return None
    slope, intercept, minimum_y, maximum_y = expected
    rows = np.linspace(minimum_y, maximum_y, 41)
    expected_x = slope * rows + intercept
    selected_x = np.asarray(
        [selected.x_ratio_at(float(row)) for row in rows]
    )
    return float(np.mean(np.abs(selected_x - expected_x)))


def evaluate(
    annotation_dir: Path,
    *,
    maximum_line_error_ratio: float = 0.04,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    documents = [
        json.loads(path.read_text(encoding="utf-8"))
        for path in annotation_dir.glob("canonical_white_*.json")
    ]
    if not documents:
        raise FileNotFoundError(
            f"no canonical_white_*.json in {annotation_dir}"
        )
    documents.sort(key=lambda item: int(item["sample_number"]))

    selector = SequenceAwareEntrySelector()
    rows: list[dict[str, Any]] = []
    counts = {
        "w1_tp": 0,
        "w1_fp": 0,
        "w1_fn": 0,
        "w2_tp": 0,
        "w2_fp": 0,
        "w2_fn": 0,
    }
    errors: dict[str, list[float]] = {"w1": [], "w2": []}

    for document in documents:
        timestamp_ns = int(document["timestamp_ns"])
        stem = annotation_dir / f"canonical_white_{timestamp_ns}"
        white = cv2.imread(f"{stem}_white.png", cv2.IMREAD_GRAYSCALE)
        yellow = cv2.imread(f"{stem}_yellow.png", cv2.IMREAD_GRAYSCALE)
        if white is None or yellow is None:
            raise FileNotFoundError(f"missing masks for {stem.name}")
        result = selector.process(white, yellow)
        row: dict[str, Any] = {
            "sample_number": int(document["sample_number"]),
            "phase": result.phase.name,
        }
        for label, selected in (("w1", result.w1), ("w2", result.w2)):
            expected = _manual_line(document, label.upper())
            error = _line_error(selected, expected)
            expected_present = expected is not None
            selected_present = selected is not None
            correct = bool(
                expected_present
                and selected_present
                and error is not None
                and error <= maximum_line_error_ratio
            )
            if expected_present:
                counts[f"{label}_{'tp' if correct else 'fn'}"] += 1
            elif selected_present:
                counts[f"{label}_fp"] += 1
            if correct and error is not None:
                errors[label].append(error)
            row[f"{label}_expected"] = int(expected_present)
            row[f"{label}_selected"] = int(selected_present)
            row[f"{label}_line_mae_ratio"] = error
            row[f"{label}_correct"] = int(correct)
        rows.append(row)

    summary: dict[str, Any] = {
        "annotation_dir": str(annotation_dir.resolve()),
        "frame_count": len(documents),
        "runtime_uses_bag_time_or_frame_number": False,
        "sequence_dependency": "previous observed line geometry only",
        "maximum_correct_line_mae_ratio": maximum_line_error_ratio,
        "counts": counts,
        "metrics": {},
    }
    for label in ("w1", "w2"):
        tp = counts[f"{label}_tp"]
        fp = counts[f"{label}_fp"]
        fn = counts[f"{label}_fn"]
        precision = tp / max(1, tp + fp)
        recall = tp / max(1, tp + fn)
        label_errors = errors[label]
        summary["metrics"][label] = {
            "precision": precision,
            "recall": recall,
            "f1": (
                2.0 * precision * recall / max(1e-12, precision + recall)
            ),
            "correct_line_mae_median_ratio": (
                float(np.median(label_errors))
                if label_errors
                else math.nan
            ),
            "correct_line_mae_max_ratio": (
                float(np.max(label_errors)) if label_errors else math.nan
            ),
        }
    return summary, rows


def _write_report(
    output_dir: Path,
    summary: dict[str, Any],
    rows: list[dict[str, Any]],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with (output_dir / "frame_results.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("annotation_dir", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--maximum-line-error-ratio", type=float, default=0.04)
    arguments = parser.parse_args()
    summary, rows = evaluate(
        arguments.annotation_dir,
        maximum_line_error_ratio=arguments.maximum_line_error_ratio,
    )
    if arguments.output_dir is not None:
        _write_report(arguments.output_dir, summary, rows)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
