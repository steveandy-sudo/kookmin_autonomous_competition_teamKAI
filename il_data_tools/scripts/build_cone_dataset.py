#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from il_data_tools.dataset_builder_common import (
    COMMON_OUTPUT_COLUMNS,
    add_common_args,
    finalize_report,
    load_samples,
    parse_config,
    split_by_session,
    write_report,
    write_splits,
)


INCLUDE_LABELS = [
    "cone_drive",
    "recovery",
]

OUTPUT_COLUMNS = COMMON_OUTPUT_COLUMNS + ["scan_npz_path"]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build processed CSV files for the steering-only cone policy."
    )
    add_common_args(parser)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    config = parse_config(args, INCLUDE_LABELS)

    samples, report = load_samples(config)
    splits = split_by_session(samples, config.val_ratio, config.test_ratio, config.seed)
    output_files = write_splits(config.output_dir, splits, OUTPUT_COLUMNS)

    final_report = finalize_report(
        report,
        config.output_dir,
        splits,
        output_files,
        extra={
            "policy": "cone",
            "output_columns": OUTPUT_COLUMNS,
            "note": "scan_npz_path is preserved for future LiDAR-aware models.",
        },
    )
    write_report(config.output_dir / "dataset_report.json", final_report)
    print(f"wrote processed cone dataset: {config.output_dir}")


if __name__ == "__main__":
    main()
