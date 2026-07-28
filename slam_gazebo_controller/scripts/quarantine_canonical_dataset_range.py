#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path


def atomic_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", newline="", encoding="utf-8", dir=path.parent, delete=False
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
        temporary = handle.name
    os.replace(temporary, path)


def atomic_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, delete=False
    ) as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False, sort_keys=True)
        handle.write("\n")
        temporary = handle.name
    os.replace(temporary, path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move a timestamp range out of a canonical dataset without "
            "destroying the original images."
        )
    )
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--start-timestamp-ns", type=int, required=True)
    parser.add_argument("--end-timestamp-ns", type=int, required=True)
    parser.add_argument("--reason", default="bad_data")
    parser.add_argument("--source-frame-start", type=int)
    parser.add_argument("--source-frame-end", type=int)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    dataset_dir = args.dataset_dir.expanduser().resolve()
    samples_path = dataset_dir / "samples.csv"
    if args.end_timestamp_ns < args.start_timestamp_ns:
        raise ValueError("end timestamp must be greater than start timestamp")
    if not samples_path.is_file():
        raise FileNotFoundError(samples_path)

    reason = args.reason.strip().replace("/", "_")
    if not reason:
        raise ValueError("reason must not be empty")
    quarantine_dir = dataset_dir / "bad_data" / reason
    quarantine_csv = quarantine_dir / "samples.csv"
    if quarantine_csv.exists():
        raise FileExistsError(
            f"quarantine already exists; refusing to overwrite: {quarantine_dir}"
        )

    with samples_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)
    if "image_timestamp_ns" not in fieldnames or "front_image_path" not in fieldnames:
        raise ValueError("samples.csv lacks canonical image timestamp/path columns")

    kept: list[dict] = []
    quarantined: list[dict] = []
    for row in rows:
        timestamp_ns = int(row["image_timestamp_ns"])
        if args.start_timestamp_ns <= timestamp_ns <= args.end_timestamp_ns:
            quarantined.append(row)
        else:
            kept.append(row)
    if not quarantined:
        raise RuntimeError("the requested range contains no dataset samples")

    print(
        f"dataset={dataset_dir}\n"
        f"keep={len(kept)} quarantine={len(quarantined)} reason={reason}"
    )
    if args.dry_run:
        return 0

    quarantine_image_dir = quarantine_dir / "images" / "front"
    quarantine_image_dir.mkdir(parents=True, exist_ok=False)
    quarantine_rows: list[dict] = []
    for row in quarantined:
        source = dataset_dir / row["front_image_path"]
        if not source.is_file():
            raise FileNotFoundError(source)
        destination = quarantine_image_dir / source.name
        shutil.move(str(source), str(destination))
        quarantined_row = dict(row)
        quarantined_row["front_image_path"] = str(
            destination.relative_to(quarantine_dir)
        )
        quarantine_rows.append(quarantined_row)

    atomic_csv(samples_path, fieldnames, kept)
    atomic_csv(quarantine_csv, fieldnames, quarantine_rows)
    manifest = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "dataset_dir": str(dataset_dir),
        "excluded_samples": len(quarantine_rows),
        "reason": reason,
        "source_frame_end_inclusive": args.source_frame_end,
        "source_frame_start_inclusive": args.source_frame_start,
        "timestamp_end_ns_inclusive": args.end_timestamp_ns,
        "timestamp_start_ns_inclusive": args.start_timestamp_ns,
    }
    atomic_json(quarantine_dir / "manifest.json", manifest)

    metadata_path = dataset_dir / "metadata.json"
    if metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        metadata["active_frames"] = len(kept)
        metadata["quarantined_frames"] = int(
            metadata.get("quarantined_frames", 0)
        ) + len(quarantine_rows)
        metadata.setdefault("quarantined_ranges", []).append(manifest)
        atomic_json(metadata_path, metadata)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
