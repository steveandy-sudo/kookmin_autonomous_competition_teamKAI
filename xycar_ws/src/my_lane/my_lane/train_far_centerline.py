from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import random
import time
from typing import NamedTuple

import cv2
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from torch.utils.data import DataLoader, Dataset

from my_lane.far_centerline_model import (
    FarCenterlineMaskAdapter,
    FarCenterlineModel,
    IMAGENET_MEAN,
    IMAGENET_STD,
    anchor_rows_for_stride,
    expected_x_from_logits,
)
from my_lane.train_row_centerline import (
    IMAGE_EXTENSIONS,
    yellow_polygon_mask,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Train a high-resolution x-bin/no-line far centerline model."
        )
    )
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--width", type=int, default=512)
    parser.add_argument("--height", type=int, default=288)
    parser.add_argument("--x-bins", type=int, default=128)
    parser.add_argument("--yellow-class-id", type=int, default=1)
    parser.add_argument("--white-class-id", type=int, default=0)
    parser.add_argument("--epochs", type=int, default=55)
    parser.add_argument("--batch-size", type=int, default=48)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--lr", type=float, default=8.0e-4)
    parser.add_argument("--weight-decay", type=float, default=1.0e-4)
    parser.add_argument("--patience", type=int, default=12)
    parser.add_argument("--seed", type=int, default=20260808)
    parser.add_argument(
        "--pretrained-backbone",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--white-far-extension",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser.parse_args()


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def mask_row_centers(mask: np.ndarray) -> np.ndarray:
    centers = np.full(mask.shape[0], np.nan, dtype=np.float32)
    for row in np.flatnonzero(np.any(mask > 0, axis=1)):
        centers[row] = float(np.median(np.flatnonzero(mask[row] > 0)))
    return centers


def white_midpoints(mask: np.ndarray) -> np.ndarray:
    height, width = mask.shape
    midpoints = np.full(height, np.nan, dtype=np.float32)
    minimum_separation = max(12.0, 0.04 * float(width))
    for row in np.flatnonzero(np.any(mask > 0, axis=1)):
        xs = np.flatnonzero(mask[row] > 0)
        groups = np.split(xs, np.flatnonzero(np.diff(xs) > 2) + 1)
        centers = [float(np.median(group)) for group in groups if group.size]
        if len(centers) >= 2 and centers[-1] - centers[0] >= minimum_separation:
            midpoints[row] = 0.5 * (centers[0] + centers[-1])
    return midpoints


def far_centerline_targets(
    yellow_mask: np.ndarray,
    white_mask: np.ndarray,
    anchor_rows: tuple[int, ...],
    x_bin_count: int,
    *,
    white_extension: bool = True,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return x/no-line targets, confidence weights, and source roles.

    Source role 0 is no line, 1 is yellow/interpolated yellow, and 2 is a
    white-boundary midpoint extension validated against visible yellow.
    """
    height, width = yellow_mask.shape
    none_class = int(x_bin_count)
    targets = np.full(len(anchor_rows), none_class, dtype=np.int64)
    quality = np.full(len(anchor_rows), 0.20, dtype=np.float32)
    source = np.zeros(len(anchor_rows), dtype=np.int64)
    yellow = mask_row_centers(yellow_mask)
    observed = np.flatnonzero(np.isfinite(yellow))
    if observed.size == 0:
        return targets, quality, source

    path = np.full(height, np.nan, dtype=np.float32)
    path_source = np.zeros(height, dtype=np.int64)
    inside_rows = np.arange(observed[0], observed[-1] + 1)
    path[inside_rows] = np.interp(
        inside_rows.astype(np.float32),
        observed.astype(np.float32),
        yellow[observed],
    )
    path_source[inside_rows] = 1

    if white_extension:
        midpoints = white_midpoints(white_mask)
        overlap = observed[np.isfinite(midpoints[observed])]
        if overlap.size >= 8:
            bias_samples = yellow[overlap] - midpoints[overlap]
            bias = float(np.median(bias_samples))
            residual = np.abs(bias_samples - bias)
            if float(np.percentile(residual, 90)) <= 10.0:
                valid_white = np.flatnonzero(
                    np.isfinite(midpoints) & (np.arange(height) < observed[0])
                )
                if valid_white.size:
                    # Keep the continuous white support nearest to yellow.
                    selected = []
                    previous = int(observed[0])
                    for row in valid_white[::-1]:
                        if previous - int(row) > 3:
                            break
                        selected.append(int(row))
                        previous = int(row)
                    if selected:
                        extension_rows = np.arange(
                            min(selected), int(observed[0]), dtype=np.int64
                        )
                        known = np.asarray(selected, dtype=np.int64)
                        path[extension_rows] = np.interp(
                            extension_rows.astype(np.float32),
                            known[::-1].astype(np.float32),
                            (midpoints[known[::-1]] + bias).astype(np.float32),
                        )
                        path_source[extension_rows] = 2

    anchors = np.asarray(anchor_rows, dtype=np.int64)
    valid = np.isfinite(path[anchors])
    normalized = path[anchors[valid]] / max(1.0, float(width - 1))
    targets[valid] = np.clip(
        np.rint(normalized * float(x_bin_count - 1)),
        0,
        x_bin_count - 1,
    ).astype(np.int64)
    source[valid] = path_source[anchors[valid]]
    quality[source == 1] = 1.0
    quality[source == 2] = 0.75
    return targets, quality, source


class FarCenterlineDataset(Dataset):
    def __init__(
        self,
        root: Path,
        split: str,
        *,
        width: int,
        height: int,
        x_bin_count: int,
        yellow_class_id: int,
        white_class_id: int,
        white_extension: bool,
        augment: bool,
    ) -> None:
        self.image_dir = root / split / "images"
        self.label_dir = root / split / "labels"
        self.width = int(width)
        self.height = int(height)
        self.x_bin_count = int(x_bin_count)
        self.yellow_class_id = int(yellow_class_id)
        self.white_class_id = int(white_class_id)
        self.white_extension = bool(white_extension)
        self.augment = bool(augment)
        self.anchor_rows = anchor_rows_for_stride(self.height)
        self.images = sorted(
            path
            for path in self.image_dir.iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        )
        if not self.images:
            raise RuntimeError(f"no images found in {self.image_dir}")

    def __len__(self) -> int:
        return len(self.images)

    def geometric_augment(self, image, yellow, white):
        if random.random() < 0.50:
            image = cv2.flip(image, 1)
            yellow = cv2.flip(yellow, 1)
            white = cv2.flip(white, 1)
        if random.random() < 0.55:
            shift = random.randint(
                -int(round(self.width * 0.07)),
                int(round(self.width * 0.07)),
            )
            transform = np.asarray(
                [[1.0, 0.0, shift], [0.0, 1.0, 0.0]], dtype=np.float32
            )
            image = cv2.warpAffine(
                image,
                transform,
                (self.width, self.height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_REFLECT_101,
            )
            yellow = cv2.warpAffine(
                yellow,
                transform,
                (self.width, self.height),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
            )
            white = cv2.warpAffine(
                white,
                transform,
                (self.width, self.height),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
            )
        return image, yellow, white

    @staticmethod
    def color_augment(image):
        if random.random() < 0.88:
            alpha = random.uniform(0.67, 1.35)
            beta = random.uniform(-30.0, 30.0)
            image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        if random.random() < 0.65:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[..., 0] = np.mod(
                hsv[..., 0] + random.uniform(-5.0, 5.0), 180.0
            )
            hsv[..., 1] *= random.uniform(0.60, 1.40)
            hsv[..., 2] *= random.uniform(0.70, 1.30)
            image = cv2.cvtColor(
                np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR
            )
        if random.random() < 0.20:
            image = cv2.GaussianBlur(image, (3, 3), 0)
        if random.random() < 0.10:
            noise = np.random.normal(0.0, 5.0, image.shape).astype(np.float32)
            image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(
                np.uint8
            )
        return image

    def __getitem__(self, index):
        image_path = self.images[index]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"failed to read {image_path}")
        image = cv2.resize(
            image, (self.width, self.height), interpolation=cv2.INTER_AREA
        )
        label_path = self.label_dir / f"{image_path.stem}.txt"
        yellow = yellow_polygon_mask(
            label_path,
            self.width,
            self.height,
            yellow_class_id=self.yellow_class_id,
        )
        white = yellow_polygon_mask(
            label_path,
            self.width,
            self.height,
            yellow_class_id=self.white_class_id,
        )
        if self.augment:
            image, yellow, white = self.geometric_augment(
                image, yellow, white
            )
            image = self.color_augment(image)
        target, quality, source = far_centerline_targets(
            yellow,
            white,
            self.anchor_rows,
            self.x_bin_count,
            white_extension=self.white_extension,
        )
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        normalized = (
            rgb - np.asarray(IMAGENET_MEAN, dtype=np.float32)
        ) / np.asarray(IMAGENET_STD, dtype=np.float32)
        image_tensor = torch.from_numpy(
            np.ascontiguousarray(normalized.transpose(2, 0, 1))
        )
        return (
            image_tensor,
            torch.from_numpy(target),
            torch.from_numpy(quality),
            torch.from_numpy(source),
        )


class FarCenterlineLoss(nn.Module):
    def __init__(self, anchor_rows, image_height, x_bin_count):
        super().__init__()
        anchors = torch.asarray(anchor_rows, dtype=torch.float32)
        normalized_y = anchors / max(1.0, float(image_height - 1))
        self.register_buffer(
            "row_weights", 1.0 + 1.5 * (1.0 - normalized_y)
        )
        self.x_bin_count = int(x_bin_count)

    def forward(self, logits, target, quality):
        valid = target != self.x_bin_count
        ce = F.cross_entropy(
            logits.transpose(1, 2),
            target,
            reduction="none",
            label_smoothing=0.015,
        )
        weights = quality * self.row_weights
        classification = (ce * weights).sum() / weights.sum().clamp_min(1.0)

        expected_x, _, _ = expected_x_from_logits(
            logits, self.x_bin_count
        )
        target_x = target.clamp_max(self.x_bin_count - 1).to(logits.dtype)
        target_x = target_x / float(self.x_bin_count - 1)
        coordinate_error = F.smooth_l1_loss(
            expected_x, target_x, reduction="none", beta=0.01
        )
        valid_weight = valid * weights
        coordinate = (coordinate_error * valid_weight).sum() / (
            valid_weight.sum().clamp_min(1.0)
        )

        pair_valid = valid[:, 1:] & valid[:, :-1]
        predicted_delta = expected_x[:, 1:] - expected_x[:, :-1]
        target_delta = target_x[:, 1:] - target_x[:, :-1]
        shape_error = F.smooth_l1_loss(
            predicted_delta, target_delta, reduction="none", beta=0.008
        )
        pair_weight = pair_valid * torch.minimum(weights[:, 1:], weights[:, :-1])
        shape = (shape_error * pair_weight).sum() / pair_weight.sum().clamp_min(
            1.0
        )
        return classification + 2.0 * coordinate + 0.50 * shape


class MetricTotals(NamedTuple):
    target_valid: int = 0
    predicted_valid: int = 0
    matched: int = 0
    within_2: int = 0
    within_4: int = 0
    within_8: int = 0
    extended_valid: int = 0
    extended_within_4: int = 0
    absolute_error: float = 0.0
    direction_pairs: int = 0
    direction_flips: int = 0


def add_totals(left, right):
    return MetricTotals(*(a + b for a, b in zip(left, right)))


def batch_totals(logits, target, source, x_bin_count, image_width):
    valid = target != x_bin_count
    expected_x, presence, confidence = expected_x_from_logits(
        logits, x_bin_count
    )
    predicted_valid = (presence >= 0.55) & (confidence >= 0.10)
    predicted_px = expected_x * float(image_width - 1)
    target_px = target.clamp_max(x_bin_count - 1).to(logits.dtype)
    target_px = target_px / float(x_bin_count - 1) * float(image_width - 1)
    error = torch.abs(predicted_px - target_px).float()
    matched = valid & predicted_valid
    extended = valid & (source == 2)

    pair = matched[:, 1:] & matched[:, :-1]
    target_delta = target_px[:, 1:] - target_px[:, :-1]
    prediction_delta = predicted_px[:, 1:] - predicted_px[:, :-1]
    meaningful = pair & (torch.abs(target_delta) >= 1.0)
    flips = meaningful & (torch.sign(target_delta) != torch.sign(prediction_delta))
    return MetricTotals(
        target_valid=int(valid.sum()),
        predicted_valid=int(predicted_valid.sum()),
        matched=int(matched.sum()),
        within_2=int((matched & (error <= 2.0)).sum()),
        within_4=int((matched & (error <= 4.0)).sum()),
        within_8=int((matched & (error <= 8.0)).sum()),
        extended_valid=int(extended.sum()),
        extended_within_4=int(
            (extended & predicted_valid & (error <= 4.0)).sum()
        ),
        absolute_error=float((error * matched).sum()),
        direction_pairs=int(meaningful.sum()),
        direction_flips=int(flips.sum()),
    )


def metrics_from_totals(totals):
    target = max(1, totals.target_valid)
    predicted = max(1, totals.predicted_valid)
    matched = max(1, totals.matched)
    extended = max(1, totals.extended_valid)
    pairs = max(1, totals.direction_pairs)
    return {
        "presence_recall": totals.matched / target,
        "presence_precision": totals.matched / predicted,
        "within_2px_recall": totals.within_2 / target,
        "within_4px_recall": totals.within_4 / target,
        "within_8px_recall": totals.within_8 / target,
        "visible_x_mae_px": totals.absolute_error / matched,
        "white_extended_within_4px_recall": (
            totals.extended_within_4 / extended
        ),
        "direction_flip_rate": totals.direction_flips / pairs,
    }


def run_epoch(
    model,
    loader,
    criterion,
    device,
    *,
    x_bin_count,
    image_width,
    optimizer=None,
    scaler=None,
):
    training = optimizer is not None
    model.train(training)
    total_loss = 0.0
    total_items = 0
    totals = MetricTotals()
    for images, targets, qualities, sources in loader:
        images = images.to(device, non_blocking=True)
        targets = targets.to(device, non_blocking=True)
        qualities = qualities.to(device, non_blocking=True)
        sources = sources.to(device, non_blocking=True)
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training), torch.autocast(
            device_type=device.type, enabled=device.type == "cuda"
        ):
            logits = model(images)
            loss = criterion(logits, targets, qualities)
        if training:
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            scaler.step(optimizer)
            scaler.update()
        batch_size = int(images.shape[0])
        total_loss += float(loss.detach()) * batch_size
        total_items += batch_size
        totals = add_totals(
            totals,
            batch_totals(
                logits.detach(),
                targets,
                sources,
                x_bin_count,
                image_width,
            ),
        )
    return total_loss / max(1, total_items), metrics_from_totals(totals)


def export_models(checkpoint, output_dir, width, height, x_bin_count):
    core = FarCenterlineModel(
        x_bin_count=x_bin_count, pretrained_backbone=False
    )
    core.load_state_dict(checkpoint["model_state"])
    core.eval()
    example = torch.zeros(1, 3, height, width)
    core_script = torch.jit.freeze(torch.jit.trace(core, example, strict=True))
    core_path = output_dir / "far_centerline_core_512x288.pt"
    core_script.save(str(core_path))
    adapter = FarCenterlineMaskAdapter(
        core,
        output_width=width,
        output_height=height,
        x_bin_count=x_bin_count,
    ).eval()
    mask_script = torch.jit.freeze(
        torch.jit.trace(adapter, example, strict=True)
    )
    mask_path = output_dir / "far_centerline_mask_512x288.pt"
    mask_script.save(str(mask_path))
    return core_path, mask_path


def benchmark_cpu(model_path, width, height):
    model = torch.jit.load(str(model_path), map_location="cpu").eval()
    torch.set_num_threads(1)
    sample = torch.zeros(1, 3, height, width)
    with torch.inference_mode():
        for _ in range(15):
            model(sample)
        started = time.perf_counter()
        for _ in range(80):
            model(sample)
    latency_ms = (time.perf_counter() - started) * 1000.0 / 80.0
    return {
        "threads": 1,
        "mean_latency_ms": latency_ms,
        "throughput_fps": 1000.0 / latency_ms,
    }


def main():
    args = parse_args()
    seed_everything(args.seed)
    root = Path(args.dataset_root).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    device = torch.device(
        args.device
        if args.device != "cuda" or torch.cuda.is_available()
        else "cpu"
    )
    anchor_rows = anchor_rows_for_stride(args.height)
    datasets = {
        split: FarCenterlineDataset(
            root,
            split,
            width=args.width,
            height=args.height,
            x_bin_count=args.x_bins,
            yellow_class_id=args.yellow_class_id,
            white_class_id=args.white_class_id,
            white_extension=args.white_far_extension,
            augment=split == "train",
        )
        for split in ("train", "valid", "test")
    }
    loaders = {
        split: DataLoader(
            dataset,
            batch_size=args.batch_size,
            shuffle=split == "train",
            num_workers=args.workers,
            pin_memory=device.type == "cuda",
            persistent_workers=args.workers > 0,
        )
        for split, dataset in datasets.items()
    }
    model = FarCenterlineModel(
        x_bin_count=args.x_bins,
        pretrained_backbone=args.pretrained_backbone,
    ).to(device)
    criterion = FarCenterlineLoss(
        anchor_rows, args.height, args.x_bins
    ).to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.lr, weight_decay=args.weight_decay
    )
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(1, args.epochs), eta_min=args.lr * 0.04
    )
    scaler = torch.amp.GradScaler(device.type, enabled=device.type == "cuda")
    history = []
    best_score = -float("inf")
    best_epoch = -1
    stale = 0
    checkpoint_path = output_dir / "best_checkpoint.pt"

    for epoch in range(1, args.epochs + 1):
        train_loss, train_metrics = run_epoch(
            model,
            loaders["train"],
            criterion,
            device,
            x_bin_count=args.x_bins,
            image_width=args.width,
            optimizer=optimizer,
            scaler=scaler,
        )
        valid_loss, valid_metrics = run_epoch(
            model,
            loaders["valid"],
            criterion,
            device,
            x_bin_count=args.x_bins,
            image_width=args.width,
        )
        score = (
            valid_metrics["within_4px_recall"]
            + 0.20 * valid_metrics["presence_precision"]
            + 0.25 * valid_metrics["white_extended_within_4px_recall"]
            - 0.25 * valid_metrics["direction_flip_rate"]
        )
        row = {
            "epoch": epoch,
            "lr": optimizer.param_groups[0]["lr"],
            "train_loss": train_loss,
            "valid_loss": valid_loss,
            **{f"train_{k}": v for k, v in train_metrics.items()},
            **{f"valid_{k}": v for k, v in valid_metrics.items()},
            "score": score,
        }
        history.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
        if score > best_score:
            best_score = score
            best_epoch = epoch
            stale = 0
            torch.save(
                {
                    "model_state": model.state_dict(),
                    "epoch": epoch,
                    "metrics": valid_metrics,
                    "score": score,
                    "width": args.width,
                    "height": args.height,
                    "x_bin_count": args.x_bins,
                    "anchor_rows": list(anchor_rows),
                    "white_far_extension": args.white_far_extension,
                },
                checkpoint_path,
            )
        else:
            stale += 1
        scheduler.step()
        if stale >= args.patience:
            break

    with (output_dir / "history.csv").open(
        "w", newline="", encoding="utf-8"
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=list(history[0].keys()))
        writer.writeheader()
        writer.writerows(history)
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    model.load_state_dict(checkpoint["model_state"])
    model.to(device)
    test_loss, test_metrics = run_epoch(
        model,
        loaders["test"],
        criterion,
        device,
        x_bin_count=args.x_bins,
        image_width=args.width,
    )
    core_path, mask_path = export_models(
        checkpoint, output_dir, args.width, args.height, args.x_bins
    )
    report = {
        "dataset_root": str(root),
        "images": {key: len(value) for key, value in datasets.items()},
        "input": [args.width, args.height],
        "x_bin_count": args.x_bins,
        "anchor_rows": list(anchor_rows),
        "parameters": sum(p.numel() for p in model.parameters()),
        "best_epoch": best_epoch,
        "test_loss": test_loss,
        "test": test_metrics,
        "white_far_extension": args.white_far_extension,
        "cpu_core": benchmark_cpu(core_path, args.width, args.height),
        "cpu_mask_adapter": benchmark_cpu(mask_path, args.width, args.height),
        "core_model": str(core_path),
        "mask_model": str(mask_path),
    }
    (output_dir / "metrics.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
