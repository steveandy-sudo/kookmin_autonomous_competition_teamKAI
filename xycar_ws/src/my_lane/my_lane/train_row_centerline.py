from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import random
from typing import NamedTuple

import cv2
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from torch.utils.data import DataLoader, Dataset

from my_lane.row_centerline_model import (
    IMAGENET_MEAN,
    IMAGENET_STD,
    RowCenterlineMaskAdapter,
    RowCenterlineModel,
    expected_row_coordinates,
    make_anchor_rows,
)


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Train a far-field-weighted yellow row-coordinate model from "
            "Roboflow YOLO segmentation polygons."
        )
    )
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--width", type=int, default=256)
    parser.add_argument("--height", type=int, default=144)
    parser.add_argument("--x-bins", type=int, default=128)
    parser.add_argument("--yellow-class-id", type=int, default=1)
    parser.add_argument("--epochs", type=int, default=45)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--lr", type=float, default=8.0e-4)
    parser.add_argument("--weight-decay", type=float, default=1.0e-4)
    parser.add_argument("--patience", type=int, default=10)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument(
        "--pretrained-backbone",
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


def yellow_polygon_mask(
    label_path: Path,
    width: int,
    height: int,
    *,
    yellow_class_id: int = 1,
) -> np.ndarray:
    """Rasterize only the yellow-centerline polygons from a YOLO label."""
    mask = np.zeros((int(height), int(width)), dtype=np.uint8)
    if not label_path.is_file():
        return mask
    for raw_line in label_path.read_text(encoding="utf-8").splitlines():
        fields = raw_line.split()
        if len(fields) < 7:
            continue
        if int(float(fields[0])) != int(yellow_class_id):
            continue
        coordinates = np.asarray(
            [float(value) for value in fields[1:]], dtype=np.float32
        )
        if coordinates.size < 6 or coordinates.size % 2:
            continue
        points = coordinates.reshape(-1, 2)
        points[:, 0] = np.clip(points[:, 0] * width, 0, width - 1)
        points[:, 1] = np.clip(points[:, 1] * height, 0, height - 1)
        cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 255)
    return mask


def row_targets_from_mask(
    mask: np.ndarray,
    anchor_rows: tuple[int, ...] | list[int],
    x_bin_count: int,
) -> np.ndarray:
    """Connect internal dash gaps and return x-bin/none row targets."""
    if mask.ndim != 2:
        raise ValueError("yellow mask must be two-dimensional")
    height, width = mask.shape
    anchors = np.asarray(anchor_rows, dtype=np.int64)
    none_class = int(x_bin_count)
    targets = np.full(anchors.size, none_class, dtype=np.int64)

    observed_rows = np.flatnonzero(np.any(mask > 0, axis=1))
    if observed_rows.size == 0:
        return targets
    observed_x = np.asarray(
        [float(np.median(np.flatnonzero(mask[row] > 0))) for row in observed_rows],
        dtype=np.float32,
    )
    if observed_rows.size == 1:
        nearest = int(np.argmin(np.abs(anchors - observed_rows[0])))
        if abs(int(anchors[nearest]) - int(observed_rows[0])) <= 2:
            normalized = observed_x[0] / max(1.0, float(width - 1))
            targets[nearest] = int(
                np.clip(round(normalized * (x_bin_count - 1)), 0, x_bin_count - 1)
            )
        return targets

    inside = (anchors >= observed_rows[0]) & (anchors <= observed_rows[-1])
    interpolated = np.interp(
        anchors[inside].astype(np.float32),
        observed_rows.astype(np.float32),
        observed_x,
    )
    normalized = interpolated / max(1.0, float(width - 1))
    targets[inside] = np.clip(
        np.rint(normalized * (x_bin_count - 1)),
        0,
        x_bin_count - 1,
    ).astype(np.int64)
    return targets


class RowCenterlineDataset(Dataset):
    def __init__(
        self,
        root: Path,
        split: str,
        *,
        width: int,
        height: int,
        anchor_rows: tuple[int, ...],
        x_bin_count: int,
        yellow_class_id: int,
        augment: bool,
    ) -> None:
        self.image_dir = root / split / "images"
        self.label_dir = root / split / "labels"
        self.width = int(width)
        self.height = int(height)
        self.anchor_rows = anchor_rows
        self.x_bin_count = int(x_bin_count)
        self.yellow_class_id = int(yellow_class_id)
        self.augment = bool(augment)
        self.images = sorted(
            path
            for path in self.image_dir.iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        )
        if not self.images:
            raise RuntimeError(f"no images found in {self.image_dir}")

    def __len__(self) -> int:
        return len(self.images)

    @staticmethod
    def augment_color(image: np.ndarray) -> np.ndarray:
        if random.random() < 0.85:
            alpha = random.uniform(0.68, 1.32)
            beta = random.uniform(-28.0, 28.0)
            image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        if random.random() < 0.60:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[..., 0] = np.mod(hsv[..., 0] + random.uniform(-4.0, 4.0), 180.0)
            hsv[..., 1] *= random.uniform(0.65, 1.35)
            hsv[..., 2] *= random.uniform(0.72, 1.28)
            image = cv2.cvtColor(
                np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR
            )
        if random.random() < 0.18:
            image = cv2.GaussianBlur(image, (3, 3), 0)
        return image

    def geometric_augment(
        self, image: np.ndarray, mask: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        if random.random() < 0.50:
            image = cv2.flip(image, 1)
            mask = cv2.flip(mask, 1)
        if random.random() < 0.50:
            shift = random.randint(
                -int(round(self.width * 0.07)),
                int(round(self.width * 0.07)),
            )
            transform = np.asarray([[1.0, 0.0, shift], [0.0, 1.0, 0.0]])
            image = cv2.warpAffine(
                image,
                transform,
                (self.width, self.height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_REFLECT_101,
            )
            mask = cv2.warpAffine(
                mask,
                transform,
                (self.width, self.height),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=0,
            )
        return image, mask

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        image_path = self.images[index]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"failed to read {image_path}")
        image = cv2.resize(
            image, (self.width, self.height), interpolation=cv2.INTER_AREA
        )
        mask = yellow_polygon_mask(
            self.label_dir / f"{image_path.stem}.txt",
            self.width,
            self.height,
            yellow_class_id=self.yellow_class_id,
        )
        if self.augment:
            image, mask = self.geometric_augment(image, mask)
            image = self.augment_color(image)

        target = row_targets_from_mask(
            mask, self.anchor_rows, self.x_bin_count
        )
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        normalized = (
            rgb - np.asarray(IMAGENET_MEAN, dtype=np.float32)
        ) / np.asarray(IMAGENET_STD, dtype=np.float32)
        image_tensor = torch.from_numpy(
            np.ascontiguousarray(normalized.transpose(2, 0, 1))
        )
        return image_tensor, torch.from_numpy(target)


class RowCenterlineLoss(nn.Module):
    def __init__(
        self,
        anchor_rows: tuple[int, ...],
        image_height: int,
        x_bin_count: int,
    ) -> None:
        super().__init__()
        anchors = torch.tensor(anchor_rows, dtype=torch.float32)
        normalized_y = anchors / max(1.0, float(image_height - 1))
        # Give the far half of the image up to 2x the near-field weight.
        row_weights = 1.0 + (1.0 - normalized_y).clamp(0.0, 1.0)
        self.register_buffer("row_weights", row_weights)
        self.x_bin_count = int(x_bin_count)

    def forward(
        self, logits: torch.Tensor, target: torch.Tensor
    ) -> torch.Tensor:
        none_class = self.x_bin_count
        valid = target != none_class
        visibility_target = valid.to(logits.dtype)
        visibility_error = F.binary_cross_entropy_with_logits(
            logits[..., 1], visibility_target, reduction="none"
        )
        visibility_weight = torch.where(
            valid,
            torch.ones_like(visibility_error),
            torch.full_like(visibility_error, 0.45),
        )
        visibility = (
            visibility_error * visibility_weight * self.row_weights
        ).sum() / (visibility_weight * self.row_weights).sum().clamp_min(1.0)

        expected_x, _ = expected_row_coordinates(logits, self.x_bin_count)
        target_x = target.clamp_max(self.x_bin_count - 1).to(logits.dtype)
        target_x = target_x / max(1.0, float(self.x_bin_count - 1))
        coordinate_error = F.smooth_l1_loss(
            expected_x, target_x, reduction="none", beta=0.02
        )
        coordinate = (
            coordinate_error * valid * self.row_weights
        ).sum() / (valid * self.row_weights).sum().clamp_min(1.0)

        pair_valid = valid[:, 1:] & valid[:, :-1]
        predicted_delta = expected_x[:, 1:] - expected_x[:, :-1]
        target_delta = target_x[:, 1:] - target_x[:, :-1]
        shape_error = F.smooth_l1_loss(
            predicted_delta, target_delta, reduction="none", beta=0.01
        )
        shape = (shape_error * pair_valid).sum() / pair_valid.sum().clamp_min(1)
        return visibility + 4.0 * coordinate + 0.75 * shape


class MetricTotals(NamedTuple):
    target_valid: int
    predicted_valid: int
    true_visible: int
    false_visible: int
    missed_visible: int
    within_4: int
    within_8: int
    far_target_valid: int
    far_within_8: int
    absolute_error: float


def empty_totals() -> MetricTotals:
    return MetricTotals(0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0)


def add_totals(left: MetricTotals, right: MetricTotals) -> MetricTotals:
    return MetricTotals(*(a + b for a, b in zip(left, right)))


def batch_totals(
    logits: torch.Tensor,
    target: torch.Tensor,
    *,
    anchor_rows: tuple[int, ...],
    image_height: int,
    x_bin_count: int,
    image_width: int,
) -> MetricTotals:
    valid = target != x_bin_count
    expected_x, visibility = expected_row_coordinates(logits, x_bin_count)
    predicted_valid = visibility >= 0.5
    expected_px = expected_x * float(image_width - 1)
    target_px = target.clamp_max(x_bin_count - 1).to(logits.dtype)
    target_px = target_px / float(x_bin_count - 1) * float(image_width - 1)
    # Metrics may run under CUDA autocast. Accumulate in float32 so a full
    # validation split cannot overflow float16's roughly 65k maximum.
    error = torch.abs(expected_px - target_px).float()
    matched = valid & predicted_valid
    far_rows = torch.tensor(
        anchor_rows, device=target.device
    ) <= int(round(0.68 * (image_height - 1)))
    far_valid = valid & far_rows.unsqueeze(0)
    return MetricTotals(
        target_valid=int(valid.sum()),
        predicted_valid=int(predicted_valid.sum()),
        true_visible=int(matched.sum()),
        false_visible=int((~valid & predicted_valid).sum()),
        missed_visible=int((valid & ~predicted_valid).sum()),
        within_4=int((matched & (error <= 4.0)).sum()),
        within_8=int((matched & (error <= 8.0)).sum()),
        far_target_valid=int(far_valid.sum()),
        far_within_8=int((far_valid & predicted_valid & (error <= 8.0)).sum()),
        absolute_error=float((error * matched).sum()),
    )


def metrics_from_totals(totals: MetricTotals) -> dict[str, float]:
    target = max(1, totals.target_valid)
    predicted = max(1, totals.predicted_valid)
    matched = max(1, totals.true_visible)
    far_target = max(1, totals.far_target_valid)
    return {
        "presence_recall": totals.true_visible / target,
        "presence_precision": totals.true_visible / predicted,
        "within_4px_recall": totals.within_4 / target,
        "within_8px_recall": totals.within_8 / target,
        "far_within_8px_recall": totals.far_within_8 / far_target,
        "visible_x_mae_px": totals.absolute_error / matched,
        "false_visible_rows": float(totals.false_visible),
        "missed_visible_rows": float(totals.missed_visible),
    }


def run_epoch(
    model: nn.Module,
    loader: DataLoader,
    criterion: RowCenterlineLoss,
    device: torch.device,
    *,
    anchor_rows: tuple[int, ...],
    image_width: int,
    image_height: int,
    x_bin_count: int,
    optimizer: torch.optim.Optimizer | None,
    scaler: torch.amp.GradScaler | None,
) -> tuple[float, dict[str, float]]:
    training = optimizer is not None
    model.train(training)
    total_loss = 0.0
    total_items = 0
    totals = empty_totals()
    for images, targets in loader:
        images = images.to(device, non_blocking=True)
        targets = targets.to(device, non_blocking=True)
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training), torch.autocast(
            device_type=device.type, enabled=device.type == "cuda"
        ):
            logits = model(images)
            loss = criterion(logits, targets)
        if training and optimizer is not None and scaler is not None:
            scaler.scale(loss).backward()
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
                anchor_rows=anchor_rows,
                image_height=image_height,
                x_bin_count=x_bin_count,
                image_width=image_width,
            ),
        )
    return total_loss / max(1, total_items), metrics_from_totals(totals)


def save_history(path: Path, history: list[dict[str, float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(history[0].keys()))
        writer.writeheader()
        writer.writerows(history)


def export_models(
    checkpoint: dict,
    output_dir: Path,
    *,
    width: int,
    height: int,
    x_bin_count: int,
    anchor_rows: tuple[int, ...],
) -> tuple[Path, Path]:
    core = RowCenterlineModel(
        row_count=len(anchor_rows),
        x_bin_count=x_bin_count,
        pretrained_backbone=False,
    )
    core.load_state_dict(checkpoint["model_state"])
    core.eval()
    example = torch.zeros(1, 3, height, width)
    core_script = torch.jit.freeze(torch.jit.trace(core, example, strict=True))
    core_path = output_dir / "yellow_row_centerline_core_256x144.pt"
    core_script.save(str(core_path))

    adapter = RowCenterlineMaskAdapter(
        core,
        anchor_rows=anchor_rows,
        output_width=width,
        output_height=height,
        x_bin_count=x_bin_count,
    ).eval()
    adapter_script = torch.jit.freeze(
        torch.jit.trace(adapter, example, strict=True)
    )
    mask_path = output_dir / "yellow_row_centerline_mask_256x144.pt"
    adapter_script.save(str(mask_path))
    return core_path, mask_path


def main() -> None:
    args = parse_args()
    seed_everything(args.seed)
    dataset_root = Path(args.dataset_root).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    anchor_rows = make_anchor_rows(args.height)
    device = torch.device(
        args.device
        if args.device != "cuda" or torch.cuda.is_available()
        else "cpu"
    )

    datasets = {
        split: RowCenterlineDataset(
            dataset_root,
            split,
            width=args.width,
            height=args.height,
            anchor_rows=anchor_rows,
            x_bin_count=args.x_bins,
            yellow_class_id=args.yellow_class_id,
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
    model = RowCenterlineModel(
        row_count=len(anchor_rows),
        x_bin_count=args.x_bins,
        pretrained_backbone=args.pretrained_backbone,
    ).to(device)
    criterion = RowCenterlineLoss(
        anchor_rows, args.height, args.x_bins
    ).to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.lr, weight_decay=args.weight_decay
    )
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="max", factor=0.5, patience=3, min_lr=2.0e-6
    )
    scaler = torch.amp.GradScaler("cuda", enabled=device.type == "cuda")

    parameters = sum(parameter.numel() for parameter in model.parameters())
    print(
        f"dataset train={len(datasets['train'])} valid={len(datasets['valid'])} "
        f"test={len(datasets['test'])} input={args.width}x{args.height} "
        f"anchors={len(anchor_rows)} bins={args.x_bins} device={device} "
        f"parameters={parameters}"
    )
    print(f"anchor_rows={list(anchor_rows)}")

    best_score = -1.0
    stale_epochs = 0
    history: list[dict[str, float]] = []
    best_path = output_dir / "best_checkpoint.pt"
    for epoch in range(1, args.epochs + 1):
        train_loss, train_metrics = run_epoch(
            model,
            loaders["train"],
            criterion,
            device,
            anchor_rows=anchor_rows,
            image_width=args.width,
            image_height=args.height,
            x_bin_count=args.x_bins,
            optimizer=optimizer,
            scaler=scaler,
        )
        with torch.inference_mode():
            val_loss, val_metrics = run_epoch(
                model,
                loaders["valid"],
                criterion,
                device,
                anchor_rows=anchor_rows,
                image_width=args.width,
                image_height=args.height,
                x_bin_count=args.x_bins,
                optimizer=None,
                scaler=None,
            )
        score = 0.55 * val_metrics["within_8px_recall"] + 0.45 * val_metrics[
            "far_within_8px_recall"
        ]
        scheduler.step(score)
        row = {
            "epoch": float(epoch),
            "lr": float(optimizer.param_groups[0]["lr"]),
            "train_loss": train_loss,
            "val_loss": val_loss,
            "val_score": score,
            **{f"train_{key}": value for key, value in train_metrics.items()},
            **{f"val_{key}": value for key, value in val_metrics.items()},
        }
        history.append(row)
        save_history(output_dir / "history.csv", history)
        print(
            f"epoch={epoch:03d}/{args.epochs} train={train_loss:.4f} "
            f"val={val_loss:.4f} score={score:.4f} "
            f"R8={val_metrics['within_8px_recall']:.4f} "
            f"far_R8={val_metrics['far_within_8px_recall']:.4f} "
            f"MAE={val_metrics['visible_x_mae_px']:.2f}px"
        )
        if score > best_score + 1.0e-4:
            best_score = score
            stale_epochs = 0
            torch.save(
                {
                    "model_state": model.state_dict(),
                    "epoch": epoch,
                    "metrics": val_metrics,
                    "score": score,
                    "width": args.width,
                    "height": args.height,
                    "x_bin_count": args.x_bins,
                    "anchor_rows": list(anchor_rows),
                    "yellow_class_id": args.yellow_class_id,
                },
                best_path,
            )
        else:
            stale_epochs += 1
        if stale_epochs >= args.patience:
            print(f"early stopping at epoch={epoch}, best_score={best_score:.4f}")
            break

    checkpoint = torch.load(best_path, map_location="cpu", weights_only=False)
    core_path, mask_path = export_models(
        checkpoint,
        output_dir,
        width=args.width,
        height=args.height,
        x_bin_count=args.x_bins,
        anchor_rows=anchor_rows,
    )
    model.load_state_dict(checkpoint["model_state"])
    model.to(device)
    with torch.inference_mode():
        test_loss, test_metrics = run_epoch(
            model,
            loaders["test"],
            criterion,
            device,
            anchor_rows=anchor_rows,
            image_width=args.width,
            image_height=args.height,
            x_bin_count=args.x_bins,
            optimizer=None,
            scaler=None,
        )
    summary = {
        "dataset_root": str(dataset_root),
        "best_epoch": int(checkpoint["epoch"]),
        "validation_score": float(checkpoint["score"]),
        "validation": checkpoint["metrics"],
        "test_loss": test_loss,
        "test": test_metrics,
        "input_width": args.width,
        "input_height": args.height,
        "anchor_rows": list(anchor_rows),
        "x_bin_count": args.x_bins,
        "parameters": parameters,
        "core_torchscript": str(core_path),
        "mask_compatible_torchscript": str(mask_path),
    }
    (output_dir / "metrics.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
