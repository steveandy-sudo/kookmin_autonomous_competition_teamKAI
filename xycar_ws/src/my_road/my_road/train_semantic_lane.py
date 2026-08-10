from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import random

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

from my_road.lightweight_lane_model import (
    IMAGE_NET_MEAN,
    IMAGE_NET_STD,
    LaneLogitsModel,
    build_lightweight_lane_model,
)


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train 3-class lightweight lane semantic segmentation."
    )
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--width", type=int, default=256)
    parser.add_argument("--height", type=int, default=144)
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch-size", type=int, default=96)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--lr", type=float, default=3.0e-4)
    parser.add_argument("--weight-decay", type=float, default=1.0e-4)
    parser.add_argument("--patience", type=int, default=12)
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


def polygons_to_mask(label_path: Path, width: int, height: int) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    if not label_path.is_file():
        return mask
    for raw_line in label_path.read_text(encoding="utf-8").splitlines():
        fields = raw_line.split()
        if len(fields) < 7:
            continue
        class_id = int(float(fields[0]))
        if class_id not in (0, 1):
            continue
        coordinates = np.asarray([float(value) for value in fields[1:]], dtype=np.float32)
        if coordinates.size % 2:
            continue
        points = coordinates.reshape(-1, 2)
        points[:, 0] = np.clip(points[:, 0] * width, 0, width - 1)
        points[:, 1] = np.clip(points[:, 1] * height, 0, height - 1)
        cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], class_id + 1)
    return mask


class LaneSemanticDataset(Dataset):
    def __init__(
        self,
        root: Path,
        split: str,
        *,
        width: int,
        height: int,
        augment: bool,
    ) -> None:
        self.image_dir = root / split / "images"
        self.label_dir = root / split / "labels"
        self.width = int(width)
        self.height = int(height)
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

    def _augment_color(self, image: np.ndarray) -> np.ndarray:
        if random.random() < 0.85:
            alpha = random.uniform(0.70, 1.30)
            beta = random.uniform(-25.0, 25.0)
            image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        if random.random() < 0.60:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
            hsv[..., 1] *= random.uniform(0.65, 1.35)
            hsv[..., 2] *= random.uniform(0.75, 1.25)
            hsv = np.clip(hsv, 0, 255).astype(np.uint8)
            image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        if random.random() < 0.15:
            image = cv2.GaussianBlur(image, (3, 3), 0)
        return image

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor]:
        image_path = self.images[index]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"failed to read {image_path}")
        source_height, source_width = image.shape[:2]
        mask = polygons_to_mask(
            self.label_dir / f"{image_path.stem}.txt",
            source_width,
            source_height,
        )
        if self.augment:
            image = self._augment_color(image)

        image = cv2.resize(
            image,
            (self.width, self.height),
            interpolation=cv2.INTER_AREA,
        )
        mask = cv2.resize(
            mask,
            (self.width, self.height),
            interpolation=cv2.INTER_NEAREST,
        )
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_tensor = torch.from_numpy(
            np.ascontiguousarray(image.transpose(2, 0, 1))
        ).float().div_(255.0)
        mean = torch.tensor(IMAGE_NET_MEAN, dtype=torch.float32).view(3, 1, 1)
        std = torch.tensor(IMAGE_NET_STD, dtype=torch.float32).view(3, 1, 1)
        image_tensor = (image_tensor - mean) / std
        return image_tensor, torch.from_numpy(mask.astype(np.int64))


class LaneLoss(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.register_buffer(
            "class_weights", torch.tensor([0.20, 1.00, 1.40], dtype=torch.float32)
        )

    def forward(self, logits: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
        cross_entropy = nn.functional.cross_entropy(
            logits, target, weight=self.class_weights
        )
        probabilities = logits.softmax(dim=1)
        one_hot = nn.functional.one_hot(target, num_classes=3).permute(0, 3, 1, 2)
        one_hot = one_hot.to(dtype=probabilities.dtype)
        intersection = (probabilities[:, 1:] * one_hot[:, 1:]).sum((0, 2, 3))
        denominator = (probabilities[:, 1:] + one_hot[:, 1:]).sum((0, 2, 3))
        dice_loss = 1.0 - ((2.0 * intersection + 1.0) / (denominator + 1.0)).mean()
        return cross_entropy + 0.75 * dice_loss


def update_confusion(
    confusion: torch.Tensor,
    logits: torch.Tensor,
    target: torch.Tensor,
) -> None:
    prediction = logits.argmax(dim=1)
    encoded = target.reshape(-1) * 3 + prediction.reshape(-1)
    confusion += torch.bincount(encoded, minlength=9).reshape(3, 3).cpu()


def metrics_from_confusion(confusion: torch.Tensor) -> dict[str, float]:
    values = confusion.to(torch.float64)
    true_positive = values.diag()
    actual = values.sum(dim=1)
    predicted = values.sum(dim=0)
    union = actual + predicted - true_positive
    iou = true_positive / union.clamp_min(1.0)
    recall = true_positive / actual.clamp_min(1.0)
    precision = true_positive / predicted.clamp_min(1.0)
    return {
        "mean_lane_iou": float(iou[1:].mean()),
        "white_iou": float(iou[1]),
        "yellow_iou": float(iou[2]),
        "white_recall": float(recall[1]),
        "yellow_recall": float(recall[2]),
        "white_precision": float(precision[1]),
        "yellow_precision": float(precision[2]),
    }


def run_epoch(
    model: nn.Module,
    loader: DataLoader,
    criterion: LaneLoss,
    device: torch.device,
    *,
    optimizer: torch.optim.Optimizer | None,
    scaler: torch.amp.GradScaler | None,
) -> tuple[float, dict[str, float]]:
    training = optimizer is not None
    model.train(training)
    confusion = torch.zeros((3, 3), dtype=torch.int64)
    total_loss = 0.0
    total_items = 0
    for images, targets in loader:
        images = images.to(device, non_blocking=True)
        targets = targets.to(device, non_blocking=True)
        if training:
            optimizer.zero_grad(set_to_none=True)
        with torch.set_grad_enabled(training), torch.autocast(
            device_type=device.type,
            enabled=device.type == "cuda",
        ):
            logits = model(images)["out"]
            loss = criterion(logits, targets)
        if training and optimizer is not None and scaler is not None:
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
        batch_size = images.shape[0]
        total_loss += float(loss.detach()) * batch_size
        total_items += batch_size
        update_confusion(confusion, logits.detach(), targets)
    return total_loss / max(total_items, 1), metrics_from_confusion(confusion)


def save_history(path: Path, history: list[dict[str, float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(history[0].keys()))
        writer.writeheader()
        writer.writerows(history)


def main() -> None:
    args = parse_args()
    seed_everything(args.seed)
    dataset_root = Path(args.dataset_root).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    device = torch.device(
        args.device if args.device != "cuda" or torch.cuda.is_available() else "cpu"
    )

    datasets = {
        "train": LaneSemanticDataset(
            dataset_root,
            "train",
            width=args.width,
            height=args.height,
            augment=True,
        ),
        "val": LaneSemanticDataset(
            dataset_root,
            "valid",
            width=args.width,
            height=args.height,
            augment=False,
        ),
        "test": LaneSemanticDataset(
            dataset_root,
            "test",
            width=args.width,
            height=args.height,
            augment=False,
        ),
    }
    loaders = {
        name: DataLoader(
            dataset,
            batch_size=args.batch_size,
            shuffle=name == "train",
            num_workers=args.workers,
            pin_memory=device.type == "cuda",
            persistent_workers=args.workers > 0,
        )
        for name, dataset in datasets.items()
    }
    model = build_lightweight_lane_model(
        pretrained_backbone=args.pretrained_backbone
    ).to(device)
    criterion = LaneLoss().to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.lr, weight_decay=args.weight_decay
    )
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="max", factor=0.5, patience=4, min_lr=1.0e-6
    )
    scaler = torch.amp.GradScaler("cuda", enabled=device.type == "cuda")

    print(
        f"dataset train={len(datasets['train'])} val={len(datasets['val'])} "
        f"test={len(datasets['test'])} input={args.width}x{args.height} "
        f"device={device}"
    )
    print(f"parameters={sum(parameter.numel() for parameter in model.parameters())}")

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
            optimizer=optimizer,
            scaler=scaler,
        )
        with torch.inference_mode():
            val_loss, val_metrics = run_epoch(
                model,
                loaders["val"],
                criterion,
                device,
                optimizer=None,
                scaler=None,
            )
        score = val_metrics["mean_lane_iou"]
        scheduler.step(score)
        row = {
            "epoch": float(epoch),
            "lr": float(optimizer.param_groups[0]["lr"]),
            "train_loss": train_loss,
            "val_loss": val_loss,
            **{f"train_{key}": value for key, value in train_metrics.items()},
            **{f"val_{key}": value for key, value in val_metrics.items()},
        }
        history.append(row)
        save_history(output_dir / "history.csv", history)
        print(
            f"epoch={epoch:03d}/{args.epochs} train_loss={train_loss:.4f} "
            f"val_loss={val_loss:.4f} lane_mIoU={score:.4f} "
            f"white_R={val_metrics['white_recall']:.4f} "
            f"yellow_R={val_metrics['yellow_recall']:.4f}"
        )
        if score > best_score + 1.0e-4:
            best_score = score
            stale_epochs = 0
            torch.save(
                {
                    "model_state": model.state_dict(),
                    "epoch": epoch,
                    "metrics": val_metrics,
                    "width": args.width,
                    "height": args.height,
                    "class_names": ["background", "white_boundary", "yellow_centerline"],
                },
                best_path,
            )
        else:
            stale_epochs += 1
        if stale_epochs >= args.patience:
            print(f"early stopping at epoch={epoch}, best_lane_mIoU={best_score:.4f}")
            break

    checkpoint = torch.load(best_path, map_location="cpu", weights_only=False)
    export_model = build_lightweight_lane_model(pretrained_backbone=False)
    export_model.load_state_dict(checkpoint["model_state"])
    export_model.eval()
    wrapper = LaneLogitsModel(export_model).eval()
    example = torch.zeros(1, 3, args.height, args.width)
    traced = torch.jit.trace(wrapper, example, strict=True)
    traced = torch.jit.freeze(traced)
    torchscript_path = output_dir / "lane_lraspp_mbv3s_256x144.torchscript.pt"
    traced.save(str(torchscript_path))

    model.load_state_dict(checkpoint["model_state"])
    model.to(device)
    with torch.inference_mode():
        test_loss, test_metrics = run_epoch(
            model,
            loaders["test"],
            criterion,
            device,
            optimizer=None,
            scaler=None,
        )
    summary = {
        "best_epoch": int(checkpoint["epoch"]),
        "best_validation": checkpoint["metrics"],
        "test_loss": test_loss,
        "test": test_metrics,
        "input_width": args.width,
        "input_height": args.height,
        "parameters": sum(parameter.numel() for parameter in model.parameters()),
        "torchscript": str(torchscript_path),
    }
    (output_dir / "metrics.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
