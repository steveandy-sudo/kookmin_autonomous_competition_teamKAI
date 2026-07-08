from __future__ import annotations

import csv
import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset

from il_data_tools.model_catalog import PolicyModelSpec
from il_data_tools.torch_model_factory import create_policy_model


class SteeringCsvDataset(Dataset):
    def __init__(
        self,
        csv_path: Path,
        image_size: Tuple[int, int],
        phase_enabled: bool,
    ) -> None:
        self.csv_path = Path(csv_path)
        self.image_size = image_size
        self.phase_enabled = phase_enabled
        with self.csv_path.open("r", newline="", encoding="utf-8") as handle:
            self.rows = list(csv.DictReader(handle))
        if not self.rows:
            raise ValueError(f"empty CSV: {self.csv_path}")
        if phase_enabled and "phase" not in self.rows[0]:
            raise ValueError(f"phase column is required for phase-enabled model: {csv_path}")

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> Dict[str, torch.Tensor]:
        row = self.rows[index]
        image = load_image_tensor(Path(row["image_path"]), self.image_size)
        steer = torch.tensor(float(row["steer_norm"]), dtype=torch.float32)
        angle = torch.tensor(float(row.get("angle_deg") or 0.0), dtype=torch.float32)
        phase_value = float(row.get("phase") or 0.0)
        phase = torch.tensor(phase_value, dtype=torch.float32)
        return {
            "image": image,
            "steer": steer,
            "angle": angle,
            "phase": phase,
        }


def load_image_tensor(path: Path, image_size: Tuple[int, int]) -> torch.Tensor:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"failed to read image: {path}")
    height, width = image_size
    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    array = image.astype(np.float32) / 255.0
    array = (array - 0.5) / 0.5
    array = np.transpose(array, (2, 0, 1))
    return torch.from_numpy(array)


def train_policy(
    spec: PolicyModelSpec,
    train_csv: Path,
    val_csv: Path,
    output_dir: Path,
    image_size: Tuple[int, int],
    max_steer_deg: float,
    epochs: int,
    batch_size: int,
    lr: float,
    weight_decay: float,
    num_workers: int,
    device_name: str,
    export_torchscript: bool,
    export_onnx: bool,
) -> Dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    device = choose_device(device_name)
    model = create_policy_model(
        spec.model_type, phase_enabled=spec.phase_enabled, image_size=image_size
    ).to(device)
    loss_fn = nn.SmoothL1Loss()
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=weight_decay)

    train_loader = make_loader(train_csv, image_size, spec.phase_enabled, batch_size, num_workers, True)
    val_loader = make_loader(val_csv, image_size, spec.phase_enabled, batch_size, num_workers, False)

    best_val = float("inf")
    best_epoch = -1
    history: List[Dict[str, float]] = []
    best_state = None
    started = time.time()

    for epoch in range(1, epochs + 1):
        train_metrics = run_epoch(
            model, train_loader, loss_fn, device, spec.phase_enabled, max_steer_deg, optimizer
        )
        val_metrics = run_epoch(
            model, val_loader, loss_fn, device, spec.phase_enabled, max_steer_deg, None
        )
        row = {
            "epoch": epoch,
            "train_loss": train_metrics["loss"],
            "train_mae_norm": train_metrics["mae_norm"],
            "val_loss": val_metrics["loss"],
            "val_mae_norm": val_metrics["mae_norm"],
            "val_mae_deg": val_metrics["mae_deg"],
        }
        history.append(row)
        print(
            "epoch={epoch} train_loss={train_loss:.5f} val_loss={val_loss:.5f} "
            "val_mae_deg={val_mae_deg:.3f}".format(**row)
        )
        if val_metrics["loss"] < best_val:
            best_val = val_metrics["loss"]
            best_epoch = epoch
            best_state = {
                key: value.detach().cpu().clone()
                for key, value in model.state_dict().items()
            }

    if best_state is not None:
        model.load_state_dict(best_state)

    checkpoint_path = output_dir / f"{spec.policy}_policy_{spec.model_variant}.pth"
    torch.save(
        {
            "policy": spec.policy,
            "model_type": spec.model_type,
            "model_variant": spec.model_variant,
            "phase_enabled": spec.phase_enabled,
            "experimental": spec.experimental,
            "image_size": list(image_size),
            "max_steer_deg": max_steer_deg,
            "state_dict": model.state_dict(),
            "best_epoch": best_epoch,
            "best_val_loss": best_val,
            "history": history,
        },
        checkpoint_path,
    )

    scripted_path = None
    onnx_path = None
    if export_torchscript:
        scripted_path = export_scripted_model(model, spec, output_dir, image_size, device)
    if export_onnx:
        onnx_path = export_onnx_model(model, spec, output_dir, image_size, device)

    report = {
        "policy": spec.policy,
        "model_type": spec.model_type,
        "model_variant": spec.model_variant,
        "phase_enabled": spec.phase_enabled,
        "experimental": spec.experimental,
        "selection_note": (
            "Final model must be selected using validation metrics, Jetson Orin Nano "
            "runtime benchmark, and low-speed closed-loop driving tests."
        ),
        "safety_note": "Policy output is steering only. Speed and safety remain rule-based.",
        "train_csv": str(train_csv),
        "val_csv": str(val_csv),
        "output_dir": str(output_dir),
        "image_size": list(image_size),
        "max_steer_deg": max_steer_deg,
        "epochs": epochs,
        "batch_size": batch_size,
        "device": str(device),
        "best_epoch": best_epoch,
        "best_val_loss": best_val,
        "checkpoint_path": str(checkpoint_path),
        "torchscript_path": str(scripted_path) if scripted_path else "",
        "onnx_path": str(onnx_path) if onnx_path else "",
        "history": history,
        "elapsed_sec": time.time() - started,
    }
    report_path = output_dir / f"{spec.policy}_policy_{spec.model_variant}_report.json"
    write_json(report_path, report)
    return report


def make_loader(
    csv_path: Path,
    image_size: Tuple[int, int],
    phase_enabled: bool,
    batch_size: int,
    num_workers: int,
    shuffle: bool,
) -> DataLoader:
    dataset = SteeringCsvDataset(csv_path, image_size, phase_enabled)
    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=num_workers,
        pin_memory=torch.cuda.is_available(),
    )


def run_epoch(
    model: nn.Module,
    loader: DataLoader,
    loss_fn: nn.Module,
    device: torch.device,
    phase_enabled: bool,
    max_steer_deg: float,
    optimizer: Optional[torch.optim.Optimizer],
) -> Dict[str, float]:
    training = optimizer is not None
    model.train(training)
    total_loss = 0.0
    total_abs = 0.0
    total_count = 0
    with torch.set_grad_enabled(training):
        for batch in loader:
            image = batch["image"].to(device)
            steer = batch["steer"].to(device)
            phase = batch["phase"].to(device)
            pred = model(image, phase) if phase_enabled else model(image)
            loss = loss_fn(pred, steer)
            if training:
                optimizer.zero_grad(set_to_none=True)
                loss.backward()
                optimizer.step()
            count = steer.numel()
            total_loss += float(loss.detach().cpu()) * count
            total_abs += float(torch.sum(torch.abs(pred.detach() - steer)).cpu())
            total_count += count
    mean_loss = total_loss / max(1, total_count)
    mae_norm = total_abs / max(1, total_count)
    return {
        "loss": mean_loss,
        "mae_norm": mae_norm,
        "mae_deg": mae_norm * max_steer_deg,
    }


def export_scripted_model(
    model: nn.Module,
    spec: PolicyModelSpec,
    output_dir: Path,
    image_size: Tuple[int, int],
    device: torch.device,
) -> Path:
    model.eval()
    example = make_example_input(image_size, device, spec.phase_enabled)
    with torch.no_grad():
        scripted = torch.jit.trace(model, example)
    specific_path = output_dir / f"{spec.policy}_policy_{spec.model_variant}_scripted.pt"
    generic_path = output_dir / f"{spec.policy}_policy_scripted.pt"
    scripted.save(str(specific_path))
    scripted.save(str(generic_path))
    return specific_path


def export_onnx_model(
    model: nn.Module,
    spec: PolicyModelSpec,
    output_dir: Path,
    image_size: Tuple[int, int],
    device: torch.device,
) -> Path:
    model.eval()
    example = make_example_input(image_size, device, spec.phase_enabled)
    output_path = output_dir / f"{spec.policy}_policy_{spec.model_variant}.onnx"
    input_names = ["image", "phase"] if spec.phase_enabled else ["image"]
    dynamic_axes = {
        "image": {0: "batch"},
        "steer_norm": {0: "batch"},
    }
    if spec.phase_enabled:
        dynamic_axes["phase"] = {0: "batch"}
    torch.onnx.export(
        model,
        example,
        str(output_path),
        input_names=input_names,
        output_names=["steer_norm"],
        dynamic_axes=dynamic_axes,
        opset_version=17,
    )
    return output_path


def make_example_input(
    image_size: Tuple[int, int],
    device: torch.device,
    phase_enabled: bool,
):
    height, width = image_size
    image = torch.zeros(1, 3, height, width, device=device)
    if phase_enabled:
        phase = torch.zeros(1, device=device)
        return image, phase
    return image


def choose_device(device_name: str) -> torch.device:
    if device_name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(device_name)


def write_json(path: Path, data: Dict[str, object]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False, sort_keys=True)
        handle.write("\n")
