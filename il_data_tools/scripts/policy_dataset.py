#!/usr/bin/env python3

from __future__ import annotations

import csv
import random
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset


class PolicyCsvDataset(Dataset):
    def __init__(
        self,
        csv_path: str,
        input_width: int = 160,
        input_height: int = 90,
        max_steer_deg: float = 100.0,
        use_phase: bool = False,
        enable_augment: bool = False,
        enable_flip: bool = False,
    ) -> None:
        self.csv_path = Path(csv_path).expanduser().resolve()
        self.input_width = input_width
        self.input_height = input_height
        self.max_steer_deg = max_steer_deg
        self.use_phase = use_phase
        self.enable_augment = enable_augment
        self.enable_flip = enable_flip

        with self.csv_path.open("r", newline="", encoding="utf-8-sig") as handle:
            rows = list(csv.DictReader(handle))
        if not rows:
            raise ValueError(f"empty dataset CSV: {self.csv_path}")

        self.rows: List[Dict[str, str]] = []
        missing = []
        for index, row in enumerate(rows, start=2):
            image_path = Path(row.get("image_path", "")).expanduser()
            if not image_path.is_absolute():
                image_path = (self.csv_path.parent / image_path).resolve()
            if not image_path.is_file():
                missing.append({"row": index, "image_path": str(image_path)})
                continue
            row = dict(row)
            row["image_path"] = str(image_path)
            self.rows.append(row)

        if missing:
            preview = missing[:5]
            raise FileNotFoundError(
                f"{len(missing)} image files are missing in {self.csv_path}: {preview}"
            )
        if use_phase and "phase" not in self.rows[0]:
            raise ValueError(f"phase column is required for phase model: {self.csv_path}")

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, index: int) -> Dict[str, object]:
        row = self.rows[index]
        image = load_image_bgr(row["image_path"])
        steer_norm = float(row["steer_norm"])

        if self.enable_augment:
            image, steer_norm = augment_image(
                image,
                steer_norm,
                enable_flip=self.enable_flip,
            )

        image = preprocess_image(image, self.input_width, self.input_height)
        phase = float(row.get("phase") or 0.0)
        angle_deg = float(row.get("angle_deg") or steer_norm * self.max_steer_deg)
        speed = float(row.get("speed") or 0.0)
        metadata = {
            "image_path": row.get("image_path", ""),
            "angle_deg": angle_deg,
            "speed": speed,
            "mission_label": row.get("mission_label", ""),
            "session_id": row.get("session_id", ""),
            "timestamp_ns": row.get("timestamp_ns", ""),
            "phase": phase,
        }
        return {
            "image": image,
            "target": torch.tensor([steer_norm], dtype=torch.float32),
            "phase": torch.tensor([phase], dtype=torch.float32),
            "metadata": metadata,
        }

    def session_ids(self) -> set:
        return {row.get("session_id", "") for row in self.rows}


def load_image_bgr(path: str) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"failed to read image: {path}")
    return image


def preprocess_image(
    image_bgr: np.ndarray,
    input_width: int,
    input_height: int,
) -> torch.Tensor:
    image = cv2.resize(image_bgr, (input_width, input_height), interpolation=cv2.INTER_AREA)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.astype(np.float32) / 255.0
    image = np.transpose(image, (2, 0, 1))
    return torch.from_numpy(image)


def augment_image(
    image_bgr: np.ndarray,
    steer_norm: float,
    enable_flip: bool = False,
) -> Tuple[np.ndarray, float]:
    image = image_bgr.copy()
    image = random_brightness_contrast_gamma(image)
    image = random_noise_blur(image)
    image = random_shift_crop(image)
    if enable_flip and random.random() < 0.5:
        image = cv2.flip(image, 1)
        steer_norm = -steer_norm
    return image, steer_norm


def random_brightness_contrast_gamma(image: np.ndarray) -> np.ndarray:
    alpha = random.uniform(0.85, 1.15)
    beta = random.uniform(-18.0, 18.0)
    out = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    gamma = random.uniform(0.85, 1.15)
    table = np.array(
        [((i / 255.0) ** (1.0 / gamma)) * 255.0 for i in range(256)]
    ).astype("uint8")
    return cv2.LUT(out, table)


def random_noise_blur(image: np.ndarray) -> np.ndarray:
    if random.random() < 0.35:
        sigma = random.uniform(1.0, 4.0)
        noise = np.random.normal(0.0, sigma, image.shape).astype(np.float32)
        image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    if random.random() < 0.20:
        image = cv2.GaussianBlur(image, (3, 3), 0)
    return image


def random_shift_crop(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    max_dx = max(1, int(width * 0.04))
    max_dy = max(1, int(height * 0.04))
    dx = random.randint(-max_dx, max_dx)
    dy = random.randint(-max_dy, max_dy)
    matrix = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(
        image,
        matrix,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    )
