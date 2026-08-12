from __future__ import annotations

import numpy as np
import torch
from torch import nn
from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small


IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


def make_anchor_rows(
    height: int,
    *,
    far_count: int = 32,
    near_count: int = 16,
    start_ratio: float = 0.25,
    split_ratio: float = 0.68,
    end_ratio: float = 0.98,
) -> tuple[int, ...]:
    """Create non-uniform rows with extra samples in the far field."""
    maximum = max(0, int(height) - 1)
    far = np.linspace(
        float(start_ratio) * maximum,
        float(split_ratio) * maximum,
        max(2, int(far_count)),
    )
    near = np.linspace(
        float(split_ratio) * maximum,
        float(end_ratio) * maximum,
        max(2, int(near_count)) + 1,
    )[1:]
    rows = np.unique(np.rint(np.concatenate((far, near))).astype(np.int64))
    return tuple(int(np.clip(row, 0, maximum)) for row in rows)


def interpolation_matrix(
    anchor_rows: tuple[int, ...] | list[int],
    height: int,
) -> np.ndarray:
    """Return a matrix that linearly expands anchor values to image rows."""
    anchors = np.asarray(anchor_rows, dtype=np.int64)
    if anchors.ndim != 1 or anchors.size < 2:
        raise ValueError("at least two anchor rows are required")
    if np.any(np.diff(anchors) <= 0):
        raise ValueError("anchor rows must be strictly increasing")
    if anchors[0] < 0 or anchors[-1] >= int(height):
        raise ValueError("anchor rows are outside the output image")

    matrix = np.zeros((int(height), anchors.size), dtype=np.float32)
    for row in range(int(height)):
        if row < anchors[0] or row > anchors[-1]:
            continue
        right = int(np.searchsorted(anchors, row, side="left"))
        if right == 0:
            matrix[row, 0] = 1.0
        elif right >= anchors.size:
            matrix[row, -1] = 1.0
        elif anchors[right] == row:
            matrix[row, right] = 1.0
        else:
            left = right - 1
            span = float(anchors[right] - anchors[left])
            right_weight = float(row - anchors[left]) / span
            matrix[row, left] = 1.0 - right_weight
            matrix[row, right] = right_weight
    return matrix


class RowCenterlineModel(nn.Module):
    """Regress normalized x and visibility at each anchor row."""

    def __init__(
        self,
        *,
        row_count: int,
        x_bin_count: int = 128,
        hidden_channels: int = 64,
        pretrained_backbone: bool = False,
    ) -> None:
        super().__init__()
        weights = (
            MobileNet_V3_Small_Weights.DEFAULT
            if pretrained_backbone
            else None
        )
        mobilenet = mobilenet_v3_small(weights=weights)
        # Stop before the final stride-32 blocks. This retains 9x16 spatial
        # evidence at 144x256 while using only about 190k backbone parameters.
        self.backbone = nn.Sequential(*list(mobilenet.features.children())[:9])
        self.projection = nn.Sequential(
            nn.Conv2d(48, 8, kernel_size=1, bias=False),
            nn.BatchNorm2d(8),
            nn.Hardswish(inplace=True),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(8 * 9 * 16, int(hidden_channels)),
            nn.Hardswish(inplace=True),
            nn.Dropout(p=0.10),
            nn.Linear(
                int(hidden_channels),
                int(row_count) * 2,
            ),
        )
        self.row_count = int(row_count)
        self.x_bin_count = int(x_bin_count)

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        feature = self.projection(self.backbone(image))
        logits = self.classifier(feature)
        return logits.reshape(
            image.shape[0], self.row_count, 2
        )


class RowCenterlineMaskAdapter(nn.Module):
    """Expose row-coordinate predictions as legacy three-class mask logits."""

    def __init__(
        self,
        core: nn.Module,
        *,
        anchor_rows: tuple[int, ...] | list[int],
        output_width: int = 256,
        output_height: int = 144,
        x_bin_count: int = 128,
        line_half_width_px: float = 1.5,
        line_sharpness: float = 2.0,
        visibility_threshold: float = 0.5,
    ) -> None:
        super().__init__()
        self.core = core
        self.output_width = int(output_width)
        self.output_height = int(output_height)
        self.x_bin_count = int(x_bin_count)
        self.line_half_width_px = float(line_half_width_px)
        self.line_sharpness = float(line_sharpness)
        self.visibility_threshold = float(visibility_threshold)
        self.register_buffer(
            "row_interpolation",
            torch.from_numpy(
                interpolation_matrix(anchor_rows, self.output_height)
            ),
        )
        self.register_buffer(
            "x_pixels",
            torch.arange(self.output_width, dtype=torch.float32).reshape(
                1, 1, self.output_width
            ),
        )

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        row_logits = self.core(image)
        expected_x, visibility = expected_row_coordinates(
            row_logits, self.x_bin_count
        )

        interpolation = self.row_interpolation.transpose(0, 1)
        curve_x = torch.matmul(expected_x, interpolation)
        curve_visibility = torch.matmul(visibility, interpolation)
        curve_x = curve_x * float(self.output_width - 1)

        distance = torch.abs(self.x_pixels - curve_x.unsqueeze(-1))
        line_logit = self.line_sharpness * (
            self.line_half_width_px - distance
        )
        visible = curve_visibility.unsqueeze(-1) >= self.visibility_threshold
        yellow = torch.where(
            visible,
            line_logit,
            torch.full_like(line_logit, -20.0),
        )
        background = torch.zeros_like(yellow)
        white = torch.full_like(yellow, -20.0)
        return torch.stack((background, white, yellow), dim=1)


def expected_row_coordinates(
    logits: torch.Tensor,
    x_bin_count: int,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Return normalized x and visibility from two regression logits."""
    del x_bin_count
    return torch.sigmoid(logits[..., 0]), torch.sigmoid(logits[..., 1])
