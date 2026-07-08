from __future__ import annotations

from typing import Optional, Tuple

import torch
from torch import nn

from il_data_tools.model_catalog import normalize_model_type


class PilotNetPolicy(nn.Module):
    def __init__(self, phase_enabled: bool = False) -> None:
        super().__init__()
        self.phase_enabled = phase_enabled
        self.features = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ELU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=3),
            nn.ELU(inplace=True),
            nn.Conv2d(64, 64, kernel_size=3),
            nn.ELU(inplace=True),
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
        )
        head_in = 64 + (1 if phase_enabled else 0)
        self.head = nn.Sequential(
            nn.Linear(head_in, 100),
            nn.ELU(inplace=True),
            nn.Linear(100, 50),
            nn.ELU(inplace=True),
            nn.Linear(50, 10),
            nn.ELU(inplace=True),
            nn.Linear(10, 1),
            nn.Tanh(),
        )

    def forward(self, image: torch.Tensor, phase: Optional[torch.Tensor] = None) -> torch.Tensor:
        x = self.features(image)
        x = append_phase(x, phase, self.phase_enabled)
        return self.head(x).squeeze(-1)


class TorchvisionFeaturePolicy(nn.Module):
    def __init__(self, model_type: str, phase_enabled: bool = False) -> None:
        super().__init__()
        self.model_type = normalize_model_type(model_type)
        self.phase_enabled = phase_enabled
        try:
            from torchvision import models
        except ImportError as exc:  # pragma: no cover
            raise ImportError(
                "torchvision is required for mobilenet_v3_small and resnet18"
            ) from exc

        if self.model_type == "mobilenet_v3_small":
            base = models.mobilenet_v3_small(weights=None)
            self.features = nn.Sequential(
                base.features,
                base.avgpool,
                nn.Flatten(),
            )
            feature_dim = base.classifier[0].in_features
        elif self.model_type == "resnet18":
            base = models.resnet18(weights=None)
            self.features = nn.Sequential(*list(base.children())[:-1], nn.Flatten())
            feature_dim = base.fc.in_features
        else:
            raise ValueError(f"unsupported torchvision model: {model_type}")

        self.head = nn.Sequential(
            nn.Linear(feature_dim + (1 if phase_enabled else 0), 128),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.1),
            nn.Linear(128, 1),
            nn.Tanh(),
        )

    def forward(self, image: torch.Tensor, phase: Optional[torch.Tensor] = None) -> torch.Tensor:
        x = self.features(image)
        x = append_phase(x, phase, self.phase_enabled)
        return self.head(x).squeeze(-1)


class ViTTinyPolicy(nn.Module):
    def __init__(
        self,
        image_size: Tuple[int, int] = (90, 160),
        patch_size: int = 10,
        embed_dim: int = 192,
        depth: int = 4,
        heads: int = 3,
        phase_enabled: bool = False,
    ) -> None:
        super().__init__()
        self.phase_enabled = phase_enabled
        height, width = image_size
        if height % patch_size != 0 or width % patch_size != 0:
            raise ValueError("vit_tiny image_size must be divisible by patch_size")
        self.patch_embed = nn.Conv2d(
            3, embed_dim, kernel_size=patch_size, stride=patch_size
        )
        patch_count = (height // patch_size) * (width // patch_size)
        self.cls_token = nn.Parameter(torch.zeros(1, 1, embed_dim))
        self.pos_embed = nn.Parameter(torch.zeros(1, patch_count + 1, embed_dim))
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=embed_dim,
            nhead=heads,
            dim_feedforward=embed_dim * 4,
            dropout=0.1,
            activation="gelu",
            batch_first=True,
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=depth)
        self.norm = nn.LayerNorm(embed_dim)
        self.head = nn.Sequential(
            nn.Linear(embed_dim + (1 if phase_enabled else 0), 128),
            nn.GELU(),
            nn.Linear(128, 1),
            nn.Tanh(),
        )
        nn.init.trunc_normal_(self.cls_token, std=0.02)
        nn.init.trunc_normal_(self.pos_embed, std=0.02)

    def forward(self, image: torch.Tensor, phase: Optional[torch.Tensor] = None) -> torch.Tensor:
        x = self.patch_embed(image)
        x = x.flatten(2).transpose(1, 2)
        cls = self.cls_token.expand(x.size(0), -1, -1)
        x = torch.cat([cls, x], dim=1)
        x = x + self.pos_embed[:, : x.size(1), :]
        x = self.encoder(x)
        x = self.norm(x[:, 0])
        x = append_phase(x, phase, self.phase_enabled)
        return self.head(x).squeeze(-1)


def append_phase(
    features: torch.Tensor,
    phase: Optional[torch.Tensor],
    phase_enabled: bool,
) -> torch.Tensor:
    if not phase_enabled:
        return features
    if phase is None:
        raise ValueError("phase tensor is required for phase-enabled policy models")
    if phase.dim() == 1:
        phase = phase.unsqueeze(1)
    phase = phase.to(device=features.device, dtype=features.dtype)
    return torch.cat([features, phase], dim=1)


def create_policy_model(
    model_type: str,
    phase_enabled: bool = False,
    image_size: Tuple[int, int] = (90, 160),
) -> nn.Module:
    model_type = normalize_model_type(model_type)
    if model_type == "pilotnet":
        return PilotNetPolicy(phase_enabled=phase_enabled)
    if model_type in {"mobilenet_v3_small", "resnet18"}:
        return TorchvisionFeaturePolicy(model_type, phase_enabled=phase_enabled)
    if model_type == "vit_tiny":
        return ViTTinyPolicy(image_size=image_size, phase_enabled=phase_enabled)
    raise ValueError(f"unsupported model_type: {model_type}")
