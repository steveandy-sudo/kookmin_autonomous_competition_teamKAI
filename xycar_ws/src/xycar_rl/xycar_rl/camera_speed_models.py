from __future__ import annotations

from pathlib import Path

import torch
from torch import nn

from xycar_rl.models import ResNet18Encoder, checkpoint_payload


DEFAULT_MIN_SPEED_COMMAND = 4.0
DEFAULT_MAX_SPEED_COMMAND = 10.0


def normalize_speed_command(
    speed_command: float,
    min_speed_command: float = DEFAULT_MIN_SPEED_COMMAND,
    max_speed_command: float = DEFAULT_MAX_SPEED_COMMAND,
) -> float:
    low = float(min_speed_command)
    high = float(max_speed_command)
    if high <= low:
        raise ValueError("max_speed_command must be greater than min_speed_command")
    return max(-1.0, min(1.0, 2.0 * (float(speed_command) - low) / (high - low) - 1.0))


def denormalize_speed_command(
    speed_norm: float,
    min_speed_command: float = DEFAULT_MIN_SPEED_COMMAND,
    max_speed_command: float = DEFAULT_MAX_SPEED_COMMAND,
) -> float:
    low = float(min_speed_command)
    high = float(max_speed_command)
    if high <= low:
        raise ValueError("max_speed_command must be greater than min_speed_command")
    normalized = max(-1.0, min(1.0, float(speed_norm)))
    return low + 0.5 * (normalized + 1.0) * (high - low)


class CameraSpeedActor(nn.Module):
    """Camera-only actor returning normalized steering and speed actions."""

    def __init__(self, temporal_frames: int = 1) -> None:
        super().__init__()
        self.temporal_frames = int(temporal_frames)
        if self.temporal_frames not in {1, 2}:
            raise ValueError("temporal_frames must be 1 or 2")
        self.image_encoder = ResNet18Encoder(input_channels=3 * self.temporal_frames)
        self.head = nn.Sequential(
            nn.Linear(self.image_encoder.feature_dim, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.1),
            nn.Linear(128, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 2),
            nn.Tanh(),
        )

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.head(self.image_encoder(image))

    def disable_dropout(self) -> None:
        self.head[2] = nn.Identity()


class CompactCameraSpeedActor(nn.Module):
    """Small canonical-mask actor intended for limited sim DAgger datasets."""

    def __init__(self, temporal_frames: int = 1) -> None:
        super().__init__()
        self.temporal_frames = int(temporal_frames)
        if self.temporal_frames not in {1, 2}:
            raise ValueError("temporal_frames must be 1 or 2")
        self.image_encoder = nn.Sequential(
            nn.Conv2d(3 * self.temporal_frames, 24, 5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(24, 48, 5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(48, 64, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 96, 3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((3, 5)),
            nn.Flatten(),
        )
        self.head = nn.Sequential(
            nn.Linear(96 * 3 * 5, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.1),
            nn.Linear(128, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 2),
            nn.Tanh(),
        )

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.head(self.image_encoder(image))

    def disable_dropout(self) -> None:
        self.head[2] = nn.Identity()


def initialize_temporal_actor(
    source: CameraSpeedActor,
    temporal_frames: int = 2,
) -> CameraSpeedActor:
    """Expand a single-frame actor while preserving its current-frame behavior."""
    if source.temporal_frames != 1:
        raise ValueError("source actor must use one temporal frame")
    target = CameraSpeedActor(temporal_frames=temporal_frames)
    source_state = source.state_dict()
    target_state = target.state_dict()
    first_conv_key = "image_encoder.features.0.weight"
    for key, value in source_state.items():
        if key != first_conv_key:
            target_state[key] = value.detach().clone()
    source_conv = source_state[first_conv_key]
    expanded_conv = torch.zeros_like(target_state[first_conv_key])
    expanded_conv[:, -3:, :, :] = source_conv
    target_state[first_conv_key] = expanded_conv
    target.load_state_dict(target_state, strict=True)
    return target


class CameraOnlyCriticEncoder(nn.Module):
    def __init__(self, temporal_frames: int = 1) -> None:
        super().__init__()
        self.temporal_frames = int(temporal_frames)
        if self.temporal_frames not in {1, 2}:
            raise ValueError("temporal_frames must be 1 or 2")
        self.image = nn.Sequential(
            nn.Conv2d(
                3 * self.temporal_frames,
                24,
                kernel_size=5,
                stride=2,
                padding=2,
            ),
            nn.ReLU(inplace=True),
            nn.Conv2d(24, 48, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((3, 5)),
            nn.Flatten(),
        )
        self.output_dim = 64 * 3 * 5

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.image(image)


class CameraSpeedQNetwork(nn.Module):
    def __init__(self, temporal_frames: int = 1) -> None:
        super().__init__()
        self.encoder = CameraOnlyCriticEncoder(temporal_frames=temporal_frames)
        self.head = nn.Sequential(
            nn.Linear(self.encoder.output_dim + 2, 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, 1),
        )

    def forward(self, image: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        return self.head(torch.cat([self.encoder(image), action], dim=1))


class CameraSpeedTwinCritic(nn.Module):
    def __init__(self, temporal_frames: int = 1) -> None:
        super().__init__()
        self.q1 = CameraSpeedQNetwork(temporal_frames=temporal_frames)
        self.q2 = CameraSpeedQNetwork(temporal_frames=temporal_frames)

    def forward(self, image: torch.Tensor, action: torch.Tensor):
        return self.q1(image, action), self.q2(image, action)


def load_camera_speed_actor(
    checkpoint_path: str | Path,
    *,
    device: str | torch.device = "cpu",
) -> tuple[nn.Module, dict]:
    payload = checkpoint_payload(checkpoint_path, device)
    model_type = payload.get("model_type")
    if model_type not in {
        "camera_speed_resnet18",
        "camera_speed_temporal_resnet18",
        "camera_speed_compact",
        "camera_speed_temporal_compact",
    }:
        raise ValueError(
            "expected a camera-speed ResNet18 checkpoint, got "
            f"{model_type!r}"
        )
    temporal_frames = int(
        payload.get(
            "temporal_frames",
            2 if "temporal" in str(model_type) else 1,
        )
    )
    actor = (
        CompactCameraSpeedActor(temporal_frames=temporal_frames)
        if "compact" in str(model_type)
        else CameraSpeedActor(temporal_frames=temporal_frames)
    )
    actor.disable_dropout()
    state_dict = payload.get("actor_state_dict", payload.get("state_dict"))
    if not isinstance(state_dict, dict):
        raise ValueError("camera-speed checkpoint has no actor state dict")
    actor.load_state_dict(state_dict, strict=True)
    actor.to(device).eval()
    return actor, payload
