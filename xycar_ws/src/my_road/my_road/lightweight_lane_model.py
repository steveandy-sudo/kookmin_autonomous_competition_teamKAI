from __future__ import annotations

import torch
from torch import nn
from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small
from torchvision.models._utils import IntermediateLayerGetter
from torchvision.models.segmentation.lraspp import LRASPP


IMAGE_NET_MEAN = (0.485, 0.456, 0.406)
IMAGE_NET_STD = (0.229, 0.224, 0.225)


def build_lightweight_lane_model(
    *,
    num_classes: int = 3,
    pretrained_backbone: bool = False,
) -> nn.Module:
    """Build LR-ASPP with a MobileNetV3-Small encoder."""
    weights = (
        MobileNet_V3_Small_Weights.DEFAULT if pretrained_backbone else None
    )
    mobilenet = mobilenet_v3_small(weights=weights)
    backbone = IntermediateLayerGetter(
        mobilenet.features,
        return_layers={"3": "low", "12": "high"},
    )
    return LRASPP(
        backbone,
        low_channels=24,
        high_channels=576,
        num_classes=num_classes,
        inter_channels=64,
    )


class LaneLogitsModel(nn.Module):
    """Expose LR-ASPP logits as one tensor for TorchScript inference."""

    def __init__(self, model: nn.Module) -> None:
        super().__init__()
        self.model = model

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.model(image)["out"]
