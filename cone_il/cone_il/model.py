import torch
import torch.nn as nn


class ConeBCNet(nn.Module):
    """Small CNN for behavior cloning: image -> normalized steering angle.

    Output is tanh-normalized in [-1, 1]. Convert it to degrees by multiplying
    by max_steer_deg in inference.
    """

    # 설명: 콘 주행 모방학습 CNN 구조의 합성곱 계층과 출력 계층을 구성한다.
    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2, padding=2),
            nn.BatchNorm2d(24),
            nn.ReLU(inplace=True),
            nn.Conv2d(24, 36, kernel_size=5, stride=2, padding=2),
            nn.BatchNorm2d(36),
            nn.ReLU(inplace=True),
            nn.Conv2d(36, 48, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(48),
            nn.ReLU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(64, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.10),
            nn.Linear(64, 1),
            nn.Tanh(),
        )

    # 설명: 신경망의 순전파를 수행해 입력 이미지에서 조향 출력을 계산한다.
    def forward(self, x):
        x = self.features(x)
        return self.head(x).squeeze(1)


class TorchvisionSteeringNet(nn.Module):
    """Torchvision backbone wrapper for behavior cloning steering output."""

    # 설명: 이미지넷 사전학습 모델을 조향각 회귀 모델처럼 사용할 수 있게 감싼다.
    def __init__(self, backbone: nn.Module, normalize: bool = True):
        super().__init__()
        self.backbone = backbone
        self.normalize = normalize
        self.register_buffer('mean', torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1))
        self.register_buffer('std', torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1))

    # 설명: 입력 이미지를 필요하면 이미지넷 기준으로 정규화한 뒤 조향값을 -1~1 범위로 출력한다.
    def forward(self, x):
        if self.normalize:
            x = (x - self.mean) / self.std
        return torch.tanh(self.backbone(x)).squeeze(1)


# 설명: 선택한 torchvision 모델의 마지막 분류 계층을 조향각 1개 출력으로 교체한다.
def _build_torchvision_backbone(arch: str, pretrained: bool) -> nn.Module:
    from torchvision import models

    if arch == 'mobilenet_v3_small':
        weights = models.MobileNet_V3_Small_Weights.DEFAULT if pretrained else None
        backbone = models.mobilenet_v3_small(weights=weights)
    elif arch == 'mobilenet_v3_large':
        weights = models.MobileNet_V3_Large_Weights.DEFAULT if pretrained else None
        backbone = models.mobilenet_v3_large(weights=weights)
    elif arch == 'efficientnet_b0':
        weights = models.EfficientNet_B0_Weights.DEFAULT if pretrained else None
        backbone = models.efficientnet_b0(weights=weights)
    else:
        raise ValueError(f'Unsupported model arch: {arch}')

    in_features = backbone.classifier[-1].in_features
    backbone.classifier[-1] = nn.Linear(in_features, 1)
    return backbone


# 설명: 학습 스크립트에서 이름만 넘기면 기존 CNN이나 더 큰 백본 모델을 생성한다.
def build_model(arch: str = 'cnn', pretrained: bool = False) -> nn.Module:
    if arch == 'cnn':
        if pretrained:
            raise ValueError('pretrained=True is only supported for torchvision model arches')
        return ConeBCNet()

    backbone = _build_torchvision_backbone(arch, pretrained=pretrained)
    return TorchvisionSteeringNet(backbone, normalize=pretrained)
