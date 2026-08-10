import cv2
import numpy as np
import torch

from my_lane.far_centerline_model import (
    FarCenterlineMaskAdapter,
    FarCenterlineModel,
    anchor_rows_for_stride,
)
from my_lane.train_far_centerline import far_centerline_targets


def test_white_boundaries_extend_validated_yellow_toward_far_field():
    height, width = 80, 120
    yellow = np.zeros((height, width), dtype=np.uint8)
    white = np.zeros_like(yellow)
    cv2.line(white, (30, 20), (15, 79), 255, 3)
    cv2.line(white, (70, 20), (105, 79), 255, 3)
    cv2.line(yellow, (52, 42), (60, 79), 255, 3)
    anchors = anchor_rows_for_stride(height)

    targets, quality, source = far_centerline_targets(
        yellow, white, anchors, 64, white_extension=True
    )

    assert np.any(source == 2)
    assert np.min(np.asarray(anchors)[source == 2]) < 42
    assert np.all(quality[source == 2] == 0.75)
    assert np.all(targets[source > 0] < 64)


def test_unvalidated_single_white_boundary_does_not_extend_yellow():
    height, width = 80, 120
    yellow = np.zeros((height, width), dtype=np.uint8)
    white = np.zeros_like(yellow)
    cv2.line(white, (20, 20), (10, 79), 255, 3)
    cv2.line(yellow, (52, 42), (60, 79), 255, 3)
    anchors = anchor_rows_for_stride(height)

    _, _, source = far_centerline_targets(
        yellow, white, anchors, 64, white_extension=True
    )

    assert not np.any(source == 2)


def test_far_model_and_mask_adapter_shapes():
    core = FarCenterlineModel(
        x_bin_count=128, pretrained_backbone=False
    ).eval()
    adapter = FarCenterlineMaskAdapter(
        core,
        output_width=512,
        output_height=288,
        x_bin_count=128,
    ).eval()
    image = torch.zeros(1, 3, 288, 512)
    with torch.inference_mode():
        logits = core(image)
        mask = adapter(image)
    assert logits.shape == (1, 72, 129)
    assert mask.shape == (1, 3, 288, 512)
    assert torch.all(mask[:, 1] < mask[:, 0])
