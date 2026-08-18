from pathlib import Path

import cv2
import numpy as np
import torch

from lane_seg_control.row_centerline_model import (
    RowCenterlineMaskAdapter,
    RowCenterlineModel,
    interpolation_matrix,
    make_anchor_rows,
)
from lane_seg_control.train_row_centerline import (
    batch_totals,
    row_targets_from_mask,
    yellow_polygon_mask,
)


def test_yellow_polygon_mask_ignores_white_class(tmp_path: Path):
    label = tmp_path / "sample.txt"
    label.write_text(
        "0 0.05 0.05 0.30 0.05 0.30 0.30 0.05 0.30\n"
        "1 0.45 0.10 0.55 0.10 0.55 0.90 0.45 0.90\n",
        encoding="utf-8",
    )
    mask = yellow_polygon_mask(label, 100, 80, yellow_class_id=1)
    assert int(mask[40, 50]) == 255
    assert int(mask[10, 10]) == 0


def test_row_targets_connect_only_between_visible_dash_fragments():
    mask = np.zeros((20, 40), dtype=np.uint8)
    cv2.line(mask, (8, 5), (10, 7), 255, 1)
    cv2.line(mask, (20, 12), (22, 15), 255, 1)
    anchors = tuple(range(2, 19))
    targets = row_targets_from_mask(mask, anchors, x_bin_count=40)
    assert np.all(targets[:3] == 40)
    assert np.all(targets[3:14] < 40)
    assert np.all(targets[14:] == 40)
    assert targets[8] > targets[3]


def test_interpolation_matrix_does_not_extrapolate():
    matrix = interpolation_matrix((2, 5, 8), 11)
    np.testing.assert_allclose(matrix[:2], 0.0)
    np.testing.assert_allclose(matrix[9:], 0.0)
    np.testing.assert_allclose(matrix[2:9].sum(axis=1), 1.0)
    np.testing.assert_allclose(matrix[5], [0.0, 1.0, 0.0])


def test_model_and_legacy_mask_adapter_shapes():
    anchors = make_anchor_rows(144)
    core = RowCenterlineModel(
        row_count=len(anchors),
        x_bin_count=128,
        pretrained_backbone=False,
    ).eval()
    adapter = RowCenterlineMaskAdapter(
        core,
        anchor_rows=anchors,
        output_width=256,
        output_height=144,
        x_bin_count=128,
    ).eval()
    image = torch.zeros(1, 3, 144, 256)
    with torch.inference_mode():
        row_logits = core(image)
        mask_logits = adapter(image)
    assert row_logits.shape == (1, len(anchors), 2)
    assert mask_logits.shape == (1, 3, 144, 256)
    assert torch.all(mask_logits[:, 1] < mask_logits[:, 0])


class _ConstantRowPrediction(torch.nn.Module):
    def __init__(self, row_count: int, visibility: float):
        super().__init__()
        row_logits = torch.zeros(row_count, 2)
        row_logits[:, 1] = torch.logit(torch.tensor(visibility))
        self.register_buffer("row_logits", row_logits)

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.row_logits.unsqueeze(0).expand(image.shape[0], -1, -1)


def test_mask_adapter_rejects_low_visibility_coordinates():
    anchors = (2, 5, 8)
    image = torch.zeros(1, 3, 11, 16)
    hidden_adapter = RowCenterlineMaskAdapter(
        _ConstantRowPrediction(len(anchors), 0.10),
        anchor_rows=anchors,
        output_width=16,
        output_height=11,
    ).eval()
    visible_adapter = RowCenterlineMaskAdapter(
        _ConstantRowPrediction(len(anchors), 0.90),
        anchor_rows=anchors,
        output_width=16,
        output_height=11,
    ).eval()

    with torch.inference_mode():
        hidden_classes = torch.argmax(hidden_adapter(image), dim=1)
        visible_classes = torch.argmax(visible_adapter(image), dim=1)

    assert not torch.any(hidden_classes == 2)
    assert torch.any(visible_classes[:, 2:9] == 2)
    assert not torch.any(visible_classes[:, :2] == 2)
    assert not torch.any(visible_classes[:, 9:] == 2)


def test_metrics_do_not_overflow_under_float16():
    anchors = make_anchor_rows(144)
    logits = torch.zeros(400, len(anchors), 2, dtype=torch.float16)
    target = torch.full((400, len(anchors)), 64, dtype=torch.int64)
    totals = batch_totals(
        logits,
        target,
        anchor_rows=anchors,
        image_height=144,
        x_bin_count=128,
        image_width=256,
    )
    assert np.isfinite(totals.absolute_error)


def test_real_launch_uses_packaged_row_model_and_no_white_lane():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "lane_seg_row_centerline_low_latency_real.launch.py"
    )
    source = launch_path.read_text(encoding="utf-8")
    assert "kookmin_yellow_row_centerline_256x144.pt" in source
    assert '"white_confidence": "0.99"' in source
    assert 'default_value="20.0"' in source
