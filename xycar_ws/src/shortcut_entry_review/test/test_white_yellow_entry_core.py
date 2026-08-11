import cv2
import numpy as np

from shortcut_entry_review.white_yellow_entry_core import (
    WhiteYellowEntrySelector,
    choose_entry_bases,
)


def masks_with_boundaries(
    *,
    width=640,
    height=480,
    white_bottom=190,
    yellow_bottom=380,
):
    white = np.zeros((height, width), dtype=np.uint8)
    yellow = np.zeros_like(white)
    cv2.line(
        white,
        (white_bottom, height - 1),
        (white_bottom - 90, 80),
        12,
    )
    cv2.line(
        yellow,
        (yellow_bottom, height - 1),
        (yellow_bottom - 70, 80),
        12,
    )
    return white, yellow


def test_white_left_and_yellow_right_make_midpoint_path():
    white, yellow = masks_with_boundaries()

    result = WhiteYellowEntrySelector().process(white, yellow)

    assert result.valid, result.reason
    assert len(result.path_pixels) == 32
    assert result.median_separation_px > 100.0
    for x, y in result.path_pixels:
        assert 100.0 < x < 400.0
        assert 0.0 <= y < white.shape[0]


def test_white_right_of_yellow_is_rejected():
    white, yellow = masks_with_boundaries(
        white_bottom=470,
        yellow_bottom=310,
    )

    result = WhiteYellowEntrySelector().process(white, yellow)

    assert not result.valid
    assert "white boundary left" in result.reason


def test_both_colors_are_required():
    white, yellow = masks_with_boundaries()
    selector = WhiteYellowEntrySelector()

    no_white = selector.process(np.zeros_like(white), yellow)
    selector.reset()
    no_yellow = selector.process(white, np.zeros_like(yellow))

    assert not no_white.valid
    assert not no_yellow.valid


def test_short_yellow_dash_can_pair_with_continuous_white_boundary():
    white, yellow = masks_with_boundaries()
    yellow.fill(0)
    cv2.line(yellow, (350, 380), (334, 325), 12)

    result = WhiteYellowEntrySelector().process(white, yellow)

    assert result.valid, result.reason
    assert len(result.yellow.centers) >= 2
    assert len(result.path_pixels) == 32


def test_single_window_yellow_noise_is_rejected():
    white, yellow = masks_with_boundaries()
    yellow.fill(0)
    cv2.circle(yellow, (350, 390), 5, -1)

    result = WhiteYellowEntrySelector().process(white, yellow)

    assert not result.valid
    assert "yellow boundary" in result.reason


def test_nearest_white_peak_left_of_yellow_is_selected():
    white, yellow = masks_with_boundaries(white_bottom=220, yellow_bottom=390)
    cv2.line(white, (45, 479), (30, 80), 12)

    white_base, yellow_base = choose_entry_bases(
        white,
        yellow,
        WhiteYellowEntrySelector().config,
    )

    assert white_base is not None
    assert yellow_base is not None
    assert white_base > 150
    assert white_base < yellow_base


def test_temporal_base_prefers_same_boundary_when_multiple_are_valid():
    white, yellow = masks_with_boundaries(white_bottom=190, yellow_bottom=390)
    cv2.line(white, (285, 479), (260, 80), 12)
    selector = WhiteYellowEntrySelector()
    selector.previous_white_base = 190.0
    selector.previous_yellow_base = 390.0

    white_base, yellow_base = choose_entry_bases(
        white,
        yellow,
        selector.config,
        previous_white_base=selector.previous_white_base,
        previous_yellow_base=selector.previous_yellow_base,
    )

    assert white_base is not None
    assert yellow_base is not None
    assert abs(white_base - 190) < abs(white_base - 285)
