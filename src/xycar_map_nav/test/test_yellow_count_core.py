import numpy as np

from xycar_map_nav.yellow_count_core import (
    YellowBandPassCounter,
    count_band_bounds,
    yellow_mask_occupies_count_band,
)


def test_count_band_ignores_components_that_are_only_below_the_band():
    mask = np.zeros((660, 640), dtype=np.uint8)
    top, _, bottom = count_band_bounds(
        mask.shape[0], line_ratio=0.68, half_height_px=10
    )
    mask[bottom + 5 : bottom + 30, 300:320] = 255
    assert not yellow_mask_occupies_count_band(
        mask,
        line_ratio=0.68,
        half_height_px=10,
        component_minimum_area_px=80,
    )


def test_count_band_detects_a_large_component_touching_the_band():
    mask = np.zeros((660, 640), dtype=np.uint8)
    top, _, bottom = count_band_bounds(
        mask.shape[0], line_ratio=0.68, half_height_px=10
    )
    mask[top - 5 : bottom + 6, 300:320] = 255
    assert yellow_mask_occupies_count_band(
        mask,
        line_ratio=0.68,
        half_height_px=10,
        component_minimum_area_px=80,
    )


def test_two_debounced_enter_leave_pulses_trigger_once():
    counter = YellowBandPassCounter(
        target=2,
        visible_frames=2,
        absent_frames=2,
    )
    events = [counter.update(value) for value in (
        False,
        True,
        True,
        False,
        False,
        True,
        True,
        False,
        False,
    )]
    assert events[2] == (True, False, False)
    assert events[4] == (False, True, False)
    assert events[6] == (True, False, False)
    assert events[8] == (False, True, True)
    assert counter.passed == 2
    assert counter.triggered


def test_first_confirmed_dash_triggers_on_first_absent_frame():
    counter = YellowBandPassCounter(
        target=1,
        visible_frames=2,
        absent_frames=1,
    )

    assert counter.update(True) == (False, False, False)
    assert counter.update(True) == (True, False, False)
    assert counter.update(False) == (False, True, True)
    assert counter.passed == 1
    assert counter.triggered


def test_two_selected_dashes_trigger_on_the_second_absent_frame():
    counter = YellowBandPassCounter(
        target=2,
        visible_frames=2,
        absent_frames=1,
    )

    events = [counter.update(value) for value in (
        True,
        True,
        False,
        True,
        True,
        False,
    )]
    assert events[2] == (False, True, False)
    assert events[5] == (False, True, True)
    assert counter.passed == 2
    assert counter.triggered


def test_a_dash_below_the_band_does_not_merge_with_the_next_crossing():
    counter = YellowBandPassCounter(
        target=2,
        visible_frames=2,
        absent_frames=2,
    )
    # Only occupancy of the narrow band is supplied. A previous dash can
    # remain visible below it while these False frames separate crossings.
    for value in (True, True, False, False):
        counter.update(value)
    assert counter.passed == 1
    for value in (True, True, False, False):
        _, _, triggered = counter.update(value)
    assert counter.passed == 2
    assert triggered
