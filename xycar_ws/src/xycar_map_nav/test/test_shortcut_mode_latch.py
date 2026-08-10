from xycar_map_nav.shortcut_mode_latch import ShortcutModeConfig
from xycar_map_nav.shortcut_mode_latch import ShortcutModeEvent
from xycar_map_nav.shortcut_mode_latch import ShortcutModeLatch


def make_latch():
    return ShortcutModeLatch(
        ShortcutModeConfig(
            minimum_confidence=0.50,
            required_frames=2,
            rearm_absence_sec=1.0,
        )
    )


def test_two_left_4_frames_latch_one_shortcut_run():
    latch = make_latch()

    assert latch.observe(
        now_sec=1.0, detected=True, confidence=0.70
    ) == ShortcutModeEvent.NONE
    assert latch.observe(
        now_sec=1.3, detected=True, confidence=0.80
    ) == ShortcutModeEvent.STARTED
    assert latch.active
    assert not latch.armed


def test_low_confidence_and_nonconsecutive_frames_do_not_start():
    latch = make_latch()

    latch.observe(now_sec=1.0, detected=True, confidence=0.70)
    latch.observe(now_sec=1.3, detected=False, confidence=0.0)
    event = latch.observe(now_sec=1.6, detected=True, confidence=0.49)

    assert event == ShortcutModeEvent.NONE
    assert not latch.active


def test_completion_requires_absence_before_rearming():
    latch = make_latch()
    latch.observe(now_sec=1.0, detected=True, confidence=0.70)
    latch.observe(now_sec=1.3, detected=True, confidence=0.80)

    assert latch.finish() == ShortcutModeEvent.FINISHED
    assert not latch.active
    assert not latch.armed
    assert latch.update(now_sec=2.0) == ShortcutModeEvent.NONE
    assert latch.update(now_sec=2.31) == ShortcutModeEvent.REARMED
    assert latch.armed


def test_active_latch_ignores_repeated_left_4_detections():
    latch = make_latch()
    latch.observe(now_sec=1.0, detected=True, confidence=0.70)
    latch.observe(now_sec=1.3, detected=True, confidence=0.80)

    event = latch.observe(now_sec=4.0, detected=True, confidence=0.90)

    assert event == ShortcutModeEvent.NONE
    assert latch.active


def test_external_traffic_sequencer_can_start_after_left_disappears():
    latch = make_latch()

    assert latch.start(
        now_sec=2.0,
        confidence=0.82,
    ) == ShortcutModeEvent.STARTED
    assert latch.active
    assert not latch.armed
    assert latch.confidence == 0.82

    assert latch.start(
        now_sec=2.1,
        confidence=0.90,
    ) == ShortcutModeEvent.NONE
