import pytest

from shortcut_entry_review.trigger_detector import (
    Left4TriggerDetector,
    TriggerConfig,
)


def test_s_is_second_missing_frame_after_two_visible_frames():
    detector = Left4TriggerDetector()

    states = [detector.update(value) for value in (0.9, 0.8, 0.0, 0.0)]

    assert not states[0].confirmed
    assert states[1].confirmed
    assert states[2].absent_streak == 1
    assert not states[2].triggered
    assert states[3].absent_streak == 2
    assert states[3].triggered


def test_unconfirmed_dropout_does_not_advance_absence_counter():
    detector = Left4TriggerDetector()

    first = detector.update(0.9)
    missing = detector.update(0.0)

    assert first.visible_streak == 1
    assert missing.visible_streak == 0
    assert missing.absent_streak == 0
    assert not missing.triggered


def test_visible_frame_cancels_an_incomplete_absence_sequence():
    detector = Left4TriggerDetector()
    detector.update(0.9)
    detector.update(0.9)
    first_missing = detector.update(0.0)
    recovered = detector.update(0.7)
    missing_again = detector.update(0.0)

    assert first_missing.absent_streak == 1
    assert recovered.absent_streak == 0
    assert missing_again.absent_streak == 1
    assert not missing_again.triggered


@pytest.mark.parametrize(
    "kwargs",
    [
        {"minimum_confidence": -0.1},
        {"minimum_confidence": 1.1},
        {"required_visible_frames": 0},
        {"required_absent_frames": 0},
    ],
)
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        TriggerConfig(**kwargs)
