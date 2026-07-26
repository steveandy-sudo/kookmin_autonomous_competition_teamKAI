import math

from xycar_map_nav.imu_yaw_calibration_core import (
    YawTracker,
    quaternion_yaw,
    summarize_markers,
)


def test_quaternion_yaw_extracts_planar_rotation():
    yaw = math.radians(73.0)
    assert math.isclose(
        quaternion_yaw(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
        yaw,
        abs_tol=1.0e-9,
    )


def test_tracker_unwraps_a_full_positive_rotation():
    tracker = YawTracker(averaging_window_sec=1.0)
    for index, degrees in enumerate((170.0, 179.0, -179.0, -90.0, 0.0, 90.0, 170.0)):
        sample = tracker.update(
            float(index),
            math.radians(degrees),
            0.0,
        )

    assert math.isclose(
        math.degrees(sample.relative_rad),
        360.0,
        abs_tol=1.0e-6,
    )


def test_zero_and_marker_measurement_use_recent_stable_samples():
    tracker = YawTracker(averaging_window_sec=2.0)
    tracker.update(0.0, 0.0, 0.0)
    tracker.set_zero()
    tracker.update(1.0, math.radians(89.0), 0.0)
    tracker.update(2.0, math.radians(90.0), 0.0)
    tracker.update(3.0, math.radians(91.0), 0.0)

    measured, deviation, count = tracker.marker_measurement()

    assert measured == 90.0
    assert count == 3
    assert 0.0 < deviation < 1.0


def test_marker_summary_recommends_scale_and_passes_good_markers():
    markers = [
        {"expected_deg": 90.0, "measured_deg": 89.0},
        {"expected_deg": -90.0, "measured_deg": -91.0},
        {"expected_deg": 180.0, "measured_deg": 178.0},
        {"expected_deg": 360.0, "measured_deg": 356.0},
    ]

    summary = summarize_markers(markers)

    assert summary["all_markers_within_limits"] is True
    assert 1.0 < summary["suggested_yaw_scale"] < 1.02
    assert summary["suggested_yaw_sign"] == 1.0


def test_empty_marker_summary_is_strict_json_compatible():
    summary = summarize_markers([])

    assert summary["maximum_absolute_error_deg"] is None
    assert summary["suggested_yaw_scale"] is None
    assert summary["all_markers_within_limits"] is False
