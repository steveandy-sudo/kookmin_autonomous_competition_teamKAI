import math

import pytest

from my_drive.corner_survey_core import measure_sector
from my_drive.corner_survey_core import waypoint_distances


def test_measure_sector_uses_robust_near_quantile_and_ignores_invalid_values():
    ranges = [float("inf")] * 9
    ranges[3:7] = [1.2, 1.0, 1.1, 8.0]
    measurement = measure_sector(
        ranges,
        angle_min_rad=math.radians(-40.0),
        angle_increment_rad=math.radians(10.0),
        range_min_m=0.1,
        range_max_m=6.0,
        center_angle_rad=0.0,
        half_angle_rad=math.radians(20.0),
        distance_quantile=0.20,
    )

    assert measurement.distance_m == pytest.approx(1.0)
    assert measurement.point_count == 3
    assert math.degrees(measurement.angular_span_rad) == pytest.approx(20.0)


def test_measure_sector_reports_unavailable_when_no_valid_scan_points():
    measurement = measure_sector(
        [float("inf"), float("nan"), 0.01],
        angle_min_rad=-0.1,
        angle_increment_rad=0.1,
        range_min_m=0.1,
        range_max_m=6.0,
        center_angle_rad=0.0,
        half_angle_rad=0.2,
    )

    assert not measurement.available
    assert measurement.point_count == 0


def test_waypoint_distances_report_segments_and_closed_total():
    incoming, cumulative, total = waypoint_distances(
        [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)],
        closed=True,
    )

    assert incoming == pytest.approx((0.0, 3.0, 4.0))
    assert cumulative == pytest.approx((0.0, 3.0, 7.0))
    assert total == pytest.approx(12.0)
