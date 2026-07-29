import math

import pytest

from xycar_map_nav.route_start_pose_publisher import route_start_pose


def test_route_start_pose_uses_first_point_and_outgoing_heading(tmp_path):
    waypoint_file = tmp_path / "waypoints.yaml"
    waypoint_file.write_text(
        "\n".join(
            [
                "frame_id: map",
                "closed: true",
                "waypoints:",
                "- name: first",
                "  x: 1.0",
                "  y: 2.0",
                "- name: second",
                "  x: 1.0",
                "  y: 5.0",
            ]
        ),
        encoding="utf-8",
    )
    x, y, yaw = route_start_pose(waypoint_file)
    assert math.isclose(x, 1.0)
    assert math.isclose(y, 2.0)
    assert math.isclose(yaw, math.pi / 2.0)


def test_route_start_pose_rejects_incomplete_route(tmp_path):
    waypoint_file = tmp_path / "waypoints.yaml"
    waypoint_file.write_text(
        "frame_id: map\nclosed: true\nwaypoints: []\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="at least two"):
        route_start_pose(waypoint_file)
