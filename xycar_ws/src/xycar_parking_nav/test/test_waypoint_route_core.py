import math
from pathlib import Path

import pytest
import yaml

from xycar_parking_nav.waypoint_route_core import (
    ReferenceLocation,
    RouteWaypoint,
    normalize_reverse_waypoint_ranges,
    parking_mission_document,
    parse_reverse_waypoint_ranges,
    reference_locations_from_mission,
    route_waypoints_from_items,
    waypoint_document,
    waypoint_headings,
)


PACKAGE = Path(__file__).resolve().parents[1]


def _route():
    return [
        RouteWaypoint("START", 1.0, 2.0),
        RouteWaypoint("TURN", 2.0, 2.0),
        RouteWaypoint("FINISH", 2.0, 3.0),
    ]


def test_open_route_headings_face_the_next_leg_and_final_arrival():
    headings = waypoint_headings(_route(), closed=False)

    assert headings[0] == pytest.approx(0.0)
    assert headings[1] == pytest.approx(math.pi / 2.0)
    assert headings[2] == pytest.approx(math.pi / 2.0)


def test_reverse_ranges_flip_body_heading_and_make_destination_reverse_only():
    route = [
        RouteWaypoint("WP_00", 0.0, 0.0),
        RouteWaypoint("WP_01", 1.0, 0.0),
        RouteWaypoint("WP_02", 2.0, 0.0),
        RouteWaypoint("WP_03", 3.0, 0.0),
    ]

    headings = waypoint_headings(
        route, closed=False, reverse_ranges=((1, 2),)
    )
    mission = parking_mission_document(
        route,
        frame_id="map",
        closed=False,
        reverse_ranges=((1, 2),),
    )["mission"]

    assert headings[1] == pytest.approx(math.pi)
    assert headings[2] == pytest.approx(math.pi)
    boundary = mission["steps"][0]
    reverse_target = mission["steps"][1]
    assert boundary["name"] == "WP_01"
    assert boundary.get("precise_goal", False) is False
    assert boundary["yaw"] == pytest.approx(math.pi)
    assert reverse_target["name"] == "WP_02"
    assert reverse_target["reverse_only"] is True
    assert reverse_target.get("precise_goal", False) is False
    assert reverse_target["yaw"] == pytest.approx(math.pi)
    assert mission["reverse_waypoint_ranges"] == ["1-2"]


def test_reverse_range_parser_accepts_cli_format_and_rejects_bad_bounds():
    parsed = parse_reverse_waypoint_ranges("4-5, 8:9")

    assert parsed == ((4, 5), (8, 9))
    assert normalize_reverse_waypoint_ranges(
        ((4, 5), (5, 7)), 10
    ) == ((4, 7),)
    with pytest.raises(ValueError, match="outside"):
        normalize_reverse_waypoint_ranges(parsed, 6)
    with pytest.raises(ValueError, match="START < END"):
        parse_reverse_waypoint_ranges("5-4")


def test_clicked_points_convert_to_base_frame_nav2_mission():
    mission = parking_mission_document(_route(), frame_id="map", closed=False)[
        "mission"
    ]

    assert mission["base_from_reference_x_m"] == 0.0
    assert mission["initial_pose"] == {
        "name": "START",
        "x": 1.0,
        "y": 2.0,
        "yaw": 0.0,
    }
    assert [step["name"] for step in mission["steps"]] == ["TURN", "FINISH"]
    assert all(step["allow_reverse"] is True for step in mission["steps"])
    assert mission["steps"][0].get("precise_goal", False) is False
    assert mission["steps"][-1].get("precise_goal", False) is False
    assert mission["steps"][-1]["hold_sec"] == pytest.approx(1.0)
    assert mission["steps"][-1]["maximum_retries"] == 2


def test_click_near_surveyed_parking_box_becomes_strict_snapped_goal():
    route = [
        RouteWaypoint("START", 1.0, 0.0),
        RouteWaypoint("NEAR_A", 0.18, 4.03),
        RouteWaypoint("EXIT", 1.0, 4.0),
    ]
    references = [
        ReferenceLocation("PREVIOUS_START", 1.8, 0.8, math.pi),
        ReferenceLocation("A_PARK", 0.0, 4.1, 0.02),
        ReferenceLocation("B_PARK", 2.1, 3.2, -1.5),
    ]

    mission = parking_mission_document(
        route,
        frame_id="map",
        closed=False,
        reference_locations=references,
        parking_snap_radius_m=0.45,
    )["mission"]

    parking = mission["steps"][0]
    assert parking["name"] == "A_PARK"
    assert parking["x"] == pytest.approx(0.0)
    assert parking["y"] == pytest.approx(4.1)
    assert parking["yaw"] == pytest.approx(0.02)
    assert parking["parking_goal"] is True
    assert parking["precise_goal"] is True
    assert parking["allow_reverse"] is True


def test_waypoint_before_parking_faces_outward_for_reverse_entry():
    route = [
        RouteWaypoint("START", 1.8, 0.8),
        RouteWaypoint("ENTRY", 1.0, 4.7),
        RouteWaypoint("NEAR_A", 0.02, 4.08),
        RouteWaypoint("EXIT", 2.4, 4.8),
    ]
    references = [
        ReferenceLocation("PREVIOUS_START", 1.8, 0.8, math.pi),
        ReferenceLocation("A_PARK", 0.0, 4.1, 0.02),
        ReferenceLocation("B_PARK", 2.1, 3.2, -1.5),
    ]

    steps = parking_mission_document(
        route,
        frame_id="map",
        closed=False,
        reference_locations=references,
    )["mission"]["steps"]

    entry, parking = steps[:2]
    assert entry["name"] == "ENTRY"
    assert entry.get("precise_goal", False) is False
    assert entry["allow_reverse"] is True
    assert entry["yaw"] == pytest.approx(math.atan2(0.6, 1.0))
    assert parking["name"] == "A_PARK"
    assert parking["allow_reverse"] is True


def test_b_parking_entry_is_exact_forward_alignment_before_reverse():
    route = [
        RouteWaypoint("START", 1.8, 0.8),
        RouteWaypoint("B_APPROACH", 2.82, 4.39),
        RouteWaypoint("WP_18", 1.984, 2.333),
        RouteWaypoint("NEAR_B", 1.998, 3.456),
        RouteWaypoint("EXIT", 3.05, 1.81),
    ]
    references = [
        ReferenceLocation("PREVIOUS_START", 1.8, 0.8, math.pi),
        ReferenceLocation("A_PARK", 0.0, 4.1, 0.02),
        ReferenceLocation("B_PARK", 2.059, 3.186, -1.503),
    ]

    steps = parking_mission_document(
        route,
        frame_id="map",
        closed=False,
        reference_locations=references,
    )["mission"]["steps"]

    entry = next(step for step in steps if step["name"] == "WP_18")
    parking = next(step for step in steps if step["name"] == "B_PARK")
    exit_step = next(step for step in steps if step["name"] == "EXIT")
    assert entry["precise_goal"] is True
    assert entry["allow_reverse"] is False
    assert entry["hold_sec"] == pytest.approx(0.0)
    assert entry["yaw"] == pytest.approx(
        math.atan2(entry["y"] - parking["y"], entry["x"] - parking["x"])
    )
    assert parking["reverse_only"] is True
    assert exit_step["allow_reverse"] is True


def test_explicit_reverse_range_takes_priority_across_b_parking():
    route = [
        RouteWaypoint("START", 1.8, 0.8),
        RouteWaypoint("WP_21", 3.01, 1.98),
        RouteWaypoint("WP_22", 2.62, 2.29),
        RouteWaypoint("WP_23", 2.21, 2.72),
        RouteWaypoint("WP_24", 2.06, 3.22),
        RouteWaypoint("WP_25", 2.01, 3.72),
        RouteWaypoint("WP_26", 2.08, 3.13),
    ]
    references = [
        ReferenceLocation("PREVIOUS_START", 1.8, 0.8, math.pi),
        ReferenceLocation("A_PARK", 0.0, 4.1, 0.02),
        ReferenceLocation("B_PARK", 2.059, 3.186, -1.503),
    ]

    steps = parking_mission_document(
        route,
        frame_id="map",
        closed=False,
        reference_locations=references,
        reverse_ranges=((1, 5),),
    )["mission"]["steps"]
    by_name = {step["name"]: step for step in steps}

    assert by_name["WP_22"]["reverse_only"] is True
    assert by_name["WP_23"]["reverse_only"] is True
    assert by_name["WP_23"]["allow_reverse"] is True
    assert by_name["WP_24"]["reverse_only"] is True
    assert by_name["WP_25"]["reverse_only"] is True
    assert by_name["WP_25"]["precise_goal"] is True
    assert by_name["B_PARK"]["allow_reverse"] is False
    assert "reverse_only" not in by_name["B_PARK"]


def test_named_alignment_and_entry_create_fixed_direction_a_parking():
    route = [
        RouteWaypoint("START", 1.8, 0.8),
        RouteWaypoint("A_ALIGN_IN", 0.534, 4.116),
        RouteWaypoint("A_ENTRY", 1.034, 4.127),
        RouteWaypoint("NEAR_A", -0.014, 4.106),
        RouteWaypoint("EXIT", 2.4, 4.8),
    ]
    references = [
        ReferenceLocation("PREVIOUS_START", 1.8, 0.8, math.pi),
        ReferenceLocation("A_PARK", -0.016, 4.105, 0.021),
        ReferenceLocation("B_PARK", 2.1, 3.2, -1.5),
    ]

    steps = parking_mission_document(
        route,
        frame_id="map",
        closed=False,
        reference_locations=references,
    )["mission"]["steps"]

    alignment, entry, parking, exit_step = steps[:4]
    assert alignment["name"] == "A_ALIGN_IN"
    assert alignment["precise_goal"] is True
    assert alignment["allow_reverse"] is True
    assert alignment["yaw"] == pytest.approx(math.atan2(0.011, 0.5))
    assert exit_step["name"] == "EXIT"
    assert exit_step["allow_reverse"] is True
    assert entry["name"] == "A_ENTRY"
    assert entry["precise_goal"] is True
    assert entry["allow_reverse"] is False
    assert entry["yaw"] == pytest.approx(parking["yaw"])
    assert parking["name"] == "A_PARK"
    assert parking["parking_goal"] is True
    assert parking["allow_reverse"] is True


def test_named_cusp_route_has_no_bidirectional_control_segment():
    route = [
        RouteWaypoint("START", 1.8, 0.8),
        RouteWaypoint("A_FORWARD_CUSP", 1.1186, 4.3908, 0.9115),
        RouteWaypoint("A_REVERSE_ALIGN", 0.534, 4.116, 0.022),
        RouteWaypoint("A_ENTRY", 1.034, 4.127, 0.021),
        RouteWaypoint("NEAR_A", -0.014, 4.106),
        RouteWaypoint("EXIT", 2.4, 4.8),
    ]
    references = [
        ReferenceLocation("PREVIOUS_START", 1.8, 0.8, math.pi),
        ReferenceLocation("A_PARK", -0.016, 4.105, 0.021),
        ReferenceLocation("B_PARK", 2.1, 3.2, -1.5),
    ]

    steps = parking_mission_document(
        route,
        frame_id="map",
        closed=False,
        reference_locations=references,
    )["mission"]["steps"]
    forward_cusp, reverse_align, entry, parking = steps[:4]

    assert forward_cusp["yaw"] == pytest.approx(0.9115)
    assert forward_cusp["precise_goal"] is True
    assert forward_cusp["allow_reverse"] is False
    assert reverse_align["yaw"] == pytest.approx(0.022)
    assert reverse_align["reverse_only"] is True
    assert reverse_align["allow_reverse"] is True
    assert entry["allow_reverse"] is False
    assert entry["yaw"] == pytest.approx(0.021)
    assert parking["reverse_only"] is True

    document = waypoint_document(route, frame_id="map", closed=False)
    loaded = route_waypoints_from_items(document["waypoints"])
    assert loaded == route


def test_closed_route_adds_explicit_return_to_first_point():
    mission = parking_mission_document(_route(), frame_id="map", closed=True)[
        "mission"
    ]

    assert mission["steps"][-1]["name"] == "START_RETURN"
    assert mission["steps"][-1]["x"] == 1.0
    assert mission["steps"][-1]["y"] == 2.0
    assert mission["steps"][-1]["precise_goal"] is True
    assert mission["steps"][-1]["allow_reverse"] is True


def test_waypoint_yaml_keeps_target_branch_capture_contract():
    document = waypoint_document(_route(), frame_id="map", closed=False)

    assert document["frame_id"] == "map"
    assert document["closed"] is False
    assert all(
        item["controller_to_next"] == "global_path"
        for item in document["waypoints"]
    )
    assert document["reverse_ranges"] == []
    loaded = route_waypoints_from_items(document["waypoints"])
    assert loaded == _route()


def test_overlapping_points_are_rejected_before_mission_write():
    route = [RouteWaypoint("A", 0.0, 0.0), RouteWaypoint("B", 0.0, 0.0)]

    with pytest.raises(ValueError, match="overlap"):
        parking_mission_document(route, frame_id="map", closed=False)


def test_previous_start_and_parking_locations_come_from_surveyed_mission():
    document = yaml.safe_load(
        (PACKAGE / "config" / "parking_mission.yaml").read_text(
            encoding="utf-8"
        )
    )

    locations = reference_locations_from_mission(document)

    assert [location.name for location in locations] == [
        "PREVIOUS_START",
        "A_PARK",
        "B_PARK",
    ]
    assert locations[0].x == pytest.approx(1.790142252)
    assert locations[0].y == pytest.approx(0.777545195)
    assert locations[1].x == pytest.approx(-0.015964721)
    assert locations[2].x == pytest.approx(2.059160896)
