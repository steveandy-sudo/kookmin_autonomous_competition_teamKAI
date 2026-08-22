from ast import literal_eval
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def _yaml(relative_path: str):
    with (PACKAGE / relative_path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_nav2_ackermann_reverse_contract():
    config = _yaml("config/nav2_parking.yaml")
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]

    assert planner["motion_model_for_search"] == "REEDS_SHEPP"
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    endpoint_curvatures = adapter["steering_map_curvatures"]
    worst_side_radius = 1.0 / min(
        abs(endpoint_curvatures[0]), abs(endpoint_curvatures[-1])
    )
    assert planner["minimum_turning_radius"] >= worst_side_radius
    assert controller["plugin"] == "nav2_mppi_controller::MPPIController"
    assert controller["motion_model"] == "Ackermann"
    assert controller["vx_min"] < 0.0 < controller["vx_max"]
    assert controller["enforce_path_inversion"] is True
    assert controller["AckermannConstraints"]["min_turning_r"] >= worst_side_radius
    assert controller["CostCritic"]["consider_footprint"] is True
    assert "PreferForwardCritic" not in controller["critics"]


def test_amcl_updates_while_stationary_for_pre_drive_gate():
    amcl = _yaml("config/nav2_parking.yaml")["amcl"]["ros__parameters"]

    assert amcl["update_min_d"] == 0.0
    assert amcl["update_min_a"] == 0.0


def test_mission_manager_runtime_limits_are_in_a_ros_parameter_file():
    manager = _yaml("config/mission_manager.yaml")["parking_mission_manager"][
        "ros__parameters"
    ]

    assert manager["required_stable_samples"] >= 5
    assert manager["maximum_pose_age_sec"] <= 1.0
    assert manager["localization_loss_cancel_sec"] <= 1.0


def test_costmaps_and_stop_shield_use_same_physical_body():
    nav2 = _yaml("config/nav2_parking.yaml")
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    global_footprint = literal_eval(
        nav2["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    )
    local_footprint = literal_eval(
        nav2["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    )

    assert global_footprint == local_footprint
    assert min(point[0] for point in global_footprint) == adapter["footprint_minimum_x"]
    assert max(point[0] for point in global_footprint) == adapter["footprint_maximum_x"]
    assert max(abs(point[1]) for point in global_footprint) == adapter["footprint_half_width"]
    global_padding = nav2["global_costmap"]["global_costmap"]["ros__parameters"][
        "footprint_padding"
    ]
    local_padding = nav2["local_costmap"]["local_costmap"]["ros__parameters"][
        "footprint_padding"
    ]
    assert global_padding == local_padding
    assert global_padding >= adapter["collision_margin_m"] + 0.05
    assert adapter["steering_settle_tolerance_command"] <= 3.0
    for costmap_name in ("global_costmap", "local_costmap"):
        params = nav2[costmap_name][costmap_name]["ros__parameters"]
        padded_inscribed_radius = min(
            max(point[0] for point in global_footprint) + global_padding,
            max(abs(point[1]) for point in global_footprint) + global_padding,
        )
        assert params["inflation_layer"]["inflation_radius"] >= padded_inscribed_radius


def test_kookmin_gazebo_real_vehicle_geometry_contract():
    """Freeze the measured geometry used by generate_kookmin_track.py."""

    nav2 = _yaml("config/nav2_parking.yaml")
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    mission = _yaml("config/parking_mission.yaml")["mission"]
    footprint = literal_eval(
        nav2["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    )

    assert mission["base_from_reference_x_m"] == 0.16
    assert min(point[0] for point in footprint) == -0.46
    assert max(point[0] for point in footprint) == 0.11
    assert max(abs(point[1]) for point in footprint) == 0.18
    assert adapter["speed_gain_mps_per_command"] == 0.080612


def test_stop_shield_projects_beyond_nav2_braking_distance():
    nav2 = _yaml("config/nav2_parking.yaml")
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    controller = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    speed = max(abs(controller["vx_min"]), controller["vx_max"])
    physical_stop = (
        speed * adapter["reaction_time_sec"]
        + speed * speed / (2.0 * adapter["braking_deceleration_mps2"])
    )

    assert adapter["minimum_projection_m"] >= physical_stop
    assert math.isfinite(physical_stop)


def test_behavior_tree_has_no_non_ackermann_recovery():
    tree = ET.parse(PACKAGE / "behavior_trees/ackermann_navigate_to_pose.xml")
    tags = {element.tag for element in tree.iter()}

    assert "Spin" not in tags
    assert "BackUp" not in tags
    assert "DriveOnHeading" not in tags
    assert {"ComputePathToPose", "FollowPath", "Wait"} <= tags


def test_mission_retains_official_reference_centers():
    mission = _yaml("config/parking_mission.yaml")["mission"]
    initial = mission["initial_pose"]
    steps = {step["name"]: step for step in mission["steps"]}

    assert (initial["x"], initial["y"], initial["yaw"]) == (
        1.8,
        0.9,
        3.141592653589793,
    )
    assert (steps["A_PARK"]["x"], steps["A_PARK"]["y"], steps["A_PARK"]["yaw"]) == (
        0.0,
        4.2,
        0.0,
    )
    assert (steps["B_PARK"]["x"], steps["B_PARK"]["y"], steps["B_PARK"]["yaw"]) == (
        2.1,
        3.3,
        -1.5707963267948966,
    )
    assert "A_CLEAR" in steps
    assert steps["A_CLEAR"]["y"] >= 4.8
    assert "A_APPROACH" in steps
    assert {"A_ROUTE_ARC_1", "A_ROUTE_STRAIGHT", "A_ROUTE_ALIGN"} <= set(steps)
    assert {"A_TURN_IN", "A_FORWARD_CUSP", "A_REVERSE_CUSP"} <= set(steps)
    assert {"A_REVERSE_ARC_1", "A_REVERSE_ARC_2", "A_REVERSE_ARC_3"} <= set(steps)
    assert {"A_CLEAR_ARC_1", "A_CLEAR_ARC_2"} <= set(steps)
    assert {"B_REVERSE_CUSP", "B_EXIT_FORWARD_CUSP"} <= set(steps)
    assert {"B_REVERSE_ARC_1", "B_REVERSE_ARC_2"} <= set(steps)
    assert "B_FORWARD_CORRECTION" in steps
    assert {"B_EXIT_FORWARD_ARC_1", "B_EXIT_FORWARD_ARC_2"} <= set(steps)
    assert {
        "START_RETURN_STRAIGHT",
        "START_RETURN_ARC",
        "START_RETURN_DIAGONAL",
        "START_RETURN_ALIGN_1",
        "START_RETURN_ALIGN_2",
    } <= set(steps)


def test_launches_use_humble_safe_python_boolean_spelling():
    for name in ("parking_navigation.launch.py", "parking_real.launch.py"):
        source = (PACKAGE / "launch" / name).read_text(encoding="utf-8")
        assert '"use_composition": "False"' in source
        assert '"use_respawn": "False"' in source
        assert '"use_composition": "false"' not in source


def test_rviz_has_no_visual_slam_or_legacy_path_topics():
    source = (PACKAGE / "rviz/parking_nav.rviz").read_text(encoding="utf-8")
    assert "slam_toolbox" not in source.lower()
    assert "/map_nav/" not in source
    assert "/amcl_pose" in source
    assert "/parking/mission_goals" in source
    assert "/plan" in source
