from pathlib import Path

import yaml


CONFIG_DIR = Path(__file__).parents[1] / "config"


def test_correlation_search_grid_is_coarse_search_aligned():
    for filename in (
        "slam_toolbox_mapping.yaml",
        "slam_toolbox_localization.yaml",
        "slam_toolbox_localization_lidar_primary.yaml",
    ):
        data = yaml.safe_load((CONFIG_DIR / filename).read_text())
        params = data["slam_toolbox"]["ros__parameters"]
        dimension = float(
            params.get("correlation_search_space_dimension", 0.30)
        )
        resolution = float(
            params.get("correlation_search_space_resolution", 0.01)
        )
        intervals = round(dimension / resolution)

        assert abs(dimension / resolution - intervals) < 1.0e-9
        assert intervals % 2 == 0


def test_slam_and_measured_odom_share_one_tf_contract():
    odom_data = yaml.safe_load(
        (CONFIG_DIR / "vesc_imu_odom_real.yaml").read_text()
    )
    odom = odom_data["vesc_imu_odom"]["ros__parameters"]

    assert odom["odom_topic"] == "/slam/odom"
    assert odom["odom_frame"] == "slam_odom"
    assert odom["base_frame"] == "base_footprint"
    assert odom["publish_tf"] is True
    assert odom["meters_per_tachometer_count"] == 0.002527806
    assert odom["imu_yaw_source"] == "gyro_z"
    assert odom["gyro_yaw_sign"] == -1.0
    assert odom["gyro_yaw_scale"] == 0.955
    assert odom["gyro_z_bias_rad_s"] == 0.030

    for filename in (
        "slam_toolbox_mapping.yaml",
        "slam_toolbox_localization.yaml",
        "slam_toolbox_localization_lidar_primary.yaml",
    ):
        data = yaml.safe_load((CONFIG_DIR / filename).read_text())
        params = data["slam_toolbox"]["ros__parameters"]
        assert params["map_frame"] == "map"
        assert params["odom_frame"] == odom["odom_frame"]
        assert params["base_frame"] == odom["base_frame"]


def test_lidar_primary_localization_can_recover_from_local_odom_error():
    data = yaml.safe_load(
        (
            CONFIG_DIR
            / "slam_toolbox_localization_lidar_primary.yaml"
        ).read_text()
    )
    params = data["slam_toolbox"]["ros__parameters"]

    assert params["mode"] == "localization"
    assert params["use_scan_matching"] is True
    assert params["minimum_travel_distance"] == 0.04
    assert params["minimum_travel_heading"] == 0.04
    assert params["correlation_search_space_dimension"] == 0.80
    assert params["distance_variance_penalty"] >= 1.0
    assert params["angle_variance_penalty"] >= 2.0
    assert params["minimum_angle_penalty"] >= 0.95
    assert params["use_response_expansion"] is True


def test_default_mapping_profile_matches_original_karto_tuning():
    data = yaml.safe_load(
        (CONFIG_DIR / "slam_toolbox_mapping.yaml").read_text()
    )
    params = data["slam_toolbox"]["ros__parameters"]

    assert params["use_scan_matching"] is True
    assert params["do_loop_closing"] is True
    assert params["minimum_travel_distance"] == 0.04
    assert params["scan_buffer_size"] == 20
    assert params["scan_buffer_maximum_scan_distance"] == 15.0
    assert params["link_match_minimum_response_fine"] == 0.10
    assert params["link_scan_maximum_distance"] == 1.50
    assert params["loop_search_maximum_distance"] == 5.0
    assert params["loop_match_minimum_chain_size"] == 8
    assert params["loop_match_maximum_variance_coarse"] == 3.0
    assert params["loop_match_minimum_response_coarse"] == 0.30
    assert params["loop_match_minimum_response_fine"] == 0.40
    assert params["correlation_search_space_dimension"] == 0.80
    assert params["correlation_search_space_smear_deviation"] == 0.1
    assert params["loop_search_space_dimension"] == 10.0
    assert params["use_response_expansion"] is True
