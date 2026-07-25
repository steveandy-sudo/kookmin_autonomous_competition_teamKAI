from pathlib import Path

import yaml


CONFIG_DIR = Path(__file__).parents[1] / "config"


def test_correlation_search_grid_is_coarse_search_aligned():
    for filename in (
        "slam_toolbox_mapping.yaml",
        "slam_toolbox_localization.yaml",
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
