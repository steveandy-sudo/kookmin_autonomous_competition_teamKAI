from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

from xycar_gazebo_bridge.slam_world_generator import (
    CONE_RULE_LAYOUTS,
    GLASS_LAYOUTS,
    MapTransform,
    apply_glass_openings,
    cone_corridor_samples,
    generate_world,
    load_map_yaml,
    map_pixel_to_xy,
    map_xy_to_pixel,
    mask_rectangles,
    route_wall_boxes,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
MAP_DIR = PACKAGE_ROOT / "maps" / "slam_glass_balanced"


def test_map_pixel_round_trip():
    config = load_map_yaml(MAP_DIR / "slam_glass_balanced.yaml")
    width, height = Image.open(config.image_path).size
    for column, row in ((0.0, 0.0), (225.0, 158.0), (449.0, 316.0)):
        x, y = map_pixel_to_xy(
            config,
            width,
            height,
            column,
            row,
        )
        actual_column, actual_row = map_xy_to_pixel(
            config,
            width,
            height,
            x,
            y,
        )
        assert abs(actual_column - column) < 1e-9
        assert abs(actual_row - row) < 1e-9


def test_map_transform_round_trip():
    transform = MapTransform(
        offset_x=1.2,
        offset_y=-0.4,
        yaw=0.37,
        scale=1.13,
    )
    world = transform.map_to_world(3.2, -1.8)
    recovered = transform.world_to_map(*world)
    assert np.allclose(recovered, (3.2, -1.8), atol=1e-10)


def test_mask_rectangles_merges_rows():
    mask = np.array(
        [
            [0, 1, 1, 0],
            [0, 1, 1, 0],
            [1, 0, 0, 1],
        ],
        dtype=bool,
    )
    assert sorted(mask_rectangles(mask)) == sorted(
        [(0, 1, 1, 2), (2, 2, 0, 0), (2, 2, 3, 3)]
    )


def test_route_wall_boxes_are_closed_and_positive():
    boxes = route_wall_boxes(MAP_DIR / "one_lap_path.csv")
    assert len(boxes) > 300
    assert all(length > 1e-4 for _, _, _, length, _ in boxes)
    assert all(width == 0.08 for _, _, _, _, width in boxes)


def test_glass_layout_opens_only_the_outer_boundaries():
    boxes = route_wall_boxes(MAP_DIR / "one_lap_path.csv")
    opened = apply_glass_openings(
        boxes,
        GLASS_LAYOUTS["upper_right"],
    )
    assert 20 < len(boxes) - len(opened) < 100
    assert len(opened) > 300


def test_bottom_right_cone_corridor_uses_repository_widths():
    layout = CONE_RULE_LAYOUTS["bottom_right_rule"]
    samples = cone_corridor_samples(
        MAP_DIR / "one_lap_path.csv",
        layout,
    )
    assert layout.road_width == 1.632
    assert layout.corridor_width == 0.85
    assert len(samples) >= 30
    assert len(samples) % 2 == 0
    for left, right in zip(samples[::2], samples[1::2]):
        assert left[0] == "left"
        assert right[0] == "right"
        separation = np.hypot(left[1] - right[1], left[2] - right[2])
        assert abs(separation - 0.85) < 1e-9


def test_generate_world_contains_map_vehicle_and_walls(tmp_path):
    output_world = tmp_path / "slam_world.sdf"
    output_texture = tmp_path / "slam_texture.png"
    metadata = generate_world(
        map_yaml=MAP_DIR / "slam_glass_balanced.yaml",
        source_world=REPOSITORY_ROOT
        / "worlds"
        / "kookmin_xycar_track_final.sdf",
        output_world=output_world,
        output_texture=output_texture,
        texture_uri="file://slam_texture.png",
        route_csv=MAP_DIR / "one_lap_path.csv",
        wall_mode="route",
    )
    assert output_world.exists()
    assert output_texture.exists()
    root = ET.parse(output_world).getroot()
    world = root.find("./world[@name='kookmin_xycar_track']")
    assert world is not None
    map_model = world.find("./model[@name='slam_map']")
    vehicle = world.find("./model[@name='xycar_ackermann']")
    assert map_model is not None
    assert vehicle is not None
    route_walls = [
        link
        for link in map_model.findall("link")
        if link.get("name", "").startswith("route_wall_")
    ]
    assert len(route_walls) == metadata["wall_elements"]
    assert len(route_walls) > 300
    assert metadata["glass_layout"] == "upper_right"
    assert len(metadata["glass_sections"]) == 2
    for name in ("glass_right", "glass_upper_right"):
        glass = map_model.find(f"./link[@name='{name}']")
        assert glass is not None
        assert glass.find("./collision[@name='low_curb_collision']") is not None
        assert glass.find("./visual[@name='glass_visual']/transparency").text == (
            "0.72"
        )
    cone_links = [
        link
        for link in map_model.findall("link")
        if link.get("name", "").startswith("cone_rule_")
    ]
    assert metadata["road_width_m"] == 1.632
    assert len(cone_links) == metadata["cone_rule_zone"]["cone_count"]
    assert len(cone_links) >= 30
    assert all(link.find("collision") is not None for link in cone_links)
    assert world.find(
        "./plugin[@name='gz::sim::systems::UserCommands']"
    ) is not None
    pose_values = [
        float(value) for value in vehicle.findtext("pose").split()
    ]
    assert len(pose_values) == 6
    assert pose_values[2] == 0.05
