"""Generate a Gazebo Sim world from a ROS occupancy map."""

from __future__ import annotations

import argparse
import copy
import csv
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image, ImageDraw


# Track dimensions copied from scripts/generate_kookmin_track.py. Keep the
# full paved width distinct from the white-line measurements and the cone
# controller's expected corridor width.
TRACK_ROAD_WIDTH_M = 1.632
TRACK_WHITE_INNER_WIDTH_M = 0.800
TRACK_WHITE_CENTER_SPACING_M = 0.824
CONE_CORRIDOR_WIDTH_M = 0.85


@dataclass(frozen=True)
class OccupancyMap:
    yaml_path: Path
    image_path: Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    negate: bool
    occupied_threshold: float
    free_threshold: float


@dataclass(frozen=True)
class MapTransform:
    offset_x: float = 0.0
    offset_y: float = 0.0
    yaw: float = 0.0
    scale: float = 1.0

    def map_to_world(self, x: float, y: float) -> tuple[float, float]:
        scaled_x = float(x) * self.scale
        scaled_y = float(y) * self.scale
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return (
            self.offset_x + cosine * scaled_x - sine * scaled_y,
            self.offset_y + sine * scaled_x + cosine * scaled_y,
        )

    def world_to_map(self, x: float, y: float) -> tuple[float, float]:
        dx = float(x) - self.offset_x
        dy = float(y) - self.offset_y
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return (
            (cosine * dx + sine * dy) / self.scale,
            (-sine * dx + cosine * dy) / self.scale,
        )


@dataclass(frozen=True)
class GlassSection:
    name: str
    x: float
    y: float
    yaw: float
    length: float
    thickness: float = 0.03
    height: float = 0.90
    opening_clearance: float = 0.50


@dataclass(frozen=True)
class ConeRuleLayout:
    name: str
    start_s: float
    end_s: float
    spacing: float
    corridor_width: float
    road_width: float
    control_polygon: tuple[tuple[float, float], ...]
    entry_gate: tuple[float, float, float]
    exit_gate: tuple[float, float, float]
    slam_reference: tuple[tuple[float, float], ...]
    phases: tuple[tuple[str, float, float], ...]


# Approximate sections identified from the real venue description and the
# glass-balanced SLAM map. The SLAM image is oriented with the glass at the
# upper-right. Values remain explicit in metadata for on-site adjustment.
GLASS_LAYOUTS = {
    "none": (),
    "upper_right": (
        GlassSection(
            "glass_right",
            x=14.07,
            y=3.35,
            yaw=math.pi * 0.5,
            length=6.70,
        ),
        GlassSection(
            "glass_upper_right",
            x=10.40,
            y=8.25,
            yaw=0.0,
            length=5.60,
            opening_clearance=0.35,
        ),
    ),
}


# Geometry from scripts/generate_kookmin_track.py and the existing hwj cone
# controller. The s ranges are measured on one_lap_path.csv and cover the
# map's lower-right section.
CONE_RULE_LAYOUTS = {
    "none": None,
    "bottom_right_rule": ConeRuleLayout(
        name="bottom_right_rule",
        start_s=24.50,
        end_s=33.10,
        spacing=0.48,
        corridor_width=CONE_CORRIDOR_WIDTH_M,
        road_width=TRACK_ROAD_WIDTH_M,
        control_polygon=(
            (5.5, -1.45),
            (13.9, -1.45),
            (14.15, 1.95),
            (11.95, 1.95),
            (11.35, 0.75),
            (5.5, 0.75),
        ),
        entry_gate=(5.95, -0.16, 0.0),
        exit_gate=(13.04, 1.35, math.pi * 0.5),
        # SLAM only supplies a coarse straight approach. Cone rule owns all
        # lateral control from the entry gate through the exit curve.
        slam_reference=((5.50, -0.16), (11.70, -0.30)),
        phases=(
            ("straight_entry", 24.50, 26.00),
            ("winding", 26.00, 29.70),
            ("straight_exit", 29.70, 30.70),
            ("turn_curve", 30.70, 33.10),
        ),
    ),
}


def _parse_list(value: str) -> list[float]:
    return [
        float(item.strip())
        for item in value.strip().strip("[]").split(",")
    ]


def load_map_yaml(path: str | Path) -> OccupancyMap:
    yaml_path = Path(path).expanduser().resolve()
    values: dict[str, str] = {}
    for raw_line in yaml_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    required = {
        "image",
        "resolution",
        "origin",
        "occupied_thresh",
        "free_thresh",
    }
    missing = sorted(required - values.keys())
    if missing:
        raise ValueError(f"map YAML is missing: {', '.join(missing)}")
    image_path = Path(values["image"])
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    origin = _parse_list(values["origin"])
    if len(origin) != 3:
        raise ValueError("origin must contain x, y, yaw")
    resolution = float(values["resolution"])
    if resolution <= 0.0:
        raise ValueError("map resolution must be positive")
    return OccupancyMap(
        yaml_path=yaml_path,
        image_path=image_path.resolve(),
        resolution=resolution,
        origin_x=origin[0],
        origin_y=origin[1],
        origin_yaw=origin[2],
        negate=bool(int(values.get("negate", "0"))),
        occupied_threshold=float(values["occupied_thresh"]),
        free_threshold=float(values["free_thresh"]),
    )


def map_pixel_to_xy(
    config: OccupancyMap,
    image_width: int,
    image_height: int,
    column: float,
    row: float,
) -> tuple[float, float]:
    local_x = (float(column) + 0.5) * config.resolution
    local_y = (
        image_height - float(row) - 0.5
    ) * config.resolution
    cosine = math.cos(config.origin_yaw)
    sine = math.sin(config.origin_yaw)
    return (
        config.origin_x + cosine * local_x - sine * local_y,
        config.origin_y + sine * local_x + cosine * local_y,
    )


def map_xy_to_pixel(
    config: OccupancyMap,
    image_width: int,
    image_height: int,
    x: float,
    y: float,
) -> tuple[float, float]:
    dx = float(x) - config.origin_x
    dy = float(y) - config.origin_y
    cosine = math.cos(config.origin_yaw)
    sine = math.sin(config.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return (
        local_x / config.resolution - 0.5,
        image_height - local_y / config.resolution - 0.5,
    )


def _classification(
    image: np.ndarray,
    config: OccupancyMap,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    values = image.astype(np.float32) / 255.0
    occupied_probability = values if config.negate else 1.0 - values
    occupied = occupied_probability >= config.occupied_threshold
    free = occupied_probability <= config.free_threshold
    unknown = ~(occupied | free)
    return occupied, free, unknown


def _remove_small_components(
    mask: np.ndarray,
    minimum_pixels: int,
) -> np.ndarray:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8),
        connectivity=8,
    )
    result = np.zeros_like(mask, dtype=bool)
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] >= minimum_pixels:
            result |= labels == label
    return result


def _downsample_any(mask: np.ndarray, factor: int) -> np.ndarray:
    factor = max(1, int(factor))
    if factor == 1:
        return mask
    height, width = mask.shape
    padded_height = math.ceil(height / factor) * factor
    padded_width = math.ceil(width / factor) * factor
    padded = np.zeros((padded_height, padded_width), dtype=bool)
    padded[:height, :width] = mask
    return padded.reshape(
        padded_height // factor,
        factor,
        padded_width // factor,
        factor,
    ).any(axis=(1, 3))


def mask_rectangles(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Merge identical horizontal occupied spans over adjacent rows."""
    active: dict[tuple[int, int], tuple[int, int]] = {}
    rectangles: list[tuple[int, int, int, int]] = []
    for row_index, row in enumerate(mask):
        padded = np.pad(row.astype(np.int8), (1, 1))
        differences = np.diff(padded)
        starts = np.flatnonzero(differences == 1)
        stops = np.flatnonzero(differences == -1) - 1
        spans = {
            (int(start), int(stop))
            for start, stop in zip(starts, stops)
        }
        next_active: dict[tuple[int, int], tuple[int, int]] = {}
        for span in spans:
            if span in active:
                start_row, _ = active[span]
                next_active[span] = (start_row, row_index)
            else:
                next_active[span] = (row_index, row_index)
        for span, (start_row, stop_row) in active.items():
            if span not in spans:
                rectangles.append(
                    (start_row, stop_row, span[0], span[1])
                )
        active = next_active
    for span, (start_row, stop_row) in active.items():
        rectangles.append((start_row, stop_row, span[0], span[1]))
    return rectangles


def _load_route(path: Path) -> list[tuple[float, float]]:
    with path.open(encoding="utf-8", newline="") as stream:
        points = [
            (float(row["x"]), float(row["y"]))
            for row in csv.DictReader(stream)
        ]
    if len(points) < 3:
        raise ValueError("route CSV needs at least three points")
    if math.hypot(
        points[-1][0] - points[0][0],
        points[-1][1] - points[0][1],
    ) > 0.02:
        points.append(points[0])
    return points


def _smooth_closed_route(
    points: list[tuple[float, float]],
    window: int = 13,
) -> list[tuple[float, float]]:
    unique = points[:-1] if points[0] == points[-1] else points
    radius = max(1, int(window) // 2)
    count = len(unique)
    smoothed = []
    for index in range(count):
        neighbors = [
            unique[(index + offset) % count]
            for offset in range(-radius, radius + 1)
        ]
        smoothed.append(
            (
                sum(point[0] for point in neighbors) / len(neighbors),
                sum(point[1] for point in neighbors) / len(neighbors),
            )
        )
    smoothed.append(smoothed[0])
    return smoothed


def _resample_closed_route(
    points: list[tuple[float, float]],
    spacing: float,
) -> list[tuple[float, float]]:
    spacing = max(0.05, float(spacing))
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(
            cumulative[-1]
            + math.hypot(second[0] - first[0], second[1] - first[1])
        )
    total = cumulative[-1]
    count = max(3, int(math.ceil(total / spacing)))
    targets = [index * total / count for index in range(count)]
    sampled = []
    segment = 0
    for target in targets:
        while (
            segment + 1 < len(cumulative)
            and cumulative[segment + 1] < target
        ):
            segment += 1
        span = cumulative[segment + 1] - cumulative[segment]
        ratio = (
            0.0
            if span <= 1e-12
            else (target - cumulative[segment]) / span
        )
        sampled.append(
            (
                points[segment][0] * (1.0 - ratio)
                + points[segment + 1][0] * ratio,
                points[segment][1] * (1.0 - ratio)
                + points[segment + 1][1] * ratio,
            )
        )
    sampled.append(sampled[0])
    return sampled


def route_wall_boxes(
    route_csv: str | Path,
    *,
    road_half_width: float = TRACK_ROAD_WIDTH_M * 0.5,
    wall_thickness: float = 0.08,
    spacing: float = 0.20,
) -> list[tuple[float, float, float, float, float]]:
    """Return x, y, yaw, length, width boxes for clean route boundaries."""
    points = _resample_closed_route(
        _smooth_closed_route(_load_route(Path(route_csv))),
        spacing,
    )
    unique = points[:-1]
    count = len(unique)
    left = []
    right = []
    for index, point in enumerate(unique):
        before = unique[(index - 1) % count]
        after = unique[(index + 1) % count]
        yaw = math.atan2(after[1] - before[1], after[0] - before[0])
        normal_x = -math.sin(yaw)
        normal_y = math.cos(yaw)
        left.append(
            (
                point[0] + normal_x * road_half_width,
                point[1] + normal_y * road_half_width,
            )
        )
        right.append(
            (
                point[0] - normal_x * road_half_width,
                point[1] - normal_y * road_half_width,
            )
        )
    boxes = []
    for boundary in (left, right):
        for index, first in enumerate(boundary):
            second = boundary[(index + 1) % count]
            length = math.hypot(second[0] - first[0], second[1] - first[1])
            if length <= 1e-4:
                continue
            boxes.append(
                (
                    (first[0] + second[0]) * 0.5,
                    (first[1] + second[1]) * 0.5,
                    math.atan2(second[1] - first[1], second[0] - first[0]),
                    length + wall_thickness * 0.5,
                    wall_thickness,
                )
            )
    return boxes


def cone_corridor_samples(
    route_csv: str | Path,
    layout: ConeRuleLayout,
) -> list[tuple[str, float, float, float]]:
    """Return side, x, y, yaw cone poses sampled along a route s interval."""
    rows = []
    with Path(route_csv).open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            rows.append(
                (
                    float(row["s"]),
                    float(row["x"]),
                    float(row["y"]),
                )
            )
    if len(rows) < 2:
        raise ValueError("cone rule layout requires at least two route rows")
    if layout.end_s <= layout.start_s or layout.spacing <= 0.0:
        raise ValueError("invalid cone rule layout s range or spacing")
    if layout.corridor_width <= 0.0:
        raise ValueError("cone corridor width must be positive")
    if layout.corridor_width >= layout.road_width:
        raise ValueError("cone corridor must fit inside the fixed road width")

    count = max(
        2,
        int(math.floor((layout.end_s - layout.start_s) / layout.spacing)) + 1,
    )
    targets = np.linspace(layout.start_s, layout.end_s, count)
    result = []
    row_index = 0
    for target in targets:
        while (
            row_index + 1 < len(rows) - 1
            and rows[row_index + 1][0] < target
        ):
            row_index += 1
        first = rows[row_index]
        second = rows[min(row_index + 1, len(rows) - 1)]
        span = second[0] - first[0]
        ratio = 0.0 if span <= 1e-12 else (target - first[0]) / span
        ratio = max(0.0, min(1.0, ratio))
        x = first[1] * (1.0 - ratio) + second[1] * ratio
        y = first[2] * (1.0 - ratio) + second[2] * ratio
        yaw = math.atan2(second[2] - first[2], second[1] - first[1])
        normal_x = -math.sin(yaw)
        normal_y = math.cos(yaw)
        half_width = layout.corridor_width * 0.5
        result.extend(
            (
                (
                    "left",
                    x + normal_x * half_width,
                    y + normal_y * half_width,
                    yaw,
                ),
                (
                    "right",
                    x - normal_x * half_width,
                    y - normal_y * half_width,
                    yaw,
                ),
            )
        )
    return result


def _box_overlaps_glass_opening(
    box: tuple[float, float, float, float, float],
    section: GlassSection,
) -> bool:
    dx = box[0] - section.x
    dy = box[1] - section.y
    cosine = math.cos(section.yaw)
    sine = math.sin(section.yaw)
    along = cosine * dx + sine * dy
    across = -sine * dx + cosine * dy
    return (
        abs(along) <= section.length * 0.5
        and abs(across) <= section.opening_clearance
    )


def apply_glass_openings(
    boxes: list[tuple[float, float, float, float, float]],
    sections: tuple[GlassSection, ...],
) -> list[tuple[float, float, float, float, float]]:
    return [
        box
        for box in boxes
        if not any(
            _box_overlaps_glass_opening(box, section)
            for section in sections
        )
    ]


def _element(parent: ET.Element, tag: str, text: str | None = None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = text
    return child


def _box_geometry(
    parent: ET.Element,
    size_x: float,
    size_y: float,
    size_z: float,
) -> None:
    geometry = _element(parent, "geometry")
    box = _element(geometry, "box")
    _element(box, "size", f"{size_x:.6f} {size_y:.6f} {size_z:.6f}")


def _material(parent: ET.Element, color: tuple[float, float, float, float]):
    material = _element(parent, "material")
    text = " ".join(f"{value:.4f}" for value in color)
    _element(material, "ambient", text)
    _element(material, "diffuse", text)


def _create_world_base(source_world: Path) -> tuple[ET.Element, ET.Element]:
    source_root = ET.parse(source_world).getroot()
    source = source_root.find("world")
    if source is None:
        raise ValueError(f"{source_world} has no SDF world")
    root = ET.Element("sdf", {"version": "1.10"})
    world = ET.SubElement(root, "world", {"name": "kookmin_xycar_track"})
    copy_tags = {
        "gravity",
        "magnetic_field",
        "atmosphere",
        "physics",
        "scene",
        "light",
    }
    for child in source:
        if child.tag in copy_tags or child.tag == "plugin":
            world.append(copy.deepcopy(child))
    if not world.findall("plugin"):
        for name, filename in (
            ("gz::sim::systems::Physics", "gz-sim-physics-system"),
            ("gz::sim::systems::UserCommands", "gz-sim-user-commands-system"),
            ("gz::sim::systems::SceneBroadcaster", "gz-sim-scene-broadcaster-system"),
            ("gz::sim::systems::Sensors", "gz-sim-sensors-system"),
        ):
            _element(world, "plugin", name=name, filename=filename)
    return root, world


def _add_map_model(
    world: ET.Element,
    config: OccupancyMap,
    image_width: int,
    image_height: int,
    rectangles: list[tuple[int, int, int, int]],
    downsample: int,
    transform: MapTransform,
    texture_uri: str,
    wall_height: float,
    route_boxes: list[tuple[float, float, float, float, float]] | None = None,
    glass_sections: tuple[GlassSection, ...] = (),
    cone_samples: list[tuple[str, float, float, float]] | None = None,
) -> None:
    model = _element(world, "model", name="slam_map")
    _element(model, "static", "true")
    _element(
        model,
        "pose",
        f"{transform.offset_x:.6f} {transform.offset_y:.6f} 0 0 0 "
        f"{transform.yaw:.9f}",
    )
    center_x, center_y = map_pixel_to_xy(
        config,
        image_width,
        image_height,
        (image_width - 1) * 0.5,
        (image_height - 1) * 0.5,
    )
    center_x *= transform.scale
    center_y *= transform.scale
    floor_width = image_width * config.resolution * transform.scale
    floor_height = image_height * config.resolution * transform.scale
    floor = _element(model, "link", name="map_floor")
    _element(floor, "pose", f"{center_x:.6f} {center_y:.6f} 0 0 0 0")
    collision = _element(floor, "collision", name="floor_collision")
    _element(collision, "pose", "0 0 -0.020 0 0 0")
    _box_geometry(collision, floor_width, floor_height, 0.04)
    surface = _element(collision, "surface")
    friction = _element(surface, "friction")
    ode = _element(friction, "ode")
    _element(ode, "mu", "50")
    _element(ode, "mu2", "50")
    visual = _element(floor, "visual", name="map_texture")
    _element(visual, "pose", "0 0 0.002 0 0 0")
    geometry = _element(visual, "geometry")
    plane = _element(geometry, "plane")
    _element(plane, "normal", "0 0 1")
    _element(plane, "size", f"{floor_width:.6f} {floor_height:.6f}")
    material = _element(visual, "material")
    pbr = _element(material, "pbr")
    metal = _element(pbr, "metal")
    _element(metal, "albedo_map", texture_uri)
    _element(metal, "roughness", "1")
    _element(metal, "metalness", "0")

    cell = config.resolution * downsample
    wall_color = (0.30, 0.32, 0.34, 1.0)
    for index, (row0, row1, column0, column1) in enumerate(rectangles):
        x0 = config.origin_x + column0 * cell
        x1 = min(
            config.origin_x + image_width * config.resolution,
            config.origin_x + (column1 + 1) * cell,
        )
        y1 = config.origin_y + image_height * config.resolution - row0 * cell
        y0 = max(
            config.origin_y,
            config.origin_y
            + image_height * config.resolution
            - (row1 + 1) * cell,
        )
        x_center = (x0 + x1) * 0.5 * transform.scale
        y_center = (y0 + y1) * 0.5 * transform.scale
        size_x = max(config.resolution, x1 - x0) * transform.scale
        size_y = max(config.resolution, y1 - y0) * transform.scale
        link = _element(model, "link", name=f"wall_{index:04d}")
        _element(
            link,
            "pose",
            f"{x_center:.6f} {y_center:.6f} {wall_height * 0.5:.6f} "
            "0 0 0",
        )
        collision = _element(link, "collision", name="collision")
        _box_geometry(collision, size_x, size_y, wall_height)
        visual = _element(link, "visual", name="visual")
        _box_geometry(visual, size_x, size_y, wall_height)
        _material(visual, wall_color)
    route_start = len(rectangles)
    for offset, (x, y, yaw, length, width) in enumerate(route_boxes or []):
        link = _element(
            model,
            "link",
            name=f"route_wall_{route_start + offset:04d}",
        )
        _element(
            link,
            "pose",
            f"{x * transform.scale:.6f} {y * transform.scale:.6f} "
            f"{wall_height * 0.5:.6f} 0 0 {yaw:.9f}",
        )
        collision = _element(link, "collision", name="collision")
        _box_geometry(
            collision,
            length * transform.scale,
            width * transform.scale,
            wall_height,
        )
        visual = _element(link, "visual", name="visual")
        _box_geometry(
            visual,
            length * transform.scale,
            width * transform.scale,
            wall_height,
        )
        _material(visual, wall_color)

    for section in glass_sections:
        link = _element(model, "link", name=section.name)
        _element(
            link,
            "pose",
            f"{section.x * transform.scale:.6f} "
            f"{section.y * transform.scale:.6f} "
            f"{section.height * 0.5:.6f} 0 0 {section.yaw:.9f}",
        )
        # A low curb prevents the wheels leaving the course while remaining
        # below the horizontal LiDAR beam. The full pane is visual-only, so
        # the simulated scan reproduces the important glass "no return" case.
        curb = _element(link, "collision", name="low_curb_collision")
        _element(
            curb,
            "pose",
            f"0 0 {-section.height * 0.5 + 0.020:.6f} 0 0 0",
        )
        _box_geometry(
            curb,
            section.length * transform.scale,
            max(0.06, section.thickness * transform.scale),
            0.04,
        )
        visual = _element(link, "visual", name="glass_visual")
        _box_geometry(
            visual,
            section.length * transform.scale,
            section.thickness * transform.scale,
            section.height,
        )
        _element(visual, "transparency", "0.72")
        _element(visual, "cast_shadows", "false")
        _material(visual, (0.35, 0.70, 0.85, 0.28))

    side_counts = {"left": 0, "right": 0}
    for side, x, y, yaw in cone_samples or []:
        index = side_counts[side]
        side_counts[side] += 1
        link = _element(
            model,
            "link",
            name=f"cone_rule_{side}_{index:02d}",
        )
        _element(
            link,
            "pose",
            f"{x * transform.scale:.6f} {y * transform.scale:.6f} "
            f"0.180000 0 0 {yaw:.9f}",
        )
        # Match the reference cone already defined by the simulation branch:
        # 0.18 x 0.18 x 0.36 m. Collision is required for LiDAR clustering.
        collision = _element(link, "collision", name="collision")
        _box_geometry(collision, 0.18, 0.18, 0.36)
        visual = _element(link, "visual", name="visual")
        _box_geometry(visual, 0.18, 0.18, 0.36)
        _material(visual, (0.8627, 0.3137, 0.0941, 1.0))


def _read_spawn(path_csv: Path | None) -> tuple[float, float, float]:
    if path_csv is None:
        return -2.70, 2.25, 0.0
    with path_csv.open(encoding="utf-8", newline="") as stream:
        row = next(csv.DictReader(stream))
    return float(row["x"]), float(row["y"]), float(row["yaw"])


def _add_vehicle(
    world: ET.Element,
    source_world: Path,
    spawn: tuple[float, float, float],
    transform: MapTransform,
) -> tuple[float, float, float]:
    source = ET.parse(source_world).getroot().find("world")
    if source is None:
        raise ValueError(f"{source_world} has no SDF world")
    vehicle = source.find("./model[@name='xycar_ackermann']")
    if vehicle is None:
        raise ValueError("source world has no xycar_ackermann model")
    vehicle = copy.deepcopy(vehicle)
    x, y = transform.map_to_world(spawn[0], spawn[1])
    yaw = spawn[2] + transform.yaw
    pose = vehicle.find("pose")
    if pose is None:
        pose = ET.Element("pose")
        vehicle.insert(0, pose)
    pose.text = f"{x:.6f} {y:.6f} 0.05 0 0 {yaw:.9f}"
    world.append(vehicle)
    return x, y, yaw


def _make_texture(
    image: np.ndarray,
    occupied: np.ndarray,
    free: np.ndarray,
    route_csv: Path | None,
    config: OccupancyMap,
    output: Path,
) -> None:
    height, width = image.shape
    texture = np.empty((height, width, 3), dtype=np.uint8)
    texture[:] = (84, 87, 91)
    texture[free] = (174, 177, 179)
    texture[occupied] = (42, 45, 48)
    pil_image = Image.fromarray(texture, "RGB")
    if route_csv is not None and route_csv.exists():
        points = []
        with route_csv.open(encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                column, image_row = map_xy_to_pixel(
                    config,
                    width,
                    height,
                    float(row["x"]),
                    float(row["y"]),
                )
                points.append((round(column), round(image_row)))
        if len(points) >= 2:
            ImageDraw.Draw(pil_image).line(
                points,
                fill=(255, 196, 12),
                width=3,
                joint="curve",
            )
    output.parent.mkdir(parents=True, exist_ok=True)
    pil_image.save(output)


def generate_world(
    *,
    map_yaml: str | Path,
    source_world: str | Path,
    output_world: str | Path,
    output_texture: str | Path,
    texture_uri: str,
    route_csv: str | Path | None = None,
    metadata_path: str | Path | None = None,
    transform: MapTransform = MapTransform(),
    wall_height: float = 0.80,
    minimum_component_pixels: int = 12,
    downsample: int = 2,
    wall_mode: str = "route",
    road_half_width: float = TRACK_ROAD_WIDTH_M * 0.5,
    wall_thickness: float = 0.08,
    wall_spacing: float = 0.20,
    glass_layout: str = "upper_right",
    cone_layout: str = "bottom_right_rule",
) -> dict:
    if transform.scale <= 0.0:
        raise ValueError("map scale must be positive")
    config = load_map_yaml(map_yaml)
    source_world = Path(source_world).expanduser().resolve()
    output_world = Path(output_world).expanduser().resolve()
    output_texture = Path(output_texture).expanduser().resolve()
    route_path = (
        Path(route_csv).expanduser().resolve()
        if route_csv is not None
        else None
    )
    image = np.asarray(Image.open(config.image_path).convert("L"))
    occupied, free, _ = _classification(image, config)
    occupied = _remove_small_components(
        occupied,
        max(1, int(minimum_component_pixels)),
    )
    reduced = _downsample_any(occupied, downsample)
    if wall_mode not in {"route", "occupancy"}:
        raise ValueError("wall_mode must be route or occupancy")
    if wall_mode == "route" and route_path is None:
        raise ValueError("route wall mode requires route_csv")
    if glass_layout not in GLASS_LAYOUTS:
        raise ValueError(
            f"glass_layout must be one of: {', '.join(GLASS_LAYOUTS)}"
        )
    if cone_layout not in CONE_RULE_LAYOUTS:
        raise ValueError(
            f"cone_layout must be one of: {', '.join(CONE_RULE_LAYOUTS)}"
        )
    glass_sections = GLASS_LAYOUTS[glass_layout]
    cone_rule_layout = CONE_RULE_LAYOUTS[cone_layout]
    rectangles = (
        mask_rectangles(reduced)
        if wall_mode == "occupancy"
        else []
    )
    route_boxes = (
        route_wall_boxes(
            route_path,
            road_half_width=road_half_width,
            wall_thickness=wall_thickness,
            spacing=wall_spacing,
        )
        if wall_mode == "route"
        else []
    )
    route_boxes = apply_glass_openings(route_boxes, glass_sections)
    cone_samples = (
        cone_corridor_samples(route_path, cone_rule_layout)
        if route_path is not None and cone_rule_layout is not None
        else []
    )
    _make_texture(
        image,
        occupied,
        free,
        route_path,
        config,
        output_texture,
    )
    root, world = _create_world_base(source_world)
    _add_map_model(
        world,
        config,
        image.shape[1],
        image.shape[0],
        rectangles,
        max(1, int(downsample)),
        transform,
        texture_uri,
        float(wall_height),
        route_boxes,
        glass_sections,
        cone_samples,
    )
    spawn_map = _read_spawn(route_path)
    spawn_world = _add_vehicle(world, source_world, spawn_map, transform)
    ET.indent(root, space="  ")
    output_world.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(
        output_world,
        encoding="utf-8",
        xml_declaration=True,
    )
    metadata = {
        "world_name": "kookmin_xycar_track",
        "map_model_name": "slam_map",
        "vehicle_model_name": "xycar_ackermann",
        "map_yaml": str(config.yaml_path),
        "map_image": str(config.image_path),
        "map_width_pixels": int(image.shape[1]),
        "map_height_pixels": int(image.shape[0]),
        "resolution_m": config.resolution,
        "map_width_m": image.shape[1] * config.resolution,
        "map_height_m": image.shape[0] * config.resolution,
        "transform": asdict(transform),
        "wall_mode": wall_mode,
        "wall_elements": len(rectangles) + len(route_boxes),
        "road_half_width_m": road_half_width,
        "road_width_m": road_half_width * 2.0,
        "road_geometry_reference": {
            "full_paved_width_m": TRACK_ROAD_WIDTH_M,
            "white_line_inner_width_m": TRACK_WHITE_INNER_WIDTH_M,
            "white_line_center_spacing_m": TRACK_WHITE_CENTER_SPACING_M,
            "cone_corridor_width_m": CONE_CORRIDOR_WIDTH_M,
            "source": (
                "scripts/generate_kookmin_track.py and "
                "hwj_cone_planner.py"
            ),
        },
        "wall_thickness_m": wall_thickness,
        "wall_height_m": wall_height,
        "glass_layout": glass_layout,
        "glass_sections": [
            {
                **asdict(section),
                "yaw_deg": math.degrees(section.yaw),
                "lidar_collision": False,
                "curb_height_m": 0.04,
            }
            for section in glass_sections
        ],
        "cone_rule_zone": (
            {
                **asdict(cone_rule_layout),
                "entry_gate": {
                    "x": cone_rule_layout.entry_gate[0],
                    "y": cone_rule_layout.entry_gate[1],
                    "yaw": cone_rule_layout.entry_gate[2],
                    "yaw_deg": math.degrees(
                        cone_rule_layout.entry_gate[2]
                    ),
                },
                "exit_gate": {
                    "x": cone_rule_layout.exit_gate[0],
                    "y": cone_rule_layout.exit_gate[1],
                    "yaw": cone_rule_layout.exit_gate[2],
                    "yaw_deg": math.degrees(
                        cone_rule_layout.exit_gate[2]
                    ),
                },
                "cone_count": len(cone_samples),
                "control": "LiDAR cone rule; keep SLAM localization running",
            }
            if cone_rule_layout is not None
            else None
        ),
        "spawn_map": {
            "x": spawn_map[0],
            "y": spawn_map[1],
            "yaw": spawn_map[2],
        },
        "spawn_world": {
            "x": spawn_world[0],
            "y": spawn_world[1],
            "yaw": spawn_world[2],
        },
        "output_world": str(output_world),
        "output_texture": str(output_texture),
    }
    if metadata_path is not None:
        metadata_output = Path(metadata_path).expanduser().resolve()
        metadata_output.parent.mkdir(parents=True, exist_ok=True)
        portable_metadata = copy.deepcopy(metadata)
        for key in (
            "map_yaml",
            "map_image",
            "output_world",
            "output_texture",
        ):
            portable_metadata[key] = os.path.relpath(
                metadata[key],
                metadata_output.parent,
            )
        metadata_output.write_text(
            json.dumps(portable_metadata, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
    return metadata


def parse_args(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--source-world", required=True)
    parser.add_argument("--output-world", required=True)
    parser.add_argument("--output-texture", required=True)
    parser.add_argument(
        "--texture-uri",
        default=(
            "file://maps/slam_glass_balanced/"
            "slam_glass_balanced_texture.png"
        ),
    )
    parser.add_argument("--route-csv")
    parser.add_argument("--metadata")
    parser.add_argument("--map-x", type=float, default=0.0)
    parser.add_argument("--map-y", type=float, default=0.0)
    parser.add_argument("--map-yaw-deg", type=float, default=0.0)
    parser.add_argument("--map-scale", type=float, default=1.0)
    parser.add_argument("--wall-height", type=float, default=0.80)
    parser.add_argument("--minimum-component-pixels", type=int, default=12)
    parser.add_argument("--downsample", type=int, default=2)
    parser.add_argument(
        "--wall-mode",
        choices=("route", "occupancy"),
        default="route",
    )
    parser.add_argument("--road-half-width", type=float, default=0.816)
    parser.add_argument("--wall-thickness", type=float, default=0.08)
    parser.add_argument("--wall-spacing", type=float, default=0.20)
    parser.add_argument(
        "--glass-layout",
        choices=tuple(GLASS_LAYOUTS),
        default="upper_right",
    )
    parser.add_argument(
        "--cone-layout",
        choices=tuple(CONE_RULE_LAYOUTS),
        default="bottom_right_rule",
    )
    return parser.parse_args(args)


def main(args=None):
    options = parse_args(args)
    metadata = generate_world(
        map_yaml=options.map_yaml,
        source_world=options.source_world,
        output_world=options.output_world,
        output_texture=options.output_texture,
        texture_uri=options.texture_uri,
        route_csv=options.route_csv,
        metadata_path=options.metadata,
        transform=MapTransform(
            offset_x=options.map_x,
            offset_y=options.map_y,
            yaw=math.radians(options.map_yaw_deg),
            scale=options.map_scale,
        ),
        wall_height=options.wall_height,
        minimum_component_pixels=options.minimum_component_pixels,
        downsample=options.downsample,
        wall_mode=options.wall_mode,
        road_half_width=options.road_half_width,
        wall_thickness=options.wall_thickness,
        wall_spacing=options.wall_spacing,
        glass_layout=options.glass_layout,
        cone_layout=options.cone_layout,
    )
    print(json.dumps(metadata, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
