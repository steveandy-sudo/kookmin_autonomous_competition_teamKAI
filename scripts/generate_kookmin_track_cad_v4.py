#!/usr/bin/env python3
"""Build a review Gazebo world from track-plan-cleaned-v4.dwg.

The source drawing contains the room outline, architectural furniture, and a
painted Xycar track on one layer.  This generator deliberately keeps only:

* the outermost closed polyline as a collidable room wall;
* the two close contour pairs that form the outer / inner white track lines;
* small closed paint polygons whose centres lie inside the track corridor as
  yellow centre dashes.

The output is a review candidate by default.  Pass ``--promote`` after visual
approval to also replace ``kookmin_xycar_track_final.sdf``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import cv2
import numpy as np
from PIL import Image

import generate_kookmin_track as legacy


ROOT = Path(__file__).resolve().parents[1]
TEXTURE_DIR = ROOT / "media" / "materials" / "textures"
WORLD_PATH = ROOT / "worlds" / "kookmin_xycar_track_cad_v4.sdf"
FINAL_WORLD_PATH = ROOT / "worlds" / "kookmin_xycar_track_final.sdf"
PREVIEW_PATH = ROOT / "preview_cad_v4_topdown.png"
METRICS_PATH = ROOT / "analysis" / "track_cad_v4_metrics.json"
BASE_TEXTURE = TEXTURE_DIR / "kookmin_cad_v4_road.png"

SOURCE_UNIT_TO_M = 0.001  # the DWG uses millimetres
PIXELS_PER_M = 100
WORLD_MARGIN_M = 0.30
WALL_WIDTH_M = 0.08
WALL_HEIGHT_M = 1.70
WALL_COLOR = (218, 218, 210, 255)
ROAD_COLOR = (92, 94, 91, 255)
WHITE_COLOR = (242, 242, 236, 255)
YELLOW_COLOR = (255, 196, 12, 255)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the CAD-v4 Gazebo review world."
    )
    parser.add_argument(
        "source",
        nargs="?",
        type=Path,
        default=Path.home() / "Downloads" / "track-plan-cleaned-v4.dwg",
    )
    parser.add_argument(
        "--dwg2dxf",
        type=Path,
        help="LibreDWG dwg2dxf executable (or set LIBREDWG_DWG2DXF).",
    )
    parser.add_argument(
        "--promote",
        action="store_true",
        help="Also write the approved geometry to the default final world.",
    )
    return parser.parse_args()


def resolved_dxf(source: Path, converter: Path | None):
    """Yield an input DXF, converting a DWG in a temporary directory."""
    source = source.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    if source.suffix.lower() == ".dxf":
        class DirectDxf:
            def __enter__(self):
                return source

            def __exit__(self, *_):
                return False

        return DirectDxf()
    if source.suffix.lower() != ".dwg":
        raise ValueError(f"expected .dwg or .dxf, got: {source}")

    configured = converter or (
        Path(os.environ["LIBREDWG_DWG2DXF"])
        if os.environ.get("LIBREDWG_DWG2DXF")
        else None
    )
    executable = (
        str(configured.expanduser().resolve())
        if configured is not None
        else shutil.which("dwg2dxf")
    )
    if not executable or not Path(executable).is_file():
        raise RuntimeError(
            "DWG conversion requires LibreDWG dwg2dxf; pass --dwg2dxf or "
            "LIBREDWG_DWG2DXF"
        )

    class ConvertedDxf:
        def __enter__(self):
            self.temp = tempfile.TemporaryDirectory(prefix="kookmin_cad_v4_")
            output = Path(self.temp.name) / "track-plan-cleaned-v4.dxf"
            env = os.environ.copy()
            library_dir = str(Path(executable).resolve().parents[1] / "lib")
            env["LD_LIBRARY_PATH"] = ":".join(
                item
                for item in (library_dir, env.get("LD_LIBRARY_PATH", ""))
                if item
            )
            subprocess.run(
                [
                    executable,
                    "--as",
                    "r2013",
                    "--overwrite",
                    "--file",
                    str(output),
                    str(source),
                ],
                check=True,
                env=env,
            )
            return output

        def __exit__(self, *_):
            self.temp.cleanup()
            return False

    return ConvertedDxf()


def closed(points: list[tuple[float, float]]) -> bool:
    return len(points) >= 3 and math.dist(points[0], points[-1]) < 1.0e-6


def bbox_area(points: list[tuple[float, float]]) -> float:
    x0, y0, x1, y1 = legacy.path_bbox(points)
    return max(0.0, x1 - x0) * max(0.0, y1 - y0)


def polygon_area(points: list[tuple[float, float]]) -> float:
    return abs(cv2.contourArea(np.asarray(points, dtype=np.float32)))


def signed_polygon_area(points) -> float:
    values = np.asarray(points, dtype=np.float64)
    following = np.roll(values, -1, axis=0)
    return 0.5 * float(
        np.sum(values[:, 0] * following[:, 1] - following[:, 0] * values[:, 1])
    )


def metric_bbox(points: list[tuple[float, float]]):
    return legacy.path_bbox(points)


def bbox_pair_score(first, second) -> float:
    a = metric_bbox(first)
    b = metric_bbox(second)
    return sum(abs(left - right) for left, right in zip(a, b))


def select_geometry(raw_paths):
    """Find the outer room and the two doubled track-line contours."""
    wall_index = max(range(len(raw_paths)), key=lambda idx: bbox_area(raw_paths[idx]))
    wall = raw_paths[wall_index]
    wall_bbox = legacy.path_bbox(wall)
    wall_width = wall_bbox[2] - wall_bbox[0]
    wall_height = wall_bbox[3] - wall_bbox[1]
    centre_x = (wall_bbox[0] + wall_bbox[2]) * 0.5
    centre_y = (wall_bbox[1] + wall_bbox[3]) * 0.5

    metric_paths = [
        [
            (
                (x - centre_x) * SOURCE_UNIT_TO_M,
                (y - centre_y) * SOURCE_UNIT_TO_M,
            )
            for x, y in points
        ]
        for points in raw_paths
    ]
    wall_metric = metric_paths[wall_index]
    wall_width_m = wall_width * SOURCE_UNIT_TO_M
    wall_height_m = wall_height * SOURCE_UNIT_TO_M

    candidates = []
    for index, points in enumerate(metric_paths):
        if index == wall_index or not closed(points):
            continue
        x0, y0, x1, y1 = metric_bbox(points)
        width = x1 - x0
        height = y1 - y0
        length = legacy.polyline_length(points)
        if (
            0.50 * wall_width_m <= width <= 0.78 * wall_width_m
            and 0.44 * wall_height_m <= height <= 0.72 * wall_height_m
            and length >= 20.0
        ):
            candidates.append(index)

    pair_options = []
    for offset, first_index in enumerate(candidates):
        for second_index in candidates[offset + 1 :]:
            first = metric_paths[first_index]
            second = metric_paths[second_index]
            score = bbox_pair_score(first, second)
            length_ratio = abs(
                legacy.polyline_length(first) - legacy.polyline_length(second)
            ) / max(legacy.polyline_length(first), legacy.polyline_length(second))
            if score <= 0.45 and length_ratio <= 0.025:
                pair_options.append((score + 5.0 * length_ratio, first_index, second_index))

    selected_pairs = []
    used = set()
    for _, first_index, second_index in sorted(pair_options):
        if first_index in used or second_index in used:
            continue
        selected_pairs.append((first_index, second_index))
        used.update((first_index, second_index))
    if len(selected_pairs) != 2:
        raise RuntimeError(
            f"expected exactly two track contour pairs, found {selected_pairs}"
        )

    selected_pairs.sort(
        key=lambda pair: sum(bbox_area(metric_paths[index]) for index in pair),
        reverse=True,
    )
    return {
        "metric_paths": metric_paths,
        "wall_index": wall_index,
        "wall": wall_metric,
        "wall_size": (wall_width_m, wall_height_m),
        "outer_pair": selected_pairs[0],
        "inner_pair": selected_pairs[1],
    }


def pixel_path(points, world_width, world_height):
    return np.asarray(
        [
            (
                int(round((x + world_width * 0.5) * PIXELS_PER_M)),
                int(round((world_height * 0.5 - y) * PIXELS_PER_M)),
            )
            for x, y in points
        ],
        dtype=np.int32,
    )


def filled_path(points, shape, world_width, world_height):
    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillPoly(mask, [pixel_path(points, world_width, world_height)], 255)
    return mask


def resample_closed_path(points, count):
    values = np.asarray(points, dtype=np.float64)
    if np.linalg.norm(values[0] - values[-1]) < 1.0e-9:
        values = values[:-1]
    following = np.roll(values, -1, axis=0)
    segment_lengths = np.linalg.norm(following - values, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    distances = np.linspace(0.0, cumulative[-1], int(count), endpoint=False)
    indices = np.searchsorted(cumulative, distances, side="right") - 1
    indices = np.clip(indices, 0, len(values) - 1)
    local = (distances - cumulative[indices]) / np.maximum(
        segment_lengths[indices], 1.0e-9
    )
    return values[indices] + local[:, None] * (
        following[indices] - values[indices]
    )


def cad_track_centerline(geometry, sample_count=4096):
    """Average the two road-facing paint edges into an ordered centreline."""
    paths = geometry["metric_paths"]
    outer_index = min(
        geometry["outer_pair"], key=lambda idx: polygon_area(paths[idx])
    )
    inner_index = max(
        geometry["inner_pair"], key=lambda idx: polygon_area(paths[idx])
    )
    outer = resample_closed_path(paths[outer_index], sample_count)
    inner = resample_closed_path(paths[inner_index], sample_count)

    # The two CAD contours use opposite winding directions and have different
    # lengths.  Equal normalized arc-length therefore drifts by almost 20 cm
    # through the S bends.  Match each outer sample to its spatially nearest
    # inner sample while requiring the matches to advance monotonically.
    if signed_polygon_area(outer) * signed_polygon_area(inner) < 0.0:
        inner = inner[::-1].copy()
    nearest_indices = []
    for chunk in np.array_split(outer, 32):
        squared = np.sum((chunk[:, None, :] - inner[None, :, :]) ** 2, axis=2)
        nearest_indices.extend(np.argmin(squared, axis=1).tolist())
    nearest_indices = np.asarray(nearest_indices, dtype=np.int64)
    unwrapped = np.unwrap(
        nearest_indices * (2.0 * math.pi / len(inner))
    ) * (len(inner) / (2.0 * math.pi))
    advances = np.diff(unwrapped)
    if float(np.min(advances)) < -1.1 or not 0.95 <= (
        unwrapped[-1] - unwrapped[0]
    ) / len(inner) <= 1.05:
        raise RuntimeError("white-line correspondence does not follow the CAD path")

    aligned_inner = inner[nearest_indices]
    separations = np.linalg.norm(aligned_inner - outer, axis=1)
    width_stats = {
        "minimum_m": float(np.min(separations)),
        "p05_m": float(np.percentile(separations, 5)),
        "median_m": float(np.median(separations)),
        "p95_m": float(np.percentile(separations, 95)),
        "maximum_m": float(np.max(separations)),
    }
    separation = width_stats["median_m"]
    if not (
        0.70 <= float(np.percentile(separations, 5))
        and float(np.percentile(separations, 95)) <= 0.90
    ):
        raise RuntimeError(
            "unexpected white-line separation: "
            f"p05={np.percentile(separations, 5):.3f} m, "
            f"median={separation:.3f} m, "
            f"p95={np.percentile(separations, 95):.3f} m"
        )
    centerline = 0.5 * (outer + aligned_inner)
    # Store anchors clockwise. TrackReference reverses them for the default
    # counter-clockwise competition direction (leftward on the top straight).
    if signed_polygon_area(centerline) > 0.0:
        centerline = centerline[::-1].copy()
    return centerline, width_stats


def track_reference_frames(centerline, spacing_m=0.25):
    sampled = resample_closed_path(
        centerline,
        max(16, int(math.ceil(legacy.polyline_length(centerline) / spacing_m))),
    )
    return "".join(
        f"<frame name=\"track_reference_anchor_{index:03d}\"><pose>{x:.6f} {y:.6f} 0 0 0 0</pose></frame>"
        for index, (x, y) in enumerate(sampled, start=1)
    )


def render_masks(geometry, world_width, world_height):
    image_width = int(round(world_width * PIXELS_PER_M))
    image_height = int(round(world_height * PIXELS_PER_M))
    shape = (image_height, image_width)
    metric_paths = geometry["metric_paths"]

    pair_masks = []
    for pair in (geometry["outer_pair"], geometry["inner_pair"]):
        first = filled_path(metric_paths[pair[0]], shape, world_width, world_height)
        second = filled_path(metric_paths[pair[1]], shape, world_width, world_height)
        pair_masks.append(cv2.bitwise_xor(first, second))
    white = cv2.bitwise_or(pair_masks[0], pair_masks[1])

    outer_contour_index = max(
        geometry["outer_pair"], key=lambda idx: polygon_area(metric_paths[idx])
    )
    inner_hole_index = min(
        geometry["inner_pair"], key=lambda idx: polygon_area(metric_paths[idx])
    )
    road = filled_path(
        metric_paths[outer_contour_index], shape, world_width, world_height
    )
    inner_hole = filled_path(
        metric_paths[inner_hole_index], shape, world_width, world_height
    )
    road[inner_hole > 0] = 0

    excluded = {
        geometry["wall_index"],
        *geometry["outer_pair"],
        *geometry["inner_pair"],
    }
    yellow = np.zeros(shape, dtype=np.uint8)
    dash_indices = []
    for index, points in enumerate(metric_paths):
        if index in excluded or not closed(points):
            continue
        x0, y0, x1, y1 = metric_bbox(points)
        width = x1 - x0
        height = y1 - y0
        if max(width, height) > 0.80 or min(width, height) > 0.20:
            continue
        pixels = pixel_path(points, world_width, world_height)
        rows = np.clip(pixels[:, 1], 0, shape[0] - 1)
        cols = np.clip(pixels[:, 0], 0, shape[1] - 1)
        if float(np.mean(road[rows, cols] > 0)) < 0.65:
            continue
        cv2.fillPoly(yellow, [pixels], 255)
        dash_indices.append(index)

    if len(dash_indices) < 20:
        raise RuntimeError(f"too few centre dashes selected: {len(dash_indices)}")
    return road, white, yellow, dash_indices


def save_rgba_parts(mask, color, prefix, world_width, world_height):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (mask > 0).astype(np.uint8), 8
    )
    specs = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < 4:
            continue
        left = max(0, int(stats[label, cv2.CC_STAT_LEFT]) - 3)
        top = max(0, int(stats[label, cv2.CC_STAT_TOP]) - 3)
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        right = min(mask.shape[1], left + width + 6)
        bottom = min(mask.shape[0], top + height + 6)
        alpha = np.where(labels == label, mask, 0).astype(np.uint8)[
            top:bottom, left:right
        ]
        rgba = np.zeros((*alpha.shape, 4), dtype=np.uint8)
        rgba[..., :3] = color[:3]
        rgba[..., 3] = alpha
        name = f"{prefix}_{len(specs) + 1:03d}"
        path = TEXTURE_DIR / f"kookmin_{name}.png"
        Image.fromarray(rgba, "RGBA").save(path)
        centre_x_px = (left + right) * 0.5
        centre_y_px = (top + bottom) * 0.5
        specs.append(
            {
                "name": name,
                "texture": path.relative_to(ROOT).as_posix(),
                "pose_xy": [
                    centre_x_px / PIXELS_PER_M - world_width * 0.5,
                    world_height * 0.5 - centre_y_px / PIXELS_PER_M,
                ],
                "size_xy": [
                    (right - left) / PIXELS_PER_M,
                    (bottom - top) / PIXELS_PER_M,
                ],
            }
        )
    return specs


def overlay_model(spec, z, emissive):
    name = spec["name"]
    x, y = spec["pose_xy"]
    size_x, size_y = spec["size_xy"]
    texture = f"file://{spec['texture']}"
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>
      <link name="{name}_surface">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{size_x:.4f} {size_y:.4f}</size></plane></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <emissive>{emissive}</emissive>
            <pbr><metal><albedo_map>{texture}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>"""


def wall_model(points):
    clean = list(points)
    if len(clean) > 1 and math.dist(clean[0], clean[-1]) < 1.0e-6:
        clean.pop()
    links = legacy.polyline_links(
        clean,
        "cad_outer_wall",
        WALL_WIDTH_M,
        WALL_HEIGHT_M,
        WALL_HEIGHT_M * 0.5,
        WALL_COLOR,
        collision=True,
        overlap=WALL_WIDTH_M,
    )
    return f"""
    <model name="cad_outer_wall">
      <static>true</static>
      {links}
    </model>"""


def track_model(world_width, world_height):
    texture = f"file://{BASE_TEXTURE.relative_to(ROOT).as_posix()}"
    return f"""
    <model name="kookmin_track">
      <static>true</static>
      <link name="floor_base">
        <pose>0 0 -0.015 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>{world_width:.4f} {world_height:.4f} 0.0300</size></box></geometry>
          <surface><friction><ode><mu>50</mu><mu2>50</mu2></ode><bullet><friction>1</friction><rolling_friction>0.1</rolling_friction></bullet></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{world_width:.4f} {world_height:.4f} 0.0300</size></box></geometry>
          {legacy.material_xml(ROAD_COLOR)}
        </visual>
      </link>
      <link name="track_texture_surface">
        <pose>0 0 0.006 0 0 0</pose>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{world_width:.4f} {world_height:.4f}</size></plane></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{texture}</albedo_map><roughness>1</roughness><metalness>0</metalness></metal></pbr></material>
        </visual>
      </link>
    </model>"""


def world_xml(
    geometry,
    world_width,
    world_height,
    white_specs,
    yellow_specs,
    spawn_y,
    centerline,
):
    legacy.XYCAR_SPAWN_X = -2.70
    legacy.XYCAR_SPAWN_Y = spawn_y
    legacy.XYCAR_SPAWN_Z = 0.05
    legacy.XYCAR_SPAWN_YAW = math.pi
    white_models = "".join(
        overlay_model(spec, 0.014 + index * 0.0002, "0.9490 0.9490 0.9255 1")
        for index, spec in enumerate(white_specs)
    )
    yellow_models = "".join(
        overlay_model(spec, 0.018 + index * 0.00005, "1.0000 0.7686 0.0471 1")
        for index, spec in enumerate(yellow_specs)
    )
    reference_frames = track_reference_frames(centerline)
    return f"""<?xml version="1.0"?>
<sdf version="1.7">
  <world name="kookmin_xycar_track_cad_v4">
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-6 2.3e-5 -4.2e-5</magnetic_field>
    <atmosphere type="adiabatic"/>
    <physics name="default_physics" default="1" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>1000</real_time_update_rate></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <scene><ambient>0.75 0.75 0.75 1</ambient><background>0.82 0.84 0.86 1</background><shadows>false</shadows></scene>
    <light name="sun" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>0.82 0.82 0.82 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.45 0.20 -0.88</direction></light>
    {track_model(world_width, world_height)}
    {wall_model(geometry['wall'])}
    {white_models}
    {yellow_models}
    {reference_frames}
    {legacy.vehicle_model()}
  </world>
</sdf>
"""


def main() -> None:
    args = arguments()
    source = args.source.expanduser().resolve()
    with resolved_dxf(source, args.dwg2dxf) as dxf:
        raw_paths = legacy.dxf_paths(dxf)
    if not raw_paths:
        raise RuntimeError("the converted drawing contains no supported paths")

    geometry = select_geometry(raw_paths)
    wall_width, wall_height = geometry["wall_size"]
    world_width = math.ceil((wall_width + 2.0 * WORLD_MARGIN_M) * 10.0) / 10.0
    world_height = math.ceil((wall_height + 2.0 * WORLD_MARGIN_M) * 10.0) / 10.0
    road, white, yellow, dash_indices = render_masks(
        geometry, world_width, world_height
    )
    centerline, clear_width = cad_track_centerline(geometry)

    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)
    base = np.empty((*road.shape, 3), dtype=np.uint8)
    base[:] = ROAD_COLOR[:3]
    Image.fromarray(base, "RGB").save(BASE_TEXTURE)
    white_specs = save_rgba_parts(
        white, WHITE_COLOR, "white_line_cad_v4", world_width, world_height
    )
    yellow_specs = save_rgba_parts(
        yellow,
        YELLOW_COLOR,
        "yellow_centerline_cad_v4_dash",
        world_width,
        world_height,
    )

    preview = base.copy()
    preview[white > 0] = WHITE_COLOR[:3]
    preview[yellow > 0] = YELLOW_COLOR[:3]
    wall_pixels = pixel_path(geometry["wall"], world_width, world_height)
    cv2.polylines(preview, [wall_pixels], False, WALL_COLOR[:3], 8, cv2.LINE_AA)
    Image.fromarray(preview, "RGB").save(PREVIEW_PATH)

    dash_centres = []
    for index in dash_indices:
        x0, y0, x1, y1 = metric_bbox(geometry["metric_paths"][index])
        if x1 - x0 > y1 - y0:
            dash_centres.append(((x0 + x1) * 0.5, (y0 + y1) * 0.5))
    upper_y = max(y for _, y in dash_centres)
    top_dashes = [y for _, y in dash_centres if y >= upper_y - 0.20]
    spawn_y = float(np.median(top_dashes))

    WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
    rendered_world = world_xml(
        geometry,
        world_width,
        world_height,
        white_specs,
        yellow_specs,
        spawn_y,
        centerline,
    )
    rendered_world = "\n".join(
        line.rstrip() for line in rendered_world.splitlines()
    ) + "\n"
    WORLD_PATH.write_text(rendered_world, encoding="utf-8")
    if args.promote:
        FINAL_WORLD_PATH.write_text(rendered_world, encoding="utf-8")

    old_paths = legacy.cad_metric_paths() or []
    old_bbox = (
        legacy.merge_bboxes([legacy.path_bbox(points) for points in old_paths])
        if old_paths
        else None
    )
    outer_pair_paths = [
        geometry["metric_paths"][index] for index in geometry["outer_pair"]
    ]
    inner_pair_paths = [
        geometry["metric_paths"][index] for index in geometry["inner_pair"]
    ]
    metrics = {
        "source": str(source),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "source_unit": "millimetre",
        "raw_closed_polylines": len(raw_paths),
        "excluded_architectural_polylines": len(raw_paths) - 5 - len(dash_indices),
        "outer_wall_size_m": [wall_width, wall_height],
        "world_size_m": [world_width, world_height],
        "outer_white_bbox_m": list(
            legacy.merge_bboxes([metric_bbox(points) for points in outer_pair_paths])
        ),
        "inner_white_bbox_m": list(
            legacy.merge_bboxes([metric_bbox(points) for points in inner_pair_paths])
        ),
        "yellow_dash_polygons": len(dash_indices),
        "median_clear_road_width_m": clear_width["median_m"],
        "clear_road_width_statistics": clear_width,
        "track_reference_length_m": legacy.polyline_length(centerline),
        "track_reference_anchors": max(
            16,
            int(math.ceil(legacy.polyline_length(centerline) / 0.25)),
        ),
        "spawn_pose": [-2.70, spawn_y, 0.05, math.pi],
        "previous_cad_white_bbox_m": list(old_bbox) if old_bbox else None,
        "outputs": {
            "world": str(WORLD_PATH.relative_to(ROOT)),
            "final_world": (
                str(FINAL_WORLD_PATH.relative_to(ROOT)) if args.promote else None
            ),
            "preview": str(PREVIEW_PATH.relative_to(ROOT)),
        },
    }
    METRICS_PATH.parent.mkdir(parents=True, exist_ok=True)
    METRICS_PATH.write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    print(f"source paths: {len(raw_paths)}")
    print(f"outer wall: {wall_width:.3f} m x {wall_height:.3f} m")
    print(f"track contour pairs: {geometry['outer_pair']} / {geometry['inner_pair']}")
    print(f"yellow dash polygons: {len(dash_indices)}")
    print(f"spawn: -2.700 {spawn_y:.3f} yaw=pi")
    print(f"wrote {WORLD_PATH}")
    if args.promote:
        print(f"promoted {FINAL_WORLD_PATH}")
    print(f"wrote {PREVIEW_PATH}")
    print(f"wrote {METRICS_PATH}")


if __name__ == "__main__":
    main()
