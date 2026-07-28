#!/usr/bin/env python3
"""Render the saved SDF lane entities into a metric top-down image."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image


LANE_MODEL_PREFIXES = (
    "white_line_",
    "yellow_centerline_",
)


def parse_pose(text: str | None) -> tuple[float, ...]:
    values = [float(value) for value in (text or "").split()]
    values.extend([0.0] * (6 - len(values)))
    return tuple(values[:6])


def resolve_texture(uri: str, project_root: Path) -> Path:
    value = uri.strip()
    if value.startswith("file://"):
        value = value[len("file://"):]
    path = Path(value)
    if not path.is_absolute():
        path = project_root / path
    return path.resolve()


def render_sdf_lanes(
    sdf_path: Path,
    *,
    project_root: Path,
    output_path: Path,
    world_width_m: float,
    world_height_m: float,
    resolution_m: float,
) -> dict:
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"SDF has no world element: {sdf_path}")

    width_px = int(round(world_width_m / resolution_m))
    height_px = int(round(world_height_m / resolution_m))
    canvas = Image.new("RGBA", (width_px, height_px), (96, 96, 94, 255))
    rendered = []

    models = []
    for model in world.findall("model"):
        name = str(model.get("name", ""))
        if not name.startswith(LANE_MODEL_PREFIXES):
            continue
        pose = parse_pose(model.findtext("pose"))
        models.append((pose[2], name, model, pose))

    for _, name, model, model_pose in sorted(models):
        visual = model.find("./link/visual")
        if visual is None:
            continue
        size_text = visual.findtext("./geometry/plane/size")
        texture_uri = visual.findtext(
            "./material/pbr/metal/albedo_map"
        )
        if not size_text or not texture_uri:
            continue
        size = [float(value) for value in size_text.split()]
        if len(size) != 2:
            raise ValueError(f"{name} has invalid plane size: {size_text}")
        texture_path = resolve_texture(texture_uri, project_root)
        if not texture_path.is_file():
            raise ValueError(f"{name} texture is missing: {texture_path}")

        texture = Image.open(texture_path).convert("RGBA")
        target_size = (
            max(1, int(round(size[0] / resolution_m))),
            max(1, int(round(size[1] / resolution_m))),
        )
        texture = texture.resize(
            target_size,
            getattr(Image, "Resampling", Image).LANCZOS,
        )
        yaw = model_pose[5] + parse_pose(visual.findtext("pose"))[5]
        if abs(yaw) > 1.0e-8:
            texture = texture.rotate(
                -math.degrees(yaw),
                resample=getattr(Image, "Resampling", Image).BICUBIC,
                expand=True,
            )

        center_x = (model_pose[0] + world_width_m * 0.5) / resolution_m
        center_y = (world_height_m * 0.5 - model_pose[1]) / resolution_m
        left = int(round(center_x - texture.width * 0.5))
        top = int(round(center_y - texture.height * 0.5))
        canvas.alpha_composite(texture, (left, top))
        rendered.append(
            {
                "name": name,
                "x": model_pose[0],
                "y": model_pose[1],
                "yaw_rad": yaw,
                "texture": str(texture_path),
            }
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.convert("RGB").save(output_path)
    return {
        "sdf": str(sdf_path),
        "output": str(output_path),
        "resolution_m": resolution_m,
        "width_px": width_px,
        "height_px": height_px,
        "rendered_lane_models": len(rendered),
        "models": rendered,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", required=True, type=Path)
    parser.add_argument("--project-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--world-width-m", type=float, default=22.5)
    parser.add_argument("--world-height-m", type=float, default=13.5)
    parser.add_argument("--resolution-m", type=float, default=0.01)
    options = parser.parse_args()
    result = render_sdf_lanes(
        options.sdf.expanduser().resolve(),
        project_root=options.project_root.expanduser().resolve(),
        output_path=options.output.expanduser().resolve(),
        world_width_m=options.world_width_m,
        world_height_m=options.world_height_m,
        resolution_m=options.resolution_m,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
