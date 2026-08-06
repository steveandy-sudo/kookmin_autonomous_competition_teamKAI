#!/usr/bin/env python3
"""Add mission objects to the approved CAD-v4 Gazebo track.

The generated world keeps the track geometry immutable and adds a separate
mission layer for cone driving, vehicle avoidance, and traffic-light tests.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = (
    ROOT
    / "xycar_ws"
    / "src"
    / "xycar_map_nav"
    / "config"
    / "sim_mission_layout.yaml"
)
METRICS_PATH = ROOT / "analysis" / "sim_mission_layout_generated.json"


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the CAD-v4 Gazebo mission-object world."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def values(items, count: int, label: str) -> tuple[float, ...]:
    result = tuple(float(item) for item in items)
    if len(result) != count:
        raise ValueError(f"{label} must contain {count} values")
    return result


def material(color, *, emissive=None) -> str:
    rgba = " ".join(f"{item:.4f}" for item in values(color, 4, "color"))
    emission = "0 0 0 1" if emissive is None else " ".join(
        f"{item:.4f}" for item in values(emissive, 4, "emissive")
    )
    return (
        f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>"
        f"<emissive>{emission}</emissive>"
        "<pbr><metal><roughness>0.72</roughness><metalness>0.0</metalness>"
        "</metal></pbr></material>"
    )


def car_visuals(size, color) -> str:
    length, width, height = values(size, 3, "vehicle size")
    wheel_radius = 0.052
    wheel_width = 0.035
    wheel_x = 0.32 * length
    wheel_y = 0.53 * width
    wheel_z = -0.5 * height + wheel_radius
    wheels = []
    for index, (x, y) in enumerate(
        (
            (wheel_x, wheel_y),
            (wheel_x, -wheel_y),
            (-wheel_x, wheel_y),
            (-wheel_x, -wheel_y),
        ),
        start=1,
    ):
        wheels.append(
            f"""
        <visual name="wheel_{index}">
          <pose>{x:.4f} {y:.4f} {wheel_z:.4f} 1.570796 0 0</pose>
          <geometry><cylinder><radius>{wheel_radius:.4f}</radius><length>{wheel_width:.4f}</length></cylinder></geometry>
          {material((0.025, 0.025, 0.025, 1.0))}
        </visual>"""
        )
    body = material(color)
    glass = material((0.04, 0.07, 0.10, 1.0), emissive=(0.01, 0.02, 0.03, 1.0))
    return f"""
        <collision name="body_collision">
          <geometry><box><size>{length:.4f} {width:.4f} {height:.4f}</size></box></geometry>
        </collision>
        <visual name="lower_body">
          <geometry><box><size>{length:.4f} {width:.4f} {height:.4f}</size></box></geometry>
          {body}
        </visual>
        <visual name="cabin">
          <pose>{-0.04 * length:.4f} 0 {0.58 * height:.4f} 0 0 0</pose>
          <geometry><box><size>{0.52 * length:.4f} {0.82 * width:.4f} {0.55 * height:.4f}</size></box></geometry>
          {body}
        </visual>
        <visual name="front_window">
          <pose>{0.235 * length:.4f} 0 {0.62 * height:.4f} 0 -0.35 0</pose>
          <geometry><box><size>0.0120 {0.67 * width:.4f} {0.34 * height:.4f}</size></box></geometry>
          {glass}
        </visual>
        {''.join(wheels)}"""


def vehicle_model(spec: dict, *, moving: bool) -> str:
    name = str(spec["name"])
    pose = values(spec["pose"], 6, f"{name} pose")
    size = values(spec["size"], 3, f"{name} size")
    pose_text = " ".join(f"{item:.6f}" for item in pose)
    inertial = ""
    plugin = ""
    static = "false" if moving else "true"
    if moving:
        mass = 2.0
        length, width, height = size
        ixx = mass * (width * width + height * height) / 12.0
        iyy = mass * (length * length + height * height) / 12.0
        izz = mass * (length * length + width * width) / 12.0
        inertial = f"""
        <inertial>
          <mass>{mass:.3f}</mass>
          <inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz></inertia>
        </inertial>"""
        speed = float(spec["speed_mps"])
        plugin = f"""
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
        <initial_linear>{speed:.4f} 0 0</initial_linear>
        <initial_angular>0 0 0</initial_angular>
      </plugin>
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_model_pose>true</publish_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <update_frequency>20</update_frequency>
      </plugin>"""
    return f"""
    <model name="{name}">
      <static>{static}</static>
      <pose>{pose_text}</pose>
      <link name="body">
        {inertial}
        {car_visuals(size, spec['color'])}
      </link>
      {plugin}
    </model>"""


def cone_model(name: str, x: float, y: float, radius: float, height: float) -> str:
    orange = material((1.0, 0.24, 0.015, 1.0), emissive=(0.18, 0.025, 0.0, 1.0))
    white = material((0.95, 0.95, 0.90, 1.0), emissive=(0.12, 0.12, 0.10, 1.0))
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <link name="cone">
        <collision name="collision">
          <pose>0 0 {0.5 * height:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{0.82 * radius:.4f}</radius><length>{height:.4f}</length></cylinder></geometry>
        </collision>
        <visual name="base">
          <pose>0 0 0.018 0 0 0</pose>
          <geometry><box><size>{2.35 * radius:.4f} {2.35 * radius:.4f} 0.036</size></box></geometry>
          {orange}
        </visual>
        <visual name="body">
          <pose>0 0 {0.5 * height:.4f} 0 0 0</pose>
          <geometry><cone><radius>{radius:.4f}</radius><length>{height:.4f}</length></cone></geometry>
          {orange}
        </visual>
        <visual name="reflective_band">
          <pose>0 0 {0.55 * height:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{0.54 * radius:.4f}</radius><length>{0.16 * height:.4f}</length></cylinder></geometry>
          {white}
        </visual>
      </link>
    </model>"""


def cone_positions(spec: dict) -> list[tuple[str, float, float]]:
    start = float(spec["start_y"])
    end = float(spec["end_y"])
    spacing = float(spec["spacing_m"])
    if spacing <= 0.0 or start <= end:
        raise ValueError("cone corridor requires start_y > end_y and positive spacing")
    count = int(math.floor((start - end) / spacing)) + 1
    positions = []
    for index in range(count):
        y = start - index * spacing
        positions.append((f"{spec['name_prefix']}_east_{index + 1:02d}", float(spec["east_x"]), y))
        positions.append((f"{spec['name_prefix']}_west_{index + 1:02d}", float(spec["west_x"]), y))
    return positions


def traffic_light_model(spec: dict) -> str:
    name = str(spec["name"])
    pose = " ".join(f"{value:.6f}" for value in values(spec["pose"], 6, f"{name} pose"))
    active = str(spec.get("active_color", "green")).lower()
    colors = {
        "red": (0.95, 0.02, 0.01, 1.0),
        "yellow": (1.0, 0.52, 0.01, 1.0),
        "green": (0.01, 0.90, 0.08, 1.0),
    }
    lens_names = ("red", "yellow", "green", "green")
    lens_ys = (-0.18, -0.06, 0.06, 0.18)
    lenses = []
    active_index = 3 if active == "green" else lens_names.index(active)
    for index, (lens_name, y) in enumerate(zip(lens_names, lens_ys)):
        lit = index == active_index
        base = colors[lens_name] if lit else (0.055, 0.055, 0.050, 1.0)
        emission = colors[lens_name] if lit else (0.0, 0.0, 0.0, 1.0)
        lenses.append(
            f"""
        <visual name="lamp_{index + 1}_{lens_name}">
          <pose>0.066 {0.58 + y:.4f} 0.72 0 1.570796 0</pose>
          <geometry><cylinder><radius>0.043</radius><length>0.018</length></cylinder></geometry>
          {material(base, emissive=emission)}
        </visual>"""
        )
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="signal">
        <collision name="pole_collision"><pose>0 0 0.36 0 0 0</pose><geometry><cylinder><radius>0.028</radius><length>0.72</length></cylinder></geometry></collision>
        <visual name="pole"><pose>0 0 0.36 0 0 0</pose><geometry><cylinder><radius>0.028</radius><length>0.72</length></cylinder></geometry>{material((0.16, 0.16, 0.16, 1.0))}</visual>
        <visual name="arm"><pose>0 0.29 0.72 1.570796 0 0</pose><geometry><cylinder><radius>0.025</radius><length>0.58</length></cylinder></geometry>{material((0.16, 0.16, 0.16, 1.0))}</visual>
        <visual name="housing"><pose>0 0.58 0.72 0 0 0</pose><geometry><box><size>0.12 0.50 0.16</size></box></geometry>{material((0.018, 0.022, 0.018, 1.0))}</visual>
        {''.join(lenses)}
      </link>
    </model>"""


def build_mission_models(config: dict) -> tuple[str, dict]:
    moving = config["moving_vehicle"]
    fixed = list(config.get("fixed_vehicles", []))
    cones = cone_positions(config["cone_corridor"])
    cone_spec = config["cone_corridor"]
    models = [vehicle_model(moving, moving=True)]
    models.extend(vehicle_model(spec, moving=False) for spec in fixed)
    models.append(traffic_light_model(config["traffic_light"]))
    models.extend(
        cone_model(
            name,
            x,
            y,
            float(cone_spec["base_radius_m"]),
            float(cone_spec["height_m"]),
        )
        for name, x, y in cones
    )
    metrics = {
        "moving_vehicle": moving,
        "fixed_vehicles": fixed,
        "traffic_light": config["traffic_light"],
        "cone_count": len(cones),
        "cone_positions": [
            {"name": name, "x": x, "y": y} for name, x, y in cones
        ],
    }
    return "\n".join(models), metrics


def main() -> None:
    args = arguments()
    config_path = args.config.expanduser().resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    world_config = config["world"]
    base_path = (ROOT / world_config["base_sdf"]).resolve()
    output_path = (
        args.output.expanduser().resolve()
        if args.output is not None
        else (ROOT / world_config["output_sdf"]).resolve()
    )
    base = base_path.read_text(encoding="utf-8")
    ET.fromstring(base)
    closing = "  </world>"
    if base.count(closing) != 1:
        raise RuntimeError(f"expected exactly one world closing tag in {base_path}")
    models, metrics = build_mission_models(config)
    world_name = str(world_config["name"])
    base = base.replace(
        '<world name="kookmin_xycar_track_cad_v4">',
        f'<world name="{world_name}">',
        1,
    )
    rendered = base.replace(closing, f"{models}\n{closing}", 1)
    rendered = "\n".join(line.rstrip() for line in rendered.splitlines()) + "\n"
    ET.fromstring(rendered)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(rendered, encoding="utf-8")

    metrics.update(
        {
            "config": str(config_path.relative_to(ROOT)),
            "base_sdf": str(base_path.relative_to(ROOT)),
            "output_sdf": str(output_path.relative_to(ROOT)),
            "world_name": world_name,
        }
    )
    METRICS_PATH.parent.mkdir(parents=True, exist_ok=True)
    METRICS_PATH.write_text(
        json.dumps(metrics, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"mission objects: moving=1 fixed={len(config.get('fixed_vehicles', []))} cones={metrics['cone_count']} signal=1")
    print(f"wrote {output_path}")
    print(f"wrote {METRICS_PATH}")


if __name__ == "__main__":
    main()
