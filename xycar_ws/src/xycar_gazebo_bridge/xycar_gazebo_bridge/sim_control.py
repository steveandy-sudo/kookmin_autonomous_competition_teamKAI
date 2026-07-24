"""Coordinate-based Gazebo Sim vehicle and obstacle control."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import re
import shlex
import shutil
import subprocess
from typing import Callable


@dataclass(frozen=True)
class ObstaclePreset:
    shape: str
    size: tuple[float, ...]
    color: tuple[float, float, float, float]
    default_z: float


OBSTACLE_PRESETS = {
    "box": ObstaclePreset(
        "box", (0.50, 0.50, 0.50), (0.76, 0.36, 0.12, 1.0), 0.25
    ),
    "person": ObstaclePreset(
        "cylinder", (0.18, 1.70), (0.16, 0.38, 0.80, 1.0), 0.85
    ),
    "chair": ObstaclePreset(
        "box", (0.48, 0.48, 0.80), (0.18, 0.18, 0.20, 1.0), 0.40
    ),
    "table": ObstaclePreset(
        "box", (1.20, 0.70, 0.75), (0.42, 0.25, 0.12, 1.0), 0.375
    ),
    "camera_pole": ObstaclePreset(
        "cylinder", (0.035, 1.50), (0.08, 0.08, 0.09, 1.0), 0.75
    ),
    # Simulation branch reference: 0.18 x 0.18 x 0.36 m.
    "traffic_cone": ObstaclePreset(
        "box", (0.18, 0.18, 0.36), (0.8627, 0.3137, 0.0941, 1.0), 0.18
    ),
    "glass_panel": ObstaclePreset(
        "box", (1.50, 0.03, 0.90), (0.35, 0.70, 0.85, 0.28), 0.45
    ),
}


def _validate_name(name: str) -> str:
    name = str(name).strip()
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", name):
        raise ValueError(
            "entity name may contain only letters, numbers, _, -, and ."
        )
    return name


def _escape_proto_string(value: str) -> str:
    return (
        value.replace("\\", "\\\\")
        .replace('"', '\\"')
        .replace("\n", "\\n")
    )


def obstacle_sdf(
    name: str,
    preset: str,
    *,
    collision_enabled: bool = True,
) -> str:
    name = _validate_name(name)
    if preset not in OBSTACLE_PRESETS:
        raise ValueError(f"unknown obstacle preset: {preset}")
    item = OBSTACLE_PRESETS[preset]
    color = " ".join(str(value) for value in item.color)
    if item.shape == "box":
        size = " ".join(str(value) for value in item.size)
        geometry = f"<box><size>{size}</size></box>"
    else:
        radius, length = item.size
        geometry = (
            "<cylinder>"
            f"<radius>{radius}</radius><length>{length}</length>"
            "</cylinder>"
        )
    collision = (
        f"<collision name='collision'><geometry>{geometry}</geometry></collision>"
        if collision_enabled
        else ""
    )
    return (
        "<sdf version='1.10'>"
        f"<model name='{name}'>"
        "<static>true</static>"
        "<link name='body'>"
        f"{collision}"
        "<visual name='visual'>"
        f"<geometry>{geometry}</geometry>"
        "<material>"
        f"<ambient>{color}</ambient><diffuse>{color}</diffuse>"
        "</material>"
        "<cast_shadows>false</cast_shadows>"
        "</visual>"
        "</link>"
        "</model>"
        "</sdf>"
    )


class GazeboController:
    def __init__(
        self,
        world_name: str = "kookmin_xycar_track",
        *,
        dry_run: bool = False,
        runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
    ):
        self.world_name = str(world_name)
        self.dry_run = bool(dry_run)
        self.runner = runner

    @property
    def available(self) -> bool:
        return shutil.which("gz") is not None

    def _service(
        self,
        endpoint: str,
        request_type: str,
        response_type: str,
        request: str,
        *,
        timeout_ms: int = 3000,
    ) -> dict:
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/{endpoint}",
            "--reqtype",
            request_type,
            "--reptype",
            response_type,
            "--timeout",
            str(int(timeout_ms)),
            "--req",
            request,
        ]
        description = {
            "command": shlex.join(command),
            "endpoint": endpoint,
            "request": request,
            "dry_run": self.dry_run,
        }
        if self.dry_run:
            return description
        if not self.available:
            raise RuntimeError(
                "Gazebo CLI 'gz' is not installed or is not on PATH"
            )
        completed = self.runner(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=max(5.0, timeout_ms / 1000.0 + 2.0),
        )
        description.update(
            {
                "returncode": completed.returncode,
                "stdout": completed.stdout.strip(),
                "stderr": completed.stderr.strip(),
            }
        )
        if (
            completed.returncode != 0
            or "true" not in completed.stdout.lower()
        ):
            detail = completed.stderr.strip() or completed.stdout.strip()
            raise RuntimeError(f"Gazebo service rejected request: {detail}")
        return description

    def set_pose(
        self,
        name: str,
        *,
        x: float,
        y: float,
        z: float,
        yaw_deg: float,
    ) -> dict:
        name = _validate_name(name)
        yaw = math.radians(float(yaw_deg))
        request = (
            f'name: "{name}", '
            f"position: {{x: {float(x):.9f}, y: {float(y):.9f}, "
            f"z: {float(z):.9f}}}, "
            "orientation: {"
            f"z: {math.sin(yaw * 0.5):.9f}, "
            f"w: {math.cos(yaw * 0.5):.9f}"
            "}"
        )
        return self._service(
            "set_pose",
            "gz.msgs.Pose",
            "gz.msgs.Boolean",
            request,
        )

    def spawn_obstacle(
        self,
        name: str,
        preset: str,
        *,
        x: float,
        y: float,
        z: float | None = None,
        yaw_deg: float = 0.0,
        collision_enabled: bool = True,
    ) -> dict:
        name = _validate_name(name)
        if preset not in OBSTACLE_PRESETS:
            raise ValueError(f"unknown obstacle preset: {preset}")
        if z is None:
            z = OBSTACLE_PRESETS[preset].default_z
        yaw = math.radians(float(yaw_deg))
        sdf = obstacle_sdf(
            name,
            preset,
            collision_enabled=collision_enabled,
        )
        request = (
            f'name: "{name}", allow_renaming: false, '
            f'sdf: "{_escape_proto_string(sdf)}", '
            "pose: {"
            f"position: {{x: {float(x):.9f}, y: {float(y):.9f}, "
            f"z: {float(z):.9f}}}, "
            "orientation: {"
            f"z: {math.sin(yaw * 0.5):.9f}, "
            f"w: {math.cos(yaw * 0.5):.9f}"
            "}"
            "}"
        )
        return self._service(
            "create",
            "gz.msgs.EntityFactory",
            "gz.msgs.Boolean",
            request,
            timeout_ms=5000,
        )

    def remove(self, name: str) -> dict:
        name = _validate_name(name)
        return self._service(
            "remove",
            "gz.msgs.Entity",
            "gz.msgs.Boolean",
            f'name: "{name}", type: MODEL',
        )

    def pause(self, paused: bool) -> dict:
        return self._service(
            "control",
            "gz.msgs.WorldControl",
            "gz.msgs.Boolean",
            f"pause: {'true' if paused else 'false'}",
        )

    def reset_world(self) -> dict:
        return self._service(
            "control",
            "gz.msgs.WorldControl",
            "gz.msgs.Boolean",
            "reset: {all: true}",
        )


def _common_pose_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float)
    parser.add_argument("--yaw-deg", type=float, default=0.0)


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="Move Xycar and obstacles by explicit Gazebo coordinates"
    )
    parser.add_argument(
        "--world",
        default="kookmin_xycar_track",
    )
    parser.add_argument("--dry-run", action="store_true")
    subparsers = parser.add_subparsers(dest="action", required=True)

    pose = subparsers.add_parser("set-pose")
    pose.add_argument("name")
    _common_pose_arguments(pose)

    spawn = subparsers.add_parser("spawn")
    spawn.add_argument("name")
    spawn.add_argument("preset", choices=sorted(OBSTACLE_PRESETS))
    _common_pose_arguments(spawn)
    spawn.add_argument(
        "--visual-only",
        action="store_true",
        help="No collision; useful for LiDAR-invisible glass tests",
    )

    move = subparsers.add_parser("move")
    move.add_argument("name")
    _common_pose_arguments(move)

    remove = subparsers.add_parser("remove")
    remove.add_argument("name")

    subparsers.add_parser("pause")
    subparsers.add_parser("resume")
    subparsers.add_parser("reset-world")
    return parser.parse_args(args)


def main(args=None):
    options = parse_args(args)
    controller = GazeboController(
        options.world,
        dry_run=options.dry_run,
    )
    if options.action in {"set-pose", "move"}:
        result = controller.set_pose(
            options.name,
            x=options.x,
            y=options.y,
            z=options.z if options.z is not None else 0.05,
            yaw_deg=options.yaw_deg,
        )
    elif options.action == "spawn":
        result = controller.spawn_obstacle(
            options.name,
            options.preset,
            x=options.x,
            y=options.y,
            z=options.z,
            yaw_deg=options.yaw_deg,
            collision_enabled=not options.visual_only,
        )
    elif options.action == "remove":
        result = controller.remove(options.name)
    elif options.action == "pause":
        result = controller.pause(True)
    elif options.action == "resume":
        result = controller.pause(False)
    else:
        result = controller.reset_world()
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
