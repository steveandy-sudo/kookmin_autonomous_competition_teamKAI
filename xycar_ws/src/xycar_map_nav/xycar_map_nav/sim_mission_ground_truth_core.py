"""Geometry helpers for deterministic Gazebo mission detections."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EgoPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MissionTarget:
    name: str
    class_name: str
    x: float
    y: float
    width_m: float
    height_m: float
    elevated: bool = False


@dataclass(frozen=True)
class DetectionBox:
    name: str
    class_name: str
    distance_m: float
    bearing_rad: float
    xmin: int
    ymin: int
    xmax: int
    ymax: int


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a normalized or non-normalized quaternion."""

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def world_to_vehicle(
    ego: EgoPose,
    target_x: float,
    target_y: float,
) -> tuple[float, float]:
    """Transform a world point into forward / left vehicle coordinates."""

    dx = float(target_x) - ego.x
    dy = float(target_y) - ego.y
    cosine = math.cos(ego.yaw)
    sine = math.sin(ego.yaw)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def project_target(
    ego: EgoPose,
    target: MissionTarget,
    *,
    image_width: int,
    image_height: int,
    horizontal_fov_rad: float,
    maximum_range_m: float,
) -> DetectionBox | None:
    """Project a mission target to the camera's horizontal image geometry.

    Only the horizontal extent is used by the YOLO-LiDAR matcher.  The
    vertical extent is a stable visualization aid rather than a calibrated
    pinhole projection.
    """

    forward, left = world_to_vehicle(ego, target.x, target.y)
    distance = math.hypot(forward, left)
    if forward <= 0.05 or distance > maximum_range_m:
        return None
    if image_width <= 0 or image_height <= 0 or horizontal_fov_rad <= 0.0:
        return None

    bearing = math.atan2(left, forward)
    half_angle = math.atan2(0.5 * max(target.width_m, 0.01), forward)
    if abs(bearing) - half_angle > 0.5 * horizontal_fov_rad:
        return None

    center_x = image_width * (0.5 - bearing / horizontal_fov_rad)
    half_width_px = max(
        2.0,
        image_width * half_angle / horizontal_fov_rad,
    )
    xmin = max(0, int(math.floor(center_x - half_width_px)))
    xmax = min(image_width - 1, int(math.ceil(center_x + half_width_px)))
    if xmax <= xmin:
        return None

    if target.elevated:
        box_height = max(
            10,
            int(round(0.15 * image_height / max(distance, 0.5))),
        )
        center_y = int(round(0.25 * image_height))
        ymin = max(0, center_y - box_height // 2)
        ymax = min(image_height - 1, center_y + box_height // 2)
    else:
        angular_height = math.atan2(max(target.height_m, 0.03), forward)
        box_height = max(
            7,
            int(round(image_height * angular_height / horizontal_fov_rad)),
        )
        ymax = int(round(0.93 * image_height))
        ymin = max(0, ymax - box_height)

    return DetectionBox(
        name=target.name,
        class_name=target.class_name,
        distance_m=distance,
        bearing_rad=bearing,
        xmin=xmin,
        ymin=ymin,
        xmax=xmax,
        ymax=ymax,
    )


def shuttle_local_speed(
    *,
    world_x: float,
    minimum_x: float,
    maximum_x: float,
    current_local_speed: float,
    speed_mps: float,
) -> float:
    """Reverse a yaw-pi vehicle's local velocity at shuttle endpoints."""

    speed = abs(float(speed_mps))
    if float(world_x) <= float(minimum_x):
        return -speed
    if float(world_x) >= float(maximum_x):
        return speed
    return speed if current_local_speed >= 0.0 else -speed
