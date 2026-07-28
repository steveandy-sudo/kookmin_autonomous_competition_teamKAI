"""Quaternion order adapters between transforms3d and ROS messages."""

from __future__ import annotations

from transforms3d.euler import euler2quat, quat2euler


def ros_quaternion_from_euler(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """Return ROS (x, y, z, w) from transforms3d's (w, x, y, z)."""
    w, x, y, z = euler2quat(roll, pitch, yaw)
    return float(x), float(y), float(z), float(w)


def euler_from_ros_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float]:
    """Convert a ROS (x, y, z, w) quaternion to roll, pitch, yaw."""
    roll, pitch, yaw = quat2euler((w, x, y, z))
    return float(roll), float(pitch), float(yaw)
