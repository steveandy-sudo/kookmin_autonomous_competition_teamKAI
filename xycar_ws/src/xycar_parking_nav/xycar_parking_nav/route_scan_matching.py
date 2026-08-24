"""Route-constrained 2D LiDAR matching against an occupancy map."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np
import yaml


Point = tuple[float, float]


@dataclass(frozen=True)
class LikelihoodField:
    distance_m: np.ndarray
    known: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    @property
    def height(self) -> int:
        return int(self.distance_m.shape[0])

    @property
    def width(self) -> int:
        return int(self.distance_m.shape[1])


@dataclass(frozen=True)
class MatchConfig:
    route_sample_spacing_m: float = 0.40
    lateral_search_m: float = 0.30
    lateral_step_m: float = 0.15
    yaw_search_rad: float = math.radians(20.0)
    yaw_step_rad: float = math.radians(5.0)
    refinement_xy_m: float = 0.12
    refinement_xy_step_m: float = 0.04
    refinement_yaw_rad: float = math.radians(4.0)
    refinement_yaw_step_rad: float = math.radians(2.0)
    inlier_distance_m: float = 0.15
    maximum_distance_m: float = 0.60
    ambiguity_separation_m: float = 2.0
    minimum_inlier_ratio: float = 0.45
    maximum_mean_distance_m: float = 0.22
    minimum_score_margin: float = 0.06
    batch_size: int = 128
    allow_reverse_heading: bool = False
    lateral_prior_weight: float = 0.02
    yaw_prior_weight: float = 0.04


@dataclass(frozen=True)
class RouteMatch:
    x: float
    y: float
    yaw: float
    route_s: float
    route_length: float
    score: float
    second_score: float
    score_margin: float
    inlier_ratio: float
    mean_distance_m: float
    accepted: bool


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def load_likelihood_field(map_yaml: str | Path) -> LikelihoodField:
    yaml_path = Path(map_yaml).expanduser().resolve()
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = Path(str(data["image"]))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"cannot read occupancy image: {image_path}")

    negate = bool(int(data.get("negate", 0)))
    normalized = image.astype(np.float32) / 255.0
    occupied_probability = normalized if negate else 1.0 - normalized
    occupied_threshold = float(data.get("occupied_thresh", 0.65))
    free_threshold = float(data.get("free_thresh", 0.25))
    occupied = occupied_probability >= occupied_threshold
    trinary_unknown = np.logical_and(
        str(data.get("mode", "trinary")).lower() == "trinary",
        image == 205,
    )
    known = np.logical_or(
        occupied,
        occupied_probability <= free_threshold,
    )
    known = np.logical_and(known, np.logical_not(trinary_unknown))
    distance_pixels = cv2.distanceTransform(
        np.logical_not(occupied).astype(np.uint8),
        cv2.DIST_L2,
        5,
    )
    origin = [float(value) for value in data["origin"]]
    return LikelihoodField(
        distance_m=distance_pixels * float(data["resolution"]),
        known=known,
        resolution=float(data["resolution"]),
        origin_x=origin[0],
        origin_y=origin[1],
        origin_yaw=origin[2],
    )


def scan_points_in_base(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float,
    laser_y: float,
    laser_yaw: float,
    maximum_points: int,
) -> np.ndarray:
    values = np.asarray(ranges, dtype=np.float32)
    if values.size == 0:
        return np.empty((0, 2), dtype=np.float32)
    angles = (
        float(angle_min)
        + np.arange(values.size, dtype=np.float32) * float(angle_increment)
    )
    valid = np.isfinite(values)
    valid &= values >= max(0.0, float(range_min))
    valid &= values <= float(range_max)
    values = values[valid]
    angles = angles[valid]
    if values.size == 0:
        return np.empty((0, 2), dtype=np.float32)

    limit = max(1, int(maximum_points))
    if values.size > limit:
        indices = np.linspace(0, values.size - 1, limit).astype(np.int32)
        values = values[indices]
        angles = angles[indices]

    laser_points = np.column_stack(
        (values * np.cos(angles), values * np.sin(angles))
    )
    cosine = math.cos(float(laser_yaw))
    sine = math.sin(float(laser_yaw))
    base_points = np.empty_like(laser_points)
    base_points[:, 0] = (
        float(laser_x)
        + cosine * laser_points[:, 0]
        - sine * laser_points[:, 1]
    )
    base_points[:, 1] = (
        float(laser_y)
        + sine * laser_points[:, 0]
        + cosine * laser_points[:, 1]
    )
    return base_points.astype(np.float32, copy=False)


def _route_samples(
    points: Sequence[Point],
    *,
    spacing_m: float,
    closed: bool,
) -> tuple[np.ndarray, np.ndarray, float]:
    route = np.asarray(points, dtype=np.float64)
    if route.ndim != 2 or route.shape[0] < 2 or route.shape[1] != 2:
        raise ValueError("route must contain at least two 2D points")
    if closed:
        segment_start = route
        segment_end = np.roll(route, -1, axis=0)
    else:
        segment_start = route[:-1]
        segment_end = route[1:]
    deltas = segment_end - segment_start
    lengths = np.linalg.norm(deltas, axis=1)
    valid = lengths > 1.0e-6
    segment_start = segment_start[valid]
    deltas = deltas[valid]
    lengths = lengths[valid]
    if not lengths.size:
        raise ValueError("route has no non-zero segments")

    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    route_length = float(cumulative[-1])
    count = max(
        2,
        int(math.ceil(route_length / max(0.05, float(spacing_m)))),
    )
    sample_s = np.linspace(
        0.0,
        route_length,
        count,
        endpoint=not closed,
        dtype=np.float64,
    )
    segment_index = np.searchsorted(
        cumulative[1:],
        sample_s,
        side="right",
    )
    segment_index = np.minimum(segment_index, len(lengths) - 1)
    local_s = sample_s - cumulative[segment_index]
    fraction = local_s / lengths[segment_index]
    sample_xy = (
        segment_start[segment_index]
        + deltas[segment_index] * fraction[:, None]
    )
    yaw = np.arctan2(
        deltas[segment_index, 1],
        deltas[segment_index, 0],
    )
    return (
        np.column_stack((sample_xy, yaw)),
        sample_s,
        route_length,
    )


def _candidate_grid(
    route_pose: np.ndarray,
    route_s: np.ndarray,
    config: MatchConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lateral = np.arange(
        -config.lateral_search_m,
        config.lateral_search_m + config.lateral_step_m * 0.5,
        max(0.01, config.lateral_step_m),
    )
    yaw_offsets = np.arange(
        -config.yaw_search_rad,
        config.yaw_search_rad + config.yaw_step_rad * 0.5,
        max(math.radians(0.5), config.yaw_step_rad),
    )
    if config.allow_reverse_heading:
        yaw_offsets = np.concatenate((yaw_offsets, yaw_offsets + math.pi))

    pose_count = route_pose.shape[0]
    combinations = lateral.size * yaw_offsets.size
    candidates = np.empty((pose_count * combinations, 3), dtype=np.float32)
    candidate_s = np.repeat(route_s, combinations).astype(np.float32)
    penalties = np.empty(pose_count * combinations, dtype=np.float32)
    cursor = 0
    for pose in route_pose:
        normal_x = -math.sin(float(pose[2]))
        normal_y = math.cos(float(pose[2]))
        for offset in lateral:
            for yaw_offset in yaw_offsets:
                candidates[cursor] = (
                    pose[0] + normal_x * offset,
                    pose[1] + normal_y * offset,
                    normalize_angle(float(pose[2] + yaw_offset)),
                )
                lateral_fraction = (
                    abs(float(offset)) / config.lateral_search_m
                    if config.lateral_search_m > 0.0
                    else 0.0
                )
                wrapped_yaw_offset = abs(normalize_angle(float(yaw_offset)))
                if config.allow_reverse_heading:
                    wrapped_yaw_offset = min(
                        wrapped_yaw_offset,
                        abs(
                            normalize_angle(
                                float(yaw_offset) - math.pi
                            )
                        ),
                    )
                yaw_fraction = (
                    wrapped_yaw_offset / config.yaw_search_rad
                    if config.yaw_search_rad > 0.0
                    else 0.0
                )
                penalties[cursor] = (
                    max(0.0, config.lateral_prior_weight)
                    * lateral_fraction
                    + max(0.0, config.yaw_prior_weight) * yaw_fraction
                )
                cursor += 1
    return candidates, candidate_s, penalties


def _evaluate_candidates(
    field: LikelihoodField,
    scan_points: np.ndarray,
    candidates: np.ndarray,
    *,
    inlier_distance_m: float,
    maximum_distance_m: float,
    batch_size: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    scores = np.empty(candidates.shape[0], dtype=np.float32)
    inlier_ratios = np.empty_like(scores)
    means = np.empty_like(scores)
    origin_cos = math.cos(field.origin_yaw)
    origin_sin = math.sin(field.origin_yaw)
    clip_distance = max(0.05, float(maximum_distance_m))

    for start in range(0, candidates.shape[0], max(1, int(batch_size))):
        batch = candidates[start : start + max(1, int(batch_size))]
        cosines = np.cos(batch[:, 2])[:, None]
        sines = np.sin(batch[:, 2])[:, None]
        world_x = (
            batch[:, 0, None]
            + cosines * scan_points[None, :, 0]
            - sines * scan_points[None, :, 1]
        )
        world_y = (
            batch[:, 1, None]
            + sines * scan_points[None, :, 0]
            + cosines * scan_points[None, :, 1]
        )
        delta_x = world_x - field.origin_x
        delta_y = world_y - field.origin_y
        local_x = origin_cos * delta_x + origin_sin * delta_y
        local_y = -origin_sin * delta_x + origin_cos * delta_y
        columns = np.floor(local_x / field.resolution).astype(np.int32)
        bottom_rows = np.floor(local_y / field.resolution).astype(np.int32)
        rows = field.height - 1 - bottom_rows
        inside = (
            (rows >= 0)
            & (rows < field.height)
            & (columns >= 0)
            & (columns < field.width)
        )
        safe_rows = np.clip(rows, 0, field.height - 1)
        safe_columns = np.clip(columns, 0, field.width - 1)
        known = field.known[safe_rows, safe_columns] & inside
        distances = field.distance_m[safe_rows, safe_columns]
        distances = np.where(
            known,
            np.minimum(distances, clip_distance),
            clip_distance,
        )
        batch_mean = np.mean(distances, axis=1)
        batch_inlier = np.mean(
            distances <= float(inlier_distance_m),
            axis=1,
        )
        batch_score = batch_inlier - 0.75 * batch_mean / clip_distance
        end = start + batch.shape[0]
        scores[start:end] = batch_score
        inlier_ratios[start:end] = batch_inlier
        means[start:end] = batch_mean
    return scores, inlier_ratios, means


def _cyclic_distance(a: np.ndarray, b: float, length: float) -> np.ndarray:
    direct = np.abs(a - b)
    if length <= 0.0:
        return direct
    return np.minimum(direct, length - direct)


def match_scan_to_route(
    field: LikelihoodField,
    route_points: Sequence[Point],
    scan_points: np.ndarray,
    *,
    closed: bool,
    config: MatchConfig,
) -> RouteMatch:
    if scan_points.ndim != 2 or scan_points.shape[0] < 3:
        raise ValueError("at least three scan points are required")
    route_pose, route_s, route_length = _route_samples(
        route_points,
        spacing_m=config.route_sample_spacing_m,
        closed=closed,
    )
    candidates, candidate_s, candidate_penalties = _candidate_grid(
        route_pose,
        route_s,
        config,
    )
    scores, inliers, means = _evaluate_candidates(
        field,
        scan_points,
        candidates,
        inlier_distance_m=config.inlier_distance_m,
        maximum_distance_m=config.maximum_distance_m,
        batch_size=config.batch_size,
    )
    scores -= candidate_penalties
    coarse_best = int(np.argmax(scores))
    distinct = (
        _cyclic_distance(
            candidate_s,
            float(candidate_s[coarse_best]),
            route_length if closed else 0.0,
        )
        >= config.ambiguity_separation_m
    )
    second_score = (
        float(np.max(scores[distinct]))
        if np.any(distinct)
        else -1.0
    )
    coarse_margin = float(scores[coarse_best]) - second_score

    base = candidates[coarse_best]
    along_values = np.arange(
        -config.refinement_xy_m,
        config.refinement_xy_m + config.refinement_xy_step_m * 0.5,
        max(0.01, config.refinement_xy_step_m),
    )
    lateral_values = along_values
    yaw_values = np.arange(
        -config.refinement_yaw_rad,
        config.refinement_yaw_rad + config.refinement_yaw_step_rad * 0.5,
        max(math.radians(0.5), config.refinement_yaw_step_rad),
    )
    refined = []
    refined_s = []
    cosine = math.cos(float(base[2]))
    sine = math.sin(float(base[2]))
    for along in along_values:
        for lateral in lateral_values:
            for yaw_offset in yaw_values:
                refined.append(
                    (
                        base[0] + cosine * along - sine * lateral,
                        base[1] + sine * along + cosine * lateral,
                        normalize_angle(float(base[2] + yaw_offset)),
                    )
                )
                s = float(candidate_s[coarse_best]) + float(along)
                if closed:
                    s %= route_length
                else:
                    s = min(route_length, max(0.0, s))
                refined_s.append(s)
    refined_array = np.asarray(refined, dtype=np.float32)
    refined_scores, refined_inliers, refined_means = _evaluate_candidates(
        field,
        scan_points,
        refined_array,
        inlier_distance_m=config.inlier_distance_m,
        maximum_distance_m=config.maximum_distance_m,
        batch_size=config.batch_size,
    )
    best = int(np.argmax(refined_scores))
    score = float(refined_scores[best])
    inlier_ratio = float(refined_inliers[best])
    mean_distance = float(refined_means[best])
    accepted = (
        inlier_ratio >= config.minimum_inlier_ratio
        and mean_distance <= config.maximum_mean_distance_m
        and coarse_margin >= config.minimum_score_margin
    )
    return RouteMatch(
        x=float(refined_array[best, 0]),
        y=float(refined_array[best, 1]),
        yaw=float(refined_array[best, 2]),
        route_s=float(refined_s[best]),
        route_length=route_length,
        score=score,
        second_score=second_score,
        score_margin=coarse_margin,
        inlier_ratio=inlier_ratio,
        mean_distance_m=mean_distance,
        accepted=accepted,
    )


def route_distance(first: RouteMatch, second: RouteMatch) -> float:
    length = min(first.route_length, second.route_length)
    direct = abs(first.route_s - second.route_s)
    if length <= 0.0:
        return direct
    return min(direct, length - direct)
