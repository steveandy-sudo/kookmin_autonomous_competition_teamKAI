from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Sequence

import numpy as np


Point2 = tuple[float, float]


@dataclass(frozen=True)
class ConePlannerConfig:
    max_range_m: float = 1.6
    min_range_m: float = 0.18
    scan_angle_offset_deg: float = 0.0
    scan_front_min_deg: float = -70.0
    scan_front_max_deg: float = 70.0
    dbscan_eps_m: float = 0.15
    dbscan_min_samples: int = 3
    sparse_dbscan_min_samples: int = 2
    sparse_cluster_min_range_m: float = 0.8
    sparse_cluster_min_intensity: float = 80.0
    sparse_cluster_match_distance_m: float = 0.18
    sparse_cluster_required_frames: int = 2
    sparse_cluster_history_frames: int = 3
    max_cone_diameter_m: float = 0.30
    angle_bin_deg: float = 12.0
    group_grow_distance_m: float = 0.50
    seed_min_angle_deg: float = 20.0
    seed_max_angle_deg: float = 100.0
    seed_max_range_m: float = 0.90
    single_boundary_min_cones: int = 2
    single_boundary_min_span_m: float = 0.20
    single_boundary_switch_frames: int = 3
    single_boundary_confidence_cap: float = 0.60
    nearest_gate_confidence_cap: float = 0.40
    single_boundary_max_speed: float = 9.5
    lidar_to_rear_axle_m: float = 0.42
    wheelbase_m: float = 0.33
    lookahead_min_m: float = 0.70
    lookahead_max_m: float = 1.45
    lookahead_scale: float = 0.12
    far_preview_distance_m: float = 0.75
    far_preview_weight: float = 0.65
    steering_gain: float = 1.18
    max_wheel_angle_deg: float = 26.0
    cone_speed: float = 17.0
    cone_min_drive_speed: float = 9.5
    cone_speed_steer_exponent: float = 1.0
    cone_speed_confidence_floor_ratio: float = 0.35
    cone_straight_boost_speed: float = 17.0
    cone_straight_boost_min_confidence: float = 0.75
    cone_straight_boost_min_path_distance_m: float = 1.35
    cone_straight_boost_full_angle_deg: float = 1.0
    cone_straight_boost_max_angle_deg: float = 3.0
    cone_speed_preview_near_m: float = 0.80
    cone_speed_preview_far_m: float = 1.45
    cone_speed_preview_samples: int = 4
    min_confidence: float = 0.30
    path_sample_count: int = 100
    path_hold_frames: int = 5
    pair_max_forward_delta_m: float = 0.30
    min_corridor_width_m: float = 0.68
    max_corridor_width_m: float = 0.98
    expected_corridor_width_m: float = 0.85
    min_path_midpoints: int = 2
    min_path_span_m: float = 0.15
    max_path_target_jump_m: float = 0.45
    inferred_max_path_target_jump_m: float = 0.25
    steering_max_delta_deg: float = 14.0
    inferred_steering_max_delta_deg: float = 10.0
    fallback_pair_min_lateral_separation_m: float = 0.40
    fallback_pair_max_center_offset_m: float = 0.65
    blind_recovery_frames: int = 10
    blind_recovery_min_clusters: int = 2
    blind_recovery_steer_decay: float = 0.92


@dataclass(frozen=True)
class ConeCommand:
    wheel_angle_deg: float
    speed_command: float
    confidence: float
    clusters: tuple[Point2, ...]
    path: tuple[Point2, ...]
    source: str
    held: bool = False

    @property
    def valid(self) -> bool:
        return self.confidence > 0.20 and bool(self.path)


class HwjConePlanner:
    """ROS-free adaptation of teamkai/hwj's LiDAR cone corridor planner."""

    def __init__(self, config: ConePlannerConfig | None = None) -> None:
        self.config = config or ConePlannerConfig()
        self.cluster_history = deque(
            maxlen=max(1, self.config.sparse_cluster_history_frames)
        )
        self.previous_path: list[Point2] | None = None
        self.path_miss_count = 0
        self.path_is_held = False
        self.midpoint_source = "none"
        self.active_inferred_boundary: str | None = None
        self.pending_inferred_boundary: str | None = None
        self.pending_inferred_frames = 0
        self.stabilized_wheel_angle: float | None = None
        self.last_valid_wheel_angle = 0.0
        self.blind_recovery_count = 0
        self.had_valid_path = False

    def process_scan(
        self,
        ranges: Sequence[float],
        intensities: Sequence[float],
        *,
        angle_min: float,
        angle_increment: float,
    ) -> ConeCommand:
        points = self.scan_to_points(
            ranges,
            intensities,
            angle_min=angle_min,
            angle_increment=angle_increment,
        )
        clusters = self.filter_front_clusters_by_angle(
            self.cluster_cones(points)
        )
        left, right = self.form_cone_groups(clusters)
        midpoints = self.calculate_midpoints(left, right)
        switch_pending = (
            self.pending_inferred_boundary is not None
            and self.pending_inferred_frames > 0
        )
        if not midpoints and not switch_pending:
            fallback = self.nearest_gate_midpoint(clusters)
            if fallback is not None:
                self.midpoint_source = "nearest_gate"
                midpoints = [fallback]
        path = self.interpolate_path(midpoints)
        if not path:
            return self.blind_recovery(clusters)

        raw_angle = self.pure_pursuit(path)
        wheel_angle = self.stabilize_steering(raw_angle)
        confidence = self.path_confidence(len(midpoints))
        speed = self.compute_speed(wheel_angle, confidence, path)
        self.last_valid_wheel_angle = wheel_angle
        self.had_valid_path = True
        if not self.path_is_held:
            self.blind_recovery_count = 0
        return ConeCommand(
            wheel_angle,
            speed,
            confidence,
            tuple(clusters),
            tuple(path),
            self.midpoint_source,
            self.path_is_held,
        )

    def scan_to_points(
        self,
        ranges: Sequence[float],
        intensities: Sequence[float],
        *,
        angle_min: float,
        angle_increment: float,
    ) -> np.ndarray:
        cfg = self.config
        values = np.asarray(ranges, dtype=np.float32)
        if values.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        angles = (
            float(angle_min)
            + np.arange(values.size, dtype=np.float32)
            * float(angle_increment)
            + math.radians(cfg.scan_angle_offset_deg)
        )
        wrapped = (angles + math.pi) % (2.0 * math.pi) - math.pi
        valid = (
            np.isfinite(values)
            & (values > cfg.min_range_m)
            & (values <= cfg.max_range_m)
            & (wrapped >= math.radians(cfg.scan_front_min_deg))
            & (wrapped <= math.radians(cfg.scan_front_max_deg))
        )
        if not np.any(valid):
            return np.empty((0, 3), dtype=np.float32)
        intensity = np.asarray(intensities, dtype=np.float32)
        if intensity.size != values.size:
            intensity = np.zeros_like(values)
        x = values[valid] * np.cos(angles[valid])
        y = values[valid] * np.sin(angles[valid])
        return np.column_stack((x, y, intensity[valid])).astype(np.float32)

    def cluster_cones(self, points: np.ndarray) -> list[Point2]:
        cfg = self.config
        sparse_min = max(2, cfg.sparse_dbscan_min_samples)
        if points.shape[0] < sparse_min:
            self.cluster_history.append([])
            return []
        labels = self.simple_dbscan(points[:, :2], cfg.dbscan_eps_m, sparse_min)
        candidates: list[tuple[Point2, int, float]] = []
        for label in sorted(set(labels.tolist())):
            if label < 0:
                continue
            cluster = points[labels == label]
            xy = cluster[:, :2]
            diameter = float(np.linalg.norm(xy.max(axis=0) - xy.min(axis=0)))
            if diameter > cfg.max_cone_diameter_m:
                continue
            closest = xy[int(np.argmin(np.linalg.norm(xy, axis=1)))]
            candidates.append(
                (
                    (float(closest[0]), float(closest[1])),
                    int(cluster.shape[0]),
                    float(np.max(cluster[:, 2])),
                )
            )
        raw_centers = [candidate[0] for candidate in candidates]
        centers = []
        for center, samples, intensity in candidates:
            if samples >= max(2, cfg.dbscan_min_samples):
                centers.append(center)
            elif self.sparse_cluster_is_confirmed(center, intensity):
                centers.append(self.temporally_smoothed_center(center))
        self.cluster_history.append(raw_centers)
        return centers

    @staticmethod
    def simple_dbscan(
        points: np.ndarray,
        eps: float,
        min_samples: int,
    ) -> np.ndarray:
        count = points.shape[0]
        labels = np.full(count, -1, dtype=np.int32)
        visited = np.zeros(count, dtype=bool)
        distances = np.linalg.norm(
            points[:, None, :] - points[None, :, :],
            axis=2,
        )
        neighbors = [
            np.where(distances[index] <= eps)[0].tolist()
            for index in range(count)
        ]
        cluster_id = 0
        for index in range(count):
            if visited[index]:
                continue
            visited[index] = True
            if len(neighbors[index]) < min_samples:
                continue
            labels[index] = cluster_id
            queue = deque(neighbors[index])
            while queue:
                neighbor = queue.popleft()
                if not visited[neighbor]:
                    visited[neighbor] = True
                    if len(neighbors[neighbor]) >= min_samples:
                        queue.extend(neighbors[neighbor])
                if labels[neighbor] < 0:
                    labels[neighbor] = cluster_id
            cluster_id += 1
        return labels

    def sparse_cluster_is_confirmed(
        self,
        center: Point2,
        maximum_intensity: float,
    ) -> bool:
        cfg = self.config
        if math.hypot(*center) < cfg.sparse_cluster_min_range_m:
            return False
        if maximum_intensity < cfg.sparse_cluster_min_intensity:
            return False
        support = 0
        for previous in self.cluster_history:
            if any(
                math.hypot(center[0] - point[0], center[1] - point[1])
                <= cfg.sparse_cluster_match_distance_m
                for point in previous
            ):
                support += 1
        return support + 1 >= max(1, cfg.sparse_cluster_required_frames)

    def temporally_smoothed_center(self, center: Point2) -> Point2:
        matches = [center]
        for previous in self.cluster_history:
            nearby = [
                point
                for point in previous
                if math.hypot(center[0] - point[0], center[1] - point[1])
                <= self.config.sparse_cluster_match_distance_m
            ]
            if nearby:
                matches.append(
                    min(
                        nearby,
                        key=lambda point: math.hypot(
                            center[0] - point[0],
                            center[1] - point[1],
                        ),
                    )
                )
        values = np.asarray(matches, dtype=np.float32)
        return float(values[:, 0].mean()), float(values[:, 1].mean())

    def filter_front_clusters_by_angle(
        self,
        centers: Sequence[Point2],
    ) -> list[Point2]:
        used_bins: set[int] = set()
        filtered = []
        for x, y in sorted(centers, key=lambda point: math.hypot(*point)):
            bin_index = int(
                math.degrees(math.atan2(y, x)) // self.config.angle_bin_deg
            )
            if bin_index not in used_bins:
                filtered.append((x, y))
                used_bins.add(bin_index)
        return filtered

    def form_cone_groups(
        self,
        centers: Sequence[Point2],
    ) -> tuple[list[Point2], list[Point2]]:
        cfg = self.config
        left_seed = self.find_seed(
            centers,
            cfg.seed_min_angle_deg,
            cfg.seed_max_angle_deg,
        )
        right_seed = self.find_seed(
            centers,
            360.0 - cfg.seed_max_angle_deg,
            360.0 - cfg.seed_min_angle_deg,
        )
        if left_seed is not None and right_seed is not None:
            return self.grow_groups_competitively(
                left_seed,
                right_seed,
                centers,
            )
        used: set[Point2] = set()
        left = self.grow_group(left_seed, centers, used) if left_seed else []
        right = self.grow_group(right_seed, centers, used) if right_seed else []
        return left, right

    def find_seed(
        self,
        centers: Sequence[Point2],
        start_deg: float,
        end_deg: float,
    ) -> Point2 | None:
        candidates = []
        for point in centers:
            distance = math.hypot(*point)
            angle = math.degrees(math.atan2(point[1], point[0])) % 360.0
            if (
                start_deg <= angle <= end_deg
                and distance <= self.config.seed_max_range_m
            ):
                candidates.append((distance, point))
        return min(candidates)[1] if candidates else None

    def grow_groups_competitively(
        self,
        left_seed: Point2,
        right_seed: Point2,
        centers: Sequence[Point2],
    ) -> tuple[list[Point2], list[Point2]]:
        groups = [[left_seed], [right_seed]]
        used = {left_seed, right_seed}
        while True:
            best = None
            for point in centers:
                if point in used:
                    continue
                for side, group in enumerate(groups):
                    distance = min(
                        math.hypot(point[0] - base[0], point[1] - base[1])
                        for base in group
                    )
                    candidate = (distance, side, point)
                    if (
                        distance <= self.config.group_grow_distance_m
                        and (best is None or candidate < best)
                    ):
                        best = candidate
            if best is None:
                break
            _, side, point = best
            groups[side].append(point)
            used.add(point)
        return (
            sorted(groups[0], key=lambda point: point[0]),
            sorted(groups[1], key=lambda point: point[0]),
        )

    def grow_group(
        self,
        seed: Point2,
        centers: Sequence[Point2],
        used: set[Point2],
    ) -> list[Point2]:
        group = []
        queue = deque([seed])
        used.add(seed)
        while queue:
            base = queue.popleft()
            group.append(base)
            for point in centers:
                if point in used:
                    continue
                if (
                    math.hypot(point[0] - base[0], point[1] - base[1])
                    <= self.config.group_grow_distance_m
                ):
                    used.add(point)
                    queue.append(point)
        return sorted(group, key=lambda point: point[0])

    def calculate_midpoints(
        self,
        left: Sequence[Point2],
        right: Sequence[Point2],
    ) -> list[Point2]:
        self.midpoint_source = "none"
        if not left and not right:
            return []
        if not left or not right:
            return self.infer_single_boundary(left, right)
        cfg = self.config
        candidates = []
        for left_index, left_point in enumerate(left):
            for right_index, right_point in enumerate(right):
                forward_delta = abs(left_point[0] - right_point[0])
                width = math.hypot(
                    left_point[0] - right_point[0],
                    left_point[1] - right_point[1],
                )
                if (
                    forward_delta <= cfg.pair_max_forward_delta_m
                    and cfg.min_corridor_width_m
                    <= width
                    <= cfg.max_corridor_width_m
                ):
                    cost = (
                        2.0 * forward_delta
                        + abs(width - cfg.expected_corridor_width_m)
                    )
                    candidates.append((cost, left_index, right_index))
        used_left: set[int] = set()
        used_right: set[int] = set()
        midpoints = []
        for _, left_index, right_index in sorted(candidates):
            if left_index in used_left or right_index in used_right:
                continue
            left_point = left[left_index]
            right_point = right[right_index]
            midpoints.append(
                (
                    0.5 * (left_point[0] + right_point[0]),
                    0.5 * (left_point[1] + right_point[1]),
                )
            )
            used_left.add(left_index)
            used_right.add(right_index)
        midpoints.sort(key=lambda point: point[0])
        if len(midpoints) >= max(2, cfg.min_path_midpoints):
            self.midpoint_source = "paired"
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return midpoints
        inferred = self.infer_richer_boundary(left, right)
        if inferred:
            return inferred
        if midpoints:
            self.midpoint_source = "paired_sparse"
            return midpoints
        return self.infer_single_boundary(left, right)

    def infer_richer_boundary(
        self,
        left: Sequence[Point2],
        right: Sequence[Point2],
    ) -> list[Point2]:
        candidates: dict[str, list[Point2]] = {}
        left_segment = self.continuous_segment(left)
        right_segment = self.continuous_segment(right)
        if self.boundary_sufficient(left_segment):
            candidates["left"] = self.offset_boundary(left_segment, True)
        if self.boundary_sufficient(right_segment):
            candidates["right"] = self.offset_boundary(right_segment, False)
        if not candidates:
            return []
        desired = max(
            candidates,
            key=lambda side: (
                len(candidates[side]),
                -float(np.mean(np.abs([point[1] for point in candidates[side]]))),
            ),
        )
        selected = self.select_inferred_boundary(desired, candidates)
        if selected is None:
            return []
        self.midpoint_source = selected + "_offset"
        return candidates[selected]

    def infer_single_boundary(
        self,
        left: Sequence[Point2],
        right: Sequence[Point2],
    ) -> list[Point2]:
        left_segment = self.continuous_segment(left)
        right_segment = self.continuous_segment(right)
        if self.boundary_sufficient(left_segment) and not right:
            selected = self.select_inferred_boundary("left", {"left": []})
            if selected == "left":
                self.midpoint_source = "left_offset"
                return self.offset_boundary(left_segment, True)
        if self.boundary_sufficient(right_segment) and not left:
            selected = self.select_inferred_boundary("right", {"right": []})
            if selected == "right":
                self.midpoint_source = "right_offset"
                return self.offset_boundary(right_segment, False)
        return []

    def select_inferred_boundary(
        self,
        desired: str,
        candidates: dict[str, list[Point2]],
    ) -> str | None:
        active = self.active_inferred_boundary
        if active is None or desired == active:
            self.active_inferred_boundary = desired
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return desired
        if self.pending_inferred_boundary == desired:
            self.pending_inferred_frames += 1
        else:
            self.pending_inferred_boundary = desired
            self.pending_inferred_frames = 1
        if (
            self.pending_inferred_frames
            >= self.config.single_boundary_switch_frames
        ):
            self.active_inferred_boundary = desired
            self.pending_inferred_boundary = None
            self.pending_inferred_frames = 0
            return desired
        return active if active in candidates else None

    def continuous_segment(
        self,
        boundary: Sequence[Point2],
    ) -> list[Point2]:
        if not boundary:
            return []
        ordered = sorted(boundary, key=lambda point: point[0])
        segments: list[list[Point2]] = [[ordered[0]]]
        for point in ordered[1:]:
            previous = segments[-1][-1]
            if (
                math.hypot(point[0] - previous[0], point[1] - previous[1])
                <= self.config.group_grow_distance_m
            ):
                segments[-1].append(point)
            else:
                segments.append([point])
        return max(segments, key=len)

    def boundary_sufficient(self, boundary: Sequence[Point2]) -> bool:
        if len(boundary) < max(2, self.config.single_boundary_min_cones):
            return False
        span = max(point[0] for point in boundary) - min(
            point[0] for point in boundary
        )
        return span >= self.config.single_boundary_min_span_m

    def offset_boundary(
        self,
        boundary: Sequence[Point2],
        is_left: bool,
    ) -> list[Point2]:
        points = sorted(boundary, key=lambda point: point[0])
        half_width = 0.5 * self.config.expected_corridor_width_m
        centerline = []
        for index, (x, y) in enumerate(points):
            before = points[max(0, index - 1)]
            after = points[min(len(points) - 1, index + 1)]
            dx = after[0] - before[0]
            dy = after[1] - before[1]
            slope = 0.0 if abs(dx) < 1.0e-6 else dy / abs(dx)
            norm = math.sqrt(1.0 + slope * slope)
            direction = 1.0 if is_left else -1.0
            centerline.append(
                (
                    x + direction * half_width * slope / norm,
                    y - direction * half_width / norm,
                )
            )
        return sorted(centerline, key=lambda point: point[0])

    def nearest_gate_midpoint(
        self,
        clusters: Sequence[Point2],
    ) -> Point2 | None:
        cfg = self.config
        candidates = []
        for index, first in enumerate(clusters):
            for second in clusters[index + 1 :]:
                forward_delta = abs(first[0] - second[0])
                lateral = abs(first[1] - second[1])
                width = math.hypot(
                    first[0] - second[0],
                    first[1] - second[1],
                )
                midpoint = (
                    0.5 * (first[0] + second[0]),
                    0.5 * (first[1] + second[1]),
                )
                if (
                    forward_delta > cfg.pair_max_forward_delta_m
                    or lateral < cfg.fallback_pair_min_lateral_separation_m
                    or not cfg.min_corridor_width_m
                    <= width
                    <= cfg.max_corridor_width_m
                    or abs(midpoint[1])
                    > cfg.fallback_pair_max_center_offset_m
                ):
                    continue
                cost = (
                    max(math.hypot(*first), math.hypot(*second))
                    + 2.0 * forward_delta
                    + abs(width - cfg.expected_corridor_width_m)
                    + 0.25 * abs(midpoint[1])
                )
                candidates.append((cost, midpoint))
        return min(candidates)[1] if candidates else None

    def interpolate_path(self, midpoints: Sequence[Point2]) -> list[Point2]:
        cfg = self.config
        if len(midpoints) < max(1, cfg.min_path_midpoints):
            return self.hold_previous_path()
        points = sorted(
            [
                (x + cfg.lidar_to_rear_axle_m, y)
                for x, y in midpoints
            ],
            key=lambda point: point[0],
        )
        if len(points) == 1:
            return self.accept_new_path(points)
        xs = np.asarray([point[0] for point in points], dtype=np.float32)
        ys = np.asarray([point[1] for point in points], dtype=np.float32)
        unique_xs, unique_indices = np.unique(xs, return_index=True)
        unique_ys = ys[unique_indices]
        if (
            len(unique_xs) < cfg.min_path_midpoints
            or float(np.ptp(unique_xs)) < cfg.min_path_span_m
        ):
            return self.hold_previous_path()
        sample_x = np.linspace(
            float(unique_xs.min()),
            float(unique_xs.max()),
            max(2, cfg.path_sample_count),
        )
        path = [
            (float(x), float(y))
            for x, y in zip(
                sample_x,
                np.interp(sample_x, unique_xs, unique_ys),
            )
        ]
        return self.accept_new_path(path)

    def accept_new_path(self, path: list[Point2]) -> list[Point2]:
        if self.previous_path is not None:
            if len(self.previous_path) > 1 and len(path) == 1:
                return self.hold_previous_path()
            inferred = self.midpoint_source in {
                "left_offset",
                "right_offset",
                "nearest_gate",
                "paired_sparse",
            }
            max_jump = (
                self.config.inferred_max_path_target_jump_m
                if inferred
                else self.config.max_path_target_jump_m
            )
            if (
                abs(
                    self.path_target_lateral(path)
                    - self.path_target_lateral(self.previous_path)
                )
                > max_jump
            ):
                return self.hold_previous_path()
        self.previous_path = path
        self.path_miss_count = 0
        self.path_is_held = False
        return path

    def hold_previous_path(self) -> list[Point2]:
        self.path_miss_count += 1
        if (
            self.previous_path is not None
            and self.path_miss_count <= self.config.path_hold_frames
        ):
            self.path_is_held = True
            return self.previous_path
        self.previous_path = None
        self.path_is_held = False
        return []

    def path_target_lateral(self, path: Sequence[Point2]) -> float:
        for x, y in path:
            if x >= self.config.lookahead_min_m:
                return float(y)
        return float(path[-1][1]) if path else 0.0

    def pure_pursuit(self, path: Sequence[Point2]) -> float:
        lookahead = self.dynamic_lookahead(path)
        near = self.pure_pursuit_at_distance(path, lookahead)
        far = self.pure_pursuit_at_distance(
            path,
            min(
                self.config.lookahead_max_m,
                lookahead + self.config.far_preview_distance_m,
            ),
        )
        angle = self.config.steering_gain * (
            (1.0 - self.config.far_preview_weight) * near
            + self.config.far_preview_weight * far
        )
        return float(
            np.clip(
                angle,
                -self.config.max_wheel_angle_deg,
                self.config.max_wheel_angle_deg,
            )
        )

    def pure_pursuit_at_distance(
        self,
        path: Sequence[Point2],
        lookahead: float,
    ) -> float:
        distances = np.asarray(
            [math.hypot(x, y) for x, y in path],
            dtype=np.float32,
        )
        candidates = np.where(distances > lookahead)[0]
        index = int(candidates[0]) if candidates.size else len(path) - 1
        target_x, target_y = path[index]
        distance = max(1.0e-6, math.hypot(target_x, target_y))
        alpha = math.atan2(target_y, target_x)
        wheel_angle = math.atan2(
            2.0 * self.config.wheelbase_m * math.sin(alpha),
            distance,
        )
        return -math.degrees(wheel_angle)

    def dynamic_lookahead(self, path: Sequence[Point2]) -> float:
        length = sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(path[:-1], path[1:])
        )
        return float(
            np.clip(
                self.config.lookahead_scale * length,
                self.config.lookahead_min_m,
                self.config.lookahead_max_m,
            )
        )

    def stabilize_steering(self, wheel_angle: float) -> float:
        inferred = self.midpoint_source in {
            "left_offset",
            "right_offset",
            "nearest_gate",
            "paired_sparse",
        }
        maximum_delta = (
            self.config.inferred_steering_max_delta_deg
            if inferred
            else self.config.steering_max_delta_deg
        )
        if self.stabilized_wheel_angle is not None:
            wheel_angle = float(
                np.clip(
                    wheel_angle,
                    self.stabilized_wheel_angle - maximum_delta,
                    self.stabilized_wheel_angle + maximum_delta,
                )
            )
        self.stabilized_wheel_angle = wheel_angle
        return wheel_angle

    def path_confidence(self, midpoint_count: int) -> float:
        cfg = self.config
        if self.path_is_held:
            remaining = max(
                0.0,
                1.0
                - float(self.path_miss_count - 1)
                / max(1, cfg.path_hold_frames),
            )
            return max(0.21, cfg.min_confidence * remaining)
        if self.midpoint_source == "paired":
            return min(1.0, 0.5 + 0.2 * max(0, midpoint_count - 1))
        if self.midpoint_source in {"left_offset", "right_offset"}:
            return min(
                cfg.single_boundary_confidence_cap,
                0.38 + 0.06 * max(0, midpoint_count - 1),
            )
        if self.midpoint_source == "nearest_gate":
            return min(
                cfg.nearest_gate_confidence_cap,
                0.30 + 0.04 * max(0, midpoint_count - 1),
            )
        return min(0.40, 0.30 + 0.05 * max(0, midpoint_count - 1))

    def preview_steering_demand(
        self,
        path: Sequence[Point2],
    ) -> tuple[float, float]:
        distances = np.asarray(
            [math.hypot(x, y) for x, y in path],
            dtype=np.float32,
        )
        available = float(np.max(distances)) if distances.size else 0.0
        near = self.config.cone_speed_preview_near_m
        far = min(self.config.cone_speed_preview_far_m, available)
        if far < near:
            return 0.0, available
        samples = np.linspace(
            near,
            far,
            max(2, self.config.cone_speed_preview_samples),
        )
        demand = max(
            abs(self.pure_pursuit_at_distance(path, float(distance)))
            for distance in samples
        )
        return float(demand), available

    def compute_speed(
        self,
        wheel_angle: float,
        confidence: float,
        path: Sequence[Point2],
    ) -> float:
        cfg = self.config
        minimum = cfg.cone_min_drive_speed
        base = max(minimum, cfg.cone_speed)
        preview_angle, available = self.preview_steering_demand(path)
        demand = max(abs(wheel_angle), abs(preview_angle))
        steer_ratio = min(1.0, demand / max(1.0, cfg.max_wheel_angle_deg))
        speed = minimum + (base - minimum) * (
            1.0 - steer_ratio ** max(0.1, cfg.cone_speed_steer_exponent)
        )
        confidence_ratio = float(
            np.clip(
                (confidence - cfg.min_confidence)
                / max(1.0e-6, 1.0 - cfg.min_confidence),
                0.0,
                1.0,
            )
        )
        floor = cfg.cone_speed_confidence_floor_ratio
        confidence_speed = minimum + (base - minimum) * (
            floor + (1.0 - floor) * confidence_ratio
        )
        speed = min(speed, confidence_speed)
        if self.path_is_held:
            hold_ratio = min(
                1.0,
                self.path_miss_count / max(1, cfg.path_hold_frames),
            )
            speed = minimum + (speed - minimum) * (
                1.0 - 0.55 * hold_ratio
            )
        if self.midpoint_source in {"left_offset", "right_offset"}:
            speed = min(speed, cfg.single_boundary_max_speed)
        elif self.midpoint_source == "nearest_gate":
            speed = minimum
        if (
            not self.path_is_held
            and self.midpoint_source == "paired"
            and confidence >= cfg.cone_straight_boost_min_confidence
            and available >= cfg.cone_straight_boost_min_path_distance_m
        ):
            full = cfg.cone_straight_boost_full_angle_deg
            maximum = max(full + 1.0e-6, cfg.cone_straight_boost_max_angle_deg)
            boost = float(np.clip((maximum - demand) / (maximum - full), 0, 1))
            speed += boost * (max(base, cfg.cone_straight_boost_speed) - base)
        return max(minimum, float(speed))

    def blind_recovery(self, clusters: Sequence[Point2]) -> ConeCommand:
        cfg = self.config
        if (
            not self.had_valid_path
            or len(clusters) < cfg.blind_recovery_min_clusters
            or self.blind_recovery_count >= cfg.blind_recovery_frames
        ):
            self.stabilized_wheel_angle = None
            return ConeCommand(
                0.0,
                0.0,
                0.0,
                tuple(clusters),
                (),
                "none",
            )
        self.blind_recovery_count += 1
        angle = self.last_valid_wheel_angle * (
            cfg.blind_recovery_steer_decay ** self.blind_recovery_count
        )
        return ConeCommand(
            angle,
            cfg.cone_min_drive_speed,
            max(0.21, cfg.min_confidence * 0.75),
            tuple(clusters),
            tuple(self.previous_path or ()),
            "blind_recovery",
            True,
        )


def wheel_angle_to_xycar_command(wheel_angle_deg: float) -> float:
    """Map hwj's physical wheel angle to the measured Xycar motor command."""
    table = ((0.0, 0.0), (4.0, 10.0), (10.0, 20.0), (16.0, 30.0), (26.0, 42.0))
    sign = -1.0 if wheel_angle_deg < 0.0 else 1.0
    target = abs(float(wheel_angle_deg))
    if target <= table[0][0]:
        return 0.0
    for (low_angle, low_command), (high_angle, high_command) in zip(
        table[:-1],
        table[1:],
    ):
        if target <= high_angle:
            ratio = (target - low_angle) / (high_angle - low_angle)
            return sign * (
                low_command + ratio * (high_command - low_command)
            )
    return sign * table[-1][1]
