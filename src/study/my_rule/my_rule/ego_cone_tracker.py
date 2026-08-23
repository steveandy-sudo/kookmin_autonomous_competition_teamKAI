"""Short-lived cone landmark tracking in an odometry frame.

The cone planner consumes points in the LiDAR frame. Reusing those coordinates
after the vehicle has moved makes a static cone appear to move with the car.
This module stores confirmed cones in ``odom`` and projects them back into the
current sensor frame on every scan.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence, Tuple


Point2 = Tuple[float, float]


@dataclass(frozen=True)
class PlanarPose:
    """Planar vehicle pose expressed in the odometry frame."""

    x: float
    y: float
    yaw: float


@dataclass
class ConeLandmark:
    """Mutable cone track stored in odometry coordinates."""

    track_id: int
    world_x: float
    world_y: float
    last_seen_sec: float
    last_semantic_sec: float
    hits: int = 1


@dataclass(frozen=True)
class TrackedCone:
    """One active landmark projected into the current sensor frame."""

    track_id: int
    x: float
    y: float
    age_sec: float
    predicted: bool


@dataclass(frozen=True)
class TrackerSnapshot:
    """Output of one tracker update."""

    cones: Tuple[TrackedCone, ...]
    observed_count: int
    predicted_count: int
    pose_jump_reset: bool

    @property
    def points(self) -> List[Point2]:
        return [(cone.x, cone.y) for cone in self.cones]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def vehicle_to_world(
    point: Point2,
    pose: PlanarPose,
    *,
    origin_x_offset_m: float = 0.0,
) -> Point2:
    """Transform a point from a vehicle-aligned frame into ``odom``.

    ``origin_x_offset_m`` describes the point frame origin relative to the
    odometry vehicle origin. The cone stack uses LiDAR coordinates while its
    odometry pose is at the rear-axle controller origin, hence the usual
    positive ``lidar_to_rear_axle_m`` offset.
    """

    local_x = float(point[0]) + float(origin_x_offset_m)
    local_y = float(point[1])
    cosine = math.cos(float(pose.yaw))
    sine = math.sin(float(pose.yaw))
    return (
        float(pose.x) + cosine * local_x - sine * local_y,
        float(pose.y) + sine * local_x + cosine * local_y,
    )


def world_to_vehicle(
    point: Point2,
    pose: PlanarPose,
    *,
    origin_x_offset_m: float = 0.0,
) -> Point2:
    """Transform an odometry-frame point into a vehicle-aligned frame."""

    delta_x = float(point[0]) - float(pose.x)
    delta_y = float(point[1]) - float(pose.y)
    cosine = math.cos(float(pose.yaw))
    sine = math.sin(float(pose.yaw))
    return (
        cosine * delta_x + sine * delta_y - float(origin_x_offset_m),
        -sine * delta_x + cosine * delta_y,
    )


def reproject_points(
    points: Sequence[Point2],
    previous_pose: PlanarPose,
    current_pose: PlanarPose,
    *,
    origin_x_offset_m: float = 0.0,
) -> List[Point2]:
    """Move cached vehicle-frame points into the current vehicle frame."""

    return [
        world_to_vehicle(
            vehicle_to_world(
                point,
                previous_pose,
                origin_x_offset_m=origin_x_offset_m,
            ),
            current_pose,
            origin_x_offset_m=origin_x_offset_m,
        )
        for point in points
    ]


def merge_points(
    primary: Sequence[Point2],
    additions: Sequence[Point2],
    *,
    minimum_separation_m: float,
) -> List[Point2]:
    """Append non-duplicate points while preserving the primary measurements."""

    result = [(float(x), float(y)) for x, y in primary]
    separation = max(0.0, float(minimum_separation_m))
    for raw_x, raw_y in additions:
        point = (float(raw_x), float(raw_y))
        if any(
            math.hypot(point[0] - old[0], point[1] - old[1])
            <= separation
            for old in result
        ):
            continue
        result.append(point)
    return result


class EgoConeTracker:
    """Track semantic cone landmarks in ``odom`` for a bounded interval."""

    def __init__(
        self,
        *,
        ttl_sec: float = 0.30,
        lidar_association_distance_m: float = 0.22,
        semantic_association_distance_m: float = 0.25,
        lidar_smoothing_alpha: float = 0.30,
        semantic_smoothing_alpha: float = 0.40,
        sensor_x_offset_m: float = 0.42,
        minimum_forward_m: float = -0.20,
        maximum_forward_m: float = 2.20,
        maximum_abs_lateral_m: float = 1.40,
        maximum_pose_jump_m: float = 0.30,
        maximum_pose_jump_yaw_deg: float = 45.0,
    ) -> None:
        self.ttl_sec = max(0.0, float(ttl_sec))
        self.lidar_association_distance_m = max(
            0.0, float(lidar_association_distance_m)
        )
        self.semantic_association_distance_m = max(
            0.0, float(semantic_association_distance_m)
        )
        self.lidar_smoothing_alpha = self._bounded_alpha(
            lidar_smoothing_alpha
        )
        self.semantic_smoothing_alpha = self._bounded_alpha(
            semantic_smoothing_alpha
        )
        self.sensor_x_offset_m = float(sensor_x_offset_m)
        self.minimum_forward_m = float(minimum_forward_m)
        self.maximum_forward_m = float(maximum_forward_m)
        self.maximum_abs_lateral_m = max(
            0.0, float(maximum_abs_lateral_m)
        )
        self.maximum_pose_jump_m = max(0.0, float(maximum_pose_jump_m))
        self.maximum_pose_jump_yaw_rad = math.radians(
            max(0.0, float(maximum_pose_jump_yaw_deg))
        )
        self._tracks: List[ConeLandmark] = []
        self._next_track_id = 1
        self._last_pose: PlanarPose | None = None

    @staticmethod
    def _bounded_alpha(value: float) -> float:
        return min(1.0, max(0.0, float(value)))

    @property
    def track_count(self) -> int:
        return len(self._tracks)

    def reset(self) -> None:
        self._tracks.clear()
        self._last_pose = None

    def _pose_jump_detected(self, pose: PlanarPose) -> bool:
        if self._last_pose is None:
            return False
        translation = math.hypot(
            float(pose.x) - float(self._last_pose.x),
            float(pose.y) - float(self._last_pose.y),
        )
        yaw_delta = abs(normalize_angle(float(pose.yaw) - self._last_pose.yaw))
        return bool(
            (
                self.maximum_pose_jump_m > 0.0
                and translation > self.maximum_pose_jump_m
            )
            or (
                self.maximum_pose_jump_yaw_rad > 0.0
                and yaw_delta > self.maximum_pose_jump_yaw_rad
            )
        )

    def _prune(self, now_sec: float) -> None:
        self._tracks = [
            track
            for track in self._tracks
            if float(now_sec) - float(track.last_seen_sec)
            <= self.ttl_sec + 1.0e-9
        ]

    @staticmethod
    def _deduplicate(points: Iterable[Point2]) -> List[Point2]:
        values: List[Point2] = []
        for raw_x, raw_y in points:
            point = (float(raw_x), float(raw_y))
            if not all(math.isfinite(value) for value in point):
                continue
            if any(
                math.hypot(point[0] - old[0], point[1] - old[1]) <= 0.04
                for old in values
            ):
                continue
            values.append(point)
        return values

    @staticmethod
    def _associate(
        measurements: Sequence[Point2],
        tracks: Sequence[ConeLandmark],
        maximum_distance_m: float,
    ) -> Tuple[List[Tuple[int, int]], List[int]]:
        candidates: List[Tuple[float, int, int]] = []
        for measurement_index, measurement in enumerate(measurements):
            for track_index, track in enumerate(tracks):
                distance = math.hypot(
                    float(measurement[0]) - float(track.world_x),
                    float(measurement[1]) - float(track.world_y),
                )
                if distance <= maximum_distance_m:
                    candidates.append(
                        (distance, measurement_index, track_index)
                    )
        used_measurements = set()
        used_tracks = set()
        matches: List[Tuple[int, int]] = []
        for _distance, measurement_index, track_index in sorted(candidates):
            if (
                measurement_index in used_measurements
                or track_index in used_tracks
            ):
                continue
            used_measurements.add(measurement_index)
            used_tracks.add(track_index)
            matches.append((measurement_index, track_index))
        unmatched = [
            index
            for index in range(len(measurements))
            if index not in used_measurements
        ]
        return matches, unmatched

    @staticmethod
    def _update_track(
        track: ConeLandmark,
        measurement: Point2,
        *,
        now_sec: float,
        alpha: float,
        semantic: bool,
    ) -> None:
        track.world_x = (
            (1.0 - alpha) * float(track.world_x)
            + alpha * float(measurement[0])
        )
        track.world_y = (
            (1.0 - alpha) * float(track.world_y)
            + alpha * float(measurement[1])
        )
        track.last_seen_sec = float(now_sec)
        if semantic:
            track.last_semantic_sec = float(now_sec)
        track.hits += 1

    def update(
        self,
        *,
        pose: PlanarPose,
        semantic_points: Sequence[Point2],
        lidar_points: Sequence[Point2],
        now_sec: float,
    ) -> TrackerSnapshot:
        """Update landmarks and return their current sensor-frame positions.

        Semantic/fused measurements may create landmarks. Raw LiDAR can only
        refresh an existing landmark, preventing an arbitrary chair leg from
        becoming a persistent cone after camera confirmation disappears.
        """

        now = float(now_sec)
        pose_jump = self._pose_jump_detected(pose)
        if pose_jump:
            self._tracks.clear()
        self._last_pose = pose
        self._prune(now)

        lidar_world = [
            vehicle_to_world(
                point,
                pose,
                origin_x_offset_m=self.sensor_x_offset_m,
            )
            for point in self._deduplicate(lidar_points)
        ]
        semantic_world = [
            vehicle_to_world(
                point,
                pose,
                origin_x_offset_m=self.sensor_x_offset_m,
            )
            for point in self._deduplicate(semantic_points)
        ]
        observed_ids = set()

        lidar_matches, _ = self._associate(
            lidar_world,
            self._tracks,
            self.lidar_association_distance_m,
        )
        for measurement_index, track_index in lidar_matches:
            track = self._tracks[track_index]
            self._update_track(
                track,
                lidar_world[measurement_index],
                now_sec=now,
                alpha=self.lidar_smoothing_alpha,
                semantic=False,
            )
            observed_ids.add(track.track_id)

        semantic_matches, semantic_unmatched = self._associate(
            semantic_world,
            self._tracks,
            self.semantic_association_distance_m,
        )
        for measurement_index, track_index in semantic_matches:
            track = self._tracks[track_index]
            self._update_track(
                track,
                semantic_world[measurement_index],
                now_sec=now,
                alpha=self.semantic_smoothing_alpha,
                semantic=True,
            )
            observed_ids.add(track.track_id)

        for measurement_index in semantic_unmatched:
            measurement = semantic_world[measurement_index]
            sensor_point = world_to_vehicle(
                measurement,
                pose,
                origin_x_offset_m=self.sensor_x_offset_m,
            )
            if not self._inside_planning_window(sensor_point):
                continue
            track = ConeLandmark(
                track_id=self._next_track_id,
                world_x=float(measurement[0]),
                world_y=float(measurement[1]),
                last_seen_sec=now,
                last_semantic_sec=now,
            )
            self._next_track_id += 1
            self._tracks.append(track)
            observed_ids.add(track.track_id)

        self._prune(now)
        projected: List[TrackedCone] = []
        for track in self._tracks:
            point = world_to_vehicle(
                (track.world_x, track.world_y),
                pose,
                origin_x_offset_m=self.sensor_x_offset_m,
            )
            if not self._inside_planning_window(point):
                continue
            age = max(0.0, now - float(track.last_seen_sec))
            projected.append(
                TrackedCone(
                    track_id=track.track_id,
                    x=float(point[0]),
                    y=float(point[1]),
                    age_sec=age,
                    predicted=track.track_id not in observed_ids,
                )
            )
        projected.sort(key=lambda cone: math.hypot(cone.x, cone.y))
        return TrackerSnapshot(
            cones=tuple(projected),
            observed_count=sum(not cone.predicted for cone in projected),
            predicted_count=sum(cone.predicted for cone in projected),
            pose_jump_reset=pose_jump,
        )

    def _inside_planning_window(self, point: Point2) -> bool:
        return bool(
            self.minimum_forward_m <= float(point[0]) <= self.maximum_forward_m
            and abs(float(point[1])) <= self.maximum_abs_lateral_m
        )
