import math

import pytest

from my_drive.sim_mission_ground_truth_core import EgoPose
from my_drive.sim_mission_ground_truth_core import MissionTarget
from my_drive.sim_mission_ground_truth_core import project_target
from my_drive.sim_mission_ground_truth_core import quaternion_to_yaw
from my_drive.sim_mission_ground_truth_core import shuttle_local_speed
from my_drive.sim_mission_ground_truth_core import world_to_vehicle


def test_world_to_vehicle_respects_competition_leftward_heading():
    ego = EgoPose(x=0.0, y=0.0, yaw=math.pi)
    forward, left = world_to_vehicle(ego, -2.0, -0.5)
    assert forward == pytest.approx(2.0)
    assert left == pytest.approx(0.5)


def test_projection_places_vehicle_left_object_on_image_right():
    ego = EgoPose(x=0.0, y=0.0, yaw=0.0)
    target = MissionTarget("car", "car", 2.0, 0.5, 0.3, 0.16)
    box = project_target(
        ego,
        target,
        image_width=320,
        image_height=256,
        horizontal_fov_rad=1.7032,
        maximum_range_m=6.0,
    )
    assert box is not None
    assert 0.5 * (box.xmin + box.xmax) < 160


def test_projection_rejects_behind_out_of_range_and_outside_fov():
    ego = EgoPose(x=0.0, y=0.0, yaw=0.0)
    kwargs = dict(
        image_width=320,
        image_height=256,
        horizontal_fov_rad=1.0,
        maximum_range_m=4.0,
    )
    behind = MissionTarget("behind", "cone", -1.0, 0.0, 0.1, 0.2)
    far = MissionTarget("far", "cone", 5.0, 0.0, 0.1, 0.2)
    side = MissionTarget("side", "cone", 1.0, 2.0, 0.1, 0.2)
    assert project_target(ego, behind, **kwargs) is None
    assert project_target(ego, far, **kwargs) is None
    assert project_target(ego, side, **kwargs) is None


def test_quaternion_to_yaw_handles_pi_and_zero_quaternion():
    assert quaternion_to_yaw(0.0, 0.0, 1.0, 0.0) == pytest.approx(math.pi)
    assert quaternion_to_yaw(0.0, 0.0, 0.0, 0.0) == 0.0


def test_shuttle_reverses_only_at_configured_endpoints():
    settings = dict(minimum_x=-7.7, maximum_x=-4.6, speed_mps=0.16)
    assert shuttle_local_speed(
        world_x=-7.8, current_local_speed=0.16, **settings
    ) == -0.16
    assert shuttle_local_speed(
        world_x=-7.0, current_local_speed=-0.16, **settings
    ) == -0.16
    assert shuttle_local_speed(
        world_x=-4.5, current_local_speed=-0.16, **settings
    ) == 0.16
