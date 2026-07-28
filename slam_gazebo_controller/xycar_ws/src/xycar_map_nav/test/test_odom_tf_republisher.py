import math

from xycar_map_nav.odom_tf_republisher import compose_planar_origin


def test_relative_gazebo_odom_is_composed_with_map_spawn_pose():
    x, y, quaternion = compose_planar_origin(
        2.0,
        0.0,
        (0.0, 0.0, 0.0, 1.0),
        origin_x=8.0,
        origin_y=7.0,
        origin_yaw=math.pi * 0.5,
    )
    assert math.isclose(x, 8.0, abs_tol=1.0e-9)
    assert math.isclose(y, 9.0, abs_tol=1.0e-9)
    assert math.isclose(quaternion[2], math.sqrt(0.5), abs_tol=1.0e-9)
    assert math.isclose(quaternion[3], math.sqrt(0.5), abs_tol=1.0e-9)
