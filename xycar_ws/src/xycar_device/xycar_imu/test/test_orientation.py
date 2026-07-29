import math

from xycar_imu.orientation import (
    euler_from_ros_quaternion,
    ros_quaternion_from_euler,
)


def test_zero_euler_is_ros_identity_quaternion():
    assert ros_quaternion_from_euler(0.0, 0.0, 0.0) == (
        0.0,
        0.0,
        0.0,
        1.0,
    )


def test_positive_yaw_uses_ros_z_and_w_components():
    x, y, z, w = ros_quaternion_from_euler(
        0.0,
        0.0,
        math.pi / 2.0,
    )

    assert math.isclose(x, 0.0, abs_tol=1.0e-12)
    assert math.isclose(y, 0.0, abs_tol=1.0e-12)
    assert math.isclose(z, math.sqrt(0.5), abs_tol=1.0e-12)
    assert math.isclose(w, math.sqrt(0.5), abs_tol=1.0e-12)


def test_ros_quaternion_round_trip_preserves_euler_angles():
    expected = (math.radians(12.0), math.radians(-7.0), math.radians(83.0))
    quaternion = ros_quaternion_from_euler(*expected)
    measured = euler_from_ros_quaternion(*quaternion)

    for actual, target in zip(measured, expected):
        assert math.isclose(actual, target, abs_tol=1.0e-12)
