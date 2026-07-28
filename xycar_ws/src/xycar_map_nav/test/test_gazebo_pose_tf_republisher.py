from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from xycar_map_nav.gazebo_pose_tf_republisher import (
    select_pose_transform,
)


def test_select_pose_transform_uses_configured_index():
    first = TransformStamped()
    first.transform.translation.x = 1.0
    second = TransformStamped()
    second.transform.translation.x = 2.0
    message = TFMessage(transforms=[first, second])

    selected = select_pose_transform(message, 0)

    assert selected is not None
    assert selected.transform.translation.x == 1.0


def test_select_pose_transform_rejects_missing_index():
    assert select_pose_transform(TFMessage(), 0) is None
