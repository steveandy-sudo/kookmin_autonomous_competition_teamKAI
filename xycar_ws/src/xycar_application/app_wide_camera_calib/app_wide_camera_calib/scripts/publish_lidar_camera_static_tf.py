#!/usr/bin/env python3

import os
import math
import yaml
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def rotation_matrix_to_quaternion(R):
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    tr = np.trace(R)

    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    return q


class LidarCameraStaticTF(Node):
    def __init__(self):
        super().__init__('lidar_camera_static_tf')

        default_yaml = os.path.expanduser(
            '~/xycar_ws/src/xycar_application/app_wide_camera_calib/config/lidar_camera_extrinsic_final.yaml'
        )
        self.declare_parameter('yaml_path', default_yaml)

        yaml_path = self.get_parameter('yaml_path').value

        if not os.path.exists(yaml_path):
            raise RuntimeError(f'YAML not found: {yaml_path}')

        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        parent_frame = data.get('camera_frame', 'wide_camera_optical_frame')
        child_frame = data.get('lidar_frame', 'laser_frame')

        R = np.array(data['T_camera_lidar']['R_row_major'], dtype=np.float64).reshape(3, 3)
        t = np.array(data['T_camera_lidar']['t_xyz'], dtype=np.float64).reshape(3)

        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame

        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])

        tf_msg.transform.rotation.x = float(qx)
        tf_msg.transform.rotation.y = float(qy)
        tf_msg.transform.rotation.z = float(qz)
        tf_msg.transform.rotation.w = float(qw)

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(tf_msg)

        self.get_logger().info(f'Loaded YAML: {yaml_path}')
        self.get_logger().info(f'Published static TF: {parent_frame} -> {child_frame}')
        self.get_logger().info(f'translation xyz: {t.tolist()}')
        self.get_logger().info(f'quaternion xyzw: {[float(qx), float(qy), float(qz), float(qw)]}')


def main():
    rclpy.init()
    node = LidarCameraStaticTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
