#!/usr/bin/env python3

import os
import cv2
import yaml
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge


class RectifyWideCameraNode(Node):
    def __init__(self):
        super().__init__('rectify_wide_camera_node')

        self.declare_parameter('input_topic', '/wide_camera_mjpeg/image_raw/compressed')
        self.declare_parameter('image_topic', '/wide_camera/rect/image_raw')
        self.declare_parameter('camera_info_topic', '/wide_camera/rect/camera_info')
        self.declare_parameter('camera_yaml', '')
        self.declare_parameter('frame_id', 'wide_camera_optical_frame')
        self.declare_parameter('balance', 0.3)

        self.input_topic = self.get_parameter('input_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_yaml = self.get_parameter('camera_yaml').value
        self.frame_id = self.get_parameter('frame_id').value
        self.balance = float(self.get_parameter('balance').value)

        if not self.camera_yaml or not os.path.exists(self.camera_yaml):
            raise RuntimeError(f'camera_yaml not found: {self.camera_yaml}')

        self.bridge = CvBridge()

        self.K, self.D, self.image_size = self.load_fisheye_yaml(self.camera_yaml)

        self.newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.K,
            self.D,
            self.image_size,
            np.eye(3),
            balance=self.balance,
            new_size=self.image_size,
            fov_scale=1.0
        )

        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K,
            self.D,
            np.eye(3),
            self.newK,
            self.image_size,
            cv2.CV_16SC2
        )

        self.pub_img = self.create_publisher(
            Image,
            self.image_topic,
            qos_profile_sensor_data
        )

        self.pub_info = self.create_publisher(
            CameraInfo,
            self.camera_info_topic,
            qos_profile_sensor_data
        )

        self.sub = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self.callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(f'input_topic: {self.input_topic}')
        self.get_logger().info(f'image_topic: {self.image_topic}')
        self.get_logger().info(f'camera_info_topic: {self.camera_info_topic}')
        self.get_logger().info(f'camera_yaml: {self.camera_yaml}')
        self.get_logger().info(f'image_size: {self.image_size}')
        self.get_logger().info(f'balance: {self.balance}')
        self.get_logger().info(f'newK:\n{self.newK}')

    def load_fisheye_yaml(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        width = int(data['image_width'])
        height = int(data['image_height'])

        K = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
        D = np.array(data['distortion_coefficients']['data'], dtype=np.float64).reshape(4, 1)

        return K, D, (width, height)

    def make_camera_info(self, stamp):
        width, height = self.image_size

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.width = int(width)
        msg.height = int(height)

        # rectified image 기준이므로 distortion은 0으로 둔다.
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        msg.k = self.newK.reshape(-1).tolist()
        msg.r = np.eye(3, dtype=np.float64).reshape(-1).tolist()

        P = np.zeros((3, 4), dtype=np.float64)
        P[:3, :3] = self.newK
        msg.p = P.reshape(-1).tolist()

        return msg

    def callback(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn('JPEG decode failed')
            return

        rect = cv2.remap(
            frame,
            self.map1,
            self.map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT
        )

        img_msg = self.bridge.cv2_to_imgmsg(rect, encoding='bgr8')
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = self.frame_id

        info_msg = self.make_camera_info(msg.header.stamp)

        self.pub_img.publish(img_msg)
        self.pub_info.publish(info_msg)


def main():
    rclpy.init()
    node = RectifyWideCameraNode()
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
