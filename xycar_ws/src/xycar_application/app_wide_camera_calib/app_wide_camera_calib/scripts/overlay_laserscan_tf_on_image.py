#!/usr/bin/env python3

import math
import os
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


def quat_to_rot(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw

    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y],
    ], dtype=np.float64)


class TfLaserImageOverlay(Node):
    def __init__(self):
        super().__init__('tf_laser_image_overlay')

        self.declare_parameter('image_topic', '/wide_camera/rect/image_raw')
        self.declare_parameter('camera_info_topic', '/wide_camera/rect/camera_info')
        self.declare_parameter('scan_topic', '/scan')

        self.declare_parameter('camera_frame', 'wide_camera_optical_frame')
        self.declare_parameter('lidar_frame', 'laser_frame')

        self.declare_parameter('min_range_m', 0.30)
        self.declare_parameter('max_range_m', 4.0)
        self.declare_parameter('angle_min_deg', -70.0)
        self.declare_parameter('angle_max_deg', 70.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value

        self.camera_frame = self.get_parameter('camera_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value

        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.angle_min_deg = float(self.get_parameter('angle_min_deg').value)
        self.angle_max_deg = float(self.get_parameter('angle_max_deg').value)

        self.bridge = CvBridge()
        self.image = None
        self.scan = None
        self.K = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.timer = self.create_timer(0.03, self.draw)

        self.get_logger().info(f'image_topic       : {self.image_topic}')
        self.get_logger().info(f'camera_info_topic : {self.camera_info_topic}')
        self.get_logger().info(f'scan_topic        : {self.scan_topic}')
        self.get_logger().info(f'TF                : {self.camera_frame} <- {self.lidar_frame}')
        self.get_logger().info('keys: q quit | c capture screenshot')

    def image_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def info_cb(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def scan_cb(self, msg):
        self.scan = msg

    def get_T_camera_lidar(self):
        tf = self.tf_buffer.lookup_transform(
            self.camera_frame,
            self.lidar_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.2)
        )

        t = tf.transform.translation
        q = tf.transform.rotation

        R = quat_to_rot(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=np.float64)

        return R, trans

    def scan_to_points(self, scan):
        points = []

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < self.min_range_m or r > self.max_range_m:
                continue

            theta = scan.angle_min + i * scan.angle_increment
            theta_deg = math.degrees(theta)

            if theta_deg < self.angle_min_deg or theta_deg > self.angle_max_deg:
                continue

            x = r * math.cos(theta)
            y = r * math.sin(theta)
            z = 0.0

            points.append([x, y, z, r, theta_deg])

        if not points:
            return np.zeros((0, 5), dtype=np.float64)

        return np.array(points, dtype=np.float64)

    def project(self, pts_lidar, R, t):
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        pixels = []

        for row in pts_lidar:
            p_l = row[:3]
            r = row[3]
            deg = row[4]

            p_c = R @ p_l + t
            X, Y, Z = p_c

            if Z <= 0.05:
                continue

            u = fx * X / Z + cx
            v = fy * Y / Z + cy

            pixels.append((int(round(u)), int(round(v)), float(r), float(deg)))

        return pixels

    def put(self, img, text, y, color=(0, 255, 255)):
        cv2.putText(img, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

    def draw(self):
        if self.image is None:
            return

        img = self.image.copy()

        if self.K is None:
            self.put(img, 'waiting camera_info...', 40, (0, 0, 255))
            cv2.imshow('tf laser overlay', cv2.resize(img, None, fx=0.6, fy=0.6))
            cv2.waitKey(1)
            return

        if self.scan is None:
            self.put(img, 'waiting scan...', 40, (0, 0, 255))
            cv2.imshow('tf laser overlay', cv2.resize(img, None, fx=0.6, fy=0.6))
            cv2.waitKey(1)
            return

        try:
            R, t = self.get_T_camera_lidar()
        except Exception as e:
            self.put(img, f'waiting TF: {self.camera_frame} <- {self.lidar_frame}', 40, (0, 0, 255))
            self.put(img, str(e)[:90], 75, (0, 0, 255))
            cv2.imshow('tf laser overlay', cv2.resize(img, None, fx=0.6, fy=0.6))
            cv2.waitKey(1)
            return

        pts_lidar = self.scan_to_points(self.scan)
        pixels = self.project(pts_lidar, R, t)

        h, w = img.shape[:2]
        drawn = 0

        for u, v, r, deg in pixels:
            if 0 <= u < w and 0 <= v < h:
                if r < 1.0:
                    color = (0, 0, 255)
                elif r < 2.0:
                    color = (0, 165, 255)
                else:
                    color = (0, 255, 0)

                cv2.circle(img, (u, v), 4, color, -1)
                drawn += 1

                # 일부 점에 거리 라벨 표시
                if drawn % 12 == 0:
                    cv2.putText(
                        img,
                        f'{r:.2f}m',
                        (u + 5, v - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        color,
                        1
                    )

        y = 35
        self.put(img, f'TF overlay projected: {drawn}/{len(pts_lidar)}', y); y += 30
        self.put(img, f'filter range: {self.min_range_m:.2f}~{self.max_range_m:.1f}m', y); y += 30
        self.put(img, f'filter angle: {self.angle_min_deg:.0f}~{self.angle_max_deg:.0f}deg', y); y += 30
        self.put(img, 'red <1m | orange 1~2m | green >=2m', y); y += 30
        self.put(img, 'keys: q quit | c capture screenshot', y)

        show = cv2.resize(img, None, fx=0.6, fy=0.6)
        cv2.imshow('tf laser overlay', show)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            rclpy.shutdown()

        elif key == ord('c'):
            out_dir = os.path.expanduser('~/xycar_ws/src/xycar_application/app_wide_camera_calib/results')
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(out_dir, f'tf_overlay_{int(time.time())}.png')
            cv2.imwrite(path, img)
            self.get_logger().info(f'saved screenshot: {path}')


def main():
    rclpy.init()
    node = TfLaserImageOverlay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
