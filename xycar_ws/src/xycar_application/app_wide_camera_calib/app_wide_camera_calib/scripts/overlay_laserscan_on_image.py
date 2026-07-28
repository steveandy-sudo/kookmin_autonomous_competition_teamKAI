#!/usr/bin/env python3

import os
import math
import yaml
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge


def deg2rad(x):
    return x * math.pi / 180.0


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]], dtype=np.float64)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]], dtype=np.float64)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]], dtype=np.float64)


class LaserImageOverlay(Node):
    def __init__(self):
        super().__init__('laser_image_overlay')

        self.declare_parameter('image_topic', '/wide_camera/rect/image_raw')
        self.declare_parameter('camera_info_topic', '/wide_camera/rect/camera_info')
        self.declare_parameter('scan_topic', '/scan')

        self.declare_parameter(
            'output_yaml',
            os.path.expanduser('~/xycar_ws/src/xycar_application/app_wide_camera_calib/config/lidar_camera_extrinsic_manual.yaml')
        )

        self.declare_parameter('min_range_m', 0.30)
        self.declare_parameter('max_range_m', 4.0)
        self.declare_parameter('angle_min_deg', -70.0)
        self.declare_parameter('angle_max_deg', 70.0)

        # camera position in laser_frame
        # laser_frame: x forward, y left, z up
        self.declare_parameter('cam_x_m', 0.00)
        self.declare_parameter('cam_y_m', 0.00)
        self.declare_parameter('cam_z_m', 0.12)

        # correction rotation after default laser->camera_optical mapping
        self.declare_parameter('roll_deg', 0.0)
        self.declare_parameter('pitch_deg', 0.0)
        self.declare_parameter('yaw_deg', 0.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.output_yaml = self.get_parameter('output_yaml').value

        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.angle_min_deg = float(self.get_parameter('angle_min_deg').value)
        self.angle_max_deg = float(self.get_parameter('angle_max_deg').value)

        self.cam_x = float(self.get_parameter('cam_x_m').value)
        self.cam_y = float(self.get_parameter('cam_y_m').value)
        self.cam_z = float(self.get_parameter('cam_z_m').value)

        self.roll = float(self.get_parameter('roll_deg').value)
        self.pitch = float(self.get_parameter('pitch_deg').value)
        self.yaw = float(self.get_parameter('yaw_deg').value)

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_scan = None
        self.K = None

        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.timer = self.create_timer(0.03, self.draw)

        self.get_logger().info(f'image_topic       : {self.image_topic}')
        self.get_logger().info(f'camera_info_topic : {self.camera_info_topic}')
        self.get_logger().info(f'scan_topic        : {self.scan_topic}')
        self.get_logger().info('keys: q quit | p save | w/s x | a/d y | r/f z | i/k pitch | j/l yaw | u/o roll')

    def image_cb(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def info_cb(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def scan_cb(self, msg):
        self.latest_scan = msg

    def get_R_lidar_to_camera_optical(self):
        # laser_frame: x forward, y left, z up
        # camera_optical_frame: x right, y down, z forward
        R0 = np.array([
            [0, -1,  0],
            [0,  0, -1],
            [1,  0,  0],
        ], dtype=np.float64)

        Rcorr = (
            rot_z(deg2rad(self.yaw)) @
            rot_y(deg2rad(self.pitch)) @
            rot_x(deg2rad(self.roll))
        )

        return Rcorr @ R0

    def scan_points_lidar(self, scan):
        pts = []

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
            pts.append([x, y, z, theta_deg, r])

        return np.array(pts, dtype=np.float64) if pts else np.zeros((0, 5), dtype=np.float64)

    def project_points(self, pts_lidar):
        if self.K is None or pts_lidar.shape[0] == 0:
            return []

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        R = self.get_R_lidar_to_camera_optical()
        cam_pos_lidar = np.array([self.cam_x, self.cam_y, self.cam_z], dtype=np.float64)

        pixels = []

        for row in pts_lidar:
            p_l = row[:3]
            p_c = R @ (p_l - cam_pos_lidar)

            X, Y, Z = p_c

            if Z <= 0.05:
                continue

            u = fx * X / Z + cx
            v = fy * Y / Z + cy

            pixels.append((int(round(u)), int(round(v)), float(row[4]), float(row[3])))

        return pixels

    def save_yaml(self):
        R = self.get_R_lidar_to_camera_optical()
        cam_pos_lidar = np.array([self.cam_x, self.cam_y, self.cam_z], dtype=np.float64)
        t = -R @ cam_pos_lidar

        data = {
            'description': 'manual initial extrinsic: laser_frame to wide_camera_optical_frame',
            'lidar_frame': 'laser_frame',
            'camera_frame': 'wide_camera_optical_frame',
            'camera_position_in_laser_frame_m': {
                'x_forward': float(self.cam_x),
                'y_left': float(self.cam_y),
                'z_up': float(self.cam_z),
            },
            'correction_rotation_deg_in_camera_optical': {
                'roll': float(self.roll),
                'pitch': float(self.pitch),
                'yaw': float(self.yaw),
            },
            'T_camera_lidar': {
                'R_row_major': R.reshape(-1).tolist(),
                't_xyz': t.reshape(-1).tolist(),
            },
            'scan_filter': {
                'min_range_m': float(self.min_range_m),
                'max_range_m': float(self.max_range_m),
                'angle_min_deg': float(self.angle_min_deg),
                'angle_max_deg': float(self.angle_max_deg),
            }
        }

        os.makedirs(os.path.dirname(self.output_yaml), exist_ok=True)

        with open(self.output_yaml, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)

        self.get_logger().info(f'saved: {self.output_yaml}')

    def put(self, img, text, y, color=(0, 255, 255)):
        cv2.putText(img, text, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

    def draw(self):
        if self.latest_image is None:
            return

        img = self.latest_image.copy()

        if self.K is None:
            self.put(img, 'waiting camera_info...', 40, (0, 0, 255))
            cv2.imshow('laser overlay', cv2.resize(img, None, fx=0.6, fy=0.6))
            cv2.waitKey(1)
            return

        if self.latest_scan is None:
            self.put(img, 'waiting scan...', 40, (0, 0, 255))
            cv2.imshow('laser overlay', cv2.resize(img, None, fx=0.6, fy=0.6))
            cv2.waitKey(1)
            return

        pts_lidar = self.scan_points_lidar(self.latest_scan)
        pixels = self.project_points(pts_lidar)

        h, w = img.shape[:2]
        drawn = 0

        for u, v, r, deg in pixels:
            if 0 <= u < w and 0 <= v < h:
                if r < 1.0:
                    color = (0, 0, 255)
                elif r < 2.0:
                    color = (0, 255, 255)
                else:
                    color = (0, 255, 0)

                cv2.circle(img, (u, v), 3, color, -1)
                drawn += 1

        y = 35
        self.put(img, f'projected: {drawn}/{len(pts_lidar)}', y); y += 30
        self.put(img, f'cam pos in laser [m] x:{self.cam_x:.3f} y:{self.cam_y:.3f} z:{self.cam_z:.3f}', y); y += 30
        self.put(img, f'rot corr [deg] roll:{self.roll:.1f} pitch:{self.pitch:.1f} yaw:{self.yaw:.1f}', y); y += 30
        self.put(img, f'filter range:{self.min_range_m:.2f}~{self.max_range_m:.1f}m angle:{self.angle_min_deg:.0f}~{self.angle_max_deg:.0f}deg', y); y += 30
        self.put(img, 'keys: q quit | p save | w/s x | a/d y | r/f z | i/k pitch | j/l yaw | u/o roll', y)

        cv2.imshow('laser overlay', cv2.resize(img, None, fx=0.6, fy=0.6))

        key = cv2.waitKey(1) & 0xFF
        step_t = 0.01
        step_r = 1.0

        if key == ord('q'):
            rclpy.shutdown()
        elif key == ord('p'):
            self.save_yaml()
        elif key == ord('w'):
            self.cam_x += step_t
        elif key == ord('s'):
            self.cam_x -= step_t
        elif key == ord('a'):
            self.cam_y += step_t
        elif key == ord('d'):
            self.cam_y -= step_t
        elif key == ord('r'):
            self.cam_z += step_t
        elif key == ord('f'):
            self.cam_z -= step_t
        elif key == ord('i'):
            self.pitch += step_r
        elif key == ord('k'):
            self.pitch -= step_r
        elif key == ord('j'):
            self.yaw += step_r
        elif key == ord('l'):
            self.yaw -= step_r
        elif key == ord('u'):
            self.roll += step_r
        elif key == ord('o'):
            self.roll -= step_r


def main():
    rclpy.init()
    node = LaserImageOverlay()

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
