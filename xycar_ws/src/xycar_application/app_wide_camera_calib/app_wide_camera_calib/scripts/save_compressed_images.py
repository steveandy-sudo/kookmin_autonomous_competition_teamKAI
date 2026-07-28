#!/usr/bin/env python3

import csv
import math
import os
import time
from collections import defaultdict

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class GuidedCheckerboardCapture(Node):
    def __init__(self):
        super().__init__('guided_checkerboard_capture')

        self.declare_parameter('topic', '/wide_camera_mjpeg/image_raw/compressed')
        self.declare_parameter('output_dir', 'calibration_images')
        self.declare_parameter('board_cols', 8)
        self.declare_parameter('board_rows', 9)
        self.declare_parameter('max_images', 100)
        self.declare_parameter('min_interval', 0.5)
        self.declare_parameter('preview', True)
        self.declare_parameter('auto_save', True)
        self.declare_parameter('blur_threshold', 35.0)
        self.declare_parameter('display_scale', 0.5)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.board_cols = self.get_parameter('board_cols').get_parameter_value().integer_value
        self.board_rows = self.get_parameter('board_rows').get_parameter_value().integer_value
        self.max_images = self.get_parameter('max_images').get_parameter_value().integer_value
        self.min_interval = self.get_parameter('min_interval').get_parameter_value().double_value
        self.preview = self.get_parameter('preview').get_parameter_value().bool_value
        self.auto_save = self.get_parameter('auto_save').get_parameter_value().bool_value
        self.blur_threshold = self.get_parameter('blur_threshold').get_parameter_value().double_value
        self.display_scale = self.get_parameter('display_scale').get_parameter_value().double_value

        os.makedirs(self.output_dir, exist_ok=True)
        self.pattern_size = (self.board_cols, self.board_rows)

        self.count = len([f for f in os.listdir(self.output_dir) if f.endswith('.png')])
        self.last_save_time = 0.0
        self.last_pose = None

        self.zone_counts = defaultdict(int)
        self.size_counts = defaultdict(int)
        self.tilt_counts = defaultdict(int)
        self.roll_counts = defaultdict(int)

        self.target_zone = 8
        self.target_size = {
            'far/small': 25,
            'mid': 45,
            'near/large': 25,
        }
        self.target_tilt = {
            'low': 20,
            'mid': 45,
            'high': 25,
        }
        self.target_roll = {
            'left': 25,
            'flat': 30,
            'right': 25,
        }

        self.csv_path = os.path.join(self.output_dir, 'capture_stats.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'filename', 'time',
            'center_x_norm', 'center_y_norm',
            'zone',
            'area_ratio', 'size_bin',
            'tilt_score', 'tilt_bin',
            'roll_deg', 'roll_bin',
            'blur_score',
            'save_reason'
        ])

        self.sub = self.create_subscription(
            CompressedImage,
            self.topic,
            self.callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(f'Subscribed: {self.topic}')
        self.get_logger().info(f'Output dir: {self.output_dir}')
        self.get_logger().info(f'Checkerboard inner corners: {self.pattern_size}')
        self.get_logger().info(f'Max images: {self.max_images}')
        self.get_logger().info('Keys: q=quit, s=force save current detected frame')

        self.latest_frame = None
        self.latest_corners = None
        self.latest_stats = None
        self.force_save_requested = False

    def detect_corners(self, gray):
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH |
            cv2.CALIB_CB_NORMALIZE_IMAGE |
            cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)

        if not found:
            return False, None

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            1e-3
        )
        corners = cv2.cornerSubPix(
            gray, corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=criteria
        )
        return True, corners

    def compute_stats(self, frame, gray, corners):
        h, w = gray.shape[:2]
        pts = corners.reshape(-1, 2)

        center = pts.mean(axis=0)
        cx_norm = float(center[0] / w)
        cy_norm = float(center[1] / h)

        col = min(2, max(0, int(cx_norm * 3)))
        row = min(2, max(0, int(cy_norm * 3)))
        zone = f'{row},{col}'

        hull = cv2.convexHull(pts.astype(np.float32))
        area = float(cv2.contourArea(hull))
        area_ratio = area / float(w * h)

        if area_ratio < 0.055:
            size_bin = 'far/small'
        elif area_ratio < 0.18:
            size_bin = 'mid'
        else:
            size_bin = 'near/large'

        cols = self.board_cols
        rows = self.board_rows

        tl = pts[0]
        tr = pts[cols - 1]
        bl = pts[(rows - 1) * cols]
        br = pts[rows * cols - 1]

        top_len = np.linalg.norm(tr - tl) + 1e-9
        bottom_len = np.linalg.norm(br - bl) + 1e-9
        left_len = np.linalg.norm(bl - tl) + 1e-9
        right_len = np.linalg.norm(br - tr) + 1e-9

        tilt_score = max(
            abs(math.log(top_len / bottom_len)),
            abs(math.log(left_len / right_len))
        )

        if tilt_score < 0.12:
            tilt_bin = 'low'
        elif tilt_score < 0.35:
            tilt_bin = 'mid'
        else:
            tilt_bin = 'high'

        roll_deg = math.degrees(math.atan2(tr[1] - tl[1], tr[0] - tl[0]))

        if roll_deg < -12.0:
            roll_bin = 'left'
        elif roll_deg > 12.0:
            roll_bin = 'right'
        else:
            roll_bin = 'flat'

        blur_score = float(cv2.Laplacian(gray, cv2.CV_64F).var())

        return {
            'cx_norm': cx_norm,
            'cy_norm': cy_norm,
            'zone': zone,
            'area_ratio': area_ratio,
            'size_bin': size_bin,
            'tilt_score': tilt_score,
            'tilt_bin': tilt_bin,
            'roll_deg': roll_deg,
            'roll_bin': roll_bin,
            'blur_score': blur_score,
        }

    def pose_vector(self, stats):
        z_row, z_col = stats['zone'].split(',')
        return np.array([
            stats['cx_norm'],
            stats['cy_norm'],
            stats['area_ratio'],
            stats['tilt_score'],
            stats['roll_deg'] / 90.0,
            float(z_row) / 2.0,
            float(z_col) / 2.0,
        ], dtype=np.float64)

    def is_duplicate_pose(self, stats):
        pose = self.pose_vector(stats)

        if self.last_pose is None:
            return False

        dist = float(np.linalg.norm(pose - self.last_pose))
        return dist < 0.08

    def need_score_and_reason(self, stats):
        score = 0
        reasons = []

        if self.zone_counts[stats['zone']] < self.target_zone:
            score += 3
            reasons.append(f'zone {stats["zone"]}')

        if self.size_counts[stats['size_bin']] < self.target_size[stats['size_bin']]:
            score += 2
            reasons.append(stats['size_bin'])

        if self.tilt_counts[stats['tilt_bin']] < self.target_tilt[stats['tilt_bin']]:
            score += 2
            reasons.append(f'tilt {stats["tilt_bin"]}')

        if self.roll_counts[stats['roll_bin']] < self.target_roll[stats['roll_bin']]:
            score += 1
            reasons.append(f'roll {stats["roll_bin"]}')

        return score, '+'.join(reasons) if reasons else 'balanced'

    def save_frame(self, frame, stats, reason):
        filename = f'calib_{self.count:03d}.png'
        path = os.path.join(self.output_dir, filename)

        ok = cv2.imwrite(path, frame)
        if not ok:
            self.get_logger().warn(f'Failed to save: {path}')
            return

        self.zone_counts[stats['zone']] += 1
        self.size_counts[stats['size_bin']] += 1
        self.tilt_counts[stats['tilt_bin']] += 1
        self.roll_counts[stats['roll_bin']] += 1

        self.csv_writer.writerow([
            filename,
            f'{time.time():.3f}',
            f'{stats["cx_norm"]:.4f}',
            f'{stats["cy_norm"]:.4f}',
            stats['zone'],
            f'{stats["area_ratio"]:.6f}',
            stats['size_bin'],
            f'{stats["tilt_score"]:.4f}',
            stats['tilt_bin'],
            f'{stats["roll_deg"]:.2f}',
            stats['roll_bin'],
            f'{stats["blur_score"]:.2f}',
            reason
        ])
        self.csv_file.flush()

        self.last_pose = self.pose_vector(stats)
        self.count += 1
        self.last_save_time = time.time()

        self.get_logger().info(
            f'Saved {filename} ({self.count}/{self.max_images}) | {reason}'
        )

    def draw_bar(self, panel, label, value, target, x, y, w=260, h=18):
        ratio = 0.0 if target <= 0 else min(1.0, value / target)
        cv2.putText(panel, f'{label}: {value}/{target}', (x, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        cv2.rectangle(panel, (x, y), (x + w, y + h), (180, 180, 180), 1)
        color = (0, 180, 0) if ratio >= 1.0 else (0, 140, 255)
        cv2.rectangle(panel, (x, y), (x + int(w * ratio), y + h), color, -1)

    def recommend_text(self):
        recs = []

        # 3x3 위치 부족
        min_zone = None
        min_val = 10**9
        for r in range(3):
            for c in range(3):
                z = f'{r},{c}'
                if self.zone_counts[z] < min_val:
                    min_val = self.zone_counts[z]
                    min_zone = (r, c)

        if min_zone is not None and min_val < self.target_zone:
            r, c = min_zone
            y_txt = ['TOP', 'CENTER-Y', 'BOTTOM'][r]
            x_txt = ['LEFT', 'CENTER-X', 'RIGHT'][c]
            recs.append(f'Move board to {y_txt}-{x_txt}')

        for k, t in self.target_size.items():
            if self.size_counts[k] < t:
                if k == 'far/small':
                    recs.append('Need FAR / SMALL board')
                elif k == 'near/large':
                    recs.append('Need NEAR / LARGE board')
                else:
                    recs.append('Need MID size board')
                break

        for k, t in self.target_tilt.items():
            if self.tilt_counts[k] < t:
                if k == 'high':
                    recs.append('Need STRONG perspective tilt')
                elif k == 'mid':
                    recs.append('Need moderate tilt')
                else:
                    recs.append('Need front-parallel view')
                break

        for k, t in self.target_roll.items():
            if self.roll_counts[k] < t:
                if k == 'left':
                    recs.append('Need rotate board LEFT')
                elif k == 'right':
                    recs.append('Need rotate board RIGHT')
                else:
                    recs.append('Need flat roll view')
                break

        return recs[:5]

    def draw_panel(self, frame_shape, found, stats):
        h = int(frame_shape[0] * self.display_scale)
        panel_w = 520
        panel = np.full((h, panel_w, 3), 245, dtype=np.uint8)

        y = 32
        cv2.putText(panel, 'GUIDED CHECKERBOARD CAPTURE', (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
        y += 35

        self.draw_bar(panel, 'TOTAL', self.count, self.max_images, 15, y, 350, 20)
        y += 46

        status = 'DETECTED' if found else 'NOT FOUND'
        status_color = (0, 150, 0) if found else (0, 0, 200)
        cv2.putText(panel, f'STATUS: {status}', (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2)
        y += 28

        if stats:
            cv2.putText(panel, f'current zone: {stats["zone"]}', (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1)
            y += 22
            cv2.putText(panel, f'area: {stats["area_ratio"]:.3f} | {stats["size_bin"]}', (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1)
            y += 22
            cv2.putText(panel, f'tilt: {stats["tilt_score"]:.3f} | {stats["tilt_bin"]}', (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1)
            y += 22
            cv2.putText(panel, f'roll: {stats["roll_deg"]:.1f} deg | {stats["roll_bin"]}', (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 1)
            y += 22
            blur_color = (0, 150, 0) if stats['blur_score'] >= self.blur_threshold else (0, 0, 200)
            cv2.putText(panel, f'blur: {stats["blur_score"]:.1f}', (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, blur_color, 1)
            y += 32
        else:
            y += 105

        cv2.putText(panel, '3x3 CENTER COVERAGE', (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 2)
        y += 14

        cell_w, cell_h = 70, 38
        start_x = 20
        start_y = y
        for r in range(3):
            for c in range(3):
                z = f'{r},{c}'
                val = self.zone_counts[z]
                ok = val >= self.target_zone
                color = (160, 230, 160) if ok else (200, 220, 255)
                x0 = start_x + c * cell_w
                y0 = start_y + r * cell_h
                cv2.rectangle(panel, (x0, y0), (x0 + cell_w - 5, y0 + cell_h - 5), color, -1)
                cv2.rectangle(panel, (x0, y0), (x0 + cell_w - 5, y0 + cell_h - 5), (80, 80, 80), 1)
                cv2.putText(panel, str(val), (x0 + 22, y0 + 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)

        y = start_y + 3 * cell_h + 30

        cv2.putText(panel, 'SIZE', (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 2)
        y += 24
        for k in ['far/small', 'mid', 'near/large']:
            self.draw_bar(panel, k, self.size_counts[k], self.target_size[k], 15, y, 290, 16)
            y += 34

        cv2.putText(panel, 'PERSPECTIVE TILT', (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 2)
        y += 24
        for k in ['low', 'mid', 'high']:
            self.draw_bar(panel, k, self.tilt_counts[k], self.target_tilt[k], 15, y, 290, 16)
            y += 34

        cv2.putText(panel, 'ROLL', (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 2)
        y += 24
        for k in ['left', 'flat', 'right']:
            self.draw_bar(panel, k, self.roll_counts[k], self.target_roll[k], 15, y, 290, 16)
            y += 34

        cv2.putText(panel, 'RECOMMENDATION', (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 2)
        y += 25
        for rec in self.recommend_text():
            cv2.putText(panel, f'- {rec}', (20, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 0, 180), 1)
            y += 22

        cv2.putText(panel, 'keys: s=force save, q=quit', (15, h - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 0, 0), 1)

        return panel

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn('JPEG decode failed')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = self.detect_corners(gray)

        stats = None
        vis = frame.copy()

        if found:
            stats = self.compute_stats(frame, gray, corners)
            cv2.drawChessboardCorners(vis, self.pattern_size, corners, found)

            now = time.time()
            enough_time = (now - self.last_save_time) >= self.min_interval
            not_blurry = stats['blur_score'] >= self.blur_threshold
            not_duplicate = not self.is_duplicate_pose(stats)
            need_score, reason = self.need_score_and_reason(stats)

            should_auto_save = (
                self.auto_save and
                self.count < self.max_images and
                enough_time and
                not_blurry and
                not_duplicate and
                need_score > 0
            )

            if should_auto_save or self.force_save_requested:
                save_reason = 'force' if self.force_save_requested else reason
                self.save_frame(frame, stats, save_reason)
                self.force_save_requested = False

        else:
            cv2.putText(vis, 'checkerboard not found', (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        if self.preview:
            show = cv2.resize(vis, None, fx=self.display_scale, fy=self.display_scale)
            panel = self.draw_panel(frame.shape, found, stats)

            if show.shape[0] != panel.shape[0]:
                panel = cv2.resize(panel, (panel.shape[1], show.shape[0]))

            combined = np.hstack([show, panel])
            cv2.imshow('guided checkerboard capture', combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rclpy.shutdown()
                return
            if key == ord('s'):
                self.force_save_requested = True

        if self.count >= self.max_images:
            self.get_logger().info('Reached max_images. Stop node.')
            rclpy.shutdown()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GuidedCheckerboardCapture()

    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
