#!/usr/bin/env python3
import csv
import os
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import XycarMotor

from cone_il.preprocess import front_scan_vector, save_preprocessed_image_bgr


KEYBOARD_HELP = """
Cone keyboard recorder
----------------------
  w : speed up
  s : speed down / reverse
  a : steer left
  d : steer right
  x : brake speed to 0
  space : stop speed and steer
  q : quit
"""

LINE_KEYBOARD_HELP = """
Cone line keyboard recorder
---------------------------
Type keys then press Enter. Examples:
  w      : speed up once
  www    : speed up three times
  a / d  : steer left / right
  x      : brake speed to 0
  space  : stop speed and steer
  q      : quit
"""


class ConeDataRecorder(Node):
    """Record imitation-learning data while a human/manual node drives the car.

    Subscribes:
      - image_topic: front camera image
      - scan_topic: LaserScan, optional but saved by default
      - motor_topic: human control command used as label

    Output dataset layout:
      dataset_dir/
        labels.csv
        images/000000.jpg
        scans/000000.npy
    """

    def __init__(self, enable_keyboard_default: bool = False):
        super().__init__('cone_data_recorder')

        self.declare_parameter('image_topic', '/usb_cam/image_raw/front')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('motor_topic', '/xycar_motor')
        self.declare_parameter('fallback_motor_topic', '')
        self.declare_parameter('enable_keyboard_control', enable_keyboard_default)
        self.declare_parameter('drive_motor_topic', '/xycar_motor')
        self.declare_parameter('keyboard_publish_rate_hz', 20.0)
        self.declare_parameter('speed_step', 0.5)
        self.declare_parameter('steer_step', 3.0)
        self.declare_parameter('max_speed', 6.0)
        self.declare_parameter('max_steer_deg', 50.0)
        self.declare_parameter('dataset_dir', str(Path.home() / 'cone_il_dataset'))
        self.declare_parameter('save_rate_hz', 10.0)
        self.declare_parameter('resize_width', 160)
        self.declare_parameter('resize_height', 90)
        self.declare_parameter('roi_top_ratio', 0.45)
        self.declare_parameter('save_scan', True)
        self.declare_parameter('scan_bins', 181)
        self.declare_parameter('scan_front_degrees', 120.0)
        self.declare_parameter('scan_max_range', 8.0)
        self.declare_parameter('scan_min_range', 0.05)
        self.declare_parameter('scan_angle_offset', 0.0)
        self.declare_parameter('require_motion', True)
        self.declare_parameter('min_abs_speed', 0.1)
        self.declare_parameter('ignore_zero_commands', True)
        self.declare_parameter('zero_command_epsilon', 1e-3)
        self.declare_parameter('max_data_age_sec', 0.5)
        self.declare_parameter('flush_every_n', 1)

        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None
        self.latest_image_time: Optional[float] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time: Optional[float] = None
        self.latest_motor: Optional[XycarMotor] = None
        self.latest_motor_time: Optional[float] = None
        self.latest_motor_source = ''
        self.received_label_count = 0
        self.ignored_zero_count = 0
        self.keyboard_speed = 0.0
        self.keyboard_steer = 0.0
        self.keyboard_running = False
        self.keyboard_pub = None
        self.keyboard_timer = None
        self.keyboard_thread = None
        self.keyboard_input_fd = None
        self.keyboard_input_old_settings = None
        self.keyboard_line_thread = None

        dataset_dir = Path(str(self.get_parameter('dataset_dir').value)).expanduser()
        self.dataset_dir = dataset_dir
        self.image_dir = dataset_dir / 'images'
        self.scan_dir = dataset_dir / 'scans'
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.scan_dir.mkdir(parents=True, exist_ok=True)

        self.csv_path = dataset_dir / 'labels.csv'
        self.csv_exists = self.csv_path.exists()
        self.csv_file = open(self.csv_path, 'a', newline='')
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                'index', 'stamp', 'image_path', 'scan_path',
                'angle', 'speed', 'roi_top_ratio', 'resize_width', 'resize_height'
            ],
        )
        if not self.csv_exists:
            self.writer.writeheader()
            self.csv_file.flush()

        self.index = self._next_index()
        self.saved_count = 0

        image_topic = str(self.get_parameter('image_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        motor_topic = str(self.get_parameter('motor_topic').value)
        fallback_motor_topic = str(self.get_parameter('fallback_motor_topic').value)

        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            XycarMotor,
            motor_topic,
            lambda msg, source=motor_topic: self.motor_callback(msg, source),
            10,
        )
        if fallback_motor_topic and fallback_motor_topic != motor_topic:
            self.create_subscription(
                XycarMotor,
                fallback_motor_topic,
                lambda msg, source=fallback_motor_topic: self.motor_callback(msg, source),
                10,
            )

        rate = max(float(self.get_parameter('save_rate_hz').value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.save_once)

        self.get_logger().info(
            f'Recording cone IL data to {dataset_dir} | image={image_topic}, scan={scan_topic}, '
            f'label={motor_topic}, fallback={fallback_motor_topic}'
        )
        self.get_logger().info('Drive manually now. Press Ctrl+C to stop recording.')
        if bool(self.get_parameter('enable_keyboard_control').value):
            self._start_keyboard_control()

    def _next_index(self) -> int:
        existing = sorted(self.image_dir.glob('*.jpg'))
        if not existing:
            return 0
        try:
            return max(int(p.stem) for p in existing) + 1
        except ValueError:
            return len(existing)

    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_image_time = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f'image conversion failed: {exc}')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.latest_scan_time = time.monotonic()

    def motor_callback(self, msg: XycarMotor, source: str):
        eps = float(self.get_parameter('zero_command_epsilon').value)
        is_zero = abs(float(msg.angle)) <= eps and abs(float(msg.speed)) <= eps
        if bool(self.get_parameter('ignore_zero_commands').value) and is_zero:
            self.ignored_zero_count += 1
            if self.ignored_zero_count == 1 or self.ignored_zero_count % 100 == 0:
                self.get_logger().info(
                    f'ignoring zero label commands from {source} count={self.ignored_zero_count}'
                )
            return

        self.latest_motor = msg
        self.latest_motor_time = time.monotonic()
        self.latest_motor_source = source
        self.received_label_count += 1
        if self.received_label_count <= 5 or self.received_label_count % 50 == 0:
            self.get_logger().info(
                f'label command from {source}: angle={float(msg.angle):.1f}, '
                f'speed={float(msg.speed):.1f}, count={self.received_label_count}'
            )

    def _start_keyboard_control(self):
        drive_topic = str(self.get_parameter('drive_motor_topic').value)
        self.keyboard_pub = self.create_publisher(XycarMotor, drive_topic, 10)
        rate = max(float(self.get_parameter('keyboard_publish_rate_hz').value), 1.0)
        self.keyboard_timer = self.create_timer(1.0 / rate, self._publish_keyboard_cmd)
        self.keyboard_running = True

        if not sys.stdin.isatty():
            self.get_logger().warn(
                'keyboard control needs an interactive terminal. Run with ros2 run, not ros2 launch.'
            )
            return

        self.keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        self.get_logger().info(f'Keyboard recorder publishing drive commands to {drive_topic}')
        print(KEYBOARD_HELP)

    def start_keyboard_control_no_thread(self):
        drive_topic = str(self.get_parameter('drive_motor_topic').value)
        self.keyboard_pub = self.create_publisher(XycarMotor, drive_topic, 10)
        self.keyboard_running = True
        self.get_logger().info(f'Keyboard recorder publishing drive commands to {drive_topic}')
        print(KEYBOARD_HELP)

    def start_keyboard_control_line_mode(self):
        drive_topic = str(self.get_parameter('drive_motor_topic').value)
        self.keyboard_pub = self.create_publisher(XycarMotor, drive_topic, 10)
        self.keyboard_running = True
        self.keyboard_line_thread = threading.Thread(target=self._keyboard_line_loop, daemon=True)
        self.keyboard_line_thread.start()
        self.get_logger().info(f'Line keyboard recorder publishing drive commands to {drive_topic}')
        print(LINE_KEYBOARD_HELP)

    def _publish_keyboard_cmd(self):
        if self.keyboard_pub is None:
            return

        msg = XycarMotor()
        msg.angle = float(self.keyboard_steer)
        msg.speed = float(self.keyboard_speed)
        self.keyboard_pub.publish(msg)
        self.motor_callback(msg, 'keyboard')

    def _keyboard_loop(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.keyboard_running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    self._handle_keyboard_key(sys.stdin.read(1))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _keyboard_line_loop(self):
        while self.keyboard_running and rclpy.ok():
            try:
                line = sys.stdin.readline()
            except Exception as exc:
                self.get_logger().warn(f'line keyboard input failed: {exc}')
                return

            if line == '':
                time.sleep(0.05)
                continue

            for ch in line.rstrip('\n'):
                self._handle_keyboard_key(ch)

    def _handle_keyboard_key(self, key: str):
        key = key.lower()
        korean_key_map = {
            'ㅈ': 'w',
            'ㅁ': 'a',
            'ㄴ': 's',
            'ㅇ': 'd',
            'ㅌ': 'x',
        }
        key = korean_key_map.get(key, key)
        speed_step = float(self.get_parameter('speed_step').value)
        steer_step = float(self.get_parameter('steer_step').value)
        max_speed = float(self.get_parameter('max_speed').value)
        max_steer = float(self.get_parameter('max_steer_deg').value)

        if key == 'w':
            self.keyboard_speed = min(self.keyboard_speed + speed_step, max_speed)
        elif key == 's':
            self.keyboard_speed = max(self.keyboard_speed - speed_step, -max_speed)
        elif key == 'a':
            self.keyboard_steer = max(self.keyboard_steer - steer_step, -max_steer)
        elif key == 'd':
            self.keyboard_steer = min(self.keyboard_steer + steer_step, max_steer)
        elif key == 'x':
            self.keyboard_speed = 0.0
        elif key == ' ':
            self.keyboard_speed = 0.0
            self.keyboard_steer = 0.0
        elif key == 'q':
            self.keyboard_running = False
            return
        else:
            if key and key not in ('\n', '\r'):
                print(f'ignored key={key!r}', flush=True)
            return

        print(
            f'keyboard angle={self.keyboard_steer:6.1f}, speed={self.keyboard_speed:5.1f}',
            flush=True,
        )

    def poll_keyboard_once(self):
        if not self.keyboard_running:
            return
        fd = self.keyboard_input_fd if self.keyboard_input_fd is not None else sys.stdin.fileno()
        try:
            while select.select([fd], [], [], 0.0)[0]:
                key = os.read(fd, 16).decode(errors='ignore')
                if key:
                    for ch in key:
                        self._handle_keyboard_key(ch)
        except Exception as exc:
            self.get_logger().warn(f'keyboard input failed: {exc}')

    def open_keyboard_tty(self) -> bool:
        try:
            self.keyboard_input_fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            self.keyboard_input_old_settings = termios.tcgetattr(self.keyboard_input_fd)
            tty.setcbreak(self.keyboard_input_fd)
            self.get_logger().info('Keyboard input attached to /dev/tty')
            return True
        except Exception as exc:
            self.get_logger().warn(f'failed to open /dev/tty for keyboard input: {exc}')
            self.keyboard_input_fd = None
            self.keyboard_input_old_settings = None
            return False

    def close_keyboard_tty(self):
        if self.keyboard_input_fd is None:
            return
        try:
            if self.keyboard_input_old_settings is not None:
                termios.tcsetattr(
                    self.keyboard_input_fd,
                    termios.TCSADRAIN,
                    self.keyboard_input_old_settings,
                )
        except Exception:
            pass
        try:
            os.close(self.keyboard_input_fd)
        except Exception:
            pass
        self.keyboard_input_fd = None
        self.keyboard_input_old_settings = None

    def _fresh(self, t: Optional[float]) -> bool:
        if t is None:
            return False
        return time.monotonic() - t <= float(self.get_parameter('max_data_age_sec').value)

    def save_once(self):
        if self.latest_image is None or self.latest_motor is None:
            return
        if not self._fresh(self.latest_image_time) or not self._fresh(self.latest_motor_time):
            return

        angle = float(self.latest_motor.angle)
        speed = float(self.latest_motor.speed)
        if bool(self.get_parameter('require_motion').value):
            if abs(speed) < float(self.get_parameter('min_abs_speed').value):
                return

        idx = self.index
        image_name = f'{idx:06d}.jpg'
        scan_name = f'{idx:06d}.npy'
        image_path = self.image_dir / image_name
        scan_path = self.scan_dir / scan_name

        roi_top_ratio = float(self.get_parameter('roi_top_ratio').value)
        resize_width = int(self.get_parameter('resize_width').value)
        resize_height = int(self.get_parameter('resize_height').value)

        save_preprocessed_image_bgr(
            self.latest_image, str(image_path),
            roi_top_ratio=roi_top_ratio,
            width=resize_width,
            height=resize_height,
        )

        scan_rel = ''
        if bool(self.get_parameter('save_scan').value) and self.latest_scan is not None:
            scan_vec = front_scan_vector(
                self.latest_scan.ranges,
                self.latest_scan.angle_min,
                self.latest_scan.angle_increment,
                front_degrees=float(self.get_parameter('scan_front_degrees').value),
                bins=int(self.get_parameter('scan_bins').value),
                max_range=float(self.get_parameter('scan_max_range').value),
                min_range=float(self.get_parameter('scan_min_range').value),
                angle_offset=float(self.get_parameter('scan_angle_offset').value),
            )
            np.save(str(scan_path), scan_vec)
            scan_rel = os.path.join('scans', scan_name)

        self.writer.writerow({
            'index': idx,
            'stamp': f'{time.time():.6f}',
            'image_path': os.path.join('images', image_name),
            'scan_path': scan_rel,
            'angle': f'{angle:.6f}',
            'speed': f'{speed:.6f}',
            'roi_top_ratio': f'{roi_top_ratio:.4f}',
            'resize_width': resize_width,
            'resize_height': resize_height,
        })

        self.index += 1
        self.saved_count += 1
        flush_every_n = max(int(self.get_parameter('flush_every_n').value), 1)
        if self.saved_count % flush_every_n == 0:
            self.csv_file.flush()
            self.get_logger().info(
                f'saved={self.saved_count}, last angle={angle:.1f}, '
                f'speed={speed:.1f}, source={self.latest_motor_source}'
            )

    def destroy_node(self):
        self.keyboard_running = False
        if self.keyboard_pub is not None and rclpy.ok():
            msg = XycarMotor()
            msg.angle = 0.0
            msg.speed = 0.0
            for _ in range(5):
                try:
                    self.keyboard_pub.publish(msg)
                    time.sleep(0.02)
                except Exception:
                    break
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ConeDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'finished recording: saved={node.saved_count}, dir={node.dataset_dir}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


def raw_keyboard_main(args=None):
    rclpy.init(args=args)
    node = ConeDataRecorder(enable_keyboard_default=False)
    old_settings = None
    try:
        if not node.open_keyboard_tty() and sys.stdin.isatty():
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        elif node.keyboard_input_fd is None:
            node.get_logger().warn('keyboard recorder needs an interactive terminal')

        node.start_keyboard_control_no_thread()
        rate = max(float(node.get_parameter('keyboard_publish_rate_hz').value), 1.0)
        period = 1.0 / rate
        while rclpy.ok() and node.keyboard_running:
            start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            node.poll_keyboard_once()
            node._publish_keyboard_cmd()
            elapsed = time.monotonic() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_keyboard_tty()
        if old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        if rclpy.ok():
            node.get_logger().info(f'finished recording: saved={node.saved_count}, dir={node.dataset_dir}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def keyboard_main(args=None):
    rclpy.init(args=args)
    node = ConeDataRecorder(enable_keyboard_default=False)
    try:
        node.start_keyboard_control_line_mode()
        rate = max(float(node.get_parameter('keyboard_publish_rate_hz').value), 1.0)
        period = 1.0 / rate
        while rclpy.ok() and node.keyboard_running:
            start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            node._publish_keyboard_cmd()
            elapsed = time.monotonic() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info(f'finished recording: saved={node.saved_count}, dir={node.dataset_dir}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
