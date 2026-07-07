#!/usr/bin/env python3

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float32MultiArray, String

try:
    from xycar_msgs.msg import XycarMotor
except ImportError:  # pragma: no cover - available on the real Xycar workspace
    XycarMotor = None

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - reported at runtime on the robot
    cv2 = None
    CvBridge = None

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

from il_data_tools.record_utils import (
    atomic_write_json,
    image_suffix,
    make_session_dir,
    motor_from_msg,
    open_samples_csv,
    relative_to_session,
    ros_time_to_ns,
    stamp_to_ns,
    string_from_msg,
    write_session_readme,
)
from il_data_tools.safety_checks import disk_space_warning, session_size_warning
from il_data_tools.sync_buffer import TimedBuffer


class DatasetRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("il_dataset_recorder")
        self._declare_parameters()
        self.params = self._read_parameters()
        self.session_dir = make_session_dir(
            self.params["output_dir"], self.params["session_name"]
        )
        self.samples_csv_handle, self.samples_writer = open_samples_csv(
            self.session_dir / "samples.csv"
        )
        self.bridge = CvBridge() if CvBridge is not None else None

        self.front_buffer = TimedBuffer()
        self.left_buffer = TimedBuffer()
        self.right_buffer = TimedBuffer()
        self.rear_buffer = TimedBuffer()
        self.scan_buffer = TimedBuffer()
        self.imu_buffer = TimedBuffer()
        self.odom_buffer = TimedBuffer()
        self.motor_buffer = TimedBuffer()
        self.label_buffer = TimedBuffer()

        self.sample_count = 0
        self.image_count = 0
        self.skipped_missing_motor = 0
        self.skipped_rate_limit = 0
        self.skipped_stopped = 0
        self.skipped_disk_limit = 0
        self.last_saved_ns: Optional[int] = None
        self.recording_enabled = self.params["enable_recording_on_start"]
        self.start_wall_time = datetime.now()
        self.warnings = []

        self._write_initial_files()
        self._create_subscriptions()
        self.create_timer(5.0, self._monitor_health)
        self.get_logger().info(f"IL recorder session: {self.session_dir}")
        self.get_logger().warn(
            "Recorder is subscribe-only and does not publish /xycar_motor."
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "output_dir": "~/xycar_ws/datasets/il",
            "session_name": "session",
            "camera_front_topic": "/usb_cam/image_raw/front",
            "camera_left_topic": "/usb_cam/image_raw/left",
            "camera_right_topic": "/usb_cam/image_raw/right",
            "camera_rear_topic": "/usb_cam/image_raw/behind",
            "scan_topic": "/scan",
            "imu_topic": "/imu",
            "odom_topic": "/odom",
            "motor_topic": "/xycar_motor",
            "mission_label_topic": "/il/mission_label",
            "motor_msg_type": "xycar_msgs/XycarMotor",
            "save_front_image": True,
            "save_side_images": False,
            "save_scan_npz": True,
            "save_imu": False,
            "save_odom": False,
            "image_format": "jpg",
            "jpeg_quality": 90,
            "max_save_rate_hz": 10.0,
            "min_abs_speed_to_save": 0.0,
            "save_when_stopped": True,
            "require_motor_command": True,
            "approximate_sync_tolerance_sec": 0.20,
            "flush_every_n_samples": 30,
            "max_session_duration_sec": 0.0,
            "max_disk_usage_gb": 20.0,
            "enable_recording_on_start": True,
            "source_mode": "manual",
            "notes": "",
            "write_metadata": True,
            "write_session_readme": False,
            "metadata_update_every_n_samples": 0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)

    def _read_parameters(self) -> Dict[str, Any]:
        return {
            "output_dir": self._get_str("output_dir"),
            "session_name": self._get_str("session_name"),
            "camera_front_topic": self._get_str("camera_front_topic"),
            "camera_left_topic": self._get_str("camera_left_topic"),
            "camera_right_topic": self._get_str("camera_right_topic"),
            "camera_rear_topic": self._get_str("camera_rear_topic"),
            "scan_topic": self._get_str("scan_topic"),
            "imu_topic": self._get_str("imu_topic"),
            "odom_topic": self._get_str("odom_topic"),
            "motor_topic": self._get_str("motor_topic"),
            "mission_label_topic": self._get_str("mission_label_topic"),
            "motor_msg_type": self._get_str("motor_msg_type"),
            "save_front_image": self._get_bool("save_front_image"),
            "save_side_images": self._get_bool("save_side_images"),
            "save_scan_npz": self._get_bool("save_scan_npz"),
            "save_imu": self._get_bool("save_imu"),
            "save_odom": self._get_bool("save_odom"),
            "image_format": image_suffix(self._get_str("image_format")),
            "jpeg_quality": self._get_int("jpeg_quality"),
            "max_save_rate_hz": self._get_float("max_save_rate_hz"),
            "min_abs_speed_to_save": self._get_float("min_abs_speed_to_save"),
            "save_when_stopped": self._get_bool("save_when_stopped"),
            "require_motor_command": self._get_bool("require_motor_command"),
            "approximate_sync_tolerance_sec": self._get_float(
                "approximate_sync_tolerance_sec"
            ),
            "flush_every_n_samples": self._get_int("flush_every_n_samples"),
            "max_session_duration_sec": self._get_float("max_session_duration_sec"),
            "max_disk_usage_gb": self._get_float("max_disk_usage_gb"),
            "enable_recording_on_start": self._get_bool("enable_recording_on_start"),
            "source_mode": self._get_str("source_mode"),
            "notes": self._get_str("notes"),
            "write_metadata": self._get_bool("write_metadata"),
            "write_session_readme": self._get_bool("write_session_readme"),
            "metadata_update_every_n_samples": self._get_int(
                "metadata_update_every_n_samples"
            ),
        }

    def _get_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _get_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        return str(value).lower() in {"1", "true", "yes", "on"}

    def _get_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _get_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _write_initial_files(self) -> None:
        if self.params["write_session_readme"]:
            write_session_readme(
                self.session_dir,
                self.params["session_name"],
                {
                    "camera_front_topic": self.params["camera_front_topic"],
                    "camera_left_topic": self.params["camera_left_topic"],
                    "camera_right_topic": self.params["camera_right_topic"],
                    "camera_rear_topic": self.params["camera_rear_topic"],
                    "scan_topic": self.params["scan_topic"],
                    "imu_topic": self.params["imu_topic"],
                    "odom_topic": self.params["odom_topic"],
                    "motor_topic": self.params["motor_topic"],
                    "mission_label_topic": self.params["mission_label_topic"],
                },
            )
        if self.params["write_metadata"]:
            self._write_metadata(final=False)

    def _create_subscriptions(self) -> None:
        qos = 10
        self.create_subscription(
            Image, self.params["camera_front_topic"], self._front_image_cb, qos
        )
        if self.params["save_side_images"]:
            self.create_subscription(
                Image,
                self.params["camera_left_topic"],
                lambda msg: self._image_to_buffer(self.left_buffer, msg),
                qos,
            )
            self.create_subscription(
                Image,
                self.params["camera_right_topic"],
                lambda msg: self._image_to_buffer(self.right_buffer, msg),
                qos,
            )
            self.create_subscription(
                Image,
                self.params["camera_rear_topic"],
                lambda msg: self._image_to_buffer(self.rear_buffer, msg),
                qos,
            )

        self.create_subscription(
            LaserScan,
            self.params["scan_topic"],
            lambda msg: self.scan_buffer.add(stamp_to_ns(msg), msg),
            qos,
        )
        self.create_subscription(
            Imu,
            self.params["imu_topic"],
            lambda msg: self.imu_buffer.add(stamp_to_ns(msg), msg),
            qos,
        )
        self.create_subscription(
            Odometry,
            self.params["odom_topic"],
            lambda msg: self.odom_buffer.add(stamp_to_ns(msg), msg),
            qos,
        )

        motor_type = self.params["motor_msg_type"].lower()
        if "float32multiarray" in motor_type or XycarMotor is None:
            self.create_subscription(
                Float32MultiArray,
                self.params["motor_topic"],
                self._motor_cb,
                qos,
            )
        else:
            self.create_subscription(XycarMotor, self.params["motor_topic"], self._motor_cb, qos)

        self.create_subscription(
            String,
            self.params["mission_label_topic"],
            lambda msg: self.label_buffer.add(stamp_to_ns(msg), msg),
            qos,
        )

    def _image_to_buffer(self, buffer: TimedBuffer, msg: Image) -> None:
        buffer.add(stamp_to_ns(msg), msg)

    def _motor_cb(self, msg: Any) -> None:
        self.motor_buffer.add(stamp_to_ns(msg), msg)

    def _front_image_cb(self, msg: Image) -> None:
        stamp_ns = stamp_to_ns(msg, ros_time_to_ns(self.get_clock().now()))
        self.front_buffer.add(stamp_ns, msg)
        if not self.recording_enabled:
            return
        self._try_record_sample(stamp_ns, msg)

    def _try_record_sample(self, stamp_ns: int, front_msg: Image) -> None:
        if not self._within_duration_limit():
            self.recording_enabled = False
            self.get_logger().warn("Max session duration reached; recording stopped.")
            return
        if not self._within_rate_limit(stamp_ns):
            self.skipped_rate_limit += 1
            return
        disk_warning = session_size_warning(
            self.session_dir, self.params["max_disk_usage_gb"]
        )
        if disk_warning:
            self.skipped_disk_limit += 1
            self.recording_enabled = False
            self._remember_warning(disk_warning)
            self.get_logger().error(disk_warning)
            return

        tolerance_ns = int(self.params["approximate_sync_tolerance_sec"] * 1e9)
        motor_item = self.motor_buffer.nearest(stamp_ns, tolerance_ns)
        if motor_item is None:
            self.skipped_missing_motor += 1
            if self.params["require_motor_command"]:
                return
            motor_angle, motor_speed = "", ""
        else:
            motor_angle, motor_speed = motor_from_msg(motor_item.msg)

        if motor_item is not None and not self._should_save_speed(float(motor_speed)):
            self.skipped_stopped += 1
            return

        label_item = self.label_buffer.nearest(stamp_ns, tolerance_ns)
        mission_label = string_from_msg(label_item.msg) if label_item else "idle"

        paths = {"front": None, "left": None, "right": None, "rear": None}
        if self.params["save_front_image"]:
            paths["front"] = self._save_image("front", stamp_ns, front_msg)
        if self.params["save_side_images"]:
            for name, buffer in [
                ("left", self.left_buffer),
                ("right", self.right_buffer),
                ("rear", self.rear_buffer),
            ]:
                item = buffer.nearest(stamp_ns, tolerance_ns)
                if item:
                    paths[name] = self._save_image(name, stamp_ns, item.msg)

        scan_path = ""
        if self.params["save_scan_npz"]:
            scan_item = self.scan_buffer.nearest(stamp_ns, tolerance_ns)
            if scan_item:
                scan_path = relative_to_session(
                    self.session_dir, self._save_scan(stamp_ns, scan_item.msg)
                )

        imu_item = self.imu_buffer.nearest(stamp_ns, tolerance_ns) if self.params["save_imu"] else None
        odom_item = (
            self.odom_buffer.nearest(stamp_ns, tolerance_ns)
            if self.params["save_odom"]
            else None
        )

        row = self._make_empty_row(stamp_ns)
        row.update(
            {
                "image_front": relative_to_session(self.session_dir, paths["front"]),
                "image_left": relative_to_session(self.session_dir, paths["left"]),
                "image_right": relative_to_session(self.session_dir, paths["right"]),
                "image_rear": relative_to_session(self.session_dir, paths["rear"]),
                "motor_angle": motor_angle,
                "motor_speed": motor_speed,
                "mission_label": mission_label,
                "lap_index": -1,
                "source_mode": self.params["source_mode"],
                "scan_path": scan_path,
                "notes": self.params["notes"],
            }
        )
        if imu_item:
            row.update(self._imu_fields(imu_item.msg))
        if odom_item:
            row.update(self._odom_fields(odom_item.msg))

        self.samples_writer.writerow(row)
        self.sample_count += 1
        self.last_saved_ns = stamp_ns
        if self.sample_count % max(1, self.params["flush_every_n_samples"]) == 0:
            self.samples_csv_handle.flush()
        metadata_update_every = self.params["metadata_update_every_n_samples"]
        if (
            self.params["write_metadata"]
            and metadata_update_every > 0
            and self.sample_count % metadata_update_every == 0
        ):
            self._write_metadata(final=False)

    def _within_duration_limit(self) -> bool:
        limit = self.params["max_session_duration_sec"]
        if limit <= 0:
            return True
        elapsed = (datetime.now() - self.start_wall_time).total_seconds()
        return elapsed <= limit

    def _within_rate_limit(self, stamp_ns: int) -> bool:
        max_rate = self.params["max_save_rate_hz"]
        if max_rate <= 0 or self.last_saved_ns is None:
            return True
        min_delta_ns = int(1e9 / max_rate)
        return stamp_ns - self.last_saved_ns >= min_delta_ns

    def _should_save_speed(self, speed: float) -> bool:
        threshold = self.params["min_abs_speed_to_save"]
        if abs(speed) >= threshold:
            return True
        return self.params["save_when_stopped"]

    def _save_image(self, camera_name: str, stamp_ns: int, msg: Image) -> Path:
        if self.bridge is None or cv2 is None:
            raise RuntimeError("cv_bridge and OpenCV are required to save images")
        suffix = self.params["image_format"]
        out_path = self.session_dir / "images" / camera_name / f"{stamp_ns}.{suffix}"
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if suffix == "jpg":
            params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.params["jpeg_quality"])]
        else:
            params = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
        ok = cv2.imwrite(str(out_path), cv_image, params)
        if not ok:
            raise RuntimeError(f"failed to write image: {out_path}")
        self.image_count += 1
        return out_path

    def _save_scan(self, stamp_ns: int, msg: LaserScan) -> Optional[Path]:
        if np is None:
            self._remember_warning("numpy is unavailable; scan npz was not saved")
            return None
        out_path = self.session_dir / "scan" / f"{stamp_ns}.npz"
        np.savez_compressed(
            out_path,
            ranges=np.array(msg.ranges, dtype=np.float32),
            intensities=np.array(msg.intensities, dtype=np.float32),
            angle_min=float(msg.angle_min),
            angle_max=float(msg.angle_max),
            angle_increment=float(msg.angle_increment),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
        )
        return out_path

    def _make_empty_row(self, stamp_ns: int) -> Dict[str, Any]:
        row = {field: "" for field in self.samples_writer.fieldnames or []}
        row["timestamp_ns"] = stamp_ns
        return row

    def _imu_fields(self, msg: Imu) -> Dict[str, float]:
        return {
            "imu_orientation_x": msg.orientation.x,
            "imu_orientation_y": msg.orientation.y,
            "imu_orientation_z": msg.orientation.z,
            "imu_orientation_w": msg.orientation.w,
            "imu_angular_velocity_x": msg.angular_velocity.x,
            "imu_angular_velocity_y": msg.angular_velocity.y,
            "imu_angular_velocity_z": msg.angular_velocity.z,
            "imu_linear_acceleration_x": msg.linear_acceleration.x,
            "imu_linear_acceleration_y": msg.linear_acceleration.y,
            "imu_linear_acceleration_z": msg.linear_acceleration.z,
        }

    def _odom_fields(self, msg: Odometry) -> Dict[str, float]:
        pose = msg.pose.pose
        twist = msg.twist.twist
        return {
            "odom_position_x": pose.position.x,
            "odom_position_y": pose.position.y,
            "odom_position_z": pose.position.z,
            "odom_orientation_x": pose.orientation.x,
            "odom_orientation_y": pose.orientation.y,
            "odom_orientation_z": pose.orientation.z,
            "odom_orientation_w": pose.orientation.w,
            "odom_linear_x": twist.linear.x,
            "odom_angular_z": twist.angular.z,
        }

    def _monitor_health(self) -> None:
        now_ns = ros_time_to_ns(self.get_clock().now())
        if len(self.front_buffer) == 0:
            self._log_warn_once("no front image received yet")
        if len(self.motor_buffer) == 0:
            self._log_warn_once("no motor command received yet")
        elif self.motor_buffer.age_ns(now_ns) and self.motor_buffer.age_ns(now_ns) > 1_000_000_000:
            self._log_warn_once("latest motor command is older than 1 second")
        if len(self.scan_buffer) == 0:
            self._log_warn_once("no /scan message received yet")
        disk_warning = disk_space_warning(self.session_dir, min_free_gb=2.0)
        if disk_warning:
            self._log_warn_once(disk_warning)
        motor_publishers = self.get_publishers_info_by_topic(self.params["motor_topic"])
        if len(motor_publishers) > 1:
            self._log_warn_once(
                f"{self.params['motor_topic']} has {len(motor_publishers)} publishers; only one controller should publish motor commands"
            )

    def _log_warn_once(self, message: str) -> None:
        if message not in self.warnings:
            self.warnings.append(message)
            self.get_logger().warn(message)

    def _remember_warning(self, message: str) -> None:
        if message not in self.warnings:
            self.warnings.append(message)

    def _write_metadata(self, final: bool) -> None:
        metadata = {
            "package": "il_data_tools",
            "session_name": self.params["session_name"],
            "session_dir": str(self.session_dir),
            "started_at": self.start_wall_time.isoformat(),
            "updated_at": datetime.now().isoformat(),
            "finished": final,
            "parameters": self.params,
            "sample_count": self.sample_count,
            "image_count": self.image_count,
            "skipped_missing_motor": self.skipped_missing_motor,
            "skipped_rate_limit": self.skipped_rate_limit,
            "skipped_stopped": self.skipped_stopped,
            "skipped_disk_limit": self.skipped_disk_limit,
            "warnings": self.warnings,
            "safety": {
                "publishes_xycar_motor": False,
                "note": "This recorder never creates a /xycar_motor publisher.",
            },
        }
        atomic_write_json(self.session_dir / "metadata.json", metadata)

    def close(self) -> None:
        self.samples_csv_handle.flush()
        self.samples_csv_handle.close()
        if self.params["write_metadata"]:
            self._write_metadata(final=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DatasetRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
