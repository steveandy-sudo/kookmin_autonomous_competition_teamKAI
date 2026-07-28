"""Interactively measure real IMU yaw sign, scale, and continuity."""

from __future__ import annotations

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String

from .imu_yaw_calibration_core import (
    YawTracker,
    marker_error_limit_deg,
    quaternion_yaw,
    summarize_markers,
)


HELP_TEXT = """
Xycar IMU yaw calibration (motor output is never published)

  0 : set the current body direction as zero
  c : mark a return to zero
  1 : mark left / counter-clockwise  +90 deg
  2 : mark right / clockwise         -90 deg
  3 : mark left / counter-clockwise +180 deg
  4 : mark right / clockwise        -180 deg
  5 : mark left / counter-clockwise +360 deg
  6 : mark right / clockwise        -360 deg
  q : write the report and quit

Hold the vehicle still for at least two seconds before pressing a marker key.
"""


KEY_MARKERS = {
    "c": ("RETURN_ZERO", 0.0),
    "1": ("LEFT_90", 90.0),
    "2": ("RIGHT_90", -90.0),
    "3": ("LEFT_180", 180.0),
    "4": ("RIGHT_180", -180.0),
    "5": ("LEFT_360", 360.0),
    "6": ("RIGHT_360", -360.0),
}


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class ImuYawCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("imu_yaw_calibrator")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter(
            "output_root",
            str(Path.home() / "xycar_test_results" / "imu_yaw"),
        )
        self.declare_parameter("session_name", "")
        self.declare_parameter("marker_average_sec", 2.0)
        self.declare_parameter("status_period_sec", 0.5)

        session_name = str(self.get_parameter("session_name").value)
        if not session_name:
            session_name = datetime.now().strftime(
                "imu_yaw_%Y%m%d_%H%M%S"
            )
        self.output_dir = (
            Path(str(self.get_parameter("output_root").value)).expanduser()
            / session_name
        )
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.samples_path = self.output_dir / "imu_samples.csv"
        self.markers_path = self.output_dir / "imu_markers.csv"
        self.report_path = self.output_dir / "imu_yaw_report.json"

        self.samples_file = self.samples_path.open(
            "w", newline="", encoding="utf-8"
        )
        self.samples_writer = csv.writer(self.samples_file)
        self.samples_writer.writerow(
            [
                "stamp_sec",
                "wrapped_yaw_deg",
                "unwrapped_yaw_deg",
                "relative_yaw_deg",
                "gyro_z_deg_s",
                "orientation_covariance_zz",
            ]
        )
        self.markers: list[dict] = []
        self.tracker = YawTracker(
            averaging_window_sec=float(
                self.get_parameter("marker_average_sec").value
            )
        )
        self.latest_sample = None
        self.quit_requested = False
        self.report_written = False
        self.flush_counter = 0

        self.stdin_is_tty = sys.stdin.isatty()
        self.original_terminal_settings = None
        if self.stdin_is_tty:
            self.original_terminal_settings = termios.tcgetattr(
                sys.stdin
            )
            tty.setcbreak(sys.stdin.fileno())
        else:
            self.get_logger().warning(
                "stdin is not a TTY; keyboard markers are unavailable"
            )

        self.marker_publisher = self.create_publisher(
            String, "/imu_calibration/marker", 10
        )
        self.yaw_publisher = self.create_publisher(
            Float64MultiArray, "/imu_calibration/yaw_deg", 10
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_timer(0.02, self._on_keyboard)
        self.create_timer(
            max(
                0.1,
                float(self.get_parameter("status_period_sec").value),
            ),
            self._print_status,
        )
        self.get_logger().info(HELP_TEXT)
        self.get_logger().info(f"Output directory: {self.output_dir}")

    def _on_imu(self, message: Imu) -> None:
        q = message.orientation
        wrapped_yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
        self.latest_sample = self.tracker.update(
            _stamp_seconds(message.header.stamp),
            wrapped_yaw,
            float(message.angular_velocity.z),
        )
        covariance_zz = (
            float(message.orientation_covariance[8])
            if len(message.orientation_covariance) >= 9
            else math.nan
        )
        self.samples_writer.writerow(
            [
                f"{self.latest_sample.stamp_sec:.9f}",
                f"{math.degrees(self.latest_sample.wrapped_rad):.9f}",
                f"{math.degrees(self.latest_sample.unwrapped_rad):.9f}",
                f"{math.degrees(self.latest_sample.relative_rad):.9f}",
                f"{math.degrees(self.latest_sample.gyro_z_rad_s):.9f}",
                f"{covariance_zz:.9f}",
            ]
        )
        self.flush_counter += 1
        if self.flush_counter >= 100:
            self.samples_file.flush()
            self.flush_counter = 0

        output = Float64MultiArray()
        output.data = [
            math.degrees(self.latest_sample.wrapped_rad),
            math.degrees(self.latest_sample.unwrapped_rad),
            math.degrees(self.latest_sample.relative_rad),
            math.degrees(self.latest_sample.gyro_z_rad_s),
        ]
        self.yaw_publisher.publish(output)

    def _read_key(self) -> str | None:
        if not self.stdin_is_tty:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        return sys.stdin.read(1)

    def _on_keyboard(self) -> None:
        while True:
            key = self._read_key()
            if key is None:
                return
            if key == "0":
                self._set_zero()
            elif key in KEY_MARKERS:
                label, expected_deg = KEY_MARKERS[key]
                self._add_marker(label, expected_deg)
            elif key in {"q", "\x03"}:
                self.quit_requested = True
                self.write_report()
                raise KeyboardInterrupt

    def _set_zero(self) -> None:
        if self.latest_sample is None:
            self.get_logger().warning(
                "Cannot set zero before the first IMU sample"
            )
            return
        self.tracker.set_zero()
        self._add_marker("ZERO", 0.0, force_measurement=0.0)
        self.get_logger().info(
            "Yaw zero set. Hold still, then start the first rotation."
        )

    def _add_marker(
        self,
        label: str,
        expected_deg: float,
        force_measurement: float | None = None,
    ) -> None:
        if self.latest_sample is None:
            self.get_logger().warning(
                "Cannot add a marker before the first IMU sample"
            )
            return
        measured_deg, standard_deviation_deg, sample_count = (
            self.tracker.marker_measurement()
        )
        if force_measurement is not None:
            measured_deg = force_measurement
            standard_deviation_deg = 0.0
        marker = {
            "index": len(self.markers) + 1,
            "label": label,
            "stamp_sec": self.latest_sample.stamp_sec,
            "expected_deg": float(expected_deg),
            "measured_deg": float(measured_deg),
            "error_deg": float(measured_deg - expected_deg),
            "absolute_error_deg": float(
                abs(measured_deg - expected_deg)
            ),
            "limit_deg": marker_error_limit_deg(expected_deg),
            "stable_standard_deviation_deg": float(
                standard_deviation_deg
            ),
            "averaged_sample_count": int(sample_count),
        }
        marker["passed"] = (
            marker["absolute_error_deg"] <= marker["limit_deg"]
        )
        self.markers.append(marker)
        marker_message = String()
        marker_message.data = json.dumps(marker, sort_keys=True)
        self.marker_publisher.publish(marker_message)
        self.samples_file.flush()
        self.get_logger().info(
            f"{label}: expected={expected_deg:+.1f} deg, "
            f"measured={measured_deg:+.2f} deg, "
            f"error={marker['error_deg']:+.2f} deg, "
            f"stable_std={standard_deviation_deg:.2f} deg"
        )

    def _print_status(self) -> None:
        if self.latest_sample is None:
            self.get_logger().warning(
                "Waiting for /imu...",
                throttle_duration_sec=2.0,
            )
            return
        measured_deg, standard_deviation_deg, _ = (
            self.tracker.marker_measurement()
        )
        self.get_logger().info(
            f"yaw relative={math.degrees(self.latest_sample.relative_rad):+8.2f} "
            f"deg | stable={measured_deg:+8.2f} "
            f"+/-{standard_deviation_deg:.2f} deg | "
            f"gyro_z={math.degrees(self.latest_sample.gyro_z_rad_s):+7.2f} "
            f"deg/s | rate={self.tracker.sample_rate_hz:.1f} Hz"
        )

    def write_report(self) -> None:
        if self.report_written:
            return
        self.samples_file.flush()
        with self.markers_path.open(
            "w", newline="", encoding="utf-8"
        ) as marker_file:
            fieldnames = list(self.markers[0].keys()) if self.markers else []
            if fieldnames:
                writer = csv.DictWriter(
                    marker_file, fieldnames=fieldnames
                )
                writer.writeheader()
                writer.writerows(self.markers)

        marker_summary = summarize_markers(self.markers)
        report = {
            "output_directory": str(self.output_dir),
            "imu_topic": str(self.get_parameter("imu_topic").value),
            "sample_count": self.tracker.sample_count,
            "sample_rate_hz": self.tracker.sample_rate_hz,
            "maximum_message_gap_sec": self.tracker.maximum_gap_sec,
            "marker_average_sec": self.tracker.averaging_window_sec,
            "markers": self.markers,
            **marker_summary,
        }
        self.report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        self.report_written = True
        self.get_logger().info(
            f"Calibration report written: {self.report_path}"
        )

    def close(self) -> None:
        self.write_report()
        self.samples_file.close()
        if self.original_terminal_settings is not None:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.original_terminal_settings,
            )
            self.original_terminal_settings = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuYawCalibrator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
