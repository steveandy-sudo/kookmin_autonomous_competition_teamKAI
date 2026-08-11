from __future__ import annotations

import csv
import math
import statistics
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray, String


@dataclass
class CalibrationSample:
    stamp_sec: float
    speed_mps: float
    angle_cmd: float
    speed_cmd: float
    steering_rad: float
    yaw_rate_rps: float
    wheel_base_m: Optional[float]
    steering_gain_rad_per_cmd: Optional[float]


class ImuVehicleSpecCalibrator(Node):
    """Estimate sim-to-real vehicle parameters from IMU yaw rate and motor commands."""

    def __init__(self) -> None:
        super().__init__("imu_vehicle_spec_calibrator")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter("bridge_debug_topic", "/xycar_motor_bridge/debug")
        self.declare_parameter("use_bridge_debug", True)
        self.declare_parameter("bridge_debug_timeout_sec", 0.30)
        self.declare_parameter("speed_gain_mps_per_cmd", 0.08)
        self.declare_parameter("steering_gain_rad_per_cmd", -0.0068)
        self.declare_parameter("known_wheel_base_m", 0.32)
        self.declare_parameter("min_speed_mps", 0.05)
        self.declare_parameter("min_abs_angle_cmd", 2.0)
        self.declare_parameter("min_abs_yaw_rate_dps", 1.0)
        self.declare_parameter("max_abs_yaw_rate_dps", 360.0)
        self.declare_parameter("window_size", 200)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("csv_path", "")

        self.samples: Deque[CalibrationSample] = deque(
            maxlen=max(int(self.get_parameter("window_size").value), 10)
        )
        self.last_motor_cmd: Optional[tuple[float, float]] = None
        self.last_bridge_debug: Optional[tuple[float, float, float, float, float]] = None
        self.last_bridge_debug_sec = 0.0
        self.csv_file = None
        self.csv_writer = None

        csv_path = str(self.get_parameter("csv_path").value)
        if csv_path:
            path = Path(csv_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = path.open("w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "stamp_sec",
                "speed_mps",
                "angle_cmd",
                "speed_cmd",
                "steering_rad",
                "yaw_rate_rps",
                "wheel_base_m",
                "steering_gain_rad_per_cmd",
            ])

        imu_topic = str(self.get_parameter("imu_topic").value)
        motor_topic = str(self.get_parameter("motor_topic").value)
        bridge_debug_topic = str(self.get_parameter("bridge_debug_topic").value)
        self.create_subscription(Imu, imu_topic, self.on_imu, 50)
        self.create_subscription(Float32MultiArray, motor_topic, self.on_motor, 20)
        self.create_subscription(Float32MultiArray, bridge_debug_topic, self.on_bridge_debug, 20)

        self.summary_pub = self.create_publisher(String, "/xycar_imu_tools/vehicle_spec_estimate", 10)
        self.wheel_base_pub = self.create_publisher(Float32, "/xycar_imu_tools/effective_wheel_base_m", 10)
        self.steering_gain_pub = self.create_publisher(Float32, "/xycar_imu_tools/steering_gain_rad_per_cmd", 10)

        rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.2)
        self.create_timer(1.0 / rate_hz, self.publish_summary)
        self.get_logger().info(
            f"IMU vehicle spec calibrator ready | imu={imu_topic}, motor={motor_topic}, bridge_debug={bridge_debug_topic}"
        )

    def on_motor(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        self.last_motor_cmd = (float(msg.data[0]), float(msg.data[1]))

    def on_bridge_debug(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 5:
            return
        self.last_bridge_debug = tuple(float(v) for v in msg.data[:5])
        self.last_bridge_debug_sec = time.monotonic()

    def on_imu(self, msg: Imu) -> None:
        command = self._current_command_state()
        if command is None:
            return
        angle_cmd, speed_cmd, steering_rad, speed_mps = command
        yaw_rate = float(msg.angular_velocity.z)
        if not self._sample_is_useful(speed_mps, angle_cmd, steering_rad, yaw_rate):
            return

        wheel_base = self._estimate_wheel_base(speed_mps, steering_rad, yaw_rate)
        steering_gain = self._estimate_steering_gain(speed_mps, angle_cmd, yaw_rate)
        sample = CalibrationSample(
            stamp_sec=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            speed_mps=speed_mps,
            angle_cmd=angle_cmd,
            speed_cmd=speed_cmd,
            steering_rad=steering_rad,
            yaw_rate_rps=yaw_rate,
            wheel_base_m=wheel_base,
            steering_gain_rad_per_cmd=steering_gain,
        )
        self.samples.append(sample)
        self._write_csv(sample)

    def _current_command_state(self) -> Optional[tuple[float, float, float, float]]:
        now = time.monotonic()
        if (
            bool(self.get_parameter("use_bridge_debug").value)
            and self.last_bridge_debug is not None
            and now - self.last_bridge_debug_sec <= float(self.get_parameter("bridge_debug_timeout_sec").value)
        ):
            angle_cmd, speed_cmd, steering_rad, speed_mps, _yaw_rate = self.last_bridge_debug
            return angle_cmd, speed_cmd, steering_rad, abs(speed_mps)

        if self.last_motor_cmd is None:
            return None
        angle_cmd, speed_cmd = self.last_motor_cmd
        steering_rad = angle_cmd * float(self.get_parameter("steering_gain_rad_per_cmd").value)
        speed_mps = abs(speed_cmd * float(self.get_parameter("speed_gain_mps_per_cmd").value))
        return angle_cmd, speed_cmd, steering_rad, speed_mps

    def _sample_is_useful(
        self,
        speed_mps: float,
        angle_cmd: float,
        steering_rad: float,
        yaw_rate: float,
    ) -> bool:
        if speed_mps < float(self.get_parameter("min_speed_mps").value):
            return False
        if abs(angle_cmd) < float(self.get_parameter("min_abs_angle_cmd").value):
            return False
        if abs(math.tan(steering_rad)) < 1.0e-4:
            return False
        yaw_rate_dps = abs(math.degrees(yaw_rate))
        if yaw_rate_dps < float(self.get_parameter("min_abs_yaw_rate_dps").value):
            return False
        if yaw_rate_dps > float(self.get_parameter("max_abs_yaw_rate_dps").value):
            return False
        return True

    @staticmethod
    def _estimate_wheel_base(speed_mps: float, steering_rad: float, yaw_rate: float) -> Optional[float]:
        if abs(yaw_rate) < 1.0e-6:
            return None
        wheel_base = speed_mps * math.tan(steering_rad) / yaw_rate
        return abs(wheel_base) if math.isfinite(wheel_base) else None

    def _estimate_steering_gain(self, speed_mps: float, angle_cmd: float, yaw_rate: float) -> Optional[float]:
        if speed_mps < 1.0e-6 or abs(angle_cmd) < 1.0e-6:
            return None
        wheel_base = max(float(self.get_parameter("known_wheel_base_m").value), 1.0e-4)
        needed_steering = math.atan2(yaw_rate * wheel_base, speed_mps)
        gain = needed_steering / angle_cmd
        return gain if math.isfinite(gain) else None

    def _write_csv(self, sample: CalibrationSample) -> None:
        if self.csv_writer is None:
            return
        self.csv_writer.writerow([
            sample.stamp_sec,
            sample.speed_mps,
            sample.angle_cmd,
            sample.speed_cmd,
            sample.steering_rad,
            sample.yaw_rate_rps,
            "" if sample.wheel_base_m is None else sample.wheel_base_m,
            "" if sample.steering_gain_rad_per_cmd is None else sample.steering_gain_rad_per_cmd,
        ])
        self.csv_file.flush()

    def publish_summary(self) -> None:
        if not self.samples:
            self.summary_pub.publish(String(data="waiting_for_motion_samples"))
            return

        wheel_bases = [s.wheel_base_m for s in self.samples if s.wheel_base_m is not None]
        steering_gains = [
            s.steering_gain_rad_per_cmd
            for s in self.samples
            if s.steering_gain_rad_per_cmd is not None
        ]
        if not wheel_bases:
            return

        wheel_base_median = statistics.median(wheel_bases)
        wheel_base_mean = statistics.fmean(wheel_bases)
        steering_text = "n/a"
        steering_gain_median = None
        if steering_gains:
            steering_gain_median = statistics.median(steering_gains)
            steering_text = f"{steering_gain_median:.6f}"

        text = (
            f"samples={len(self.samples)} "
            f"wheel_base_m median={wheel_base_median:.4f} mean={wheel_base_mean:.4f} "
            f"steering_gain_rad_per_cmd median={steering_text}"
        )
        self.summary_pub.publish(String(data=text))
        self.wheel_base_pub.publish(Float32(data=float(wheel_base_median)))
        if steering_gain_median is not None:
            self.steering_gain_pub.publish(Float32(data=float(steering_gain_median)))
        self.get_logger().info(text)

    def destroy_node(self) -> bool:
        if self.csv_file is not None:
            self.csv_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuVehicleSpecCalibrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
