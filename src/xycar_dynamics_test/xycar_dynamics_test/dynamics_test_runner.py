import csv
from dataclasses import dataclass
from datetime import datetime
import math
import os
import time
from typing import Dict, List, Optional

from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String


@dataclass
class Phase:
    name: str
    angle_cmd: float
    speed_cmd: float
    duration_sec: float
    kind: str


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _fmt_cmd(value: float) -> str:
    rounded = round(float(value), 3)
    text = f"{rounded:g}"
    return text.replace("-", "m").replace("+", "p").replace(".", "p")


def _parse_float_list(value) -> List[float]:
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return []
        if text.startswith("[") and text.endswith("]"):
            text = text[1:-1]
        return [float(part.strip()) for part in text.split(",") if part.strip()]
    if isinstance(value, (list, tuple)):
        return [float(item) for item in value]
    return [float(value)]


def _parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y", "on")
    return bool(value)


class DynamicsTestRunner(Node):
    FIELDNAMES = [
        "wall_time",
        "ros_time",
        "phase",
        "phase_elapsed",
        "target_angle_cmd",
        "target_speed_cmd",
        "published_angle_cmd",
        "published_speed_cmd",
        "imu_stamp",
        "yaw_rate_z_rad_s",
        "accel_x_mps2",
        "accel_y_mps2",
        "odom_stamp",
        "odom_speed_mps",
        "odom_yaw_rate_rad_s",
        "estimated_speed_mps",
        "estimated_turn_radius_m",
        "event",
    ]

    def __init__(self) -> None:
        super().__init__("xycar_dynamics_test_runner")

        self._declare_parameters()
        self.motor_topic = str(self.get_parameter("motor_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.use_odom = _parse_bool(self.get_parameter("use_odom").value)
        self.dry_run = _parse_bool(self.get_parameter("dry_run").value)
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.test_name = str(self.get_parameter("test_name").value).strip().lower()
        self.output_dir = os.path.expanduser(str(self.get_parameter("output_dir").value))
        self.speed_gain = float(self.get_parameter("speed_gain_mps_per_cmd").value)
        self.min_response_eval_sec = float(self.get_parameter("min_response_eval_sec").value)
        self.accel_threshold = float(self.get_parameter("accel_threshold_mps2").value)
        self.yaw_rate_threshold = float(self.get_parameter("yaw_rate_threshold_rad_s").value)
        self.odom_speed_threshold = float(self.get_parameter("odom_speed_threshold_mps").value)
        self.sensor_timeout_sec = float(self.get_parameter("sensor_timeout_sec").value)

        self.motor_pub = self.create_publisher(Float32MultiArray, self.motor_topic, 10)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self._on_imu, qos_profile_sensor_data
        )
        self.manual_event_sub = self.create_subscription(
            String,
            "/xycar_dynamics_test/manual_event",
            self._on_manual_event,
            10,
        )
        self.odom_sub = None
        if self.use_odom:
            self.odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data
            )

        self.latest_imu: Dict[str, Optional[float]] = {}
        self.latest_odom: Dict[str, Optional[float]] = {}
        self.pending_event = ""
        self.active_phase_idx = -1
        self.phase_start_wall = time.monotonic()
        self.test_start_wall = self.phase_start_wall
        self.baseline = {}
        self.response_logged = False
        self.finished = False
        self.last_target = (0.0, 0.0)
        self.last_published = (0.0, 0.0)

        self.phases = self._make_phases()
        if not self.phases:
            raise RuntimeError(f"Unknown test_name: {self.test_name}")

        self.csv_file = self._open_csv_file()
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.FIELDNAMES)
        self.csv_writer.writeheader()

        timer_period = 1.0 / max(self.command_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._on_timer)

        mode_text = "DRY RUN: motor commands are not published" if self.dry_run else "LIVE: publishing /xycar_motor"
        self.get_logger().warning(mode_text)
        self.get_logger().info(f"test={self.test_name}, phases={len(self.phases)}, log={self.csv_file.name}")
        self.get_logger().info(
            "manual marker: ros2 topic pub --once /xycar_dynamics_test/manual_event "
            "std_msgs/msg/String \"{data: observed_response}\""
        )

    def _declare_parameters(self) -> None:
        dynamic_parameter = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("use_odom", False)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("output_dir", "~/xycar_dynamics_logs")
        self.declare_parameter("test_name", "speed_step")
        self.declare_parameter("warmup_sec", 2.0)
        self.declare_parameter("settle_sec", 1.5)
        self.declare_parameter("hold_sec", 3.0)
        self.declare_parameter("stop_sec", 2.0)
        self.declare_parameter("speed_commands", "5,10,15,20", dynamic_parameter)
        self.declare_parameter(
            "angle_commands", "10,-10,20,-20,30,-30,40,-40", dynamic_parameter
        )
        self.declare_parameter(
            "turn_angle_commands", "20,-20,30,-30,40,-40", dynamic_parameter
        )
        self.declare_parameter("steer_step_speed_cmd", 6.0)
        self.declare_parameter("turn_speed_cmd", 8.0)
        self.declare_parameter("speed_gain_mps_per_cmd", 0.08)
        self.declare_parameter("wheelbase_m", 0.32)
        self.declare_parameter("min_response_eval_sec", 0.10)
        self.declare_parameter("accel_threshold_mps2", 0.35)
        self.declare_parameter("yaw_rate_threshold_rad_s", 0.08)
        self.declare_parameter("odom_speed_threshold_mps", 0.05)
        self.declare_parameter("sensor_timeout_sec", 0.50)

    def _make_phases(self) -> List[Phase]:
        warmup_sec = float(self.get_parameter("warmup_sec").value)
        settle_sec = float(self.get_parameter("settle_sec").value)
        hold_sec = float(self.get_parameter("hold_sec").value)
        stop_sec = float(self.get_parameter("stop_sec").value)
        speed_commands = _parse_float_list(self.get_parameter("speed_commands").value)
        angle_commands = _parse_float_list(self.get_parameter("angle_commands").value)
        turn_angle_commands = _parse_float_list(self.get_parameter("turn_angle_commands").value)
        steer_speed = float(self.get_parameter("steer_step_speed_cmd").value)
        turn_speed = float(self.get_parameter("turn_speed_cmd").value)

        phases = [Phase("warmup_zero", 0.0, 0.0, warmup_sec, "idle")]

        if self.test_name in ("speed_step", "all"):
            for speed_cmd in speed_commands:
                phases.append(Phase("settle_before_speed", 0.0, 0.0, settle_sec, "idle"))
                phases.append(
                    Phase(
                        f"speed_cmd_{_fmt_cmd(speed_cmd)}",
                        0.0,
                        speed_cmd,
                        hold_sec,
                        "speed",
                    )
                )
                phases.append(Phase("stop_after_speed", 0.0, 0.0, stop_sec, "idle"))

        if self.test_name in ("steer_step", "all"):
            for angle_cmd in angle_commands:
                phases.append(
                    Phase("straight_before_steer", 0.0, steer_speed, settle_sec, "idle")
                )
                phases.append(
                    Phase(
                        f"steer_cmd_{_fmt_cmd(angle_cmd)}",
                        angle_cmd,
                        steer_speed,
                        hold_sec,
                        "steer",
                    )
                )
                phases.append(
                    Phase("straight_after_steer", 0.0, steer_speed, settle_sec, "idle")
                )
            phases.append(Phase("stop_after_steer", 0.0, 0.0, stop_sec, "idle"))

        if self.test_name in ("turn_radius", "all"):
            for angle_cmd in turn_angle_commands:
                phases.append(
                    Phase("straight_before_turn", 0.0, turn_speed, settle_sec, "idle")
                )
                phases.append(
                    Phase(
                        f"turn_angle_{_fmt_cmd(angle_cmd)}",
                        angle_cmd,
                        turn_speed,
                        hold_sec,
                        "turn",
                    )
                )
                phases.append(Phase("stop_after_turn", 0.0, 0.0, stop_sec, "idle"))

        phases.append(Phase("final_stop", 0.0, 0.0, stop_sec, "idle"))
        return phases if len(phases) > 2 else []

    def _open_csv_file(self):
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.output_dir, f"{timestamp}_{self.test_name}.csv")
        return open(path, "w", newline="", encoding="utf-8")

    def _on_imu(self, msg: Imu) -> None:
        self.latest_imu = {
            "received_wall": time.monotonic(),
            "stamp": _stamp_to_float(msg.header.stamp),
            "yaw_rate_z": float(msg.angular_velocity.z),
            "accel_x": float(msg.linear_acceleration.x),
            "accel_y": float(msg.linear_acceleration.y),
        }

    def _on_odom(self, msg: Odometry) -> None:
        linear = msg.twist.twist.linear
        self.latest_odom = {
            "received_wall": time.monotonic(),
            "stamp": _stamp_to_float(msg.header.stamp),
            "speed": math.hypot(float(linear.x), float(linear.y)),
            "yaw_rate": float(msg.twist.twist.angular.z),
        }

    def _on_manual_event(self, msg: String) -> None:
        phase_elapsed = time.monotonic() - self.phase_start_wall
        label = msg.data.strip() or "manual_event"
        self.pending_event = f"manual:{label};delay={phase_elapsed:.3f}s"
        self.get_logger().info(self.pending_event)

    def _on_timer(self) -> None:
        elapsed = time.monotonic() - self.test_start_wall
        phase_idx, phase, phase_elapsed = self._phase_at(elapsed)
        if phase is None:
            self.last_target = (0.0, 0.0)
            self._publish_command(0.0, 0.0)
            self.finished = True
            self._flush_close()
            self.get_logger().info(f"test complete: {self.csv_file.name}")
            return

        if phase_idx != self.active_phase_idx:
            self._start_phase(phase_idx, phase)
            phase_elapsed = 0.0

        self.last_target = (phase.angle_cmd, phase.speed_cmd)
        self._publish_command(phase.angle_cmd, phase.speed_cmd)
        event = self._detect_response_event(phase, phase_elapsed)
        if self.pending_event:
            event = f"{event};{self.pending_event}" if event else self.pending_event
            self.pending_event = ""
        self._write_row(phase, phase_elapsed, event)

    def _phase_at(self, elapsed: float):
        cursor = 0.0
        for idx, phase in enumerate(self.phases):
            next_cursor = cursor + phase.duration_sec
            if elapsed < next_cursor:
                return idx, phase, elapsed - cursor
            cursor = next_cursor
        return len(self.phases), None, 0.0

    def _start_phase(self, phase_idx: int, phase: Phase) -> None:
        self.active_phase_idx = phase_idx
        self.phase_start_wall = time.monotonic()
        self.baseline = {
            "yaw_rate_z": self.latest_imu.get("yaw_rate_z"),
            "accel_x": self.latest_imu.get("accel_x"),
            "odom_speed": self.latest_odom.get("speed"),
            "odom_yaw_rate": self.latest_odom.get("yaw_rate"),
        }
        self.response_logged = False
        self.get_logger().info(
            f"phase {phase_idx + 1}/{len(self.phases)}: {phase.name} "
            f"angle={phase.angle_cmd:g}, speed={phase.speed_cmd:g}"
        )

    def _publish_command(self, angle_cmd: float, speed_cmd: float) -> None:
        if self.dry_run:
            self.last_published = (0.0, 0.0)
            return
        msg = Float32MultiArray()
        msg.data = [float(angle_cmd), float(speed_cmd)]
        self.motor_pub.publish(msg)
        self.last_published = (float(angle_cmd), float(speed_cmd))

    def _detect_response_event(self, phase: Phase, phase_elapsed: float) -> str:
        if self.response_logged or phase.kind not in ("speed", "steer", "turn"):
            return ""
        if phase_elapsed < self.min_response_eval_sec:
            return ""

        if phase.kind == "speed":
            if self._fresh_odom():
                baseline_speed = self.baseline.get("odom_speed")
                odom_speed = self.latest_odom.get("speed")
                if baseline_speed is not None and odom_speed is not None:
                    if abs(odom_speed - baseline_speed) >= self.odom_speed_threshold:
                        self.response_logged = True
                        return f"auto_speed_response_delay={phase_elapsed:.3f}s"
            baseline_accel = self.baseline.get("accel_x")
            accel_x = self.latest_imu.get("accel_x")
            if baseline_accel is not None and accel_x is not None:
                if abs(accel_x - baseline_accel) >= self.accel_threshold:
                    self.response_logged = True
                    return f"auto_speed_response_delay={phase_elapsed:.3f}s"

        if phase.kind in ("steer", "turn"):
            baseline_yaw = self.baseline.get("yaw_rate_z")
            yaw_rate = self.latest_imu.get("yaw_rate_z")
            if baseline_yaw is not None and yaw_rate is not None:
                if abs(yaw_rate - baseline_yaw) >= self.yaw_rate_threshold:
                    self.response_logged = True
                    return f"auto_steering_response_delay={phase_elapsed:.3f}s"

        return ""

    def _write_row(self, phase: Phase, phase_elapsed: float, event: str) -> None:
        now_wall = time.time()
        ros_time = self.get_clock().now().nanoseconds * 1e-9
        estimated_speed = self._estimated_speed_mps(phase.speed_cmd)
        yaw_rate = self._best_yaw_rate()
        turn_radius = ""
        if yaw_rate is not None and abs(yaw_rate) > 1e-3 and abs(estimated_speed) > 1e-3:
            turn_radius = abs(estimated_speed / yaw_rate)

        row = {
            "wall_time": f"{now_wall:.6f}",
            "ros_time": f"{ros_time:.6f}",
            "phase": phase.name,
            "phase_elapsed": f"{phase_elapsed:.6f}",
            "target_angle_cmd": f"{phase.angle_cmd:.6f}",
            "target_speed_cmd": f"{phase.speed_cmd:.6f}",
            "published_angle_cmd": f"{self.last_published[0]:.6f}",
            "published_speed_cmd": f"{self.last_published[1]:.6f}",
            "imu_stamp": self._fmt_optional(self.latest_imu.get("stamp")),
            "yaw_rate_z_rad_s": self._fmt_optional(self.latest_imu.get("yaw_rate_z")),
            "accel_x_mps2": self._fmt_optional(self.latest_imu.get("accel_x")),
            "accel_y_mps2": self._fmt_optional(self.latest_imu.get("accel_y")),
            "odom_stamp": self._fmt_optional(self.latest_odom.get("stamp")),
            "odom_speed_mps": self._fmt_optional(self.latest_odom.get("speed")),
            "odom_yaw_rate_rad_s": self._fmt_optional(self.latest_odom.get("yaw_rate")),
            "estimated_speed_mps": f"{estimated_speed:.6f}",
            "estimated_turn_radius_m": self._fmt_optional(turn_radius),
            "event": event,
        }
        self.csv_writer.writerow(row)

    def _fmt_optional(self, value) -> str:
        if value is None or value == "":
            return ""
        return f"{float(value):.6f}"

    def _fresh_odom(self) -> bool:
        received_wall = self.latest_odom.get("received_wall")
        if received_wall is None:
            return False
        return (time.monotonic() - received_wall) <= self.sensor_timeout_sec

    def _estimated_speed_mps(self, speed_cmd: float) -> float:
        if self.use_odom and self._fresh_odom():
            odom_speed = self.latest_odom.get("speed")
            if odom_speed is not None:
                return float(odom_speed)
        return abs(float(speed_cmd)) * self.speed_gain

    def _best_yaw_rate(self) -> Optional[float]:
        if self.use_odom and self._fresh_odom():
            odom_yaw = self.latest_odom.get("yaw_rate")
            if odom_yaw is not None:
                return float(odom_yaw)
        yaw = self.latest_imu.get("yaw_rate_z")
        return None if yaw is None else float(yaw)

    def _flush_close(self) -> None:
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

    def safe_stop(self) -> None:
        if self.dry_run:
            return
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0]
        for _ in range(5):
            self.motor_pub.publish(msg)
            time.sleep(0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicsTestRunner()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warning("interrupted; sending zero motor command")
    finally:
        node.safe_stop()
        node._flush_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
