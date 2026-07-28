"""Native ROS 2 Xycar motor bridge and VESC serial driver."""

from __future__ import annotations

import math
import os
from pathlib import Path
import queue
import shutil
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from xycar_msgs.msg import XycarSystemTelemetry, XycarVescState

from .protocol import (
    COMM_FW_VERSION,
    COMM_GET_VALUES,
    FrameParser,
    decode_values,
    request_firmware_version,
    request_values,
    set_rpm,
    set_servo,
)
from .safety import LATCHED, LIMITED, VoltageGuard, slew


FAULT_NAMES = {
    0: "NONE",
    1: "OVER_VOLTAGE",
    2: "UNDER_VOLTAGE",
    3: "DRV8302",
    4: "ABS_OVER_CURRENT",
    5: "OVER_TEMP_FET",
    6: "OVER_TEMP_MOTOR",
}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def read_cpu_times() -> tuple[int, int]:
    fields = Path("/proc/stat").read_text(encoding="ascii").splitlines()[0].split()
    values = [int(value) for value in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def read_memory() -> tuple[float, float]:
    values: dict[str, float] = {}
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        key, raw = line.split(":", 1)
        values[key] = float(raw.strip().split()[0])
    total_kb = values.get("MemTotal", 0.0)
    available_kb = values.get("MemAvailable", 0.0)
    used_percent = (
        100.0 * (total_kb - available_kb) / total_kb
        if total_kb > 0.0
        else math.nan
    )
    return used_percent, available_kb / 1024.0


def read_cpu_temperature_c() -> float:
    temperatures = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            value = float(path.read_text(encoding="ascii").strip()) / 1000.0
        except (OSError, ValueError):
            continue
        if -20.0 <= value <= 150.0:
            temperatures.append(value)
    return max(temperatures) if temperatures else math.nan


class XycarVescDriver(Node):
    """Own the VESC serial device and expose the established ROS 2 contract."""

    def __init__(self) -> None:
        super().__init__("xycar_vesc_driver")
        self._declare_parameters()
        self._load_parameters()

        self._vesc_pub = self.create_publisher(
            XycarVescState, self.vesc_state_topic, 20
        )
        self._system_pub = self.create_publisher(
            XycarSystemTelemetry, self.system_telemetry_topic, 10
        )
        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self._debug_pub = self.create_publisher(
            Float32MultiArray, self.debug_topic, 10
        )
        self._diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self._motor_sub = self.create_subscription(
            Float32MultiArray, self.motor_topic, self._on_motor_command, 10
        )
        self._clear_fault_service = self.create_service(
            Trigger, self.clear_fault_service, self._on_clear_fault
        )

        self._serial = None
        self._serial_lock = threading.Lock()
        self._stop_reader = threading.Event()
        self._rx_packets: queue.SimpleQueue[bytes] = queue.SimpleQueue()
        self._parser = FrameParser()
        self._reader_thread = threading.Thread(
            target=self._serial_reader, daemon=True, name="vesc_serial_reader"
        )
        self._last_serial_error = ""
        self._last_serial_error_log = 0.0
        self._firmware_version: tuple[int, int] | None = None

        self._target_angle_cmd = 0.0
        self._target_speed_cmd = 0.0
        self._applied_speed_mps = 0.0
        self._applied_steering_rad = 0.0
        self._last_command_monotonic: float | None = None
        self._last_control_monotonic = time.monotonic()
        self._last_telemetry_monotonic: float | None = None
        self._last_values = None
        self._last_guard_state = ""
        self._last_fault_code = 0

        self._guard = VoltageGuard(
            limit_voltage=self.low_voltage_limit,
            stop_voltage=self.low_voltage_stop,
            recovery_voltage=self.low_voltage_recovery,
            recovery_stable_sec=self.recovery_stable_sec,
            auto_recover=self.auto_recover,
        )

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._last_odom_monotonic: float | None = None
        self._previous_cpu = read_cpu_times()

        self._reader_thread.start()
        self._connect_serial()
        self.create_timer(1.0, self._connect_serial)
        self.create_timer(1.0 / self.telemetry_poll_hz, self._poll_vesc)
        self.create_timer(0.005, self._process_received_packets)
        self.create_timer(1.0 / self.control_hz, self._control_update)
        self.create_timer(1.0, self._publish_system_telemetry)
        self.create_timer(0.5, self._publish_diagnostics)

        self.get_logger().info(
            "Native ROS 2 VESC stack ready: %s -> %s at %d baud; "
            "drive_enabled=%s, watchdog=%.2fs"
            % (
                self.motor_topic,
                self.port,
                self.baud_rate,
                self.drive_enabled,
                self.command_timeout_sec,
            )
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "port": "/dev/ttyMOTOR",
            "baud_rate": 115200,
            "drive_enabled": False,
            "require_telemetry": True,
            "motor_topic": "/xycar_motor",
            "vesc_state_topic": "/vehicle/vesc_state",
            "system_telemetry_topic": "/vehicle/system_telemetry",
            "odom_topic": "/odom",
            "debug_topic": "/xycar_motor_bridge/debug",
            "clear_fault_service": "/vehicle/clear_motor_fault",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "publish_tf": True,
            "angle_command_min": -50.0,
            "angle_command_max": 100.0,
            "speed_command_min": -50.0,
            "speed_command_max": 100.0,
            "angle_to_steering_gain": -0.0068,
            "speed_to_mps_gain": 0.08,
            "speed_to_erpm_gain": 4614.0,
            "speed_to_erpm_offset": 0.0,
            "steering_to_servo_gain": -1.2135,
            "steering_to_servo_offset": 0.5004,
            "servo_min": 0.15,
            "servo_max": 0.85,
            "wheelbase_m": 0.32,
            "control_hz": 20.0,
            "telemetry_poll_hz": 50.0,
            "command_timeout_sec": 0.5,
            "telemetry_timeout_sec": 0.25,
            "acceleration_limit_mps2": 0.3,
            "deceleration_limit_mps2": 1.5,
            "low_voltage_limit": 7.5,
            "low_voltage_stop": 6.0,
            "low_voltage_recovery": 8.0,
            "recovery_stable_sec": 2.0,
            "auto_recover": False,
            "disk_path": str(Path.home()),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        names = (
            "port",
            "baud_rate",
            "drive_enabled",
            "require_telemetry",
            "motor_topic",
            "vesc_state_topic",
            "system_telemetry_topic",
            "odom_topic",
            "debug_topic",
            "clear_fault_service",
            "odom_frame",
            "base_frame",
            "publish_tf",
            "angle_command_min",
            "angle_command_max",
            "speed_command_min",
            "speed_command_max",
            "angle_to_steering_gain",
            "speed_to_mps_gain",
            "speed_to_erpm_gain",
            "speed_to_erpm_offset",
            "steering_to_servo_gain",
            "steering_to_servo_offset",
            "servo_min",
            "servo_max",
            "wheelbase_m",
            "control_hz",
            "telemetry_poll_hz",
            "command_timeout_sec",
            "telemetry_timeout_sec",
            "acceleration_limit_mps2",
            "deceleration_limit_mps2",
            "low_voltage_limit",
            "low_voltage_stop",
            "low_voltage_recovery",
            "recovery_stable_sec",
            "auto_recover",
            "disk_path",
        )
        for name in names:
            setattr(self, name, self.get_parameter(name).value)
        self.control_hz = max(1.0, float(self.control_hz))
        self.telemetry_poll_hz = max(1.0, float(self.telemetry_poll_hz))
        self.command_timeout_sec = max(0.05, float(self.command_timeout_sec))
        self.telemetry_timeout_sec = max(
            0.05, float(self.telemetry_timeout_sec)
        )

    def _connect_serial(self) -> None:
        with self._serial_lock:
            if self._serial is not None and self._serial.is_open:
                return
        try:
            device = serial.Serial(
                port=str(self.port),
                baudrate=int(self.baud_rate),
                timeout=0.02,
                write_timeout=0.1,
            )
        except (OSError, serial.SerialException) as exc:
            self._record_serial_error(str(exc))
            return
        with self._serial_lock:
            if self._serial is None:
                self._serial = device
                self._firmware_version = None
                self.get_logger().info(f"Opened VESC serial port: {self.port}")
            else:
                device.close()

    def _record_serial_error(self, message: str) -> None:
        self._last_serial_error = message
        now = time.monotonic()
        if now - self._last_serial_error_log >= 2.0:
            self.get_logger().error(f"VESC serial error: {message}")
            self._last_serial_error_log = now

    def _drop_serial(self, device, message: str) -> None:
        with self._serial_lock:
            if self._serial is device:
                self._serial = None
                self._firmware_version = None
        try:
            device.close()
        except (OSError, TypeError, serial.SerialException):
            pass
        self._record_serial_error(message)

    def _serial_reader(self) -> None:
        while not self._stop_reader.is_set():
            with self._serial_lock:
                device = self._serial
            if device is None:
                self._stop_reader.wait(0.1)
                continue
            try:
                amount = max(1, min(4096, int(device.in_waiting)))
                data = device.read(amount)
            except (OSError, TypeError, serial.SerialException) as exc:
                self._drop_serial(device, str(exc))
                continue
            if not data:
                continue
            for payload in self._parser.feed(data):
                self._rx_packets.put(payload)

    def _write(self, frame: bytes) -> bool:
        with self._serial_lock:
            device = self._serial
            if device is None:
                return False
            try:
                written = device.write(frame)
            except (OSError, serial.SerialException) as exc:
                written = -1
                error = str(exc)
            else:
                error = f"short write: {written}/{len(frame)}"
                if written == len(frame):
                    return True
        self._drop_serial(device, error)
        return False

    def _poll_vesc(self) -> None:
        if self._firmware_version is None:
            self._write(request_firmware_version())
        else:
            self._write(request_values())

    def _process_received_packets(self) -> None:
        for _ in range(100):
            try:
                payload = self._rx_packets.get_nowait()
            except queue.Empty:
                return
            if not payload:
                continue
            if payload[0] == COMM_FW_VERSION and len(payload) >= 3:
                version = (int(payload[1]), int(payload[2]))
                if version != self._firmware_version:
                    self._firmware_version = version
                    self.get_logger().info(
                        f"Connected to VESC firmware {version[0]}.{version[1]}"
                    )
            elif payload[0] == COMM_GET_VALUES:
                try:
                    values = decode_values(payload)
                except ValueError as exc:
                    self.get_logger().error(f"Invalid VESC values packet: {exc}")
                    continue
                self._handle_values(values)

    def _handle_values(self, values) -> None:
        now_monotonic = time.monotonic()
        self._last_values = values
        self._last_telemetry_monotonic = now_monotonic
        state = self._guard.update(
            values.voltage_input, values.fault_code, now_monotonic
        )
        if state != self._last_guard_state:
            if state == LATCHED:
                fault_name = FAULT_NAMES.get(
                    values.fault_code, f"UNKNOWN_{values.fault_code}"
                )
                self.get_logger().error(
                    "Motor output latched off: voltage=%.2f V, fault=%s"
                    % (values.voltage_input, fault_name)
                )
            elif state == LIMITED:
                self.get_logger().warning(
                    "Motor acceleration inhibited at %.2f V"
                    % values.voltage_input
                )
            else:
                self.get_logger().info("Motor voltage guard is normal")
            self._last_guard_state = state
        self._last_fault_code = values.fault_code

        speed_mps = (
            (values.speed_erpm - float(self.speed_to_erpm_offset))
            / float(self.speed_to_erpm_gain)
            if abs(float(self.speed_to_erpm_gain)) > 1.0e-9
            else math.nan
        )
        stamp = self.get_clock().now().to_msg()
        message = XycarVescState()
        message.header.stamp = stamp
        message.header.frame_id = "vesc"
        message.voltage_input = values.voltage_input
        message.temperature_pcb = values.temperature_pcb
        message.current_motor = values.current_motor
        message.current_input = values.current_input
        message.speed_erpm = values.speed_erpm
        message.speed_mps = speed_mps
        message.input_power_w = values.voltage_input * values.current_input
        message.duty_cycle = values.duty_cycle
        message.charge_drawn = values.charge_drawn
        message.charge_regen = values.charge_regen
        message.energy_drawn = values.energy_drawn
        message.energy_regen = values.energy_regen
        message.displacement = values.displacement
        message.distance_traveled = values.distance_traveled
        message.fault_code = values.fault_code
        self._vesc_pub.publish(message)
        self._publish_odometry(stamp, speed_mps, now_monotonic)

    def _on_motor_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            self.get_logger().warning(
                "Ignoring /xycar_motor: expected [angle, speed]"
            )
            return
        angle = float(message.data[0])
        speed = float(message.data[1])
        if not (math.isfinite(angle) and math.isfinite(speed)):
            self.get_logger().warning(
                "Ignoring /xycar_motor containing a non-finite value"
            )
            return
        self._target_angle_cmd = clamp(
            angle,
            float(self.angle_command_min),
            float(self.angle_command_max),
        )
        self._target_speed_cmd = clamp(
            speed,
            float(self.speed_command_min),
            float(self.speed_command_max),
        )
        self._last_command_monotonic = time.monotonic()

    def _control_update(self) -> None:
        now = time.monotonic()
        dt = clamp(now - self._last_control_monotonic, 0.0, 0.2)
        self._last_control_monotonic = now
        self.drive_enabled = bool(self.get_parameter("drive_enabled").value)

        command_fresh = (
            self._last_command_monotonic is not None
            and now - self._last_command_monotonic <= self.command_timeout_sec
        )
        telemetry_fresh = (
            self._last_telemetry_monotonic is not None
            and now - self._last_telemetry_monotonic
            <= self.telemetry_timeout_sec
        )
        ready = (
            self.drive_enabled
            and self._firmware_version is not None
            and command_fresh
            and (telemetry_fresh or not bool(self.require_telemetry))
            and self._guard.output_allowed
        )

        desired_speed = (
            self._target_speed_cmd
            * float(self.speed_to_mps_gain)
            * self._guard.output_scale
            if ready
            else 0.0
        )

        if ready:
            self._applied_speed_mps = slew(
                self._applied_speed_mps,
                desired_speed,
                dt,
                float(self.acceleration_limit_mps2),
                float(self.deceleration_limit_mps2),
            )
            angle_cmd = self._target_angle_cmd
        else:
            self._applied_speed_mps = 0.0
            angle_cmd = 0.0

        steering_rad = angle_cmd * float(self.angle_to_steering_gain)
        self._applied_steering_rad = steering_rad
        servo_position = clamp(
            float(self.steering_to_servo_gain) * steering_rad
            + float(self.steering_to_servo_offset),
            float(self.servo_min),
            float(self.servo_max),
        )
        erpm = (
            float(self.speed_to_erpm_gain) * self._applied_speed_mps
            + float(self.speed_to_erpm_offset)
        )
        if self._firmware_version is not None:
            self._write(set_servo(servo_position))
            self._write(set_rpm(erpm))

        curvature = (
            math.tan(steering_rad) / float(self.wheelbase_m)
            if float(self.wheelbase_m) > 1.0e-4
            else 0.0
        )
        debug = Float32MultiArray()
        debug.data = [
            float(angle_cmd),
            float(self._target_speed_cmd if ready else 0.0),
            float(steering_rad),
            float(self._applied_speed_mps),
            float(self._applied_speed_mps * curvature),
            float(curvature),
        ]
        self._debug_pub.publish(debug)

    def _on_clear_fault(self, _request, response):
        success, message = self._guard.clear(time.monotonic())
        response.success = success
        response.message = message
        if success:
            self._last_guard_state = self._guard.state
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)
        return response

    def _publish_odometry(self, stamp, speed_mps: float, now: float) -> None:
        steering = self._applied_steering_rad
        yaw_rate = (
            speed_mps * math.tan(steering) / float(self.wheelbase_m)
            if float(self.wheelbase_m) > 1.0e-4
            else 0.0
        )
        if self._last_odom_monotonic is not None:
            dt = clamp(now - self._last_odom_monotonic, 0.0, 0.1)
            mid_yaw = self._odom_yaw + 0.5 * yaw_rate * dt
            self._odom_x += speed_mps * math.cos(mid_yaw) * dt
            self._odom_y += speed_mps * math.sin(mid_yaw) * dt
            self._odom_yaw += yaw_rate * dt
            self._odom_yaw = math.atan2(
                math.sin(self._odom_yaw), math.cos(self._odom_yaw)
            )
        self._last_odom_monotonic = now
        half_yaw = 0.5 * self._odom_yaw

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = str(self.odom_frame)
        odom.child_frame_id = str(self.base_frame)
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)
        odom.twist.twist.linear.x = speed_mps
        odom.twist.twist.angular.z = yaw_rate
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.03
        odom.twist.covariance[35] = 0.1
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = str(self.odom_frame)
            transform.child_frame_id = str(self.base_frame)
            transform.transform.translation.x = self._odom_x
            transform.transform.translation.y = self._odom_y
            transform.transform.rotation.z = math.sin(half_yaw)
            transform.transform.rotation.w = math.cos(half_yaw)
            self._tf_broadcaster.sendTransform(transform)

    def _publish_system_telemetry(self) -> None:
        try:
            current_cpu = read_cpu_times()
            total_delta = current_cpu[0] - self._previous_cpu[0]
            idle_delta = current_cpu[1] - self._previous_cpu[1]
            cpu_percent = (
                100.0 * (total_delta - idle_delta) / total_delta
                if total_delta > 0
                else math.nan
            )
            self._previous_cpu = current_cpu
            memory_percent, memory_available_mb = read_memory()
            disk_free_gb = (
                shutil.disk_usage(str(self.disk_path)).free / (1024.0**3)
            )
        except (OSError, ValueError) as exc:
            self.get_logger().warning(f"System telemetry failed: {exc}")
            return

        message = XycarSystemTelemetry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "vehicle_pc"
        message.cpu_percent = cpu_percent
        message.load_1m = os.getloadavg()[0]
        message.memory_used_percent = memory_percent
        message.memory_available_mb = memory_available_mb
        message.disk_free_gb = disk_free_gb
        message.cpu_temperature_c = read_cpu_temperature_c()
        self._system_pub.publish(message)

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        with self._serial_lock:
            serial_connected = self._serial is not None
        telemetry_age = (
            now - self._last_telemetry_monotonic
            if self._last_telemetry_monotonic is not None
            else math.inf
        )
        command_age = (
            now - self._last_command_monotonic
            if self._last_command_monotonic is not None
            else math.inf
        )

        status = DiagnosticStatus()
        status.name = "xycar_vesc_driver"
        status.hardware_id = str(self.port)
        if not serial_connected or self._firmware_version is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "VESC serial disconnected"
        elif self._guard.state == LATCHED:
            status.level = DiagnosticStatus.ERROR
            status.message = "motor output fault-latched"
        elif self._guard.state == LIMITED:
            status.level = DiagnosticStatus.WARN
            status.message = "low-voltage propulsion derating"
        elif telemetry_age > self.telemetry_timeout_sec:
            status.level = DiagnosticStatus.ERROR
            status.message = "VESC telemetry timeout"
        elif not self.drive_enabled:
            status.level = DiagnosticStatus.WARN
            status.message = "drive disabled"
        elif command_age > self.command_timeout_sec:
            status.level = DiagnosticStatus.WARN
            status.message = "motor command timeout"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "operating"

        firmware = (
            f"{self._firmware_version[0]}.{self._firmware_version[1]}"
            if self._firmware_version is not None
            else "unknown"
        )
        values = {
            "serial_connected": str(serial_connected),
            "firmware": firmware,
            "drive_enabled": str(self.drive_enabled),
            "guard_state": self._guard.state,
            "voltage_output_scale": f"{self._guard.output_scale:.3f}",
            "voltage_input": f"{self._guard.voltage:.3f}",
            "fault_code": str(self._last_fault_code),
            "command_age_sec": f"{command_age:.3f}",
            "telemetry_age_sec": f"{telemetry_age:.3f}",
            "target_speed_cmd": f"{self._target_speed_cmd:.3f}",
            "applied_speed_mps": f"{self._applied_speed_mps:.3f}",
            "parser_crc_errors": str(self._parser.crc_errors),
            "parser_frame_errors": str(self._parser.frame_errors),
            "serial_error": self._last_serial_error,
        }
        status.values = [
            KeyValue(key=key, value=value) for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostic_pub.publish(array)

    def destroy_node(self):
        self._stop_reader.set()
        if self._firmware_version is not None:
            self._write(set_rpm(0.0))
        with self._serial_lock:
            device = self._serial
        if device is not None and hasattr(device, "cancel_read"):
            try:
                device.cancel_read()
            except (OSError, TypeError, serial.SerialException):
                pass
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        with self._serial_lock:
            if self._serial is device:
                self._serial = None
        if device is not None:
            try:
                device.close()
            except (OSError, TypeError, serial.SerialException):
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XycarVescDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
