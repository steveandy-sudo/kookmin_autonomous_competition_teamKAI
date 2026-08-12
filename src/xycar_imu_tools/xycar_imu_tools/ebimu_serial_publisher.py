from __future__ import annotations

import math
import re
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu

try:
    import serial
except ImportError:  # pragma: no cover - reported at runtime on the vehicle.
    serial = None


FLOAT_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"
IMU_RECORD_PATTERN = re.compile(
    r"\*(" + FLOAT_PATTERN + r"(?:," + FLOAT_PATTERN + r"){9,})"
)


class EbimuSerialPublisher(Node):
    """Read E2BOX EBIMU ASCII frames over CP210x USB-UART and publish /imu."""

    def __init__(self) -> None:
        super().__init__("ebimu_serial_publisher")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 460800)
        self.declare_parameter("topic", "/imu")
        self.declare_parameter("raw_topic", "/imu/raw_data")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("configure_sensor", True)
        self.declare_parameter("output_rate_command", "<sor10>")

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)
        raw_topic = str(self.get_parameter("raw_topic").value)

        self.imu_pub = self.create_publisher(Imu, topic, QoSProfile(depth=10))
        self.raw_imu_pub = None
        if raw_topic and raw_topic != topic:
            self.raw_imu_pub = self.create_publisher(Imu, raw_topic, QoSProfile(depth=10))

        self.ser = None
        self.connected = False
        self.read_buffer = ""
        self.last_record_time = self.get_clock().now().nanoseconds / 1e9
        self.last_warn_time = 0.0

        if serial is None:
            self.get_logger().error("python3-serial is not installed")
            return

        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.05)
            self.connected = True
        except serial.SerialException as exc:
            self.get_logger().error(f"failed to open EBIMU serial port {self.port}: {exc}")
            return

        self.get_logger().info(
            f"EBIMU connected | port={self.port}, baudrate={self.baudrate}, topic={topic}"
        )
        if bool(self.get_parameter("configure_sensor").value):
            self.setup_imu()

        rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

    def setup_imu(self) -> None:
        output_rate_command = str(self.get_parameter("output_rate_command").value)
        commands = [
            "<soc1>",   # ASCII output
            "<sof2>",   # quaternion
            "<sog1>",   # gyro
            "<soa1>",   # acceleration
            "<sem1>",   # magnetometer enabled on the sensor
            "<sot0>",   # temperature off
            "<sod0>",   # distance off
            "<sots0>",  # timestamp off
            output_rate_command,
        ]
        for cmd in commands:
            self.send_command(cmd)
            self.get_logger().info(f"EBIMU setup: {cmd}")
        self.ser.reset_input_buffer()

    def send_command(self, cmd: str) -> None:
        if not (cmd.startswith("<") and cmd.endswith(">")):
            raise ValueError(f"invalid EBIMU command: {cmd}")
        self.ser.write(b"<")
        self.ser.flush()
        time.sleep(0.15)
        self.ser.write(cmd[1:].encode("ascii"))
        self.ser.flush()
        time.sleep(0.15)
        waiting = self.ser.in_waiting
        if waiting:
            self.ser.read(waiting)

    def timer_callback(self) -> None:
        if not self.connected or self.ser is None:
            return
        try:
            read_size = max(1, self.ser.in_waiting)
            chunk = self.ser.read(read_size).decode("utf-8", errors="ignore")
        except serial.SerialException as exc:
            now = self.get_clock().now().nanoseconds / 1e9
            text = str(exc)
            if "returned no data" in text and now - self.last_warn_time > 1.0:
                self.get_logger().warn(f"EBIMU serial read skipped: {text}")
                self.last_warn_time = now
            elif "returned no data" not in text:
                self.get_logger().error(f"EBIMU serial read failed: {exc}")
            return

        if not chunk:
            return

        self.read_buffer += chunk
        if len(self.read_buffer) > 4096:
            self.read_buffer = self.read_buffer[-4096:]

        matches = list(IMU_RECORD_PATTERN.finditer(self.read_buffer))
        if not matches:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_record_time > 1.0 and now - self.last_warn_time > 1.0:
                self.get_logger().warn("no complete EBIMU ASCII record received recently")
                self.last_warn_time = now
            return

        match = matches[-1]
        self.last_record_time = self.get_clock().now().nanoseconds / 1e9
        self.read_buffer = self.read_buffer[match.end():]
        fields = match.group(1).split(",")[:10]
        msg = self._imu_from_fields(fields)
        if msg is None:
            return

        self.imu_pub.publish(msg)
        if self.raw_imu_pub is not None:
            self.raw_imu_pub.publish(msg)

    def _imu_from_fields(self, fields: list[str]) -> Optional[Imu]:
        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # EBIMU ASCII order used here: qz, qy, qx, qw, gx, gy, gz, ax, ay, az.
            msg.orientation = Quaternion(
                x=float(fields[2]),
                y=float(fields[1]),
                z=float(fields[0]),
                w=float(fields[3]),
            )
            msg.orientation_covariance = [
                0.0007, 0.0, 0.0,
                0.0, 0.0007, 0.0,
                0.0, 0.0, 0.0007,
            ]
            msg.angular_velocity.x = math.radians(float(fields[4]))
            msg.angular_velocity.y = math.radians(float(fields[5]))
            msg.angular_velocity.z = math.radians(float(fields[6]))
            msg.angular_velocity_covariance = [
                0.001, 0.0, 0.0,
                0.0, 0.001, 0.0,
                0.0, 0.0, 0.001,
            ]
            msg.linear_acceleration.x = -float(fields[7])
            msg.linear_acceleration.y = -float(fields[8])
            msg.linear_acceleration.z = -float(fields[9])
            msg.linear_acceleration_covariance = [
                0.005, 0.0, 0.0,
                0.0, 0.005, 0.0,
                0.0, 0.0, 0.005,
            ]
            return msg
        except (IndexError, ValueError) as exc:
            self.get_logger().warn(f"failed to parse EBIMU fields {fields!r}: {exc}")
            return None


def main(args=None) -> int:
    rclpy.init(args=args)
    node = EbimuSerialPublisher()
    if not node.connected:
        node.destroy_node()
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
