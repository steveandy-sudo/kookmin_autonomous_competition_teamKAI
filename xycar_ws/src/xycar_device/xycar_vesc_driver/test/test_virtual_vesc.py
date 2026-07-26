"""PTY integration test for the native ROS 2 VESC driver."""

from __future__ import annotations

import errno
import math
import os
import pty
import select
import struct
import threading
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from xycar_msgs.msg import XycarVescState

from xycar_vesc_driver.driver_node import XycarVescDriver
from xycar_vesc_driver.protocol import (
    COMM_FW_VERSION,
    COMM_GET_VALUES,
    COMM_SET_RPM,
    COMM_SET_SERVO_POS,
    FrameParser,
    encode_frame,
)


class VirtualVesc:
    """Respond to the subset of VESC FW 2.18 packets used by the driver."""

    def __init__(self) -> None:
        master_fd, slave_fd = pty.openpty()
        self.master_fd = master_fd
        self.port = os.ttyname(slave_fd)
        os.close(slave_fd)
        os.set_blocking(self.master_fd, False)
        self.parser = FrameParser()
        self.rpm_commands: list[int] = []
        self.servo_commands: list[int] = []
        self.current_erpm = 0
        self.tachometer = 0
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _write(self, payload: bytes) -> None:
        try:
            os.write(self.master_fd, encode_frame(payload))
        except OSError as exc:
            if exc.errno not in (errno.EIO, errno.EBADF):
                raise

    def _values_payload(self) -> bytes:
        self.tachometer += int(self.current_erpm / 300.0)
        payload = bytearray(56)
        payload[0] = COMM_GET_VALUES
        payload[13:15] = struct.pack(">h", 315)
        payload[15:19] = struct.pack(">i", 250)
        payload[19:23] = struct.pack(">i", 100)
        payload[23:25] = struct.pack(">h", 50)
        payload[25:29] = struct.pack(">i", self.current_erpm)
        payload[29:31] = struct.pack(">h", 85)
        payload[47:51] = struct.pack(">i", self.tachometer)
        payload[51:55] = struct.pack(">i", abs(self.tachometer))
        payload[55] = 0
        return bytes(payload)

    def _handle(self, payload: bytes) -> None:
        if payload == bytes((COMM_FW_VERSION,)):
            self._write(bytes((COMM_FW_VERSION, 2, 18)))
        elif payload == bytes((COMM_GET_VALUES,)):
            self._write(self._values_payload())
        elif payload and payload[0] == COMM_SET_RPM and len(payload) >= 5:
            self.current_erpm = struct.unpack(">i", payload[1:5])[0]
            self.rpm_commands.append(self.current_erpm)
        elif (
            payload
            and payload[0] == COMM_SET_SERVO_POS
            and len(payload) >= 3
        ):
            self.servo_commands.append(struct.unpack(">h", payload[1:3])[0])

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                readable, _, _ = select.select(
                    [self.master_fd], [], [], 0.02
                )
                if not readable:
                    continue
                data = os.read(self.master_fd, 4096)
            except OSError as exc:
                if exc.errno in (errno.EIO, errno.EBADF):
                    self.stop_event.wait(0.01)
                    continue
                raise
            for payload in self.parser.feed(data):
                self._handle(payload)

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        try:
            os.close(self.master_fd)
        except OSError:
            pass


class VirtualVescIntegrationTest(unittest.TestCase):
    def _spin_until(
        self,
        executor: SingleThreadedExecutor,
        predicate,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        return False

    def test_firmware_telemetry_ramp_watchdog_and_shutdown_stop(self):
        virtual = VirtualVesc()
        old_domain = os.environ.get("ROS_DOMAIN_ID")
        os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
        rclpy.init(
            args=[
                "--ros-args",
                "-p",
                f"port:={virtual.port}",
                "-p",
                "drive_enabled:=true",
                "-p",
                "publish_tf:=false",
            ]
        )
        driver = XycarVescDriver()
        probe = Node("virtual_vesc_test_probe")
        executor = SingleThreadedExecutor()
        executor.add_node(driver)
        executor.add_node(probe)
        states: list[XycarVescState] = []
        probe.create_subscription(
            XycarVescState,
            "/vehicle/vesc_state",
            states.append,
            20,
        )
        command_publisher = probe.create_publisher(
            Float32MultiArray,
            "/xycar_motor",
            10,
        )
        driver_destroyed = False

        try:
            self.assertTrue(
                self._spin_until(
                    executor,
                    lambda: driver._firmware_compatible and len(states) >= 3,
                    2.0,
                )
            )
            self.assertTrue(all(state.fault_code == 0 for state in states))
            self.assertTrue(
                all(
                    math.isclose(state.voltage_input, 8.5, abs_tol=0.01)
                    for state in states
                )
            )

            command = Float32MultiArray()
            command.data = [10.0, 10.0]
            command_publisher.publish(command)
            start_index = len(virtual.rpm_commands)
            self.assertTrue(
                self._spin_until(
                    executor,
                    lambda: any(
                        rpm > 0
                        for rpm in virtual.rpm_commands[start_index:]
                    ),
                    1.0,
                )
            )
            ramp = [
                rpm
                for rpm in virtual.rpm_commands[start_index:]
                if rpm > 0
            ]
            self.assertLess(ramp[0], 500)
            self.assertLess(max(ramp), 3692)
            self.assertTrue(
                any(575 <= servo <= 590 for servo in virtual.servo_commands)
            )

            self.assertTrue(
                self._spin_until(
                    executor,
                    lambda: (
                        len(virtual.rpm_commands) > start_index
                        and virtual.rpm_commands[-1] == 0
                    ),
                    1.2,
                )
            )

            command_publisher.publish(command)
            second_start = len(virtual.rpm_commands)
            self.assertTrue(
                self._spin_until(
                    executor,
                    lambda: any(
                        rpm > 0
                        for rpm in virtual.rpm_commands[second_start:]
                    ),
                    1.0,
                )
            )
            driver.destroy_node()
            driver_destroyed = True
            deadline = time.monotonic() + 0.3
            while (
                time.monotonic() < deadline
                and virtual.rpm_commands[-1] != 0
            ):
                time.sleep(0.01)
            self.assertEqual(virtual.rpm_commands[-1], 0)
        finally:
            if not driver_destroyed:
                driver.destroy_node()
            probe.destroy_node()
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()
            virtual.close()
            if old_domain is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = old_domain
