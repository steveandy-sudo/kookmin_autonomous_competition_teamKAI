from __future__ import annotations

from collections import deque
import csv
import math
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from xycar_msgs.msg import XycarSystemTelemetry, XycarVescState


VESC_FIELDS = (
    "voltage_input",
    "temperature_pcb",
    "current_motor",
    "current_input",
    "speed",
    "duty_cycle",
    "charge_drawn",
    "charge_regen",
    "energy_drawn",
    "energy_regen",
    "displacement",
    "distance_traveled",
    "fault_code",
)


class VescCsvDecoder:
    def __init__(self) -> None:
        self.header: list[str] = []

    def feed(self, line: str) -> dict[str, float] | None:
        stripped = line.strip()
        if not stripped:
            return None
        values = next(csv.reader([stripped]))
        if values and values[0].lstrip().startswith("%time"):
            self.header = [value.strip() for value in values]
            return None
        if not self.header or len(values) < len(self.header):
            return None
        row = dict(zip(self.header, values))
        decoded: dict[str, float] = {}
        for field in VESC_FIELDS:
            key = next(
                (
                    candidate
                    for candidate in row
                    if candidate.endswith(f".state.{field}")
                    or candidate.endswith(f"state.{field}")
                ),
                None,
            )
            if key is None:
                raise ValueError(f"missing VESC CSV field: {field}")
            decoded[field] = float(row[key])
        return decoded


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


class VehicleTelemetryBridge(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_recording_telemetry")
        self.declare_parameter("container_name", "ros1_container")
        self.declare_parameter("ros1_topic", "/sensors/core")
        self.declare_parameter("retry_sec", 2.0)
        self.declare_parameter("disk_path", str(Path.home()))
        self.declare_parameter("vesc_topic", "/vehicle/vesc_state")
        self.declare_parameter("system_topic", "/vehicle/system_telemetry")
        self.declare_parameter("speed_to_erpm_gain", 4614.0)
        self.declare_parameter("speed_to_erpm_offset", 0.0)

        self.vesc_publisher = self.create_publisher(
            XycarVescState,
            str(self.get_parameter("vesc_topic").value),
            20,
        )
        self.system_publisher = self.create_publisher(
            XycarSystemTelemetry,
            str(self.get_parameter("system_topic").value),
            10,
        )
        self.samples: deque[dict[str, float]] = deque(maxlen=1000)
        self.samples_lock = threading.Lock()
        self.process: subprocess.Popen | None = None
        self.reader_thread: threading.Thread | None = None
        self.last_start_wall_sec = 0.0
        self.last_error = ""
        self.previous_cpu = read_cpu_times()
        self.create_timer(0.01, self._publish_vesc_samples)
        self.create_timer(1.0, self._publish_system_telemetry)
        self.create_timer(0.5, self._ensure_ros1_reader)

    def _reader_command(self) -> list[str]:
        container = str(self.get_parameter("container_name").value)
        topic = str(self.get_parameter("ros1_topic").value)
        shell = (
            "source /opt/ros/noetic/setup.bash; "
            "source /root/noetic_ws/devel/setup.bash 2>/dev/null || true; "
            f"exec stdbuf -oL rostopic echo -p {topic}"
        )
        return ["docker", "exec", container, "bash", "-lc", shell]

    def _ensure_ros1_reader(self) -> None:
        if self.process is not None and self.process.poll() is None:
            return
        now = time.monotonic()
        retry_sec = max(0.5, float(self.get_parameter("retry_sec").value))
        if now - self.last_start_wall_sec < retry_sec:
            return
        self.last_start_wall_sec = now
        try:
            self.process = subprocess.Popen(
                self._reader_command(),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            self.last_error = str(exc)
            return
        self.reader_thread = threading.Thread(
            target=self._read_ros1_output,
            daemon=True,
        )
        self.reader_thread.start()

    def _read_ros1_output(self) -> None:
        process = self.process
        if process is None or process.stdout is None:
            return
        decoder = VescCsvDecoder()
        try:
            for line in process.stdout:
                try:
                    sample = decoder.feed(line)
                except (ValueError, csv.Error) as exc:
                    self.last_error = str(exc)
                    continue
                if sample is not None:
                    with self.samples_lock:
                        self.samples.append(sample)
        finally:
            if process.stderr is not None:
                error = process.stderr.read().strip()
                if error:
                    self.last_error = error.splitlines()[-1]

    def _publish_vesc_samples(self) -> None:
        pending = []
        with self.samples_lock:
            while self.samples:
                pending.append(self.samples.popleft())
        for sample in pending:
            message = XycarVescState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "vesc"
            message.voltage_input = sample["voltage_input"]
            message.temperature_pcb = sample["temperature_pcb"]
            message.current_motor = sample["current_motor"]
            message.current_input = sample["current_input"]
            message.speed_erpm = sample["speed"]
            speed_gain = float(
                self.get_parameter("speed_to_erpm_gain").value
            )
            speed_offset = float(
                self.get_parameter("speed_to_erpm_offset").value
            )
            message.speed_mps = (
                (sample["speed"] - speed_offset) / speed_gain
                if abs(speed_gain) > 1.0e-9
                else math.nan
            )
            message.input_power_w = (
                sample["voltage_input"] * sample["current_input"]
            )
            message.duty_cycle = sample["duty_cycle"]
            message.charge_drawn = sample["charge_drawn"]
            message.charge_regen = sample["charge_regen"]
            message.energy_drawn = sample["energy_drawn"]
            message.energy_regen = sample["energy_regen"]
            message.displacement = sample["displacement"]
            message.distance_traveled = sample["distance_traveled"]
            message.fault_code = int(sample["fault_code"])
            self.vesc_publisher.publish(message)

    def _publish_system_telemetry(self) -> None:
        current_cpu = read_cpu_times()
        total_delta = current_cpu[0] - self.previous_cpu[0]
        idle_delta = current_cpu[1] - self.previous_cpu[1]
        cpu_percent = (
            100.0 * (total_delta - idle_delta) / total_delta
            if total_delta > 0
            else math.nan
        )
        self.previous_cpu = current_cpu
        memory_percent, memory_available_mb = read_memory()
        disk_path = str(self.get_parameter("disk_path").value)
        disk_free_gb = shutil.disk_usage(disk_path).free / (1024.0**3)

        message = XycarSystemTelemetry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "vehicle_pc"
        message.cpu_percent = cpu_percent
        message.load_1m = os.getloadavg()[0]
        message.memory_used_percent = memory_percent
        message.memory_available_mb = memory_available_mb
        message.disk_free_gb = disk_free_gb
        message.cpu_temperature_c = read_cpu_temperature_c()
        self.system_publisher.publish(message)

    def destroy_node(self):
        process = self.process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleTelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
