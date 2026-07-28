"""Minimal VESC UART protocol used by the legacy Xycar controller."""

from __future__ import annotations

from dataclasses import dataclass
import struct


COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_RPM = 8
COMM_SET_SERVO_POS = 11

MAX_PAYLOAD_SIZE = 1024


def crc16(payload: bytes) -> int:
    """Return the CRC-16/CCITT-FALSE variant used by VESC frames."""
    crc = 0
    for value in payload:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(payload: bytes) -> bytes:
    """Wrap a payload in the VESC UART frame format."""
    size = len(payload)
    if size > MAX_PAYLOAD_SIZE:
        raise ValueError(f"VESC payload is too large: {size}")
    if size < 256:
        header = bytes((2, size))
    else:
        header = bytes((3, (size >> 8) & 0xFF, size & 0xFF))
    checksum = crc16(payload)
    return header + payload + checksum.to_bytes(2, "big") + b"\x03"


def request_firmware_version() -> bytes:
    return encode_frame(bytes((COMM_FW_VERSION,)))


def request_values() -> bytes:
    return encode_frame(bytes((COMM_GET_VALUES,)))


def set_rpm(erpm: float) -> bytes:
    value = max(-(2**31), min(2**31 - 1, int(erpm)))
    return encode_frame(bytes((COMM_SET_RPM,)) + struct.pack(">i", value))


def set_servo(position: float) -> bytes:
    value = max(-32768, min(32767, int(round(position * 1000.0))))
    return encode_frame(
        bytes((COMM_SET_SERVO_POS,)) + struct.pack(">h", value)
    )


class FrameParser:
    """Incrementally extract verified VESC payloads from a byte stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.discarded_bytes = 0
        self.crc_errors = 0
        self.frame_errors = 0

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        payloads: list[bytes] = []
        while self.buffer:
            start = next(
                (
                    index
                    for index, value in enumerate(self.buffer)
                    if value in (2, 3)
                ),
                None,
            )
            if start is None:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]

            if self.buffer[0] == 2:
                if len(self.buffer) < 2:
                    break
                payload_size = self.buffer[1]
                header_size = 2
            else:
                if len(self.buffer) < 3:
                    break
                payload_size = (self.buffer[1] << 8) | self.buffer[2]
                header_size = 3

            if payload_size > MAX_PAYLOAD_SIZE:
                self.frame_errors += 1
                del self.buffer[0]
                continue

            frame_size = header_size + payload_size + 3
            if len(self.buffer) < frame_size:
                break
            if self.buffer[frame_size - 1] != 3:
                self.frame_errors += 1
                del self.buffer[0]
                continue

            payload = bytes(
                self.buffer[header_size : header_size + payload_size]
            )
            expected_crc = int.from_bytes(
                self.buffer[
                    header_size + payload_size : header_size + payload_size + 2
                ],
                "big",
            )
            if crc16(payload) != expected_crc:
                self.crc_errors += 1
                del self.buffer[:frame_size]
                continue

            payloads.append(payload)
            del self.buffer[:frame_size]
        return payloads


@dataclass(frozen=True)
class VescValues:
    voltage_input: float
    temperature_pcb: float
    current_motor: float
    current_input: float
    speed_erpm: float
    duty_cycle: float
    charge_drawn: float
    charge_regen: float
    energy_drawn: float
    energy_regen: float
    displacement: float
    distance_traveled: float
    fault_code: int


def _signed(payload: bytes, start: int, size: int) -> int:
    return int.from_bytes(payload[start : start + size], "big", signed=True)


def decode_values(payload: bytes) -> VescValues:
    """Decode COMM_GET_VALUES used by the installed legacy VESC firmware."""
    if not payload or payload[0] != COMM_GET_VALUES:
        raise ValueError("payload is not COMM_GET_VALUES")
    if len(payload) < 56:
        raise ValueError(
            f"COMM_GET_VALUES payload is too short: {len(payload)}"
        )
    return VescValues(
        temperature_pcb=_signed(payload, 13, 2) / 10.0,
        current_motor=_signed(payload, 15, 4) / 100.0,
        current_input=_signed(payload, 19, 4) / 100.0,
        duty_cycle=_signed(payload, 23, 2) / 1000.0,
        speed_erpm=float(_signed(payload, 25, 4)),
        voltage_input=_signed(payload, 29, 2) / 10.0,
        charge_drawn=float(_signed(payload, 31, 4)),
        charge_regen=float(_signed(payload, 35, 4)),
        energy_drawn=float(_signed(payload, 39, 4)),
        energy_regen=float(_signed(payload, 43, 4)),
        displacement=float(_signed(payload, 47, 4)),
        distance_traveled=float(_signed(payload, 51, 4)),
        fault_code=int(payload[55]),
    )
