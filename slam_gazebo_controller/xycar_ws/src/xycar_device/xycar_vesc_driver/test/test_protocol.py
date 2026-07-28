import struct
import unittest

from xycar_vesc_driver.protocol import (
    COMM_GET_VALUES,
    FrameParser,
    crc16,
    decode_values,
    encode_frame,
    set_rpm,
    set_servo,
)


class VescProtocolTest(unittest.TestCase):
    def test_crc_reference_vector(self):
        self.assertEqual(crc16(b"123456789"), 0x31C3)

    def test_frame_parser_handles_fragmentation_and_noise(self):
        first = encode_frame(b"\x00\x02\x12")
        second = encode_frame(b"\x04" + bytes(range(55)))
        parser = FrameParser()

        self.assertEqual(parser.feed(b"\x99\x88" + first[:3]), [])
        self.assertEqual(
            parser.feed(first[3:] + second),
            [b"\x00\x02\x12", b"\x04" + bytes(range(55))],
        )
        self.assertEqual(parser.discarded_bytes, 2)

    def test_frame_parser_rejects_bad_crc(self):
        damaged = bytearray(encode_frame(b"\x04hello"))
        damaged[-3] ^= 0x01
        valid = encode_frame(b"\x00\x02\x12")
        parser = FrameParser()
        self.assertEqual(
            parser.feed(bytes(damaged) + valid),
            [b"\x00\x02\x12"],
        )
        self.assertEqual(parser.crc_errors, 1)

    def test_frame_parser_recovers_from_fake_incomplete_prefix(self):
        valid = encode_frame(b"\x00\x02\x12")
        parser = FrameParser()
        self.assertEqual(
            parser.feed(b"\x02\xf0noise" + valid),
            [b"\x00\x02\x12"],
        )
        self.assertEqual(parser.discarded_bytes, 7)

    def test_long_frame_round_trip_with_fragmentation(self):
        payload = bytes(index % 251 for index in range(300))
        frame = encode_frame(payload)
        self.assertEqual(frame[0], 3)
        parser = FrameParser()
        self.assertEqual(parser.feed(frame[:111]), [])
        self.assertEqual(parser.feed(frame[111:]), [payload])

    def test_command_encoding(self):
        rpm_payload = FrameParser().feed(set_rpm(-1234))[0]
        servo_payload = FrameParser().feed(set_servo(0.5004))[0]
        self.assertEqual(rpm_payload[0], 8)
        self.assertEqual(struct.unpack(">i", rpm_payload[1:])[0], -1234)
        self.assertEqual(servo_payload[0], 11)
        self.assertEqual(struct.unpack(">h", servo_payload[1:])[0], 500)

    def test_decode_legacy_values_packet(self):
        payload = bytearray(56)
        payload[0] = COMM_GET_VALUES
        payload[13:15] = struct.pack(">h", 318)
        payload[15:19] = struct.pack(">i", 659)
        payload[19:23] = struct.pack(">i", 67)
        payload[23:25] = struct.pack(">h", 103)
        payload[25:29] = struct.pack(">i", 2884)
        payload[29:31] = struct.pack(">h", 79)
        payload[31:35] = struct.pack(">i", 234)
        payload[35:39] = struct.pack(">i", 2)
        payload[39:43] = struct.pack(">i", 1785)
        payload[43:47] = struct.pack(">i", 22)
        payload[47:51] = struct.pack(">i", 43557)
        payload[51:55] = struct.pack(">i", 43651)
        payload[55] = 2

        values = decode_values(bytes(payload))
        self.assertAlmostEqual(values.voltage_input, 7.9)
        self.assertAlmostEqual(values.temperature_pcb, 31.8)
        self.assertAlmostEqual(values.current_motor, 6.59)
        self.assertAlmostEqual(values.current_input, 0.67)
        self.assertEqual(values.speed_erpm, 2884)
        self.assertAlmostEqual(values.duty_cycle, 0.103)
        self.assertEqual(values.fault_code, 2)
