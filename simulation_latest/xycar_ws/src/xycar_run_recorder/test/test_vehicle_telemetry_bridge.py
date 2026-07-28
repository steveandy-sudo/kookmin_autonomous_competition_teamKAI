import unittest

from xycar_run_recorder.vehicle_telemetry_bridge import VescCsvDecoder


class VescCsvDecoderTest(unittest.TestCase):
    def test_decodes_ros1_state_csv(self):
        fields = (
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
        decoder = VescCsvDecoder()
        decoder.feed(
            ",".join(["%time"] + [f"field.state.{field}" for field in fields])
        )

        sample = decoder.feed(
            ",".join(["123"] + [str(index + 1) for index in range(len(fields))])
        )

        self.assertIsNotNone(sample)
        self.assertEqual(sample["voltage_input"], 1.0)
        self.assertEqual(sample["fault_code"], 13.0)


if __name__ == "__main__":
    unittest.main()
