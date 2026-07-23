import ast
from pathlib import Path
import re
import unittest
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).parents[1]
NODE_PATH = PACKAGE_ROOT / "track_drive" / "final_driver_node.py"


class FinalDriverNodeContractTest(unittest.TestCase):
    def test_final_driver_is_the_only_xycar_motor_publisher(self):
        motor_sources = []
        for path in (PACKAGE_ROOT / "track_drive").rglob("*.py"):
            source = path.read_text(encoding="utf-8")
            if "XycarMotor" in source:
                motor_sources.append(path.relative_to(PACKAGE_ROOT).as_posix())

        self.assertEqual(motor_sources, ["track_drive/final_driver_node.py"])

        source = NODE_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        publisher_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "create_publisher"
        ]
        self.assertEqual(len(publisher_calls), 1)
        self.assertEqual(publisher_calls[0].args[0].id, "XycarMotor")
        self.assertIn(
            'self.declare_parameter("motor_topic", "/xycar_motor")',
            source,
        )

    def test_final_driver_subscribes_directly_to_all_numeric_sources(self):
        source = NODE_PATH.read_text(encoding="utf-8")
        tree = ast.parse(source)
        subscription_types = [
            node.args[0].id
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "create_subscription"
        ]

        self.assertCountEqual(
            subscription_types,
            [
                "MissionDecisionMsg",
                "Float32MultiArray",
                "LaneFallbackCommand",
                "Float32MultiArray",
            ],
        )
        for topic in (
            "/mission/decision",
            "/il/policy_debug",
            "/lane_fallback/command",
            "/my_rule/cone_cmd",
        ):
            with self.subTest(topic=topic):
                self.assertIn(topic, source)

    def test_final_driver_uses_50_hz_latest_value_qos(self):
        source = NODE_PATH.read_text(encoding="utf-8")

        self.assertIn("DEFAULT_PUBLISH_RATE_HZ = 50.0", source)
        self.assertIn(
            "self.create_timer(1.0 / publish_rate_hz, self._publish_selected)",
            source,
        )
        self.assertIn("HistoryPolicy.KEEP_LAST", source)
        self.assertIn("ReliabilityPolicy.BEST_EFFORT", source)
        self.assertIn("ReliabilityPolicy.RELIABLE", source)
        self.assertGreaterEqual(source.count("depth=1"), 3)
        self.assertNotIn("rate_limit", source.lower())
        self.assertNotIn("smoothing", source.lower())

    def test_drive_launch_requires_fallback_speed_and_draft_stays_motor_free(self):
        drive_launch = (
            PACKAGE_ROOT / "launch" / "mission_manager_drive.launch.py"
        ).read_text(encoding="utf-8")
        draft_launch = (
            PACKAGE_ROOT / "launch" / "mission_manager_draft.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'DeclareLaunchArgument(\n                "fallback_speed"',
            drive_launch,
        )
        self.assertNotIn("default_value", drive_launch)
        self.assertIn('executable="final_driver"', drive_launch)
        self.assertNotIn('executable="final_driver"', draft_launch)

    def test_manifest_and_setup_install_single_final_driver(self):
        setup_source = (PACKAGE_ROOT / "setup.py").read_text(
            encoding="utf-8"
        )
        manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
        dependencies = {
            item.text for item in manifest.findall("exec_depend")
        }

        self.assertIn("xycar_msgs", dependencies)
        self.assertIn(
            "'launch/mission_manager_drive.launch.py'",
            setup_source,
        )
        self.assertIn(
            "final_driver = track_drive.final_driver_node:main",
            setup_source,
        )

    def test_default_yaml_does_not_invent_fallback_speed(self):
        yaml_source = (
            PACKAGE_ROOT / "config" / "mission_manager.yaml"
        ).read_text(encoding="utf-8")

        self.assertIn("final_driver:", yaml_source)
        self.assertIn("publish_rate_hz: 50.0", yaml_source)
        self.assertIsNone(
            re.search(
                r"^\s+fallback_speed\s*:",
                yaml_source,
                flags=re.MULTILINE,
            )
        )


if __name__ == "__main__":
    unittest.main()
