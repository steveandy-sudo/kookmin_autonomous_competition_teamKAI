import ast
from pathlib import Path
import unittest
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).parents[1]
MISSION_ROOT = PACKAGE_ROOT / "track_drive" / "mission"


def mission_runtime_source():
    return "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted(MISSION_ROOT.glob("*.py"))
    )


class RuntimeBoundaryTest(unittest.TestCase):
    def test_runtime_has_no_sensor_model_steering_or_motor_dependencies(self):
        source = mission_runtime_source().lower()
        forbidden = (
            "xycar_motor",
            "xycarmotor",
            "float32multiarray",
            "sensor_msgs",
            "laser_scan",
            "laserscan",
            "image_raw",
            "cv2",
            "ultralytics",
            "resnet",
            "torch",
            "tensorflow",
        )
        for token in forbidden:
            with self.subTest(token=token):
                self.assertNotIn(token, source)

    def test_core_has_no_ros_wall_clock_sleep_or_print(self):
        core_source = (
            MISSION_ROOT / "mission_manager.py"
        ).read_text(encoding="utf-8")
        tree = ast.parse(core_source)
        imported_roots = {
            node.names[0].name.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
        }
        imported_roots.update(
            node.module.split(".")[0]
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module
        )
        calls = {
            node.func.id
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
        }

        self.assertNotIn("rclpy", imported_roots)
        self.assertNotIn("time", imported_roots)
        self.assertNotIn("sleep", calls)
        self.assertNotIn("print", calls)

    def test_wrapper_has_exact_integration_topics_and_types(self):
        node_path = MISSION_ROOT / "mission_manager_node.py"
        tree = ast.parse(node_path.read_text(encoding="utf-8"))
        subscriptions = {}
        for node in ast.walk(tree):
            if not (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "create_subscription"
            ):
                continue
            message_type = node.args[0]
            topic = node.args[1]
            self.assertIsInstance(message_type, ast.Name)
            self.assertIsInstance(topic, ast.Constant)
            subscriptions[topic.value] = message_type.id

        self.assertEqual(
            subscriptions,
            {
                "/mission/override": "String",
                "/mission/input/emergency_stop": "Bool",
                "/mission/input/start_signal_go": "Bool",
                "/mission/input/drive_policy_valid": "Bool",
                "/mission/input/lane_fallback_valid": "Bool",
                "/mission/input/cone_detected": "Bool",
                "/mission/input/cone_exit_ready": "Bool",
                "/mission/input/cone_confidence": "Float32",
                "/mission/input/cone_count": "Int32",
            },
        )

    def test_wrapper_runs_at_20_hz_and_has_no_publishers(self):
        node_path = MISSION_ROOT / "mission_manager_node.py"
        source = node_path.read_text(encoding="utf-8")
        tree = ast.parse(source)

        period_assignments = [
            node
            for node in tree.body
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "UPDATE_PERIOD_SEC"
                for target in node.targets
            )
        ]
        self.assertEqual(len(period_assignments), 1)
        self.assertEqual(period_assignments[0].value.value, 0.05)

        publisher_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "create_publisher"
        ]
        self.assertEqual(publisher_calls, [])
        self.assertNotIn("xycar_motor", source)

    def test_wrapper_retains_emergency_assertion_until_update(self):
        source = (
            MISSION_ROOT / "mission_manager_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("_emergency_assertion_pending = True", source)
        self.assertIn("or self._emergency_assertion_pending", source)


class PackageContractTest(unittest.TestCase):
    def test_yaml_contains_exact_parameters_and_values(self):
        yaml_text = (
            PACKAGE_ROOT / "config" / "mission_manager.yaml"
        ).read_text(encoding="utf-8")
        required_lines = (
            "start_signal_hold_sec: 0.3",
            "cone_enter_hold_sec: 0.25",
            "cone_exit_hold_sec: 0.7",
            "cone_min_dwell_sec: 1.0",
            "cone_reenter_cooldown_sec: 1.0",
            "drive_recover_hold_sec: 0.4",
            "lane_fallback_enter_hold_sec: 0.2",
            "minimum_cone_count: 2",
            "minimum_cone_confidence: 0.5",
            "status_log_period_sec: 1.0",
        )
        for line in required_lines:
            with self.subTest(line=line):
                self.assertIn(line, yaml_text)

    def test_manifest_has_runtime_dependencies(self):
        root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
        dependencies = {
            item.text for item in root.findall("exec_depend")
        }
        self.assertTrue(
            {
                "rclpy",
                "std_msgs",
                "ament_index_python",
                "launch",
                "launch_ros",
            }.issubset(dependencies)
        )

    def test_required_launch_and_korean_document_exist(self):
        self.assertTrue(
            (
                PACKAGE_ROOT
                / "launch"
                / "mission_manager_draft.launch.py"
            ).is_file()
        )
        document = (
            PACKAGE_ROOT / "docs" / "MISSION_MANAGER_V01.md"
        )
        self.assertTrue(document.is_file())
        self.assertIn("MissionContext", document.read_text(encoding="utf-8"))

    def test_mission_modules_do_not_form_a_nested_ros_package(self):
        forbidden_metadata = (
            "package.xml",
            "setup.py",
            "setup.cfg",
            "resource",
        )
        for name in forbidden_metadata:
            with self.subTest(name=name):
                self.assertFalse((MISSION_ROOT / name).exists())

    def test_existing_package_installs_manager_assets_and_entry_point(self):
        setup_source = (PACKAGE_ROOT / "setup.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("glob('config/*.yaml')", setup_source)
        self.assertIn("['docs/MISSION_MANAGER_V01.md']", setup_source)
        self.assertIn(
            "mission_manager = "
            "track_drive.mission.mission_manager_node:main",
            setup_source,
        )

    def test_launch_targets_existing_track_drive_package(self):
        launch_source = (
            PACKAGE_ROOT / "launch" / "mission_manager_draft.launch.py"
        ).read_text(encoding="utf-8")
        self.assertEqual(launch_source.count('"track_drive"'), 2)
        self.assertIn('executable="mission_manager"', launch_source)
        self.assertNotIn('"state_machine"', launch_source)


if __name__ == "__main__":
    unittest.main()
