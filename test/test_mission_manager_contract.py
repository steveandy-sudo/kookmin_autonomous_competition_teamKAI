import ast
from pathlib import Path
import unittest
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).parents[1]
MISSION_ROOT = PACKAGE_ROOT / "track_drive" / "mission"
INTEGRATION_ROOT = PACKAGE_ROOT / "track_drive" / "integration"


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

        runtime_source = mission_runtime_source()
        self.assertNotIn("MissionState.EMERGENCY_STOP", runtime_source)
        self.assertNotIn('"RESET_EMERGENCY"', runtime_source)

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
                "/mission/input/safety_stop_required": "Bool",
                "/mission/input/start_signal": "String",
                "/mission/input/start_signal_valid": "Bool",
                "/mission/input/safety_ready": "Bool",
                "/mission/input/drive_policy_valid": "Bool",
                "/mission/input/lane_fallback_valid": "Bool",
                "/mission/input/camera_cone_valid": "Bool",
                "/mission/input/camera_cone_count": "Int32",
                "/mission/input/lidar_cone_source_valid": "Bool",
                "/mission/input/lidar_cone_path_ready": "Bool",
                "/mission/input/lidar_cone_present": "Bool",
            },
        )

    def test_wrapper_runs_at_20_hz_and_publishes_only_decision(self):
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
        self.assertEqual(len(publisher_calls), 1)
        publisher_call = publisher_calls[0]
        self.assertEqual(publisher_call.args[0].id, "MissionDecisionMsg")
        self.assertEqual(publisher_call.args[1].id, "DECISION_TOPIC")
        self.assertEqual(publisher_call.args[2].id, "DECISION_QOS")
        self.assertIn('DECISION_TOPIC = "/mission/decision"', source)
        self.assertIn("ReliabilityPolicy.RELIABLE", source)
        self.assertIn("DurabilityPolicy.VOLATILE", source)
        self.assertIn("HistoryPolicy.KEEP_LAST", source)
        self.assertIn("depth=1", source)
        self.assertIn(
            "decision = self._manager.update(observation)",
            source,
        )
        self.assertIn(
            "self._publish_decision(decision, now.to_msg())",
            source,
        )
        self.assertNotIn("xycar_motor", source)

    def test_wrapper_retains_safety_stop_assertion_until_update(self):
        source = (
            MISSION_ROOT / "mission_manager_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("_safety_stop_assertion_pending = True", source)
        self.assertIn("or self._safety_stop_assertion_pending", source)

    def test_start_signal_adapter_has_exact_topic_contract(self):
        source = (
            MISSION_ROOT / "start_signal_adapter_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            '"/track_drive/traffic_light_debug/state"',
            source,
        )
        self.assertIn('"/mission/input/start_signal"', source)
        self.assertIn('"/mission/input/start_signal_valid"', source)
        self.assertEqual(source.count("create_subscription("), 1)
        self.assertEqual(source.count("create_publisher("), 2)
        self.assertNotIn("xycar_motor", source.lower())

    def test_drive_policy_adapter_has_exact_topic_contract(self):
        source = (
            INTEGRATION_ROOT / "drive_policy_adapter_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"/il/policy_debug"', source)
        self.assertIn(
            '"/mission/input/drive_policy_valid"',
            source,
        )
        self.assertEqual(source.count("create_subscription("), 1)
        self.assertEqual(source.count("create_publisher("), 1)
        self.assertNotIn("policy_motor_shadow", source)
        self.assertNotIn("xycar_motor", source.lower())

    def test_lane_fallback_adapter_has_exact_topic_contract(self):
        source = (
            INTEGRATION_ROOT / "lane_fallback_adapter_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"/perception/centerline"', source)
        self.assertIn(
            '"/mission/input/lane_fallback_valid"',
            source,
        )
        self.assertIn("Centerline", source)
        self.assertEqual(source.count("create_subscription("), 1)
        self.assertEqual(source.count("create_publisher("), 1)
        self.assertNotIn("xycar_motor", source.lower())

    def test_camera_cone_adapter_has_exact_topic_contract(self):
        source = (
            INTEGRATION_ROOT / "camera_cone_adapter_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"/perception/camera_cone_count"', source)
        self.assertIn(
            '"/mission/input/camera_cone_count"',
            source,
        )
        self.assertIn(
            '"/mission/input/camera_cone_valid"',
            source,
        )
        self.assertEqual(source.count("create_subscription("), 1)
        self.assertEqual(source.count("create_publisher("), 2)
        self.assertNotIn("xycar_motor", source.lower())

    def test_traffic_light_node_reuses_inference_for_camera_cone_count(self):
        node_source = (
            PACKAGE_ROOT / "track_drive" / "traffic_light_debug_node.py"
        ).read_text(encoding="utf-8")
        detector_source = (
            PACKAGE_ROOT / "track_drive" / "traffic_light_detector.py"
        ).read_text(encoding="utf-8")
        self.assertIn("camera_cone_count_pub", node_source)
        self.assertIn("count_camera_cones", node_source)
        self.assertIn(
            "camera_cone_count_topic', '/perception/camera_cone_count'",
            detector_source,
        )
        self.assertIn("camera_cone_class_ids', [0]", detector_source)
        self.assertIn(
            "yolo_red_light_class_ids', [4]",
            detector_source,
        )
        self.assertIn(
            "yolo_yellow_light_class_ids', [5]",
            detector_source,
        )
        self.assertIn(
            "yolo_yellow_light_conf_threshold', 0.35",
            detector_source,
        )
        self.assertNotIn(
            "yolo_red_light_class_ids', [4, 5]",
            detector_source,
        )
        self.assertEqual(node_source.count("self.detector.detect(frame)"), 1)
        self.assertNotIn("xycar_motor", node_source.lower())

    def test_lidar_cone_adapter_has_exact_topic_contract(self):
        source = (
            INTEGRATION_ROOT / "lidar_cone_adapter_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"/my_rule/cone_cmd"', source)
        self.assertIn('"/my_rule/cone_clusters"', source)
        self.assertIn(
            '"/mission/input/lidar_cone_source_valid"',
            source,
        )
        self.assertIn(
            '"/mission/input/lidar_cone_path_ready"',
            source,
        )
        self.assertIn(
            '"/mission/input/lidar_cone_present"',
            source,
        )
        self.assertEqual(source.count("create_subscription("), 2)
        self.assertEqual(source.count("create_publisher("), 3)
        self.assertNotIn("xycar_motor", source.lower())
        self.assertNotIn('"/scan"', source)

    def test_duplicate_raw_scan_cone_detector_is_removed(self):
        self.assertFalse(
            (PACKAGE_ROOT / "track_drive" / "lidar_cone_detector.py").exists()
        )
        self.assertFalse(
            (
                PACKAGE_ROOT
                / "track_drive"
                / "lidar_cone_detector_node.py"
            ).exists()
        )


class PackageContractTest(unittest.TestCase):
    def test_yaml_contains_exact_parameters_and_values(self):
        yaml_text = (
            PACKAGE_ROOT / "config" / "mission_manager.yaml"
        ).read_text(encoding="utf-8")
        required_lines = (
            "start_signal_red_hold_sec: 0.3",
            "start_signal_green_hold_sec: 0.3",
            "cone_enter_hold_sec: 0.25",
            "cone_exit_hold_sec: 0.7",
            "cone_min_dwell_sec: 1.0",
            "cone_reenter_cooldown_sec: 1.0",
            "drive_recover_hold_sec: 0.4",
            "lane_fallback_ready_hold_sec: 0.2",
            "minimum_camera_cone_count: 4",
            "maximum_camera_cone_count_for_exit: 1",
            "status_log_period_sec: 1.0",
            "source_state_topic: /track_drive/traffic_light_debug/state",
            "output_signal_topic: /mission/input/start_signal",
            "output_valid_topic: /mission/input/start_signal_valid",
            "source_timeout_sec: 0.5",
            "publish_rate_hz: 20.0",
            "source_debug_topic: /il/policy_debug",
            "output_valid_topic: /mission/input/drive_policy_valid",
            "source_centerline_topic: /perception/centerline",
            "output_valid_topic: /mission/input/lane_fallback_valid",
            "source_timeout_sec: 0.4",
            "minimum_point_count: 3",
            "minimum_confidence: 0.25",
            "source_count_topic: /perception/camera_cone_count",
            "output_count_topic: /mission/input/camera_cone_count",
            "output_valid_topic: /mission/input/camera_cone_valid",
            "source_timeout_sec: 0.2",
            "source_command_topic: /my_rule/cone_cmd",
            "source_cluster_topic: /my_rule/cone_clusters",
            "output_source_valid_topic: /mission/input/lidar_cone_source_valid",
            "output_path_ready_topic: /mission/input/lidar_cone_path_ready",
            "output_present_topic: /mission/input/lidar_cone_present",
            "path_ready_confidence: 0.35",
            "presence_confidence: 0.2",
            "presence_min_clusters: 2",
        )
        for line in required_lines:
            with self.subTest(line=line):
                self.assertIn(line, yaml_text)

    def test_adapter_timeouts_are_scoped_to_the_correct_yaml_blocks(self):
        yaml_text = (
            PACKAGE_ROOT / "config" / "mission_manager.yaml"
        ).read_text(encoding="utf-8")
        start_block = yaml_text.split(
            "mission_start_signal_adapter:",
            1,
        )[1].split("mission_drive_policy_adapter:", 1)[0]
        lidar_block = yaml_text.split(
            "mission_lidar_cone_adapter:",
            1,
        )[1]

        self.assertIn("source_timeout_sec: 0.5", start_block)
        self.assertIn("source_timeout_sec: 0.2", lidar_block)
        self.assertNotIn("source_timeout_sec: 0.5", lidar_block)

    def test_manifest_has_runtime_dependencies(self):
        root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
        dependencies = {
            item.text for item in root.findall("exec_depend")
        }
        self.assertTrue(
            {
                "rclpy",
                "std_msgs",
                "kaiev26_msgs",
                "teamkai_interfaces",
                "ament_index_python",
                "launch",
                "launch_ros",
            }.issubset(dependencies)
        )
        self.assertTrue(
            {
                "nav_msgs",
                "visualization_msgs",
                "xycar_msgs",
                "cone_il",
            }.isdisjoint(dependencies)
        )
        self.assertEqual(root.find("version").text, "0.2.0")

    def test_required_launch_and_korean_document_exist(self):
        self.assertTrue(
            (
                PACKAGE_ROOT
                / "launch"
                / "mission_manager_draft.launch.py"
            ).is_file()
        )
        document = (
            PACKAGE_ROOT / "docs" / "MISSION_MANAGER_V02.md"
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

    def test_mission_decision_interface_has_exact_output_contract(self):
        interface_root = PACKAGE_ROOT / "teamkai_interfaces"
        message_path = interface_root / "msg" / "MissionDecision.msg"

        self.assertTrue((interface_root / "CMakeLists.txt").is_file())
        self.assertTrue((interface_root / "package.xml").is_file())
        self.assertTrue(message_path.is_file())

        message_lines = [
            line.strip()
            for line in message_path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]
        fields = [line for line in message_lines if "=" not in line]
        self.assertEqual(
            fields,
            [
                "builtin_interfaces/Time stamp",
                "uint8 mission_state",
                "uint8 control_mode",
                "uint8 selected_source",
                "uint8 speed_profile",
                "bool stop_required",
            ],
        )
        self.assertNotIn("sequence", message_path.read_text(encoding="utf-8"))

        required_constants = {
            "MISSION_STATE_WAIT_START_SIGNAL",
            "MISSION_STATE_LANE_DRIVING",
            "MISSION_STATE_CONE_SECTION",
            "CONTROL_MODE_STOP",
            "CONTROL_MODE_NORMAL_IL",
            "CONTROL_MODE_LANE_FALLBACK",
            "CONTROL_MODE_CONE_DRIVE_RULE",
            "SOURCE_NONE",
            "SOURCE_DRIVE_IL",
            "SOURCE_LANE_FALLBACK",
            "SOURCE_CONE_RULE",
            "SPEED_PROFILE_STOP",
            "SPEED_PROFILE_NORMAL",
            "SPEED_PROFILE_FALLBACK",
            "SPEED_PROFILE_CONE",
        }
        constants = {
            line.split()[1].split("=")[0]
            for line in message_lines
            if "=" in line
        }
        self.assertTrue(required_constants.issubset(constants))

        interface_manifest = ElementTree.parse(
            interface_root / "package.xml"
        ).getroot()
        self.assertEqual(
            interface_manifest.find("name").text,
            "teamkai_interfaces",
        )
        self.assertEqual(
            interface_manifest.find("member_of_group").text,
            "rosidl_interface_packages",
        )

    def test_package_installs_only_v02_runtime_assets_and_entry_points(self):
        setup_source = (PACKAGE_ROOT / "setup.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("glob('config/*.yaml')", setup_source)
        self.assertIn("['docs/MISSION_MANAGER_V02.md']", setup_source)
        self.assertIn("'launch/mission_manager_draft.launch.py'", setup_source)
        self.assertIn("'launch/traffic_light_debug.launch.py'", setup_source)
        self.assertIn("'assets/models/final.onnx'", setup_source)
        self.assertIn("'rviz/traffic_light_debug.rviz'", setup_source)
        self.assertNotIn("glob('launch/*.launch.py')", setup_source)
        self.assertNotIn("glob('assets/models/*')", setup_source)
        self.assertNotIn("glob('rviz/*.rviz')", setup_source)
        self.assertIn(
            "mission_manager = "
            "track_drive.mission.mission_manager_node:main",
            setup_source,
        )
        self.assertIn(
            "mission_start_signal_adapter = "
            "track_drive.mission.start_signal_adapter_node:main",
            setup_source,
        )
        self.assertIn(
            "mission_drive_policy_adapter = "
            "track_drive.integration.drive_policy_adapter_node:main",
            setup_source,
        )
        self.assertIn(
            "mission_lane_fallback_adapter = "
            "track_drive.integration.lane_fallback_adapter_node:main",
            setup_source,
        )
        self.assertIn(
            "mission_camera_cone_adapter = "
            "track_drive.integration.camera_cone_adapter_node:main",
            setup_source,
        )
        self.assertNotIn("lidar_cone_detector =", setup_source)
        self.assertIn(
            "mission_lidar_cone_adapter = "
            "track_drive.integration.lidar_cone_adapter_node:main",
            setup_source,
        )
        forbidden_entry_points = (
            "track_drive = track_drive.track_drive:main",
            "ai_drive_switch =",
            "switchable_cone_ai_driver =",
            "stop_line_bev_debug =",
            "school_zone_debug =",
            "intersection_debug =",
        )
        for entry_point in forbidden_entry_points:
            with self.subTest(entry_point=entry_point):
                self.assertNotIn(entry_point, setup_source)

    def test_legacy_runtime_dependencies_and_broken_launch_are_excluded(self):
        requirements = (PACKAGE_ROOT / "requirements.txt").read_text(
            encoding="utf-8"
        ).lower()
        self.assertNotIn("torch", requirements)
        self.assertNotIn("torchvision", requirements)
        self.assertFalse(
            (
                PACKAGE_ROOT
                / "launch"
                / "stop_line_bev_debug.launch.py"
            ).exists()
        )

        model_paths = (
            PACKAGE_ROOT / "track_drive" / "model_paths.py"
        ).read_text(encoding="utf-8")
        self.assertIn("cone_bc_scripted_5.pt", model_paths)
        self.assertIn("final.onnx", model_paths)
        self.assertNotIn("cnn_steering_model.pt", model_paths)
        self.assertNotIn("object_detector_model.onnx", model_paths)

    def test_launch_targets_existing_track_drive_package(self):
        launch_source = (
            PACKAGE_ROOT / "launch" / "mission_manager_draft.launch.py"
        ).read_text(encoding="utf-8")
        self.assertEqual(launch_source.count('"track_drive"'), 7)
        self.assertIn('executable="mission_manager"', launch_source)
        self.assertIn(
            'executable="mission_start_signal_adapter"',
            launch_source,
        )
        self.assertIn(
            'executable="mission_drive_policy_adapter"',
            launch_source,
        )
        self.assertIn(
            'executable="mission_lane_fallback_adapter"',
            launch_source,
        )
        self.assertIn(
            'executable="mission_camera_cone_adapter"',
            launch_source,
        )
        self.assertNotIn('executable="lidar_cone_detector"', launch_source)
        self.assertIn(
            'executable="mission_lidar_cone_adapter"',
            launch_source,
        )
        self.assertNotIn('"state_machine"', launch_source)


if __name__ == "__main__":
    unittest.main()
