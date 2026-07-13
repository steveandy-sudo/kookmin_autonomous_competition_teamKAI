import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from il_data_tools.domain_randomization import generate_randomized_assets


def _prepare_environment(context):
    project_root = Path(LaunchConfiguration("project_root").perform(context)).resolve()
    output_dir = Path(LaunchConfiguration("generated_output_dir").perform(context)).resolve()
    seed = int(LaunchConfiguration("seed").perform(context))
    preset = LaunchConfiguration("preset").perform(context)
    source_world = LaunchConfiguration("source_world").perform(context)
    source_bridge = LaunchConfiguration("source_bridge_config").perform(context)
    output_dir.mkdir(parents=True, exist_ok=True)

    stem = f"{preset}_seed_{seed}"
    generated_world = output_dir / f"{stem}.sdf"
    generated_bridge = output_dir / f"{stem}_bridge.yaml"
    manifest = output_dir / f"{stem}_manifest.json"
    event_log = output_dir / f"{stem}_scenario_events.jsonl"
    if event_log.exists():
        event_log.unlink()

    result = generate_randomized_assets(
        source_world=source_world,
        output_world=str(generated_world),
        source_bridge_config=source_bridge,
        output_bridge_config=str(generated_bridge),
        manifest_path=str(manifest),
        seed=seed,
        preset=preset,
    )
    result["scenario_event_log"] = str(event_log)
    manifest.write_text(
        json.dumps(result, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    context.launch_configurations["generated_world"] = str(generated_world)
    context.launch_configurations["generated_bridge_config"] = str(generated_bridge)
    context.launch_configurations["run_manifest_path"] = str(manifest)
    context.launch_configurations["scenario_event_log"] = str(event_log)
    print(
        "[domain-randomization] "
        f"preset={result['preset']} seed={result['seed']} world={generated_world}"
    )

    show_gui = LaunchConfiguration("show_gui").perform(context).lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    command = ["gz", "sim", "-r", str(generated_world)]
    if not show_gui:
        command.insert(2, "-s")
    return [
        ExecuteProcess(
            cmd=command,
            name="kookmin_randomized_gazebo",
            output="screen",
        )
    ]


def generate_launch_description():
    project_root = LaunchConfiguration("project_root")
    output_root = LaunchConfiguration("output_root")
    session_name = LaunchConfiguration("session_name")
    max_samples = LaunchConfiguration("max_samples")
    seed = LaunchConfiguration("seed")
    preset = LaunchConfiguration("preset")
    show_gui = LaunchConfiguration("show_gui")
    camera_front_topic = LaunchConfiguration("camera_front_topic")
    image_format = LaunchConfiguration("image_format")
    max_save_rate_hz = LaunchConfiguration("max_save_rate_hz")

    bridge_launch = PathJoinSubstitution(
        [FindPackageShare("xycar_gazebo_bridge"), "launch", "xycar_gazebo_rviz.launch.py"]
    )
    rule_launch = PathJoinSubstitution(
        [FindPackageShare("xycar_rule_drive"), "launch", "lane_rule_driver.launch.py"]
    )
    recorder = Node(
        package="il_data_tools",
        executable="il_common_recorder",
        name="il_common_recorder_drive",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "output_root": output_root,
                "session_name": session_name,
                "session_auto_increment": True,
                "dataset_profile": "drive",
                "allowed_labels": "general_drive,lane_drive,hill_drive,shortcut,recovery",
                "camera_front_topic": camera_front_topic,
                "scan_topic": "/scan",
                "motor_topic": "/xycar_motor",
                "motor_msg_type": "float32_multi_array",
                "mission_label_topic": "/il/mission_label",
                "default_mission_label": "general_drive",
                "run_manifest_path": LaunchConfiguration("run_manifest_path"),
                "save_front_image": True,
                "save_scan_npz": True,
                "require_scan": True,
                "approximate_sync_tolerance_sec": 0.05,
                "sync_wait_sec": 0.10,
                "writer_queue_size": 128,
                "min_free_disk_gb": 10.0,
                "disk_check_period_sec": 5.0,
                "stop_on_low_disk": True,
                "image_format": image_format,
                "jpeg_quality": 90,
                "max_save_rate_hz": ParameterValue(
                    max_save_rate_hz, value_type=float
                ),
                "enable_recording_on_start": True,
                "exclude_bad_data": True,
                "exclude_idle": True,
                "exclude_zero_speed": True,
                "bad_data_preroll_sec": 3.0,
                "max_samples": ParameterValue(max_samples, value_type=int),
                "exit_on_limit_reached": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "xycar_kookmin_gazebo_track"]
                ),
            ),
            DeclareLaunchArgument(
                "source_world",
                default_value=PathJoinSubstitution(
                    [project_root, "worlds", "kookmin_xycar_track_final.sdf"]
                ),
            ),
            DeclareLaunchArgument(
                "source_bridge_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("xycar_gazebo_bridge"),
                        "config",
                        "xycar_gazebo_bridge.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "generated_output_dir",
                default_value=PathJoinSubstitution(
                    [project_root, "generated", "domain_randomization"]
                ),
            ),
            DeclareLaunchArgument(
                "output_root",
                default_value=PathJoinSubstitution([project_root, "datasets", "il"]),
            ),
            DeclareLaunchArgument("session_name", default_value="sim_randomized"),
            DeclareLaunchArgument("max_samples", default_value="5000"),
            DeclareLaunchArgument(
                "camera_front_topic",
                default_value="/image_raw",
                description="Raw RGB or canonical BEV image stored by the recorder.",
            ),
            DeclareLaunchArgument(
                "image_format",
                default_value="jpg",
                description="Use png for canonical semantic images.",
            ),
            DeclareLaunchArgument("max_save_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("seed", default_value="2026"),
            DeclareLaunchArgument(
                "preset",
                default_value="mixed",
                description="baseline, visual_light, visual_dark, sensor, dynamics, or mixed",
            ),
            DeclareLaunchArgument(
                "show_gui",
                default_value="true",
                description="Show Gazebo and RViz for inspection; false is faster for batches.",
            ),
            DeclareLaunchArgument("generated_world", default_value=""),
            DeclareLaunchArgument("generated_bridge_config", default_value=""),
            DeclareLaunchArgument("run_manifest_path", default_value=""),
            DeclareLaunchArgument("scenario_event_log", default_value=""),
            DeclareLaunchArgument("scenario_interval_sec", default_value="30.0"),
            DeclareLaunchArgument("scenario_warmup_sec", default_value="12.0"),
            DeclareLaunchArgument("scenario_settle_sec", default_value="0.8"),
            DeclareLaunchArgument("recovery_hold_sec", default_value="8.0"),
            DeclareLaunchArgument(
                "lane_offset_from_yellow_m",
                default_value="0.05",
                description="Nominal rule path offset from the yellow centerline.",
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    project_root,
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            OpaqueFunction(function=_prepare_environment),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
                launch_arguments={
                    "params_file": LaunchConfiguration("generated_bridge_config"),
                    "enable_rviz": show_gui,
                }.items(),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(rule_launch)),
            Node(
                package="il_data_tools",
                executable="il_recovery_scenario_manager",
                name="il_recovery_scenario_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "seed": ParameterValue(seed, value_type=int),
                        "preset": preset,
                        "event_log_path": LaunchConfiguration("scenario_event_log"),
                        "warmup_sec": ParameterValue(
                            LaunchConfiguration("scenario_warmup_sec"), value_type=float
                        ),
                        "settle_sec": ParameterValue(
                            LaunchConfiguration("scenario_settle_sec"), value_type=float
                        ),
                        "scenario_interval_sec": ParameterValue(
                            LaunchConfiguration("scenario_interval_sec"), value_type=float
                        ),
                        "recovery_hold_sec": ParameterValue(
                            LaunchConfiguration("recovery_hold_sec"), value_type=float
                        ),
                        "lane_offset_from_yellow_m": ParameterValue(
                            LaunchConfiguration("lane_offset_from_yellow_m"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            recorder,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=recorder,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="randomized IL session reached its sample limit"
                            )
                        )
                    ],
                )
            ),
        ]
    )
