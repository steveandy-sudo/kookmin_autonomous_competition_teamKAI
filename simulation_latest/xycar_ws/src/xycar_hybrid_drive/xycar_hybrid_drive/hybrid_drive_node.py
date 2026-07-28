from __future__ import annotations

import math
from pathlib import Path
import time

import cv2
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TwistStamped
from nav_msgs.msg import Path as PathMessage
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray, String
import torch

from il_data_tools.runtime_preprocessing import preprocess_bgr_image
from xycar_rl.camera_speed_models import denormalize_speed_command
from xycar_rl.policy_loader import load_camera_speed_policy
from xycar_rl.policy_runtime_node import shift_canonical_image_for_target
from xycar_rule_drive.canonical_stanley_pursuit_driver import (
    CanonicalStanleyPursuitDriver,
    path_heading_change_per_m,
)

from .hwj_cone_planner import (
    ConeCommand,
    ConePlannerConfig,
    HwjConePlanner,
    wheel_angle_to_xycar_command,
)
from .mode_manager import (
    DriveMode,
    HybridModeManager,
    ModeManagerConfig,
)


def latest_sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


def message_stamp_ns(message: Image) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class HybridDriveNode(CanonicalStanleyPursuitDriver):
    """Compute all three driving modes and publish one direct motor command."""

    def __init__(self) -> None:
        super().__init__(node_name="xycar_hybrid_drive")
        self._declare_hybrid_parameters()

        device_name = str(self.get_parameter("hybrid_device").value)
        if device_name == "auto":
            device_name = "cuda" if torch.cuda.is_available() else "cpu"
        self.policy_device = torch.device(device_name)
        cpu_threads = max(
            1, int(self.get_parameter("hybrid_policy_cpu_threads").value)
        )
        if self.policy_device.type == "cpu":
            torch.set_num_threads(cpu_threads)
            try:
                torch.set_num_interop_threads(1)
            except RuntimeError:
                pass
        cv2.setNumThreads(
            max(1, int(self.get_parameter("hybrid_opencv_threads").value))
        )

        checkpoint = Path(
            str(self.get_parameter("hybrid_checkpoint_path").value)
        ).expanduser().resolve()
        self.policy, payload = load_camera_speed_policy(
            checkpoint,
            device=self.policy_device,
        )
        self.policy_min_speed = float(
            payload.get(
                "min_speed_command",
                self.get_parameter("hybrid_model_min_speed_command").value,
            )
        )
        self.policy_max_speed = float(
            payload.get(
                "max_speed_command",
                self.get_parameter("hybrid_model_max_speed_command").value,
            )
        )
        self.policy_input_width = int(
            self.get_parameter("hybrid_policy_input_width").value
        )
        self.policy_input_height = int(
            self.get_parameter("hybrid_policy_input_height").value
        )

        self.mode_manager = HybridModeManager(
            ModeManagerConfig(
                curve_entry_per_m=float(
                    self.get_parameter(
                        "hybrid_curve_entry_curvature_per_m"
                    ).value
                ),
                curve_exit_per_m=float(
                    self.get_parameter(
                        "hybrid_curve_exit_curvature_per_m"
                    ).value
                ),
                curve_entry_angle_command=float(
                    self.get_parameter(
                        "hybrid_curve_entry_angle_command"
                    ).value
                ),
                curve_exit_angle_command=float(
                    self.get_parameter(
                        "hybrid_curve_exit_angle_command"
                    ).value
                ),
                curve_entry_frames=int(
                    self.get_parameter("hybrid_curve_entry_frames").value
                ),
                curve_exit_frames=int(
                    self.get_parameter("hybrid_curve_exit_frames").value
                ),
                cone_entry_confidence=float(
                    self.get_parameter(
                        "hybrid_cone_entry_confidence"
                    ).value
                ),
                cone_entry_frames=int(
                    self.get_parameter("hybrid_cone_entry_frames").value
                ),
                cone_min_duration_sec=float(
                    self.get_parameter(
                        "hybrid_cone_min_duration_sec"
                    ).value
                ),
                cone_exit_timeout_sec=float(
                    self.get_parameter(
                        "hybrid_cone_exit_timeout_sec"
                    ).value
                ),
                cone_lane_recovery_frames=int(
                    self.get_parameter(
                        "hybrid_cone_lane_recovery_frames"
                    ).value
                ),
            )
        )
        self.cone_planner = HwjConePlanner(self._cone_planner_config())
        self.cone_enabled = bool(
            self.get_parameter("hybrid_cone_enabled").value
        )

        self.canonical_generation = 0
        self.cone_generation = 0
        self.last_canonical_wall_sec = 0.0
        self.last_policy_wall_sec = 0.0
        self.last_policy_stamp_ns = 0
        self.last_cone_wall_sec = 0.0
        self.policy_angle_command = 0.0
        self.policy_speed_command = 0.0
        self.last_selected_angle = 0.0
        self.last_selected_speed = 0.0
        self.last_selected_mode = DriveMode.MODEL_STRAIGHT
        self.defer_motor_publish = False
        self.deferred_rule_command: tuple[float, float] | None = None
        self.latest_cone_command = ConeCommand(
            0.0, 0.0, 0.0, (), (), "none"
        )
        self.last_mode_log_sec = 0.0

        self.mode_pub = self.create_publisher(
            String,
            str(self.get_parameter("hybrid_mode_topic").value),
            10,
        )
        self.hybrid_debug_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("hybrid_debug_topic").value),
            10,
        )
        self.cone_cluster_pub = self.create_publisher(
            PoseArray,
            str(self.get_parameter("hybrid_cone_cluster_topic").value),
            10,
        )
        self.cone_path_pub = self.create_publisher(
            PathMessage,
            str(self.get_parameter("hybrid_cone_path_topic").value),
            10,
        )
        if self.cone_enabled:
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("hybrid_scan_topic").value),
                self.on_scan,
                latest_sensor_qos(),
            )

        mode = "DRIVE" if self.drive_enabled else "SHADOW"
        self.get_logger().info(
            "hybrid driver ready: "
            f"{mode}, policy={checkpoint}, device={self.policy_device}, "
            "priority=cone_rule>lane_rule_curve>model_straight"
        )

    def _declare_hybrid_parameters(self) -> None:
        default_checkpoint = (
            Path.home()
            / "xycar_kookmin_gazebo_track"
            / "xycar_ws"
            / "src"
            / "xycar_rl"
            / "models"
            / "final_rule_td3_bc_uncapped_avg17_20260723"
            / "camera_speed_td3_bc_best.pth"
        )
        self.declare_parameter(
            "hybrid_checkpoint_path",
            str(default_checkpoint),
        )
        self.declare_parameter("hybrid_device", "cpu")
        self.declare_parameter("hybrid_policy_cpu_threads", 4)
        self.declare_parameter("hybrid_opencv_threads", 1)
        self.declare_parameter("hybrid_policy_input_width", 160)
        self.declare_parameter("hybrid_policy_input_height", 90)
        self.declare_parameter("hybrid_target_lateral_offset_m", 0.0)
        self.declare_parameter("hybrid_canonical_lateral_range_m", 1.4)
        self.declare_parameter("hybrid_canonical_background_gray", 36)
        self.declare_parameter("hybrid_model_min_speed_command", 4.0)
        self.declare_parameter("hybrid_model_max_speed_command", 24.0)
        self.declare_parameter("hybrid_model_speed_cap", 10.0)
        self.declare_parameter("hybrid_model_steering_gain", 1.0)
        self.declare_parameter("hybrid_model_steering_sign", 1.0)
        self.declare_parameter("hybrid_model_timeout_sec", 0.35)
        self.declare_parameter("hybrid_skip_model_in_curve", True)
        self.declare_parameter("hybrid_temporal_reset_gap_sec", 0.25)
        self.declare_parameter("hybrid_canonical_timeout_sec", 0.50)
        self.declare_parameter("hybrid_curve_entry_curvature_per_m", 0.16)
        self.declare_parameter("hybrid_curve_exit_curvature_per_m", 0.10)
        self.declare_parameter("hybrid_curve_entry_angle_command", 10.0)
        self.declare_parameter("hybrid_curve_exit_angle_command", 6.0)
        self.declare_parameter("hybrid_curve_entry_frames", 1)
        self.declare_parameter("hybrid_curve_exit_frames", 3)
        self.declare_parameter("hybrid_curve_near_x_m", 0.15)
        self.declare_parameter("hybrid_curve_far_x_m", 1.35)
        self.declare_parameter("hybrid_cone_enabled", True)
        self.declare_parameter("hybrid_scan_topic", "/scan")
        self.declare_parameter("hybrid_cone_entry_confidence", 0.35)
        self.declare_parameter("hybrid_cone_entry_frames", 3)
        self.declare_parameter("hybrid_cone_min_duration_sec", 1.0)
        self.declare_parameter("hybrid_cone_exit_timeout_sec", 0.8)
        self.declare_parameter("hybrid_cone_exit_creep_timeout_sec", 1.4)
        self.declare_parameter("hybrid_cone_exit_creep_speed", 8.0)
        self.declare_parameter("hybrid_cone_lane_recovery_frames", 4)
        self.declare_parameter("hybrid_cone_speed_cap", 10.0)
        self.declare_parameter("hybrid_mode_transition_max_delta_cmd", 14.0)
        self.declare_parameter("hybrid_mode_topic", "/hybrid/mode")
        self.declare_parameter("hybrid_debug_topic", "/hybrid/debug")
        self.declare_parameter(
            "hybrid_cone_cluster_topic",
            "/hybrid/cone_clusters",
        )
        self.declare_parameter("hybrid_cone_path_topic", "/hybrid/cone_path")

        self.declare_parameter("hybrid_cone_max_range_m", 1.6)
        self.declare_parameter("hybrid_cone_min_range_m", 0.18)
        self.declare_parameter("hybrid_cone_scan_angle_offset_deg", 0.0)
        self.declare_parameter("hybrid_cone_dbscan_eps_m", 0.15)
        self.declare_parameter("hybrid_cone_expected_corridor_width_m", 0.85)
        self.declare_parameter("hybrid_cone_min_corridor_width_m", 0.68)
        self.declare_parameter("hybrid_cone_max_corridor_width_m", 0.98)
        self.declare_parameter("hybrid_cone_lidar_to_rear_axle_m", 0.42)
        self.declare_parameter("hybrid_cone_wheelbase_m", 0.33)
        self.declare_parameter("hybrid_cone_speed", 17.0)
        self.declare_parameter("hybrid_cone_min_drive_speed", 9.5)
        self.declare_parameter("hybrid_cone_straight_boost_speed", 21.0)

    def _cone_planner_config(self) -> ConePlannerConfig:
        return ConePlannerConfig(
            max_range_m=float(
                self.get_parameter("hybrid_cone_max_range_m").value
            ),
            min_range_m=float(
                self.get_parameter("hybrid_cone_min_range_m").value
            ),
            scan_angle_offset_deg=float(
                self.get_parameter(
                    "hybrid_cone_scan_angle_offset_deg"
                ).value
            ),
            dbscan_eps_m=float(
                self.get_parameter("hybrid_cone_dbscan_eps_m").value
            ),
            expected_corridor_width_m=float(
                self.get_parameter(
                    "hybrid_cone_expected_corridor_width_m"
                ).value
            ),
            min_corridor_width_m=float(
                self.get_parameter(
                    "hybrid_cone_min_corridor_width_m"
                ).value
            ),
            max_corridor_width_m=float(
                self.get_parameter(
                    "hybrid_cone_max_corridor_width_m"
                ).value
            ),
            lidar_to_rear_axle_m=float(
                self.get_parameter(
                    "hybrid_cone_lidar_to_rear_axle_m"
                ).value
            ),
            wheelbase_m=float(
                self.get_parameter("hybrid_cone_wheelbase_m").value
            ),
            cone_speed=float(
                self.get_parameter("hybrid_cone_speed").value
            ),
            cone_min_drive_speed=float(
                self.get_parameter(
                    "hybrid_cone_min_drive_speed"
                ).value
            ),
            cone_straight_boost_speed=float(
                self.get_parameter(
                    "hybrid_cone_straight_boost_speed"
                ).value
            ),
        )

    def on_canonical(self, message: Image) -> None:
        now = time.monotonic()
        self.canonical_generation += 1
        self.last_canonical_wall_sec = now
        self.defer_motor_publish = True
        self.deferred_rule_command = None
        try:
            super().on_canonical(message)
        finally:
            self.defer_motor_publish = False

        if self.deferred_rule_command is None:
            return
        rule_angle, rule_speed = self.deferred_rule_command
        if self._model_needed_for_current_frame(rule_angle):
            try:
                image = self.bridge.imgmsg_to_cv2(
                    message,
                    desired_encoding="bgr8",
                )
                self._update_policy_candidate(image, message, now)
            except Exception as exc:
                if now - self.last_mode_log_sec > 2.0:
                    self.get_logger().warn(
                        f"hybrid policy inference failed: {exc}"
                    )
                    self.last_mode_log_sec = now
        self._select_and_publish(rule_angle, rule_speed)

    def _model_needed_for_current_frame(self, rule_angle: float) -> bool:
        if not bool(
            self.get_parameter("hybrid_skip_model_in_curve").value
        ):
            return True
        cone = self.latest_cone_command
        cone_about_to_enter = (
            cone.confidence
            >= float(
                self.get_parameter("hybrid_cone_entry_confidence").value
            )
            and len(cone.clusters) >= 2
            and self.mode_manager.cone_frames
            >= max(
                0,
                int(
                    self.get_parameter(
                        "hybrid_cone_entry_frames"
                    ).value
                )
                - 1,
            )
        )
        if (
            self.mode_manager.mode == DriveMode.CONE_RULE
            or cone_about_to_enter
        ):
            return False
        curvature = self._path_curvature()
        curve_now = self.path_valid and (
            curvature
            >= float(
                self.get_parameter(
                    "hybrid_curve_entry_curvature_per_m"
                ).value
            )
            or abs(float(rule_angle))
            >= float(
                self.get_parameter(
                    "hybrid_curve_entry_angle_command"
                ).value
            )
        )
        return not curve_now

    def _update_policy_candidate(
        self,
        image: np.ndarray,
        message: Image,
        now: float,
    ) -> None:
        stamp_ns = message_stamp_ns(message)
        if self.last_policy_stamp_ns > 0 and stamp_ns > 0:
            gap_sec = (stamp_ns - self.last_policy_stamp_ns) / 1.0e9
            reset_gap = float(
                self.get_parameter("hybrid_temporal_reset_gap_sec").value
            )
            if gap_sec <= 0.0 or (reset_gap > 0.0 and gap_sec > reset_gap):
                self.policy.reset()
        shifted_image = shift_canonical_image_for_target(
            image,
            float(
                self.get_parameter(
                    "hybrid_target_lateral_offset_m"
                ).value
            ),
            float(
                self.get_parameter(
                    "hybrid_canonical_lateral_range_m"
                ).value
            ),
            int(
                self.get_parameter(
                    "hybrid_canonical_background_gray"
                ).value
            ),
        )
        model_image = preprocess_bgr_image(
            shifted_image,
            self.policy_input_width,
            self.policy_input_height,
        )
        action = self.policy({"image": model_image})
        normalized_angle = float(
            np.clip(
                float(action[0])
                * float(
                    self.get_parameter(
                        "hybrid_model_steering_gain"
                    ).value
                )
                * float(
                    self.get_parameter(
                        "hybrid_model_steering_sign"
                    ).value
                ),
                -1.0,
                1.0,
            )
        )
        learned_speed = denormalize_speed_command(
            float(action[1]),
            self.policy_min_speed,
            self.policy_max_speed,
        )
        speed_cap = float(
            self.get_parameter("hybrid_model_speed_cap").value
        )
        if speed_cap > 0.0:
            learned_speed = min(learned_speed, speed_cap)
        self.policy_angle_command = normalized_angle * max(
            abs(self.angle_command_min),
            abs(self.angle_command_max),
        )
        self.policy_speed_command = learned_speed
        self.last_policy_wall_sec = now
        self.last_policy_stamp_ns = stamp_ns

    def on_scan(self, message: LaserScan) -> None:
        self.cone_generation += 1
        self.latest_cone_command = self.cone_planner.process_scan(
            message.ranges,
            message.intensities,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
        )
        self.last_cone_wall_sec = time.monotonic()
        self._publish_cone_debug(message)

    def publish_motor(self, rule_angle: float, rule_speed: float) -> None:
        if self.defer_motor_publish:
            self.deferred_rule_command = (
                float(rule_angle),
                float(rule_speed),
            )
            return
        self._select_and_publish(rule_angle, rule_speed)

    def _select_and_publish(
        self,
        rule_angle: float,
        rule_speed: float,
    ) -> None:
        now = time.monotonic()
        policy_age = (
            now - self.last_policy_wall_sec
            if self.last_policy_wall_sec > 0.0
            else float("inf")
        )
        cone_age = (
            now - self.last_cone_wall_sec
            if self.last_cone_wall_sec > 0.0
            else float("inf")
        )
        curvature = self._path_curvature()
        decision = self.mode_manager.update(
            now_sec=now,
            canonical_generation=self.canonical_generation,
            cone_generation=self.cone_generation,
            lane_available=bool(self.path_valid),
            model_available=(
                policy_age
                <= float(
                    self.get_parameter("hybrid_model_timeout_sec").value
                )
            ),
            path_curvature_per_m=curvature,
            rule_angle_command=float(rule_angle),
            cone_confidence=float(self.latest_cone_command.confidence),
            cone_age_sec=cone_age,
            cone_cluster_count=len(self.latest_cone_command.clusters),
        )

        angle, speed = self._command_for_mode(
            decision.mode,
            rule_angle=float(rule_angle),
            rule_speed=float(rule_speed),
            policy_age=policy_age,
            cone_age=cone_age,
        )
        canonical_stale = (
            self.last_canonical_wall_sec <= 0.0
            or now - self.last_canonical_wall_sec
            > float(
                self.get_parameter("hybrid_canonical_timeout_sec").value
            )
        )
        fresh_cone_override = (
            decision.mode == DriveMode.CONE_RULE
            and cone_age
            <= float(
                self.get_parameter("hybrid_cone_exit_creep_timeout_sec").value
            )
        )
        if canonical_stale and not fresh_cone_override:
            angle, speed = 0.0, 0.0
            reason = "canonical timeout stop"
        else:
            reason = decision.reason

        if decision.changed and (
            decision.mode == DriveMode.CONE_RULE
            or self.last_selected_mode == DriveMode.CONE_RULE
        ):
            maximum_delta = max(
                0.0,
                float(
                    self.get_parameter(
                        "hybrid_mode_transition_max_delta_cmd"
                    ).value
                ),
            )
            angle = float(
                np.clip(
                    angle,
                    self.last_selected_angle - maximum_delta,
                    self.last_selected_angle + maximum_delta,
                )
            )

        self.last_selected_angle = float(angle)
        self.last_selected_speed = float(speed)
        self.last_selected_mode = decision.mode
        # Parent tracking and latency compensation must use the command actually
        # sent, not the curve-rule candidate that called this override.
        self.last_angle_command = float(angle)
        self.last_speed_command = float(speed)

        message = Float32MultiArray()
        message.data = [float(angle), float(speed)]
        self.shadow_motor_pub.publish(message)
        if self.motor_pub is not None:
            self.motor_pub.publish(message)
        if self.latest_header is not None:
            trace = TwistStamped()
            trace.header = self.latest_header
            trace.twist.angular.z = float(angle)
            trace.twist.linear.x = float(speed)
            self.action_trace_pub.publish(trace)
        self._publish_hybrid_status(
            decision.mode,
            reason,
            angle,
            speed,
            rule_angle,
            rule_speed,
            curvature,
            policy_age,
            cone_age,
        )
        if decision.changed:
            self.get_logger().info(
                f"hybrid mode -> {decision.mode.name}: {reason}"
            )

    def _command_for_mode(
        self,
        mode: DriveMode,
        *,
        rule_angle: float,
        rule_speed: float,
        policy_age: float,
        cone_age: float,
    ) -> tuple[float, float]:
        if mode == DriveMode.MODEL_STRAIGHT:
            if policy_age <= float(
                self.get_parameter("hybrid_model_timeout_sec").value
            ):
                return self.policy_angle_command, self.policy_speed_command
            return rule_angle, rule_speed
        if mode == DriveMode.LANE_RULE_CURVE:
            return rule_angle, rule_speed

        command = self.latest_cone_command
        exit_timeout = float(
            self.get_parameter("hybrid_cone_exit_timeout_sec").value
        )
        if cone_age <= exit_timeout and command.confidence > 0.20:
            speed = command.speed_command
        elif cone_age <= float(
            self.get_parameter(
                "hybrid_cone_exit_creep_timeout_sec"
            ).value
        ):
            speed = float(
                self.get_parameter("hybrid_cone_exit_creep_speed").value
            )
        else:
            return 0.0, 0.0
        cap = float(self.get_parameter("hybrid_cone_speed_cap").value)
        if cap > 0.0:
            speed = min(speed, cap)
        return wheel_angle_to_xycar_command(command.wheel_angle_deg), speed

    def _path_curvature(self) -> float:
        if self.latest_path is None or len(self.latest_path) < 3:
            return 0.0
        curvature = path_heading_change_per_m(
            self.latest_path,
            near_x_m=float(
                self.get_parameter("hybrid_curve_near_x_m").value
            ),
            far_x_m=float(
                self.get_parameter("hybrid_curve_far_x_m").value
            ),
        )
        return float(curvature) if math.isfinite(curvature) else 0.0

    def _publish_hybrid_status(
        self,
        mode: DriveMode,
        reason: str,
        angle: float,
        speed: float,
        rule_angle: float,
        rule_speed: float,
        curvature: float,
        policy_age: float,
        cone_age: float,
    ) -> None:
        status = String()
        status.data = (
            f"{mode.name} | {reason} | angle={angle:.2f} speed={speed:.2f} "
            f"| curve={curvature:.3f}/m | cone={self.latest_cone_command.confidence:.2f}"
        )
        self.mode_pub.publish(status)
        debug = Float32MultiArray()
        debug.data = [
            float(mode),
            float(angle),
            float(speed),
            float(self.policy_angle_command),
            float(self.policy_speed_command),
            float(rule_angle),
            float(rule_speed),
            float(
                wheel_angle_to_xycar_command(
                    self.latest_cone_command.wheel_angle_deg
                )
            ),
            float(self.latest_cone_command.speed_command),
            float(self.latest_cone_command.confidence),
            float(curvature),
            float(policy_age),
            float(cone_age),
        ]
        self.hybrid_debug_pub.publish(debug)

    def _publish_cone_debug(self, scan: LaserScan) -> None:
        clusters = PoseArray()
        clusters.header = scan.header
        clusters.header.frame_id = scan.header.frame_id or "laser_frame"
        for x, y in self.latest_cone_command.clusters:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            clusters.poses.append(pose)
        self.cone_cluster_pub.publish(clusters)

        path = PathMessage()
        path.header = scan.header
        path.header.frame_id = "rear_axle"
        for x, y in self.latest_cone_command.path:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            path.poses.append(pose)
        self.cone_path_pub.publish(path)

    def destroy_node(self):
        if self.drive_enabled and rclpy.ok():
            message = Float32MultiArray()
            message.data = [0.0, 0.0]
            if self.motor_pub is not None:
                try:
                    self.motor_pub.publish(message)
                except Exception:
                    pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HybridDriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass


if __name__ == "__main__":
    main()
