#!/usr/bin/env python3
"""Sequential Nav2 parking mission with localization and actuation gates."""

from __future__ import annotations

import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (
    Point,
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .mission_core import (
    ForwardProgressWatchdog,
    LocalizationGate,
    LocalizationGateConfig,
    MissionStep,
    Pose2D,
    assess_transit_waypoint_pass,
    assess_mission_time,
    direct_reverse_parking_command,
    is_reverse_fallback_candidate,
    mission_steps_from_dicts,
    pose_error,
    reference_pose_to_base,
)


STATE_KO = {
    "LOCALIZING": "위치추정 안정화 중",
    "READY": "시작 준비 완료",
    "RUNNING": "주차 미션 주행 중",
    "HOLDING": "목표 지점 정지 확인 중",
    "PAUSED_LOCALIZATION": "위치추정 이상으로 일시정지",
    "RECOVERING": "장애물 우회 경로 재탐색 중",
    "COMPLETED": "주차 미션 완료",
    "ABORTED": "주차 미션 중단",
}

REASON_KO = {
    "": "",
    "ok": "AMCL 연속 정상 표본 수가 아직 부족함",
    "no_pose": "AMCL 위치정보가 아직 없음",
    "bad_covariance": "AMCL 오차정보 형식이 올바르지 않음",
    "non_finite": "AMCL 위치정보에 유효하지 않은 값이 있음",
    "bad_stamp": "AMCL 시간정보가 올바르지 않음",
    "stale_pose": "AMCL 위치정보가 오래되어 갱신 필요",
    "xy_uncertain": "AMCL 위치 오차가 허용범위보다 큼",
    "yaw_uncertain": "AMCL 방향 오차가 허용범위보다 큼",
    "non_monotonic_stamp": "AMCL 시간정보 순서가 뒤바뀜",
    "position_jump": "AMCL 위치가 갑자기 크게 변함",
    "yaw_jump": "AMCL 방향이 갑자기 크게 변함",
    "route_localization_not_ready": "경로 기반 LiDAR 초기 정합이 아직 완료되지 않음",
    "operator_abort": "운전자 중단 요청",
    "operator_reset": "운전자 초기화 요청",
    "waiting_for_nav2_action_server": "Nav2 경로주행 서버 시작 대기 중",
    "waiting_for_nav2_activation": "Nav2 전체 활성화 대기 중(재시도 횟수 차감 안 함)",
    "nav2_activation_timeout": "Nav2가 제한시간 안에 활성화되지 않아 안전 중단",
    "goal_rejected": "Nav2가 목표를 거부함",
    "no_final_pose": "목표 도착 후 AMCL 위치정보가 없음",
    "goal_cancelled_for_localization": "위치추정 이상으로 현재 목표 취소",
    "aligned_completion_waiting_for_localization": (
        "주차점/최종점 도착을 확정할 정합 상태가 아니어서 같은 목표 재검증 대기"
    ),
    "returned_to_start": "출발지 복귀 완료",
    "mission_time_limit": "경기 제한시간을 초과하여 안전 중단",
    "amcl_stable": "AMCL 위치추정이 안정됨",
    "localization_recovered": "AMCL 위치추정이 다시 정상화됨",
    "automatic_recovery": "짧은 재계획 간격 후 같은 단계에서 우회 경로 자동 재시도",
    "forward_path_failed_reverse_fallback": (
        "전진 경로를 찾지 못해 같은 목표를 후진 허용 경로로 재계획"
    ),
    "forward_progress_stalled_reverse_fallback": (
        "전진 진행이 멈춰 같은 목표를 후진 허용 경로로 재계획"
    ),
    "reverse_fallback_active": "전진 경로 실패 후 후진 허용 우회경로 주행 중",
    "lidar_obstacle_reverse_recovery": (
        "LiDAR 장애물 정지 후 짧게 후진하고 같은 단계를 재계산하는 중"
    ),
    "transit_waypoint_passed": "일반 웨이포인트 통과를 확인해 다음 단계로 진행",
    "direct_reverse_parking": (
        "Nav2를 우회해 AMCL 오차를 보며 주차점까지 직접 연속 후진 중"
    ),
}


def reason_to_korean(reason: str) -> str:
    if reason in REASON_KO:
        return REASON_KO[reason]
    if reason.endswith("_automatic_recovery"):
        return "경로주행 실패 후 잠시 정지하고 같은 단계에서 우회 경로 자동 재시도"
    if reason.startswith("sending_"):
        return f"{reason.removeprefix('sending_')} 목표 전송"
    if reason.startswith("reached_"):
        return f"{reason.removeprefix('reached_')} 도착"
    if reason.startswith("nav2_status_"):
        return f"Nav2 주행 실패(상태코드 {reason.removeprefix('nav2_status_')})"
    if reason.startswith("goal_error_xy_"):
        return "목표 도착 오차가 허용범위를 벗어남"
    if reason.endswith("_retries_exhausted"):
        return "단계별 재시도 횟수를 모두 사용하여 미션 중단"
    if "_retry_" in reason:
        retry = reason.rsplit("_retry_", 1)[-1]
        return f"목표 주행 실패로 {retry}번째 재시도 대기"
    return reason


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def yaw_from_quaternion(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class ParkingMissionManager(Node):
    STATES = {
        "LOCALIZING",
        "READY",
        "RUNNING",
        "HOLDING",
        "PAUSED_LOCALIZATION",
        "RECOVERING",
        "COMPLETED",
        "ABORTED",
    }

    def __init__(self) -> None:
        super().__init__("parking_mission_manager")
        self._declare_parameters()
        share_dir = Path(get_package_share_directory("xycar_parking_nav"))
        mission_path = str(self.get_parameter("mission_config").value).strip()
        if not mission_path:
            mission_path = str(share_dir / "config" / "parking_mission.yaml")
        behavior_tree = str(self.get_parameter("behavior_tree").value).strip()
        if not behavior_tree:
            behavior_tree = str(
                share_dir / "behavior_trees" / "ackermann_navigate_to_pose.xml"
            )
        self.behavior_tree = behavior_tree
        forward_behavior_tree = str(
            self.get_parameter("forward_behavior_tree").value
        ).strip()
        if not forward_behavior_tree:
            forward_behavior_tree = str(
                share_dir
                / "behavior_trees"
                / "ackermann_forward_navigate_to_pose.xml"
            )
        self.forward_behavior_tree = forward_behavior_tree
        precise_forward_behavior_tree = str(
            self.get_parameter("precise_forward_behavior_tree").value
        ).strip()
        if not precise_forward_behavior_tree:
            precise_forward_behavior_tree = str(
                share_dir
                / "behavior_trees"
                / "ackermann_forward_precise_navigate_to_pose.xml"
            )
        self.precise_forward_behavior_tree = precise_forward_behavior_tree
        precise_reverse_behavior_tree = str(
            self.get_parameter("precise_reverse_behavior_tree").value
        ).strip()
        if not precise_reverse_behavior_tree:
            precise_reverse_behavior_tree = str(
                share_dir
                / "behavior_trees"
                / "ackermann_bidirectional_precise_navigate_to_pose.xml"
            )
        self.precise_reverse_behavior_tree = precise_reverse_behavior_tree
        parking_reverse_behavior_tree = str(
            self.get_parameter("parking_reverse_behavior_tree").value
        ).strip()
        if not parking_reverse_behavior_tree:
            parking_reverse_behavior_tree = str(
                share_dir
                / "behavior_trees"
                / "ackermann_reverse_parking_navigate_to_pose.xml"
            )
        self.parking_reverse_behavior_tree = parking_reverse_behavior_tree
        reverse_only_transit_behavior_tree = str(
            self.get_parameter("reverse_only_transit_behavior_tree").value
        ).strip()
        if not reverse_only_transit_behavior_tree:
            reverse_only_transit_behavior_tree = str(
                share_dir
                / "behavior_trees"
                / "ackermann_reverse_only_transit_navigate_to_pose.xml"
            )
        self.reverse_only_transit_behavior_tree = (
            reverse_only_transit_behavior_tree
        )
        reverse_fallback_behavior_tree = str(
            self.get_parameter("reverse_fallback_behavior_tree").value
        ).strip()
        if not reverse_fallback_behavior_tree:
            reverse_fallback_behavior_tree = str(
                share_dir
                / "behavior_trees"
                / "ackermann_reverse_transit_navigate_to_pose.xml"
            )
        self.reverse_fallback_behavior_tree = reverse_fallback_behavior_tree
        self.frame_id, self.base_offset, self.initial_reference, self.steps = (
            self._load_mission(mission_path)
        )
        self.base_goals = [
            reference_pose_to_base(step.reference_pose, self.base_offset)
            for step in self.steps
        ]

        self.gate = LocalizationGate(
            LocalizationGateConfig(
                maximum_xy_variance=float(
                    self.get_parameter("maximum_xy_variance").value
                ),
                maximum_yaw_variance=float(
                    self.get_parameter("maximum_yaw_variance").value
                ),
                maximum_pose_age_sec=float(
                    self.get_parameter("maximum_pose_age_sec").value
                ),
                maximum_position_jump_m=float(
                    self.get_parameter("maximum_position_jump_m").value
                ),
                maximum_yaw_jump_rad=float(
                    self.get_parameter("maximum_yaw_jump_rad").value
                ),
                required_stable_samples=int(
                    self.get_parameter("required_stable_samples").value
                ),
            )
        )
        self.localization_loss_cancel_sec = float(
            self.get_parameter("localization_loss_cancel_sec").value
        )
        self.position_tolerance = float(
            self.get_parameter("goal_position_tolerance_m").value
        )
        self.yaw_tolerance = float(
            self.get_parameter("goal_yaw_tolerance_rad").value
        )
        self.transit_position_tolerance = float(
            self.get_parameter("transit_goal_position_tolerance_m").value
        )
        self.transit_yaw_tolerance = float(
            self.get_parameter("transit_goal_yaw_tolerance_rad").value
        )
        self.transit_pass_radius = float(
            self.get_parameter("transit_pass_radius_m").value
        )
        self.transit_pass_maximum_miss_distance = float(
            self.get_parameter("transit_pass_maximum_miss_distance_m").value
        )
        self.transit_pass_lateral_tolerance = float(
            self.get_parameter("transit_pass_lateral_tolerance_m").value
        )
        self.action_server_timeout = float(
            self.get_parameter("action_server_timeout_sec").value
        )
        self.nav2_activation_timeout = float(
            self.get_parameter("nav2_activation_timeout_sec").value
        )
        self.retry_delay = float(self.get_parameter("retry_delay_sec").value)
        self.forward_failure_reverse_fallback_enabled = bool(
            self.get_parameter("forward_failure_reverse_fallback_enabled").value
        )
        self.forward_progress_watchdog = ForwardProgressWatchdog(
            timeout_sec=float(
                self.get_parameter("forward_no_progress_timeout_sec").value
            ),
            minimum_improvement_m=float(
                self.get_parameter(
                    "forward_progress_minimum_improvement_m"
                ).value
            ),
        )
        self.replan_authorization_grace = float(
            self.get_parameter("replan_authorization_grace_sec").value
        )
        self.automatic_recovery_enabled = bool(
            self.get_parameter("automatic_recovery_enabled").value
        )
        self.automatic_recovery_delay = float(
            self.get_parameter("automatic_recovery_delay_sec").value
        )
        self.obstacle_reverse_replan_delay = float(
            self.get_parameter("obstacle_reverse_replan_delay_sec").value
        )
        self.obstacle_reverse_authorization_sec = float(
            self.get_parameter("obstacle_reverse_authorization_sec").value
        )
        self.mission_time_limit = float(
            self.get_parameter("mission_time_limit_sec").value
        )
        self.mission_time_limit_enforced = bool(
            self.get_parameter("mission_time_limit_enforced").value
        )
        self.time_warning_remaining = float(
            self.get_parameter("time_warning_remaining_sec").value
        )
        self.time_log_period = float(
            self.get_parameter("time_log_period_sec").value
        )
        self.diagnostic_log_period = float(
            self.get_parameter("diagnostic_log_period_sec").value
        )
        self.visualization_topic = str(
            self.get_parameter("visualization_topic").value
        )
        self.parking_box_length = float(
            self.get_parameter("parking_box_length_m").value
        )
        self.parking_box_width = float(
            self.get_parameter("parking_box_width_m").value
        )
        self.nomotion_update_period = float(
            self.get_parameter("nomotion_update_period_sec").value
        )
        self.nomotion_update_pose_silence = float(
            self.get_parameter("nomotion_update_pose_silence_sec").value
        )
        self.direct_reverse_parking_enabled = bool(
            self.get_parameter("direct_reverse_parking_enabled").value
        )
        self.direct_reverse_goal_names = {
            str(name)
            for name in self.get_parameter(
                "direct_reverse_parking_goal_names"
            ).value
        }
        self.direct_reverse_speed = float(
            self.get_parameter("direct_reverse_speed_mps").value
        )
        self.direct_reverse_heading_gain = float(
            self.get_parameter("direct_reverse_heading_gain").value
        )
        self.direct_reverse_maximum_curvature = float(
            self.get_parameter("direct_reverse_maximum_curvature").value
        )
        if self.mission_time_limit <= 0.0:
            raise ValueError("mission time limit must be positive")
        if self.nav2_activation_timeout <= 0.0:
            raise ValueError("Nav2 activation timeout must be positive")
        if not 0.0 <= self.time_warning_remaining < self.mission_time_limit:
            raise ValueError("time warning must be within the mission time limit")
        if self.time_log_period <= 0.0:
            raise ValueError("time log period must be positive")
        if self.diagnostic_log_period <= 0.0:
            raise ValueError("diagnostic log period must be positive")
        if self.parking_box_length <= 0.0 or self.parking_box_width <= 0.0:
            raise ValueError("parking visualization box dimensions must be positive")
        if self.automatic_recovery_delay <= 0.0:
            raise ValueError("automatic recovery delay must be positive")
        if self.obstacle_reverse_replan_delay <= 0.0:
            raise ValueError("obstacle reverse replan delay must be positive")
        if (
            self.obstacle_reverse_authorization_sec
            <= self.obstacle_reverse_replan_delay
        ):
            raise ValueError(
                "obstacle reverse authorization must exceed replan delay"
            )
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self.transit_pass_radius,
                self.transit_pass_maximum_miss_distance,
                self.transit_pass_lateral_tolerance,
                self.replan_authorization_grace,
            )
        ):
            raise ValueError("transit pass and replan grace limits must be positive")
        if self.nomotion_update_period <= 0.0:
            raise ValueError("AMCL no-motion update period must be positive")
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self.direct_reverse_speed,
                self.direct_reverse_heading_gain,
                self.direct_reverse_maximum_curvature,
            )
        ):
            raise ValueError("direct reverse parking limits must be positive")
        if not (
            0.0 < self.nomotion_update_pose_silence
            < self.gate.config.maximum_pose_age_sec
        ):
            raise ValueError(
                "AMCL no-motion pose silence must be positive and shorter "
                "than the pose age limit"
            )
        self.autostart_mission = bool(
            self.get_parameter("autostart_mission").value
        )
        self.require_route_localization = bool(
            self.get_parameter("require_route_localization").value
        )
        self.route_localization_ready = not self.require_route_localization

        self.state = "LOCALIZING"
        self.state_reason = ""
        self.current_index = 0
        self.current_retry = 0
        self.reverse_fallback_active = False
        self.replan_authorized_until = 0.0
        self.current_pose: Pose2D | None = None
        self.last_amcl_received_at: float | None = None
        self.localization_ready = False
        self.localization_reason = "no_pose"
        self.localization_lost_since: float | None = None
        self.hold_until: float | None = None
        self.retry_at: float | None = None
        self.mission_started_at: float | None = None
        self.mission_finished_at: float | None = None
        self.next_time_log_at: float | None = None
        self.next_step_diagnostic_at = 0.0
        self.time_warning_emitted = False
        self.nav2_first_goal_accepted = False
        self.nav2_activation_deadline: float | None = None
        self.start_requested = self.autostart_mission
        self.goal_handle = None
        self.goal_active = False
        self.direct_parking_active = False
        self.goal_serial = 0
        self.cancel_for_localization = False
        self.initial_pose_remaining = int(
            self.get_parameter("initial_pose_publish_count").value
        )
        self.next_initial_pose_publish = 0.0
        self.initial_pose_period = float(
            self.get_parameter("initial_pose_publish_period_sec").value
        )
        self.next_nomotion_update = 0.0
        self.nomotion_update_future = None
        self.nomotion_update_error_logged = False

        self.authorization_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("authorization_topic").value),
            10,
        )
        self.state_publisher = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self.direct_cmd_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("direct_cmd_vel_topic").value),
            10,
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initial_pose_topic").value),
            10,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_array_publisher = self.create_publisher(
            PoseArray,
            str(self.get_parameter("goal_pose_array_topic").value),
            latched_qos,
        )
        self.visualization_publisher = self.create_publisher(
            MarkerArray,
            self.visualization_topic,
            latched_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("amcl_pose_topic").value),
            self._on_amcl_pose,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("obstacle_recovery_request_topic").value),
            self._on_obstacle_recovery_request,
            10,
        )
        if self.require_route_localization:
            self.create_subscription(
                Bool,
                str(self.get_parameter("route_localization_ready_topic").value),
                self._on_route_localization_ready,
                latched_qos,
            )
        self.create_service(Trigger, "~/start", self._on_start)
        self.create_service(Trigger, "~/abort", self._on_abort)
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.nomotion_update_client = self.create_client(
            Empty,
            str(self.get_parameter("amcl_nomotion_update_service").value),
        )
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_to_pose_action").value),
        )
        self.create_timer(0.10, self._on_timer)
        self._publish_goal_array()
        self._publish_visualization()
        self.get_logger().info(
            "%d단계 주차 미션을 불러왔습니다: 설정=%s, 차량 기준점 보정=%.3f m"
            % (len(self.steps), mission_path, self.base_offset)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("mission_config", "")
        self.declare_parameter("behavior_tree", "")
        self.declare_parameter("forward_behavior_tree", "")
        self.declare_parameter("precise_forward_behavior_tree", "")
        self.declare_parameter("precise_reverse_behavior_tree", "")
        self.declare_parameter("parking_reverse_behavior_tree", "")
        self.declare_parameter("reverse_only_transit_behavior_tree", "")
        self.declare_parameter("reverse_fallback_behavior_tree", "")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter(
            "amcl_nomotion_update_service", "/request_nomotion_update"
        )
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("authorization_topic", "/parking/drive_authorized")
        self.declare_parameter("state_topic", "/parking/mission_state")
        self.declare_parameter("direct_cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("goal_pose_array_topic", "/parking/mission_goals")
        self.declare_parameter("autostart_mission", False)
        self.declare_parameter("require_route_localization", False)
        self.declare_parameter(
            "route_localization_ready_topic",
            "/parking/route_localization/ready",
        )
        self.declare_parameter("initial_pose_publish_count", 5)
        self.declare_parameter("initial_pose_publish_period_sec", 0.25)
        self.declare_parameter("maximum_xy_variance", 0.0625)
        self.declare_parameter("maximum_yaw_variance", 0.12)
        self.declare_parameter("maximum_pose_age_sec", 4.00)
        self.declare_parameter("maximum_position_jump_m", 0.60)
        self.declare_parameter("maximum_yaw_jump_rad", 0.80)
        self.declare_parameter("required_stable_samples", 6)
        self.declare_parameter("nomotion_update_period_sec", 0.25)
        self.declare_parameter("nomotion_update_pose_silence_sec", 0.20)
        self.declare_parameter("localization_loss_cancel_sec", 1.00)
        self.declare_parameter("goal_position_tolerance_m", 0.16)
        self.declare_parameter("goal_yaw_tolerance_rad", 0.18)
        self.declare_parameter("transit_goal_position_tolerance_m", 0.48)
        self.declare_parameter("transit_goal_yaw_tolerance_rad", 3.14)
        self.declare_parameter("transit_pass_radius_m", 0.45)
        self.declare_parameter("transit_pass_maximum_miss_distance_m", 0.75)
        self.declare_parameter("transit_pass_lateral_tolerance_m", 0.55)
        self.declare_parameter("action_server_timeout_sec", 1.0)
        self.declare_parameter("nav2_activation_timeout_sec", 10.0)
        self.declare_parameter("retry_delay_sec", 1.0)
        self.declare_parameter("forward_failure_reverse_fallback_enabled", True)
        self.declare_parameter("forward_no_progress_timeout_sec", 3.0)
        self.declare_parameter("forward_progress_minimum_improvement_m", 0.08)
        self.declare_parameter("replan_authorization_grace_sec", 0.45)
        self.declare_parameter("automatic_recovery_enabled", True)
        self.declare_parameter("automatic_recovery_delay_sec", 2.0)
        self.declare_parameter(
            "obstacle_recovery_request_topic",
            "/parking/obstacle_recovery_request",
        )
        self.declare_parameter("obstacle_reverse_replan_delay_sec", 1.05)
        self.declare_parameter("obstacle_reverse_authorization_sec", 1.35)
        self.declare_parameter("mission_time_limit_sec", 180.0)
        self.declare_parameter("mission_time_limit_enforced", False)
        self.declare_parameter("time_warning_remaining_sec", 30.0)
        self.declare_parameter("time_log_period_sec", 10.0)
        self.declare_parameter("diagnostic_log_period_sec", 2.0)
        # B_PARK's one-metre straight reverse is deterministic.  Driving this
        # final segment directly avoids Nav2 abort/zero-command loops while
        # retaining the adapter's LiDAR, VESC and authorization gates.
        self.declare_parameter("direct_reverse_parking_enabled", True)
        self.declare_parameter(
            "direct_reverse_parking_goal_names", ["B_PARK"]
        )
        self.declare_parameter("direct_reverse_speed_mps", 0.322448)
        self.declare_parameter("direct_reverse_heading_gain", 0.8)
        self.declare_parameter("direct_reverse_maximum_curvature", 1.2)
        self.declare_parameter("visualization_topic", "/parking/mission_visualization")
        self.declare_parameter("parking_box_length_m", 0.80)
        self.declare_parameter("parking_box_width_m", 0.50)

    @staticmethod
    def _load_mission(path: str) -> tuple[str, float, Pose2D, list[MissionStep]]:
        with open(path, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        mission = document["mission"]
        initial = mission["initial_pose"]
        return (
            str(mission.get("frame_id", "map")),
            float(mission.get("base_from_reference_x_m", 0.0)),
            Pose2D(float(initial["x"]), float(initial["y"]), float(initial["yaw"])),
            mission_steps_from_dicts(mission["steps"]),
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _set_state(self, new_state: str, reason: str = "") -> None:
        if new_state not in self.STATES:
            raise ValueError("unknown mission state: %s" % new_state)
        if new_state != self.state:
            detail = " | 사유: %s" % reason_to_korean(reason) if reason else ""
            self.get_logger().info(
                "미션 상태: %s -> %s%s"
                % (STATE_KO[self.state], STATE_KO[new_state], detail)
            )
            self.state = new_state
        self.state_reason = reason
        self._publish_state()

    def _mission_timing(self, now_sec: float | None = None):
        return assess_mission_time(
            started_at_sec=self.mission_started_at,
            now_sec=self._now_sec() if now_sec is None else float(now_sec),
            limit_sec=self.mission_time_limit,
            warning_remaining_sec=self.time_warning_remaining,
            finished_at_sec=self.mission_finished_at,
        )

    def _start_mission_clock(self, now_sec: float) -> None:
        if self.mission_started_at is not None:
            return
        self.mission_started_at = now_sec
        self.mission_finished_at = None
        self.next_time_log_at = now_sec + self.time_log_period
        self.time_warning_emitted = False
        if self.mission_time_limit_enforced:
            self.get_logger().info(
                "경기 시간 측정을 시작합니다: 제한 %.0f초(3분), %.0f초 전 경고"
                % (self.mission_time_limit, self.time_warning_remaining)
            )
        else:
            self.get_logger().warning(
                "경기 시간은 계속 기록하지만 3분 초과 자동 중단은 꺼져 있습니다"
            )

    def _publish_state(self) -> None:
        step = self.steps[self.current_index].name if self.current_index < len(self.steps) else "done"
        timing = self._mission_timing()
        if self.current_pose is not None and self.current_index < len(self.base_goals):
            position_error, yaw_error = pose_error(
                self.current_pose, self.base_goals[self.current_index]
            )
        else:
            position_error, yaw_error = math.nan, math.nan
        authorized = self._drive_authorized()
        reverse_crawl = (
            self.current_index < len(self.steps)
            and self.steps[self.current_index].reverse_only
        )
        self.state_publisher.publish(
            String(
                data=(
                    "state=%s step=%s index=%d/%d retry=%d precise=%s reverse_crawl=%s localization=%s "
                    "goal_xy_error=%.3f goal_yaw_error=%.3f authorized=%s "
                    "elapsed=%.1f remaining=%.1f limit=%.1f limit_enforced=%s%s"
                    % (
                        self.state,
                        step,
                        self.current_index,
                        len(self.steps),
                        self.current_retry,
                        (
                            "true"
                            if self.current_index < len(self.steps)
                            and (
                                self.steps[self.current_index].precise_goal
                                or self.steps[self.current_index].parking_goal
                            )
                            else "false"
                        ),
                        "true" if reverse_crawl else "false",
                        self.localization_reason,
                        position_error,
                        yaw_error,
                        "true" if authorized else "false",
                        timing.elapsed_sec,
                        timing.remaining_sec,
                        self.mission_time_limit,
                        "true" if self.mission_time_limit_enforced else "false",
                        " reason=" + self.state_reason if self.state_reason else "",
                    )
                )
            )
        )

    def _drive_authorized(self, now_sec: float | None = None) -> bool:
        now = self._now_sec() if now_sec is None else float(now_sec)
        active_goal = self.state == "RUNNING" and (
            self.goal_active or self.direct_parking_active
        )
        replan_grace = (
            self.state in {"RUNNING", "RECOVERING", "HOLDING"}
            and now < self.replan_authorized_until
        )
        return self.localization_ready and (active_goal or replan_grace)

    def _arm_replan_grace(self, now_sec: float | None = None) -> None:
        now = self._now_sec() if now_sec is None else float(now_sec)
        self.replan_authorized_until = max(
            self.replan_authorized_until,
            now + self.replan_authorization_grace,
        )

    def _step_mode(self, step: MissionStep) -> str:
        if self.reverse_fallback_active:
            return "후진허용복구"
        if step.parking_goal and step.allow_reverse:
            return "정밀후진주차"
        if step.reverse_only:
            return "정밀후진정렬" if step.precise_goal else "후진전용경유"
        if step.precise_goal and step.allow_reverse:
            return "정밀양방향정렬"
        if step.allow_reverse:
            return "후진허용경유"
        if step.precise_goal or step.parking_goal:
            return "정밀전진"
        return "일반전진"

    def _step_tolerances(self, step: MissionStep) -> tuple[float, float]:
        # Only direction boundaries and the two recorded parking poses need
        # the strict checker. Intermediate reverse arcs are shaping waypoints;
        # holding them to final-parking accuracy causes needless F/R hunting.
        is_transit = not (step.precise_goal or step.parking_goal)
        if is_transit:
            return self.transit_position_tolerance, self.transit_yaw_tolerance
        return self.position_tolerance, self.yaw_tolerance

    def _log_step_diagnostic(self, label: str, *, warning: bool = False) -> None:
        if self.current_index >= len(self.steps):
            return
        step = self.steps[self.current_index]
        target = self.base_goals[self.current_index]
        xy_tolerance, yaw_tolerance = self._step_tolerances(step)
        if self.current_pose is None:
            current_text = "없음"
            error_text = "계산불가"
        else:
            xy_error, yaw_error = pose_error(self.current_pose, target)
            current_text = "(%.3f,%.3f,%.3f)" % (
                self.current_pose.x,
                self.current_pose.y,
                self.current_pose.yaw,
            )
            error_text = "xy=%.3f/%.3f yaw=%.3f/%.3f" % (
                xy_error,
                xy_tolerance,
                yaw_error,
                yaw_tolerance,
            )
        message = (
            "[%s %02d/%02d] 단계=%s 모드=%s 현재=%s "
            "목표=(%.3f,%.3f,%.3f) 오차/완료조건=%s "
            "상태=%s goal_active=%s 위치추정=%s 사유=%s"
            % (
                label,
                self.current_index + 1,
                len(self.steps),
                step.name,
                self._step_mode(step),
                current_text,
                target.x,
                target.y,
                target.yaw,
                error_text,
                self.state,
                "true"
                if (self.goal_active or self.direct_parking_active)
                else "false",
                self.localization_reason,
                reason_to_korean(self.state_reason),
            )
        )
        # Keep the two ROS severity call sites distinct. rclpy throttling caches
        # the call site and rejects changing its severity between invocations.
        if warning:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        received_at = self._now_sec()
        pose = message.pose.pose
        current = Pose2D(
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        stamp_sec = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        assessment = self.gate.update(
            pose=current,
            covariance=message.pose.covariance,
            stamp_sec=stamp_sec,
            now_sec=received_at,
        )
        self.current_pose = current
        self.last_amcl_received_at = received_at
        self.localization_ready = (
            assessment.ready and self.route_localization_ready
        )
        self.localization_reason = (
            assessment.reason
            if not assessment.ready
            else (
                "ok"
                if self.route_localization_ready
                else "route_localization_not_ready"
            )
        )
        if self.localization_ready:
            self.localization_lost_since = None

    def _refresh_localization_status(self, now_sec: float) -> bool:
        """Refresh the exact gate state used to accept a parking/final pose."""

        age = self.gate.age_assessment(now_sec)
        self.localization_ready = age.ready and self.route_localization_ready
        if not age.ready:
            self.localization_reason = age.reason
        elif not self.route_localization_ready:
            self.localization_reason = "route_localization_not_ready"
        else:
            self.localization_reason = "ok"
        return self.localization_ready

    def _current_step_requires_aligned_completion(self) -> bool:
        if self.current_index >= len(self.steps):
            return False
        return (
            self.steps[self.current_index].parking_goal
            or self.current_index == len(self.steps) - 1
        )

    def _on_route_localization_ready(self, message: Bool) -> None:
        self.route_localization_ready = bool(message.data)
        if not self.route_localization_ready:
            self.localization_ready = False
            self.localization_reason = "route_localization_not_ready"

    def _on_obstacle_recovery_request(self, message: String) -> None:
        if (
            self.state != "RUNNING"
            or not self.start_requested
            or self.current_index >= len(self.steps)
        ):
            return
        now_sec = self._now_sec()
        handle = self.goal_handle
        self.goal_serial += 1
        self.goal_active = False
        self.direct_parking_active = False
        self.goal_handle = None
        self._publish_direct_stop()
        self.forward_progress_watchdog.reset()
        if handle is not None:
            handle.cancel_goal_async()
        self.current_retry = 0
        self.retry_at = now_sec + self.obstacle_reverse_replan_delay
        self.replan_authorized_until = max(
            self.replan_authorized_until,
            now_sec + self.obstacle_reverse_authorization_sec,
        )
        self._set_state("RECOVERING", "lidar_obstacle_reverse_recovery")
        self.get_logger().warning(
            "%s: 전방 장애물 감지(%s). 짧은 직선 후진 뒤 같은 단계의 "
            "경로를 다시 계산합니다"
            % (self.steps[self.current_index].name, message.data)
        )
        self._log_step_diagnostic("장애물 후진복구", warning=True)

    def _request_nomotion_update_if_needed(self, now_sec: float) -> None:
        """Force scan matching when AMCL is quiet because the car is stopped."""

        if self.state not in {
            "LOCALIZING",
            "READY",
            "RUNNING",
            "HOLDING",
            "PAUSED_LOCALIZATION",
            "RECOVERING",
        }:
            return
        if now_sec < self.next_nomotion_update:
            return
        if (
            self.last_amcl_received_at is not None
            and now_sec - self.last_amcl_received_at
            < self.nomotion_update_pose_silence
        ):
            return
        if (
            self.nomotion_update_future is not None
            and not self.nomotion_update_future.done()
        ):
            return
        if not self.nomotion_update_client.service_is_ready():
            self.next_nomotion_update = now_sec + self.nomotion_update_period
            return

        self.next_nomotion_update = now_sec + self.nomotion_update_period
        future = self.nomotion_update_client.call_async(Empty.Request())
        self.nomotion_update_future = future
        future.add_done_callback(self._on_nomotion_update_done)

    def _on_nomotion_update_done(self, future) -> None:
        if future is self.nomotion_update_future:
            self.nomotion_update_future = None
        try:
            future.result()
        except Exception as error:  # pragma: no cover - middleware failure path
            if not self.nomotion_update_error_logged:
                self.get_logger().warning(
                    "AMCL 정차 갱신 서비스 호출 실패: %s" % error
                )
                self.nomotion_update_error_logged = True
        else:
            self.nomotion_update_error_logged = False

    def _publish_initial_pose(self) -> None:
        base_pose = reference_pose_to_base(self.initial_reference, self.base_offset)
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = base_pose.x
        message.pose.pose.position.y = base_pose.y
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = (
            quaternion_from_yaw(base_pose.yaw)
        )
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.03
        self.initial_pose_publisher.publish(message)

    def _publish_goal_array(self) -> None:
        array = PoseArray()
        array.header.frame_id = self.frame_id
        array.header.stamp = self.get_clock().now().to_msg()
        for goal in self.base_goals:
            pose = PoseStamped().pose
            pose.position.x = goal.x
            pose.position.y = goal.y
            pose.orientation.z, pose.orientation.w = quaternion_from_yaw(goal.yaw)
            array.poses.append(pose)
        self.goal_array_publisher.publish(array)

    @staticmethod
    def _set_marker_color(marker: Marker, red: float, green: float, blue: float, alpha: float) -> None:
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    def _parking_box_marker(
        self,
        *,
        marker_id: int,
        name: str,
        pose: Pose2D,
        red: float,
        green: float,
        blue: float,
        stamp,
    ) -> list[Marker]:
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        half_length = 0.5 * self.parking_box_length
        half_width = 0.5 * self.parking_box_width
        local_corners = (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
        )
        outline = Marker()
        outline.header.frame_id = self.frame_id
        outline.header.stamp = stamp
        outline.ns = "parking_spaces"
        outline.id = marker_id
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.scale.x = 0.045
        self._set_marker_color(outline, red, green, blue, 0.95)
        for local_x, local_y in local_corners:
            outline.points.append(
                Point(
                    x=pose.x + cosine * local_x - sine * local_y,
                    y=pose.y + sine * local_x + cosine * local_y,
                    z=0.035,
                )
            )

        fill = Marker()
        fill.header.frame_id = self.frame_id
        fill.header.stamp = stamp
        fill.ns = "parking_space_fill"
        fill.id = marker_id
        fill.type = Marker.CUBE
        fill.action = Marker.ADD
        fill.pose.position.x = pose.x
        fill.pose.position.y = pose.y
        fill.pose.position.z = 0.012
        fill.pose.orientation.z, fill.pose.orientation.w = quaternion_from_yaw(pose.yaw)
        fill.scale.x = self.parking_box_length
        fill.scale.y = self.parking_box_width
        fill.scale.z = 0.02
        self._set_marker_color(fill, red, green, blue, 0.16)

        label = Marker()
        label.header.frame_id = self.frame_id
        label.header.stamp = stamp
        label.ns = "parking_space_labels"
        label.id = marker_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = pose.x
        label.pose.position.y = pose.y
        label.pose.position.z = 0.18
        label.pose.orientation.w = 1.0
        label.scale.z = 0.16
        label.text = name
        self._set_marker_color(label, red, green, blue, 1.0)
        return [fill, outline, label]

    def _publish_visualization(self) -> None:
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers: list[Marker] = [clear]

        route = Marker()
        route.header.frame_id = self.frame_id
        route.header.stamp = stamp
        route.ns = "nominal_route"
        route.id = 0
        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD
        route.scale.x = 0.035
        self._set_marker_color(route, 0.10, 0.95, 0.45, 0.90)
        initial_base = reference_pose_to_base(self.initial_reference, self.base_offset)
        route.points.append(Point(x=initial_base.x, y=initial_base.y, z=0.045))
        for goal in self.base_goals:
            route.points.append(Point(x=goal.x, y=goal.y, z=0.045))
        markers.append(route)

        parking_colors = {
            "A_PARK": (0.10, 0.70, 1.00),
            "B_PARK": (1.00, 0.45, 0.10),
        }
        next_id = 10
        for step in self.steps:
            if step.name not in parking_colors:
                continue
            red, green, blue = parking_colors[step.name]
            markers.extend(
                self._parking_box_marker(
                    marker_id=next_id,
                    name=step.name,
                    pose=step.reference_pose,
                    red=red,
                    green=green,
                    blue=blue,
                    stamp=stamp,
                )
            )
            next_id += 1

        if self.current_index < len(self.base_goals):
            target = self.base_goals[self.current_index]
            target_marker = Marker()
            target_marker.header.frame_id = self.frame_id
            target_marker.header.stamp = stamp
            target_marker.ns = "current_target"
            target_marker.id = 0
            target_marker.type = Marker.ARROW
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = target.x
            target_marker.pose.position.y = target.y
            target_marker.pose.position.z = 0.08
            target_marker.pose.orientation.z, target_marker.pose.orientation.w = quaternion_from_yaw(target.yaw)
            target_marker.scale.x = 0.38
            target_marker.scale.y = 0.09
            target_marker.scale.z = 0.09
            self._set_marker_color(target_marker, 1.0, 0.95, 0.05, 1.0)
            markers.append(target_marker)

            target_label = Marker()
            target_label.header.frame_id = self.frame_id
            target_label.header.stamp = stamp
            target_label.ns = "current_target_label"
            target_label.id = 0
            target_label.type = Marker.TEXT_VIEW_FACING
            target_label.action = Marker.ADD
            target_label.pose.position.x = target.x
            target_label.pose.position.y = target.y
            target_label.pose.position.z = 0.28
            target_label.pose.orientation.w = 1.0
            target_label.scale.z = 0.14
            target_label.text = "%02d/%02d %s" % (
                self.current_index + 1,
                len(self.steps),
                self.steps[self.current_index].name,
            )
            self._set_marker_color(target_label, 1.0, 0.95, 0.05, 1.0)
            markers.append(target_label)

        self.visualization_publisher.publish(MarkerArray(markers=markers))

    def _on_start(self, _request, response):
        if self.state in {
            "RUNNING",
            "HOLDING",
            "PAUSED_LOCALIZATION",
            "RECOVERING",
        }:
            response.success = False
            response.message = "주차 미션이 이미 진행 중입니다"
            return response
        if self.state == "COMPLETED":
            response.success = False
            response.message = "완료된 미션을 먼저 초기화하세요"
            return response
        if self.state == "ABORTED":
            response.success = False
            response.message = "중단된 미션을 먼저 초기화하세요"
            return response
        self.start_requested = True
        if not self.localization_ready:
            response.success = False
            response.message = (
                "시작 요청을 저장했습니다. AMCL 위치추정이 안정되면 자동으로 시작합니다. "
                "현재 부족 항목: %s" % reason_to_korean(self.localization_reason)
            )
            return response
        response.success = True
        response.message = "주차 미션 시작 요청을 승인했습니다"
        return response

    def _on_abort(self, _request, response):
        self.start_requested = False
        self._cancel_active_goal(localization_pause=False)
        if self.mission_started_at is not None:
            self.mission_finished_at = self._now_sec()
        self._set_state("ABORTED", "operator_abort")
        response.success = True
        response.message = "주차 미션을 중단했고 모터 주행 권한을 해제했습니다"
        return response

    def _on_reset(self, _request, response):
        self._cancel_active_goal(localization_pause=False)
        self.current_index = 0
        self.current_retry = 0
        self.reverse_fallback_active = False
        self.replan_authorized_until = 0.0
        self.forward_progress_watchdog.reset()
        self.hold_until = None
        self.retry_at = None
        self.mission_started_at = None
        self.mission_finished_at = None
        self.next_time_log_at = None
        self.next_step_diagnostic_at = 0.0
        self.time_warning_emitted = False
        self.nav2_first_goal_accepted = False
        self.nav2_activation_deadline = None
        self.start_requested = False
        self.initial_pose_remaining = int(
            self.get_parameter("initial_pose_publish_count").value
        )
        self.next_initial_pose_publish = 0.0
        self.next_nomotion_update = 0.0
        self.last_amcl_received_at = None
        self.gate.stable_samples = 0
        self.localization_ready = False
        self._set_state("LOCALIZING", "operator_reset")
        self._publish_visualization()
        response.success = True
        response.message = "주차 미션과 초기 위치추정을 초기화했습니다"
        return response

    def _cancel_active_goal(self, *, localization_pause: bool) -> None:
        self.forward_progress_watchdog.reset()
        self.replan_authorized_until = 0.0
        self.cancel_for_localization = localization_pause
        if not localization_pause:
            self.goal_serial += 1
        if self.goal_handle is not None and self.goal_active:
            self.goal_handle.cancel_goal_async()
        self.goal_active = False
        self.direct_parking_active = False
        self.goal_handle = None
        self._publish_direct_stop()

    def _uses_direct_reverse_parking(self, step: MissionStep) -> bool:
        return (
            self.direct_reverse_parking_enabled
            and step.name in self.direct_reverse_goal_names
            and step.parking_goal
            and step.reverse_only
        )

    def _publish_direct_stop(self) -> None:
        if hasattr(self, "direct_cmd_publisher"):
            self.direct_cmd_publisher.publish(Twist())

    def _start_direct_reverse_parking(self) -> None:
        step = self.steps[self.current_index]
        self.cancel_for_localization = False
        self.forward_progress_watchdog.reset()
        self.retry_at = None
        self.current_retry = 0
        self.goal_active = False
        self.goal_handle = None
        self.direct_parking_active = True
        self._set_state("RUNNING", "direct_reverse_parking")
        self.get_logger().warning(
            "%s: Nav2 경로/제자리 정지 출력을 사용하지 않고 AMCL 위치 오차로 "
            "직접 연속 후진합니다" % step.name
        )
        self._log_step_diagnostic("직접후진 시작")
        self.next_step_diagnostic_at = self._now_sec() + self.diagnostic_log_period

    def _update_direct_reverse_parking(self) -> None:
        if (
            not self.direct_parking_active
            or self.state != "RUNNING"
            or not self.localization_ready
            or self.current_pose is None
            or self.current_index >= len(self.steps)
        ):
            return
        step = self.steps[self.current_index]
        target = self.base_goals[self.current_index]
        command = direct_reverse_parking_command(
            current=self.current_pose,
            target=target,
            speed_mps=self.direct_reverse_speed,
            position_tolerance_m=self.position_tolerance,
            yaw_tolerance_rad=self.yaw_tolerance,
            heading_gain=self.direct_reverse_heading_gain,
            maximum_curvature=self.direct_reverse_maximum_curvature,
        )
        if command.reached:
            self.direct_parking_active = False
            self.current_retry = 0
            self.reverse_fallback_active = False
            self.replan_authorized_until = 0.0
            self.hold_until = self._now_sec() + step.hold_sec
            self._set_state("HOLDING", "reached_" + step.name)
            # Revoke authorization before sending zero so the adapter cannot
            # reinterpret the final zero as reverse-crawl continuation.
            self.authorization_publisher.publish(Bool(data=False))
            self._publish_direct_stop()
            self.get_logger().info(
                "%s 직접후진 도착: xy=%.3fm yaw=%.3frad, 모터 권한 해제"
                % (step.name, command.distance_m, abs(command.yaw_error_rad))
            )
            self._log_step_diagnostic("직접후진 도착")
            return

        twist = Twist()
        twist.linear.x = command.linear_x
        twist.angular.z = command.angular_z
        self.direct_cmd_publisher.publish(twist)

    def _send_current_goal(self) -> None:
        if (
            self.current_index >= len(self.steps)
            or self.goal_active
            or self.direct_parking_active
        ):
            return
        if not self.localization_ready:
            self._set_state("PAUSED_LOCALIZATION", self.localization_reason)
            return
        step = self.steps[self.current_index]
        self._publish_visualization()
        if self._uses_direct_reverse_parking(step):
            self._start_direct_reverse_parking()
            return
        if not self.nav_client.wait_for_server(timeout_sec=self.action_server_timeout):
            self._defer_for_nav2_activation("waiting_for_nav2_action_server")
            return

        target = self.base_goals[self.current_index]
        message = NavigateToPose.Goal()
        message.pose.header.frame_id = self.frame_id
        message.pose.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = target.x
        message.pose.pose.position.y = target.y
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = (
            quaternion_from_yaw(target.yaw)
        )
        if self.reverse_fallback_active:
            message.behavior_tree = self.reverse_fallback_behavior_tree
        elif step.reverse_only and (step.precise_goal or step.parking_goal):
            message.behavior_tree = self.parking_reverse_behavior_tree
        elif step.reverse_only:
            message.behavior_tree = self.reverse_only_transit_behavior_tree
        elif step.parking_goal and step.allow_reverse:
            # Surveyed A/B centers are approached in reverse only.  The path
            # is computed once, preventing a 2 Hz replan from alternating
            # forward/reverse commands while the steering servo is settling.
            message.behavior_tree = self.parking_reverse_behavior_tree
        elif step.precise_goal and step.allow_reverse:
            message.behavior_tree = self.precise_reverse_behavior_tree
        elif step.allow_reverse:
            message.behavior_tree = self.reverse_fallback_behavior_tree
        elif step.precise_goal or step.parking_goal:
            message.behavior_tree = self.precise_forward_behavior_tree
        else:
            message.behavior_tree = self.forward_behavior_tree
        self.cancel_for_localization = False
        self.forward_progress_watchdog.reset()
        self.goal_serial += 1
        serial = self.goal_serial
        future = self.nav_client.send_goal_async(
            message,
            feedback_callback=self._on_feedback,
        )
        future.add_done_callback(
            lambda completed, goal_serial=serial: self._on_goal_response(
                completed, goal_serial
            )
        )
        if self.reverse_fallback_active:
            self.get_logger().warning(
                "%s: 전진 경로 실패로 후진 허용 경로를 실행합니다"
                % self.steps[self.current_index].name
            )
            self._set_state("RUNNING", "reverse_fallback_active")
        else:
            self._set_state(
                "RUNNING", "sending_" + self.steps[self.current_index].name
            )
        self._log_step_diagnostic("단계 시작")
        self.next_step_diagnostic_at = self._now_sec() + self.diagnostic_log_period

    def _on_goal_response(self, future, serial: int) -> None:
        if serial != self.goal_serial:
            return
        handle = future.result()
        if not handle.accepted:
            if not self.nav2_first_goal_accepted:
                self._defer_for_nav2_activation("waiting_for_nav2_activation")
                return
            self._handle_goal_failure("goal_rejected")
            return
        self.nav2_first_goal_accepted = True
        self.nav2_activation_deadline = None
        self.goal_handle = handle
        self.goal_active = True
        self._log_step_diagnostic("Nav2 목표수락")
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, goal_serial=serial: self._on_goal_result(
                completed, goal_serial
            )
        )

    def _defer_for_nav2_activation(self, reason: str) -> None:
        """Wait for lifecycle activation without spending a driving retry."""

        now_sec = self._now_sec()
        if self.nav2_activation_deadline is None:
            self.nav2_activation_deadline = now_sec + self.nav2_activation_timeout
        if now_sec >= self.nav2_activation_deadline:
            self.start_requested = False
            if self.mission_started_at is not None:
                self.mission_finished_at = now_sec
            self._set_state("ABORTED", "nav2_activation_timeout")
            return
        self.retry_at = now_sec + self.retry_delay
        self._set_state("RUNNING", reason)

    def _on_feedback(self, feedback_message) -> None:
        if (
            not self.goal_active
            or self.reverse_fallback_active
            or not self.forward_failure_reverse_fallback_enabled
            or self.current_index >= len(self.steps)
            or not is_reverse_fallback_candidate(self.steps[self.current_index])
        ):
            return
        feedback = feedback_message.feedback
        if not self.forward_progress_watchdog.update(
            distance_m=float(feedback.distance_remaining),
            now_sec=self._now_sec(),
        ):
            return

        step = self.steps[self.current_index]
        handle = self.goal_handle
        now_sec = self._now_sec()
        self.goal_serial += 1
        self.goal_active = False
        self.goal_handle = None
        self.reverse_fallback_active = True
        self.current_retry = 0
        self.retry_at = now_sec + self.retry_delay
        self._arm_replan_grace(now_sec)
        self.forward_progress_watchdog.reset()
        if handle is not None:
            handle.cancel_goal_async()
        self.get_logger().warning(
            "%s: 전진 진행이 제한시간 동안 개선되지 않아 %.2f초 후 "
            "후진 허용 경로로 전환합니다" % (step.name, self.retry_delay)
        )
        self._set_state(
            "RUNNING", "forward_progress_stalled_reverse_fallback"
        )
        self._log_step_diagnostic("전진 진행정체", warning=True)

    def _on_goal_result(self, future, serial: int) -> None:
        if serial != self.goal_serial:
            return
        wrapped = future.result()
        self.forward_progress_watchdog.reset()
        self.goal_active = False
        self.goal_handle = None
        if self.cancel_for_localization:
            self.cancel_for_localization = False
            self._set_state("PAUSED_LOCALIZATION", "goal_cancelled_for_localization")
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            step = self.steps[self.current_index]
            if (
                wrapped.status == GoalStatus.STATUS_ABORTED
                and self.forward_failure_reverse_fallback_enabled
                and not self.reverse_fallback_active
                and is_reverse_fallback_candidate(step)
            ):
                self.reverse_fallback_active = True
                self.current_retry = 0
                now_sec = self._now_sec()
                self.retry_at = now_sec + self.retry_delay
                self._arm_replan_grace(now_sec)
                self.get_logger().warning(
                    "%s: 전진 전용 경로가 실패했습니다. %.2f초 후 "
                    "후진 허용 경로로 같은 목표를 재계획합니다"
                    % (step.name, self.retry_delay)
                )
                self._set_state(
                    "RUNNING", "forward_path_failed_reverse_fallback"
                )
                self._log_step_diagnostic("전진경로 실패", warning=True)
                return
            self._handle_goal_failure("nav2_status_%d" % wrapped.status)
            return
        if self.current_pose is None:
            self._handle_goal_failure("no_final_pose")
            return

        step = self.steps[self.current_index]
        if self._current_step_requires_aligned_completion():
            now_sec = self._now_sec()
            if not self._refresh_localization_status(now_sec):
                self.hold_until = None
                self.retry_at = None
                self.localization_lost_since = now_sec
                self._set_state(
                    "PAUSED_LOCALIZATION",
                    "aligned_completion_waiting_for_localization",
                )
                self.get_logger().warning(
                    "%s: Nav2 도착 응답은 받았지만 위치정합=%s이므로 "
                    "도착을 확정하지 않습니다. 정합 복구 후 같은 목표를 "
                    "다시 검증합니다"
                    % (step.name, reason_to_korean(self.localization_reason))
                )
                self._log_step_diagnostic("정합 도착확정 보류", warning=True)
                return
        position_error, yaw_error = pose_error(
            self.current_pose,
            self.base_goals[self.current_index],
        )
        is_transit = not (step.precise_goal or step.parking_goal)
        position_tolerance = (
            self.transit_position_tolerance if is_transit else self.position_tolerance
        )
        yaw_tolerance = self.transit_yaw_tolerance if is_transit else self.yaw_tolerance
        if position_error > position_tolerance or yaw_error > yaw_tolerance:
            self._handle_goal_failure(
                "goal_error_xy_%.3f_yaw_%.3f" % (position_error, yaw_error)
            )
            return

        self.current_retry = 0
        self.reverse_fallback_active = False
        self.hold_until = self._now_sec() + step.hold_sec
        self._set_state("HOLDING", "reached_" + step.name)
        self._log_step_diagnostic("단계 도착")

    def _accept_passed_transit_waypoint(self, now_sec: float) -> bool:
        if (
            self.state != "RUNNING"
            or self.current_pose is None
            or self.current_index >= len(self.steps)
        ):
            return False
        step = self.steps[self.current_index]
        if step.precise_goal or step.parking_goal:
            return False
        previous = (
            reference_pose_to_base(self.initial_reference, self.base_offset)
            if self.current_index == 0
            else self.base_goals[self.current_index - 1]
        )
        target = self.base_goals[self.current_index]
        assessment = assess_transit_waypoint_pass(
            current=self.current_pose,
            previous=previous,
            target=target,
            radius_m=self.transit_pass_radius,
            maximum_miss_distance_m=self.transit_pass_maximum_miss_distance,
            lateral_tolerance_m=self.transit_pass_lateral_tolerance,
        )
        if not assessment.passed:
            return False

        self.get_logger().info(
            "%s: 일반 웨이포인트 통과 인정(%s, 거리 %.2fm, "
            "통과 %.2fm, 횡오차 %.2fm)"
            % (
                step.name,
                assessment.reason,
                assessment.distance_m,
                assessment.along_past_m,
                assessment.lateral_error_m,
            )
        )
        self._cancel_active_goal(localization_pause=False)
        self._arm_replan_grace(now_sec)
        self.current_retry = 0
        self.reverse_fallback_active = False
        self.hold_until = now_sec
        self._set_state("HOLDING", "transit_waypoint_passed")
        self._log_step_diagnostic("일반점 통과")
        return True

    def _handle_goal_failure(self, reason: str) -> None:
        step = self.steps[self.current_index]
        self.goal_active = False
        self.goal_handle = None
        if self.current_retry < step.maximum_retries:
            self.current_retry += 1
            now_sec = self._now_sec()
            self.retry_at = now_sec + self.retry_delay
            self._arm_replan_grace(now_sec)
            self._set_state("RUNNING", "%s_retry_%d" % (reason, self.current_retry))
            self._log_step_diagnostic("단계 실패", warning=True)
            return
        if self.automatic_recovery_enabled:
            self.current_retry = 0
            now_sec = self._now_sec()
            self.retry_at = now_sec + self.automatic_recovery_delay
            self._arm_replan_grace(now_sec)
            self._set_state("RECOVERING", reason + "_automatic_recovery")
            self._log_step_diagnostic("자동복구 대기", warning=True)
            return
        self.start_requested = False
        if self.mission_started_at is not None:
            self.mission_finished_at = self._now_sec()
        self._set_state("ABORTED", "%s_retries_exhausted" % reason)
        self._log_step_diagnostic("미션 중단", warning=True)

    def _advance_after_hold(self) -> None:
        self.hold_until = None
        self.reverse_fallback_active = False
        self.current_index += 1
        self._publish_visualization()
        if self.current_index >= len(self.steps):
            self.start_requested = False
            self.mission_finished_at = self._now_sec()
            self._set_state("COMPLETED", "returned_to_start")
            timing = self._mission_timing()
            self.get_logger().info(
                "주차 미션 완주: 총 %.1f초 (3분 기준 %s)"
                % (
                    timing.elapsed_sec,
                    "이내" if timing.elapsed_sec <= self.mission_time_limit else "초과",
                )
            )
            return
        self._send_current_goal()

    def _on_timer(self) -> None:
        now_sec = self._now_sec()
        self._refresh_localization_status(now_sec)

        if self.initial_pose_remaining > 0 and now_sec >= self.next_initial_pose_publish:
            self._publish_initial_pose()
            self.initial_pose_remaining -= 1
            self.next_initial_pose_publish = now_sec + self.initial_pose_period

        self._request_nomotion_update_if_needed(now_sec)

        if self.state == "LOCALIZING" and self.localization_ready:
            self._set_state("READY", "amcl_stable")

        if self.state in {"RUNNING", "HOLDING"} and not self.localization_ready:
            if self.localization_lost_since is None:
                self.localization_lost_since = now_sec
            elif (
                (self.goal_active or self.direct_parking_active)
                and now_sec - self.localization_lost_since
                >= self.localization_loss_cancel_sec
            ):
                self._cancel_active_goal(localization_pause=True)
                self._set_state("PAUSED_LOCALIZATION", self.localization_reason)
            elif (
                self.state == "HOLDING"
                and self._current_step_requires_aligned_completion()
                and now_sec - self.localization_lost_since
                >= self.localization_loss_cancel_sec
            ):
                self.hold_until = None
                self.retry_at = None
                self._set_state(
                    "PAUSED_LOCALIZATION",
                    "aligned_completion_waiting_for_localization",
                )
                self._log_step_diagnostic("정합 정지확인 보류", warning=True)

        if self.state == "PAUSED_LOCALIZATION" and self.localization_ready:
            self.localization_lost_since = None
            self.retry_at = now_sec + self.retry_delay
            self._set_state("RUNNING", "localization_recovered")

        self._accept_passed_transit_waypoint(now_sec)

        if self.start_requested and self.state == "READY":
            self._start_mission_clock(now_sec)
            self._send_current_goal()

        # Unlike a Nav2 action, the direct B parking controller publishes a
        # fresh reverse command every timer tick and completes from AMCL pose.
        self._update_direct_reverse_parking()

        active_states = {
            "RUNNING",
            "HOLDING",
            "PAUSED_LOCALIZATION",
            "RECOVERING",
        }
        if self.mission_started_at is not None and self.state in active_states:
            timing = self._mission_timing(now_sec)
            if self.mission_time_limit_enforced and timing.expired:
                self.start_requested = False
                self._cancel_active_goal(localization_pause=False)
                self.mission_finished_at = now_sec
                self._set_state("ABORTED", "mission_time_limit")
                self._log_step_diagnostic("제한시간 중단", warning=True)
                self.get_logger().error(
                    "경기 제한시간 초과: %.1f/%.1f초, 모터 권한을 해제했습니다"
                    % (timing.elapsed_sec, self.mission_time_limit)
                )
            else:
                if (
                    self.mission_time_limit_enforced
                    and not self.time_warning_emitted
                    and timing.warning
                ):
                    self.time_warning_emitted = True
                    self.get_logger().warning(
                        "경기 종료까지 %.1f초 남았습니다: 현재 %d/%d단계"
                        % (
                            timing.remaining_sec,
                            self.current_index + 1,
                            len(self.steps),
                        )
                    )
                if (
                    self.next_time_log_at is not None
                    and now_sec >= self.next_time_log_at
                ):
                    self.get_logger().info(
                        "경기 진행시간 %.1f초, 3분 기준 남은 시간 %.1f초, "
                        "진행 %d/%d단계"
                        % (
                            timing.elapsed_sec,
                            timing.remaining_sec,
                            self.current_index + 1,
                            len(self.steps),
                        )
                    )
                    self.next_time_log_at = now_sec + self.time_log_period
        if (
            self.state in active_states
            and now_sec >= self.next_step_diagnostic_at
        ):
            self._log_step_diagnostic("단계 진행")
            self.next_step_diagnostic_at = now_sec + self.diagnostic_log_period
        if (
            self.state == "RECOVERING"
            and not self.goal_active
            and self.retry_at is not None
            and now_sec >= self.retry_at
        ):
            self.retry_at = None
            self._set_state("RUNNING", "automatic_recovery")
            self._send_current_goal()
        if (
            self.state == "RUNNING"
            and not self.goal_active
            and self.retry_at is not None
            and now_sec >= self.retry_at
        ):
            self.retry_at = None
            self._send_current_goal()
        if (
            self.state == "HOLDING"
            and self.hold_until is not None
            and now_sec >= self.hold_until
            and self.localization_ready
        ):
            self._advance_after_hold()

        authorized = self._drive_authorized(now_sec)
        self.authorization_publisher.publish(Bool(data=authorized))
        self._publish_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingMissionManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.authorization_publisher.publish(Bool(data=False))
            node._publish_direct_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
