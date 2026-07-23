"""20 Hz ROS2 integration-test adapter for Mission Manager V0.2."""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Int32, String
from teamkai_interfaces.msg import MissionDecision as MissionDecisionMsg

from track_drive.mission.mission_manager import MissionManager
from track_drive.mission.mission_types import (
    MissionDecision,
    MissionManagerConfig,
    MissionObservation,
)
from track_drive.mission.states import ControlMode, MissionState, StartSignal


UPDATE_PERIOD_SEC = 0.05
DECISION_TOPIC = "/mission/decision"
DECISION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

_MISSION_STATE_CODES = {
    MissionState.WAIT_START_SIGNAL: (
        MissionDecisionMsg.MISSION_STATE_WAIT_START_SIGNAL
    ),
    MissionState.LANE_DRIVING: MissionDecisionMsg.MISSION_STATE_LANE_DRIVING,
    MissionState.CONE_SECTION: MissionDecisionMsg.MISSION_STATE_CONE_SECTION,
    MissionState.FIXED_OBSTACLE_SECTION: (
        MissionDecisionMsg.MISSION_STATE_FIXED_OBSTACLE_SECTION
    ),
    MissionState.OVERTAKE_SECTION: (
        MissionDecisionMsg.MISSION_STATE_OVERTAKE_SECTION
    ),
    MissionState.ROUTE_SELECTION: (
        MissionDecisionMsg.MISSION_STATE_ROUTE_SELECTION
    ),
    MissionState.SHORTCUT_SECTION: (
        MissionDecisionMsg.MISSION_STATE_SHORTCUT_SECTION
    ),
}

_CONTROL_MODE_CODES = {
    ControlMode.STOP: MissionDecisionMsg.CONTROL_MODE_STOP,
    ControlMode.NORMAL_IL: MissionDecisionMsg.CONTROL_MODE_NORMAL_IL,
    ControlMode.CONE_DRIVE_RULE: (
        MissionDecisionMsg.CONTROL_MODE_CONE_DRIVE_RULE
    ),
    ControlMode.LANE_FALLBACK: (
        MissionDecisionMsg.CONTROL_MODE_LANE_FALLBACK
    ),
    ControlMode.FIXED_OBSTACLE_RULE: (
        MissionDecisionMsg.CONTROL_MODE_FIXED_OBSTACLE_RULE
    ),
    ControlMode.VEHICLE_FOLLOW: (
        MissionDecisionMsg.CONTROL_MODE_VEHICLE_FOLLOW
    ),
    ControlMode.VEHICLE_OVERTAKE: (
        MissionDecisionMsg.CONTROL_MODE_VEHICLE_OVERTAKE
    ),
    ControlMode.SHORTCUT_RULE: (
        MissionDecisionMsg.CONTROL_MODE_SHORTCUT_RULE
    ),
}

_SELECTED_SOURCE_CODES = {
    "none": MissionDecisionMsg.SOURCE_NONE,
    "drive_il": MissionDecisionMsg.SOURCE_DRIVE_IL,
    "cone_rule": MissionDecisionMsg.SOURCE_CONE_RULE,
    "lane_fallback": MissionDecisionMsg.SOURCE_LANE_FALLBACK,
    "fixed_obstacle_rule": MissionDecisionMsg.SOURCE_FIXED_OBSTACLE_RULE,
    "vehicle_rule": MissionDecisionMsg.SOURCE_VEHICLE_RULE,
    "shortcut": MissionDecisionMsg.SOURCE_SHORTCUT,
}

_SPEED_PROFILE_CODES = {
    "stop": MissionDecisionMsg.SPEED_PROFILE_STOP,
    "normal": MissionDecisionMsg.SPEED_PROFILE_NORMAL,
    "cone": MissionDecisionMsg.SPEED_PROFILE_CONE,
    "fallback": MissionDecisionMsg.SPEED_PROFILE_FALLBACK,
    "obstacle": MissionDecisionMsg.SPEED_PROFILE_OBSTACLE,
    "vehicle_follow": MissionDecisionMsg.SPEED_PROFILE_VEHICLE_FOLLOW,
    "vehicle_overtake": MissionDecisionMsg.SPEED_PROFILE_VEHICLE_OVERTAKE,
    "shortcut": MissionDecisionMsg.SPEED_PROFILE_SHORTCUT,
}


class MissionManagerNode(Node):
    """의미 기반 입력을 판단하고 완성된 MissionDecision을 발행한다."""

    def __init__(self) -> None:
        super().__init__("mission_manager")

        defaults = MissionManagerConfig()
        self.declare_parameter(
            "start_signal_red_hold_sec",
            defaults.start_signal_red_hold_sec,
        )
        self.declare_parameter(
            "start_signal_green_hold_sec",
            defaults.start_signal_green_hold_sec,
        )
        self.declare_parameter(
            "cone_enter_hold_sec", defaults.cone_enter_hold_sec
        )
        self.declare_parameter(
            "cone_exit_hold_sec", defaults.cone_exit_hold_sec
        )
        self.declare_parameter(
            "cone_min_dwell_sec", defaults.cone_min_dwell_sec
        )
        self.declare_parameter(
            "cone_reenter_cooldown_sec",
            defaults.cone_reenter_cooldown_sec,
        )
        self.declare_parameter(
            "drive_recover_hold_sec", defaults.drive_recover_hold_sec
        )
        self.declare_parameter(
            "lane_fallback_ready_hold_sec",
            defaults.lane_fallback_ready_hold_sec,
        )
        self.declare_parameter(
            "lane_source_loss_grace_sec",
            defaults.lane_source_loss_grace_sec,
        )
        self.declare_parameter(
            "minimum_camera_cone_count",
            defaults.minimum_camera_cone_count,
        )
        self.declare_parameter(
            "maximum_camera_cone_count_for_exit",
            defaults.maximum_camera_cone_count_for_exit,
        )
        self.declare_parameter(
            "status_log_period_sec", defaults.status_log_period_sec
        )

        config = MissionManagerConfig(
            start_signal_red_hold_sec=self._float_parameter(
                "start_signal_red_hold_sec"
            ),
            start_signal_green_hold_sec=self._float_parameter(
                "start_signal_green_hold_sec"
            ),
            cone_enter_hold_sec=self._float_parameter(
                "cone_enter_hold_sec"
            ),
            cone_exit_hold_sec=self._float_parameter(
                "cone_exit_hold_sec"
            ),
            cone_min_dwell_sec=self._float_parameter(
                "cone_min_dwell_sec"
            ),
            cone_reenter_cooldown_sec=self._float_parameter(
                "cone_reenter_cooldown_sec"
            ),
            drive_recover_hold_sec=self._float_parameter(
                "drive_recover_hold_sec"
            ),
            lane_fallback_ready_hold_sec=self._float_parameter(
                "lane_fallback_ready_hold_sec"
            ),
            lane_source_loss_grace_sec=self._float_parameter(
                "lane_source_loss_grace_sec"
            ),
            minimum_camera_cone_count=int(
                self.get_parameter("minimum_camera_cone_count").value
            ),
            maximum_camera_cone_count_for_exit=int(
                self.get_parameter(
                    "maximum_camera_cone_count_for_exit"
                ).value
            ),
            status_log_period_sec=self._float_parameter(
                "status_log_period_sec"
            ),
        )

        self._manager = MissionManager(config)
        self._manager.set_logger(self.get_logger().info)

        self._safety_stop_required = False
        self._safety_stop_assertion_pending = False
        self._start_signal = StartSignal.UNKNOWN
        self._start_signal_valid = False
        self._safety_ready = True
        self._drive_policy_valid = False
        self._lane_fallback_valid = False
        self._camera_cone_valid = False
        self._camera_cone_count = 0
        self._lidar_cone_source_valid = False
        self._lidar_cone_path_ready = False
        self._lidar_cone_present = False

        self._decision_publisher = self.create_publisher(
            MissionDecisionMsg,
            DECISION_TOPIC,
            DECISION_QOS,
        )

        # 아래 토픽은 V0.2 통합시험용 adapter 입력이다.
        self.create_subscription(
            String, "/mission/override", self._on_override, 10
        )
        self.create_subscription(
            Bool,
            "/mission/input/safety_stop_required",
            self._on_safety_stop_required,
            10,
        )
        self.create_subscription(
            String,
            "/mission/input/start_signal",
            self._on_start_signal,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/input/start_signal_valid",
            self._on_start_signal_valid,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/input/safety_ready",
            self._on_safety_ready,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/input/drive_policy_valid",
            self._on_drive_policy_valid,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/input/lane_fallback_valid",
            self._on_lane_fallback_valid,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/input/camera_cone_valid",
            self._on_camera_cone_valid,
            10,
        )
        self.create_subscription(
            Int32,
            "/mission/input/camera_cone_count",
            self._on_camera_cone_count,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/input/lidar_cone_source_valid",
            self._on_lidar_cone_source_valid,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/input/lidar_cone_path_ready",
            self._on_lidar_cone_path_ready,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/input/lidar_cone_present",
            self._on_lidar_cone_present,
            10,
        )

        self.create_timer(UPDATE_PERIOD_SEC, self._on_update_timer)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _on_override(self, message: String) -> None:
        self._manager.set_manual_override(message.data)

    def _on_safety_stop_required(self, message: Bool) -> None:
        self._safety_stop_required = bool(message.data)
        if message.data:
            # true/false가 한 timer 사이에 들어와도 assertion을 한 번 전달한다.
            self._safety_stop_assertion_pending = True

    def _on_start_signal(self, message: String) -> None:
        normalized = message.data.strip().upper()
        normalized = {
            "BLUE": "GREEN",
        }.get(normalized, normalized)
        self._start_signal = StartSignal.__members__.get(
            normalized,
            StartSignal.UNKNOWN,
        )

    def _on_start_signal_valid(self, message: Bool) -> None:
        self._start_signal_valid = bool(message.data)

    def _on_safety_ready(self, message: Bool) -> None:
        self._safety_ready = bool(message.data)

    def _on_drive_policy_valid(self, message: Bool) -> None:
        self._drive_policy_valid = bool(message.data)

    def _on_lane_fallback_valid(self, message: Bool) -> None:
        self._lane_fallback_valid = bool(message.data)

    def _on_camera_cone_valid(self, message: Bool) -> None:
        self._camera_cone_valid = bool(message.data)

    def _on_camera_cone_count(self, message: Int32) -> None:
        self._camera_cone_count = int(message.data)

    def _on_lidar_cone_source_valid(self, message: Bool) -> None:
        self._lidar_cone_source_valid = bool(message.data)

    def _on_lidar_cone_path_ready(self, message: Bool) -> None:
        self._lidar_cone_path_ready = bool(message.data)

    def _on_lidar_cone_present(self, message: Bool) -> None:
        self._lidar_cone_present = bool(message.data)

    def _publish_decision(self, decision: MissionDecision, stamp) -> None:
        message = MissionDecisionMsg()
        message.stamp = stamp
        message.mission_state = _MISSION_STATE_CODES[
            decision.mission_state
        ]
        message.control_mode = _CONTROL_MODE_CODES[
            decision.control_mode
        ]
        message.selected_source = _SELECTED_SOURCE_CODES[
            decision.selected_source
        ]
        message.speed_profile = _SPEED_PROFILE_CODES[
            decision.speed_profile
        ]
        message.stop_required = decision.stop_required
        self._decision_publisher.publish(message)

    def _on_update_timer(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1.0e-9
        observation = MissionObservation(
            now_sec=now_sec,
            safety_stop_required=(
                self._safety_stop_required
                or self._safety_stop_assertion_pending
            ),
            start_signal=self._start_signal,
            start_signal_valid=self._start_signal_valid,
            safety_ready=self._safety_ready,
            drive_policy_valid=self._drive_policy_valid,
            lane_fallback_valid=self._lane_fallback_valid,
            camera_cone_valid=self._camera_cone_valid,
            camera_cone_count=self._camera_cone_count,
            lidar_cone_source_valid=self._lidar_cone_source_valid,
            lidar_cone_path_ready=self._lidar_cone_path_ready,
            lidar_cone_present=self._lidar_cone_present,
        )
        decision = self._manager.update(observation)
        self._publish_decision(decision, now.to_msg())
        self._safety_stop_assertion_pending = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
