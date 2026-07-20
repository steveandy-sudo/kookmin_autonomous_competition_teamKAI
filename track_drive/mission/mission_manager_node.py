"""20 Hz ROS2 integration-test adapter for Mission Manager V0.1."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String

from track_drive.mission.mission_manager import MissionManager
from track_drive.mission.mission_types import MissionManagerConfig, MissionObservation


UPDATE_PERIOD_SEC = 0.05


class MissionManagerNode(Node):
    """임시 의미 기반 토픽을 MissionObservation으로 변환한다."""

    def __init__(self) -> None:
        super().__init__("mission_manager")

        defaults = MissionManagerConfig()
        self.declare_parameter(
            "start_signal_hold_sec", defaults.start_signal_hold_sec
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
            "lane_fallback_enter_hold_sec",
            defaults.lane_fallback_enter_hold_sec,
        )
        self.declare_parameter(
            "minimum_cone_count", defaults.minimum_cone_count
        )
        self.declare_parameter(
            "minimum_cone_confidence",
            defaults.minimum_cone_confidence,
        )
        self.declare_parameter(
            "status_log_period_sec", defaults.status_log_period_sec
        )

        config = MissionManagerConfig(
            start_signal_hold_sec=self._float_parameter(
                "start_signal_hold_sec"
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
            lane_fallback_enter_hold_sec=self._float_parameter(
                "lane_fallback_enter_hold_sec"
            ),
            minimum_cone_count=int(
                self.get_parameter("minimum_cone_count").value
            ),
            minimum_cone_confidence=self._float_parameter(
                "minimum_cone_confidence"
            ),
            status_log_period_sec=self._float_parameter(
                "status_log_period_sec"
            ),
        )

        self._manager = MissionManager(config)
        self._manager.set_logger(self.get_logger().info)

        self._emergency_stop = False
        self._emergency_assertion_pending = False
        self._start_signal_go = False
        self._drive_policy_valid = False
        self._lane_fallback_valid = False
        self._cone_detected = False
        self._cone_confidence = 0.0
        self._cone_count = 0
        self._cone_exit_ready = False

        # 아래 토픽은 V0.1 통합시험용 adapter 입력이다.
        self.create_subscription(
            String, "/mission/override", self._on_override, 10
        )
        self.create_subscription(
            Bool,
            "/mission/input/emergency_stop",
            self._on_emergency_stop,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/input/start_signal_go",
            self._on_start_signal_go,
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
            "/mission/input/cone_detected",
            self._on_cone_detected,
            10,
        )
        self.create_subscription(
            Bool,
            "/mission/input/cone_exit_ready",
            self._on_cone_exit_ready,
            10,
        )
        self.create_subscription(
            Float32,
            "/mission/input/cone_confidence",
            self._on_cone_confidence,
            10,
        )
        self.create_subscription(
            Int32,
            "/mission/input/cone_count",
            self._on_cone_count,
            10,
        )

        self.create_timer(UPDATE_PERIOD_SEC, self._on_update_timer)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _on_override(self, message: String) -> None:
        self._manager.set_manual_override(message.data)

    def _on_emergency_stop(self, message: Bool) -> None:
        self._emergency_stop = bool(message.data)
        if message.data:
            # true/false가 한 timer 사이에 들어와도 assertion을 한 번 전달한다.
            self._emergency_assertion_pending = True

    def _on_start_signal_go(self, message: Bool) -> None:
        self._start_signal_go = bool(message.data)

    def _on_drive_policy_valid(self, message: Bool) -> None:
        self._drive_policy_valid = bool(message.data)

    def _on_lane_fallback_valid(self, message: Bool) -> None:
        self._lane_fallback_valid = bool(message.data)

    def _on_cone_detected(self, message: Bool) -> None:
        self._cone_detected = bool(message.data)

    def _on_cone_exit_ready(self, message: Bool) -> None:
        self._cone_exit_ready = bool(message.data)

    def _on_cone_confidence(self, message: Float32) -> None:
        self._cone_confidence = float(message.data)

    def _on_cone_count(self, message: Int32) -> None:
        self._cone_count = int(message.data)

    def _on_update_timer(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        observation = MissionObservation(
            now_sec=now_sec,
            emergency_stop=(
                self._emergency_stop
                or self._emergency_assertion_pending
            ),
            start_signal_go=self._start_signal_go,
            drive_policy_valid=self._drive_policy_valid,
            lane_fallback_valid=self._lane_fallback_valid,
            cone_detected=self._cone_detected,
            cone_confidence=self._cone_confidence,
            cone_count=self._cone_count,
            cone_exit_ready=self._cone_exit_ready,
        )
        self._manager.update(observation)
        self._emergency_assertion_pending = False


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
