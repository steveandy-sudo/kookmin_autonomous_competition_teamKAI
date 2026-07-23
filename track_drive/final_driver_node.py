"""Single 50 Hz motor publisher for Mission Manager selected controllers."""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray
from teamkai_interfaces.msg import LaneFallbackCommand
from teamkai_interfaces.msg import MissionDecision as MissionDecisionMsg
from xycar_msgs.msg import XycarMotor

from track_drive.final_driver.cone_input import ConeInputConfig
from track_drive.final_driver.input_selector import DriveInputSelectorConfig
from track_drive.final_driver.runtime import (
    FinalDriverRuntime,
    FinalDriverRuntimeConfig,
    stop_decision,
)
from track_drive.final_driver.steering_converter import SteeringCalibration
from track_drive.mission.mission_types import MissionDecision
from track_drive.mission.states import ControlMode, MissionState


DEFAULT_PUBLISH_RATE_HZ = 50.0

SOURCE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
DECISION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
MOTOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

_MISSION_STATES = {
    MissionDecisionMsg.MISSION_STATE_WAIT_START_SIGNAL: (
        MissionState.WAIT_START_SIGNAL
    ),
    MissionDecisionMsg.MISSION_STATE_LANE_DRIVING: (
        MissionState.LANE_DRIVING
    ),
    MissionDecisionMsg.MISSION_STATE_CONE_SECTION: (
        MissionState.CONE_SECTION
    ),
    MissionDecisionMsg.MISSION_STATE_FIXED_OBSTACLE_SECTION: (
        MissionState.FIXED_OBSTACLE_SECTION
    ),
    MissionDecisionMsg.MISSION_STATE_OVERTAKE_SECTION: (
        MissionState.OVERTAKE_SECTION
    ),
    MissionDecisionMsg.MISSION_STATE_ROUTE_SELECTION: (
        MissionState.ROUTE_SELECTION
    ),
    MissionDecisionMsg.MISSION_STATE_SHORTCUT_SECTION: (
        MissionState.SHORTCUT_SECTION
    ),
}
_CONTROL_MODES = {
    MissionDecisionMsg.CONTROL_MODE_STOP: ControlMode.STOP,
    MissionDecisionMsg.CONTROL_MODE_NORMAL_IL: ControlMode.NORMAL_IL,
    MissionDecisionMsg.CONTROL_MODE_CONE_DRIVE_RULE: (
        ControlMode.CONE_DRIVE_RULE
    ),
    MissionDecisionMsg.CONTROL_MODE_LANE_FALLBACK: (
        ControlMode.LANE_FALLBACK
    ),
    MissionDecisionMsg.CONTROL_MODE_FIXED_OBSTACLE_RULE: (
        ControlMode.FIXED_OBSTACLE_RULE
    ),
    MissionDecisionMsg.CONTROL_MODE_VEHICLE_FOLLOW: (
        ControlMode.VEHICLE_FOLLOW
    ),
    MissionDecisionMsg.CONTROL_MODE_VEHICLE_OVERTAKE: (
        ControlMode.VEHICLE_OVERTAKE
    ),
    MissionDecisionMsg.CONTROL_MODE_SHORTCUT_RULE: (
        ControlMode.SHORTCUT_RULE
    ),
}
_SELECTED_SOURCES = {
    MissionDecisionMsg.SOURCE_NONE: "none",
    MissionDecisionMsg.SOURCE_DRIVE_IL: "drive_il",
    MissionDecisionMsg.SOURCE_CONE_RULE: "cone_rule",
    MissionDecisionMsg.SOURCE_LANE_FALLBACK: "lane_fallback",
    MissionDecisionMsg.SOURCE_FIXED_OBSTACLE_RULE: "fixed_obstacle_rule",
    MissionDecisionMsg.SOURCE_VEHICLE_RULE: "vehicle_rule",
    MissionDecisionMsg.SOURCE_SHORTCUT: "shortcut",
}
_SPEED_PROFILES = {
    MissionDecisionMsg.SPEED_PROFILE_STOP: "stop",
    MissionDecisionMsg.SPEED_PROFILE_NORMAL: "normal",
    MissionDecisionMsg.SPEED_PROFILE_CONE: "cone",
    MissionDecisionMsg.SPEED_PROFILE_FALLBACK: "fallback",
    MissionDecisionMsg.SPEED_PROFILE_OBSTACLE: "obstacle",
    MissionDecisionMsg.SPEED_PROFILE_VEHICLE_FOLLOW: "vehicle_follow",
    MissionDecisionMsg.SPEED_PROFILE_VEHICLE_OVERTAKE: "vehicle_overtake",
    MissionDecisionMsg.SPEED_PROFILE_SHORTCUT: "shortcut",
}


def _decision_from_message(message: MissionDecisionMsg) -> MissionDecision:
    try:
        return MissionDecision(
            mission_state=_MISSION_STATES[int(message.mission_state)],
            control_mode=_CONTROL_MODES[int(message.control_mode)],
            selected_source=_SELECTED_SOURCES[int(message.selected_source)],
            speed_profile=_SPEED_PROFILES[int(message.speed_profile)],
            stop_required=bool(message.stop_required),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("unsupported MissionDecision code") from exc


class FinalDriverNode(Node):
    """Apply one selected source and remain the sole motor publisher."""

    def __init__(self) -> None:
        super().__init__("final_driver")

        self.declare_parameter("decision_topic", "/mission/decision")
        self.declare_parameter("il_debug_topic", "/il/policy_debug")
        self.declare_parameter(
            "lane_command_topic",
            "/lane_fallback/command",
        )
        self.declare_parameter("cone_command_topic", "/my_rule/cone_cmd")
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )
        self.declare_parameter("decision_timeout_sec", 0.2)
        self.declare_parameter("il_source_timeout_sec", 0.5)
        self.declare_parameter("lane_command_timeout_sec", 0.2)
        self.declare_parameter("lane_max_steering_angle_deg", 26.0)
        self.declare_parameter("cone_source_timeout_sec", 0.2)
        self.declare_parameter("cone_minimum_confidence", 0.2)
        self.declare_parameter("cone_max_steering_angle_deg", 26.0)
        self.declare_parameter("cone_minimum_requested_speed", 9.5)
        self.declare_parameter("cone_maximum_requested_speed", 21.0)
        self.declare_parameter(
            "physical_angle_deg",
            [0.0, 4.0, 10.0, 16.0, 26.0],
        )
        self.declare_parameter(
            "xycar_steering_command",
            [0.0, 10.0, 20.0, 30.0, 42.0],
        )
        self.declare_parameter("physical_steering_sign", 1.0)
        self.declare_parameter("fallback_speed", Parameter.Type.DOUBLE)

        fallback_speed = self._required_positive_float("fallback_speed")
        publish_rate_hz = self._positive_float("publish_rate_hz")

        selector_config = DriveInputSelectorConfig(
            fallback_speed=fallback_speed,
            il_source_timeout_sec=self._float("il_source_timeout_sec"),
            lane_command_timeout_sec=self._float(
                "lane_command_timeout_sec"
            ),
            lane_max_steering_angle_deg=self._float(
                "lane_max_steering_angle_deg"
            ),
            cone=ConeInputConfig(
                source_timeout_sec=self._float(
                    "cone_source_timeout_sec"
                ),
                minimum_confidence=self._float(
                    "cone_minimum_confidence"
                ),
                max_steering_angle_deg=self._float(
                    "cone_max_steering_angle_deg"
                ),
                minimum_requested_speed=self._float(
                    "cone_minimum_requested_speed"
                ),
                maximum_requested_speed=self._float(
                    "cone_maximum_requested_speed"
                ),
            ),
        )
        steering_calibration = SteeringCalibration(
            physical_angle_deg=self._float_tuple("physical_angle_deg"),
            xycar_command=self._float_tuple(
                "xycar_steering_command"
            ),
            physical_steering_sign=self._float(
                "physical_steering_sign"
            ),
        )
        self._runtime = FinalDriverRuntime(
            FinalDriverRuntimeConfig(
                input_selector=selector_config,
                decision_timeout_sec=self._float(
                    "decision_timeout_sec"
                ),
                steering_calibration=steering_calibration,
            )
        )

        decision_topic = self._string("decision_topic")
        il_debug_topic = self._string("il_debug_topic")
        lane_command_topic = self._string("lane_command_topic")
        cone_command_topic = self._string("cone_command_topic")
        motor_topic = self._string("motor_topic")

        self._motor_publisher = self.create_publisher(
            XycarMotor,
            motor_topic,
            MOTOR_QOS,
        )
        self.create_subscription(
            MissionDecisionMsg,
            decision_topic,
            self._on_decision,
            DECISION_QOS,
        )
        self.create_subscription(
            Float32MultiArray,
            il_debug_topic,
            self._on_il,
            SOURCE_QOS,
        )
        self.create_subscription(
            LaneFallbackCommand,
            lane_command_topic,
            self._on_lane,
            SOURCE_QOS,
        )
        self.create_subscription(
            Float32MultiArray,
            cone_command_topic,
            self._on_cone,
            SOURCE_QOS,
        )

        self._last_status: tuple[bool, str, bool] | None = None
        self.create_timer(1.0 / publish_rate_hz, self._publish_selected)

        self.get_logger().info(
            "Final Driver ready | "
            f"rate={publish_rate_hz:.1f}Hz motor={motor_topic} "
            f"fallback_speed={fallback_speed:.3f}"
        )

    def _float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _positive_float(self, name: str) -> float:
        value = self._float(name)
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _required_positive_float(self, name: str) -> float:
        try:
            return self._positive_float(name)
        except Exception as exc:
            raise RuntimeError(
                f"{name} is required; pass {name}:=<vehicle-tested value>"
            ) from exc

    def _float_tuple(self, name: str) -> tuple[float, ...]:
        values = tuple(
            float(value) for value in self.get_parameter(name).value
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain only finite values")
        return values

    def _string(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f"{name} must not be empty")
        return value

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_decision(self, message: MissionDecisionMsg) -> None:
        try:
            decision = _decision_from_message(message)
        except ValueError:
            decision = stop_decision()
            self.get_logger().warning(
                "Unsupported MissionDecision received; requesting stop"
            )
        self._runtime.update_decision(
            decision,
            receive_sec=self._now_sec(),
        )

    def _on_il(self, message: Float32MultiArray) -> None:
        self._runtime.update_il(
            message.data,
            receive_sec=self._now_sec(),
        )

    def _on_lane(self, message: LaneFallbackCommand) -> None:
        self._runtime.update_lane(
            steering_angle_deg=message.steering_angle_deg,
            command_valid=message.valid,
            receive_sec=self._now_sec(),
        )

    def _on_cone(self, message: Float32MultiArray) -> None:
        self._runtime.update_cone(
            message.data,
            receive_sec=self._now_sec(),
        )

    def _publish_selected(self) -> None:
        command = self._runtime.command(now_sec=self._now_sec())
        message = XycarMotor()
        if command.valid:
            message.angle = float(command.steering_command)
            message.speed = float(command.requested_speed)
        else:
            message.angle = 0.0
            message.speed = 0.0
        self._motor_publisher.publish(message)

        status = (
            command.valid,
            command.selected_source,
            command.steering_held,
        )
        if status != self._last_status:
            self._last_status = status
            self.get_logger().info(
                "Final Driver selection | "
                f"source={command.selected_source} "
                f"valid={str(command.valid).lower()} "
                f"held={str(command.steering_held).lower()}"
            )

    def publish_stop(self) -> None:
        """Send one neutral command during an orderly node shutdown."""

        message = XycarMotor()
        message.angle = 0.0
        message.speed = 0.0
        self._motor_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FinalDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
