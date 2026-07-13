import math
from bisect import bisect_right
from collections import deque
from typing import Iterable, Sequence

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def interpolate_clamped(value: float, inputs: Sequence[float], outputs: Sequence[float]) -> float:
    """Piecewise-linear interpolation with endpoint clamping."""
    if len(inputs) != len(outputs) or len(inputs) < 2:
        raise ValueError("lookup inputs and outputs must have the same length >= 2")
    if any(right <= left for left, right in zip(inputs, inputs[1:])):
        raise ValueError("lookup inputs must be strictly increasing")
    if value <= inputs[0]:
        return float(outputs[0])
    if value >= inputs[-1]:
        return float(outputs[-1])

    upper = bisect_right(inputs, value)
    lower = upper - 1
    fraction = (value - inputs[lower]) / (inputs[upper] - inputs[lower])
    return float(outputs[lower] + fraction * (outputs[upper] - outputs[lower]))


def apply_command_deadzone(command: float, deadzone: float) -> float:
    """Return zero below the measured launch threshold."""
    return 0.0 if abs(command) < max(0.0, deadzone) else float(command)


def first_order_response(current: float, target: float, dt: float, tau: float) -> float:
    """Advance a first-order response without depending on the timer rate."""
    if dt <= 0.0:
        return float(current)
    if tau <= 0.0:
        return float(target)
    alpha = -math.expm1(-dt / tau)
    return float(current + alpha * (target - current))


class XycarMotorBridge(Node):
    """Convert real Xycar [angle, speed] commands into Gazebo cmd_vel."""

    def __init__(self) -> None:
        super().__init__("xycar_motor_bridge")

        self.declare_parameter("motor_topics", ["xycar_motor", "/xycar_motor"])
        self.declare_parameter("cmd_vel_topic", "/model/xycar_ackermann/cmd_vel")
        self.declare_parameter("wheel_base", 0.32)
        self.declare_parameter("angle_command_min", -50.0)
        self.declare_parameter("angle_command_max", 100.0)
        self.declare_parameter("steering_gain", -0.0068)
        self.declare_parameter("steering_min", -0.56)
        self.declare_parameter("steering_max", 0.56)
        self.declare_parameter("use_measured_steering_map", True)
        self.declare_parameter(
            "steering_map_commands",
            [
                -42.0,
                -40.0,
                -35.0,
                -30.0,
                -20.0,
                -10.0,
                0.0,
                10.0,
                20.0,
                30.0,
                35.0,
                40.0,
                42.0,
            ],
        )
        self.declare_parameter(
            "steering_map_curvatures",
            [
                1.502435,
                1.383494,
                1.174860,
                0.922781,
                0.552809,
                0.194230,
                0.0,
                -0.556883,
                -0.959829,
                -1.369323,
                -1.601706,
                -1.853397,
                -1.939236,
            ],
        )
        self.declare_parameter("speed_command_min", -50.0)
        self.declare_parameter("speed_command_max", 100.0)
        self.declare_parameter("speed_gain", 0.080612)
        self.declare_parameter("speed_deadzone_command", 3.0)
        self.declare_parameter("speed_min", -4.0)
        self.declare_parameter("speed_max", 8.0)
        self.declare_parameter("steering_delay_sec", 0.10)
        self.declare_parameter("speed_delay_sec", 0.20)
        self.declare_parameter("speed_accel_tau_sec", 0.19)
        self.declare_parameter("speed_brake_tau_sec", 0.09)
        self.declare_parameter("command_update_period_sec", 0.005)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("debug_topic", "/xycar_motor_bridge/debug")

        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.angle_command_min = float(self.get_parameter("angle_command_min").value)
        self.angle_command_max = float(self.get_parameter("angle_command_max").value)
        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.steering_min = float(self.get_parameter("steering_min").value)
        self.steering_max = float(self.get_parameter("steering_max").value)
        self.use_measured_steering_map = bool(
            self.get_parameter("use_measured_steering_map").value
        )
        self.steering_map_commands = [
            float(value) for value in self.get_parameter("steering_map_commands").value
        ]
        self.steering_map_curvatures = [
            float(value) for value in self.get_parameter("steering_map_curvatures").value
        ]
        if self.use_measured_steering_map:
            interpolate_clamped(
                0.0,
                self.steering_map_commands,
                self.steering_map_curvatures,
            )

        self.speed_command_min = float(self.get_parameter("speed_command_min").value)
        self.speed_command_max = float(self.get_parameter("speed_command_max").value)
        self.speed_gain = float(self.get_parameter("speed_gain").value)
        self.speed_deadzone_command = max(
            0.0, float(self.get_parameter("speed_deadzone_command").value)
        )
        self.speed_min = float(self.get_parameter("speed_min").value)
        self.speed_max = float(self.get_parameter("speed_max").value)
        self.steering_delay_sec = max(
            0.0, float(self.get_parameter("steering_delay_sec").value)
        )
        self.speed_delay_sec = max(0.0, float(self.get_parameter("speed_delay_sec").value))
        self.speed_accel_tau_sec = max(
            0.0, float(self.get_parameter("speed_accel_tau_sec").value)
        )
        self.speed_brake_tau_sec = max(
            0.0, float(self.get_parameter("speed_brake_tau_sec").value)
        )
        self.command_update_period_sec = max(
            0.001, float(self.get_parameter("command_update_period_sec").value)
        )
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, debug_topic, 10)

        self.last_command_time = None
        self.stop_sent = True
        self.active_angle_cmd = 0.0
        self.active_speed_cmd = 0.0
        self.applied_speed_mps = 0.0
        self.last_update_ns = self.get_clock().now().nanoseconds
        self.angle_queue = deque()
        self.speed_queue = deque()
        self._motor_subscriptions = []
        for topic in self._unique_resolved_topics(self.get_parameter("motor_topics").value):
            self._motor_subscriptions.append(
                self.create_subscription(Float32MultiArray, topic, self.on_motor_command, 10)
            )

        self.create_timer(self.command_update_period_sec, self.on_timer)
        profile = (
            "2026-07-13 measured curvature map"
            if self.use_measured_steering_map
            else "linear"
        )
        self.get_logger().info(
            "Xycar motor bridge ready: %s, speed %.6f m/s/cmd, deadzone %.1f, "
            "delays steer=%.3fs speed=%.3fs, speed tau accel=%.3fs brake=%.3fs"
            % (
                profile,
                self.speed_gain,
                self.speed_deadzone_command,
                self.steering_delay_sec,
                self.speed_delay_sec,
                self.speed_accel_tau_sec,
                self.speed_brake_tau_sec,
            )
        )

    def _unique_resolved_topics(self, topics: Iterable[str]) -> list[str]:
        resolved = set()
        unique = []
        for topic in topics:
            topic_name = str(topic)
            resolved_name = self.resolve_topic_name(topic_name)
            if resolved_name in resolved:
                continue
            resolved.add(resolved_name)
            unique.append(topic_name)
            self.get_logger().info(f"subscribing motor topic: {resolved_name}")
        return unique

    def on_motor_command(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warning("xycar_motor command ignored: expected [angle, speed]")
            return

        angle_cmd = clamp(float(msg.data[0]), self.angle_command_min, self.angle_command_max)
        speed_cmd = clamp(float(msg.data[1]), self.speed_command_min, self.speed_command_max)
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        self.angle_queue.append(
            (now_ns + int(self.steering_delay_sec * 1.0e9), angle_cmd)
        )
        self.speed_queue.append((now_ns + int(self.speed_delay_sec * 1.0e9), speed_cmd))
        self.last_command_time = now
        self.stop_sent = False

    def _steering_response(self, angle_cmd: float) -> tuple[float, float]:
        if self.use_measured_steering_map:
            curvature = interpolate_clamped(
                angle_cmd,
                self.steering_map_commands,
                self.steering_map_curvatures,
            )
            steering = math.atan(self.wheel_base * curvature)
        else:
            steering = self.steering_gain * angle_cmd

        steering = clamp(steering, self.steering_min, self.steering_max)
        if self.wheel_base <= 1.0e-4:
            return steering, 0.0
        return steering, math.tan(steering) / self.wheel_base

    def _target_speed_mps(self) -> float:
        speed_cmd = apply_command_deadzone(
            self.active_speed_cmd,
            self.speed_deadzone_command,
        )
        return clamp(
            self.speed_gain * speed_cmd,
            self.speed_min,
            self.speed_max,
        )

    def _publish_active_command(self, dt: float) -> None:
        steering, curvature = self._steering_response(self.active_angle_cmd)
        target_speed = self._target_speed_mps()
        accelerating = abs(target_speed) > abs(self.applied_speed_mps) and (
            target_speed == 0.0
            or self.applied_speed_mps == 0.0
            or math.copysign(1.0, target_speed) == math.copysign(1.0, self.applied_speed_mps)
        )
        tau = self.speed_accel_tau_sec if accelerating else self.speed_brake_tau_sec
        self.applied_speed_mps = first_order_response(
            self.applied_speed_mps,
            target_speed,
            dt,
            tau,
        )
        if abs(self.applied_speed_mps - target_speed) < 1.0e-5:
            self.applied_speed_mps = target_speed
        speed = self.applied_speed_mps
        yaw_rate = speed * curvature

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = yaw_rate
        self.cmd_vel_pub.publish(twist)

        debug = Float32MultiArray()
        debug.data = [
            self.active_angle_cmd,
            self.active_speed_cmd,
            steering,
            speed,
            yaw_rate,
            curvature,
        ]
        self.debug_pub.publish(debug)

    def on_timer(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        dt = max(0.0, (now_ns - self.last_update_ns) * 1.0e-9)
        self.last_update_ns = now_ns
        while self.angle_queue and self.angle_queue[0][0] <= now_ns:
            _, self.active_angle_cmd = self.angle_queue.popleft()
        while self.speed_queue and self.speed_queue[0][0] <= now_ns:
            _, self.active_speed_cmd = self.speed_queue.popleft()

        if self.last_command_time is not None and not self.stop_sent:
            age = (now - self.last_command_time).nanoseconds * 1.0e-9
            if age >= self.cmd_timeout_sec:
                self.angle_queue.clear()
                self.speed_queue.clear()
                self.active_angle_cmd = 0.0
                self.active_speed_cmd = 0.0
                self.applied_speed_mps = 0.0
                self.cmd_vel_pub.publish(Twist())
                self.stop_sent = True
                return

        if not self.stop_sent:
            self._publish_active_command(dt)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XycarMotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
