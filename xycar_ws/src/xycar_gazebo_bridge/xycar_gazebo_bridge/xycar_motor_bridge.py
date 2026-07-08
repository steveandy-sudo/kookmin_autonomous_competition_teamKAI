import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


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
        self.declare_parameter("steering_min", -0.2881)
        self.declare_parameter("steering_max", 0.2888)
        self.declare_parameter("speed_command_min", -50.0)
        self.declare_parameter("speed_command_max", 100.0)
        self.declare_parameter("speed_gain", 0.08)
        self.declare_parameter("speed_min", -4.0)
        self.declare_parameter("speed_max", 8.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("debug_topic", "/xycar_motor_bridge/debug")

        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.angle_command_min = float(self.get_parameter("angle_command_min").value)
        self.angle_command_max = float(self.get_parameter("angle_command_max").value)
        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.steering_min = float(self.get_parameter("steering_min").value)
        self.steering_max = float(self.get_parameter("steering_max").value)
        self.speed_command_min = float(self.get_parameter("speed_command_min").value)
        self.speed_command_max = float(self.get_parameter("speed_command_max").value)
        self.speed_gain = float(self.get_parameter("speed_gain").value)
        self.speed_min = float(self.get_parameter("speed_min").value)
        self.speed_max = float(self.get_parameter("speed_max").value)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, debug_topic, 10)

        self.last_command_time = None
        self.stop_sent = True
        self._motor_subscriptions = []
        for topic in self._unique_resolved_topics(self.get_parameter("motor_topics").value):
            self._motor_subscriptions.append(
                self.create_subscription(Float32MultiArray, topic, self.on_motor_command, 10)
            )

        self.create_timer(0.05, self.on_timer)
        self.get_logger().info(
            "Xycar motor bridge ready: angle->steering %.4f rad/cmd, speed->%.3f m/s/cmd"
            % (self.steering_gain, self.speed_gain)
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
        steering = clamp(
            self.steering_gain * angle_cmd,
            self.steering_min,
            self.steering_max,
        )
        speed = clamp(self.speed_gain * speed_cmd, self.speed_min, self.speed_max)
        yaw_rate = 0.0
        if abs(speed) > 1.0e-4 and self.wheel_base > 1.0e-4:
            yaw_rate = speed / self.wheel_base * math.tan(steering)

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = yaw_rate
        self.cmd_vel_pub.publish(twist)

        debug = Float32MultiArray()
        debug.data = [angle_cmd, speed_cmd, steering, speed, yaw_rate]
        self.debug_pub.publish(debug)

        self.last_command_time = self.get_clock().now()
        self.stop_sent = False

    def on_timer(self) -> None:
        if self.last_command_time is None or self.stop_sent:
            return

        age = (self.get_clock().now() - self.last_command_time).nanoseconds * 1.0e-9
        if age < self.cmd_timeout_sec:
            return

        self.cmd_vel_pub.publish(Twist())
        self.stop_sent = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XycarMotorBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
