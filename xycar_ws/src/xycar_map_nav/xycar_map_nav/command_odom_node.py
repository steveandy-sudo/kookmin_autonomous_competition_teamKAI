#!/usr/bin/env python3
"""Publish approximate odometry from Xycar steering and speed commands."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

from .command_odom_core import (
    OdomState,
    curvature_for_steering_command,
    integrate_ackermann,
)


class CommandOdomNode(Node):
    """Dead-reckon only enough to seed scan matching when encoders are absent."""

    def __init__(self) -> None:
        super().__init__("xycar_command_odom")
        self._declare_parameters()
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.speed_gain = float(
            self.get_parameter("speed_gain_mps_per_command").value
        )
        self.command_timeout = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.maximum_dt = float(self.get_parameter("maximum_dt_sec").value)
        self.commands = [
            float(value)
            for value in self.get_parameter("steering_map_commands").value
        ]
        self.curvatures = [
            float(value)
            for value in self.get_parameter("steering_map_curvatures").value
        ]
        self.state = OdomState()
        self.angle_command = 0.0
        self.speed_command = 0.0
        self.last_command_sec = 0.0
        self.last_update_sec = time.monotonic()

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 20
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("motor_topic").value),
            self._on_motor,
            20,
        )
        self.create_service(Empty, "~/reset", self._on_reset)
        rate = max(10.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().warning(
            "command odometry is approximate and will drift; use it only as "
            "a scan-matching seed until encoder odometry is available"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("command_timeout_sec", 0.30)
        self.declare_parameter("maximum_dt_sec", 0.10)
        self.declare_parameter("speed_gain_mps_per_command", 0.080612)
        self.declare_parameter(
            "steering_map_commands",
            [
                -42.0, -40.0, -35.0, -30.0, -20.0, -10.0, 0.0,
                10.0, 20.0, 30.0, 35.0, 40.0, 42.0,
            ],
        )
        self.declare_parameter(
            "steering_map_curvatures",
            [
                1.502435, 1.383494, 1.174860, 0.922781, 0.552809,
                0.194230, 0.0, -0.556883, -0.959829, -1.369323,
                -1.601706, -1.853397, -1.939236,
            ],
        )

    def _on_motor(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            return
        self.angle_command = float(message.data[0])
        self.speed_command = float(message.data[1])
        self.last_command_sec = time.monotonic()

    def _on_reset(self, _, response):
        self.state = OdomState()
        self.last_update_sec = time.monotonic()
        return response

    def _on_timer(self) -> None:
        now_monotonic = time.monotonic()
        dt = min(
            self.maximum_dt,
            max(0.0, now_monotonic - self.last_update_sec),
        )
        self.last_update_sec = now_monotonic
        command_fresh = (
            now_monotonic - self.last_command_sec <= self.command_timeout
        )
        speed_mps = (
            self.speed_command * self.speed_gain if command_fresh else 0.0
        )
        curvature = curvature_for_steering_command(
            self.angle_command,
            self.commands,
            self.curvatures,
        )
        self.state = integrate_ackermann(
            self.state,
            speed_mps=speed_mps,
            curvature_per_m=curvature,
            dt_sec=dt,
        )
        self._publish(speed_mps, speed_mps * curvature)

    def _publish(self, speed_mps: float, yaw_rate: float) -> None:
        stamp = self.get_clock().now().to_msg()
        sine = math.sin(self.state.yaw * 0.5)
        cosine = math.cos(self.state.yaw * 0.5)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.state.x
        odom.pose.pose.position.y = self.state.y
        odom.pose.pose.orientation.z = sine
        odom.pose.pose.orientation.w = cosine
        odom.twist.twist.linear.x = float(speed_mps)
        odom.twist.twist.angular.z = float(yaw_rate)
        odom.pose.covariance[0] = 0.10
        odom.pose.covariance[7] = 0.10
        odom.pose.covariance[35] = 0.20
        odom.twist.covariance[0] = 0.15
        odom.twist.covariance[35] = 0.25
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.state.x
        transform.transform.translation.y = self.state.y
        transform.transform.rotation.z = sine
        transform.transform.rotation.w = cosine
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandOdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
