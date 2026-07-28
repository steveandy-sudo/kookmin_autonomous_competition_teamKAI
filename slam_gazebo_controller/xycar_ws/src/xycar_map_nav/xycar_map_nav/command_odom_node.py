#!/usr/bin/env python3
"""Publish approximate odometry from Xycar steering and speed commands."""

from __future__ import annotations

import math
from collections import deque

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

from .command_odom_core import (
    OdomState,
    curvature_for_steering_command,
    first_order_response,
    integrate_planar_velocity,
    integrate_with_heading,
    normalize_angle,
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
        self.speed_deadzone = float(
            self.get_parameter("speed_deadzone_command").value
        )
        self.speed_min = float(self.get_parameter("speed_min_mps").value)
        self.speed_max = float(self.get_parameter("speed_max_mps").value)
        self.speed_delay = float(
            self.get_parameter("speed_delay_sec").value
        )
        self.steering_delay = float(
            self.get_parameter("steering_delay_sec").value
        )
        self.accel_tau = float(
            self.get_parameter("speed_accel_tau_sec").value
        )
        self.brake_tau = float(
            self.get_parameter("speed_brake_tau_sec").value
        )
        self.use_imu_yaw = bool(self.get_parameter("use_imu_yaw").value)
        self.imu_timeout = float(
            self.get_parameter("imu_timeout_sec").value
        )
        self.imu_yaw_sign = float(
            self.get_parameter("imu_yaw_sign").value
        )
        self.imu_yaw_offset = float(
            self.get_parameter("imu_yaw_offset_rad").value
        )
        self.imu_rebase_when_stopped = bool(
            self.get_parameter("imu_rebase_when_stopped").value
        )
        self.stopped_speed_threshold = float(
            self.get_parameter("stopped_speed_threshold_mps").value
        )
        self.commands = [
            float(value)
            for value in self.get_parameter("steering_map_commands").value
        ]
        self.curvatures = [
            float(value)
            for value in self.get_parameter("steering_map_curvatures").value
        ]
        self.state = OdomState()
        self.speed_mps = 0.0
        self.angle_command = 0.0
        self.speed_command = 0.0
        self.last_command_sec = 0.0
        self.last_update_sec = self._now_sec()
        self.angle_history = deque()
        self.speed_history = deque()
        self.use_gyro_yaw = bool(
            self.get_parameter("use_gyro_yaw").value
        )
        self.gyro_bias = float(
            self.get_parameter("gyro_z_bias_rad_s").value
        )
        self.gyro_sign = float(
            self.get_parameter("gyro_z_sign").value
        )
        self.gyro_scale = float(
            self.get_parameter("gyro_z_scale").value
        )
        self.gyro_alpha = min(
            1.0,
            max(0.0, float(self.get_parameter("gyro_low_pass_alpha").value)),
        )
        self.gyro_blend = min(
            1.0,
            max(0.0, float(self.get_parameter("gyro_command_blend").value)),
        )
        self.gyro_timeout = float(
            self.get_parameter("gyro_timeout_sec").value
        )
        self.filtered_gyro_yaw_rate = None
        self.last_gyro_sec = 0.0
        self.last_imu_sec = 0.0
        self.latest_imu_yaw = None
        self.imu_reference_yaw = None
        self.warned_imu_stale = False

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
        if self.use_imu_yaw or self.use_gyro_yaw:
            self.create_subscription(
                Imu,
                str(self.get_parameter("imu_topic").value),
                self._on_imu,
                50,
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
        self.declare_parameter("speed_deadzone_command", 3.0)
        self.declare_parameter("speed_min_mps", -4.0)
        self.declare_parameter("speed_max_mps", 8.0)
        self.declare_parameter("speed_delay_sec", 0.20)
        self.declare_parameter("speed_accel_tau_sec", 0.19)
        self.declare_parameter("speed_brake_tau_sec", 0.09)
        self.declare_parameter("steering_delay_sec", 0.10)
        self.declare_parameter("use_gyro_yaw", False)
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("gyro_z_bias_rad_s", 0.026983)
        self.declare_parameter("gyro_z_sign", -1.0)
        self.declare_parameter("gyro_z_scale", 1.061768)
        self.declare_parameter("gyro_low_pass_alpha", 0.35)
        self.declare_parameter("gyro_command_blend", 0.65)
        self.declare_parameter("gyro_timeout_sec", 0.15)
        self.declare_parameter("use_imu_yaw", True)
        self.declare_parameter("imu_timeout_sec", 0.30)
        self.declare_parameter("imu_yaw_sign", 1.0)
        self.declare_parameter("imu_yaw_offset_rad", 0.0)
        self.declare_parameter("imu_rebase_when_stopped", True)
        self.declare_parameter("stopped_speed_threshold_mps", 0.02)
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
        now_sec = self._now_sec()
        self.angle_command = float(message.data[0])
        self.speed_command = float(message.data[1])
        self.angle_history.append((now_sec, self.angle_command))
        self.speed_history.append((now_sec, self.speed_command))
        self.last_command_sec = now_sec

    def _on_imu(self, message: Imu) -> None:
        now_sec = self._now_sec()
        if self.use_gyro_yaw:
            corrected = (
                self.gyro_sign
                * (float(message.angular_velocity.z) - self.gyro_bias)
                * self.gyro_scale
            )
            if self.filtered_gyro_yaw_rate is None:
                self.filtered_gyro_yaw_rate = corrected
            else:
                self.filtered_gyro_yaw_rate = (
                    self.gyro_alpha * corrected
                    + (1.0 - self.gyro_alpha)
                    * self.filtered_gyro_yaw_rate
                )
            self.last_gyro_sec = now_sec

        if not self.use_imu_yaw:
            return
        quaternion = message.orientation
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if norm < 0.5 or not math.isfinite(norm):
            return
        yaw = math.atan2(
            2.0
            * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0
            - 2.0
            * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )
        yaw = normalize_angle(self.imu_yaw_sign * yaw)
        self.latest_imu_yaw = yaw
        if self.imu_reference_yaw is None:
            self.imu_reference_yaw = yaw
            self.get_logger().info(
                "IMU yaw reference acquired; SLAM odometry heading is now "
                "IMU-assisted"
            )
        self.last_imu_sec = now_sec
        self.warned_imu_stale = False

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _delayed_value(history, cutoff_sec: float, default: float) -> float:
        while len(history) >= 2 and history[1][0] <= cutoff_sec:
            history.popleft()
        if history and history[0][0] <= cutoff_sec:
            return float(history[0][1])
        return float(default)

    def _speed_target(self, command: float) -> float:
        if abs(command) < self.speed_deadzone:
            return 0.0
        return min(
            self.speed_max,
            max(self.speed_min, command * self.speed_gain),
        )

    def _on_reset(self, _, response):
        self.state = OdomState()
        self.speed_mps = 0.0
        self.angle_history.clear()
        self.speed_history.clear()
        self.last_update_sec = self._now_sec()
        self.imu_reference_yaw = self.latest_imu_yaw
        self.filtered_gyro_yaw_rate = None
        self.last_gyro_sec = 0.0
        self.warned_imu_stale = False
        return response

    def _on_timer(self) -> None:
        now_monotonic = self._now_sec()
        dt = min(
            self.maximum_dt,
            max(0.0, now_monotonic - self.last_update_sec),
        )
        self.last_update_sec = now_monotonic
        command_fresh = (
            now_monotonic - self.last_command_sec <= self.command_timeout
        )
        delayed_angle = self._delayed_value(
            self.angle_history,
            now_monotonic - self.steering_delay,
            self.angle_command,
        )
        delayed_speed = self._delayed_value(
            self.speed_history,
            now_monotonic - self.speed_delay,
            0.0,
        )
        speed_target = (
            self._speed_target(delayed_speed) if command_fresh else 0.0
        )
        accelerating = abs(speed_target) > abs(self.speed_mps)
        tau = self.accel_tau if accelerating else self.brake_tau
        previous_speed = self.speed_mps
        self.speed_mps = first_order_response(
            previous_speed,
            speed_target,
            dt_sec=dt,
            time_constant_sec=tau,
        )
        speed_mps = 0.5 * (previous_speed + self.speed_mps)
        curvature = curvature_for_steering_command(
            delayed_angle,
            self.commands,
            self.curvatures,
        )
        command_yaw_rate = speed_mps * curvature
        imu_fresh = (
            self.use_imu_yaw
            and self.latest_imu_yaw is not None
            and self.imu_reference_yaw is not None
            and now_monotonic - self.last_imu_sec <= self.imu_timeout
        )
        if imu_fresh:
            if (
                self.imu_rebase_when_stopped
                and abs(speed_mps) <= self.stopped_speed_threshold
            ):
                # An Ackermann car cannot rotate in place. Rebase the AHRS
                # while stopped so its static yaw bias cannot move the map.
                self.imu_reference_yaw = normalize_angle(
                    self.latest_imu_yaw
                    + self.imu_yaw_offset
                    - self.state.yaw
                )
                heading = self.state.yaw
            else:
                heading = normalize_angle(
                    self.latest_imu_yaw
                    - self.imu_reference_yaw
                    + self.imu_yaw_offset
                )
            yaw_delta = normalize_angle(heading - self.state.yaw)
            self.state = integrate_with_heading(
                self.state,
                speed_mps=speed_mps,
                heading_rad=heading,
                dt_sec=dt,
            )
            yaw_rate = yaw_delta / dt if dt > 1.0e-6 else 0.0
        else:
            yaw_rate = command_yaw_rate
            gyro_fresh = (
                self.use_gyro_yaw
                and self.filtered_gyro_yaw_rate is not None
                and now_monotonic - self.last_gyro_sec <= self.gyro_timeout
            )
            if gyro_fresh:
                yaw_rate = (
                    (1.0 - self.gyro_blend) * command_yaw_rate
                    + self.gyro_blend * float(self.filtered_gyro_yaw_rate)
                )
            self.state = integrate_planar_velocity(
                self.state,
                speed_mps=speed_mps,
                yaw_rate_rad_s=yaw_rate,
                dt_sec=dt,
            )
            if self.use_imu_yaw and not self.warned_imu_stale:
                self.get_logger().warning(
                    "IMU orientation yaw is unavailable or stale; falling "
                    "back to gyro/command-derived heading"
                )
                self.warned_imu_stale = True
        self._publish(self.speed_mps, yaw_rate)

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
