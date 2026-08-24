#!/usr/bin/env python3
"""Fail-closed Nav2 Twist to calibrated Xycar motor adapter."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String
from xycar_msgs.msg import XycarVescState

from .command_core import (
    DirectionChangeGuard,
    Footprint,
    MotorCalibration,
    TransientZeroHold,
    rotate_request_to_reverse_crawl,
    stopped_request_to_reverse_crawl,
    scan_points_in_base,
    should_hold_for_steering_settle,
    slew,
    swept_footprint_collision,
    twist_to_motor_command,
)
from .parking_keyboard_core import parse_status_line


GATE_REASON_KO = {
    "ok": "정상(주행 허용)",
    "not_authorized": "미션 주행 권한이 아직 없음",
    "no_cmd_vel": "Nav2 주행 명령(/cmd_vel)이 아직 없음",
    "stale_cmd_vel": "Nav2 주행 명령(/cmd_vel)이 끊김",
    "no_scan": "주차용 LiDAR 데이터가 아직 없음",
    "stale_scan": "주차용 LiDAR 데이터가 끊김",
    "insufficient_scan": "유효한 LiDAR 점 개수가 부족함",
    "non_finite_twist": "유효하지 않은 주행 명령을 받음",
    "stopped": "정지 명령",
    "rotate_in_place_rejected": "차량이 수행할 수 없는 제자리 회전 명령을 거부함",
    "reverse_align_rotate_crawl": "제자리 회전 대신 조향을 유지하며 -4 후진 중",
    "reverse_align_zero_crawl": "Nav2 0속도 대신 마지막 조향을 유지하며 -4 후진 중",
    "lidar_swept_collision": "예상 주행 궤적에서 장애물을 감지함",
    "obstacle_reverse_settle": "장애물 회피 후진 전 안전 정지 중",
    "obstacle_reverse_active": "장애물에서 벗어나기 위해 약 20cm 직선 후진 중",
    "obstacle_reverse_complete": "짧은 후진 완료 후 경로 재계산 대기 중",
    "direction_change_dwell": "전진·후진 전환 전 안전 정지 중",
    "steering_settle": "목표 조향각 정렬 중",
    "transient_nav2_zero_hold": "짧은 Nav2 0 명령을 무시하고 이전 ±4를 유지 중",
    "no_vesc_telemetry": "VESC 상태정보가 아직 없음",
    "stale_vesc_telemetry": "VESC 상태정보가 끊김",
    "vesc_low_voltage_stop": "VESC 입력전압이 모터 정지 기준 이하임",
    "vesc_fault": "VESC 하드웨어 fault가 발생함",
    "shutdown": "노드 종료로 정지",
}


class CmdVelAdapter(Node):
    """Apply physical calibration and an independent LiDAR stop shield."""

    def __init__(self) -> None:
        super().__init__("parking_cmd_vel_adapter")
        self._declare_parameters()

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.motor_topic = str(self.get_parameter("motor_topic").value)
        self.shadow_topic = str(self.get_parameter("shadow_topic").value)
        self.authorization_topic = str(
            self.get_parameter("authorization_topic").value
        )
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.mission_state_topic = str(
            self.get_parameter("mission_state_topic").value
        )
        self.vesc_state_topic = str(self.get_parameter("vesc_state_topic").value)
        self.drive_enabled = bool(self.get_parameter("drive_enabled").value)
        self.command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_sec").value)
        self.timer_period = float(self.get_parameter("timer_period_sec").value)
        self.diagnostic_log_period = float(
            self.get_parameter("diagnostic_log_period_sec").value
        )
        self.vesc_telemetry_timeout = float(
            self.get_parameter("vesc_telemetry_timeout_sec").value
        )
        self.vesc_low_voltage_limit = float(
            self.get_parameter("vesc_low_voltage_limit").value
        )
        self.vesc_low_voltage_stop = float(
            self.get_parameter("vesc_low_voltage_stop").value
        )
        self.maximum_steering_rate = float(
            self.get_parameter("maximum_steering_command_rate").value
        )
        self.steering_settle_tolerance = float(
            self.get_parameter("steering_settle_tolerance_command").value
        )
        self.transient_nav2_zero_hold_sec = float(
            self.get_parameter("transient_nav2_zero_hold_sec").value
        )
        self.obstacle_reverse_recovery_enabled = bool(
            self.get_parameter("obstacle_reverse_recovery_enabled").value
        )
        self.obstacle_reverse_settle_sec = float(
            self.get_parameter("obstacle_reverse_settle_sec").value
        )
        self.obstacle_reverse_duration_sec = float(
            self.get_parameter("obstacle_reverse_duration_sec").value
        )
        self.obstacle_reverse_cooldown_sec = float(
            self.get_parameter("obstacle_reverse_cooldown_sec").value
        )

        self.calibration = MotorCalibration(
            speed_gain_mps_per_command=float(
                self.get_parameter("speed_gain_mps_per_command").value
            ),
            minimum_moving_command=float(
                self.get_parameter("minimum_moving_command").value
            ),
            maximum_forward_command=float(
                self.get_parameter("maximum_forward_command").value
            ),
            maximum_reverse_command=float(
                self.get_parameter("maximum_reverse_command").value
            ),
            steering_commands=tuple(
                float(value)
                for value in self.get_parameter("steering_map_commands").value
            ),
            steering_curvatures=tuple(
                float(value)
                for value in self.get_parameter("steering_map_curvatures").value
            ),
        )
        self.footprint = Footprint(
            minimum_x=float(self.get_parameter("footprint_minimum_x").value),
            maximum_x=float(self.get_parameter("footprint_maximum_x").value),
            half_width=float(self.get_parameter("footprint_half_width").value),
        )
        self.collision_margin = float(
            self.get_parameter("collision_margin_m").value
        )
        self.reaction_time = float(
            self.get_parameter("reaction_time_sec").value
        )
        self.braking_deceleration = float(
            self.get_parameter("braking_deceleration_mps2").value
        )
        self.minimum_projection = float(
            self.get_parameter("minimum_projection_m").value
        )
        self.maximum_projection = float(
            self.get_parameter("maximum_projection_m").value
        )
        self.projection_step = float(
            self.get_parameter("projection_sample_step_m").value
        )
        self.minimum_scan_points = int(
            self.get_parameter("minimum_scan_points").value
        )
        self.laser_x = float(self.get_parameter("laser_x").value)
        self.laser_y = float(self.get_parameter("laser_y").value)
        self.laser_yaw = float(self.get_parameter("laser_yaw").value)

        direction_change_dwell_sec = float(
            self.get_parameter("direction_change_dwell_sec").value
        )
        if (
            self.obstacle_reverse_settle_sec < direction_change_dwell_sec
            or self.obstacle_reverse_duration_sec <= 0.0
            or self.obstacle_reverse_cooldown_sec < 0.0
        ):
            raise ValueError(
                "obstacle reverse timing must be positive and include direction dwell"
            )
        self.direction_guard = DirectionChangeGuard(direction_change_dwell_sec)
        self.transient_zero_hold = TransientZeroHold(
            self.transient_nav2_zero_hold_sec
        )
        self.latest_twist: Twist | None = None
        self.latest_twist_time: float | None = None
        self.latest_scan_points: list[tuple[float, float]] = []
        self.latest_scan_time: float | None = None
        self.authorized = False
        self.applied_steering = 0.0
        self.applied_speed = 0.0
        self.last_timer_time = time.monotonic()
        self.last_reason = "startup"
        self.next_diagnostic_log_at = 0.0
        self.mission_state = "UNKNOWN"
        self.mission_step = "-"
        self.mission_precise = True
        self.mission_reverse_crawl = False
        self.latest_vesc_time: float | None = None
        self.vesc_voltage = math.nan
        self.vesc_fault_code = 0
        self.obstacle_recovery_phase = "idle"
        self.obstacle_reverse_at = 0.0
        self.obstacle_reverse_until = 0.0
        self.obstacle_reverse_cooldown_until = 0.0

        self.motor_publisher = self.create_publisher(
            Float32MultiArray, self.motor_topic, 10
        )
        self.shadow_publisher = self.create_publisher(
            Float32MultiArray, self.shadow_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, "~/status", 10
        )
        self.obstacle_recovery_publisher = self.create_publisher(
            String,
            str(self.get_parameter("obstacle_recovery_request_topic").value),
            10,
        )
        self.debug_publisher = self.create_publisher(
            Float32MultiArray, "~/debug", 10
        )
        self.create_subscription(Twist, self.input_topic, self._on_twist, 10)
        self.create_subscription(
            Bool, self.authorization_topic, self._on_authorization, 10
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            String, self.mission_state_topic, self._on_mission_state, 20
        )
        self.create_subscription(
            XycarVescState,
            self.vesc_state_topic,
            self._on_vesc_state,
            qos_profile_sensor_data,
        )
        self.create_timer(self.timer_period, self._on_timer)
        self.get_logger().info(
            "주차 모터 명령기가 준비되었습니다: 실차출력=%s, 입력=%s, 모터=%s, "
            "주행명령=연속 ±%.1f"
            % (
                "켜짐" if self.drive_enabled else "꺼짐",
                self.input_topic,
                self.motor_topic,
                self.calibration.minimum_moving_command,
            )
        )
        self.get_logger().info(
            "모터 차단진단: 단계·미션상태·cmd_vel/scan 나이·LiDAR 점수·"
            "VESC 전압/fault를 %.1f초마다 출력합니다"
            % self.diagnostic_log_period
        )
        self.get_logger().info(
            "LiDAR 안전입력: 유효점 최소=%d, scan timeout=%.2fs, "
            "자체반사 제거영역 x=[%.2f,%.2f] y=±%.2f m, "
            "예상충돌 검사거리=%.2f~%.2f m"
            % (
                self.minimum_scan_points,
                self.scan_timeout,
                self.footprint.minimum_x,
                self.footprint.maximum_x,
                self.footprint.half_width,
                self.minimum_projection,
                self.maximum_projection,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_topic", "/cmd_vel_nav")
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter("shadow_topic", "/parking/xycar_motor_shadow")
        self.declare_parameter("authorization_topic", "/parking/drive_authorized")
        self.declare_parameter("scan_topic", "/slam/scan_filtered")
        self.declare_parameter("mission_state_topic", "/parking/mission_state")
        self.declare_parameter("vesc_state_topic", "/vehicle/vesc_state")
        self.declare_parameter("drive_enabled", False)
        self.declare_parameter("command_timeout_sec", 0.35)
        self.declare_parameter("scan_timeout_sec", 0.35)
        self.declare_parameter("timer_period_sec", 0.05)
        self.declare_parameter("diagnostic_log_period_sec", 2.0)
        self.declare_parameter("vesc_telemetry_timeout_sec", 0.35)
        self.declare_parameter("vesc_low_voltage_limit", 7.5)
        self.declare_parameter("vesc_low_voltage_stop", 6.0)
        self.declare_parameter("maximum_steering_command_rate", 160.0)
        self.declare_parameter("steering_settle_tolerance_command", 3.0)
        self.declare_parameter("transient_nav2_zero_hold_sec", 0.40)
        self.declare_parameter("direction_change_dwell_sec", 0.40)
        self.declare_parameter("obstacle_reverse_recovery_enabled", True)
        self.declare_parameter("obstacle_reverse_settle_sec", 0.40)
        self.declare_parameter("obstacle_reverse_duration_sec", 0.62)
        self.declare_parameter("obstacle_reverse_cooldown_sec", 0.80)
        self.declare_parameter(
            "obstacle_recovery_request_topic",
            "/parking/obstacle_recovery_request",
        )

        self.declare_parameter("speed_gain_mps_per_command", 0.080612)
        self.declare_parameter("minimum_moving_command", 4.0)
        self.declare_parameter("maximum_forward_command", 4.0)
        self.declare_parameter("maximum_reverse_command", 4.0)
        self.declare_parameter(
            "steering_map_commands",
            [-42.0, -40.0, -35.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 35.0, 40.0, 42.0],
        )
        self.declare_parameter(
            "steering_map_curvatures",
            [1.502435, 1.383494, 1.174860, 0.922781, 0.552809, 0.194230, 0.0, -0.556883, -0.959829, -1.369323, -1.601706, -1.853397, -1.939236],
        )

        # Front-wheel-center base frame, including the physical shell and tires.
        self.declare_parameter("footprint_minimum_x", -0.46)
        self.declare_parameter("footprint_maximum_x", 0.11)
        self.declare_parameter("footprint_half_width", 0.18)
        self.declare_parameter("collision_margin_m", 0.045)
        self.declare_parameter("reaction_time_sec", 0.20)
        self.declare_parameter("braking_deceleration_mps2", 1.50)
        self.declare_parameter("minimum_projection_m", 0.14)
        self.declare_parameter("maximum_projection_m", 0.80)
        self.declare_parameter("projection_sample_step_m", 0.025)
        self.declare_parameter("minimum_scan_points", 60)
        self.declare_parameter("laser_x", 0.065)
        self.declare_parameter("laser_y", 0.0)
        self.declare_parameter("laser_yaw", 0.0)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_twist(self, message: Twist) -> None:
        self.latest_twist = message
        self.latest_twist_time = self._now_sec()

    def _on_authorization(self, message: Bool) -> None:
        self.authorized = bool(message.data)
        if not self.authorized:
            self.direction_guard.reset()
            self.obstacle_recovery_phase = "idle"

    def _on_mission_state(self, message: String) -> None:
        fields = parse_status_line(message.data)
        self.mission_state = fields.get("state", "UNKNOWN")
        self.mission_step = fields.get("step", "-")
        self.mission_precise = fields.get("precise", "true") == "true"
        self.mission_reverse_crawl = (
            fields.get("reverse_crawl", "false") == "true"
        )

    def _on_vesc_state(self, message: XycarVescState) -> None:
        self.latest_vesc_time = self._now_sec()
        self.vesc_voltage = float(message.voltage_input)
        self.vesc_fault_code = int(message.fault_code)

    def _on_scan(self, message: LaserScan) -> None:
        points = scan_points_in_base(
            message.ranges,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            range_min=max(float(message.range_min), 0.10),
            range_max=min(float(message.range_max), 6.0),
            laser_x=self.laser_x,
            laser_y=self.laser_y,
            laser_yaw=self.laser_yaw,
        )
        # A near return inside the known body is a self reflection. Keeping it
        # would permanently stop a 360-degree scanner mounted over the chassis.
        self.latest_scan_points = [
            (x, y)
            for x, y in points
            if not (
                self.footprint.minimum_x <= x <= self.footprint.maximum_x
                and abs(y) <= self.footprint.half_width
            )
        ]
        self.latest_scan_time = self._now_sec()

    def _stop(self, reason: str) -> None:
        self.transient_zero_hold.reset()
        self.applied_steering = 0.0
        self.applied_speed = 0.0
        self._publish(0.0, 0.0, 0.0, reason, live=self.drive_enabled)

    def _start_obstacle_reverse_recovery(self, now_sec: float) -> bool:
        if (
            not self.obstacle_reverse_recovery_enabled
            or self.mission_state != "RUNNING"
            or now_sec < self.obstacle_reverse_cooldown_until
        ):
            return False
        self.obstacle_recovery_phase = "settle"
        self.obstacle_reverse_at = now_sec + self.obstacle_reverse_settle_sec
        self.obstacle_reverse_until = (
            self.obstacle_reverse_at + self.obstacle_reverse_duration_sec
        )
        self.obstacle_reverse_cooldown_until = (
            self.obstacle_reverse_until + self.obstacle_reverse_cooldown_sec
        )
        self.transient_zero_hold.reset()
        self.direction_guard.reset()
        self.obstacle_recovery_publisher.publish(
            String(data=f"step={self.mission_step}")
        )
        return True

    def _run_obstacle_reverse_recovery(
        self,
        now_sec: float,
        dt_sec: float,
    ) -> bool:
        if self.obstacle_recovery_phase == "idle":
            return False
        self.applied_steering = slew(
            self.applied_steering,
            0.0,
            self.maximum_steering_rate,
            dt_sec,
        )
        if now_sec < self.obstacle_reverse_at:
            self.applied_speed = 0.0
            self._publish(
                self.applied_steering,
                0.0,
                0.0,
                "obstacle_reverse_settle",
                live=self.drive_enabled,
            )
            return True
        if now_sec < self.obstacle_reverse_until:
            self.obstacle_recovery_phase = "reverse"
            self.applied_speed = -self.calibration.minimum_moving_command
            self._publish(
                self.applied_steering,
                self.applied_speed,
                0.0,
                "obstacle_reverse_active",
                live=self.drive_enabled,
            )
            return True
        self.obstacle_recovery_phase = "idle"
        self.direction_guard.reset()
        self.applied_speed = 0.0
        self._publish(
            self.applied_steering,
            0.0,
            0.0,
            "obstacle_reverse_complete",
            live=self.drive_enabled,
        )
        return True

    @staticmethod
    def _age_text(now_sec: float, stamp_sec: float | None) -> str:
        if stamp_sec is None:
            return "없음"
        return "%.3fs" % max(0.0, now_sec - stamp_sec)

    def _vesc_text(self, now_sec: float) -> str:
        if self.latest_vesc_time is None:
            return "없음"
        age = max(0.0, now_sec - self.latest_vesc_time)
        if age > self.vesc_telemetry_timeout:
            return "stale(%.3fs)" % age
        if self.vesc_fault_code != 0:
            return "%.2fV fault=%d" % (self.vesc_voltage, self.vesc_fault_code)
        if self.vesc_voltage <= self.vesc_low_voltage_stop:
            state = "정지전압"
        elif self.vesc_voltage < self.vesc_low_voltage_limit:
            state = "저전압출력제한"
        else:
            state = "정상"
        return "%.2fV(%s)" % (self.vesc_voltage, state)

    def _publish(
        self,
        steering: float,
        speed: float,
        curvature: float,
        reason: str,
        *,
        live: bool,
    ) -> None:
        command = Float32MultiArray(data=[float(steering), float(speed)])
        self.shadow_publisher.publish(command)
        if live:
            self.motor_publisher.publish(command)
        self.debug_publisher.publish(
            Float32MultiArray(
                data=[
                    float(steering),
                    float(speed),
                    float(curvature),
                    1.0 if self.authorized else 0.0,
                    float(len(self.latest_scan_points)),
                ]
            )
        )
        now_sec = self._now_sec()
        reason_changed = reason != self.last_reason
        if reason_changed:
            self.status_publisher.publish(String(data=reason))
        moving = abs(float(speed)) > 1.0e-6
        should_log = reason_changed or (
            reason != "ok" and now_sec >= self.next_diagnostic_log_at
        )
        if should_log:
            label = "모터 출력" if moving else "모터 정지"
            message = (
                "[%s] 단계=%s 미션=%s 이유=%s 명령=[조향 %.1f, 속도 %.1f] "
                "주행권한=%s cmd_vel나이=%s scan나이=%s LiDAR점=%d VESC=%s"
                % (
                    label,
                    self.mission_step,
                    self.mission_state,
                    GATE_REASON_KO.get(reason, reason),
                    steering,
                    speed,
                    "있음" if self.authorized else "없음",
                    self._age_text(now_sec, self.latest_twist_time),
                    self._age_text(now_sec, self.latest_scan_time),
                    len(self.latest_scan_points),
                    self._vesc_text(now_sec),
                )
            )
            if moving:
                self.get_logger().info(message)
            else:
                self.get_logger().warning(message)
            self.next_diagnostic_log_at = now_sec + self.diagnostic_log_period
        if reason_changed:
            self.last_reason = reason

    def _on_timer(self) -> None:
        now_sec = self._now_sec()
        monotonic_now = time.monotonic()
        dt_sec = max(0.0, min(0.25, monotonic_now - self.last_timer_time))
        self.last_timer_time = monotonic_now

        if not self.authorized:
            self._stop("not_authorized")
            return
        if self.drive_enabled:
            if self.latest_vesc_time is None:
                self._stop("no_vesc_telemetry")
                return
            if now_sec - self.latest_vesc_time > self.vesc_telemetry_timeout:
                self._stop("stale_vesc_telemetry")
                return
            if self.vesc_fault_code != 0:
                self._stop("vesc_fault")
                return
            if self.vesc_voltage <= self.vesc_low_voltage_stop:
                self._stop("vesc_low_voltage_stop")
                return
        # Nav2 is cancelled as soon as an obstacle recovery starts, so its
        # cmd_vel may disappear. Keep only the bounded recovery maneuver alive
        # while authorization, VESC and fresh scan health remain valid.
        if self.obstacle_recovery_phase != "idle":
            if self.latest_scan_time is None:
                self._stop("no_scan")
                return
            if now_sec - self.latest_scan_time > self.scan_timeout:
                self._stop("stale_scan")
                return
            if len(self.latest_scan_points) < self.minimum_scan_points:
                self._stop("insufficient_scan")
                return
            self._run_obstacle_reverse_recovery(now_sec, dt_sec)
            return
        if self.latest_twist is None or self.latest_twist_time is None:
            self._stop("no_cmd_vel")
            return
        if now_sec - self.latest_twist_time > self.command_timeout:
            self._stop("stale_cmd_vel")
            return
        if self.latest_scan_time is None:
            self._stop("no_scan")
            return
        if now_sec - self.latest_scan_time > self.scan_timeout:
            self._stop("stale_scan")
            return
        if len(self.latest_scan_points) < self.minimum_scan_points:
            self._stop("insufficient_scan")
            return

        desired = twist_to_motor_command(
            self.latest_twist.linear.x,
            self.latest_twist.angular.z,
            self.calibration,
        )
        reverse_align_rotate_crawl = False
        reverse_align_zero_crawl = False
        if (
            desired.reason == "rotate_in_place_rejected"
            and self.mission_state == "RUNNING"
            and self.mission_reverse_crawl
        ):
            desired = rotate_request_to_reverse_crawl(
                self.latest_twist.angular.z,
                self.calibration,
            )
            reverse_align_rotate_crawl = desired.reason == "ok"
        elif (
            desired.reason == "stopped"
            and self.mission_state == "RUNNING"
            and self.mission_reverse_crawl
        ):
            desired = stopped_request_to_reverse_crawl(
                self.applied_steering,
                self.calibration,
            )
            reverse_align_zero_crawl = desired.reason == "ok"
        desired, holding_transient_zero = self.transient_zero_hold.filter(
            desired,
            now_sec,
            eligible=(
                self.mission_state == "RUNNING" and not self.mission_precise
            ),
        )
        if desired.reason != "ok":
            self._stop(desired.reason)
            return

        collision = swept_footprint_collision(
            self.latest_scan_points,
            # The real chassis runs every non-zero request at command 4, so
            # stopping distance must use that executable speed.
            speed_mps=(
                desired.speed_command
                * self.calibration.speed_gain_mps_per_command
            ),
            curvature=desired.curvature,
            footprint=self.footprint,
            margin_m=self.collision_margin,
            reaction_time_sec=self.reaction_time,
            braking_deceleration_mps2=self.braking_deceleration,
            minimum_projection_m=self.minimum_projection,
            maximum_projection_m=self.maximum_projection,
            sample_step_m=self.projection_step,
        )
        if collision.collision:
            if (
                desired.speed_command > 0.0
                and self._start_obstacle_reverse_recovery(now_sec)
            ):
                self._run_obstacle_reverse_recovery(now_sec, dt_sec)
                return
            # Stay stopped but allow the servo to follow a newly safe steering
            # request. Centering it here creates repeatable stop/retry drift.
            self.transient_zero_hold.reset()
            self.applied_steering = slew(
                self.applied_steering,
                desired.steering_command,
                self.maximum_steering_rate,
                dt_sec,
            )
            self.applied_speed = 0.0
            self._publish(
                self.applied_steering,
                0.0,
                desired.curvature,
                "lidar_swept_collision",
                live=self.drive_enabled,
            )
            return

        guarded_speed, guard_reason = self.direction_guard.filter(
            desired.speed_command,
            now_sec,
        )
        if guard_reason == "direction_change_dwell":
            # Pre-position steering while the car is physically stopped.
            self.transient_zero_hold.reset()
            self.applied_steering = slew(
                self.applied_steering,
                desired.steering_command,
                self.maximum_steering_rate,
                dt_sec,
            )
            self.applied_speed = 0.0
            self._publish(
                self.applied_steering,
                0.0,
                desired.curvature,
                guard_reason,
                live=self.drive_enabled,
            )
            return

        next_steering = slew(
            self.applied_steering,
            desired.steering_command,
            self.maximum_steering_rate,
            dt_sec,
        )
        if should_hold_for_steering_settle(
            applied_speed_command=self.applied_speed,
            desired_steering_command=desired.steering_command,
            next_steering_command=next_steering,
            tolerance_command=self.steering_settle_tolerance,
        ):
            # Pre-align before initial motion, after a direction dwell, or
            # after a safety stop. During ordinary same-direction driving the
            # servo slews while command 4 remains continuous.
            self.applied_steering = next_steering
            self.applied_speed = 0.0
            self._publish(
                self.applied_steering,
                0.0,
                desired.curvature,
                "steering_settle",
                live=self.drive_enabled,
            )
            return
        self.applied_steering = next_steering
        # Do not ramp through commands below 4: the real drivetrain cannot
        # move there. The VESC driver smooths the physical acceleration while
        # this adapter continuously publishes exactly +4 or -4.
        self.applied_speed = guarded_speed
        self._publish(
            self.applied_steering,
            self.applied_speed,
            desired.curvature,
            (
                "transient_nav2_zero_hold"
                if holding_transient_zero
                else "reverse_align_zero_crawl"
                if reverse_align_zero_crawl
                else "reverse_align_rotate_crawl"
                if reverse_align_rotate_crawl
                else "ok"
            ),
            live=self.drive_enabled,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._stop("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
