#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""국민대 예선용 차선주행 중심 통합 ROS2 노드입니다.

기본은 shadow 모드라 /xycar_motor를 절대 publish하지 않고,
OpenCV 디버그 창으로 FSM/인터럽트/차선·라바콘 출력을 확인합니다.
실주행은 --ros-args -p drive_mode:=auto 로 전환합니다.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Float32, Float32MultiArray

from . import config
from .cone_core import ConeCore
from .debug_tools import DebugOverlay, format_transition_log
from .fsm import FsmEvents, MissionFSM, MissionState
from .interrupts import ControlCommand, InterruptManager
from .lane_core import LaneCore
from .sign_core import SignCore
from .stopline_core import StopLineCore
from .police_core import PoliceCore
from .shortcut_core import ShortcutCore
from .world_model import WorldModel


class TrackDriveNode(Node):
    """sub/pub, control loop, FSM, shadow/auto 발행을 묶은 통합 노드입니다."""

    def __init__(self) -> None:
        """ROS 통신과 core 객체를 초기화합니다."""
        super().__init__("track_drive_sve")

        self.declare_parameter("drive_mode", config.DEFAULT_DRIVE_MODE)
        self.bridge = CvBridge()

        self.image_bgr: Optional[np.ndarray] = None
        self.scan_msg: Optional[LaserScan] = None
        self.imu_msg: Optional[Imu] = None
        self.current_yaw_deg: Optional[float] = None

        self.world = WorldModel()
        self.fsm = MissionFSM()
        self.interrupts = InterruptManager()
        self.lane_core = LaneCore(logger=lambda msg: self.get_logger().warn(msg))
        self.sign_core = SignCore()        
        self.cone_core = ConeCore()
        self.shortcut_core = ShortcutCore(logger=lambda m: self.get_logger().info(m))
        self.police_core = PoliceCore()
        self.stopline_core = StopLineCore()
        self.debug_overlay = DebugOverlay()

        self.last_summary_time = 0.0
        self.last_base_cmd = ControlCommand()
        self.last_final_cmd = ControlCommand()
        self.last_lane_cmd: Tuple[float, float] = (0.0, 0.0)
        self.cone_exit_time: float = -1.0   # 콘->차선 전환 시각 (직진 하드코딩용)
        self.post_ped_winding_active: bool = False
        self._last_loop_dt: float = 0.0     # 직전 control_loop 처리시간(진단 로그용)
        self._last_diag_time: float = 0.0   # [DIAG] 진단 로그 rate-limit용
        
        self.create_subscription(Image, config.CAMERA_TOPIC, self.image_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, config.SCAN_TOPIC, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, config.IMU_TOPIC, self.imu_callback, qos_profile_sensor_data)
        # 회피 노드의 offset/speed_limit 구독 → lane_core에 전달
        self.create_subscription(Float32, "/obstacle_avoidance/path_offset", self.avoid_offset_callback, 10)
        self.create_subscription(Float32, "/obstacle_avoidance/speed_limit", self.avoid_speed_callback, 10)

        self.motor_pub = self.create_publisher(Float32MultiArray, config.MOTOR_TOPIC, 10)
        self.lane_debug_pub = self.create_publisher(Image, config.LANE_DEBUG_TOPIC, 10)
        self.timer = self.create_timer(config.CONTROL_PERIOD_SEC, self.control_loop)

        self.fsm.reset(self._now_sec())
        self.get_logger().info("track_drive_sve Gazebo 통합 노드 시작")
        self.get_logger().info("기본 drive_mode=shadow: /xycar_motor publish 없음")
        self.get_logger().info("auto 실행: ros2 run track_drive_sve track_drive_sve --ros-args -p drive_mode:=auto")

    # =====================================================================
    # ROS callback
    # =====================================================================
    def image_callback(self, msg: Image) -> None:
        """카메라 이미지를 OpenCV BGR로 변환하고 최신 프레임을 보관합니다."""
        now = self._now_sec()
        self.world.update_image_time(now)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_bgr = cv2.resize(
                frame,
                (config.IMAGE_WIDTH, config.IMAGE_HEIGHT),
                interpolation=cv2.INTER_LINEAR,
            )
        except Exception as exc:
            self.get_logger().warn(f"카메라 변환 실패: {exc}")

    def scan_callback(self, msg: LaserScan) -> None:
        """최신 LaserScan을 보관합니다."""
        self.world.update_scan_time(self._now_sec())
        self.scan_msg = msg

    def imu_callback(self, msg: Imu) -> None:
        """최신 IMU와 yaw 값을 보관합니다."""
        self.world.update_imu_time(self._now_sec())
        self.imu_msg = msg
        self.current_yaw_deg = self._yaw_from_imu(msg)

    def avoid_offset_callback(self, msg: Float32) -> None:
        """회피 노드의 path_offset을 lane_core에 전달합니다."""
        self.lane_core.set_external_command(path_offset=float(msg.data))

    def avoid_speed_callback(self, msg: Float32) -> None:
        """회피 노드의 speed_limit을 lane_core에 전달합니다."""
        self.lane_core.set_external_command(speed_limit=float(msg.data))


    # =====================================================================
    # 메인 제어 루프
    # =====================================================================
    def control_loop(self) -> None:
        """한 주기마다 인지 core, FSM, 인터럽트, publish/debug를 순서대로 실행합니다."""
        t0 = self._now_sec()  
        now = self._now_sec()
        drive_mode = self._drive_mode()

        # 차선 core는 기본 주행이자 CONE 종료 판단용이므로 매 프레임 계산합니다.
        lane_angle, lane_speed = self.lane_core.compute(self.image_bgr)
        self.last_lane_cmd = (lane_angle, lane_speed)
        self._publish_lane_debug_if_possible()

        # 라바콘 core는 CONE 상태에서만 제어에 사용하지만, 상태 전이 판단을 위해 먼저 계산합니다.
        cone_angle, cone_speed = (0.0, 0.0)
        if self.fsm.state == MissionState.CONE:
            cone_angle, cone_speed = self.cone_core.compute(self.scan_msg)

        self._update_finish_line_if_needed(now)
        police_blocking = self.police_core.detect(self.scan_msg)
        stopline_now = self.stopline_core.detect(self.image_bgr)
        decision_signals = self.interrupts.decision_signals_stub(self.image_bgr, self.scan_msg)

        events = self._build_fsm_events(decision_signals)
        before_state = self.fsm.state
        transition = self.fsm.update(events, self.world, now)
        # === 경찰차 인식 검증용 로그 (임시) ===
        if int(now * 2) != getattr(self, "_last_police_log", -1):
            self._last_police_log = int(now * 2)
            sign_now = self.sign_core.detect(self.image_bgr)
            self.get_logger().info(
                f"[meas] sign={sign_now} g={self.sign_core.last_green} r={self.sign_core.last_red} police_cnt={self.police_core.last_count} "
                f"police_det={police_blocking} police_dist={self.police_core.last_min_dist:.2f} "
                f"stopline_rows={self.stopline_core.last_line_rows} stopline_det={stopline_now}"
            )
        if transition is not None:
            self.get_logger().info(
                format_transition_log(
                    transition.before.value,
                    transition.after.value,
                    transition.reason,
                    transition.lap,
                    transition.route,
                )
            )
            if transition.after == MissionState.TURN_LEFT:
                self.fsm.on_entered_turn_left_with_yaw(self.current_yaw_deg)
                self.shortcut_core.reset()
            if transition.before == MissionState.CONE and transition.after == MissionState.LANE:
                # 콘 직후 직진 하드코딩 시작 시각 기록
                self.cone_exit_time = now

        # WAIT_START에서 곧바로 CONE으로 넘어온 첫 프레임은 여기서 라바콘 명령을 계산합니다.
        if self.fsm.state == MissionState.CONE and before_state != MissionState.CONE:
            cone_angle, cone_speed = self.cone_core.compute(self.scan_msg)

        base_cmd = self._select_base_command(lane_angle, lane_speed, cone_angle, cone_speed, now, police_blocking)
        arbitration = self.interrupts.arbitrate(base_cmd, self.fsm.state, self.world, self.scan_msg, now)
        final_cmd = arbitration.final

        pedestrian_active = any(
            action.name == "PEDESTRIAN"
            for action in arbitration.active_actions
        )
        obstacle_avoid_active = (
            self.lane_core.external_command_active()
            and self.lane_core.external_speed_limit > 0.0
        )

        if self.fsm.state != MissionState.LANE:
            self.post_ped_winding_active = False
        elif pedestrian_active and not self.post_ped_winding_active:
            self.post_ped_winding_active = True
            self.get_logger().info(
                "보행자 감지 후 S자 저속 프로파일 시작: "
                f"cap={config.LANE_POST_PED_WINDING_SPEED_CAP:.1f}"
            )

        if obstacle_avoid_active and self.post_ped_winding_active:
            self.post_ped_winding_active = False
            self.get_logger().info("차량 회피 시작: 보행자 이후 S자 저속 프로파일 종료")

        if self.post_ped_winding_active:
            final_cmd = ControlCommand(
                final_cmd.angle,
                min(final_cmd.speed, config.LANE_POST_PED_WINDING_SPEED_CAP),
                f"{final_cmd.source}+POST_PED_WINDING",
            )
            arbitration.final = final_cmd
            arbitration.debug["final_speed"] = float(final_cmd.speed)
        arbitration.debug["post_ped_winding_active"] = self.post_ped_winding_active

        # 인터럽트가 속도를 0/낮은 cap으로 막았을 때 lane_core 내부 속도 램프도 최종값에
        # 맞춰 낮춘다. 보행자 정지 해제 직후 내부 속도가 이미 커져 곡선에서 튀는 것을 막는다.
        self.lane_core.sync_with_final_command(final_cmd.angle, final_cmd.speed)

        self.last_base_cmd = base_cmd
        self.last_final_cmd = final_cmd

        if drive_mode == "auto":
            self._publish_motor(final_cmd.angle, final_cmd.speed)

        health = self.world.sensor_health(now)

        # === [DIAG] 문제3 진단용 (동작 변경 없음, 로그 전용) ===
        # 곡선에서 빠른지 / 센서 stale(P1) / 루프부하의 상관을 한 줄로 본다.
        # 확인 후 이 블록은 제거하면 된다.
        if now - self._last_diag_time >= 0.30 and self.fsm.state == MissionState.LANE:
            self._last_diag_time = now
            ld = self.lane_core.debug_summary()
            lane_a, lane_s = self.last_lane_cmd
            p1_on = "센서 끊김" in arbitration.interrupt_text
            self.get_logger().info(
                "[DIAG] curv=%.4f angle=%.1f lane_spd=%.1f base_spd=%.1f final_spd=%.1f "
                "P1=%s img_age=%.2f scan_age=%.2f loop_dt=%.0fms src=%s" % (
                    float(ld.get("filtered_curvature", 0.0)),
                    float(lane_a), float(lane_s),
                    float(base_cmd.speed), float(final_cmd.speed),
                    "Y" if p1_on else "n",
                    float(health.image_age), float(health.scan_age),
                    self._last_loop_dt * 1000.0, base_cmd.source,
                )
            )

        self._maybe_summary_log(now, drive_mode, base_cmd, final_cmd, arbitration.interrupt_text)

        elapsed = now - self.fsm.state_enter_time
        key = self.debug_overlay.show(
            self.image_bgr,
            self.fsm.state,
            self.fsm.guidance_text(self.world),
            arbitration.interrupt_text,
            self.last_lane_cmd,
            (base_cmd.angle, base_cmd.speed),
            (final_cmd.angle, final_cmd.speed),
            base_cmd.source,
            self.world,
            health,
            self.lane_core.debug_summary(),
            self.cone_core.debug_summary(),
            arbitration.debug,
            elapsed,
            drive_mode,
        )

        # 이번 프레임에서 사용한 수동 이벤트를 지우고, 방금 눌린 키는 다음 프레임에 사용합니다.
        self.world.manual.clear()
        key_msg = self.world.apply_debug_key(key)
        if key_msg:
            self.get_logger().info(f"shadow 검증 키 입력: {key_msg}")
        
        dt = self._now_sec() - t0
        self._last_loop_dt = dt
        if dt > 0.05:
            self.get_logger().warn(f"control_loop 느림: {dt*1000:.0f}ms (목표 50ms)")
    
    # =====================================================================
    # FSM 이벤트 / 기본 명령 선택
    # =====================================================================
    def _build_fsm_events(self, decision_signals) -> FsmEvents:
        """WorldModel, core debug, interrupt stub 결과를 FSM 이벤트로 묶습니다."""
        lane_stable = self.lane_core.is_lane_stable()
        cone_missing = self.cone_core.cone_missing_now()
        decision_anchor = self.stopline_core.detect(self.image_bgr)
        sign = self.sign_core.detect(self.image_bgr)
        start_allowed = bool(config.GAZEBO_START_IN_LANE) or (sign == "green")
        police = self.police_core.detect(self.scan_msg)

        return FsmEvents(
            start_allowed=start_allowed,
            cone_missing=cone_missing,
            lane_stable=lane_stable,
            decision_anchor=decision_anchor,
            finish_line_crossed=self.world.finish_line_crossed,
            shortcut_done=getattr(self, "_shortcut_done", False),
            arrow_green=(sign == "left"),
            straight_green=(sign == "green"),
            police_blocked=police,
            current_yaw_deg=self.current_yaw_deg,
            manual_decision=self.world.manual.force_decision,
            manual_cone_done=self.world.manual.force_cone_done,
            manual_left=self.world.manual.choose_left,
            manual_straight=self.world.manual.choose_straight,
            manual_turn_done=self.world.manual.finish_turn_left,
        )

    def _select_base_command(
        self,
        lane_angle: float,
        lane_speed: float,
        cone_angle: float,
        cone_speed: float,
        now_sec: float,
        police_blocking: bool = False,
    ) -> ControlCommand:
        """현재 FSM 상태에 맞는 기본 주행 명령을 선택합니다."""
        state = self.fsm.state
        if state == MissionState.WAIT_START:
            return ControlCommand(0.0, 0.0, "WAIT_START_STOP")
        if state == MissionState.CONE:
            return ControlCommand(cone_angle, cone_speed, "CONE_CORE")
        if state == MissionState.LANE:
            # 콘 직후에는 안정 검출된 lane_core 조향을 유지하고 속도만 잠깐 제한한다.
            if self.cone_exit_time >= 0.0:
                dt = now_sec - self.cone_exit_time
                if dt < config.CONE_EXIT_DELAY_SEC:
                    return ControlCommand(
                        lane_angle,
                        min(lane_speed, config.CONE_EXIT_STRAIGHT_SPEED),
                        "CONE_EXIT_LANE",
                    )
                self.cone_exit_time = -1.0
            return ControlCommand(lane_angle, lane_speed, "LANE_CORE")
        if state == MissionState.DECISION:
            # 경찰이 없어 좌회전 화살표를 기다리며 정지하는 경우, DECISION 진입 직후
            # 짧게 후진해 정지선과 거리를 벌린다(좌회전 각도 확보). 경찰 차단 시(직진해야 함)
            # 또는 lap0(즉시 직진)에는 후진하지 않는다.
            elapsed_decision = now_sec - self.fsm.state_enter_time
            if (
                not police_blocking
                and self.world.lap > 0
                and elapsed_decision < config.DECISION_REVERSE_SEC
            ):
                return ControlCommand(
                    config.DECISION_REVERSE_ANGLE,
                    config.DECISION_REVERSE_SPEED,
                    "DECISION_REVERSE",
                )
            return ControlCommand(0.0, 0.0, "DECISION_STOP")
        if state == MissionState.TURN_LEFT:
            angle, speed, done = self.shortcut_core.compute(self.image_bgr, now_sec)
            self._shortcut_done = done
            return ControlCommand(angle, speed, "SHORTCUT")
        return ControlCommand(0.0, 0.0, "FINISH_STOP")

    def _update_finish_line_if_needed(self, now_sec: float) -> None:
        """LANE 상태에서만 도착선 카운트를 갱신합니다.
        회피 중(차량 회피로 offset 활성)에는 차량의 흑백 대비를 도착선으로 오인하지
        않도록 도착선 검출을 건너뜁니다. 회피 구간은 도착선과 멀어 안전합니다."""
        avoiding = (
            self.lane_core.external_command_active()
            and abs(self.lane_core.external_path_offset) >= 1.0
        )
        if avoiding:
            self.world.finish_line_crossed = False
            return
        if self.fsm.state == MissionState.LANE and self.world.cone_done:
            crossed = self.world.update_finish_line(self.image_bgr, now_sec)
            if crossed:
                self.get_logger().info(
                    "\n================ 랩 카운트 ================\n"
                    f"  흑백 도착선 통과 → lap={self.world.lap}/{config.TOTAL_LAPS}\n"
                    f"  route는 새 lap 기본값 MAIN으로 초기화\n"
                    "=========================================="
                )
        else:
            self.world.finish_line_crossed = False

    # =====================================================================
    # publish / 로그 / 유틸
    # =====================================================================
    def _publish_motor(self, angle: float, speed: float) -> None:
        """auto 모드에서만 /xycar_motor 명령을 발행합니다."""
        msg = Float32MultiArray()
        msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(msg)

    def _publish_lane_debug_if_possible(self) -> None:
        """차선 BirdView 디버그 이미지를 ROS 토픽으로도 발행합니다."""
        if self.lane_core.last_debug_image is None:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(self.lane_core.last_debug_image, "bgr8")
            self.lane_debug_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"차선 디버그 이미지 발행 실패: {exc}")

    def _maybe_summary_log(
        self,
        now_sec: float,
        drive_mode: str,
        base_cmd: ControlCommand,
        final_cmd: ControlCommand,
        interrupt_text: str,
    ) -> None:
        """평소에는 1~2Hz 수준으로만 조용히 요약 로그를 찍습니다."""
        if now_sec - self.last_summary_time < config.SUMMARY_LOG_INTERVAL_SEC:
            return
        self.last_summary_time = now_sec
        self.get_logger().info(
            f"요약: mode={drive_mode}, state={self.fsm.state.value}, lap={self.world.lap}, "
            f"route={self.world.route_text()}, base={base_cmd.source}({base_cmd.angle:.1f},{base_cmd.speed:.1f}), "
            f"final=({final_cmd.angle:.1f},{final_cmd.speed:.1f}), {interrupt_text}"    
        )

    def _drive_mode(self) -> str:
        """ROS 파라미터 drive_mode를 읽어 shadow/auto 중 하나로 정규화합니다."""
        value = str(self.get_parameter("drive_mode").value).strip().lower()
        if value not in ("shadow", "auto"):
            self.get_logger().warn(f"알 수 없는 drive_mode={value}, shadow로 처리합니다.")
            return "shadow"
        return value

    def _now_sec(self) -> float:
        """ROS clock 현재 시간을 초 단위 float로 반환합니다."""
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _yaw_from_imu(self, msg: Imu) -> float:
        """IMU quaternion을 yaw degree로 변환합니다."""
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def destroy_node(self) -> bool:
        """종료 시 auto 모드에서만 정지 명령을 내고 창을 닫습니다."""
        try:
            if self._drive_mode() == "auto":
                self._publish_motor(0.0, 0.0)
            cv2.destroyAllWindows()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    """ROS2 엔트리 포인트입니다."""
    rclpy.init(args=args)
    node = TrackDriveNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        node.get_logger().info("사용자 종료 요청")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
