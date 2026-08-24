#!/usr/bin/env python3
"""Terminal SPACE toggle for the parking mission services."""

from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .parking_keyboard_core import (
    ParkingKeyboardIntent,
    explain_status,
    parse_status_line,
)


class ParkingKeyboardControl(Node):
    def __init__(self) -> None:
        super().__init__("parking_keyboard_control")
        self.intent = ParkingKeyboardIntent()
        self.original_terminal_settings = None
        self.stdin_is_tty = sys.stdin.isatty()
        self.sequence = 0
        self.start_phase = "IDLE"
        self.abort_pending = False
        self.abort_in_flight = False
        self.state_fields: dict[str, str] = {}
        self.motor_reason = "startup"
        self.last_display = ""

        self.reset_client = self.create_client(
            Trigger, "/parking_mission_manager/reset"
        )
        self.start_client = self.create_client(
            Trigger, "/parking_mission_manager/start"
        )
        self.abort_client = self.create_client(
            Trigger, "/parking_mission_manager/abort"
        )
        self.create_subscription(
            String, "/parking/mission_state", self._on_mission_state, 20
        )
        self.create_subscription(
            String,
            "/parking_cmd_vel_adapter/status",
            self._on_motor_status,
            20,
        )
        self.create_timer(0.05, self._on_timer)
        self._configure_terminal()
        self.get_logger().info(
            "SPACE: 주차 시작/정지 | Q 또는 ESC: 안전 정지 후 종료"
        )

    def _configure_terminal(self) -> None:
        if not self.stdin_is_tty:
            self.get_logger().error(
                "키보드 입력을 받을 수 없습니다. ros2 run 명령을 별도 터미널에서 직접 실행하세요."
            )
            return
        self.original_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def _read_key(self) -> str | None:
        if not self.stdin_is_tty:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        return sys.stdin.read(1)

    def _on_mission_state(self, message: String) -> None:
        self.state_fields = parse_status_line(message.data)

    def _on_motor_status(self, message: String) -> None:
        self.motor_reason = str(message.data)

    def _handle_key(self, key: str) -> None:
        action = self.intent.handle_key(key)
        if action == "START":
            self.sequence += 1
            if self.abort_pending or self.abort_in_flight:
                self.start_phase = "WAIT_ABORT"
                self.get_logger().warning(
                    "[RUN 예약] 이전 STOP 완료 후 위치추정을 초기화하고 시작합니다"
                )
            else:
                self.start_phase = "NEED_RESET"
                self.get_logger().warning(
                    "[RUN 요청] 위치추정을 새로 갱신한 뒤 자동으로 미션을 시작합니다"
                )
        elif action in {"STOP", "QUIT"}:
            self.sequence += 1
            self.start_phase = "IDLE"
            self.abort_pending = True
            self.get_logger().warning(
                "[STOP 요청] 미션 취소와 모터 주행 권한 해제를 요청했습니다"
            )

    def _advance_services(self) -> None:
        if self.abort_pending:
            if not self.abort_client.service_is_ready():
                return
            self.abort_pending = False
            self.abort_in_flight = True
            future = self.abort_client.call_async(Trigger.Request())
            future.add_done_callback(self._on_abort_done)
            return

        token = self.sequence
        if self.start_phase == "NEED_RESET":
            if not self.reset_client.service_is_ready():
                return
            self.start_phase = "WAIT_RESET"
            future = self.reset_client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda completed, expected=token: self._on_reset_done(
                    completed, expected
                )
            )
        elif self.start_phase == "NEED_START":
            if not self.start_client.service_is_ready():
                return
            self.start_phase = "WAIT_START"
            future = self.start_client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda completed, expected=token: self._on_start_done(
                    completed, expected
                )
            )

    def _on_reset_done(self, future, token: int) -> None:
        if token != self.sequence or not self.intent.desired_running:
            return
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future reports transport errors
            self.start_phase = "IDLE"
            self.intent.desired_running = False
            self.get_logger().error(f"[시작 실패] reset 서비스 오류: {error}")
            return
        if not response.success:
            self.start_phase = "IDLE"
            self.intent.desired_running = False
            self.get_logger().error(f"[시작 실패] {response.message}")
            return
        self.get_logger().info(f"[초기화 완료] {response.message}")
        self.start_phase = "NEED_START"

    def _on_start_done(self, future, token: int) -> None:
        if token != self.sequence or not self.intent.desired_running:
            return
        self.start_phase = "IDLE"
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.intent.desired_running = False
            self.get_logger().error(f"[시작 실패] start 서비스 오류: {error}")
            return
        log = self.get_logger().info if response.success else self.get_logger().warning
        log(f"[START 처리] {response.message}")

    def _on_abort_done(self, future) -> None:
        self.abort_in_flight = False
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.start_phase = "IDLE"
            self.intent.desired_running = False
            self.get_logger().error(f"[정지 실패] abort 서비스 오류: {error}")
            return
        if response.success:
            self.get_logger().warning(f"[STOP 완료] {response.message}")
            if self.intent.desired_running:
                self.start_phase = "NEED_RESET"
                self.get_logger().warning(
                    "[RUN 재개] STOP 완료를 확인해 초기화부터 진행합니다"
                )
        else:
            self.start_phase = "IDLE"
            self.intent.desired_running = False
            self.get_logger().error(f"[정지 실패] {response.message}")

    def _display_status(self) -> None:
        state_text, reason = explain_status(
            self.state_fields,
            self.motor_reason,
            desired_running=self.intent.desired_running,
        )
        mode = "RUN 요청" if self.intent.desired_running else "STOP"
        step = self.state_fields.get("step", "-")
        state = self.state_fields.get("state", "")
        localization = (
            "-"
            if state in {"ABORTED", "COMPLETED"}
            else self.state_fields.get("localization", "-")
        )
        raw_index = self.state_fields.get("index", "-")
        try:
            zero_based, total = raw_index.split("/", 1)
            stage_index = "%d/%s" % (int(zero_based) + 1, total)
        except (TypeError, ValueError):
            stage_index = raw_index
        display = (
            f"[{mode}] 상태={state_text} | 단계={stage_index} {step} | "
            f"위치추정={localization} | 이유={reason}"
        )
        if display != self.last_display:
            self.get_logger().info(display)
            self.last_display = display

    def _on_timer(self) -> None:
        while True:
            key = self._read_key()
            if key is None:
                break
            self._handle_key(key)
        self._advance_services()
        self._display_status()
        if self.intent.quit_requested:
            raise KeyboardInterrupt

    def restore_terminal(self) -> None:
        if self.original_terminal_settings is not None:
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, self.original_terminal_settings
            )
            self.original_terminal_settings = None

    def stop_and_wait(self) -> None:
        self.intent.desired_running = False
        self.sequence += 1
        if self.abort_client.wait_for_service(timeout_sec=0.2):
            future = self.abort_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ParkingKeyboardControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_and_wait()
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
