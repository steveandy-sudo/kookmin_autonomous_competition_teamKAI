"""Live, motor-disconnected telemetry for integrated cone entry."""

from __future__ import annotations

import math
import re
import sys
import time

from geometry_msgs.msg import PoseArray
from my_rule_msgs.msg import ObjectDetectionArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from .cone_entry_monitor_core import classify_entry_stage
from .cone_entry_monitor_core import ConeEntrySnapshot
from .cone_entry_monitor_core import ConeEntryThresholds
from .cone_entry_monitor_core import direction_label
from .cone_entry_monitor_core import entry_checks
from .cone_entry_monitor_core import physical_angle_to_command


STATUS_PATTERN = re.compile(
    r"state=(?P<state>\S+)\s+source=(?P<source>\S+)\s+"
    r"mode_label=(?P<label>\S+)\s+"
    r"cmd=\[(?P<angle>[-+0-9.eE]+),(?P<speed>[-+0-9.eE]+)\]\s+"
    r"reason=(?P<reason>.*)$"
)


class ConeEntryMonitor(Node):
    def __init__(self) -> None:
        super().__init__("cone_entry_monitor")
        self.declare_parameter("display_rate_hz", 4.0)
        self.declare_parameter("clear_terminal", True)
        self.declare_parameter("yolo_confidence", 0.50)
        self.declare_parameter("yolo_required_frames", 2)
        self.declare_parameter("yolo_timeout_sec", 0.75)
        self.declare_parameter("entry_distance_m", 3.0)
        self.declare_parameter("cluster_timeout_sec", 0.50)
        self.declare_parameter("path_confidence", 0.35)
        self.declare_parameter("command_required_frames", 3)
        self.declare_parameter("command_timeout_sec", 0.35)

        self.thresholds = ConeEntryThresholds(
            yolo_confidence=float(
                self.get_parameter("yolo_confidence").value
            ),
            yolo_frames=int(
                self.get_parameter("yolo_required_frames").value
            ),
            lidar_distance_m=float(
                self.get_parameter("entry_distance_m").value
            ),
            path_confidence=float(
                self.get_parameter("path_confidence").value
            ),
            command_frames=int(
                self.get_parameter("command_required_frames").value
            ),
        )
        self.yolo_timeout_sec = float(
            self.get_parameter("yolo_timeout_sec").value
        )
        self.cluster_timeout_sec = float(
            self.get_parameter("cluster_timeout_sec").value
        )
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.clear_terminal = bool(
            self.get_parameter("clear_terminal").value
        )

        self.object_time = float("-inf")
        self.yolo_time = float("-inf")
        self.yolo_confidence = 0.0
        self.yolo_frames = 0
        self.cluster_time = float("-inf")
        self.cluster_distance_m = float("inf")
        self.cluster_count = 0
        self.command_time = float("-inf")
        self.command_angle_deg = float("nan")
        self.command_speed = float("nan")
        self.command_confidence = 0.0
        self.command_frames = 0
        self.scan_time = float("-inf")
        self.front_lidar_distance_m = float("inf")
        self.processing_enabled = False
        self.processing_time = float("-inf")
        self.selector_state = "대기"
        self.selector_source = "대기"
        self.selector_label = "대기"
        self.selector_reason = "통합 선택기 메시지 대기"
        self.selector_time = float("-inf")
        self.shadow_angle = float("nan")
        self.shadow_speed = float("nan")
        self.shadow_time = float("-inf")
        self.last_selector_active = False
        self.entry_event_count = 0

        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ObjectDetectionArray,
            "/my_rule/object_detections",
            self._on_objects,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            "/my_rule/cone_clusters",
            self._on_clusters,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray,
            "/my_rule/cone_cmd",
            self._on_cone_command,
            10,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            "/my_rule/cone_processing_enabled",
            self._on_processing,
            transient_qos,
        )
        self.create_subscription(
            String,
            "/hybrid_gate/status",
            self._on_status,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/hybrid_gate/xycar_motor_shadow",
            self._on_shadow,
            10,
        )
        rate_hz = max(
            1.0, float(self.get_parameter("display_rate_hz").value)
        )
        self.create_timer(1.0 / rate_hz, self._display)

    @staticmethod
    def _fresh(now: float, stamp: float, timeout: float) -> bool:
        return math.isfinite(stamp) and now - stamp <= timeout

    def _yolo_confirmed(self, now: float) -> bool:
        return bool(
            self.yolo_frames >= self.thresholds.yolo_frames
            and self._fresh(now, self.yolo_time, self.yolo_timeout_sec)
        )

    def _on_objects(self, message: ObjectDetectionArray) -> None:
        now = time.monotonic()
        confidence = max(
            (
                float(item.confidence)
                for item in message.detections
                if str(item.class_name).strip().lower() == "cone"
            ),
            default=0.0,
        )
        self.object_time = now
        self.yolo_confidence = confidence
        if confidence >= self.thresholds.yolo_confidence:
            self.yolo_frames += 1
            self.yolo_time = now
        else:
            self.yolo_frames = 0

    def _on_clusters(self, message: PoseArray) -> None:
        distances = [
            math.hypot(float(pose.position.x), float(pose.position.y))
            for pose in message.poses
            if (
                float(pose.position.x) > 0.0
                and math.isfinite(float(pose.position.x))
                and math.isfinite(float(pose.position.y))
            )
        ]
        self.cluster_count = len(distances)
        self.cluster_distance_m = min(distances, default=float("inf"))
        self.cluster_time = time.monotonic()

    def _on_cone_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 3:
            return
        now = time.monotonic()
        self.command_angle_deg = float(message.data[0])
        self.command_speed = float(message.data[1])
        self.command_confidence = float(message.data[2])
        self.command_time = now
        valid = bool(
            self._yolo_confirmed(now)
            and self._fresh(
                now, self.cluster_time, self.cluster_timeout_sec
            )
            and self.cluster_count > 0
            and self.cluster_distance_m
            <= self.thresholds.lidar_distance_m
            and self.command_confidence
            >= self.thresholds.path_confidence
            and self.command_speed > 0.0
        )
        if not self._selector_active():
            self.command_frames = self.command_frames + 1 if valid else 0

    def _on_scan(self, message: LaserScan) -> None:
        samples = []
        angle = float(message.angle_min)
        lower = max(0.0, float(message.range_min))
        upper = float(message.range_max)
        if not math.isfinite(upper) or upper <= lower:
            upper = 20.0
        for value in message.ranges:
            distance = float(value)
            if (
                abs(math.degrees(angle)) <= 70.0
                and math.isfinite(distance)
                and lower <= distance <= upper
            ):
                samples.append(distance)
            angle += float(message.angle_increment)
        samples.sort()
        if samples:
            # A low quantile is steadier than one isolated minimum return.
            index = min(len(samples) - 1, int(0.10 * len(samples)))
            self.front_lidar_distance_m = samples[index]
        else:
            self.front_lidar_distance_m = float("inf")
        self.scan_time = time.monotonic()

    def _on_processing(self, message: Bool) -> None:
        self.processing_enabled = bool(message.data)
        self.processing_time = time.monotonic()

    def _on_status(self, message: String) -> None:
        match = STATUS_PATTERN.search(str(message.data))
        if match is None:
            return
        was_active = self._selector_active()
        self.selector_state = match.group("state")
        self.selector_source = match.group("source")
        self.selector_label = match.group("label")
        self.selector_reason = match.group("reason")
        self.selector_time = time.monotonic()
        active = self._selector_active()
        if active and not was_active:
            self.entry_event_count += 1
        elif was_active and not active:
            self.command_frames = 0
        self.last_selector_active = active

    def _on_shadow(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            return
        self.shadow_angle = float(message.data[0])
        self.shadow_speed = float(message.data[1])
        self.shadow_time = time.monotonic()

    def _selector_active(self) -> bool:
        return bool(
            self.selector_source == "CONE_RULE"
            or self.selector_label == "CONE_RULE"
        )

    def _snapshot(self, now: float) -> ConeEntrySnapshot:
        return ConeEntrySnapshot(
            scan_fresh=self._fresh(now, self.scan_time, 0.50),
            object_stream_fresh=self._fresh(
                now, self.object_time, self.yolo_timeout_sec
            ),
            selector_active=self._selector_active(),
            yolo_confidence=self.yolo_confidence,
            yolo_frames=self.yolo_frames,
            yolo_fresh=self._fresh(
                now, self.yolo_time, self.yolo_timeout_sec
            ),
            lidar_distance_m=self.cluster_distance_m,
            lidar_count=self.cluster_count,
            lidar_fresh=self._fresh(
                now, self.cluster_time, self.cluster_timeout_sec
            ),
            command_confidence=self.command_confidence,
            command_speed=self.command_speed,
            command_fresh=self._fresh(
                now, self.command_time, self.command_timeout_sec
            ),
            command_frames=self.command_frames,
        )

    @staticmethod
    def _mark(value: bool) -> str:
        return "충족" if value else "대기"

    @staticmethod
    def _number(value: float, unit: str = "", digits: int = 2) -> str:
        if not math.isfinite(float(value)):
            return "미확인"
        return f"{float(value):.{digits}f}{unit}"

    def _display(self) -> None:
        now = time.monotonic()
        snapshot = self._snapshot(now)
        yolo_ok, lidar_ok, command_ok = entry_checks(
            snapshot, self.thresholds
        )
        stage = classify_entry_stage(snapshot, self.thresholds)
        entered = "예" if snapshot.selector_active else "아니오"
        ready = "예" if yolo_ok and lidar_ok and command_ok else "아니오"
        processing = "작동" if self.processing_enabled else "대기"
        distance = self._number(snapshot.lidar_distance_m, "m")
        front = self._number(
            self.front_lidar_distance_m
            if snapshot.scan_fresh
            else float("inf"),
            "m",
        )
        target_angle = self._number(self.command_angle_deg, "°", 1)
        converted = self._number(
            physical_angle_to_command(self.command_angle_deg), "", 1
        )
        target_speed = self._number(self.command_speed, "", 1)
        shadow_angle = self._number(self.shadow_angle, "", 1)
        shadow_speed = self._number(self.shadow_speed, "", 1)
        yolo_age = self._number(now - self.object_time, "s", 2)
        cluster_age = self._number(now - self.cluster_time, "s", 2)
        command_age = self._number(now - self.command_time, "s", 2)

        lines = [
            "============================================================",
            " 라바콘 통합 진입 실시간 감시 — 손으로 차량 밀기 전용",
            " [안전] VESC/모터 드라이버 미실행 · /xycar_motor 출력 미생성",
            "============================================================",
            f"구간 돌입 여부 : {entered} | 진입 준비: {ready}",
            f"현재 단계      : {stage}",
            f"라바콘 처리    : {processing}",
            "",
            "[통합 진입 판정 조건]",
            (
                f"  1. YOLO     [{self._mark(yolo_ok)}] "
                f"confidence={snapshot.yolo_confidence:.2f} "
                f"(기준 {self.thresholds.yolo_confidence:.2f}), "
                f"연속={snapshot.yolo_frames}/"
                f"{self.thresholds.yolo_frames}, age={yolo_age}"
            ),
            (
                f"  2. LiDAR    [{self._mark(lidar_ok)}] "
                f"진입구간까지 거리={distance} "
                f"(기준 <= {self.thresholds.lidar_distance_m:.2f}m), "
                f"후보={snapshot.lidar_count}개, age={cluster_age}"
            ),
            (
                f"  3. 경로명령 [{self._mark(command_ok)}] "
                f"confidence={snapshot.command_confidence:.2f} "
                f"(기준 {self.thresholds.path_confidence:.2f}), "
                f"연속={snapshot.command_frames}/"
                f"{self.thresholds.command_frames}, age={command_age}"
            ),
            "",
            "[계산된 목표값 — 실제 바퀴에는 전달되지 않음]",
            (
                f"  라바콘 목표각 : {target_angle} "
                f"({direction_label(self.command_angle_deg)}) "
                f"-> Xycar 조향명령 예상={converted}"
            ),
            f"  라바콘 목표속도: {target_speed} (진입 후 설정값은 10.0)",
            (
                f"  통합 선택 후보 : 조향={shadow_angle}, "
                f"속도={shadow_speed}, 모드={self.selector_source}/"
                f"{self.selector_label}"
            ),
            f"  선택기 상태    : {self.selector_state} | {self.selector_reason}",
            "",
            (
                f"전방 LiDAR 참고(±70°, 10% 분위값): {front} "
                "— 라바콘 확정 거리가 아님"
            ),
            (
                "※ '진입구간까지 거리'는 통합 진입 판정에 실제 사용되는 "
                "가장 가까운 라바콘 후보 거리입니다."
            ),
            f"감지된 CONE_RULE 진입 횟수: {self.entry_event_count}",
            "종료: Ctrl+C",
        ]
        prefix = "\033[2J\033[H" if self.clear_terminal else ""
        sys.stdout.write(prefix + "\n".join(lines) + "\n")
        sys.stdout.flush()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ConeEntryMonitor()
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
