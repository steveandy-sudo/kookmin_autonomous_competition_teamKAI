#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""차량 2대 회피 offset 계산 노드.

YOLO 노드가 발행한 /yolo_obstacle/stable_counts(front,left,right,behind 개수)를 받아
NORMAL -> AVOID_RIGHT -> PASS_LEFT -> RETURN_CENTER 상태머신으로
/obstacle_avoidance/path_offset 과 /obstacle_avoidance/speed_limit 을 발행한다.
lane_core가 이 offset/speed_limit을 받아 경로를 밀고 속도를 제한한다.

원본 ObstacleAvoidanceDriver 알고리즘 그대로이다.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String


class ObstacleAvoidanceDriver(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_driver")

        self.declare_parameter("right_offset", 85.0)
        self.declare_parameter("left_offset", 85.0)
        self.declare_parameter("offset_step", 6.0)
        self.declare_parameter("speed_limit", 1.2)
        self.declare_parameter("front_trigger", 1)
        self.declare_parameter("behind_passed_trigger", 1)
        self.declare_parameter("behind_return_trigger", 2)
        self.declare_parameter("return_deadband", 8.0)
        self.declare_parameter("publish_hz", 20.0)
        # AVOID_RIGHT에서 front·behind 둘 다 0이 이 시간(초) 지속되면 오검출로 보고 NORMAL 복귀
        self.declare_parameter("avoid_clear_reset_sec", 0.5)

        self.right_offset = float(self.get_parameter("right_offset").value)
        self.left_offset = float(self.get_parameter("left_offset").value)
        self.offset_step = float(self.get_parameter("offset_step").value)
        self.speed_limit = float(self.get_parameter("speed_limit").value)
        self.front_trigger = int(self.get_parameter("front_trigger").value)
        self.behind_passed_trigger = int(
            self.get_parameter("behind_passed_trigger").value
        )
        self.behind_return_trigger = int(
            self.get_parameter("behind_return_trigger").value
        )
        self.return_deadband = float(self.get_parameter("return_deadband").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self.avoid_clear_reset_sec = float(
            self.get_parameter("avoid_clear_reset_sec").value
        )

        self.counts = {
            "front": 0,
            "left": 0,
            "right": 0,
            "behind": 0,
        }
        self.mode = "NORMAL"
        self.last_mode = None
        self.target_offset = 0.0
        self.current_offset = 0.0
        # AVOID_RIGHT에서 front·behind 둘 다 0이 된 시각(자동복귀 타이머). None이면 미시작.
        self.clear_start_time = None

        self.count_sub = self.create_subscription(
            Int32MultiArray,
            "/yolo_obstacle/stable_counts",
            self.count_callback,
            10,
        )
        self.path_offset_pub = self.create_publisher(
            Float32,
            "/obstacle_avoidance/path_offset",
            10,
        )
        self.speed_limit_pub = self.create_publisher(
            Float32,
            "/obstacle_avoidance/speed_limit",
            10,
        )
        self.mode_pub = self.create_publisher(
            String,
            "/obstacle_avoidance/mode",
            10,
        )

        self.timer = self.create_timer(1.0 / max(publish_hz, 1.0), self.loop)
        self.get_logger().info("Obstacle Avoidance Driver started")
        self.get_logger().info("Subscribing: /yolo_obstacle/stable_counts")
        self.get_logger().info("Publishing: /obstacle_avoidance/path_offset")
        self.get_logger().info("Publishing: /obstacle_avoidance/speed_limit")

    def count_callback(self, msg):
        if len(msg.data) < 4:
            return

        self.counts = {
            "front": int(msg.data[0]),
            "left": int(msg.data[1]),
            "right": int(msg.data[2]),
            "behind": int(msg.data[3]),
        }

    def loop(self):
        self.update_mode()
        self.update_target_offset()

        self.current_offset = self.ramp_value(
            self.current_offset,
            self.target_offset,
            self.offset_step,
        )

        self.path_offset_pub.publish(Float32(data=float(self.current_offset)))
        self.speed_limit_pub.publish(Float32(data=float(self.get_speed_limit())))
        self.mode_pub.publish(String(data=self.mode))

        if self.last_mode != self.mode:
            self.get_logger().info(
                f"mode={self.mode}, "
                f"front={self.counts['front']}, "
                f"behind={self.counts['behind']}, "
                f"target_offset={self.target_offset:.1f}, "
                f"speed_limit={self.get_speed_limit():.1f}"
            )
            self.last_mode = self.mode

        # === [AVOID] 진단용: 0.5초마다 현재 상태를 찍는다 (동작 변경 없음, 로그 전용) ===
        # CLI 없이 launch 터미널에서 바로 회피 상태/offset/speed_limit을 확인하기 위함.
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now - getattr(self, "_last_diag", 0.0) >= 0.5:
            self._last_diag = now
            self.get_logger().info(
                "[AVOID] mode=%s offset=%.1f target=%.1f spd_lim=%.1f front=%d behind=%d"
                % (
                    self.mode,
                    float(self.current_offset),
                    float(self.target_offset),
                    float(self.get_speed_limit()),
                    int(self.counts["front"]),
                    int(self.counts["behind"]),
                )
            )

    def update_mode(self):
        front_count = self.counts["front"]
        behind_count = self.counts["behind"]

        if self.mode == "NORMAL":
            if front_count >= self.front_trigger:
                self.mode = "AVOID_RIGHT"
                self.clear_start_time = None

        elif self.mode == "AVOID_RIGHT":
            if behind_count >= self.behind_passed_trigger:
                self.mode = "PASS_LEFT"
                self.clear_start_time = None
            else:
                # 오검출/조기검출로 AVOID_RIGHT에 영구히 갇히는 것 방지.
                # front·behind 둘 다 0인 상태가 avoid_clear_reset_sec 이상 지속되면
                # (= 추월할 실제 차가 없다는 뜻) NORMAL로 자동 복귀한다.
                # 실제 차 회피 중에는 추월 직전까지 front가 잡히고, 추월 순간 behind가
                # 잡혀 PASS_LEFT로 넘어가므로 "둘 다 0이 1.5초"는 거의 발생하지 않는다.
                now = self.get_clock().now().nanoseconds * 1.0e-9
                if front_count == 0 and behind_count == 0:
                    if self.clear_start_time is None:
                        self.clear_start_time = now
                    elif now - self.clear_start_time >= self.avoid_clear_reset_sec:
                        self.get_logger().info(
                            "[AVOID] front/behind 0 %.1fs 지속 → 오검출로 보고 NORMAL 복귀"
                            % self.avoid_clear_reset_sec
                        )
                        self.mode = "NORMAL"
                        self.clear_start_time = None
                else:
                    # front가 다시 잡히면(실제 회피 진행 가능성) 타이머 리셋
                    self.clear_start_time = None

        elif self.mode == "PASS_LEFT":
            if behind_count >= self.behind_return_trigger:
                self.mode = "RETURN_CENTER"

        elif self.mode == "RETURN_CENTER":
            if abs(self.current_offset) <= self.return_deadband:
                self.mode = "NORMAL"
                self.clear_start_time = None

    def update_target_offset(self):
        if self.mode == "AVOID_RIGHT":
            # pid.py 차량 좌표계: 양수 y가 왼쪽. 오른쪽 회피는 음수.
            self.target_offset = -self.right_offset
        elif self.mode == "PASS_LEFT":
            self.target_offset = self.left_offset
        else:
            self.target_offset = 0.0

    def get_speed_limit(self):
        if self.mode in ("AVOID_RIGHT", "PASS_LEFT"):
            return self.speed_limit

        return -1.0

    def ramp_value(self, current, target, step):
        delta = target - current

        if abs(delta) <= step:
            return float(target)

        return float(current + math.copysign(step, delta))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Obstacle Avoidance Driver terminated")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
