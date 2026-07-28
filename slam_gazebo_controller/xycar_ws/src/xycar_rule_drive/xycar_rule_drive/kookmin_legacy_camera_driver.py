from __future__ import annotations

from collections import deque
import math
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


class KookminLegacyCameraDriver(Node):
    """Camera-only rule driver inspired by the Kookmin Hough/BEV examples.

    The original contest code combines a bird's-eye crop, Canny edges,
    Hough line segments, clustered lane x positions, and a short temporal
    prediction. This node keeps that shape but removes contest-specific
    custom messages and publishes the Xycar Float32MultiArray command used by
    our Gazebo bridge.
    """

    def __init__(self) -> None:
        super().__init__("kookmin_legacy_camera_driver")
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter("debug_image_topic", "/rule_drive/legacy_debug_image")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("bev_width", 640)
        self.declare_parameter("bev_height", 160)
        self.declare_parameter("src_top_y_ratio", 0.62)
        self.declare_parameter("src_bottom_y_ratio", 0.82)
        self.declare_parameter("src_top_half_width_ratio", 0.34)
        self.declare_parameter("src_bottom_half_width_ratio", 0.50)
        self.declare_parameter("dst_margin_ratio", 0.10)

        self.declare_parameter("canny_low", 50)
        self.declare_parameter("canny_high", 150)
        self.declare_parameter("blur_kernel", 7)
        self.declare_parameter("hough_threshold", 22)
        self.declare_parameter("hough_min_line_length", 10)
        self.declare_parameter("hough_max_line_gap", 6)
        self.declare_parameter("lane_row_ratio", 0.55)
        self.declare_parameter("angle_tolerance_rad", 0.65)
        self.declare_parameter("lane_cluster_threshold_px", 32.0)
        self.declare_parameter("lane_match_threshold_px", 74.0)
        self.declare_parameter("lane_left_offset_px", -200.0)
        self.declare_parameter("lane_right_offset_px", 200.0)
        self.declare_parameter("min_cos_angle", 0.50)
        self.declare_parameter("lane_update_weight", 0.68)
        self.declare_parameter("prediction_weight", 0.32)
        self.declare_parameter("angle_prev_weight", 0.72)
        self.declare_parameter("angle_new_weight", 0.28)
        self.declare_parameter("max_lane_delta_px", 80.0)
        self.declare_parameter("angle_correction_gain_px", 70.0)

        self.declare_parameter("target_lane", "auto")
        self.declare_parameter("target_x_offset_px", 0.0)
        self.declare_parameter("color_target_weight", 0.45)
        self.declare_parameter("white_s_max", 85)
        self.declare_parameter("white_v_min", 140)
        self.declare_parameter("yellow_h_min", 15)
        self.declare_parameter("yellow_h_max", 42)
        self.declare_parameter("yellow_s_min", 55)
        self.declare_parameter("yellow_v_min", 105)
        self.declare_parameter("color_band_half_height_px", 12)
        self.declare_parameter("color_min_pixels", 8)

        self.declare_parameter("speed_command", 7.0)
        self.declare_parameter("min_speed_command", 4.0)
        self.declare_parameter("slow_down_error_px", 55.0)
        self.declare_parameter("stop_error_px", 145.0)
        self.declare_parameter("stop_on_large_error", False)
        self.declare_parameter("steering_gain_cmd_per_px", 0.30)
        self.declare_parameter("heading_gain_cmd_per_rad", 12.0)
        self.declare_parameter("angle_command_min", -42.0)
        self.declare_parameter("angle_command_max", 42.0)
        self.declare_parameter("perception_timeout_sec", 0.35)

        self.bridge = CvBridge()
        self.bev_width = int(self.get_parameter("bev_width").value)
        self.bev_height = int(self.get_parameter("bev_height").value)
        self.lane_y = int(
            clamp(
                float(self.get_parameter("lane_row_ratio").value),
                0.05,
                0.95,
            )
            * self.bev_height
        )
        self.target_lane = str(self.get_parameter("target_lane").value).lower()
        self.color_target_weight = clamp(
            float(self.get_parameter("color_target_weight").value),
            0.0,
            1.0,
        )
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("motor_topic").value),
            10,
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("debug_image_topic").value),
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            10,
        )

        rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.min_period = 1.0 / rate_hz
        self.last_process_time = 0.0
        self.last_command_time = time.monotonic()

        center = self.bev_width * 0.5
        self.lane = np.array([center - 200.0, center, center + 200.0], dtype=np.float64)
        self.lane_angle = 0.0
        self.prev_angles = deque([0.0], maxlen=3)
        self.last_angle_command = 0.0

        self.get_logger().info(
            "legacy camera driver ready: /image_raw -> BEV/Hough lane target -> /xycar_motor"
        )

    def on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_process_time < self.min_period:
            return
        self.last_process_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}", throttle_duration_sec=2.0)
            return

        result = self.detect_target(frame)
        if result is None:
            if now - self.last_command_time > float(self.get_parameter("perception_timeout_sec").value):
                self.publish_motor(0.0, 0.0)
            return

        angle_cmd, speed_cmd, debug = result
        self.last_command_time = now
        self.last_angle_command = angle_cmd
        self.publish_motor(angle_cmd, speed_cmd)

        if self.publish_debug_image and self.debug_image_pub.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

    def detect_target(self, frame: np.ndarray) -> tuple[float, float, np.ndarray] | None:
        bev, src = self.make_birds_eye_view(frame)
        canny = self.apply_canny(bev)
        lines = cv2.HoughLinesP(
            canny,
            1,
            np.pi / 180.0,
            int(self.get_parameter("hough_threshold").value),
            minLineLength=int(self.get_parameter("hough_min_line_length").value),
            maxLineGap=int(self.get_parameter("hough_max_line_gap").value),
        )

        positions, accepted_lines = self.hough_lane_positions(lines)
        lane_candidates = self.cluster_positions(positions)
        predicted = self.predict_lane()
        self.refine_lane(lane_candidates, predicted)

        hough_target = self.target_from_legacy_lane()
        color_target = self.target_from_color_masks(bev)
        if color_target is None:
            target_x = hough_target
        else:
            target_x = (1.0 - self.color_target_weight) * hough_target
            target_x += self.color_target_weight * color_target
        target_x += float(self.get_parameter("target_x_offset_px").value)
        target_x = clamp(target_x, 0.0, float(self.bev_width - 1))

        center_x = self.bev_width * 0.5
        error_px = target_x - center_x
        angle_cmd = self.compute_angle_command(error_px)
        speed_cmd = self.compute_speed_command(error_px)
        debug = self.draw_debug(frame, bev, canny, src, accepted_lines, lane_candidates, target_x, color_target)
        return angle_cmd, speed_cmd, debug

    def make_birds_eye_view(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        height, width = frame.shape[:2]
        cx = width * 0.5
        top_y = height * float(self.get_parameter("src_top_y_ratio").value)
        bottom_y = height * float(self.get_parameter("src_bottom_y_ratio").value)
        top_half = width * float(self.get_parameter("src_top_half_width_ratio").value)
        bottom_half = width * float(self.get_parameter("src_bottom_half_width_ratio").value)
        src = np.float32(
            [
                [cx - top_half, top_y],
                [cx + top_half, top_y],
                [cx + bottom_half, bottom_y],
                [cx - bottom_half, bottom_y],
            ]
        )
        margin = self.bev_width * float(self.get_parameter("dst_margin_ratio").value)
        dst = np.float32(
            [
                [margin, 0.0],
                [self.bev_width - margin, 0.0],
                [self.bev_width - margin, self.bev_height - 1.0],
                [margin, self.bev_height - 1.0],
            ]
        )
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(frame, matrix, (self.bev_width, self.bev_height)), src

    def apply_canny(self, bev: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        kernel = int(self.get_parameter("blur_kernel").value)
        if kernel % 2 == 0:
            kernel += 1
        kernel = max(3, kernel)
        blurred = cv2.GaussianBlur(gray, (kernel, kernel), 0)
        return cv2.Canny(
            blurred,
            int(self.get_parameter("canny_low").value),
            int(self.get_parameter("canny_high").value),
        )

    def hough_lane_positions(self, lines) -> tuple[list[float], list[tuple[int, int, int, int]]]:
        positions: list[float] = []
        accepted_lines: list[tuple[int, int, int, int]] = []
        thetas: list[float] = []
        tolerance = float(self.get_parameter("angle_tolerance_rad").value)

        if lines is None:
            self.prev_angles.append(self.lane_angle)
            return positions, accepted_lines

        for line in lines[:, 0]:
            x1, y1, x2, y2 = [int(value) for value in line]
            if y1 == y2:
                continue
            flag = 1.0 if y1 - y2 > 0 else -1.0
            theta = math.atan2(flag * (x2 - x1), flag * (y1 - y2))
            if abs(theta - self.lane_angle) > tolerance:
                continue
            x_at_row = float((x2 - x1) * (self.lane_y - y1)) / float(y2 - y1) + x1
            if 0.0 <= x_at_row < self.bev_width:
                positions.append(x_at_row)
                thetas.append(theta)
                accepted_lines.append((x1, y1, x2, y2))

        self.prev_angles.append(self.lane_angle)
        if thetas:
            old_weight = float(self.get_parameter("angle_prev_weight").value)
            new_weight = float(self.get_parameter("angle_new_weight").value)
            self.lane_angle = old_weight * self.lane_angle + new_weight * float(np.mean(thetas))
        return positions, accepted_lines

    def cluster_positions(self, positions: list[float]) -> list[float]:
        threshold = float(self.get_parameter("lane_cluster_threshold_px").value)
        clusters: list[list[float]] = []
        for position in positions:
            for cluster in clusters:
                if abs(float(np.mean(cluster)) - position) < threshold:
                    cluster.append(position)
                    break
            else:
                clusters.append([position])
        return [float(np.mean(cluster)) for cluster in clusters]

    def predict_lane(self) -> np.ndarray:
        denom = max(math.cos(self.lane_angle), float(self.get_parameter("min_cos_angle").value))
        left_offset = float(self.get_parameter("lane_left_offset_px").value)
        right_offset = float(self.get_parameter("lane_right_offset_px").value)
        predicted = self.lane[1] + np.array([left_offset / denom, 0.0, right_offset / denom])
        predicted += (
            self.lane_angle - float(np.mean(self.prev_angles))
        ) * float(self.get_parameter("angle_correction_gain_px").value)
        return predicted

    def refine_lane(self, candidates: list[float], predicted: np.ndarray) -> None:
        if not candidates:
            self.lane = predicted
            return

        match_threshold = float(self.get_parameter("lane_match_threshold_px").value)
        denom = max(math.cos(self.lane_angle), float(self.get_parameter("min_cos_angle").value))
        left_to_center = abs(float(self.get_parameter("lane_left_offset_px").value))
        right_to_center = abs(float(self.get_parameter("lane_right_offset_px").value))
        outer_width = left_to_center + right_to_center

        possible_lanes: list[list[float]] = []
        for candidate in candidates:
            lane_index = int(np.argmin(np.abs(self.lane - candidate)))
            if lane_index == 0:
                estimate = [
                    candidate,
                    candidate + left_to_center / denom,
                    candidate + outer_width / denom,
                ]
            elif lane_index == 1:
                estimate = [
                    candidate - left_to_center / denom,
                    candidate,
                    candidate + right_to_center / denom,
                ]
            else:
                estimate = [
                    candidate - outer_width / denom,
                    candidate - right_to_center / denom,
                    candidate,
                ]

            expanded = []
            for index, value in enumerate(estimate):
                nearby = [c for c in candidates if abs(c - value) < match_threshold]
                expanded.append(nearby if nearby else [value])
            for left in expanded[0]:
                for center in expanded[1]:
                    for right in expanded[2]:
                        possible_lanes.append([left, center, right])

        if not possible_lanes:
            self.lane = predicted
            return

        possible = np.array(possible_lanes, dtype=np.float64)
        best = possible[int(np.argmin(np.sum((possible - predicted) ** 2, axis=1)))]
        new_lane = (
            float(self.get_parameter("lane_update_weight").value) * best
            + float(self.get_parameter("prediction_weight").value) * predicted
        )
        max_delta = float(self.get_parameter("max_lane_delta_px").value)
        self.lane += np.clip(new_lane - self.lane, -max_delta, max_delta)

    def target_from_legacy_lane(self) -> float:
        mode = self.target_lane
        if mode == "left":
            return float((self.lane[0] + self.lane[1]) * 0.5)
        if mode == "right":
            return float((self.lane[1] + self.lane[2]) * 0.5)
        if mode == "center":
            return float(self.lane[1])

        center_x = self.bev_width * 0.5
        left_target = float((self.lane[0] + self.lane[1]) * 0.5)
        right_target = float((self.lane[1] + self.lane[2]) * 0.5)
        return left_target if abs(left_target - center_x) < abs(right_target - center_x) else right_target

    def target_from_color_masks(self, bev: np.ndarray) -> float | None:
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, int(self.get_parameter("white_v_min").value)], dtype=np.uint8),
            np.array([180, int(self.get_parameter("white_s_max").value), 255], dtype=np.uint8),
        )
        yellow_mask = cv2.inRange(
            hsv,
            np.array(
                [
                    int(self.get_parameter("yellow_h_min").value),
                    int(self.get_parameter("yellow_s_min").value),
                    int(self.get_parameter("yellow_v_min").value),
                ],
                dtype=np.uint8,
            ),
            np.array([int(self.get_parameter("yellow_h_max").value), 255, 255], dtype=np.uint8),
        )
        yellow_x = self.mean_x_in_band(yellow_mask)
        white_xs = self.cluster_positions(self.xs_in_band(white_mask))
        if yellow_x is None or not white_xs:
            return None

        center_x = self.bev_width * 0.5
        if self.target_lane == "left":
            candidates = [x for x in white_xs if x < yellow_x]
        elif self.target_lane == "right":
            candidates = [x for x in white_xs if x > yellow_x]
        else:
            side_sign = -1.0 if yellow_x > center_x else 1.0
            candidates = [x for x in white_xs if (x - yellow_x) * side_sign > 0.0]

        if not candidates:
            candidates = white_xs
        white_x = min(candidates, key=lambda value: abs(value - center_x))
        return float((yellow_x + white_x) * 0.5)

    def xs_in_band(self, mask: np.ndarray) -> list[float]:
        half = int(self.get_parameter("color_band_half_height_px").value)
        y0 = max(0, self.lane_y - half)
        y1 = min(self.bev_height, self.lane_y + half + 1)
        ys, xs = np.nonzero(mask[y0:y1, :])
        min_pixels = int(self.get_parameter("color_min_pixels").value)
        if xs.size < min_pixels:
            return []
        return [float(x) for x in xs]

    def mean_x_in_band(self, mask: np.ndarray) -> float | None:
        xs = self.xs_in_band(mask)
        if not xs:
            return None
        clusters = self.cluster_positions(xs)
        if not clusters:
            return None
        center_x = self.bev_width * 0.5
        return min(clusters, key=lambda value: abs(value - center_x))

    def compute_angle_command(self, error_px: float) -> float:
        command = error_px * float(self.get_parameter("steering_gain_cmd_per_px").value)
        command += self.lane_angle * float(self.get_parameter("heading_gain_cmd_per_rad").value)
        return clamp(
            command,
            float(self.get_parameter("angle_command_min").value),
            float(self.get_parameter("angle_command_max").value),
        )

    def compute_speed_command(self, error_px: float) -> float:
        error_abs = abs(error_px)
        stop_error = float(self.get_parameter("stop_error_px").value)
        if bool(self.get_parameter("stop_on_large_error").value) and error_abs >= stop_error:
            return 0.0
        slow_error = float(self.get_parameter("slow_down_error_px").value)
        base_speed = float(self.get_parameter("speed_command").value)
        min_speed = float(self.get_parameter("min_speed_command").value)
        if error_abs <= slow_error:
            return base_speed
        ratio = (error_abs - slow_error) / max(1.0, stop_error - slow_error)
        return base_speed + ratio * (min_speed - base_speed)

    def publish_motor(self, angle: float, speed: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(angle), float(speed)]
        self.motor_pub.publish(msg)

    def draw_debug(
        self,
        frame: np.ndarray,
        bev: np.ndarray,
        canny: np.ndarray,
        src: np.ndarray,
        accepted_lines: list[tuple[int, int, int, int]],
        candidates: list[float],
        target_x: float,
        color_target: float | None,
    ) -> np.ndarray:
        frame_view = frame.copy()
        cv2.polylines(frame_view, [src.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
        frame_view = cv2.resize(frame_view, (self.bev_width, 360))

        bev_view = bev.copy()
        for x1, y1, x2, y2 in accepted_lines:
            cv2.line(bev_view, (x1, y1), (x2, y2), (0, 0, 255), 1)
        cv2.line(bev_view, (0, self.lane_y), (self.bev_width - 1, self.lane_y), (0, 220, 255), 1)
        for candidate in candidates:
            cv2.circle(bev_view, (int(candidate), self.lane_y), 4, (255, 0, 255), -1)

        colors = [(255, 0, 0), (0, 220, 255), (0, 0, 255)]
        labels = ["L", "C", "R"]
        for value, color, label in zip(self.lane, colors, labels):
            cv2.circle(bev_view, (int(value), self.lane_y), 5, color, -1)
            cv2.putText(
                bev_view,
                label,
                (int(value) + 6, self.lane_y - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )

        center_x = int(self.bev_width * 0.5)
        cv2.line(bev_view, (center_x, 0), (center_x, self.bev_height - 1), (255, 255, 255), 1)
        cv2.circle(bev_view, (int(target_x), self.lane_y), 6, (0, 255, 0), -1)
        if color_target is not None:
            cv2.circle(bev_view, (int(color_target), self.lane_y), 5, (0, 180, 255), 2)

        error = target_x - self.bev_width * 0.5
        info = f"target={target_x:.1f}px error={error:.1f}px angle={self.last_angle_command:.1f}"
        cv2.putText(bev_view, info, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1)

        canny_bgr = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
        lower = np.vstack([bev_view, canny_bgr])
        return np.vstack([frame_view, lower])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KookminLegacyCameraDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
