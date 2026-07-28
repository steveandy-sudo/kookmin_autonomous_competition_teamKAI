from __future__ import annotations

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class BevPreviewHomography(Node):
    def __init__(self) -> None:
        super().__init__("bev_preview_homography")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("use_compressed", False)
        self.declare_parameter("calib_yaml", "")

        self.declare_parameter("enable_rectify", False)
        self.declare_parameter("rect_balance", 0.3)

        self.declare_parameter("src_tl_x_ratio", 0.39)
        self.declare_parameter("src_tr_x_ratio", 0.67)
        self.declare_parameter("src_br_x_ratio", 1.10)
        self.declare_parameter("src_bl_x_ratio", -0.10)
        self.declare_parameter("src_top_y_ratio", 0.48)
        self.declare_parameter("src_bottom_y_ratio", 0.72)

        self.declare_parameter("bev_width", 640)
        self.declare_parameter("bev_height", 220)
        self.declare_parameter("dst_left_ratio", 0.10)
        self.declare_parameter("dst_right_ratio", 0.90)

        self.declare_parameter("lateral_m_per_px", 0.0022)
        self.declare_parameter("forward_m_per_px", 0.005)
        self.declare_parameter("black_threshold", 70)

        self.bridge = CvBridge()
        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.K: np.ndarray | None = None
        self.D: np.ndarray | None = None
        self.distortion_model = "fisheye"
        self.K_rect: np.ndarray | None = None
        self.rect_map1: np.ndarray | None = None
        self.rect_map2: np.ndarray | None = None
        self.rect_size: tuple[int, int] | None = None
        self.M: np.ndarray | None = None
        self.M_inv: np.ndarray | None = None
        self.src: np.ndarray | None = None
        self.dst: np.ndarray | None = None
        self.last_input_shape: tuple[int, int] | None = None
        self.last_bev_shape: tuple[int, int] | None = None

        self.load_calib_yaml()

        image_topic = str(self.get_parameter("image_topic").value)
        use_compressed = bool(self.get_parameter("use_compressed").value)
        if use_compressed:
            self.create_subscription(
                CompressedImage,
                image_topic,
                self.compressed_cb,
                self.sensor_qos,
            )
            self.get_logger().info(f"subscribe compressed image: {image_topic}")
        else:
            self.create_subscription(Image, image_topic, self.image_cb, self.sensor_qos)
            self.get_logger().info(f"subscribe raw image: {image_topic}")

        cv2.namedWindow("source", cv2.WINDOW_NORMAL)
        cv2.namedWindow("rectified_with_roi", cv2.WINDOW_NORMAL)
        cv2.namedWindow("BEV", cv2.WINDOW_NORMAL)
        cv2.namedWindow("BEV black mask", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("BEV", self.on_bev_mouse)

    def load_calib_yaml(self) -> None:
        path = str(self.get_parameter("calib_yaml").value)
        if not path:
            self.get_logger().warn("calib_yaml empty. rectification will be disabled.")
            return
        try:
            import yaml

            with open(path, "r", encoding="utf-8") as file:
                data = yaml.safe_load(file)
            if "camera_matrix" in data:
                self.K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
            elif "K" in data:
                self.K = np.array(data["K"], dtype=np.float64).reshape(3, 3)
            else:
                raise RuntimeError("camera_matrix or K is missing")

            if "distortion_coefficients" in data:
                self.D = np.array(data["distortion_coefficients"]["data"], dtype=np.float64)
            elif "D" in data:
                self.D = np.array(data["D"], dtype=np.float64)
            else:
                self.D = np.zeros(4, dtype=np.float64)

            self.distortion_model = str(data.get("distortion_model", "fisheye")).lower()
            self.get_logger().info(f"loaded calib yaml: {path}")
            self.get_logger().info(f"distortion_model: {self.distortion_model}")
        except Exception as exc:
            self.get_logger().error(f"failed to load calib_yaml: {exc}")
            self.K = None
            self.D = None

    def build_rectify_map(self, width: int, height: int) -> bool:
        if self.K is None or self.D is None:
            return False

        size = (width, height)
        balance = float(self.get_parameter("rect_balance").value)
        R = np.eye(3, dtype=np.float64)
        model = self.distortion_model.lower()

        try:
            if "fisheye" in model or "equidistant" in model:
                d4 = np.zeros((4, 1), dtype=np.float64)
                n = min(4, len(self.D))
                d4[:n, 0] = self.D[:n]
                self.K_rect = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    self.K,
                    d4,
                    size,
                    R,
                    balance=balance,
                    new_size=size,
                )
                self.rect_map1, self.rect_map2 = cv2.fisheye.initUndistortRectifyMap(
                    self.K,
                    d4,
                    R,
                    self.K_rect,
                    size,
                    cv2.CV_32FC1,
                )
            else:
                self.K_rect, _ = cv2.getOptimalNewCameraMatrix(
                    self.K,
                    self.D,
                    size,
                    alpha=balance,
                    newImgSize=size,
                )
                self.rect_map1, self.rect_map2 = cv2.initUndistortRectifyMap(
                    self.K,
                    self.D,
                    R,
                    self.K_rect,
                    size,
                    cv2.CV_32FC1,
                )
            self.rect_size = size
            self.get_logger().info(f"rectify map built: {size}, balance={balance}")
            return True
        except cv2.error as exc:
            self.get_logger().error(f"rectify map failed: {exc}")
            return False

    def rectify(self, frame: np.ndarray) -> np.ndarray:
        if not bool(self.get_parameter("enable_rectify").value):
            return frame
        height, width = frame.shape[:2]
        if self.rect_map1 is None or self.rect_map2 is None or self.rect_size != (width, height):
            if not self.build_rectify_map(width, height):
                return frame
        return cv2.remap(
            frame,
            self.rect_map1,
            self.rect_map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

    def build_homography(self, width: int, height: int) -> None:
        out_w = int(self.get_parameter("bev_width").value)
        out_h = int(self.get_parameter("bev_height").value)

        tl_x = float(self.get_parameter("src_tl_x_ratio").value) * width
        tr_x = float(self.get_parameter("src_tr_x_ratio").value) * width
        br_x = float(self.get_parameter("src_br_x_ratio").value) * width
        bl_x = float(self.get_parameter("src_bl_x_ratio").value) * width
        top_y = float(self.get_parameter("src_top_y_ratio").value) * height
        bottom_y = float(self.get_parameter("src_bottom_y_ratio").value) * height

        dst_l = float(self.get_parameter("dst_left_ratio").value) * out_w
        dst_r = float(self.get_parameter("dst_right_ratio").value) * out_w

        self.src = np.float32([[tl_x, top_y], [tr_x, top_y], [br_x, bottom_y], [bl_x, bottom_y]])
        self.dst = np.float32([[dst_l, 0], [dst_r, 0], [dst_r, out_h], [dst_l, out_h]])
        self.M = cv2.getPerspectiveTransform(self.src, self.dst)
        self.M_inv = cv2.getPerspectiveTransform(self.dst, self.src)
        self.last_input_shape = (width, height)
        self.last_bev_shape = (out_w, out_h)

    def make_bev(self, rectified: np.ndarray) -> np.ndarray:
        height, width = rectified.shape[:2]
        out_w = int(self.get_parameter("bev_width").value)
        out_h = int(self.get_parameter("bev_height").value)
        if (
            self.M is None
            or self.last_input_shape != (width, height)
            or self.last_bev_shape != (out_w, out_h)
        ):
            self.build_homography(width, height)
        return cv2.warpPerspective(rectified, self.M, (out_w, out_h))

    def draw_roi(self, image: np.ndarray) -> np.ndarray:
        out = image.copy()
        if self.src is None:
            return out
        cv2.polylines(out, [self.src.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
        for label, point in zip(("TL", "TR", "BR", "BL"), self.src):
            p = tuple(point.astype(int))
            cv2.circle(out, p, 5, (0, 255, 255), -1)
            cv2.putText(
                out,
                label,
                tuple((point + np.array([5, -5])).astype(int)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )
        return out

    def draw_bev_guides(self, bev: np.ndarray) -> np.ndarray:
        out = bev.copy()
        height, width = out.shape[:2]
        lat_m_per_px = float(self.get_parameter("lateral_m_per_px").value)
        center_x = width / 2.0

        def col_from_y_left(y_left_m: float) -> int:
            return int(round(center_x - y_left_m / lat_m_per_px))

        for y_left, label, color in [
            (0.40, "left +0.40m", (255, 0, 0)),
            (0.00, "center 0.00m", (0, 255, 255)),
            (-0.40, "right -0.40m", (0, 0, 255)),
        ]:
            col = col_from_y_left(y_left)
            if 0 <= col < width:
                cv2.line(out, (col, 0), (col, height - 1), color, 1)
                cv2.putText(out, label, (col + 5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        for row in range(0, height, 20):
            cv2.line(out, (0, row), (width - 1, row), (70, 70, 70), 1)
        return out

    def make_black_mask(self, bev: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        threshold = int(self.get_parameter("black_threshold").value)
        return cv2.inRange(gray, 0, threshold)

    def process_frame(self, frame: np.ndarray) -> None:
        rectified = self.rectify(frame)
        bev = self.make_bev(rectified)
        cv2.imshow("source", frame)
        cv2.imshow("rectified_with_roi", self.draw_roi(rectified))
        cv2.imshow("BEV", self.draw_bev_guides(bev))
        cv2.imshow("BEV black mask", self.make_black_mask(bev))
        cv2.waitKey(1)

    def compressed_cb(self, msg: CompressedImage) -> None:
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("compressed decode failed")
            return
        self.process_frame(frame)

    def image_cb(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process_frame(frame)

    def on_bev_mouse(self, event, x, y, flags, param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        out_w = int(self.get_parameter("bev_width").value)
        out_h = int(self.get_parameter("bev_height").value)
        lat_m_per_px = float(self.get_parameter("lateral_m_per_px").value)
        fwd_m_per_px = float(self.get_parameter("forward_m_per_px").value)
        center_x = out_w / 2.0
        y_left_m = (center_x - x) * lat_m_per_px
        x_forward_rel_m = (out_h - y) * fwd_m_per_px
        print(
            f"[BEV click] pixel=({x}, {y}) | "
            f"y_left≈{y_left_m:+.3f} m | "
            f"x_forward_rel≈{x_forward_rel_m:.3f} m"
        )
        if self.M_inv is not None:
            point = np.array([[[x, y]]], dtype=np.float32)
            source_point = cv2.perspectiveTransform(point, self.M_inv)[0, 0]
            print(f"            rectified_pixel≈({source_point[0]:.1f}, {source_point[1]:.1f})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BevPreviewHomography()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
