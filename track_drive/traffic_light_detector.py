#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

import cv2
import numpy as np

from track_drive.package_paths import default_yolo_model_path


Box = Tuple[int, int, int, int]


@dataclass
class TrafficLightDetection:
    box: Box
    score: float
    class_id: int
    valid: bool
    red_present: bool
    red_ratio: float
    green_ratio: float
    yellow_ratio: float


@dataclass
class TrafficLightResult:
    state: str = 'unknown'
    detections: List[TrafficLightDetection] = None
    class_scores: List[float] = None
    raw_shape: str = ''
    debug_image: Optional[np.ndarray] = None

    def __post_init__(self):
        # 데이터클래스 생성 뒤 파생 필드나 기본 상태를 정리한다.
        if self.detections is None:
            self.detections = []
        if self.class_scores is None:
            self.class_scores = []


def declare_traffic_light_parameters(node):
    # 신호등 검출에서 사용하는 ROS 파라미터 기본값을 선언한다.
    node.declare_parameter('camera_topic', '/usb_cam/image_raw/front')
    node.declare_parameter('traffic_light_debug_topic', '/track_drive/traffic_light_debug/image')
    node.declare_parameter('traffic_light_state_topic', '/track_drive/traffic_light_debug/state')
    node.declare_parameter('traffic_light_log_period_sec', 0.5)
    node.declare_parameter('yolo_light_model_path', default_yolo_model_path())
    node.declare_parameter('yolo_light_input_size', 640)
    node.declare_parameter('yolo_light_class_count', 6)
    node.declare_parameter('yolo_dnn_backend', 'auto')
    node.declare_parameter('yolo_dnn_target', 'auto')
    node.declare_parameter('yolo_light_conf_threshold', 0.35)
    node.declare_parameter('yolo_stop_light_conf_threshold', 0.55)
    node.declare_parameter('yolo_left_light_conf_threshold', 0.28)
    node.declare_parameter('yolo_light_class_ids', [0, 1, 2, 3, 4, 5])
    node.declare_parameter('yolo_red_light_class_ids', [4, 5])
    node.declare_parameter('yolo_go_light_class_ids', [1])
    node.declare_parameter('yolo_left_light_class_ids', [2])
    node.declare_parameter('yolo_nms_threshold', 0.45)
    node.declare_parameter('yolo_light_min_box_height_ratio', 0.025)
    node.declare_parameter('yolo_light_min_box_width_ratio', 0.015)
    node.declare_parameter('yolo_light_max_box_height_ratio', 0.65)
    node.declare_parameter('yolo_light_min_box_area_ratio', 0.00012)
    node.declare_parameter('yolo_light_max_box_area_ratio', 0.20)
    node.declare_parameter('yolo_light_max_box_bottom_ratio', 0.98)
    node.declare_parameter('red_light_min_area', 12.0)
    node.declare_parameter('red_light_min_ratio', 0.0012)
    node.declare_parameter('red_light_min_dominance', 1.20)
    node.declare_parameter('red_light_min_circularity', 0.10)


class YoloTrafficLightDetector:
    """Standalone YOLO traffic-light detector for debug/test nodes."""

    def __init__(self, node):
        # YoloTrafficLightDetector 객체를 초기화하고 필요한 파라미터와 내부 상태를 준비한다.
        self.node = node
        self.net = None
        self.net_path = ''
        self.ort_sessions: Dict[Tuple[str, str, str], Tuple[object, str, str]] = {}
        self.class_scores: List[float] = []
        self.raw_shape = ''

    def detect(self, image: Optional[np.ndarray]) -> TrafficLightResult:
        # 입력 데이터에서 detect 조건을 감지한다.
        if image is None or image.size == 0:
            return TrafficLightResult()

        detections = self._run_yolo(image)
        allowed_ids = self._int_set_parameter('yolo_light_class_ids')
        red_ids = self._int_set_parameter('yolo_red_light_class_ids')
        go_ids = self._int_set_parameter('yolo_go_light_class_ids')
        left_ids = self._int_set_parameter('yolo_left_light_class_ids')
        stop_threshold = max(float(self._param('yolo_stop_light_conf_threshold', 0.55)), 0.0)
        left_threshold = max(float(self._param('yolo_left_light_conf_threshold', 0.28)), 0.0)

        light_debug: List[TrafficLightDetection] = []
        red_seen = False
        go_seen = False
        left_seen = False
        yellow_seen = False
        best_label = 'unknown'
        best_score = 0.0

        for box, score, class_id in detections:
            if not self._class_id_allowed(class_id, allowed_ids):
                continue
            valid = self._valid_light_detection(image, box)
            red_color, red_ratio, green_ratio, yellow_ratio = self._red_light_box_metrics(image, box)
            class_red = valid and score >= stop_threshold and self._class_id_allowed(class_id, red_ids)
            class_go = valid and self._class_id_allowed(class_id, go_ids)
            class_left = valid and score >= left_threshold and self._class_id_allowed(class_id, left_ids)
            red_present = bool(class_red or (valid and red_color and self._class_id_allowed(class_id, red_ids)))
            yellow_present = valid and int(class_id) == 5

            red_seen = red_seen or red_present
            go_seen = go_seen or class_go
            left_seen = left_seen or class_left
            yellow_seen = yellow_seen or yellow_present
            if valid and score > best_score:
                best_score = float(score)
                best_label = self._light_class_name(class_id)

            light_debug.append(TrafficLightDetection(
                box=box,
                score=float(score),
                class_id=int(class_id),
                valid=bool(valid),
                red_present=bool(red_present),
                red_ratio=float(red_ratio),
                green_ratio=float(green_ratio),
                yellow_ratio=float(yellow_ratio),
            ))

        if not left_seen:
            left_seen = self._score_fallback(left_ids, left_threshold)

        state = self._state_from_flags(red_seen, go_seen, left_seen, yellow_seen, best_label)
        debug = self._draw_debug(image, state, light_debug)
        return TrafficLightResult(
            state=state,
            detections=light_debug,
            class_scores=list(self.class_scores),
            raw_shape=self.raw_shape,
            debug_image=debug,
        )

    def _run_yolo(self, image: np.ndarray):
        # 신호등 검출 처리 파이프라인을 실행하고 결과를 반환한다.
        model_path = Path(str(self._param('yolo_light_model_path', default_yolo_model_path()))).expanduser()
        if not model_path.exists():
            self._warn_once(f'YOLO light model not found: {model_path}')
            return []

        input_size = max(int(self._param('yolo_light_input_size', 640)), 32)
        padded, scale, pad_x, pad_y = self._letterbox_image(image, input_size)
        blob = cv2.dnn.blobFromImage(
            padded,
            scalefactor=1.0 / 255.0,
            size=(input_size, input_size),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )
        output = self._run_ort(model_path, blob)
        if output is None:
            net = self._get_net(model_path)
            if net is None:
                return []
            try:
                net.setInput(blob)
                output = net.forward()
            except Exception as exc:
                self._warn_once(f'YOLO light inference failed: {exc}')
                return []

        class_count = int(self._param('yolo_light_class_count', 6))
        self._cache_class_scores(output, class_count)
        return self._decode_yolo_output(
            output,
            image.shape[:2],
            scale,
            pad_x,
            pad_y,
            float(self._param('yolo_light_conf_threshold', 0.35)),
            float(self._param('yolo_nms_threshold', 0.45)),
            class_count,
        )

    def _run_ort(self, model_path: Path, blob: np.ndarray):
        # 신호등 검출 처리 파이프라인을 실행하고 결과를 반환한다.
        session_info = self._get_ort_session(model_path)
        if session_info is None:
            return None
        session, input_name, provider_text = session_info
        try:
            outputs = session.run(None, {input_name: blob.astype(np.float32, copy=False)})
            if not outputs:
                return None
            return outputs[0]
        except Exception as exc:
            self._warn_once(f'YOLO light ONNX Runtime failed ({provider_text}); falling back to OpenCV DNN: {exc}')
            return None

    def _get_ort_session(self, model_path: Path):
        # get ort session 값을 현재 상태에서 계산하거나 조회한다.
        backend_name = str(self._param('yolo_dnn_backend', 'auto')).strip().lower()
        target_name = str(self._param('yolo_dnn_target', 'auto')).strip().lower()
        if backend_name not in ('auto', 'cuda', 'onnxruntime', 'ort') and target_name not in ('cuda', 'cuda_fp16', 'fp16'):
            return None

        key = (str(model_path), backend_name, target_name)
        if key in self.ort_sessions:
            return self.ort_sessions[key]

        try:
            import onnxruntime as ort
        except Exception as exc:
            self._warn_once(f'ONNX Runtime is not available; using OpenCV DNN: {exc}')
            return None

        available = set(ort.get_available_providers())
        providers = []
        if 'CUDAExecutionProvider' in available and backend_name in ('auto', 'cuda', 'onnxruntime', 'ort'):
            providers.append('CUDAExecutionProvider')
        if 'CPUExecutionProvider' in available and backend_name in ('auto', 'onnxruntime', 'ort'):
            providers.append('CPUExecutionProvider')
        if 'CPUExecutionProvider' in available and not providers and backend_name == 'cuda':
            self._warn_once('ONNX Runtime CUDA provider is not available; using CPU provider')
            providers.append('CPUExecutionProvider')
        if not providers:
            return None

        try:
            session = ort.InferenceSession(str(model_path), providers=providers)
        except Exception as exc:
            self._warn_once(f'YOLO light ONNX Runtime load failed; using OpenCV DNN: {exc}')
            return None

        active = session.get_providers()
        input_name = session.get_inputs()[0].name
        provider_text = '+'.join(active)
        info = (session, input_name, provider_text)
        self.ort_sessions[key] = info
        self.node.get_logger().info(f'YOLO light model loaded: {model_path} | ort={provider_text}')
        return info

    def _get_net(self, model_path: Path):
        # get net 값을 현재 상태에서 계산하거나 조회한다.
        path_text = str(model_path)
        if self.net is not None and self.net_path == path_text:
            return self.net
        try:
            net = cv2.dnn.readNetFromONNX(path_text)
            backend, target = self._configure_dnn_net(net)
        except Exception as exc:
            self._warn_once(f'YOLO light model load failed: {model_path} | {exc}')
            return None
        self.net = net
        self.net_path = path_text
        self.node.get_logger().info(f'YOLO light model loaded: {model_path} | dnn={backend}/{target}')
        return net

    def _configure_dnn_net(self, net):
        # 신호등 검출의 configure dnn net 로직을 수행한다.
        backend_name = str(self._param('yolo_dnn_backend', 'auto')).strip().lower()
        target_name = str(self._param('yolo_dnn_target', 'auto')).strip().lower()
        cuda_requested = backend_name == 'cuda' or target_name in ('cuda', 'cuda_fp16', 'fp16')
        cuda_allowed = backend_name in ('auto', 'cuda') and target_name in ('auto', 'cuda', 'cuda_fp16', 'fp16')
        if cuda_allowed and self._opencv_dnn_cuda_available():
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            if target_name in ('cuda_fp16', 'fp16') and hasattr(cv2.dnn, 'DNN_TARGET_CUDA_FP16'):
                net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
                return 'cuda', 'cuda_fp16'
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            return 'cuda', 'cuda'

        if cuda_requested:
            self._warn_once('OpenCV DNN CUDA is not available; using CPU backend')
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return 'opencv', 'cpu'

    @staticmethod
    def _opencv_dnn_cuda_available() -> bool:
        # 신호등 검출의 opencv dnn cuda available 로직을 수행한다.
        if not hasattr(cv2.dnn, 'DNN_BACKEND_CUDA') or not hasattr(cv2.dnn, 'DNN_TARGET_CUDA'):
            return False
        try:
            return bool(hasattr(cv2, 'cuda') and cv2.cuda.getCudaEnabledDeviceCount() > 0)
        except Exception:
            return False

    def _cache_class_scores(self, output, class_count: int):
        # 신호등 검출의 캐시 클래스 scores 로직을 수행한다.
        self.class_scores = []
        self.raw_shape = ''
        if class_count <= 0:
            return
        try:
            predictions = np.asarray(output)
            self.raw_shape = 'x'.join(str(dim) for dim in predictions.shape)
            if predictions.ndim == 3:
                predictions = predictions[0]
            if predictions.ndim != 2:
                return
            if predictions.shape[0] < predictions.shape[1] and predictions.shape[0] <= 64:
                predictions = predictions.T

            if predictions.shape[1] == 4 + class_count:
                class_scores = predictions[:, 4:4 + class_count]
            elif predictions.shape[1] >= 5 + class_count:
                obj_conf = predictions[:, 4:5]
                class_scores = predictions[:, 5:5 + class_count]
                if np.nanmax(obj_conf) <= 1.0 and np.nanmax(class_scores) <= 1.0:
                    class_scores = obj_conf * class_scores
            else:
                return

            self.class_scores = [
                float(np.nanmax(class_scores[:, class_id]))
                for class_id in range(class_count)
            ]
        except Exception as exc:
            self._warn_once(f'YOLO light score debug failed: {exc}')

    @staticmethod
    def _letterbox_image(image: np.ndarray, input_size: int):
        # 신호등 검출의 letterbox 이미지 로직을 수행한다.
        height, width = image.shape[:2]
        scale = min(input_size / max(width, 1), input_size / max(height, 1))
        new_width = max(int(round(width * scale)), 1)
        new_height = max(int(round(height * scale)), 1)
        resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        padded = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
        pad_x = (input_size - new_width) // 2
        pad_y = (input_size - new_height) // 2
        padded[pad_y:pad_y + new_height, pad_x:pad_x + new_width] = resized
        return padded, scale, pad_x, pad_y

    @staticmethod
    def _decode_yolo_output(
        output,
        image_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
        conf_threshold: float,
        nms_threshold: float,
        class_count: Optional[int] = None,
    ):
        # 신호등 검출의 decode YOLO output 로직을 수행한다.
        predictions = np.asarray(output)
        if predictions.ndim == 3:
            predictions = predictions[0]
        if predictions.ndim != 2 or predictions.shape[1] < 5:
            return []
        if predictions.shape[0] < predictions.shape[1] and predictions.shape[0] <= 32:
            predictions = predictions.T

        image_height, image_width = image_shape
        boxes = []
        scores = []
        class_ids = []
        for row in predictions:
            if len(row) < 5:
                continue

            obj_conf = float(row[4])
            class_id = 0
            if class_count is not None and len(row) == 4 + class_count:
                class_scores = row[4:4 + class_count]
                class_id = int(np.argmax(class_scores))
                score = float(class_scores[class_id])
            elif class_count is not None and len(row) >= 5 + class_count:
                class_scores = row[5:5 + class_count]
                class_id = int(np.argmax(class_scores))
                class_conf = float(class_scores[class_id])
                score = obj_conf * class_conf if obj_conf <= 1.0 and class_conf <= 1.0 else max(obj_conf, class_conf)
            elif len(row) == 5:
                score = obj_conf
            elif len(row) == 6 and float(row[5]) > 1.0 and obj_conf <= 1.0:
                score = obj_conf
                class_id = int(row[5])
            else:
                class_scores = row[5:]
                class_id = int(np.argmax(class_scores))
                class_conf = float(class_scores[class_id])
                score = obj_conf * class_conf if obj_conf <= 1.0 and class_conf <= 1.0 else max(obj_conf, class_conf)

            if score < conf_threshold:
                continue

            x, y, width, height = [float(v) for v in row[:4]]
            if len(row) == 6 and float(row[5]) > 1.0 and row[2] > row[0] and row[3] > row[1]:
                x0 = (x - pad_x) / max(scale, 1e-6)
                y0 = (y - pad_y) / max(scale, 1e-6)
                x1 = (width - pad_x) / max(scale, 1e-6)
                y1 = (height - pad_y) / max(scale, 1e-6)
            else:
                x0 = (x - 0.5 * width - pad_x) / max(scale, 1e-6)
                y0 = (y - 0.5 * height - pad_y) / max(scale, 1e-6)
                x1 = (x + 0.5 * width - pad_x) / max(scale, 1e-6)
                y1 = (y + 0.5 * height - pad_y) / max(scale, 1e-6)

            x0 = int(np.clip(x0, 0, image_width - 1))
            y0 = int(np.clip(y0, 0, image_height - 1))
            x1 = int(np.clip(x1, 0, image_width - 1))
            y1 = int(np.clip(y1, 0, image_height - 1))
            box_width = x1 - x0
            box_height = y1 - y0
            if box_width <= 2 or box_height <= 2:
                continue

            boxes.append([x0, y0, box_width, box_height])
            scores.append(float(score))
            class_ids.append(int(class_id))

        if not boxes:
            return []
        indices = cv2.dnn.NMSBoxes(boxes, scores, conf_threshold, nms_threshold)
        if len(indices) == 0:
            return []
        return [
            ((boxes[int(idx)][0], boxes[int(idx)][1],
              boxes[int(idx)][0] + boxes[int(idx)][2],
              boxes[int(idx)][1] + boxes[int(idx)][3]),
             scores[int(idx)], class_ids[int(idx)])
            for idx in np.asarray(indices).reshape(-1)
        ]

    def _valid_light_detection(self, image: np.ndarray, box: Box) -> bool:
        # valid 신호등 detection 후보가 유효한 조건을 만족하는지 검사한다.
        image_height, image_width = image.shape[:2]
        x0, y0, x1, y1 = box
        box_width = max(x1 - x0, 0)
        box_height = max(y1 - y0, 0)
        image_area = float(max(image_width * image_height, 1))
        box_area_ratio = (box_width * box_height) / image_area
        height_ratio = box_height / float(max(image_height, 1))
        min_height_ratio = float(np.clip(self._param('yolo_light_min_box_height_ratio', 0.025), 0.0, 1.0))
        min_width_ratio = float(np.clip(self._param('yolo_light_min_box_width_ratio', 0.015), 0.0, 1.0))
        max_height_ratio = float(np.clip(self._param('yolo_light_max_box_height_ratio', 0.65), min_height_ratio, 1.0))
        min_area_ratio = float(np.clip(self._param('yolo_light_min_box_area_ratio', 0.00012), 0.0, 1.0))
        max_area_ratio = float(np.clip(self._param('yolo_light_max_box_area_ratio', 0.20), min_area_ratio, 1.0))
        max_bottom_ratio = float(np.clip(self._param('yolo_light_max_box_bottom_ratio', 0.98), 0.05, 1.0))
        return (
            min_height_ratio <= height_ratio <= max_height_ratio
            and box_width >= min_width_ratio * max(image_width, 1)
            and min_area_ratio <= box_area_ratio <= max_area_ratio
            and y1 <= max_bottom_ratio * max(image_height, 1)
        )

    def _red_light_box_metrics(self, image: np.ndarray, box: Box) -> Tuple[bool, float, float, float]:
        # 신호등 상태 중 빨간불 신호등 박스 metrics 조건을 판단한다.
        x0, y0, x1, y1 = box
        pad = 4
        x0 = max(x0 - pad, 0)
        y0 = max(y0 - pad, 0)
        x1 = min(x1 + pad, image.shape[1] - 1)
        y1 = min(y1 + pad, image.shape[0] - 1)
        crop = image[y0:y1, x0:x1]
        if crop.size == 0:
            return False, 0.0, 0.0, 0.0
        return self._red_light_color_metrics(crop)

    def _red_light_color_metrics(self, image: np.ndarray) -> Tuple[bool, float, float, float]:
        # 신호등 상태 중 빨간불 신호등 color metrics 조건을 판단한다.
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv, np.array([0, 90, 80], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8))
        red_mask |= cv2.inRange(hsv, np.array([170, 90, 80], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8))
        green_mask = cv2.inRange(hsv, np.array([38, 70, 70], dtype=np.uint8), np.array([95, 255, 255], dtype=np.uint8))
        yellow_mask = cv2.inRange(hsv, np.array([18, 80, 80], dtype=np.uint8), np.array([36, 255, 255], dtype=np.uint8))
        kernel = np.ones((3, 3), np.uint8)
        red_mask = cv2.morphologyEx(cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)

        image_area = float(max(width * height, 1))
        red_ratio = float(np.count_nonzero(red_mask)) / image_area
        green_ratio = float(np.count_nonzero(green_mask)) / image_area
        yellow_ratio = float(np.count_nonzero(yellow_mask)) / image_area
        if red_ratio < float(self._param('red_light_min_ratio', 0.0012)):
            return False, red_ratio, green_ratio, yellow_ratio
        dominance = max(float(self._param('red_light_min_dominance', 1.20)), 1.0)
        if red_ratio < dominance * max(green_ratio, yellow_ratio):
            return False, red_ratio, green_ratio, yellow_ratio
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = float(self._param('red_light_min_area', 12.0))
        min_circularity = float(self._param('red_light_min_circularity', 0.10))
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter < 1e-3:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity >= min_circularity:
                return True, red_ratio, green_ratio, yellow_ratio
        return False, red_ratio, green_ratio, yellow_ratio

    def _score_fallback(self, class_ids: Optional[Set[int]], threshold: float) -> bool:
        # score fallback 후보의 점수나 보조 판정값을 계산한다.
        return any(
            0 <= int(class_id) < len(self.class_scores)
            and float(self.class_scores[int(class_id)]) >= threshold
            for class_id in (class_ids or set())
        )

    def _state_from_flags(self, red: bool, go: bool, left: bool, yellow: bool, best_label: str) -> str:
        # 신호등 검출의 상태 from flags 로직을 수행한다.
        if go and not red:
            return 'green'
        if left:
            return 'left'
        if red:
            return 'red'
        if yellow:
            return 'yellow'
        if best_label != 'unknown':
            return best_label
        return 'none'

    def _draw_debug(self, image: np.ndarray, state: str, detections: Sequence[TrafficLightDetection]) -> np.ndarray:
        # draw 디버그 정보를 디버그 이미지 위에 그린다.
        debug = image.copy()
        color = {
            'green': (0, 220, 0),
            'left': (255, 180, 0),
            'red': (0, 0, 255),
            'yellow': (0, 220, 255),
            'none': (180, 180, 180),
        }.get(state, (0, 180, 255))
        cv2.rectangle(debug, (6, 6), (min(debug.shape[1] - 6, 820), 112), (0, 0, 0), thickness=-1)
        cv2.putText(debug, f'TRAFFIC LIGHT: {state}', (14, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
        cv2.putText(debug, f'det={len(detections)} raw={self.raw_shape}', (14, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (230, 230, 230), 1, cv2.LINE_AA)
        if self.class_scores:
            scores = ' '.join(f'{idx}:{self._light_class_name(idx)}={score:.2f}' for idx, score in enumerate(self.class_scores[:6]))
            cv2.putText(debug, scores, (14, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)

        for detection in detections:
            x0, y0, x1, y1 = detection.box
            if not detection.valid:
                box_color = (130, 130, 130)
                label = 'reject'
            elif detection.red_present:
                box_color = (0, 0, 255)
                label = 'red'
            elif self._class_id_allowed(detection.class_id, self._int_set_parameter('yolo_go_light_class_ids')):
                box_color = (0, 220, 0)
                label = 'green'
            elif self._class_id_allowed(detection.class_id, self._int_set_parameter('yolo_left_light_class_ids')):
                box_color = (255, 180, 0)
                label = 'left'
            else:
                box_color = (0, 220, 255)
                label = self._light_class_name(detection.class_id)
            cv2.rectangle(debug, (x0, y0), (x1, y1), box_color, 2)
            text = (
                f'{label}:id{detection.class_id}:{self._light_class_name(detection.class_id)} '
                f's={detection.score:.2f} r={detection.red_ratio:.3f} '
                f'g={detection.green_ratio:.3f} y={detection.yellow_ratio:.3f}'
            )
            cv2.putText(debug, text, (x0, max(y0 - 8, 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, box_color, 1, cv2.LINE_AA)
        return debug

    def _int_set_parameter(self, name: str) -> Optional[Set[int]]:
        # 리스트형 ROS 파라미터를 정수 집합으로 변환한다.
        value = self._param(name, None)
        if value is None:
            return None
        if isinstance(value, str):
            text = value.strip()
            if not text or text.lower() in ('all', 'any', '*'):
                return None
            value = text.strip('[]()').replace(';', ',').split(',')
        if isinstance(value, np.ndarray):
            value = value.tolist()
        if isinstance(value, (list, tuple)):
            ids = {int(item) for item in value if str(item).strip()}
            return ids or None
        return {int(value)}

    @staticmethod
    def _class_id_allowed(class_id: int, allowed_class_ids: Optional[Set[int]]) -> bool:
        # 검출 클래스 ID가 허용 목록에 포함되는지 확인한다.
        return allowed_class_ids is None or int(class_id) in allowed_class_ids

    @staticmethod
    def _light_class_name(class_id: int) -> str:
        # 신호등 검출의 신호등 클래스 name 로직을 수행한다.
        names = {
            0: 'cone',
            1: 'green',
            2: 'left',
            3: 'pedestrian',
            4: 'red',
            5: 'yellow',
        }
        return names.get(int(class_id), f'class{int(class_id)}')

    def _param(self, name: str, default):
        # ROS 파라미터 값을 읽고 없으면 기본값을 사용한다.
        try:
            return self.node.get_parameter(name).value
        except Exception:
            return default

    def _warn_once(self, reason: str):
        # 신호등 검출의 warn once 로직을 수행한다.
        now_sec = self.node.get_clock().now().nanoseconds // 1_000_000_000
        if getattr(self, '_last_warn_sec', None) == now_sec:
            return
        self._last_warn_sec = now_sec
        self.node.get_logger().warn(reason)


class TrafficLightDetector:
    """Traffic-light perception boundary for TrackDriverNode.

    This first split keeps the existing detector implementation on the node,
    but routes light-related calls through one component so the policy code
    no longer reaches into every light helper directly.
    """

    def __init__(self, node):
        # TrafficLightDetector 객체를 초기화하고 필요한 파라미터와 내부 상태를 준비한다.
        self.node = node

    def update(self, image: Optional[np.ndarray]):
        # 최신 입력을 기준으로 update 관련 캐시와 상태를 갱신한다.
        if bool(self.node.get_parameter('yolo_safety_enabled').value):
            self.node._update_yolo_safety_cache(image)

    def red_stop_requested(self, image: Optional[np.ndarray]) -> bool:
        # 신호등 검출의 빨간불 정지 requested 로직을 수행한다.
        return self.node._detect_red_light(image)

    def startup_check_pending(self) -> bool:
        # 신호등 검출의 startup check pending 로직을 수행한다.
        return self.node._startup_light_check_pending()

    def startup_allows_stop_without_line(self) -> bool:
        # 신호등 검출의 startup allows 정지 without line 로직을 수행한다.
        return self.node._startup_light_allows_stop_without_line()

    def visible(self) -> bool:
        # 신호등 검출의 가시 상태 로직을 수행한다.
        return self.node._traffic_light_visible()

    def publish_cached_debug_image(self):
        # publish cached 디버그 이미지 결과를 ROS 토픽이나 디버그 출력으로 발행한다.
        self.node._publish_cached_light_debug_image()
