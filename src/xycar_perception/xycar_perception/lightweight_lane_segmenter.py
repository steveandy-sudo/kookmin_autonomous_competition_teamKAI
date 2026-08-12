from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from xycar_perception.lightweight_lane_model import IMAGE_NET_MEAN, IMAGE_NET_STD


class LightweightLaneSegmenter:
    """TorchScript semantic lane wrapper returning source-resolution masks."""

    def __init__(
        self,
        model_path: str,
        *,
        device: str = "cpu",
        input_width: int = 256,
        input_height: int = 144,
        cpu_threads: int = 1,
    ) -> None:
        path = Path(model_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(
                f"semantic lane model does not exist: {path}"
            )
        try:
            import torch
        except ImportError as exc:
            raise RuntimeError("PyTorch is required for semantic lane inference") from exc

        requested_device = str(device).lower()
        if requested_device == "cuda" and not torch.cuda.is_available():
            requested_device = "cpu"
        if requested_device == "cpu" and int(cpu_threads) > 0:
            torch.set_num_threads(int(cpu_threads))

        self.torch = torch
        self.model_path = path
        self.device = torch.device(requested_device)
        self.input_width = int(input_width)
        self.input_height = int(input_height)
        self.cpu_threads = int(cpu_threads)
        self.model = torch.jit.load(str(path), map_location=self.device)
        self.model.eval()
        self.mean = torch.tensor(IMAGE_NET_MEAN, dtype=torch.float32).view(1, 3, 1, 1)
        self.std = torch.tensor(IMAGE_NET_STD, dtype=torch.float32).view(1, 3, 1, 1)
        self.mean = self.mean.to(self.device)
        self.std = self.std.to(self.device)

        warmup = np.zeros(
            (self.input_height, self.input_width, 3), dtype=np.uint8
        )
        self._predict_classes(warmup)

    def _predict_classes(self, image_bgr: np.ndarray) -> np.ndarray:
        resized = cv2.resize(
            image_bgr,
            (self.input_width, self.input_height),
            interpolation=cv2.INTER_AREA,
        )
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = self.torch.from_numpy(
            np.ascontiguousarray(rgb.transpose(2, 0, 1))
        ).float()
        tensor = tensor.unsqueeze(0).to(self.device).div_(255.0)
        tensor = (tensor - self.mean) / self.std
        with self.torch.inference_mode():
            logits = self.model(tensor)
            classes = logits.argmax(dim=1)[0].to("cpu", dtype=self.torch.uint8)
        return classes.numpy()

    def predict(
        self,
        image_bgr: np.ndarray,
        *,
        render_debug: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        if image_bgr is None or image_bgr.size == 0:
            raise ValueError("semantic lane input image is empty")
        white, yellow, debug = self.predict_low_resolution(
            image_bgr, render_debug=render_debug
        )
        height, width = image_bgr.shape[:2]
        white = cv2.resize(
            white,
            (width, height),
            interpolation=cv2.INTER_NEAREST,
        )
        yellow = cv2.resize(
            yellow,
            (width, height),
            interpolation=cv2.INTER_NEAREST,
        )
        if debug is not None:
            debug = cv2.resize(debug, (width, height), interpolation=cv2.INTER_LINEAR)
        return white, yellow, debug

    def predict_low_resolution(
        self,
        image_bgr: np.ndarray,
        *,
        render_debug: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
        """Return masks at network resolution for a direct low-cost BEV warp."""
        if image_bgr is None or image_bgr.size == 0:
            raise ValueError("semantic lane input image is empty")
        classes = self._predict_classes(image_bgr)
        white = np.where(classes == 1, 255, 0).astype(np.uint8)
        yellow = np.where(classes == 2, 255, 0).astype(np.uint8)
        debug = None
        if render_debug:
            debug = cv2.resize(
                image_bgr,
                (self.input_width, self.input_height),
                interpolation=cv2.INTER_AREA,
            )
            overlay = np.zeros_like(debug)
            overlay[white > 0] = (255, 255, 255)
            overlay[yellow > 0] = (0, 255, 255)
            debug = cv2.addWeighted(debug, 0.70, overlay, 0.30, 0.0)
        return white, yellow, debug
