#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenCV 디버그 창과 터미널 요약 로그를 만드는 도구입니다."""

from __future__ import annotations

import os
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np

from . import config
from .fsm import MissionState

try:
    from PIL import Image, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except Exception:
    Image = None
    ImageDraw = None
    ImageFont = None
    PIL_AVAILABLE = False


STATE_COLORS: Dict[str, Tuple[int, int, int]] = {
    "WAIT_START": (80, 180, 255),
    "CONE": (0, 180, 255),
    "LANE": (70, 220, 70),
    "DECISION": (255, 200, 80),
    "TURN_LEFT": (255, 120, 220),
    "FINISH": (180, 180, 180),
}


class TextRenderer:
    """PIL이 있으면 한글을 그려 주고, 없으면 OpenCV 기본 글꼴로 대체합니다."""

    def __init__(self) -> None:
        """시스템에 있는 한글 폰트를 찾아 renderer를 준비합니다."""
        self.use_pil = PIL_AVAILABLE
        self.font_big = None
        self.font_mid = None
        self.font_small = None
        if self.use_pil:
            font_path = self._find_korean_font()
            try:
                self.font_big = ImageFont.truetype(font_path, 58) if font_path else ImageFont.load_default()
                self.font_mid = ImageFont.truetype(font_path, 24) if font_path else ImageFont.load_default()
                self.font_small = ImageFont.truetype(font_path, 18) if font_path else ImageFont.load_default()
            except Exception:
                self.use_pil = False

    def _find_korean_font(self) -> Optional[str]:
        """Ubuntu에서 흔한 한글 폰트 경로를 순서대로 찾습니다."""
        candidates = [
            "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ]
        for path in candidates:
            if os.path.exists(path):
                return path
        return None

    def draw_text(
        self,
        img: np.ndarray,
        text: str,
        org: Tuple[int, int],
        size: str = "mid",
        color: Tuple[int, int, int] = (255, 255, 255),
    ) -> np.ndarray:
    
        """이미지 위에 텍스트를 그립니다. (OpenCV 전용, 영어 텍스트라 PIL 불필요 → 가볍고 빠름)"""
        scale = {"big": 1.4, "mid": 0.7, "small": 0.5}.get(size, 0.7)
        thick = {"big": 3, "mid": 2, "small": 1}.get(size, 2)
        # PIL은 org가 글자 좌상단, cv2.putText는 좌하단(baseline)이라 글자높이만큼 내려서 맞춤
        baseline_org = (org[0], org[1] + int(28 * scale))
        safe_text = text.encode("ascii", "ignore").decode("ascii") or text[:20]
        cv2.putText(img, safe_text, baseline_org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)
        return img


def _clip_text(text: str, limit: int = 78) -> str:
    """너무 긴 문장을 디버그 창 너비에 맞게 잘라 냅니다."""
    if len(text) <= limit:
        return text
    return text[: limit - 3] + "..."


class DebugOverlay:
    """FSM 상태를 크게 보여 주는 OpenCV 디버그 화면을 만듭니다."""

    def __init__(self) -> None:
        """텍스트 renderer를 초기화합니다."""
        self.renderer = TextRenderer()
        self.last_key = -1

    def show(
        self,
        image_bgr: Optional[np.ndarray],
        state: MissionState,
        guidance: str,
        interrupt_text: str,
        lane_cmd: Tuple[float, float],
        base_cmd: Tuple[float, float],
        final_cmd: Tuple[float, float],
        base_source: str,
        world,
        sensor_health,
        lane_debug: Dict[str, object],
        cone_debug: Dict[str, object],
        arbitration_debug: Dict[str, object],
        elapsed_in_state: float,
        drive_mode: str,
    ) -> int:
        """한 프레임의 디버그 화면을 그리고 눌린 키 코드를 반환합니다."""
        if not config.SHOW_DEBUG_WINDOW:
            return -1

        canvas = self._make_canvas(image_bgr)
        color = STATE_COLORS.get(state.value, (255, 255, 255))

        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 112), (25, 25, 25), -1)
        canvas = self.renderer.draw_text(canvas, state.value, (24, 18), "big", color)
        canvas = self.renderer.draw_text(
            canvas,
            f"mode={drive_mode} | lap={world.lap}/{config.TOTAL_LAPS} | route={world.route_text()} | state_time={elapsed_in_state:.1f}s",
            (28, 86),
            "mid",
            (230, 230, 230),
        )

        # 가벼운 디버그: state(큰글씨) + mode/lap/route만 표시 (나머지 제거로 control_loop 부하 감소)
        cv2.imshow(config.DEBUG_WINDOW_NAME, canvas)
        self.last_key = cv2.waitKey(1)
        return self.last_key

    def _make_canvas(self, image_bgr: Optional[np.ndarray]) -> np.ndarray:
        """카메라 영상 위에 반투명 검은 패널을 올린 canvas를 만듭니다."""
        w = config.DEBUG_WINDOW_WIDTH
        h = config.DEBUG_WINDOW_HEIGHT
        if image_bgr is None:
            base = np.zeros((h, w, 3), dtype=np.uint8)
        else:
            resized = cv2.resize(image_bgr, (w, h), interpolation=cv2.INTER_LINEAR)
            dark = np.zeros_like(resized)
            base = cv2.addWeighted(resized, 0.40, dark, 0.60, 0.0)
        return base

    

def format_transition_log(before: str, after: str, reason: str, lap: int, route: str) -> str:
    """상태 전환 순간에만 찍을 굵은 로그 문자열을 만듭니다."""
    return (
        "\n================ STATE TRANSITION ================\n"
        f"  {before} -> {after}\n"
        f"  reason: {reason}\n"
        f"  lap={lap}, route={route}\n"
    )
