#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class SafetyStopDecision:
    should_stop: bool
    reason: str = ''


class SafetySupervisor:
    """Top-level safety policy boundary.

    Perception components update state; this supervisor decides whether the
    planner should be interrupted for red lights, startup light gates, people,
    vehicles, or other safety holds.
    """

    def __init__(self, node):
        self.node = node
        self.next_stop_line_update_sec = 0.0
        self.next_school_zone_update_sec = 0.0
        self.last_stop_line_image_id = None

    def update_perception(self, image: Optional[np.ndarray]):
        if image is None:
            self.last_stop_line_image_id = None
            self.node.stop_line_detector.update(image)
            self.node.traffic_light_detector.update(image)
            self.node.school_zone_detector.update(image)
            self.node.publish_ai_speed_limit()
            return

        now = time.monotonic()
        if self._stop_line_update_due(now, image):
            self.node.stop_line_detector.update(image)
        self.node.traffic_light_detector.update(image)
        if self._period_due(now, 'school_zone_update_period_sec', 'next_school_zone_update_sec'):
            self.node.school_zone_detector.update(image)
        self.node.publish_ai_speed_limit()

    def _period_due(self, now: float, parameter_name: str, next_attr: str) -> bool:
        period = max(float(self.node.get_parameter(parameter_name).value), 0.0)
        next_sec = float(getattr(self, next_attr))
        if period <= 0.0 or now >= next_sec:
            setattr(self, next_attr, now + period)
            return True
        return False

    def _stop_line_update_due(self, now: float, image: np.ndarray) -> bool:
        period = max(float(self.node.get_parameter('stop_line_update_period_sec').value), 0.0)
        image_id = id(image)
        new_image = image_id != self.last_stop_line_image_id
        if period <= 0.0 or new_image or now >= self.next_stop_line_update_sec:
            self.next_stop_line_update_sec = now + period
            self.last_stop_line_image_id = image_id
            return True
        return False

    def detect_safety_stop(self) -> SafetyStopDecision:
        should_stop, reason = self.node._detect_safety_stop()
        return SafetyStopDecision(should_stop=should_stop, reason=reason)

    def startup_light_check_pending(self) -> bool:
        return self.node.traffic_light_detector.startup_check_pending()
