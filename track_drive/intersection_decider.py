#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional, Sequence, Tuple


Point = Tuple[float, float]


class IntersectionDecider:
    """Intersection route-decision boundary.

    Route policy such as "left cone absent means left turn, present means
    straight" belongs here, separate from traffic-light color detection.
    """

    def __init__(self, node):
        self.node = node

    def route_command(self, cones: Sequence[Point]) -> Optional[Tuple[str, float, float]]:
        return self.node._intersection_route_command(cones)

    def trigger_ready(self) -> bool:
        return self.node._intersection_trigger_ready()

    def log_text(self) -> str:
        return self.node._intersection_log_text()
