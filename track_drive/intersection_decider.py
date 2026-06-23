#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Optional, Sequence, Tuple


Point = Tuple[float, float]


class IntersectionDecider:
    """Intersection route-decision boundary."""

    def __init__(self, node):
        # 교차로 판단에 필요한 실제 센서/상태/파라미터는 TrackDriverNode가 들고 있다.
        self.node = node

    def route_command(self, cones: Sequence[Point]) -> Optional[Tuple[str, float, float]]:
        # 교차로에서 직진/좌회전/대기 명령을 결정하는 핵심 분기 함수로 위임한다.
        return self.node._intersection_route_command(cones)

    def trigger_ready(self) -> bool:
        # 신호등과 정지선 조건이 교차로 판단을 시작할 만큼 준비됐는지 확인한다.
        return self.node._intersection_trigger_ready()

    def log_text(self) -> str:
        # 현재 교차로 판단 상태를 주행 로그 문자열로 변환한다.
        return self.node._intersection_log_text()
