#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 교차로 경로 선택 판단을 메인 주행 노드와 분리하기 위한 얇은 래퍼 모듈이다.
# 실제 판단 상태는 TrackDriverNode가 보유하고, 이 파일은 좌회전/직진/대기 명령 호출 경계를 담당한다.
from typing import Optional, Sequence, Tuple


Point = Tuple[float, float]


# 초기 라바콘/교차로 특수 구간을 별도 객체로 분리해 메인 노드의 제어 흐름을 단순하게 유지한다.
# 교차로에서 좌회전/직진 판단 결과를 간단한 명령 문자열로 관리하는 보조 클래스이다.
class IntersectionDecider:
    """메인 주행 노드의 교차로 판단 함수를 외부 모듈 경계로 제공한다."""

    def __init__(self, node):
        # 교차로 판단에 필요한 실제 센서/상태/파라미터는 TrackDriverNode가 들고 있다.
        self.node = node

    # 현재 제출 버전에서는 대부분의 교차로 판단을 TrackDriverNode가 수행하므로 기본값은 명령 없음이다.
    # 현재 교차로에서 외부로 내보낼 경로 명령을 반환한다.
    def route_command(self, cones: Sequence[Point]) -> Optional[Tuple[str, float, float]]:
        # 교차로에서 직진/좌회전/대기 명령을 결정하는 핵심 분기 함수로 위임한다.
        return self.node._intersection_route_command(cones)

    # 경로 명령이 존재할 때만 교차로 동작을 시작할 수 있게 한다.
    def trigger_ready(self) -> bool:
        # 신호등과 정지선 조건이 교차로 판단을 시작할 만큼 준비됐는지 확인한다.
        return self.node._intersection_trigger_ready()

    def log_text(self) -> str:
        # 현재 교차로 판단 상태를 주행 로그 문자열로 변환한다.
        return self.node._intersection_log_text()
