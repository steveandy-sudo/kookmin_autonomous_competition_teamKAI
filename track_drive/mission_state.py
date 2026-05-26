"""미션 상태머신 스켈레톤."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Dict


class MissionState(Enum):
    WAIT_TRAFFIC_LIGHT = auto()
    CONE_DRIVE = auto()
    LANE_DRIVE = auto()
    PEDESTRIAN_AVOID = auto()
    VEHICLE_AVOID = auto()
    SCHOOL_ZONE = auto()
    LEFT_TURN_DECISION = auto()
    SHORTCUT = auto()
    FINISH = auto()


@dataclass
class MissionStateMachine:
    """대회 미션 순서를 관리하는 단순 상태머신."""

    state: MissionState = MissionState.WAIT_TRAFFIC_LIGHT

    def update(self, perception: Dict[str, Any], obstacles: Dict[str, Any]) -> MissionState:
        """센서 결과를 바탕으로 상태를 갱신한다.

        NOTE: 실제 전이 조건은 대회 규칙/튜닝에 맞춰 확장해야 한다.
        """
        if self.state == MissionState.WAIT_TRAFFIC_LIGHT:
            # TODO: 신호등 조건(초록불 유지 시간 등) 강화
            if perception.get('traffic_light_green', False):
                self.state = MissionState.CONE_DRIVE

        elif self.state == MissionState.CONE_DRIVE:
            # TODO: 라바콘 구간 종료 조건 추가
            self.state = MissionState.LANE_DRIVE

        elif self.state == MissionState.LANE_DRIVE:
            # TODO: 장애물/표지 인식 기반 세부 분기 구현
            if obstacles.get('pedestrian_detected', False):
                self.state = MissionState.PEDESTRIAN_AVOID
            elif obstacles.get('vehicle_detected', False):
                self.state = MissionState.VEHICLE_AVOID
            elif perception.get('school_zone_detected', False):
                self.state = MissionState.SCHOOL_ZONE

        elif self.state == MissionState.PEDESTRIAN_AVOID:
            # TODO: 보행자 회피 완료 조건 구현
            self.state = MissionState.LANE_DRIVE

        elif self.state == MissionState.VEHICLE_AVOID:
            # TODO: 차량 회피 완료 조건 구현
            self.state = MissionState.LANE_DRIVE

        elif self.state == MissionState.SCHOOL_ZONE:
            # TODO: 스쿨존 구간 종료 판단 구현
            self.state = MissionState.LEFT_TURN_DECISION

        elif self.state == MissionState.LEFT_TURN_DECISION:
            # TODO: 좌회전/지름길 의사결정 로직 구현
            if perception.get('left_turn_signal', False):
                self.state = MissionState.SHORTCUT

        elif self.state == MissionState.SHORTCUT:
            # TODO: 지름길 완료 조건 구현
            self.state = MissionState.FINISH

        return self.state
