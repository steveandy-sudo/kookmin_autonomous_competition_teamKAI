"""High-level mission states and lower-level control modes."""

from enum import Enum, auto


class MissionState(Enum):
    WAIT_START_SIGNAL = auto()
    RACING = auto()
    EMERGENCY_STOP = auto()


class ControlMode(Enum):
    STOP = auto()

    NORMAL_IL = auto()
    CONE_DRIVE_RULE = auto()
    LANE_FALLBACK = auto()

    FIXED_OBSTACLE_RULE = auto()
    VEHICLE_FOLLOW = auto()
    VEHICLE_OVERTAKE = auto()
    ROUTE_SELECT = auto()
    SHORTCUT = auto()
