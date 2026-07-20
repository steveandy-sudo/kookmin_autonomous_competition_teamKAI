"""High-level mission states and lower-level control modes."""

from enum import Enum, auto


class StartSignal(Enum):
    UNKNOWN = auto()
    RED = auto()
    YELLOW = auto()
    GO = auto()


class MissionState(Enum):
    WAIT_START_SIGNAL = auto()

    LANE_DRIVING = auto()
    CONE_SECTION = auto()
    FIXED_OBSTACLE_SECTION = auto()
    OVERTAKE_SECTION = auto()
    ROUTE_SELECTION = auto()
    SHORTCUT_SECTION = auto()


class ControlMode(Enum):
    STOP = auto()

    NORMAL_IL = auto()
    CONE_DRIVE_RULE = auto()
    LANE_FALLBACK = auto()

    FIXED_OBSTACLE_RULE = auto()
    VEHICLE_FOLLOW = auto()
    VEHICLE_OVERTAKE = auto()
    SHORTCUT_RULE = auto()
