"""Pure left_4 confirmation and disappearance detector used by bag tools."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TriggerConfig:
    minimum_confidence: float = 0.50
    required_visible_frames: int = 2
    required_absent_frames: int = 2

    def __post_init__(self) -> None:
        if not 0.0 <= float(self.minimum_confidence) <= 1.0:
            raise ValueError("minimum_confidence must be in [0, 1]")
        if int(self.required_visible_frames) < 1:
            raise ValueError("required_visible_frames must be positive")
        if int(self.required_absent_frames) < 1:
            raise ValueError("required_absent_frames must be positive")


@dataclass(frozen=True)
class TriggerState:
    present: bool
    confidence: float
    visible_streak: int
    confirmed: bool
    absent_streak: int
    triggered: bool


class Left4TriggerDetector:
    """Latch S on the Nth missing detector frame after left_4 confirmation."""

    def __init__(self, config: TriggerConfig | None = None) -> None:
        self.config = config or TriggerConfig()
        self.reset()

    def reset(self) -> None:
        self.visible_streak = 0
        self.confirmed = False
        self.absent_streak = 0
        self.triggered = False

    def update(self, confidence: float | None) -> TriggerState:
        score = max(0.0, float(confidence or 0.0))
        present = score >= float(self.config.minimum_confidence)

        if self.triggered:
            return self._state(present, score)

        if present:
            self.visible_streak += 1
            self.absent_streak = 0
            if self.visible_streak >= int(self.config.required_visible_frames):
                self.confirmed = True
        elif self.confirmed:
            self.absent_streak += 1
            if self.absent_streak >= int(self.config.required_absent_frames):
                self.triggered = True
        else:
            self.visible_streak = 0

        return self._state(present, score)

    def _state(self, present: bool, confidence: float) -> TriggerState:
        return TriggerState(
            present=bool(present),
            confidence=float(confidence),
            visible_streak=int(self.visible_streak),
            confirmed=bool(self.confirmed),
            absent_streak=int(self.absent_streak),
            triggered=bool(self.triggered),
        )
