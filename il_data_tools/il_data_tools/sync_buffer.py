from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Optional


@dataclass(frozen=True)
class TimedMessage:
    stamp_ns: int
    msg: Any


class TimedBuffer:
    """Small nearest-timestamp buffer for approximate synchronization."""

    def __init__(self, maxlen: int = 200):
        self._items: Deque[TimedMessage] = deque(maxlen=maxlen)

    def add(self, stamp_ns: int, msg: Any) -> None:
        self._items.append(TimedMessage(int(stamp_ns), msg))

    def latest(self) -> Optional[TimedMessage]:
        if not self._items:
            return None
        return self._items[-1]

    def nearest(self, stamp_ns: int, tolerance_ns: int) -> Optional[TimedMessage]:
        if not self._items:
            return None
        target = int(stamp_ns)
        best = min(self._items, key=lambda item: abs(item.stamp_ns - target))
        if abs(best.stamp_ns - target) <= int(tolerance_ns):
            return best
        return None

    def age_ns(self, now_ns: int) -> Optional[int]:
        latest = self.latest()
        if latest is None:
            return None
        return int(now_ns) - latest.stamp_ns

    def __len__(self) -> int:
        return len(self._items)
