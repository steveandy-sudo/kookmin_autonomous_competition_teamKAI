from types import SimpleNamespace
from unittest.mock import patch

from std_msgs.msg import Bool

from xycar_map_nav.sequential_hybrid_driver import SequentialHybridDriver
from xycar_map_nav.shortcut_mode_latch import ShortcutModeEvent
from xycar_map_nav.traffic_light_control import TrafficLightAction


class FakePublisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(bool(message.data))


class FakeLogger:
    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class FakeLatch:
    def __init__(self):
        self.active = False
        self.start_calls = 0

    def start(self, *, now_sec, confidence):
        del now_sec, confidence
        self.start_calls += 1
        self.active = True
        return ShortcutModeEvent.STARTED


def make_driver(*, timeout=12.0):
    driver = object.__new__(SequentialHybridDriver)
    values = {
        "shortcut_enabled": True,
        "shortcut_wait_for_entry_ready": True,
        "shortcut_entry_search_timeout_sec": float(timeout),
        "shortcut_candidate_timeout_sec": 0.35,
    }
    driver.get_parameter = lambda name: SimpleNamespace(value=values[name])
    driver.get_logger = lambda: FakeLogger()
    driver.shortcut_processing_pub = FakePublisher()
    driver.shortcut_latch = FakeLatch()
    driver.shortcut_entry_search_active = False
    driver.shortcut_entry_search_started_time = float("-inf")
    driver.shortcut_entry_ready = False
    driver.shortcut_command_time = 10.9
    driver.drive_armed = True
    driver.traffic_light_controller = SimpleNamespace(
        left_confidence=0.85,
        latest_decision=SimpleNamespace(
            action=TrafficLightAction.CLEAR,
            shortcut_start=False,
        ),
        shortcut_finished=lambda: None,
    )
    driver.traffic_light_decision = SimpleNamespace(
        action=TrafficLightAction.CLEAR,
        shortcut_start=True,
    )
    driver.handled_events = []
    driver._handle_shortcut_event = driver.handled_events.append
    return driver


def test_s_arms_perception_search_without_starting_shortcut_latch():
    driver = make_driver()

    driver._handle_traffic_shortcut_request(8.344)

    assert driver.shortcut_entry_search_active
    assert driver.shortcut_entry_search_started_time == 8.344
    assert driver.shortcut_processing_pub.values == [True]
    assert driver.shortcut_latch.start_calls == 0
    assert driver.handled_events == []


def test_w1_ready_starts_override_only_during_live_search():
    driver = make_driver()
    driver._handle_traffic_shortcut_request(10.0)
    driver.shortcut_command_time = 10.9

    with patch(
        "xycar_map_nav.sequential_hybrid_driver.time.monotonic",
        return_value=11.0,
    ):
        driver._on_shortcut_entry_ready(Bool(data=True))

    assert driver.shortcut_latch.active
    assert driver.shortcut_latch.start_calls == 1
    assert not driver.shortcut_entry_search_active
    assert driver.handled_events == [ShortcutModeEvent.STARTED]


def test_expired_or_stop_blocked_ready_never_starts_override():
    expired = make_driver(timeout=1.0)
    expired._handle_traffic_shortcut_request(10.0)
    with patch(
        "xycar_map_nav.sequential_hybrid_driver.time.monotonic",
        return_value=11.1,
    ):
        expired._on_shortcut_entry_ready(Bool(data=True))
    assert expired.shortcut_latch.start_calls == 0
    assert not expired.shortcut_entry_search_active
    assert expired.shortcut_processing_pub.values[-1] is False

    stopped = make_driver()
    stopped._handle_traffic_shortcut_request(10.0)
    stopped.traffic_light_decision = SimpleNamespace(
        action=TrafficLightAction.STOP,
        shortcut_start=False,
    )
    with patch(
        "xycar_map_nav.sequential_hybrid_driver.time.monotonic",
        return_value=11.0,
    ):
        stopped._on_shortcut_entry_ready(Bool(data=True))
    assert stopped.shortcut_latch.start_calls == 0
    assert stopped.shortcut_entry_search_active
