from datetime import datetime
from pathlib import Path

from il_data_tools.record_utils import (
    atomic_write_json,
    make_session_dir,
    motor_from_msg,
    sanitize_session_name,
)
from il_data_tools.sync_buffer import TimedBuffer


class FakeMotor:
    angle = 1.5
    speed = 8.0


class FakeArray:
    data = [2.0, 9.0]


def test_sanitize_session_name():
    assert sanitize_session_name(" lane run 01 ") == "lane_run_01"
    assert sanitize_session_name("!!!") == "session"


def test_motor_from_msg_supports_xycar_and_array_shapes():
    assert motor_from_msg(FakeMotor()) == (1.5, 8.0)
    assert motor_from_msg(FakeArray()) == (2.0, 9.0)


def test_sync_buffer_nearest_timestamp():
    buffer = TimedBuffer()
    buffer.add(100, "old")
    buffer.add(210, "best")
    buffer.add(500, "far")
    assert buffer.nearest(200, 20).msg == "best"
    assert buffer.nearest(200, 5) is None


def test_metadata_writer_and_session_dir(tmp_path: Path):
    session_dir = make_session_dir(
        str(tmp_path), "unit test", now=datetime(2026, 7, 6, 12, 0, 0)
    )
    assert session_dir.name == "20260706_120000_unit_test"
    metadata_path = session_dir / "metadata.json"
    atomic_write_json(metadata_path, {"ok": True})
    assert '"ok": true' in metadata_path.read_text(encoding="utf-8")
