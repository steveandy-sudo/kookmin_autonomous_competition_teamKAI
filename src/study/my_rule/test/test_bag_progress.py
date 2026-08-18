from pathlib import Path

import pytest

from my_rule.bag_progress_node import (
    find_metadata,
    format_duration,
    load_bag_timing,
    progress_line,
)


def test_load_bag_timing_from_outer_directory(tmp_path: Path):
    bag = tmp_path / "archive" / "bag"
    bag.mkdir(parents=True)
    metadata = bag / "metadata.yaml"
    metadata.write_text(
        """rosbag2_bagfile_information:
  starting_time:
    nanoseconds_since_epoch: 1000000000
  duration:
    nanoseconds: 99100000000
""",
        encoding="utf-8",
    )

    assert find_metadata(str(tmp_path)) == metadata
    timing = load_bag_timing(str(tmp_path))
    assert timing.start_ns == 1_000_000_000
    assert timing.duration_ns == 99_100_000_000


def test_multiple_nested_bags_require_an_exact_directory(tmp_path: Path):
    for name in ("first", "second"):
        bag = tmp_path / name
        bag.mkdir()
        (bag / "metadata.yaml").write_text("metadata", encoding="utf-8")
    with pytest.raises(ValueError, match="multiple bags"):
        find_metadata(str(tmp_path))


def test_human_readable_progress_line():
    assert format_duration(99.1) == "01:39.1"
    assert format_duration(3661.2) == "1:01:01.2"
    line = progress_line("PLAYING", 24.75, 99.0, 0.5, 0, width=10)
    assert "00:24.8 / 01:39.0" in line
    assert "25.0%" in line
    assert "0.50x" in line
