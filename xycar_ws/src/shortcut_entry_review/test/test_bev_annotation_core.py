import pytest

from shortcut_entry_review.bev_annotation_core import BevAnnotationSession


def add_line(session, color, points):
    label = session.begin(color)
    for point in points:
        session.add_point(*point)
    return label


def test_w_w_y_y_sequence_builds_four_labeled_lines():
    session = BevAnnotationSession()
    assert add_line(session, "white", [(10, 90), (20, 20)]) == "W1"
    assert add_line(session, "white", [(40, 90), (45, 20)]) == "W2"
    assert add_line(session, "yellow", [(60, 90), (62, 20)]) == "Y1"
    assert add_line(session, "yellow", [(80, 90), (75, 20)]) == "Y2"
    session.finish()

    assert session.completed()
    assert [line.label for line in session.all_lines()] == [
        "W1",
        "W2",
        "Y1",
        "Y2",
    ]


def test_document_stores_normalized_trend_and_color_rank():
    session = BevAnnotationSession()
    add_line(session, "white", [(80, 90), (70, 50), (60, 10)])
    add_line(session, "white", [(20, 90), (25, 50), (30, 10)])
    add_line(session, "yellow", [(40, 90), (45, 50), (50, 10)])
    add_line(session, "yellow", [(90, 90), (85, 50), (80, 10)])
    session.finish()

    document = session.to_document(
        image_width=100,
        image_height=100,
        bag_offset_sec=8.344,
        timestamp_ns=123,
    )

    lines = {line["label"]: line for line in document["lines"]}
    assert lines["W2"]["left_to_right_rank_within_color"] == 0
    assert lines["W1"]["left_to_right_rank_within_color"] == 1
    assert lines["W1"]["direction_dx_dy"] > 0.0
    assert document["selected_white"] is None
    assert document["selected_yellow"] is None


def test_point_requires_active_color_and_each_line_requires_two_points():
    session = BevAnnotationSession()
    with pytest.raises(ValueError):
        session.add_point(1, 2)
    session.begin("white")
    session.add_point(1, 2)
    with pytest.raises(ValueError):
        session.finish()


def test_more_than_two_lines_per_color_is_rejected():
    session = BevAnnotationSession()
    add_line(session, "white", [(1, 2), (3, 4)])
    add_line(session, "white", [(5, 6), (7, 8)])
    session.finish()
    with pytest.raises(ValueError):
        session.begin("white")


def test_undo_removes_only_latest_current_point():
    session = BevAnnotationSession()
    session.begin("yellow")
    session.add_point(1, 2)
    session.add_point(3, 4)
    assert session.undo()
    assert session.current_points == [(1.0, 2.0)]
    assert session.undo()
    assert not session.undo()


def test_empty_y1_is_preserved_and_y2_can_be_annotated():
    session = BevAnnotationSession()
    add_line(session, "white", [(10, 90), (20, 20)])
    add_line(session, "white", [(40, 90), (45, 20)])
    assert session.begin("yellow") == "Y1"
    assert session.begin("yellow") == "Y2"
    session.add_point(60, 90)
    session.add_point(65, 20)
    session.finish()

    document = session.to_document(
        image_width=100,
        image_height=100,
        bag_offset_sec=8.344,
        timestamp_ns=123,
    )

    lines = {line["label"]: line for line in document["lines"]}
    assert session.completed()
    assert lines["Y1"]["detected"] is False
    assert lines["Y1"]["points_px"] == []
    assert lines["Y1"]["left_to_right_rank_within_color"] is None
    assert lines["Y2"]["detected"] is True


def test_save_can_finalize_an_empty_last_line():
    session = BevAnnotationSession()
    add_line(session, "white", [(10, 90), (20, 20)])
    add_line(session, "white", [(40, 90), (45, 20)])
    add_line(session, "yellow", [(60, 90), (65, 20)])
    session.begin("yellow")
    line = session.finish(allow_empty=True)

    assert line.label == "Y2"
    assert line.points_px == ()
    assert session.completed()
