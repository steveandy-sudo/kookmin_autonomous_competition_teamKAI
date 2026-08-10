import numpy as np

from my_rule.perception.object_perception import (
    DetectionRecord,
    apply_class_aliases,
    detection_side_counts,
    filter_detections,
    green_hsv_evidence_in_box,
    normalize_class_name,
    parse_class_aliases,
)


def record(name, confidence, xmin=10, ymin=10, xmax=30, ymax=30):
    return DetectionRecord(
        class_name=name,
        class_id=0,
        confidence=confidence,
        xmin=xmin,
        ymin=ymin,
        xmax=xmax,
        ymax=ymax,
    )


def test_object_class_names_are_normalized_without_aliasing():
    assert normalize_class_name("Traffic Cone") == "traffic_cone"
    assert normalize_class_name("yellow-centerline") == "yellow_centerline"


def test_model_specific_classes_map_to_mission_classes():
    aliases = parse_class_aliases(
        ["green_3=green", "red_car=car", "green_car=car"]
    )
    mapped = apply_class_aliases(
        [
            record("Green 3", 0.8),
            record("red-car", 0.9),
            record("cone", 0.7),
        ],
        aliases,
    )

    assert [item.class_name for item in mapped] == ["green", "car", "cone"]


def test_invalid_class_alias_is_rejected():
    try:
        parse_class_aliases(["green_3"])
    except ValueError as exc:
        assert "source=target" in str(exc)
    else:
        raise AssertionError("invalid alias must raise ValueError")


def test_detection_record_exposes_box_geometry():
    detection = record("cone", 0.8, 10, 20, 30, 60)

    assert detection.center_x == 20.0
    assert detection.center_y == 40.0
    assert detection.area == 800


def test_each_class_uses_its_own_confidence_threshold():
    detections = [
        record("cone", 0.49),
        record("car", 0.49),
        record("red", 0.51),
    ]

    accepted = filter_detections(
        detections,
        {
            "cone": 0.50,
            "car": 0.45,
            "red": 0.50,
        },
    )

    assert [item.class_name for item in accepted] == ["car", "red"]


def test_left_4_is_kept_when_the_shortcut_threshold_is_configured():
    detections = [record("left_4", 0.72), record("null_4", 0.99)]

    accepted = filter_detections(detections, {"left_4": 0.50})

    assert [item.class_name for item in accepted] == ["left_4"]


def test_four_lamp_signal_names_can_be_kept_distinct():
    detections = [
        record("red_4", 0.72),
        record("yellow_4", 0.73),
        record("green_4", 0.74),
    ]

    accepted = filter_detections(
        detections,
        {"red_4": 0.50, "yellow_4": 0.50, "green_4": 0.50},
    )

    assert [item.class_name for item in accepted] == [
        "red_4",
        "yellow_4",
        "green_4",
    ]


def test_green_hsv_is_measured_only_inside_detector_box():
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    image[10:30, 10:30] = (0, 255, 0)
    image[60:90, 60:90] = (0, 255, 0)

    evidence = green_hsv_evidence_in_box(
        image,
        (8, 8, 32, 32),
        lower_hsv=[40, 80, 80],
        upper_hsv=[90, 255, 255],
    )

    assert evidence.valid
    assert evidence.green_pixels == 400
    assert evidence.pixel_ratio > 0.60


def test_cone_side_counts_respect_center_deadband():
    detections = [
        record("cone", 0.9, 5, 10, 15, 30),
        record("cone", 0.9, 45, 10, 55, 30),
        record("cone", 0.9, 85, 10, 95, 30),
    ]

    total, left, right = detection_side_counts(
        detections,
        image_width=100,
        class_name="cone",
        center_deadband_ratio=0.1,
    )

    assert (total, left, right) == (3, 1, 1)
