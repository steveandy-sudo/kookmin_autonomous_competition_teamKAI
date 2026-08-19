from pathlib import Path

import numpy as np

from my_rule.perception.lidar_camera_association import (
    associate_lidar_clusters_with_boxes,
    load_lidar_camera_extrinsic,
    project_laser_xy,
)


def test_project_laser_xy_uses_rectified_camera_coordinates():
    rotation = np.asarray(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ]
    )
    matrix = np.asarray(
        [
            [100.0, 0.0, 320.0],
            [0.0, 100.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )
    pixels, valid = project_laser_xy(
        np.asarray([(1.0, 0.0), (-1.0, 0.0)]),
        rotation,
        np.zeros(3),
        matrix,
    )
    assert valid.tolist() == [True, False]
    assert np.allclose(pixels[0], [320.0, 240.0])


def test_association_keeps_only_clusters_inside_a_cone_box():
    rotation = np.asarray(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ]
    )
    matrix = np.asarray(
        [
            [100.0, 0.0, 320.0],
            [0.0, 100.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )
    clusters = [(1.0, 0.0), (1.0, -0.5)]
    associated = associate_lidar_clusters_with_boxes(
        clusters,
        [(300.0, 220.0, 340.0, 260.0)],
        rotation_camera_laser=rotation,
        translation_camera_laser=np.zeros(3),
        camera_matrix=matrix,
        image_width=640,
        image_height=480,
        padding_ratio=0.0,
        minimum_padding_px=0.0,
    )
    assert associated == [(1.0, 0.0)]


def test_association_keeps_only_nearest_cluster_in_each_box():
    rotation = np.asarray(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ]
    )
    matrix = np.asarray(
        [
            [100.0, 0.0, 320.0],
            [0.0, 100.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )
    associated = associate_lidar_clusters_with_boxes(
        [(2.0, 0.0), (1.0, 0.0)],
        [(300.0, 220.0, 340.0, 260.0)],
        rotation_camera_laser=rotation,
        translation_camera_laser=np.zeros(3),
        camera_matrix=matrix,
        image_width=640,
        image_height=480,
        padding_ratio=0.0,
        minimum_padding_px=0.0,
    )
    assert associated == [(1.0, 0.0)]


def test_association_keeps_one_nearest_cluster_for_each_separate_box():
    rotation = np.asarray(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ]
    )
    matrix = np.asarray(
        [
            [100.0, 0.0, 320.0],
            [0.0, 100.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )
    associated = associate_lidar_clusters_with_boxes(
        [(1.0, 0.2), (2.0, 0.4), (1.0, -0.2)],
        [
            (290.0, 220.0, 310.0, 260.0),
            (330.0, 220.0, 350.0, 260.0),
        ],
        rotation_camera_laser=rotation,
        translation_camera_laser=np.zeros(3),
        camera_matrix=matrix,
        image_width=640,
        image_height=480,
        padding_ratio=0.0,
        minimum_padding_px=0.0,
    )
    assert associated == [(1.0, 0.2), (1.0, -0.2)]


def test_horizontal_only_match_does_not_require_planar_lidar_height():
    rotation = np.asarray(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ]
    )
    matrix = np.asarray(
        [
            [100.0, 0.0, 320.0],
            [0.0, 100.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )
    kwargs = dict(
        rotation_camera_laser=rotation,
        translation_camera_laser=np.zeros(3),
        camera_matrix=matrix,
        image_width=640,
        image_height=480,
        padding_ratio=0.0,
        minimum_padding_px=0.0,
    )
    box_above_lidar_plane = [(300.0, 100.0, 340.0, 180.0)]
    assert associate_lidar_clusters_with_boxes(
        [(1.0, 0.0)],
        box_above_lidar_plane,
        match_vertical=True,
        **kwargs,
    ) == []
    assert associate_lidar_clusters_with_boxes(
        [(1.0, 0.0)],
        box_above_lidar_plane,
        match_vertical=False,
        **kwargs,
    ) == [(1.0, 0.0)]


def test_measured_extrinsic_from_repository_is_valid():
    source_root = Path(__file__).resolve().parents[3]
    path = (
        source_root
        / "xycar_perception"
        / "config"
        / "lidar_camera_extrinsic_measured.yaml"
    )
    rotation, translation = load_lidar_camera_extrinsic(path)
    assert rotation.shape == (3, 3)
    assert translation.shape == (3,)
    assert np.isfinite(rotation).all()
    assert np.isfinite(translation).all()
