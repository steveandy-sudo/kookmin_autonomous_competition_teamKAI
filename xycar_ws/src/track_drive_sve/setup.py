#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""track_drive_sve ROS2 Python 패키지 설치 설정입니다."""

from setuptools import setup

package_name = "track_drive_sve"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/launch",
            ["launch/mission.launch.py", "launch/gazebo_drive.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="track_drive_sve_team",
    maintainer_email="student@example.com",
    description="Gazebo에 맞춘 국민대 예선용 track_drive_sve 주행 패키지",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "track_drive_sve = track_drive_sve.track_drive:main",
            "shortcut_candidate_node = "
            "track_drive_sve.shortcut_candidate_node:main",
            "yolo_node = track_drive_sve.yolo_node:main",
            "obstacle_avoidance_node = track_drive_sve.obstacle_avoidance_node:main",
        ],
    },
)
