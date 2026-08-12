#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gazebo Kookmin track용 track_drive_sve 단독 주행 launch 파일."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drive_mode = LaunchConfiguration("drive_mode")

    return LaunchDescription([
        DeclareLaunchArgument(
            "drive_mode",
            default_value="auto",
            description="shadow(모터 미발행, 검증용) 또는 auto(/xycar_motor 발행)",
        ),
        Node(
            package="track_drive_sve",
            executable="track_drive_sve",
            name="track_drive_sve",
            output="screen",
            parameters=[{"drive_mode": drive_mode}],
        ),
    ])
