#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""미션 주행 통합 launch 파일.

한 번에 띄우는 노드:
  - track_drive_sve         : 메인 FSM 주행 (차선/콘/신호/정지선/좌회전/지름길)
  - yolo_node               : 4방향 카메라 차량 탐지 (best.pt, GPU)
  - obstacle_avoidance_node : YOLO 카운트로 회피 offset 계산

사용법:
  ros2 launch track_drive_sve mission.launch.py
  ros2 launch track_drive_sve mission.launch.py drive_mode:=auto         (실제 주행)
  ros2 launch track_drive_sve mission.launch.py drive_mode:=auto show_yolo:=true  (YOLO 창 보기)
  기본 drive_mode는 shadow(검증용, 모터 미발행), show_yolo는 false(성능 우선).

주의: ROS-TCP-Endpoint와 시뮬레이터는 따로 띄운 뒤 이 launch를 실행한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drive_mode = LaunchConfiguration("drive_mode")
    show_yolo = LaunchConfiguration("show_yolo")

    return LaunchDescription([
        DeclareLaunchArgument(
            "drive_mode",
            default_value="shadow",
            description="shadow(모터 미발행, 검증용) 또는 auto(실제 주행)",
        ),
        DeclareLaunchArgument(
            "show_yolo",
            default_value="false",
            description="YOLO 4분할 창 표시 여부(false=성능 우선, true=디버그)",
        ),

        # 메인 FSM 주행 노드
        Node(
            package="track_drive_sve",
            executable="track_drive_sve",
            name="track_drive_sve",
            output="screen",
            parameters=[{"drive_mode": drive_mode}],
        ),

        # YOLO 차량 탐지 노드 (GPU). show_window로 창 표시 제어.
        Node(
            package="track_drive_sve",
            executable="yolo_node",
            name="yolo_obstacle_quad_viewer",
            output="screen",
            parameters=[{"show_window": show_yolo}],
        ),

        # 차량 회피 offset 계산 노드
        Node(
            package="track_drive_sve",
            executable="obstacle_avoidance_node",
            name="obstacle_avoidance_driver",
            output="screen",
        ),
    ])
