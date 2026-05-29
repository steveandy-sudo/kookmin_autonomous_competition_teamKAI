#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point as RosPoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray
from xycar_msgs.msg import XycarMotor


Point = Tuple[float, float]


@dataclass
class PathCandidate:
    offset: float
    path: List[Point]
    cost: float
    min_clearance: float


@dataclass
class LocalObstacle:
    track_id: float = -1.0
    x: float = 0.0
    y: float = 0.0
    size_x: float = 0.0
    size_y: float = 0.0
    size_z: float = 0.0
    type_id: float = 0.0
    speed: float = 0.0
    camera_confirmed: float = 0.0
    motion_state: float = 0.0


@dataclass
class VehicleTrack:
    track_id: int
    class_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    last_seen_sec: float = 0.0
    yolo_last_seen_sec: float = 0.0
    seen_count: int = 0
    box: Optional[Tuple[int, int, int, int]] = None


@dataclass
class VehicleBehavior:
    mode: str
    speed: float
    path: Optional[List[Point]] = None
    max_steer_deg: Optional[float] = None
    steer_smoothing: Optional[float] = None
    lookahead_scale: Optional[float] = None


class TrackDriverNode(Node):
    def __init__(self):
        super().__init__('driver')

        self.declare_parameter('camera_topic', '/usb_cam/image_raw/front')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('motor_topic', 'xycar_motor')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('publish_debug_visualization', True)
        self.declare_parameter('viz_frame_id', 'map')
        self.declare_parameter('viz_marker_lifetime_sec', 1.0)
        self.declare_parameter('publish_light_debug_image', True)
        self.declare_parameter('light_debug_image_topic', '/track_drive/light_debug_image')

        self.declare_parameter('base_speed', 5.0)
        self.declare_parameter('min_speed', 3.0)
        self.declare_parameter('school_zone_enabled', True)
        self.declare_parameter('school_zone_speed', 5.0)
        self.declare_parameter('school_zone_roi_top_ratio', 0.40)
        self.declare_parameter('school_zone_left_edge_max_ratio', 0.36)
        self.declare_parameter('school_zone_right_edge_min_ratio', 0.64)
        self.declare_parameter('school_zone_yellow_ratio_threshold', 0.006)
        self.declare_parameter('school_zone_yellow_row_ratio_threshold', 0.14)
        self.declare_parameter('school_zone_yellow_pair_row_ratio_threshold', 0.12)
        self.declare_parameter('school_zone_yellow_bottom_pair_row_ratio_threshold', 0.10)
        self.declare_parameter('school_zone_yellow_min_pair_rows', 6)
        self.declare_parameter('school_zone_yellow_min_separation_ratio', 0.42)
        self.declare_parameter('school_zone_yellow_row_max_width_ratio', 0.10)
        self.declare_parameter('school_zone_yellow_min_pixels', 120)
        self.declare_parameter('school_zone_preslow_enabled', True)
        self.declare_parameter('school_zone_preslow_ratio', 0.60)
        self.declare_parameter('school_zone_confirm_frames', 2)
        self.declare_parameter('school_zone_lost_frames', 3)
        self.declare_parameter('school_zone_hold_sec', 1.0)
        self.declare_parameter('school_zone_mask_vehicle_boxes', True)
        self.declare_parameter('school_zone_suppress_when_vehicle_visible', True)
        self.declare_parameter('school_zone_follow_yellow_centerline', True)
        self.declare_parameter('school_zone_takeover_enabled', False)
        self.declare_parameter('school_zone_center_left_ratio', 0.30)
        self.declare_parameter('school_zone_center_right_ratio', 0.70)
        self.declare_parameter('school_zone_center_min_pixels', 24)
        self.declare_parameter('school_zone_center_band_count', 11)
        self.declare_parameter('school_zone_center_band_half_height', 34.0)
        self.declare_parameter('school_zone_center_min_band_pixels', 2)
        self.declare_parameter('school_zone_center_memory_sec', 1.20)
        self.declare_parameter('school_zone_center_memory_weight', 0.45)
        self.declare_parameter('school_zone_edge_center_weight', 0.20)
        self.declare_parameter('school_zone_yellow_path_edge_guard_enabled', True)
        self.declare_parameter('school_zone_yellow_path_edge_limit', 0.35)
        self.declare_parameter('school_zone_max_steer_deg', 70.0)
        self.declare_parameter('school_zone_steer_smoothing', 0.05)
        self.declare_parameter('school_zone_lookahead_scale', 0.72)
        self.declare_parameter('stop_distance', 0.75)
        self.declare_parameter('slow_distance', 1.35)
        self.declare_parameter('lookahead_min', 1.0)
        self.declare_parameter('lookahead_max', 2.4)
        self.declare_parameter('wheelbase', 0.32)
        self.declare_parameter('max_steer_deg', 50.0)
        self.declare_parameter('heading_steer_gain', 0.60)
        self.declare_parameter('cross_track_steer_gain', 0.35)
        self.declare_parameter('curve_preview_gain', 0.35)
        self.declare_parameter('steer_smoothing', 0.15)
        self.declare_parameter('invert_steering', True)

        self.declare_parameter('ai_hybrid_enabled', True)
        self.declare_parameter('ai_model_path', '~/cone_il_model_merged_new2/cone_bc_scripted.pt')
        self.declare_parameter('ai_speed', 10.0)
        self.declare_parameter('ai_fixed_speed_enabled', True)
        self.declare_parameter('ai_passthrough_enabled', True)
        self.declare_parameter('ai_command_passthrough_enabled', False)
        self.declare_parameter('ai_command_topic', '/track_drive/ai_motor')
        self.declare_parameter('hybrid_trigger_topic', '/track_drive/hybrid_trigger')
        self.declare_parameter('hybrid_standby_enabled', False)
        self.declare_parameter('ai_enable_topic', '/cone_ai/enable')
        self.declare_parameter('ai_speed_limit_topic', '/cone_ai/speed_limit')
        self.declare_parameter('yolo_safety_enabled', True)
        self.declare_parameter('yolo_person_model_path', '/home/xytron/model/best.onnx')
        self.declare_parameter('yolo_light_model_path', '/home/xytron/model/best_new.onnx')
        self.declare_parameter('yolo_person_input_size', 640)
        self.declare_parameter('yolo_light_input_size', 640)
        self.declare_parameter('yolo_person_class_count', 4)
        self.declare_parameter('yolo_light_class_count', 5)
        self.declare_parameter('yolo_person_conf_threshold', 0.26)
        self.declare_parameter('yolo_light_conf_threshold', 0.70)
        self.declare_parameter('yolo_person_class_ids', [3])
        self.declare_parameter('yolo_vehicle_class_ids', [0, 2])
        self.declare_parameter('yolo_light_class_ids', [0, 1, 2, 3, 4])
        self.declare_parameter('yolo_red_light_class_ids', [3, 4])
        self.declare_parameter('yolo_go_light_class_ids', [1])
        self.declare_parameter('yolo_person_min_box_height_ratio', 0.035)
        self.declare_parameter('yolo_person_min_box_bottom_ratio', 0.24)
        self.declare_parameter('yolo_vehicle_min_box_height_ratio', 0.025)
        self.declare_parameter('yolo_vehicle_min_box_width_ratio', 0.025)
        self.declare_parameter('yolo_vehicle_min_box_bottom_ratio', 0.18)
        self.declare_parameter('yolo_light_min_box_height_ratio', 0.015)
        self.declare_parameter('yolo_light_max_box_height_ratio', 0.45)
        self.declare_parameter('yolo_light_min_box_area_ratio', 0.00008)
        self.declare_parameter('yolo_light_max_box_area_ratio', 0.080)
        self.declare_parameter('yolo_light_max_box_bottom_ratio', 0.70)
        self.declare_parameter('yolo_nms_threshold', 0.45)
        self.declare_parameter('yolo_safety_period_sec', 0.02)
        self.declare_parameter('yolo_red_light_period_sec', 0.10)
        self.declare_parameter('stop_on_red_light_enabled', True)
        self.declare_parameter('red_light_confirm_frames', 2)
        self.declare_parameter('red_light_color_fallback_enabled', False)
        self.declare_parameter('red_light_roi_top_ratio', 0.0)
        self.declare_parameter('red_light_roi_bottom_ratio', 0.65)
        self.declare_parameter('red_light_min_area', 30.0)
        self.declare_parameter('red_light_min_ratio', 0.006)
        self.declare_parameter('red_light_min_dominance', 1.80)
        self.declare_parameter('red_light_min_circularity', 0.25)
        self.declare_parameter('red_light_go_release_enabled', True)
        self.declare_parameter('startup_light_check_enabled', False)
        self.declare_parameter('startup_light_check_timeout_sec', 0.80)
        self.declare_parameter('stop_on_person_enabled', False)
        self.declare_parameter('stop_on_vehicle_enabled', False)
        self.declare_parameter('vehicle_overtake_enabled', False)
        self.declare_parameter('vehicle_follow_enabled', False)
        self.declare_parameter('vehicle_camera_fallback_enabled', False)
        self.declare_parameter('vehicle_fusion_camera_fov_deg', 62.0)
        self.declare_parameter('vehicle_fusion_angle_margin_deg', 12.0)
        self.declare_parameter('vehicle_fusion_min_x', 0.65)
        self.declare_parameter('vehicle_fusion_max_distance', 12.0)
        self.declare_parameter('vehicle_fusion_lateral_limit', 1.60)
        self.declare_parameter('vehicle_fusion_lidar_min_points', 1)
        self.declare_parameter('vehicle_camera_min_distance', 1.20)
        self.declare_parameter('vehicle_track_timeout_sec', 1.20)
        self.declare_parameter('vehicle_yolo_required_timeout_sec', 0.90)
        self.declare_parameter('vehicle_track_match_distance', 1.20)
        self.declare_parameter('vehicle_track_velocity_smoothing', 0.55)
        self.declare_parameter('vehicle_current_lane_lateral_limit', 1.00)
        self.declare_parameter('vehicle_follow_distance', 2.80)
        self.declare_parameter('vehicle_follow_min_distance', 0.60)
        self.declare_parameter('vehicle_follow_stop_distance', 0.90)
        self.declare_parameter('vehicle_follow_close_distance', 1.80)
        self.declare_parameter('vehicle_follow_close_speed', 3.5)
        self.declare_parameter('vehicle_follow_max_speed', 12.0)
        self.declare_parameter('vehicle_follow_gap_gain', 2.00)
        self.declare_parameter('vehicle_follow_relative_gain', 1.20)
        self.declare_parameter('vehicle_follow_closing_gain', 3.00)
        self.declare_parameter('vehicle_follow_speed_smoothing', 0.25)
        self.declare_parameter('vehicle_follow_target_switch_margin', 0.80)
        self.declare_parameter('vehicle_follow_lateral_gain', 0.70)
        self.declare_parameter('vehicle_follow_lateral_smoothing', 0.65)
        self.declare_parameter('vehicle_follow_lateral_max_step', 0.08)
        self.declare_parameter('vehicle_follow_lateral_deadband', 0.03)
        self.declare_parameter('vehicle_fast_follow_enabled', False)
        self.declare_parameter('vehicle_fast_class_ids', [0])
        self.declare_parameter('vehicle_slow_class_ids', [2])
        self.declare_parameter('vehicle_fast_follow_min_sec', 2.50)
        self.declare_parameter('vehicle_follow_steer_gain', 1.15)
        self.declare_parameter('vehicle_follow_min_steer_deg', 0.0)
        self.declare_parameter('vehicle_follow_max_steer_deg', 60.0)
        self.declare_parameter('vehicle_fast_slow_speed_gap', 0.10)
        self.declare_parameter('vehicle_slow_relative_speed_threshold', 0.35)
        self.declare_parameter('vehicle_lead_min_x', 0.45)
        self.declare_parameter('vehicle_overtake_trigger_distance', 5.50)
        self.declare_parameter('vehicle_overtake_confirm_sec', 0.00)
        self.declare_parameter('vehicle_overtake_min_seen', 1)
        self.declare_parameter('vehicle_overtake_speed', 12.0)
        self.declare_parameter('vehicle_overtake_max_speed', 12.0)
        self.declare_parameter('vehicle_overtake_relative_speed_gain', 0.0)
        self.declare_parameter('vehicle_overtake_prediction_sec', 0.90)
        self.declare_parameter('vehicle_overtake_speed_hold_gain', 1.20)
        self.declare_parameter('vehicle_overtake_speed_pass_gain', 0.70)
        self.declare_parameter('vehicle_overtake_plan_length', 22.0)
        self.declare_parameter('vehicle_overtake_offset', 0.30)
        self.declare_parameter('vehicle_overtake_prefer_right', False)
        self.declare_parameter('vehicle_overtake_shift_start', 0.20)
        self.declare_parameter('vehicle_overtake_shift_length', 2.00)
        self.declare_parameter('vehicle_overtake_hold_length', 4.50)
        self.declare_parameter('vehicle_overtake_return_length', 4.50)
        self.declare_parameter('vehicle_overtake_pass_margin', 1.50)
        self.declare_parameter('vehicle_overtake_return_to_slow_lane_enabled', True)
        self.declare_parameter('vehicle_overtake_min_time_sec', 1.80)
        self.declare_parameter('vehicle_overtake_timeout_sec', 8.00)
        self.declare_parameter('vehicle_overtake_close_stop_distance', 0.0)
        self.declare_parameter('vehicle_overtake_safety_distance', 2.20)
        self.declare_parameter('vehicle_overtake_max_steer_deg', 100.0)
        self.declare_parameter('vehicle_overtake_steer_smoothing', 0.05)
        self.declare_parameter('vehicle_overtake_steer_gain', 1.35)
        self.declare_parameter('vehicle_overtake_min_steer_deg', 10.0)
        self.declare_parameter('vehicle_overtake_lookahead_scale', 0.70)
        self.declare_parameter('vehicle_lane_edge_follow_enabled', True)
        self.declare_parameter('vehicle_outer_wheel_lateral_offset', 0.22)
        self.declare_parameter('vehicle_lane_edge_margin', 0.12)
        self.declare_parameter('lane_edge_steer_guard_enabled', True)
        self.declare_parameter('lane_edge_steer_guard_start_ratio', 0.75)
        self.declare_parameter('lane_edge_steer_guard_max_outward_deg', 18.0)
        self.declare_parameter('lane_edge_steer_guard_min_outward_deg', 6.0)
        self.declare_parameter('vehicle_lidar_fallback_enabled', False)
        self.declare_parameter('vehicle_lidar_fallback_min_x', 0.08)
        self.declare_parameter('vehicle_lidar_fallback_max_distance', 6.0)
        self.declare_parameter('vehicle_lidar_fallback_lateral_limit', 0.90)
        self.declare_parameter('vehicle_lidar_fallback_cluster_gap', 0.35)
        self.declare_parameter('vehicle_lidar_fallback_min_points', 2)
        self.declare_parameter('vehicle_lidar_fallback_min_size', 0.12)
        self.declare_parameter('vehicle_lidar_fallback_skip_on_person', True)
        self.declare_parameter('vehicle_lidar_fallback_require_yolo_seed', True)
        self.declare_parameter('person_stop_distance', 1.15)
        self.declare_parameter('person_lateral_limit', 0.38)
        self.declare_parameter('person_lidar_min_points', 2)
        self.declare_parameter('person_lidar_fallback_enabled', True)
        self.declare_parameter('person_camera_enabled', False)
        self.declare_parameter('person_yolo_camera_fallback_enabled', True)
        self.declare_parameter('person_hog_min_weight', 0.55)
        self.declare_parameter('person_fusion_enabled', True)
        self.declare_parameter('person_fusion_camera_fov_deg', 62.0)
        self.declare_parameter('person_fusion_angle_margin_deg', 16.0)
        self.declare_parameter('person_fusion_max_distance', 6.0)
        self.declare_parameter('person_fusion_lateral_limit', 1.60)
        self.declare_parameter('person_fusion_lidar_min_points', 1)
        self.declare_parameter('person_dynamic_enabled', True)
        self.declare_parameter('person_dynamic_prediction_sec', 0.80)
        self.declare_parameter('person_dynamic_velocity_smoothing', 0.45)
        self.declare_parameter('person_dynamic_max_speed', 2.50)
        self.declare_parameter('person_dynamic_lateral_deadband', 0.15)
        self.declare_parameter('person_dynamic_image_center_deadband', 0.12)
        self.declare_parameter('person_dynamic_image_velocity_deadband', 0.20)
        self.declare_parameter('person_dynamic_crossing_lateral_limit', 0.55)
        self.declare_parameter('person_dynamic_crossing_x_limit', 3.00)
        self.declare_parameter('person_dynamic_crossing_speed', 4.0)
        self.declare_parameter('person_reverse_enabled', False)
        self.declare_parameter('person_reverse_distance', 1.05)
        self.declare_parameter('person_reverse_release_distance', 1.35)
        self.declare_parameter('person_reverse_speed', -4.0)
        self.declare_parameter('person_reverse_max_sec', 1.00)
        self.declare_parameter('person_reverse_steer_deg', 0.0)
        self.declare_parameter('person_avoidance_enabled', False)
        self.declare_parameter('person_wait_release_left_y', 0.55)
        self.declare_parameter('person_wait_release_image_left_ratio', -0.35)
        self.declare_parameter('person_wait_release_confirm_frames', 2)
        self.declare_parameter('person_wait_lost_release_sec', 0.50)
        self.declare_parameter('person_avoidance_offset', 2.80)
        self.declare_parameter('person_avoidance_two_lane_offset', 5.60)
        self.declare_parameter('person_avoidance_stop_close_x', 5.50)
        self.declare_parameter('person_avoidance_plan_length', 30.0)
        self.declare_parameter('person_avoidance_prefer_right', False)
        self.declare_parameter('person_avoidance_obstacle_x', 2.20)
        self.declare_parameter('person_avoidance_obstacle_half_x', 2.00)
        self.declare_parameter('person_avoidance_shift_start', 1.00)
        self.declare_parameter('person_avoidance_shift_length', 15.00)
        self.declare_parameter('person_avoidance_hold_length', 8.00)
        self.declare_parameter('person_avoidance_return_length', 20.00)
        self.declare_parameter('person_avoidance_allow_return', False)
        self.declare_parameter('person_avoidance_speed', 10.0)
        self.declare_parameter('person_avoidance_boost_speed', 14.0)
        self.declare_parameter('person_avoidance_boost_sec', 0.0)
        self.declare_parameter('person_avoidance_hold_sec', 0.0)
        self.declare_parameter('person_avoidance_close_stop_distance', 0.05)
        self.declare_parameter('person_avoidance_steer_gain', 1.0)
        self.declare_parameter('person_avoidance_max_steer_deg', 100.0)
        self.declare_parameter('person_avoidance_min_steer_deg', 0.0)
        self.declare_parameter('person_avoidance_steer_smoothing', 0.0)
        self.declare_parameter('safety_stop_hold_sec', 0.35)
        self.declare_parameter('hybrid_on_obstacle_enabled', False)
        self.declare_parameter('hybrid_obstacle_distance', 0.75)
        self.declare_parameter('ai_curve_speed', 3.0)
        self.declare_parameter('ai_curve_start_steer_deg', 8.0)
        self.declare_parameter('ai_curve_full_steer_deg', 45.0)
        self.declare_parameter('ai_speed_smoothing', 0.20)
        self.declare_parameter('ai_max_steer_deg', 100.0)
        self.declare_parameter('ai_invert_steering', False)
        self.declare_parameter('ai_steer_smoothing', 0.20)
        self.declare_parameter('ai_steer_deadband_deg', 1.5)
        self.declare_parameter('ai_max_steer_step_deg', 7.0)
        self.declare_parameter('ai_roi_top_ratio', 0.45)
        self.declare_parameter('ai_resize_width', 160)
        self.declare_parameter('ai_resize_height', 90)

        self.declare_parameter('lane_width', 1.2)
        self.declare_parameter('path_length', 7.0)
        self.declare_parameter('path_spacing', 0.35)
        self.declare_parameter('lattice_offsets', [0.0])
        self.declare_parameter('cone_lattice_offsets', [0.0])
        self.declare_parameter('cone_collision_radius', 0.28)
        self.declare_parameter('cone_clearance_weight', 1.7)
        self.declare_parameter('offset_weight', 0.45)
        self.declare_parameter('cone_offset_weight', 3.0)
        self.declare_parameter('continuity_weight', 0.80)
        self.declare_parameter('curvature_weight', 0.35)
        self.declare_parameter('cone_path_weight', 0.90)
        self.declare_parameter('cone_lookahead_scale', 0.58)
        self.declare_parameter('front_obstacle_half_width', 0.18)

        self.declare_parameter('lane_roi_top_ratio', 0.56)
        self.declare_parameter('lane_center_gain', 1.15)
        self.declare_parameter('lane_heading_gain', 2.20)
        self.declare_parameter('camera_lateral_scale', 0.0027)
        self.declare_parameter('camera_forward_scale', 0.020)
        self.declare_parameter('white_lane_min_pixels', 80)
        self.declare_parameter('white_lane_expected_width_px', 380.0)
        self.declare_parameter('lane_guard_memory_sec', 2.00)

        self.declare_parameter('scan_min_range', 0.10)
        self.declare_parameter('scan_max_range', 8.0)
        self.declare_parameter('scan_lateral_limit', 1.0)
        self.declare_parameter('nearest_obstacle_min_x', 0.20)
        self.declare_parameter('scan_front_index', -1)
        self.declare_parameter('scan_reverse', False)
        self.declare_parameter('scan_angle_offset', 0.0)
        self.declare_parameter('cone_cluster_gap', 0.24)
        self.declare_parameter('cone_cluster_min_points', 1)
        self.declare_parameter('cone_cluster_max_size', 0.60)
        self.declare_parameter('cone_side_min_y', 0.15)
        self.declare_parameter('cone_pair_min_width', 0.45)
        self.declare_parameter('cone_pair_max_width', 2.40)
        self.declare_parameter('cone_pair_max_x_gap', 1.30)
        self.declare_parameter('cone_waypoint_max_pairs', 6)
        self.declare_parameter('cone_boundary_link_max', 3.00)
        self.declare_parameter('cone_boundary_lateral_limit', 1.00)
        self.declare_parameter('cone_center_ignore_x', 2.60)
        self.declare_parameter('cone_boundary_min_points', 2)
        self.declare_parameter('cone_seed_neighbor_radius', 1.80)

        self.image: Optional[np.ndarray] = None
        self.scan_msg: Optional[LaserScan] = None
        self.bridge = CvBridge()
        self.motor_msg = XycarMotor()
        self.ai_model = None
        self.ai_torch = None
        self.ai_device = None
        self.prev_ai_steer = 0.0
        self.prev_ai_speed: Optional[float] = None
        self._ai_last_warn_sec = -1
        self.hybrid_trigger_active = False
        self.last_ai_motor_msg: Optional[XycarMotor] = None
        self.last_ai_enable: Optional[bool] = None
        self.person_hog = None
        self.yolo_nets = {}
        self.yolo_last_warn_sec = {}
        self.next_yolo_safety_check_sec = 0.0
        self.next_yolo_light_check_sec = 0.0
        self.startup_time_sec = time.monotonic()
        self.yolo_light_checked_once = False
        self.yolo_light_last_check_sec = 0.0
        self.cached_yolo_person = False
        self.cached_yolo_person_detections: List[Tuple[Tuple[int, int, int, int], float, int]] = []
        self.cached_yolo_person_box: Optional[Tuple[int, int, int, int]] = None
        self.cached_yolo_vehicle = False
        self.cached_yolo_vehicle_raw_count = 0
        self.cached_yolo_vehicle_detections: List[Tuple[Tuple[int, int, int, int], float, int]] = []
        self.cached_yolo_vehicle_box: Optional[Tuple[int, int, int, int]] = None
        self.cached_yolo_red_light = False
        self.red_light_confirm_count = 0
        self.vehicle_tracks: Dict[int, VehicleTrack] = {}
        self.next_vehicle_track_id = 1
        self.vehicle_overtake_track_id: Optional[int] = None
        self.vehicle_overtake_started_sec: Optional[float] = None
        self.vehicle_slow_confirm_start_sec: Optional[float] = None
        self.vehicle_fast_follow_track_id: Optional[int] = None
        self.vehicle_fast_follow_started_sec: Optional[float] = None
        self.prev_vehicle_follow_speed: Optional[float] = None
        self.vehicle_follow_track_id: Optional[int] = None
        self.vehicle_follow_lateral_offset = 0.0
        self.vehicle_overtake_last_target: Optional[VehicleTrack] = None
        self.vehicle_overtake_return_offset = 0.0
        self.vehicle_overtake_offset_sign = 1.0
        self.person_fusion_distance: Optional[float] = None
        self.person_fusion_point: Optional[Point] = None
        self.person_predicted_point: Optional[Point] = None
        self.prev_person_fusion_point: Optional[Point] = None
        self.prev_person_fusion_time_sec: Optional[float] = None
        self.person_velocity: Point = (0.0, 0.0)
        self.person_image_center_ratio: Optional[float] = None
        self.prev_person_image_center_ratio: Optional[float] = None
        self.prev_person_image_time_sec: Optional[float] = None
        self.person_image_velocity_ratio = 0.0
        self.person_avoidance_until_sec = 0.0
        self.person_avoidance_active = False
        self.person_avoidance_started_sec: Optional[float] = None
        self.person_reverse_until_sec = 0.0
        self.person_avoidance_offset_sign = 1.0
        self.person_lattice_selected_offset = 0.0
        self.person_lattice_should_stop = False
        self.person_wait_left_confirm_count = 0
        self.person_wait_last_seen_sec: Optional[float] = None
        self.school_zone_active = False
        self.school_zone_yellow_left_ratio = 0.0
        self.school_zone_yellow_right_ratio = 0.0
        self.school_zone_yellow_pair_row_ratio = 0.0
        self.school_zone_yellow_bottom_pair_row_ratio = 0.0
        self.school_zone_yellow_separation_ratio = 0.0
        self.school_zone_candidate_active = False
        self.school_zone_candidate_last_seen_sec: Optional[float] = None
        self.school_zone_confirm_count = 0
        self.school_zone_lost_count = 0
        self.school_zone_last_seen_sec: Optional[float] = None
        self.last_school_zone_path: List[Point] = []
        self.last_school_zone_path_seen_sec: Optional[float] = None
        self.safety_stop_until_sec = 0.0
        self.last_safety_stop_reason = 'safety_stop'

        self.prev_offset = 0.0
        self.prev_steer = 0.0
        self.last_center_path: List[Point] = self._straight_path()
        self.last_left_boundary: List[Point] = []
        self.last_right_boundary: List[Point] = []
        self.last_cone_centerline: List[Point] = []
        self.last_white_lane_path: List[Point] = []
        self.last_white_lane_seen_sec: Optional[float] = None
        self.current_left_boundary: List[Point] = []
        self.current_right_boundary: List[Point] = []
        self.current_cone_centerline: List[Point] = []
        self.last_mode = 'straight'
        self.last_candidates: List[PathCandidate] = []

        self._load_ai_model()

        camera_topic = self.get_parameter('camera_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        motor_topic = self.get_parameter('motor_topic').value
        ai_command_topic = self.get_parameter('ai_command_topic').value
        hybrid_trigger_topic = self.get_parameter('hybrid_trigger_topic').value
        ai_enable_topic = self.get_parameter('ai_enable_topic').value
        ai_speed_limit_topic = self.get_parameter('ai_speed_limit_topic').value
        light_debug_image_topic = self.get_parameter('light_debug_image_topic').value

        self.motor_pub = self.create_publisher(XycarMotor, motor_topic, 10)
        self.ai_enable_pub = self.create_publisher(Bool, ai_enable_topic, 10)
        self.ai_speed_limit_pub = self.create_publisher(Float32, ai_speed_limit_topic, 10)
        self.nearest_obstacle_pub = self.create_publisher(
            Float32, '/track_drive/nearest_obstacle_distance', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/track_drive/markers', 10)
        self.light_debug_pub = self.create_publisher(Image, light_debug_image_topic, 10)
        self.create_subscription(Image, camera_topic, self.cam_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(XycarMotor, ai_command_topic, self.ai_motor_callback, 10)
        self.create_subscription(Bool, hybrid_trigger_topic, self.hybrid_trigger_callback, 10)

        self.get_logger().info(
            f'Track driver ready | camera={camera_topic}, scan={scan_topic}, motor={motor_topic}'
        )

    def _load_ai_model(self):
        if not bool(self.get_parameter('ai_hybrid_enabled').value):
            return

        model_path = Path(str(self.get_parameter('ai_model_path').value)).expanduser()
        if not model_path.exists():
            self.get_logger().warn(f'AI model not found: {model_path} | using rule fallback')
            return

        try:
            import torch
            self.ai_torch = torch
            self.ai_device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.ai_model = torch.jit.load(str(model_path), map_location=self.ai_device)
            self.ai_model.eval()
            self.get_logger().info(f'AI hybrid enabled | model={model_path}, device={self.ai_device}')
        except Exception as exc:
            self.ai_model = None
            self.get_logger().warn(f'AI model load failed: {exc} | using rule fallback')

    def cam_callback(self, data: Image):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as exc:
            self.get_logger().warn(f'camera conversion failed: {exc}')

    def lidar_callback(self, msg: LaserScan):
        self.scan_msg = msg

    def hybrid_trigger_callback(self, msg: Bool):
        self.hybrid_trigger_active = bool(msg.data)

    def ai_motor_callback(self, msg: XycarMotor):
        self.last_ai_motor_msg = msg
        if self._can_relay_ai_motor_immediately():
            self.drive(angle=msg.angle, speed=msg.speed)

    def drive(self, angle: float, speed: float):
        if not rclpy.ok():
            return

        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(self._apply_school_zone_speed_limit(speed))
        try:
            self.motor_pub.publish(self.motor_msg)
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().warn(f'motor publish failed: {exc}')

    def _apply_school_zone_speed_limit(self, speed: float) -> float:
        speed = float(speed)
        if not self.school_zone_active:
            return speed
        limit = max(float(self.get_parameter('school_zone_speed').value), 0.0)
        if speed > 0.0:
            return min(speed, limit)
        return speed

    def publish_ai_enable(self, enabled: bool):
        self.last_ai_enable = bool(enabled)
        msg = Bool()
        msg.data = bool(enabled)
        self.ai_enable_pub.publish(msg)

    def publish_ai_speed_limit(self):
        msg = Float32()
        if self.school_zone_active or (
            self.school_zone_candidate_active
            and bool(self.get_parameter('school_zone_preslow_enabled').value)
        ):
            msg.data = max(float(self.get_parameter('school_zone_speed').value), 0.0)
        else:
            msg.data = -1.0
        self.ai_speed_limit_pub.publish(msg)

    def main_loop(self):
        self.get_logger().info('START DRIVING: lane/cone local lattice planner enabled')

        rate_hz = max(float(self.get_parameter('control_rate_hz').value), 1.0)
        period = 1.0 / rate_hz

        while rclpy.ok():
            start = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.0)
            self.control_once()

            elapsed = time.monotonic() - start
            if elapsed < period:
                time.sleep(period - elapsed)

    def control_once(self):
        if bool(self.get_parameter('yolo_safety_enabled').value):
            self._update_yolo_safety_cache(self.image)
        self._update_school_zone_state(self.image)
        self.publish_ai_speed_limit()

        command_passthrough_enabled = bool(self.get_parameter('ai_command_passthrough_enabled').value)
        if command_passthrough_enabled and self._can_relay_ai_motor_immediately():
            self.last_mode = 'ai_command_passthrough'
            if self.last_ai_motor_msg is None:
                self._log_status('waiting_ai_command', 0.0, 0.0, 0, None, 0, None)
            else:
                self._log_status(
                    self.last_mode, self.last_ai_motor_msg.speed,
                    self.last_ai_motor_msg.angle, 0, None, 0, None)
            return

        nearest_obstacle = self._nearest_scan_obstacle_distance(self.scan_msg)
        self._publish_nearest_obstacle_distance(nearest_obstacle)
        force_rule_hybrid = False
        standby_enabled = bool(self.get_parameter('hybrid_standby_enabled').value)
        safety_stop, safety_reason = self._detect_safety_stop()
        if safety_stop:
            if standby_enabled:
                self.publish_ai_enable(False)
            self.prev_steer *= 0.5
            self.last_mode = safety_reason
            self.drive(angle=0.0, speed=0.0)
            self._log_status(self.last_mode, 0.0, 0.0, 0, None, 0, nearest_obstacle)
            return

        if self._startup_light_check_pending():
            if standby_enabled:
                self.publish_ai_enable(False)
            self.prev_steer *= 0.5
            self.last_mode = 'startup_light_check'
            self.drive(angle=0.0, speed=0.0)
            self._log_status(self.last_mode, 0.0, 0.0, 0, None, 0, nearest_obstacle)
            return

        person_avoidance_active = self._person_avoidance_requested()
        person_reverse_active = self._person_reverse_requested(person_avoidance_active)
        if person_reverse_active:
            if standby_enabled:
                self.publish_ai_enable(False)
            steer = float(np.clip(
                self.get_parameter('person_reverse_steer_deg').value,
                -float(self.get_parameter('max_steer_deg').value),
                float(self.get_parameter('max_steer_deg').value),
            ))
            speed = -abs(float(self.get_parameter('person_reverse_speed').value))
            self.prev_steer = steer
            self.last_mode = 'reverse_person_close'
            self.drive(angle=steer, speed=speed)
            self._log_status(self.last_mode, speed, steer, 0, None, 0, nearest_obstacle)
            return

        if not person_avoidance_active:
            self._update_vehicle_lidar_fallback(self.scan_msg)
        vehicle_rule_requested = self._vehicle_rule_requested()
        school_zone_takeover_requested = self._school_zone_takeover_requested()
        if person_avoidance_active:
            if standby_enabled:
                self.publish_ai_enable(False)
            self.prev_steer *= 0.5
            self.last_mode = 'stop_person_wait_left'
            self.drive(angle=0.0, speed=0.0)
            self._log_status(self.last_mode, 0.0, 0.0, 0, None, 0, nearest_obstacle)
            return

        if standby_enabled:
            use_hybrid = (
                self._should_use_hybrid_mode(nearest_obstacle)
                or person_avoidance_active
                or vehicle_rule_requested
                or school_zone_takeover_requested
            )
            if not use_hybrid:
                self.publish_ai_enable(True)
                self.last_mode = 'ai_direct_standby'
                self._log_status(self.last_mode, 0.0, 0.0, 0, None, 0, nearest_obstacle)
                return
            self.publish_ai_enable(False)
            force_rule_hybrid = True

        if command_passthrough_enabled:
            use_hybrid = (
                self._should_use_hybrid_mode(nearest_obstacle)
                or person_avoidance_active
                or vehicle_rule_requested
                or school_zone_takeover_requested
            )
            if not use_hybrid:
                if self.last_ai_motor_msg is not None:
                    self.drive(
                        angle=self.last_ai_motor_msg.angle,
                        speed=self.last_ai_motor_msg.speed,
                    )
                    self.last_mode = 'ai_command_passthrough'
                    self._log_status(
                        self.last_mode, self.last_ai_motor_msg.speed,
                        self.last_ai_motor_msg.angle, 0, None,
                        0, nearest_obstacle)
                else:
                    self.drive(angle=0.0, speed=0.0)
                    self._log_status(
                        'waiting_ai_command', 0.0, 0.0, 0, None,
                        0, nearest_obstacle)
                return

        passthrough_enabled = bool(self.get_parameter('ai_passthrough_enabled').value)
        use_hybrid = (
            self._should_use_hybrid_mode(nearest_obstacle)
            or person_avoidance_active
            or vehicle_rule_requested
            or school_zone_takeover_requested
        )
        ai_steer = None if (force_rule_hybrid or person_avoidance_active) else self._predict_ai_steer(
            self.image,
            passthrough=passthrough_enabled and not use_hybrid,
        )

        if ai_steer is not None and passthrough_enabled and not use_hybrid:
            speed = max(float(self.get_parameter('ai_speed').value), 0.0)
            self.prev_steer = ai_steer
            self.last_mode = 'ai_passthrough'
            self.drive(angle=ai_steer, speed=speed)
            self._log_status(
                self.last_mode, speed, ai_steer, 0, None,
                0, nearest_obstacle)
            return

        raw_cones = self._extract_cones_from_scan(self.scan_msg)
        cones = raw_cones
        white_lane_path = self._build_lane_center_path(self.image)
        school_zone_path = (
            self._build_school_zone_center_path(self.image)
            if school_zone_takeover_requested
            else None
        )
        lane_guard_path = self._lane_guard_reference_path(white_lane_path)
        lane_path = school_zone_path if school_zone_path is not None else white_lane_path
        cone_path = None if school_zone_path is not None else self._build_cone_center_path(cones)
        if school_zone_path is not None:
            center_path = school_zone_path
            self.last_center_path = school_zone_path
            self.last_mode = 'school_zone_yellow_line'
        else:
            center_path = self._fuse_center_paths(lane_path, cone_path)
        vehicle_behavior: Optional[VehicleBehavior] = None
        if person_avoidance_active:
            avoidance_path = self._build_person_avoidance_path(center_path)
            if len(avoidance_path) >= 2:
                center_path = avoidance_path
                self.last_center_path = avoidance_path
                self.last_mode = 'person_avoidance'
            else:
                self.prev_steer *= 0.5
                self.drive(angle=self.prev_steer, speed=0.0)
                self._log_status(
                    'person_avoidance_no_path', 0.0, self.prev_steer, len(cones), None,
                    len(raw_cones), nearest_obstacle)
                return
        elif school_zone_path is not None:
            vehicle_behavior = None
        else:
            vehicle_behavior = self._vehicle_behavior(center_path, nearest_obstacle)
            if vehicle_behavior is not None:
                ai_steer = None
                if vehicle_behavior.path is not None and len(vehicle_behavior.path) >= 2:
                    guarded_path = vehicle_behavior.path
                    if lane_guard_path is not None:
                        guarded_path = self._clamp_path_to_white_lane(guarded_path, lane_guard_path)
                    center_path = guarded_path
                    self.last_center_path = guarded_path
                self.last_mode = vehicle_behavior.mode

        if school_zone_path is not None:
            candidates = [PathCandidate(offset=0.0, path=list(center_path), cost=0.0, min_clearance=float('inf'))]
        elif vehicle_behavior is not None and vehicle_behavior.path is not None:
            candidates = [PathCandidate(offset=0.0, path=list(center_path), cost=0.0, min_clearance=float('inf'))]
        else:
            candidates = self._make_lattice_candidates(center_path)
        best = self._select_best_candidate(candidates, cones)
        if person_avoidance_active and best is not None:
            best.offset = self.person_lattice_selected_offset

        if ai_steer is not None:
            min_clearance = best.min_clearance if best is not None else float('inf')
            speed = self._target_ai_speed(ai_steer, cones, min_clearance)
            self._publish_debug_visualization(
                raw_cones, cones, lane_path, cone_path, center_path,
                best.path if best is not None else None)

            self.prev_steer = ai_steer
            self.last_candidates = candidates
            self.drive(angle=ai_steer, speed=speed)
            self._log_status(
                f'ai+{self.last_mode}', speed, ai_steer, len(cones), best,
                len(raw_cones), nearest_obstacle)
            return

        if best is None:
            self._publish_debug_visualization(
                raw_cones, cones, lane_path, cone_path, center_path, None)
            self.prev_steer *= 0.5
            self.drive(angle=self.prev_steer, speed=0.0)
            self._log_status(
                'stop', 0.0, self.prev_steer, len(cones), None,
                len(raw_cones), nearest_obstacle)
            return

        if person_avoidance_active:
            steer = self._pure_pursuit_steer(
                best.path,
                max_steer_override=float(self.get_parameter('person_avoidance_max_steer_deg').value),
                smoothing_override=float(self.get_parameter('person_avoidance_steer_smoothing').value),
            )
            steer = self._boost_person_avoidance_steer(steer, best.path)
            speed = self._target_person_avoidance_speed(nearest_obstacle)
            self.last_mode = 'person_avoidance'
        elif vehicle_behavior is not None:
            steer = self._pure_pursuit_steer(
                best.path,
                max_steer_override=vehicle_behavior.max_steer_deg,
                smoothing_override=vehicle_behavior.steer_smoothing,
                lookahead_scale_override=vehicle_behavior.lookahead_scale,
            )
            if vehicle_behavior.mode == 'vehicle_overtake':
                steer = self._boost_vehicle_overtake_steer(steer, best.path)
            elif vehicle_behavior.mode.startswith('vehicle_follow_'):
                steer = self._boost_vehicle_follow_steer(steer, best.path)
            steer = self._lane_edge_steer_guard(steer, best.path, lane_guard_path)
            speed = vehicle_behavior.speed
            self.last_mode = vehicle_behavior.mode
        elif school_zone_path is not None:
            steer = self._pure_pursuit_steer(
                best.path,
                max_steer_override=float(self.get_parameter('school_zone_max_steer_deg').value),
                smoothing_override=float(self.get_parameter('school_zone_steer_smoothing').value),
                lookahead_scale_override=float(self.get_parameter('school_zone_lookahead_scale').value),
            )
            speed = self._target_speed(steer, cones, best.min_clearance)
            self.last_mode = 'school_zone_yellow_line'
        else:
            steer = self._pure_pursuit_steer(best.path)
            steer = self._lane_edge_steer_guard(steer, best.path, lane_guard_path)
            speed = self._target_speed(steer, cones, best.min_clearance)
        self._publish_debug_visualization(
            raw_cones, cones, lane_path, cone_path, center_path, best.path)

        self.prev_offset = best.offset
        self.prev_steer = steer
        self.last_candidates = candidates

        self.drive(angle=steer, speed=speed)
        self._log_status(
            self.last_mode, speed, steer, len(cones), best,
            len(raw_cones), nearest_obstacle)

    def _should_use_hybrid_mode(self, nearest_obstacle: Optional[float]) -> bool:
        if self.hybrid_trigger_active:
            return True

        if not bool(self.get_parameter('hybrid_on_obstacle_enabled').value):
            return False

        if nearest_obstacle is None:
            return False

        trigger_distance = max(float(self.get_parameter('hybrid_obstacle_distance').value), 0.0)
        return nearest_obstacle < trigger_distance

    def _school_zone_takeover_requested(self) -> bool:
        return (
            self.school_zone_active
            and bool(self.get_parameter('school_zone_takeover_enabled').value)
        )

    def _can_relay_ai_motor_immediately(self) -> bool:
        if not bool(self.get_parameter('ai_command_passthrough_enabled').value):
            return False
        if self.hybrid_trigger_active:
            return False
        if self._school_zone_takeover_requested():
            return False
        return not bool(self.get_parameter('hybrid_on_obstacle_enabled').value)

    def _target_ai_speed(self, steer: float, cones: Sequence[Point], min_clearance: float) -> float:
        straight_speed = max(float(self.get_parameter('ai_speed').value), 0.0)
        if bool(self.get_parameter('ai_fixed_speed_enabled').value):
            speed = straight_speed
        else:
            curve_speed = max(float(self.get_parameter('ai_curve_speed').value), 0.0)
            start_steer = max(float(self.get_parameter('ai_curve_start_steer_deg').value), 0.0)
            full_steer = max(float(self.get_parameter('ai_curve_full_steer_deg').value), start_steer + 1.0)

            curve_ratio = float(np.clip((abs(steer) - start_steer) / (full_steer - start_steer), 0.0, 1.0))
            curve_ratio = curve_ratio * curve_ratio * (3.0 - 2.0 * curve_ratio)
            speed = straight_speed + (curve_speed - straight_speed) * curve_ratio

        stop_distance = float(self.get_parameter('stop_distance').value)
        slow_distance = float(self.get_parameter('slow_distance').value)
        front_half_width = float(self.get_parameter('front_obstacle_half_width').value)
        nearest_front = min(
            [p[0] for p in cones if abs(p[1]) < front_half_width],
            default=float('inf'),
        )

        if nearest_front < stop_distance:
            return 0.0
        if nearest_front < slow_distance or min_clearance < 0.55:
            speed = min(speed, float(self.get_parameter('min_speed').value))

        smoothing = float(np.clip(self.get_parameter('ai_speed_smoothing').value, 0.0, 0.95))
        if self.prev_ai_speed is not None:
            speed = (1.0 - smoothing) * speed + smoothing * self.prev_ai_speed
        self.prev_ai_speed = speed

        return speed

    def _target_person_avoidance_speed(self, nearest_obstacle: Optional[float]) -> float:
        close_stop = max(float(self.get_parameter('person_avoidance_close_stop_distance').value), 0.0)
        if close_stop > 0.0 and nearest_obstacle is not None and nearest_obstacle < close_stop:
            return 0.0

        boost_speed = max(float(self.get_parameter('person_avoidance_boost_speed').value), 0.0)
        boost_sec = max(float(self.get_parameter('person_avoidance_boost_sec').value), 0.0)
        if self.person_avoidance_started_sec is not None:
            if time.monotonic() - self.person_avoidance_started_sec < boost_sec:
                return boost_speed

        return max(float(self.get_parameter('person_avoidance_speed').value), 0.0)

    def _predict_ai_steer(self, image: Optional[np.ndarray], passthrough: bool = False) -> Optional[float]:
        if self.ai_model is None or self.ai_torch is None:
            return None
        if image is None:
            self._log_ai_warning('waiting_for_image')
            return None

        try:
            tensor_np = self._preprocess_ai_image(image)
            x = self.ai_torch.from_numpy(tensor_np).unsqueeze(0).to(self.ai_device)
            with self.ai_torch.no_grad():
                pred_norm = float(self.ai_model(x).detach().cpu().numpy().reshape(-1)[0])
        except Exception as exc:
            self._log_ai_warning(f'predict_failed: {exc}')
            return None

        max_steer = float(self.get_parameter('ai_max_steer_deg').value)
        steer = float(np.clip(pred_norm * max_steer, -max_steer, max_steer))
        if bool(self.get_parameter('ai_invert_steering').value):
            steer = -steer

        smoothing = float(np.clip(self.get_parameter('ai_steer_smoothing').value, 0.0, 0.95))
        steer = (1.0 - smoothing) * steer + smoothing * self.prev_ai_steer

        if not passthrough:
            deadband = max(float(self.get_parameter('ai_steer_deadband_deg').value), 0.0)
            if abs(steer) < deadband:
                steer = 0.0

            max_step = max(float(self.get_parameter('ai_max_steer_step_deg').value), 0.0)
            if max_step > 0.0:
                steer_delta = float(np.clip(steer - self.prev_ai_steer, -max_step, max_step))
                steer = self.prev_ai_steer + steer_delta

        self.prev_ai_steer = steer
        return steer

    def _preprocess_ai_image(self, image_bgr: np.ndarray) -> np.ndarray:
        height, _ = image_bgr.shape[:2]
        roi_top_ratio = float(self.get_parameter('ai_roi_top_ratio').value)
        roi_top = int(np.clip(roi_top_ratio, 0.0, 0.9) * height)
        roi = image_bgr[roi_top:height, :]
        if roi.size == 0:
            roi = image_bgr

        width = int(self.get_parameter('ai_resize_width').value)
        height_out = int(self.get_parameter('ai_resize_height').value)
        resized = cv2.resize(roi, (width, height_out), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))

    def _log_ai_warning(self, reason: str):
        now_sec = self.get_clock().now().nanoseconds // 1_000_000_000
        if now_sec == self._ai_last_warn_sec:
            return
        self._ai_last_warn_sec = now_sec
        self.get_logger().info(f'AI fallback to rule: {reason}')

    def _extract_cones_from_scan(self, msg: Optional[LaserScan]) -> List[Point]:
        if msg is None or not msg.ranges:
            return []

        min_range = float(self.get_parameter('scan_min_range').value)
        max_range = float(self.get_parameter('scan_max_range').value)
        lateral_limit = float(self.get_parameter('scan_lateral_limit').value)
        front_index_param = int(self.get_parameter('scan_front_index').value)
        scan_reverse = bool(self.get_parameter('scan_reverse').value)
        angle_offset = float(self.get_parameter('scan_angle_offset').value)
        front_index = front_index_param if front_index_param >= 0 else (len(msg.ranges) // 2)
        angle_step = abs(float(msg.angle_increment)) if abs(float(msg.angle_increment)) > 1e-6 else math.radians(1.0)

        points: List[Point] = []
        for idx, r in enumerate(msg.ranges):
            if math.isfinite(r) and min_range <= r <= max_range:
                scan_dir = -1.0 if scan_reverse else 1.0
                scan_angle = scan_dir * (idx - front_index) * angle_step + angle_offset
                x = r * math.cos(scan_angle)
                y = r * math.sin(scan_angle)
                if x > 0.05 and abs(y) <= lateral_limit:
                    points.append((x, y))

        if not points:
            return []

        points.sort(key=lambda p: (math.atan2(p[1], p[0]), p[0]))
        gap = float(self.get_parameter('cone_cluster_gap').value)
        min_points = int(self.get_parameter('cone_cluster_min_points').value)
        max_size = float(self.get_parameter('cone_cluster_max_size').value)

        clusters: List[List[Point]] = []
        current = [points[0]]
        for point in points[1:]:
            if math.hypot(point[0] - current[-1][0], point[1] - current[-1][1]) <= gap:
                current.append(point)
            else:
                clusters.append(current)
                current = [point]
        clusters.append(current)

        cones: List[Point] = []
        for cluster in clusters:
            if len(cluster) < min_points:
                continue

            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            width = max(xs) - min(xs)
            height = max(ys) - min(ys)
            if max(width, height) > max_size:
                continue

            cones.append((float(np.mean(xs)), float(np.mean(ys))))

        cones.sort(key=lambda p: p[0])
        return cones

    @staticmethod
    def _nearest_obstacle_distance(points: Sequence[Point]) -> Optional[float]:
        if not points:
            return None
        return min(math.hypot(x, y) for x, y in points)

    def _nearest_scan_obstacle_distance(self, msg: Optional[LaserScan]) -> Optional[float]:
        if msg is None or not msg.ranges:
            return None

        min_range = float(self.get_parameter('scan_min_range').value)
        min_x = max(float(self.get_parameter('nearest_obstacle_min_x').value), 0.05)
        max_range = float(self.get_parameter('scan_max_range').value)
        lateral_limit = float(self.get_parameter('scan_lateral_limit').value)
        front_index_param = int(self.get_parameter('scan_front_index').value)
        scan_reverse = bool(self.get_parameter('scan_reverse').value)
        angle_offset = float(self.get_parameter('scan_angle_offset').value)
        front_index = front_index_param if front_index_param >= 0 else (len(msg.ranges) // 2)
        angle_step = abs(float(msg.angle_increment)) if abs(float(msg.angle_increment)) > 1e-6 else math.radians(1.0)
        scan_dir = -1.0 if scan_reverse else 1.0

        nearest = None
        for idx, r in enumerate(msg.ranges):
            if not math.isfinite(r) or not (min_range <= r <= max_range):
                continue

            scan_angle = scan_dir * (idx - front_index) * angle_step + angle_offset
            x = r * math.cos(scan_angle)
            y = r * math.sin(scan_angle)
            if x <= min_x or abs(y) > lateral_limit:
                continue

            if nearest is None or r < nearest:
                nearest = float(r)

        return nearest

    def _publish_nearest_obstacle_distance(self, distance: Optional[float]):
        msg = Float32()
        msg.data = -1.0 if distance is None else float(distance)
        self.nearest_obstacle_pub.publish(msg)

    def _detect_safety_stop(self) -> Tuple[bool, str]:
        now = time.monotonic()
        detected_reason = None

        if bool(self.get_parameter('yolo_safety_enabled').value):
            self._update_yolo_safety_cache(self.image)

        if self._detect_red_light(self.image):
            detected_reason = 'stop_red_light'
        elif self._detect_vehicle(self.image):
            detected_reason = 'stop_vehicle'

        if detected_reason is not None:
            hold_sec = max(float(self.get_parameter('safety_stop_hold_sec').value), 0.0)
            self.safety_stop_until_sec = max(self.safety_stop_until_sec, now + hold_sec)
            self.last_safety_stop_reason = detected_reason

        if now < self.safety_stop_until_sec:
            return True, self.last_safety_stop_reason
        return False, ''

    def _startup_light_check_pending(self) -> bool:
        if not bool(self.get_parameter('startup_light_check_enabled').value):
            return False
        if not bool(self.get_parameter('stop_on_red_light_enabled').value):
            return False

        now = time.monotonic()
        timeout_sec = max(float(self.get_parameter('startup_light_check_timeout_sec').value), 0.0)
        if timeout_sec > 0.0 and now - self.startup_time_sec > timeout_sec:
            return False

        if self.image is None:
            return True
        if not self.yolo_light_checked_once:
            return True

        required_frames = max(int(self.get_parameter('red_light_confirm_frames').value), 1)
        return 0 < self.red_light_confirm_count < required_frames

    def _person_avoidance_requested(self) -> bool:
        if not bool(self.get_parameter('person_avoidance_enabled').value):
            return False

        now = time.monotonic()
        if self.red_light_confirm_count > 0:
            self._reset_person_wait_state()
            return False

        if self._detect_person(self.image, self.scan_msg):
            release_ready = self._person_wait_release_ready()
            if release_ready and not self.person_avoidance_active:
                self._reset_person_wait_state()
                return False

            if not self.person_avoidance_active:
                self.person_avoidance_started_sec = now
            self.person_avoidance_active = True
            self.person_wait_last_seen_sec = now
            self.person_avoidance_offset_sign = self._person_avoidance_offset_sign()

            if release_ready:
                self.person_wait_left_confirm_count += 1
            else:
                self.person_wait_left_confirm_count = 0

            required_frames = max(int(self.get_parameter('person_wait_release_confirm_frames').value), 1)
            if self.person_wait_left_confirm_count >= required_frames:
                self._reset_person_wait_state()
                return False

            return True

        if self.person_avoidance_active:
            lost_release_sec = max(float(self.get_parameter('person_wait_lost_release_sec').value), 0.0)
            if (
                self.person_wait_last_seen_sec is not None
                and now - self.person_wait_last_seen_sec < lost_release_sec
            ):
                self._refresh_person_prediction(now)
                return True

        self._reset_person_wait_state()
        return False

    def _person_wait_release_ready(self) -> bool:
        left_y = float(self.get_parameter('person_wait_release_left_y').value)
        point = self.person_fusion_point
        if point is not None and point[1] >= left_y:
            return True

        image_left_ratio = float(self.get_parameter('person_wait_release_image_left_ratio').value)
        if self.person_image_center_ratio is not None and self.person_image_center_ratio <= image_left_ratio:
            return True

        return False

    def _person_reverse_requested(self, person_detected: bool) -> bool:
        if not bool(self.get_parameter('person_reverse_enabled').value):
            self.person_reverse_until_sec = 0.0
            return False
        if not person_detected:
            self.person_reverse_until_sec = 0.0
            return False

        point = self.person_fusion_point
        if point is None or self.person_fusion_distance is None:
            return False

        distance = float(self.person_fusion_distance)
        lateral_limit = max(float(self.get_parameter('person_lateral_limit').value), 0.05)
        if abs(point[1]) > lateral_limit:
            self.person_reverse_until_sec = 0.0
            return False

        now = time.monotonic()
        reverse_distance = max(float(self.get_parameter('person_reverse_distance').value), 0.05)
        release_distance = max(
            float(self.get_parameter('person_reverse_release_distance').value),
            reverse_distance + 0.05,
        )
        if now < self.person_reverse_until_sec:
            if distance >= release_distance:
                self.person_reverse_until_sec = 0.0
                return False
            return True

        if distance <= reverse_distance:
            max_sec = max(float(self.get_parameter('person_reverse_max_sec').value), 0.05)
            self.person_reverse_until_sec = now + max_sec
            return True

        return False

    def _reset_person_wait_state(self):
        self.person_avoidance_until_sec = 0.0
        self.person_avoidance_active = False
        self.person_avoidance_started_sec = None
        self.person_reverse_until_sec = 0.0
        self.person_lattice_selected_offset = 0.0
        self.person_lattice_should_stop = False
        self.person_wait_left_confirm_count = 0
        self.person_wait_last_seen_sec = None

    def _detect_red_light(self, image: Optional[np.ndarray]) -> bool:
        if not bool(self.get_parameter('stop_on_red_light_enabled').value):
            return False
        if image is None:
            return False

        if bool(self.get_parameter('yolo_safety_enabled').value):
            if self.cached_yolo_red_light:
                return True
            if not bool(self.get_parameter('red_light_color_fallback_enabled').value):
                return False

        height, width = image.shape[:2]
        top_ratio = float(np.clip(self.get_parameter('red_light_roi_top_ratio').value, 0.0, 0.95))
        bottom_ratio = float(np.clip(self.get_parameter('red_light_roi_bottom_ratio').value, top_ratio + 0.05, 1.0))
        y0 = int(top_ratio * height)
        y1 = int(bottom_ratio * height)
        roi = image[y0:y1, :]
        if roi.size == 0:
            return False

        return self._red_light_color_present(roi)

    def _detect_person(self, image: Optional[np.ndarray], scan: Optional[LaserScan]) -> bool:
        if not bool(self.get_parameter('stop_on_person_enabled').value):
            return False

        if bool(self.get_parameter('yolo_safety_enabled').value) and self.cached_yolo_person:
            if bool(self.get_parameter('person_fusion_enabled').value):
                if self._detect_person_from_camera_lidar(scan):
                    return True
                return bool(self.get_parameter('person_yolo_camera_fallback_enabled').value)
            return True

        if bool(self.get_parameter('person_lidar_fallback_enabled').value) and self._detect_person_from_scan(scan):
            return True

        if bool(self.get_parameter('person_camera_enabled').value):
            return self._detect_person_from_image(image)

        return False

    def _detect_vehicle(self, image: Optional[np.ndarray]) -> bool:
        if not self._vehicle_processing_enabled():
            return False
        if not bool(self.get_parameter('stop_on_vehicle_enabled').value):
            return False
        if image is None:
            return False
        if not bool(self.get_parameter('yolo_safety_enabled').value):
            return False
        return self.cached_yolo_vehicle

    def _vehicle_rule_requested(self) -> bool:
        if not bool(self.get_parameter('yolo_safety_enabled').value):
            return False
        if not self._vehicle_processing_enabled():
            return False
        return self.cached_yolo_vehicle or self._has_yolo_confirmed_vehicle_track()

    def _vehicle_processing_enabled(self) -> bool:
        return (
            bool(self.get_parameter('stop_on_vehicle_enabled').value)
            or bool(self.get_parameter('vehicle_overtake_enabled').value)
            or bool(self.get_parameter('vehicle_follow_enabled').value)
        )

    def _update_yolo_safety_cache(self, image: Optional[np.ndarray]):
        if image is None:
            self.cached_yolo_person = False
            self.cached_yolo_person_detections = []
            self.cached_yolo_person_box = None
            self.cached_yolo_vehicle = False
            self.cached_yolo_vehicle_raw_count = 0
            self.cached_yolo_vehicle_detections = []
            self.cached_yolo_vehicle_box = None
            self.cached_yolo_red_light = False
            self.cached_yolo_go_light = False
            self.cached_yolo_raw_red_light = False
            self.red_light_confirm_count = 0
            return

        now = time.monotonic()
        safety_due = now >= self.next_yolo_safety_check_sec
        light_enabled = bool(self.get_parameter('stop_on_red_light_enabled').value)
        light_due = light_enabled and now >= self.next_yolo_light_check_sec
        if not safety_due and not light_due:
            return

        vehicle_enabled = self._vehicle_processing_enabled()
        if not vehicle_enabled:
            self.vehicle_tracks.clear()

        if safety_due:
            period = max(float(self.get_parameter('yolo_safety_period_sec').value), 0.02)
            self.next_yolo_safety_check_sec = now + period
            self.cached_yolo_person = False
            self.cached_yolo_person_detections = []
            self.cached_yolo_person_box = None
            self.cached_yolo_vehicle = False
            self.cached_yolo_vehicle_raw_count = 0
            self.cached_yolo_vehicle_detections = []
            self.cached_yolo_vehicle_box = None

        if safety_due and (bool(self.get_parameter('stop_on_person_enabled').value) or vehicle_enabled):
            safety_detections = self._run_yolo_detector(
                'safety',
                str(self.get_parameter('yolo_person_model_path').value),
                int(self.get_parameter('yolo_person_input_size').value),
                float(self.get_parameter('yolo_person_conf_threshold').value),
                image,
                int(self.get_parameter('yolo_person_class_count').value),
            )
            person_class_ids = self._int_set_parameter('yolo_person_class_ids')
            valid_person_detections = [
                detection
                for detection in safety_detections
                if self._class_id_allowed(detection[2], person_class_ids)
                and self._valid_person_detection(image, detection[0])
            ]
            self.cached_yolo_person = bool(valid_person_detections)
            if valid_person_detections:
                valid_person_detections.sort(key=lambda item: item[1], reverse=True)
                self.cached_yolo_person_detections = valid_person_detections
                self.cached_yolo_person_box = valid_person_detections[0][0]
                self._update_person_image_motion(self.cached_yolo_person_box, image.shape[1])

            if vehicle_enabled:
                vehicle_class_ids = self._int_set_parameter('yolo_vehicle_class_ids')
                raw_vehicle_detections = [
                    detection
                    for detection in safety_detections
                    if self._class_id_allowed(detection[2], vehicle_class_ids)
                ]
                self.cached_yolo_vehicle_raw_count = len(raw_vehicle_detections)
                valid_vehicle_detections = [
                    detection
                    for detection in raw_vehicle_detections
                    if self._valid_vehicle_detection(image, detection[0])
                ]
                self.cached_yolo_vehicle = bool(valid_vehicle_detections)
                if valid_vehicle_detections:
                    valid_vehicle_detections.sort(key=lambda item: item[1], reverse=True)
                    self.cached_yolo_vehicle_detections = valid_vehicle_detections
                    self.cached_yolo_vehicle_box = valid_vehicle_detections[0][0]
                self._update_vehicle_tracks(image, self.scan_msg, valid_vehicle_detections)
            else:
                self.vehicle_tracks.clear()

        if light_due:
            period = max(float(self.get_parameter('yolo_red_light_period_sec').value), 0.05)
            self.next_yolo_light_check_sec = now + period
            self.yolo_light_checked_once = True
            self.yolo_light_last_check_sec = now
            self.cached_yolo_red_light = False
            self.cached_yolo_go_light = False
            self.cached_yolo_raw_red_light = False
            light_detections = self._run_yolo_detector(
                'light',
                str(self.get_parameter('yolo_light_model_path').value),
                int(self.get_parameter('yolo_light_input_size').value),
                float(self.get_parameter('yolo_light_conf_threshold').value),
                image,
                int(self.get_parameter('yolo_light_class_count').value),
            )
            light_debug = []
            raw_red_light = False
            raw_go_light = False
            light_class_ids = self._int_set_parameter('yolo_light_class_ids')
            red_light_class_ids = self._int_set_parameter('yolo_red_light_class_ids')
            go_light_class_ids = self._int_set_parameter('yolo_go_light_class_ids')
            for box, score, class_id in light_detections:
                if not self._class_id_allowed(class_id, light_class_ids):
                    continue
                valid = self._valid_light_detection(image, box)
                red_present, red_ratio, green_ratio, yellow_ratio = self._red_light_box_metrics(image, box)
                is_stop_light_class = valid and self._class_id_allowed(class_id, red_light_class_ids)
                red_present = is_stop_light_class
                go_present = valid and self._class_id_allowed(class_id, go_light_class_ids)
                raw_red_light = raw_red_light or red_present
                raw_go_light = raw_go_light or go_present
                light_debug.append((
                    box, score, class_id, valid,
                    red_present, red_ratio, green_ratio, yellow_ratio,
                ))

            self.cached_yolo_go_light = raw_go_light
            if (
                raw_go_light
                and not raw_red_light
                and bool(self.get_parameter('red_light_go_release_enabled').value)
            ):
                raw_red_light = False
            self.cached_yolo_raw_red_light = raw_red_light

            if raw_red_light:
                self.red_light_confirm_count += 1
            else:
                self.red_light_confirm_count = 0

            required_frames = max(int(self.get_parameter('red_light_confirm_frames').value), 1)
            self.cached_yolo_red_light = self.red_light_confirm_count >= required_frames
            self._publish_light_debug_image(image, light_debug)
        elif not light_enabled:
            self.red_light_confirm_count = 0
            self.cached_yolo_red_light = False
            self.cached_yolo_go_light = False
            self.cached_yolo_raw_red_light = False
            self._publish_light_debug_image(image, [])

    def _run_yolo_detector(
        self,
        name: str,
        model_path_value: str,
        input_size: int,
        conf_threshold: float,
        image: np.ndarray,
        class_count: Optional[int] = None,
    ):
        model_path = Path(model_path_value).expanduser()
        if not model_path.exists():
            self._warn_yolo_once(name, f'YOLO model not found: {model_path}')
            return []

        input_size = max(int(input_size), 32)
        net = self._get_yolo_net(name, model_path)
        if net is None:
            return []

        padded, scale, pad_x, pad_y = self._letterbox_image(image, input_size)
        blob = cv2.dnn.blobFromImage(
            padded, scalefactor=1.0 / 255.0, size=(input_size, input_size),
            mean=(0.0, 0.0, 0.0), swapRB=True, crop=False)

        try:
            net.setInput(blob)
            output = net.forward()
        except Exception as exc:
            self._warn_yolo_once(name, f'YOLO inference failed: {exc}')
            return []

        return self._decode_yolo_output(
            output, image.shape[:2], scale, pad_x, pad_y,
            float(conf_threshold), float(self.get_parameter('yolo_nms_threshold').value),
            int(class_count) if class_count is not None and int(class_count) > 0 else None)

    def _int_set_parameter(self, name: str) -> Optional[Set[int]]:
        value = self.get_parameter(name).value
        if value is None:
            return None
        if isinstance(value, str):
            text = value.strip()
            if not text or text.lower() in ('all', 'any', '*'):
                return None
            value = text.strip('[]()').replace(';', ',').split(',')
        if isinstance(value, np.ndarray):
            value = value.tolist()
        if isinstance(value, (list, tuple)):
            ids = {int(item) for item in value if str(item).strip()}
            return ids or None
        return {int(value)}

    @staticmethod
    def _class_id_allowed(class_id: int, allowed_class_ids: Optional[Set[int]]) -> bool:
        return allowed_class_ids is None or int(class_id) in allowed_class_ids

    def _get_yolo_net(self, name: str, model_path: Path):
        key = (name, str(model_path))
        if key in self.yolo_nets:
            return self.yolo_nets[key]

        try:
            net = cv2.dnn.readNetFromONNX(str(model_path))
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        except Exception as exc:
            self._warn_yolo_once(name, f'YOLO model load failed: {model_path} | {exc}')
            return None

        self.yolo_nets[key] = net
        self.get_logger().info(f'YOLO {name} model loaded: {model_path}')
        return net

    @staticmethod
    def _letterbox_image(image: np.ndarray, input_size: int):
        height, width = image.shape[:2]
        scale = min(input_size / max(width, 1), input_size / max(height, 1))
        new_width = max(int(round(width * scale)), 1)
        new_height = max(int(round(height * scale)), 1)
        resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        padded = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
        pad_x = (input_size - new_width) // 2
        pad_y = (input_size - new_height) // 2
        padded[pad_y:pad_y + new_height, pad_x:pad_x + new_width] = resized
        return padded, scale, pad_x, pad_y

    @staticmethod
    def _decode_yolo_output(
        output,
        image_shape: Tuple[int, int],
        scale: float,
        pad_x: int,
        pad_y: int,
        conf_threshold: float,
        nms_threshold: float,
        class_count: Optional[int] = None,
    ):
        predictions = np.asarray(output)
        if predictions.ndim == 3:
            predictions = predictions[0]
        if predictions.ndim != 2 or predictions.shape[1] < 5:
            return []
        if predictions.shape[0] < predictions.shape[1] and predictions.shape[0] <= 32:
            predictions = predictions.T

        image_height, image_width = image_shape
        boxes = []
        scores = []
        class_ids = []

        for row in predictions:
            if len(row) < 5:
                continue

            obj_conf = float(row[4])
            class_id = 0
            if class_count is not None and len(row) == 4 + class_count:
                class_scores = row[4:4 + class_count]
                class_id = int(np.argmax(class_scores))
                score = float(class_scores[class_id])
            elif class_count is not None and len(row) >= 5 + class_count:
                class_scores = row[5:5 + class_count]
                class_id = int(np.argmax(class_scores))
                class_conf = float(class_scores[class_id])
                score = obj_conf * class_conf if obj_conf <= 1.0 and class_conf <= 1.0 else max(obj_conf, class_conf)
            elif len(row) == 5:
                score = obj_conf
            elif len(row) == 6 and float(row[5]) > 1.0 and obj_conf <= 1.0:
                score = obj_conf
                class_id = int(row[5])
            else:
                class_scores = row[5:]
                class_id = int(np.argmax(class_scores))
                class_conf = float(class_scores[class_id])
                score = obj_conf * class_conf if obj_conf <= 1.0 and class_conf <= 1.0 else max(obj_conf, class_conf)

            if score < conf_threshold:
                continue

            x, y, w, h = [float(v) for v in row[:4]]
            if len(row) == 6 and float(row[5]) > 1.0 and row[2] > row[0] and row[3] > row[1]:
                x0 = (x - pad_x) / max(scale, 1e-6)
                y0 = (y - pad_y) / max(scale, 1e-6)
                x1 = (w - pad_x) / max(scale, 1e-6)
                y1 = (h - pad_y) / max(scale, 1e-6)
            else:
                x0 = (x - 0.5 * w - pad_x) / max(scale, 1e-6)
                y0 = (y - 0.5 * h - pad_y) / max(scale, 1e-6)
                x1 = (x + 0.5 * w - pad_x) / max(scale, 1e-6)
                y1 = (y + 0.5 * h - pad_y) / max(scale, 1e-6)

            x0 = int(np.clip(x0, 0, image_width - 1))
            y0 = int(np.clip(y0, 0, image_height - 1))
            x1 = int(np.clip(x1, 0, image_width - 1))
            y1 = int(np.clip(y1, 0, image_height - 1))
            box_width = x1 - x0
            box_height = y1 - y0
            if box_width <= 2 or box_height <= 2:
                continue

            boxes.append([x0, y0, box_width, box_height])
            scores.append(float(score))
            class_ids.append(class_id)

        if not boxes:
            return []

        indices = cv2.dnn.NMSBoxes(boxes, scores, conf_threshold, nms_threshold)
        if len(indices) == 0:
            return []

        detections = []
        for idx in np.asarray(indices).reshape(-1):
            x, y, width, height = boxes[int(idx)]
            detections.append(((x, y, x + width, y + height), scores[int(idx)], class_ids[int(idx)]))
        return detections

    def _box_contains_red_light(self, image: np.ndarray, box: Tuple[int, int, int, int]) -> bool:
        return self._red_light_box_metrics(image, box)[0]

    def _red_light_box_metrics(
        self,
        image: np.ndarray,
        box: Tuple[int, int, int, int],
    ) -> Tuple[bool, float, float, float]:
        x0, y0, x1, y1 = box
        pad = 4
        x0 = max(x0 - pad, 0)
        y0 = max(y0 - pad, 0)
        x1 = min(x1 + pad, image.shape[1] - 1)
        y1 = min(y1 + pad, image.shape[0] - 1)
        crop = image[y0:y1, x0:x1]
        if crop.size == 0:
            return False, 0.0, 0.0, 0.0
        return self._red_light_color_metrics(crop)

    def _valid_person_detection(self, image: np.ndarray, box: Tuple[int, int, int, int]) -> bool:
        image_height, image_width = image.shape[:2]
        x0, y0, x1, y1 = box
        box_height = max(y1 - y0, 0)
        box_width = max(x1 - x0, 0)
        min_height_ratio = float(np.clip(
            self.get_parameter('yolo_person_min_box_height_ratio').value, 0.0, 1.0))
        min_bottom_ratio = float(np.clip(
            self.get_parameter('yolo_person_min_box_bottom_ratio').value, 0.0, 1.0))
        return (
            box_height >= min_height_ratio * max(image_height, 1)
            and box_width >= 0.02 * max(image_width, 1)
            and y1 >= min_bottom_ratio * max(image_height, 1)
        )

    def _valid_vehicle_detection(self, image: np.ndarray, box: Tuple[int, int, int, int]) -> bool:
        image_height, image_width = image.shape[:2]
        x0, y0, x1, y1 = box
        box_height = max(y1 - y0, 0)
        box_width = max(x1 - x0, 0)
        min_height_ratio = float(np.clip(
            self.get_parameter('yolo_vehicle_min_box_height_ratio').value, 0.0, 1.0))
        min_width_ratio = float(np.clip(
            self.get_parameter('yolo_vehicle_min_box_width_ratio').value, 0.0, 1.0))
        min_bottom_ratio = float(np.clip(
            self.get_parameter('yolo_vehicle_min_box_bottom_ratio').value, 0.0, 1.0))
        return (
            box_height >= min_height_ratio * max(image_height, 1)
            and box_width >= min_width_ratio * max(image_width, 1)
            and y1 >= min_bottom_ratio * max(image_height, 1)
        )

    def _update_vehicle_tracks(
        self,
        image: np.ndarray,
        scan: Optional[LaserScan],
        detections: Sequence[Tuple[Tuple[int, int, int, int], float, int]],
    ):
        now = time.monotonic()
        matched_ids: Set[int] = set()
        detections = sorted(detections, key=lambda item: item[1], reverse=True)

        for box, _, class_id in detections:
            point = self._vehicle_point_for_detection(image, scan, box)
            if point is None:
                continue

            track = self._upsert_vehicle_track(point, int(class_id), now, matched_ids, box)

            matched_ids.add(track.track_id)

        self._prune_vehicle_tracks(now)

    def _upsert_vehicle_track(
        self,
        point: Point,
        class_id: int,
        now: float,
        matched_ids: Set[int],
        box: Optional[Tuple[int, int, int, int]] = None,
    ) -> VehicleTrack:
        track = self._match_vehicle_track(point, int(class_id), now, matched_ids)
        if track is None:
            track_id = self.next_vehicle_track_id
            self.next_vehicle_track_id += 1
            track = VehicleTrack(
                track_id=track_id,
                class_id=int(class_id),
                x=point[0],
                y=point[1],
                last_seen_sec=now,
                yolo_last_seen_sec=now,
                seen_count=1,
                box=box,
            )
            self.vehicle_tracks[track_id] = track
            return track

        dt = now - track.last_seen_sec
        if 0.04 <= dt <= 1.50:
            raw_vx = (point[0] - track.x) / dt
            raw_vy = (point[1] - track.y) / dt
            smoothing = float(np.clip(
                self.get_parameter('vehicle_track_velocity_smoothing').value, 0.0, 0.95))
            track.vx = smoothing * track.vx + (1.0 - smoothing) * raw_vx
            track.vy = smoothing * track.vy + (1.0 - smoothing) * raw_vy
        track.class_id = int(class_id)
        track.x = point[0]
        track.y = point[1]
        track.last_seen_sec = now
        track.yolo_last_seen_sec = now
        track.seen_count += 1
        if box is not None:
            track.box = box
        return track

    def _match_vehicle_track(
        self,
        point: Point,
        class_id: int,
        now: float,
        matched_ids: Set[int],
    ) -> Optional[VehicleTrack]:
        timeout = max(float(self.get_parameter('vehicle_track_timeout_sec').value), 0.05)
        max_distance = max(float(self.get_parameter('vehicle_track_match_distance').value), 0.10)
        best_track = None
        best_cost = float('inf')

        for track in self.vehicle_tracks.values():
            if track.track_id in matched_ids:
                continue
            age = now - track.last_seen_sec
            if age > timeout:
                continue
            pred_x = track.x + track.vx * age
            pred_y = track.y + track.vy * age
            distance = math.hypot(point[0] - pred_x, point[1] - pred_y)
            if distance > max_distance:
                continue
            class_penalty = 0.20 if track.class_id != int(class_id) else 0.0
            cost = distance + class_penalty
            if cost < best_cost:
                best_cost = cost
                best_track = track

        return best_track

    def _prune_vehicle_tracks(self, now: Optional[float] = None):
        now = time.monotonic() if now is None else now
        timeout = max(float(self.get_parameter('vehicle_track_timeout_sec').value), 0.05)
        yolo_timeout = max(float(self.get_parameter('vehicle_yolo_required_timeout_sec').value), 0.02)
        stale_ids = []
        for track_id, track in self.vehicle_tracks.items():
            if (
                now - track.last_seen_sec <= timeout
                and now - track.yolo_last_seen_sec <= yolo_timeout
            ):
                continue
            stale_ids.append(track_id)
        for track_id in stale_ids:
            self.vehicle_tracks.pop(track_id, None)

        if self.vehicle_overtake_track_id is not None and self.vehicle_overtake_track_id not in self.vehicle_tracks:
            started = self.vehicle_overtake_started_sec or now
            min_time = max(float(self.get_parameter('vehicle_overtake_min_time_sec').value), 0.0)
            if now - started >= min_time:
                self._reset_vehicle_overtake_state()

    def _vehicle_point_for_detection(
        self,
        image: np.ndarray,
        scan: Optional[LaserScan],
        box: Tuple[int, int, int, int],
    ) -> Optional[Point]:
        point = self._vehicle_lidar_point_for_camera_box(scan, box)
        if point is not None:
            return point

        if bool(self.get_parameter('vehicle_camera_fallback_enabled').value):
            return self._vehicle_point_from_box(image, box)
        return None

    def _vehicle_lidar_point_for_camera_box(
        self,
        msg: Optional[LaserScan],
        box: Tuple[int, int, int, int],
    ) -> Optional[Point]:
        if msg is None or not msg.ranges or self.image is None:
            return None

        image_height, image_width = self.image.shape[:2]
        x0, _, x1, _ = box
        center_x = 0.5 * (x0 + x1)
        box_width = max(x1 - x0, 1)

        camera_fov = math.radians(max(float(self.get_parameter('vehicle_fusion_camera_fov_deg').value), 1.0))
        box_center_ratio = (center_x - 0.5 * image_width) / max(0.5 * image_width, 1.0)
        camera_angle = -box_center_ratio * 0.5 * camera_fov
        half_box_angle = 0.5 * camera_fov * (box_width / max(image_width, 1))
        angle_margin = math.radians(max(float(self.get_parameter('vehicle_fusion_angle_margin_deg').value), 0.0))
        max_angle_error = max(half_box_angle + angle_margin, math.radians(3.0))

        min_range = float(self.get_parameter('scan_min_range').value)
        min_x = max(float(self.get_parameter('vehicle_fusion_min_x').value), 0.05)
        max_range = min(
            float(self.get_parameter('scan_max_range').value),
            max(float(self.get_parameter('vehicle_fusion_max_distance').value), min_range))
        lateral_limit = max(float(self.get_parameter('vehicle_fusion_lateral_limit').value), 0.05)
        min_points = max(int(self.get_parameter('vehicle_fusion_lidar_min_points').value), 1)
        front_index_param = int(self.get_parameter('scan_front_index').value)
        scan_reverse = bool(self.get_parameter('scan_reverse').value)
        angle_offset = float(self.get_parameter('scan_angle_offset').value)
        front_index = front_index_param if front_index_param >= 0 else (len(msg.ranges) // 2)
        angle_step = abs(float(msg.angle_increment)) if abs(float(msg.angle_increment)) > 1e-6 else math.radians(1.0)
        scan_dir = -1.0 if scan_reverse else 1.0

        hits: List[Point] = []
        for idx, r in enumerate(msg.ranges):
            if not math.isfinite(r) or not (min_range <= r <= max_range):
                continue
            scan_angle = scan_dir * (idx - front_index) * angle_step + angle_offset
            angle_error = abs(self._normalize_angle(scan_angle - camera_angle))
            if angle_error > max_angle_error:
                continue
            x = r * math.cos(scan_angle)
            y = r * math.sin(scan_angle)
            if x <= min_x or abs(y) > lateral_limit:
                continue
            hits.append((float(x), float(y)))

        if len(hits) < min_points:
            return None

        hits.sort(key=lambda point: point[0])
        nearest = hits[:max(min(len(hits), 3), min_points)]
        return (
            float(sum(point[0] for point in nearest) / len(nearest)),
            float(sum(point[1] for point in nearest) / len(nearest)),
        )

    def _vehicle_point_from_box(
        self,
        image: np.ndarray,
        box: Tuple[int, int, int, int],
    ) -> Optional[Point]:
        image_height, image_width = image.shape[:2]
        x0, y0, x1, y1 = box
        box_height = max(y1 - y0, 1)
        height_ratio = box_height / float(max(image_height, 1))
        if height_ratio <= 0.01:
            return None

        camera_fov = math.radians(max(float(self.get_parameter('vehicle_fusion_camera_fov_deg').value), 1.0))
        center_x = 0.5 * (x0 + x1)
        center_ratio = (center_x - 0.5 * image_width) / max(0.5 * image_width, 1.0)
        angle = -center_ratio * 0.5 * camera_fov
        max_range = max(float(self.get_parameter('vehicle_fusion_max_distance').value), 1.0)
        min_distance = max(float(self.get_parameter('vehicle_camera_min_distance').value), 0.30)
        distance = float(np.clip(0.95 / max(height_ratio, 0.04), min_distance, max_range))
        return distance * math.cos(angle), distance * math.sin(angle)

    def _update_vehicle_lidar_fallback(self, scan: Optional[LaserScan]):
        if not bool(self.get_parameter('vehicle_lidar_fallback_enabled').value):
            return
        if not (
            bool(self.get_parameter('vehicle_overtake_enabled').value)
            or bool(self.get_parameter('vehicle_follow_enabled').value)
        ):
            return
        if (
            bool(self.get_parameter('vehicle_lidar_fallback_skip_on_person').value)
            and (self.cached_yolo_person or self.person_avoidance_active)
        ):
            return
        if self.cached_yolo_vehicle_detections:
            return

        now = time.monotonic()
        self._prune_vehicle_tracks(now)
        seeded_tracks = self._yolo_seeded_vehicle_tracks(now)
        if bool(self.get_parameter('vehicle_lidar_fallback_require_yolo_seed').value) and not seeded_tracks:
            return

        point = self._front_lidar_vehicle_point(scan)
        if point is None:
            return

        track = self._match_yolo_seeded_track_for_lidar(point, seeded_tracks, now)
        if track is None:
            return
        self._update_existing_vehicle_track(track, point, now)

    def _yolo_seeded_vehicle_tracks(self, now: float) -> List[VehicleTrack]:
        timeout = max(float(self.get_parameter('vehicle_track_timeout_sec').value), 0.05)
        yolo_timeout = max(float(self.get_parameter('vehicle_yolo_required_timeout_sec').value), 0.02)
        vehicle_class_ids = self._int_set_parameter('yolo_vehicle_class_ids')
        return [
            track for track in self.vehicle_tracks.values()
            if now - track.last_seen_sec <= timeout
            and now - track.yolo_last_seen_sec <= yolo_timeout
            and self._class_id_allowed(track.class_id, vehicle_class_ids)
        ]

    def _has_yolo_confirmed_vehicle_track(self) -> bool:
        return bool(self._yolo_seeded_vehicle_tracks(time.monotonic()))

    def _match_yolo_seeded_track_for_lidar(
        self,
        point: Point,
        tracks: Sequence[VehicleTrack],
        now: float,
    ) -> Optional[VehicleTrack]:
        max_distance = max(float(self.get_parameter('vehicle_track_match_distance').value), 0.10)
        best_track = None
        best_distance = float('inf')
        for track in tracks:
            age = max(now - track.last_seen_sec, 0.0)
            pred_x = track.x + track.vx * age
            pred_y = track.y + track.vy * age
            distance = math.hypot(point[0] - pred_x, point[1] - pred_y)
            if distance > max_distance:
                continue
            if distance < best_distance:
                best_distance = distance
                best_track = track
        return best_track

    def _update_existing_vehicle_track(
        self,
        track: VehicleTrack,
        point: Point,
        now: float,
    ):
        dt = now - track.last_seen_sec
        if 0.04 <= dt <= 1.50:
            raw_vx = (point[0] - track.x) / dt
            raw_vy = (point[1] - track.y) / dt
            smoothing = float(np.clip(
                self.get_parameter('vehicle_track_velocity_smoothing').value, 0.0, 0.95))
            track.vx = smoothing * track.vx + (1.0 - smoothing) * raw_vx
            track.vy = smoothing * track.vy + (1.0 - smoothing) * raw_vy
        track.x = point[0]
        track.y = point[1]
        track.last_seen_sec = now
        track.seen_count += 1

    def _front_lidar_vehicle_point(self, msg: Optional[LaserScan]) -> Optional[Point]:
        if msg is None or not msg.ranges:
            return None

        min_x = max(float(self.get_parameter('vehicle_lidar_fallback_min_x').value), 0.0)
        min_range = max(float(self.get_parameter('scan_min_range').value), 0.0)
        max_range = min(
            float(self.get_parameter('scan_max_range').value),
            max(float(self.get_parameter('vehicle_lidar_fallback_max_distance').value), min_range),
        )
        lateral_limit = max(float(self.get_parameter('vehicle_lidar_fallback_lateral_limit').value), 0.05)
        front_index_param = int(self.get_parameter('scan_front_index').value)
        scan_reverse = bool(self.get_parameter('scan_reverse').value)
        angle_offset = float(self.get_parameter('scan_angle_offset').value)
        front_index = front_index_param if front_index_param >= 0 else (len(msg.ranges) // 2)
        angle_step = abs(float(msg.angle_increment)) if abs(float(msg.angle_increment)) > 1e-6 else math.radians(1.0)
        scan_dir = -1.0 if scan_reverse else 1.0

        points: List[Point] = []
        for idx, r in enumerate(msg.ranges):
            if not math.isfinite(r) or not (min_range <= r <= max_range):
                continue
            scan_angle = scan_dir * (idx - front_index) * angle_step + angle_offset
            x = r * math.cos(scan_angle)
            y = r * math.sin(scan_angle)
            if x < min_x or abs(y) > lateral_limit:
                continue
            points.append((float(x), float(y)))

        if not points:
            return None

        points.sort(key=lambda point: math.atan2(point[1], point[0]))
        gap = max(float(self.get_parameter('vehicle_lidar_fallback_cluster_gap').value), 0.05)
        clusters: List[List[Point]] = []
        current = [points[0]]
        for point in points[1:]:
            if math.hypot(point[0] - current[-1][0], point[1] - current[-1][1]) <= gap:
                current.append(point)
            else:
                clusters.append(current)
                current = [point]
        clusters.append(current)

        min_points = max(int(self.get_parameter('vehicle_lidar_fallback_min_points').value), 1)
        min_size = max(float(self.get_parameter('vehicle_lidar_fallback_min_size').value), 0.0)
        candidates: List[Tuple[float, Point]] = []
        for cluster in clusters:
            if len(cluster) < min_points:
                continue
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            cluster_size = max(max(xs) - min(xs), max(ys) - min(ys))
            if cluster_size < min_size and len(cluster) <= min_points:
                continue
            nearest = sorted(cluster, key=lambda point: point[0])[:min(len(cluster), 5)]
            point = (
                float(sum(p[0] for p in nearest) / len(nearest)),
                float(sum(p[1] for p in nearest) / len(nearest)),
            )
            candidates.append((min(xs), point))

        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    def _vehicle_behavior(
        self,
        center_path: Sequence[Point],
        nearest_obstacle: Optional[float],
    ) -> Optional[VehicleBehavior]:
        follow_enabled = bool(self.get_parameter('vehicle_follow_enabled').value)
        if not follow_enabled:
            self.vehicle_follow_track_id = None
            self.vehicle_follow_lateral_offset = 0.0
            return None

        now = time.monotonic()
        self._prune_vehicle_tracks(now)
        tracks = self._vehicle_track_candidates(center_path, now)
        self.vehicle_overtake_track_id = None
        self.vehicle_overtake_started_sec = None
        self.vehicle_overtake_last_target = None

        if not tracks:
            self.vehicle_slow_confirm_start_sec = None
            self.vehicle_fast_follow_track_id = None
            self.vehicle_fast_follow_started_sec = None
            self.prev_vehicle_follow_speed = None
            self.vehicle_follow_track_id = None
            self.vehicle_follow_lateral_offset = 0.0
            return None

        target = self._select_slow_follow_vehicle_track(tracks)
        if target is None:
            self.vehicle_follow_track_id = None
            self.vehicle_follow_lateral_offset = 0.0
            return None

        if self.vehicle_fast_follow_track_id != target.track_id:
            self.vehicle_fast_follow_started_sec = now
        self.vehicle_fast_follow_track_id = target.track_id
        self.vehicle_slow_confirm_start_sec = None
        follow_path = self._build_vehicle_follow_path(center_path, target)
        return VehicleBehavior(
            mode=f'vehicle_follow_slow_{self._vehicle_class_name(target.class_id)}',
            speed=self._target_vehicle_follow_speed(target),
            path=follow_path if len(follow_path) >= 2 else None,
            max_steer_deg=float(self.get_parameter('vehicle_follow_max_steer_deg').value),
            steer_smoothing=float(self.get_parameter('steer_smoothing').value),
        )

    def _select_slow_follow_vehicle_track(
        self,
        tracks: Sequence[VehicleTrack],
    ) -> Optional[VehicleTrack]:
        if not tracks:
            return None

        slow_class_ids = self._int_set_parameter('vehicle_slow_class_ids')
        preferred = [
            track for track in tracks
            if self._class_id_allowed(track.class_id, slow_class_ids)
        ]
        candidates = preferred if preferred else list(tracks)
        best = min(candidates, key=lambda track: (track.x, abs(track.y), -track.seen_count))

        current = next(
            (track for track in candidates if track.track_id == self.vehicle_follow_track_id),
            None,
        )
        if current is None:
            return best

        switch_margin = max(
            float(self.get_parameter('vehicle_follow_target_switch_margin').value),
            0.0,
        )
        if current.x <= best.x + switch_margin:
            return current
        return best

    def _lead_vehicle_track(self, center_path: Sequence[Point], now: float) -> Optional[VehicleTrack]:
        candidates = self._vehicle_track_candidates(center_path, now)
        if not candidates:
            return None
        return min(candidates, key=lambda track: track.x)

    def _vehicle_track_candidates(self, center_path: Sequence[Point], now: float) -> List[VehicleTrack]:
        timeout = max(float(self.get_parameter('vehicle_track_timeout_sec').value), 0.05)
        yolo_timeout = max(float(self.get_parameter('vehicle_yolo_required_timeout_sec').value), 0.02)
        lateral_limit = max(float(self.get_parameter('vehicle_current_lane_lateral_limit').value), 0.05)
        max_distance = max(float(self.get_parameter('vehicle_fusion_max_distance').value), 0.50)
        min_x = max(float(self.get_parameter('vehicle_lead_min_x').value), 0.0)
        vehicle_class_ids = self._int_set_parameter('yolo_vehicle_class_ids')

        candidates = []
        for track in self.vehicle_tracks.values():
            if now - track.last_seen_sec > timeout:
                continue
            if now - track.yolo_last_seen_sec > yolo_timeout:
                continue
            if not self._class_id_allowed(track.class_id, vehicle_class_ids):
                continue
            if track.x <= min_x or track.x > max_distance:
                continue
            lane_y = self._path_y_at_x(center_path, track.x) if center_path else 0.0
            if abs(track.y - lane_y) > lateral_limit:
                continue
            candidates.append(track)

        return candidates

    def _select_fast_and_slow_vehicle_tracks(
        self,
        tracks: Sequence[VehicleTrack],
    ) -> Tuple[Optional[VehicleTrack], Optional[VehicleTrack]]:
        if not tracks:
            return None, None

        fast = self._select_fast_follow_vehicle_track(tracks)
        remaining = [track for track in tracks if fast is None or track.track_id != fast.track_id]
        slow = min(remaining, key=lambda track: track.x) if remaining else None
        return fast, slow

    def _select_fast_follow_vehicle_track(
        self,
        tracks: Sequence[VehicleTrack],
    ) -> Optional[VehicleTrack]:
        if not tracks:
            return None

        fast_class_ids = self._int_set_parameter('vehicle_fast_class_ids')
        preferred = [
            track for track in tracks
            if self._class_id_allowed(track.class_id, fast_class_ids)
        ]
        if preferred:
            return min(preferred, key=lambda track: (track.x, abs(track.y)))
        return max(tracks, key=self._vehicle_speed_score)

    def _update_fast_follow_state(
        self,
        fast_target: Optional[VehicleTrack],
        now: float,
    ) -> bool:
        if fast_target is None:
            self.vehicle_fast_follow_track_id = None
            self.vehicle_fast_follow_started_sec = None
            return True

        if self.vehicle_fast_follow_track_id != fast_target.track_id:
            self.vehicle_fast_follow_track_id = fast_target.track_id
            self.vehicle_fast_follow_started_sec = now

        min_sec = max(float(self.get_parameter('vehicle_fast_follow_min_sec').value), 0.0)
        started = self.vehicle_fast_follow_started_sec or now
        return now - started >= min_sec

    def _vehicle_speed_score(self, track: VehicleTrack) -> Tuple[float, float, int]:
        return (float(track.vx), float(track.x), int(track.seen_count))

    def _vehicle_is_slow_overtake_candidate(
        self,
        track: VehicleTrack,
        fast_track: Optional[VehicleTrack] = None,
    ) -> bool:
        trigger_distance = max(float(self.get_parameter('vehicle_overtake_trigger_distance').value), 0.50)
        slow_threshold = float(self.get_parameter('vehicle_slow_relative_speed_threshold').value)
        speed_gap = max(float(self.get_parameter('vehicle_fast_slow_speed_gap').value), 0.0)
        min_seen = max(int(self.get_parameter('vehicle_overtake_min_seen').value), 1)
        fast_enough_gap = False
        if fast_track is not None and fast_track.track_id != track.track_id:
            fast_enough_gap = (fast_track.vx - track.vx) >= speed_gap
        close_single_vehicle = (
            fast_track is None
            or fast_track.track_id == track.track_id
        ) and track.x <= 0.80 * trigger_distance
        return (
            track.seen_count >= min_seen
            and track.x <= trigger_distance
            and (track.vx <= slow_threshold or fast_enough_gap or close_single_vehicle)
        )

    def _target_vehicle_follow_speed(self, track: VehicleTrack) -> float:
        base_speed = max(float(self.get_parameter('base_speed').value), 0.0)
        min_speed = max(float(self.get_parameter('min_speed').value), 0.0)
        max_speed = max(float(self.get_parameter('vehicle_follow_max_speed').value), min_speed)
        min_distance = max(float(self.get_parameter('vehicle_follow_min_distance').value), 0.10)
        stop_distance = max(float(self.get_parameter('vehicle_follow_stop_distance').value), min_distance)
        close_distance = max(float(self.get_parameter('vehicle_follow_close_distance').value), stop_distance + 0.10)
        follow_distance = max(float(self.get_parameter('vehicle_follow_distance').value), close_distance + 0.10)
        close_speed = float(np.clip(
            self.get_parameter('vehicle_follow_close_speed').value, 0.0, max_speed))
        gap_gain = float(self.get_parameter('vehicle_follow_gap_gain').value)
        relative_gain = float(self.get_parameter('vehicle_follow_relative_gain').value)
        closing_gain = max(float(self.get_parameter('vehicle_follow_closing_gain').value), 0.0)

        if track.x <= stop_distance:
            speed = 0.0
        elif track.x <= close_distance:
            ratio = (track.x - stop_distance) / max(close_distance - stop_distance, 1e-6)
            speed = close_speed * float(np.clip(ratio, 0.0, 1.0))
        elif track.x <= follow_distance:
            ratio = (track.x - close_distance) / max(follow_distance - close_distance, 1e-6)
            speed = close_speed + (base_speed - close_speed) * float(np.clip(ratio, 0.0, 1.0))
        else:
            speed = base_speed + gap_gain * (track.x - follow_distance) + relative_gain * max(track.vx, 0.0)

        if track.vx < 0.0:
            speed += closing_gain * track.vx

        if track.x < follow_distance:
            speed = min(speed, max(base_speed, close_speed))

        speed = float(np.clip(speed, 0.0, max_speed))
        smoothing = float(np.clip(self.get_parameter('vehicle_follow_speed_smoothing').value, 0.0, 0.80))
        if self.prev_vehicle_follow_speed is not None and speed > self.prev_vehicle_follow_speed:
            speed = smoothing * self.prev_vehicle_follow_speed + (1.0 - smoothing) * speed
        self.prev_vehicle_follow_speed = speed
        return speed

    def _build_vehicle_follow_path(
        self,
        center_path: Sequence[Point],
        track: VehicleTrack,
    ) -> List[Point]:
        if len(center_path) < 2:
            return []

        offset = self._smoothed_vehicle_follow_offset(center_path, track)
        path = self._offset_path(center_path, offset)
        return self._clamp_path_to_white_lane(path, center_path)

    def _smoothed_vehicle_follow_offset(
        self,
        center_path: Sequence[Point],
        track: VehicleTrack,
    ) -> float:
        desired = self._vehicle_lane_offset_for_track(center_path, track)
        lateral_gain = float(np.clip(
            self.get_parameter('vehicle_follow_lateral_gain').value,
            0.0,
            1.0,
        ))
        desired = self._clamp_vehicle_lane_offset(desired * lateral_gain)

        deadband = max(float(self.get_parameter('vehicle_follow_lateral_deadband').value), 0.0)
        if abs(desired) < deadband:
            desired = 0.0

        if self.vehicle_follow_track_id != track.track_id:
            self.vehicle_follow_track_id = track.track_id
            self.vehicle_follow_lateral_offset = self._clamp_vehicle_lane_offset(
                0.5 * self.vehicle_follow_lateral_offset
            )

        smoothing = float(np.clip(
            self.get_parameter('vehicle_follow_lateral_smoothing').value,
            0.0,
            0.95,
        ))
        current = self._clamp_vehicle_lane_offset(self.vehicle_follow_lateral_offset)
        blended = smoothing * current + (1.0 - smoothing) * desired

        max_step = max(float(self.get_parameter('vehicle_follow_lateral_max_step').value), 0.0)
        if max_step > 0.0:
            blended = current + float(np.clip(blended - current, -max_step, max_step))

        self.vehicle_follow_lateral_offset = self._clamp_vehicle_lane_offset(blended)
        return self.vehicle_follow_lateral_offset

    def _vehicle_return_lane_offset(
        self,
        center_path: Sequence[Point],
        track: VehicleTrack,
    ) -> float:
        if not bool(self.get_parameter('vehicle_overtake_return_to_slow_lane_enabled').value):
            return 0.0
        return self._vehicle_lane_offset_for_track(center_path, track)

    def _vehicle_lane_offset_for_track(
        self,
        center_path: Sequence[Point],
        track: VehicleTrack,
    ) -> float:
        lane_y = self._path_y_at_x(center_path, track.x) if center_path else 0.0
        offset = track.y - lane_y
        return self._clamp_vehicle_lane_offset(offset)

    def _vehicle_overtake_edge_offset(self, sign: float) -> float:
        requested = max(float(self.get_parameter('vehicle_overtake_offset').value), 0.0)
        return math.copysign(min(requested, self._vehicle_lane_center_offset_limit()), sign)

    def _clamp_vehicle_lane_offset(self, offset: float) -> float:
        limit = self._vehicle_lane_center_offset_limit()
        return float(np.clip(offset, -limit, limit))

    def _vehicle_lane_center_offset_limit(self) -> float:
        lane_half_width = 0.5 * max(float(self.get_parameter('lane_width').value), 0.20)
        if not bool(self.get_parameter('vehicle_lane_edge_follow_enabled').value):
            return lane_half_width
        wheel_offset = max(float(self.get_parameter('vehicle_outer_wheel_lateral_offset').value), 0.0)
        margin = max(float(self.get_parameter('vehicle_lane_edge_margin').value), 0.0)
        return max(lane_half_width - wheel_offset - margin, 0.05)

    def _lane_guard_reference_path(
        self,
        current_white_lane_path: Optional[Sequence[Point]],
    ) -> Optional[List[Point]]:
        if current_white_lane_path is not None and len(current_white_lane_path) >= 2:
            return list(current_white_lane_path)

        if self.last_white_lane_seen_sec is None or len(self.last_white_lane_path) < 2:
            return None

        memory_sec = max(float(self.get_parameter('lane_guard_memory_sec').value), 0.0)
        if time.monotonic() - self.last_white_lane_seen_sec <= memory_sec:
            return list(self.last_white_lane_path)
        return None

    def _clamp_path_to_white_lane(
        self,
        path: Sequence[Point],
        center_path: Sequence[Point],
    ) -> List[Point]:
        if not bool(self.get_parameter('vehicle_lane_edge_follow_enabled').value):
            return list(path)
        limit = self._vehicle_lane_center_offset_limit()
        return self._clamp_path_to_guard(path, center_path, limit)

    def _clamp_path_to_guard(
        self,
        path: Sequence[Point],
        center_path: Sequence[Point],
        limit: float,
    ) -> List[Point]:
        if len(path) < 2 or len(center_path) < 2:
            return list(path)
        limit = max(float(limit), 0.05)
        clamped: List[Point] = []
        for x, y in path:
            center_y = self._path_y_at_x(center_path, x) if center_path else 0.0
            clamped.append((x, float(np.clip(y, center_y - limit, center_y + limit))))
        return clamped

    def _target_vehicle_overtake_speed(
        self,
        track: VehicleTrack,
        nearest_obstacle: Optional[float],
    ) -> float:
        base_speed = max(float(self.get_parameter('vehicle_overtake_speed').value), 0.0)
        max_speed = max(float(self.get_parameter('vehicle_overtake_max_speed').value), base_speed)
        relative_gain = max(float(self.get_parameter('vehicle_overtake_relative_speed_gain').value), 0.0)
        safety_distance = max(float(self.get_parameter('vehicle_overtake_safety_distance').value), 0.0)
        if safety_distance > 0.0 and track.x <= safety_distance:
            return min(base_speed, self._target_vehicle_follow_speed(track))

        speed = base_speed + relative_gain * max(track.vx, 0.0)
        return float(np.clip(speed, base_speed, max_speed))

    def _predicted_vehicle_x(self, track: VehicleTrack) -> float:
        prediction_sec = max(float(self.get_parameter('vehicle_overtake_prediction_sec').value), 0.0)
        predicted_x = track.x + track.vx * prediction_sec
        if track.vx < 0.0:
            predicted_x = min(track.x, predicted_x)
        else:
            predicted_x = max(track.x, predicted_x)
        return float(predicted_x)

    def _boost_vehicle_overtake_steer(self, steer: float, path: Sequence[Point]) -> float:
        gain = max(float(self.get_parameter('vehicle_overtake_steer_gain').value), 1.0)
        max_steer = max(float(self.get_parameter('vehicle_overtake_max_steer_deg').value), 1.0)
        min_steer = max(float(self.get_parameter('vehicle_overtake_min_steer_deg').value), 0.0)

        boosted = steer * gain
        if abs(boosted) < min_steer:
            sign = 1.0 if boosted > 0.0 else -1.0 if boosted < 0.0 else self._vehicle_overtake_steer_sign(path)
            boosted = sign * min_steer

        return float(np.clip(boosted, -max_steer, max_steer))

    def _boost_vehicle_follow_steer(self, steer: float, path: Sequence[Point]) -> float:
        gain = max(float(self.get_parameter('vehicle_follow_steer_gain').value), 1.0)
        max_steer = max(float(self.get_parameter('vehicle_follow_max_steer_deg').value), 1.0)
        min_steer = max(float(self.get_parameter('vehicle_follow_min_steer_deg').value), 0.0)

        boosted = steer * gain
        if abs(boosted) < min_steer:
            preview_x = min(max(self._lookahead_distance(), 0.50), path[-1][0]) if path else 0.0
            preview_y = self._path_y_at_x(path, preview_x) if path else 0.0
            if abs(preview_y) < max(
                float(self.get_parameter('vehicle_follow_lateral_deadband').value),
                0.03,
            ):
                return float(np.clip(boosted, -max_steer, max_steer))
            sign = 1.0 if boosted > 0.0 else -1.0 if boosted < 0.0 else self._vehicle_overtake_steer_sign(path)
            boosted = sign * min_steer

        return float(np.clip(boosted, -max_steer, max_steer))

    def _lane_edge_steer_guard(
        self,
        steer: float,
        path: Sequence[Point],
        lane_center_path: Optional[Sequence[Point]],
    ) -> float:
        if (
            lane_center_path is None
            or len(path) < 2
            or len(lane_center_path) < 2
            or not bool(self.get_parameter('lane_edge_steer_guard_enabled').value)
        ):
            return steer

        limit = self._vehicle_lane_center_offset_limit()
        if limit <= 0.05:
            return steer

        preview_x = min(max(self._lookahead_distance() * 1.25, 0.60), path[-1][0])
        path_y = self._path_y_at_x(path, preview_x)
        lane_y = self._path_y_at_x(lane_center_path, preview_x)
        offset = path_y - lane_y

        start_ratio = float(np.clip(
            self.get_parameter('lane_edge_steer_guard_start_ratio').value, 0.20, 0.95))
        guard_start = limit * start_ratio
        abs_offset = abs(offset)
        if abs_offset <= guard_start:
            return steer

        outward_sign = 1.0 if offset > 0.0 else -1.0
        if bool(self.get_parameter('invert_steering').value):
            outward_sign = -outward_sign

        if steer * outward_sign <= 0.0:
            return steer

        ratio = float(np.clip((abs_offset - guard_start) / max(limit - guard_start, 1e-6), 0.0, 1.0))
        max_outward = max(float(self.get_parameter('lane_edge_steer_guard_max_outward_deg').value), 0.0)
        min_outward = max(float(self.get_parameter('lane_edge_steer_guard_min_outward_deg').value), 0.0)
        allowed = max(min_outward, max_outward * (1.0 - ratio))
        return float(outward_sign * min(abs(steer), allowed))

    def _vehicle_overtake_steer_sign(self, path: Sequence[Point]) -> float:
        lookahead = self._lookahead_distance()
        target_y = self._path_y_at_x(path, min(lookahead, 2.0))
        sign = 1.0 if target_y >= 0.0 else -1.0
        if bool(self.get_parameter('invert_steering').value):
            sign = -sign
        return sign

    def _build_vehicle_overtake_path(
        self,
        center_path: Sequence[Point],
        track: VehicleTrack,
        final_offset: Optional[float] = None,
    ) -> List[Point]:
        if len(center_path) < 2:
            return []

        spacing = max(float(self.get_parameter('path_spacing').value), 0.10)
        plan_length = max(float(self.get_parameter('vehicle_overtake_plan_length').value), 8.0)
        planner_center_path = self._resample_polyline(
            center_path,
            spacing=min(spacing, 0.30),
            target_length=plan_length,
        )
        arc_path = self._build_arc_path(planner_center_path, spacing=min(spacing, 0.30))
        if len(arc_path) < 8:
            return []

        total_s = arc_path[-1][3]
        start_offset = 0.0
        final_offset = self._clamp_vehicle_lane_offset(0.0 if final_offset is None else final_offset)
        target_offset = self._vehicle_overtake_edge_offset(self.vehicle_overtake_offset_sign)
        shift_start = float(np.clip(
            self.get_parameter('vehicle_overtake_shift_start').value,
            0.0,
            max(total_s - 0.20, 0.0),
        ))
        shift_len = max(float(self.get_parameter('vehicle_overtake_shift_length').value), 0.80)
        hold_len = max(float(self.get_parameter('vehicle_overtake_hold_length').value), 0.0)
        return_len = max(float(self.get_parameter('vehicle_overtake_return_length').value), 0.80)
        pass_margin = max(float(self.get_parameter('vehicle_overtake_pass_margin').value), 0.0)
        relative_speed = abs(track.vx)
        hold_len += relative_speed * max(float(self.get_parameter('vehicle_overtake_speed_hold_gain').value), 0.0)
        pass_margin += relative_speed * max(float(self.get_parameter('vehicle_overtake_speed_pass_gain').value), 0.0)

        predicted_x = self._predicted_vehicle_x(track)
        obs_s = float(np.clip(predicted_x, shift_start + 1.0, max(total_s - return_len - 1.0, shift_start + 1.0)))
        desired_shift_end = obs_s - max(pass_margin, 1.0)
        shift_end = float(np.clip(desired_shift_end, shift_start + 1.0, shift_start + shift_len))
        hold_end = min(max(shift_end + 0.50, obs_s + hold_len + pass_margin), max(total_s - return_len, shift_end))
        return_end = min(total_s, hold_end + return_len)

        candidate: List[Point] = []
        for x, y, yaw, s in arc_path:
            offset = self._vehicle_lane_change_offset_profile(
                s, start_offset, target_offset, final_offset,
                shift_start, shift_end, hold_end, return_end)
            nx = -math.sin(yaw)
            ny = math.cos(yaw)
            candidate.append((x + offset * nx, y + offset * ny))

        candidate = self._smooth_path(candidate, iterations=4)
        candidate = self._sanitize_forward_path(candidate)
        return self._clamp_path_to_white_lane(candidate, center_path)

    @staticmethod
    def _vehicle_lane_change_offset_profile(
        s: float,
        start_offset: float,
        target_offset: float,
        final_offset: float,
        shift_start: float,
        shift_end: float,
        hold_end: float,
        return_end: float,
    ) -> float:
        if s <= shift_start:
            return start_offset
        if s < shift_end:
            t = (s - shift_start) / max(shift_end - shift_start, 1e-6)
            return start_offset + (target_offset - start_offset) * TrackDriverNode._quintic_blend(t)
        if s <= hold_end:
            return target_offset
        if s < return_end:
            t = (s - hold_end) / max(return_end - hold_end, 1e-6)
            return target_offset + (final_offset - target_offset) * TrackDriverNode._quintic_blend(t)
        return final_offset

    def _vehicle_overtake_offset_sign(
        self,
        center_path: Optional[Sequence[Point]] = None,
        slow_track: Optional[VehicleTrack] = None,
        fast_track: Optional[VehicleTrack] = None,
    ) -> float:
        if (
            fast_track is not None
            and center_path
            and (slow_track is None or fast_track.track_id != slow_track.track_id)
        ):
            fast_offset = self._vehicle_lane_offset_for_track(center_path, fast_track)
            if abs(fast_offset) > 0.05:
                return 1.0 if fast_offset > 0.0 else -1.0

        if slow_track is not None and center_path:
            slow_offset = self._vehicle_lane_offset_for_track(center_path, slow_track)
            if abs(slow_offset) > 0.05:
                return -1.0 if slow_offset > 0.0 else 1.0

        return -1.0 if bool(self.get_parameter('vehicle_overtake_prefer_right').value) else 1.0

    def _reset_vehicle_overtake_state(self):
        self.vehicle_overtake_track_id = None
        self.vehicle_overtake_started_sec = None
        self.vehicle_slow_confirm_start_sec = None
        self.vehicle_fast_follow_track_id = None
        self.vehicle_fast_follow_started_sec = None
        self.prev_vehicle_follow_speed = None
        self.vehicle_follow_track_id = None
        self.vehicle_follow_lateral_offset = 0.0
        self.vehicle_overtake_last_target = None
        self.vehicle_overtake_return_offset = 0.0
        self.vehicle_overtake_offset_sign = self._vehicle_overtake_offset_sign()

    @staticmethod
    def _vehicle_class_name(class_id: int) -> str:
        if int(class_id) == 0:
            return 'bcar'
        if int(class_id) == 2:
            return 'gcar'
        if int(class_id) == -1:
            return 'lidar'
        return f'class{int(class_id)}'

    def _valid_light_detection(self, image: np.ndarray, box: Tuple[int, int, int, int]) -> bool:
        image_height, image_width = image.shape[:2]
        x0, y0, x1, y1 = box
        box_width = max(x1 - x0, 0)
        box_height = max(y1 - y0, 0)
        image_area = float(max(image_width * image_height, 1))
        box_area_ratio = (box_width * box_height) / image_area
        height_ratio = box_height / float(max(image_height, 1))

        min_height_ratio = float(np.clip(
            self.get_parameter('yolo_light_min_box_height_ratio').value, 0.0, 1.0))
        max_height_ratio = float(np.clip(
            self.get_parameter('yolo_light_max_box_height_ratio').value, min_height_ratio, 1.0))
        min_area_ratio = float(np.clip(
            self.get_parameter('yolo_light_min_box_area_ratio').value, 0.0, 1.0))
        max_area_ratio = float(np.clip(
            self.get_parameter('yolo_light_max_box_area_ratio').value, min_area_ratio, 1.0))
        max_bottom_ratio = float(np.clip(
            self.get_parameter('yolo_light_max_box_bottom_ratio').value, 0.05, 1.0))
        return (
            min_height_ratio <= height_ratio <= max_height_ratio
            and min_area_ratio <= box_area_ratio <= max_area_ratio
            and y1 <= max_bottom_ratio * max(image_height, 1)
        )

    def _red_light_color_present(self, image: np.ndarray) -> bool:
        return self._red_light_color_metrics(image)[0]

    def _red_light_color_metrics(self, image: np.ndarray) -> Tuple[bool, float, float, float]:
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 90, 80], dtype=np.uint8)
        upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
        lower_red2 = np.array([170, 90, 80], dtype=np.uint8)
        upper_red2 = np.array([180, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        green_mask = cv2.inRange(
            hsv,
            np.array([38, 70, 70], dtype=np.uint8),
            np.array([95, 255, 255], dtype=np.uint8),
        )
        yellow_mask = cv2.inRange(
            hsv,
            np.array([18, 80, 80], dtype=np.uint8),
            np.array([36, 255, 255], dtype=np.uint8),
        )

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        image_area = float(max(width * height, 1))
        red_ratio = float(np.count_nonzero(mask)) / image_area
        green_ratio = float(np.count_nonzero(green_mask)) / image_area
        yellow_ratio = float(np.count_nonzero(yellow_mask)) / image_area
        if red_ratio < float(self.get_parameter('red_light_min_ratio').value):
            return False, red_ratio, green_ratio, yellow_ratio

        dominance = max(float(self.get_parameter('red_light_min_dominance').value), 1.0)
        if red_ratio < dominance * max(green_ratio, yellow_ratio):
            return False, red_ratio, green_ratio, yellow_ratio

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = float(self.get_parameter('red_light_min_area').value)
        min_circularity = float(self.get_parameter('red_light_min_circularity').value)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter < 1e-3:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity >= min_circularity:
                return True, red_ratio, green_ratio, yellow_ratio
        return False, red_ratio, green_ratio, yellow_ratio

    def _publish_light_debug_image(self, image: np.ndarray, light_debug):
        if not bool(self.get_parameter('publish_light_debug_image').value):
            return

        debug = image.copy()
        status = 'STOP CONFIRMED' if self.cached_yolo_red_light else 'NO STOP'
        if self.cached_yolo_go_light and not self.cached_yolo_red_light:
            status = 'GO LIGHT'
        if self.red_light_confirm_count > 0 and not self.cached_yolo_red_light:
            status = f'RAW STOP {self.red_light_confirm_count}'
        status_color = (0, 0, 255) if self.cached_yolo_red_light else (0, 180, 255)
        if self.cached_yolo_go_light and not self.cached_yolo_red_light:
            status_color = (0, 220, 0)
        elif self.red_light_confirm_count == 0 and not self.cached_yolo_red_light:
            status_color = (0, 220, 0)

        cv2.rectangle(debug, (6, 6), (360, 56), (0, 0, 0), thickness=-1)
        cv2.putText(
            debug,
            f'light: {status}',
            (14, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            status_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            f'red_count={self.red_light_confirm_count} go={int(self.cached_yolo_go_light)}',
            (14, 49),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )

        if not light_debug:
            cv2.putText(
                debug,
                'no light boxes',
                (14, 78),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (180, 180, 180),
                1,
                cv2.LINE_AA,
            )

        for box, score, class_id, valid, red_present, red_ratio, green_ratio, yellow_ratio in light_debug:
            x0, y0, x1, y1 = box
            class_name = self._light_class_name(class_id)
            if int(class_id) == 0:
                color = (0, 165, 255)
                label = f'CONE:{class_name}'
            elif not valid:
                color = (130, 130, 130)
                label = f'reject:{class_name}'
            elif red_present:
                color = (0, 0, 255)
                label = f'STOP:{class_name}'
            elif self._class_id_allowed(class_id, self._int_set_parameter('yolo_go_light_class_ids')):
                color = (0, 220, 0)
                label = f'GO:{class_name}'
            elif int(class_id) == 2:
                color = (255, 160, 0)
                label = f'LEFT:{class_name}'
            elif yellow_ratio >= red_ratio and yellow_ratio >= green_ratio:
                color = (0, 220, 255)
                label = f'yellow?:{class_name}'
            else:
                color = (255, 180, 0)
                label = f'light:{class_name}'

            cv2.rectangle(debug, (x0, y0), (x1, y1), color, thickness=2)
            text = f'{label} s={score:.2f} r={red_ratio:.3f} g={green_ratio:.3f} y={yellow_ratio:.3f}'
            text_y = max(y0 - 8, 18)
            cv2.putText(
                debug,
                text,
                (x0, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )

        try:
            msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'front_camera'
            self.light_debug_pub.publish(msg)
        except Exception as exc:
            self._warn_yolo_once('light_debug', f'light debug image publish failed: {exc}')

    @staticmethod
    def _light_class_name(class_id: int) -> str:
        names = {
            0: 'cone',
            1: 'green',
            2: 'left',
            3: 'red',
            4: 'yellow',
        }
        return names.get(int(class_id), f'class{int(class_id)}')

    def _warn_yolo_once(self, name: str, reason: str):
        now_sec = self.get_clock().now().nanoseconds // 1_000_000_000
        if self.yolo_last_warn_sec.get(name) == now_sec:
            return
        self.yolo_last_warn_sec[name] = now_sec
        self.get_logger().warn(reason)

    def _detect_person_from_camera_lidar(self, msg: Optional[LaserScan]) -> bool:
        if msg is None or not msg.ranges or self.image is None:
            self.person_fusion_distance = None
            self.person_fusion_point = None
            return False

        detections = self.cached_yolo_person_detections
        if not detections and self.cached_yolo_person_box is not None:
            detections = [(self.cached_yolo_person_box, 1.0, 0)]

        for box, _, _ in detections:
            fused_point = self._lidar_point_for_camera_box(msg, box)
            if fused_point is None:
                continue

            self.cached_yolo_person_box = box
            self.person_fusion_point = fused_point
            self.person_fusion_distance = fused_point[0]
            self._update_person_motion(fused_point)
            return True

        self.person_fusion_distance = None
        self.person_fusion_point = None
        return False

    def _lidar_point_for_camera_box(
        self,
        msg: LaserScan,
        box: Tuple[int, int, int, int],
    ) -> Optional[Point]:
        image_height, image_width = self.image.shape[:2]
        x0, _, x1, _ = box
        center_x = 0.5 * (x0 + x1)
        box_width = max(x1 - x0, 1)

        camera_fov = math.radians(max(float(self.get_parameter('person_fusion_camera_fov_deg').value), 1.0))
        box_center_ratio = (center_x - 0.5 * image_width) / max(0.5 * image_width, 1.0)
        camera_angle = -box_center_ratio * 0.5 * camera_fov
        half_box_angle = 0.5 * camera_fov * (box_width / max(image_width, 1))
        angle_margin = math.radians(max(float(self.get_parameter('person_fusion_angle_margin_deg').value), 0.0))
        max_angle_error = max(half_box_angle + angle_margin, math.radians(3.0))

        min_range = float(self.get_parameter('scan_min_range').value)
        max_range = min(
            float(self.get_parameter('scan_max_range').value),
            max(float(self.get_parameter('person_fusion_max_distance').value), min_range))
        lateral_limit = max(float(self.get_parameter('person_fusion_lateral_limit').value), 0.05)
        min_points = max(int(self.get_parameter('person_fusion_lidar_min_points').value), 1)
        front_index_param = int(self.get_parameter('scan_front_index').value)
        scan_reverse = bool(self.get_parameter('scan_reverse').value)
        angle_offset = float(self.get_parameter('scan_angle_offset').value)
        front_index = front_index_param if front_index_param >= 0 else (len(msg.ranges) // 2)
        angle_step = abs(float(msg.angle_increment)) if abs(float(msg.angle_increment)) > 1e-6 else math.radians(1.0)
        scan_dir = -1.0 if scan_reverse else 1.0

        hits: List[Point] = []
        for idx, r in enumerate(msg.ranges):
            if not math.isfinite(r) or not (min_range <= r <= max_range):
                continue

            scan_angle = scan_dir * (idx - front_index) * angle_step + angle_offset
            angle_error = abs(self._normalize_angle(scan_angle - camera_angle))
            if angle_error > max_angle_error:
                continue

            x = r * math.cos(scan_angle)
            y = r * math.sin(scan_angle)
            if x <= 0.05 or abs(y) > lateral_limit:
                continue

            hits.append((float(x), float(y)))

        if len(hits) < min_points:
            return None

        return min(hits, key=lambda point: point[0])

    def _update_person_motion(self, point: Point):
        now = time.monotonic()
        vx = 0.0
        vy = 0.0

        if self.prev_person_fusion_point is not None and self.prev_person_fusion_time_sec is not None:
            dt = now - self.prev_person_fusion_time_sec
            if 0.04 <= dt <= 1.50:
                raw_vx = (point[0] - self.prev_person_fusion_point[0]) / dt
                raw_vy = (point[1] - self.prev_person_fusion_point[1]) / dt
                max_speed = max(float(self.get_parameter('person_dynamic_max_speed').value), 0.10)
                speed = math.hypot(raw_vx, raw_vy)
                if speed > max_speed:
                    scale = max_speed / speed
                    raw_vx *= scale
                    raw_vy *= scale

                smoothing = float(np.clip(
                    self.get_parameter('person_dynamic_velocity_smoothing').value, 0.0, 0.95))
                vx = smoothing * self.person_velocity[0] + (1.0 - smoothing) * raw_vx
                vy = smoothing * self.person_velocity[1] + (1.0 - smoothing) * raw_vy

        self.person_velocity = (vx, vy)
        self.prev_person_fusion_point = point
        self.prev_person_fusion_time_sec = now

        prediction_sec = max(float(self.get_parameter('person_dynamic_prediction_sec').value), 0.0)
        if bool(self.get_parameter('person_dynamic_enabled').value):
            self.person_predicted_point = (
                max(point[0] + vx * prediction_sec, 0.05),
                point[1] + vy * prediction_sec,
            )
        else:
            self.person_predicted_point = point

    def _refresh_person_prediction(self, now: float):
        if (
            not bool(self.get_parameter('person_dynamic_enabled').value)
            or self.prev_person_fusion_point is None
            or self.prev_person_fusion_time_sec is None
        ):
            return

        prediction_sec = max(float(self.get_parameter('person_dynamic_prediction_sec').value), 0.0)
        elapsed = float(np.clip(now - self.prev_person_fusion_time_sec, 0.0, prediction_sec))
        self.person_predicted_point = (
            max(self.prev_person_fusion_point[0] + self.person_velocity[0] * elapsed, 0.05),
            self.prev_person_fusion_point[1] + self.person_velocity[1] * elapsed,
        )

    def _update_person_image_motion(self, box: Tuple[int, int, int, int], image_width: int):
        if image_width <= 0:
            return

        x0, _, x1, _ = box
        center_x = 0.5 * (x0 + x1)
        center_ratio = (center_x - 0.5 * image_width) / max(0.5 * image_width, 1.0)
        now = time.monotonic()

        if self.prev_person_image_center_ratio is not None and self.prev_person_image_time_sec is not None:
            dt = now - self.prev_person_image_time_sec
            if 0.04 <= dt <= 1.50:
                raw_velocity = (center_ratio - self.prev_person_image_center_ratio) / dt
                smoothing = float(np.clip(
                    self.get_parameter('person_dynamic_velocity_smoothing').value, 0.0, 0.95))
                self.person_image_velocity_ratio = (
                    smoothing * self.person_image_velocity_ratio
                    + (1.0 - smoothing) * raw_velocity
                )

        self.person_image_center_ratio = float(np.clip(center_ratio, -1.0, 1.0))
        self.prev_person_image_center_ratio = self.person_image_center_ratio
        self.prev_person_image_time_sec = now

    def _detect_person_from_scan(self, msg: Optional[LaserScan]) -> bool:
        if msg is None or not msg.ranges:
            return False

        min_range = float(self.get_parameter('scan_min_range').value)
        max_range = min(float(self.get_parameter('scan_max_range').value),
                        float(self.get_parameter('person_stop_distance').value))
        lateral_limit = float(self.get_parameter('person_lateral_limit').value)
        min_points = max(int(self.get_parameter('person_lidar_min_points').value), 1)
        front_index_param = int(self.get_parameter('scan_front_index').value)
        scan_reverse = bool(self.get_parameter('scan_reverse').value)
        angle_offset = float(self.get_parameter('scan_angle_offset').value)
        front_index = front_index_param if front_index_param >= 0 else (len(msg.ranges) // 2)
        angle_step = abs(float(msg.angle_increment)) if abs(float(msg.angle_increment)) > 1e-6 else math.radians(1.0)
        scan_dir = -1.0 if scan_reverse else 1.0

        hit_count = 0
        for idx, r in enumerate(msg.ranges):
            if not math.isfinite(r) or not (min_range <= r <= max_range):
                continue

            scan_angle = scan_dir * (idx - front_index) * angle_step + angle_offset
            x = r * math.cos(scan_angle)
            y = r * math.sin(scan_angle)
            if 0.05 < x <= max_range and abs(y) <= lateral_limit:
                hit_count += 1
                if hit_count >= min_points:
                    return True

        return False

    def _detect_person_from_image(self, image: Optional[np.ndarray]) -> bool:
        if image is None:
            return False

        if self.person_hog is None:
            try:
                self.person_hog = cv2.HOGDescriptor()
                self.person_hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            except Exception as exc:
                self.get_logger().warn(f'person detector init failed: {exc}')
                return False

        height, width = image.shape[:2]
        scale = min(1.0, 480.0 / max(width, 1))
        detector_image = image
        if scale < 1.0:
            detector_image = cv2.resize(
                image, (int(width * scale), int(height * scale)), interpolation=cv2.INTER_AREA)

        rects, weights = self.person_hog.detectMultiScale(
            detector_image,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )
        min_weight = float(self.get_parameter('person_hog_min_weight').value)
        return any(float(weight) >= min_weight for weight in weights)

    def _build_person_avoidance_path(self, center_path: Sequence[Point]) -> List[Point]:
        if len(center_path) < 2:
            return []

        spacing = max(float(self.get_parameter('path_spacing').value), 0.10)
        plan_length = max(float(self.get_parameter('person_avoidance_plan_length').value), 8.0)
        planner_center_path = self._resample_polyline(
            center_path,
            spacing=min(spacing, 0.30),
            target_length=plan_length,
        )
        if len(planner_center_path) < 8:
            return []

        arc_path = self._build_arc_path(planner_center_path, spacing=min(spacing, 0.30))
        total_s = arc_path[-1][3] if arc_path else plan_length
        obstacle = self._person_local_obstacle(total_s)
        obstacle_half_x = self._person_lattice_obstacle_half_x()
        fixed_offset = self._person_lattice_fixed_offset()

        ok, out_path, should_stop, selected_offset, _ = self._lattice_plan(
            planner_center_path,
            [obstacle],
            use_fixed_offset=True,
            fixed_offset=fixed_offset,
            allow_return=bool(self.get_parameter('person_avoidance_allow_return').value),
            force_obs_x=obstacle.x,
            force_obs_half_x=obstacle_half_x,
        )
        self.person_lattice_should_stop = should_stop
        self.person_lattice_selected_offset = selected_offset
        if not ok or should_stop:
            return []

        return out_path

    def _person_local_obstacle(self, total_s: float) -> LocalObstacle:
        point = self.person_predicted_point or self.person_fusion_point
        if point is None:
            x = self._person_avoidance_obstacle_x(total_s)
            y = self._person_lateral_from_image(x)
        else:
            x, y = point

        speed = math.hypot(*self.person_velocity)
        return LocalObstacle(
            track_id=1.0,
            x=float(np.clip(x, -3.0, 42.0)),
            y=float(np.clip(y, -3.0, 3.0)),
            size_x=0.60,
            size_y=0.60,
            size_z=1.70,
            type_id=1.0,
            speed=speed,
            camera_confirmed=1.0,
            motion_state=2.0 if speed > 0.05 else 1.0,
        )

    def _person_lateral_from_image(self, obstacle_x: float) -> float:
        if self.person_image_center_ratio is None:
            return 0.0

        camera_fov = math.radians(max(float(self.get_parameter('person_fusion_camera_fov_deg').value), 1.0))
        camera_angle = -self.person_image_center_ratio * 0.5 * camera_fov
        return obstacle_x * math.tan(camera_angle)

    def _person_lattice_obstacle_half_x(self) -> float:
        obstacle_half_x = max(float(self.get_parameter('person_avoidance_obstacle_half_x').value), 2.0)
        if bool(self.get_parameter('person_dynamic_enabled').value):
            prediction_sec = max(float(self.get_parameter('person_dynamic_prediction_sec').value), 0.0)
            dynamic_padding = min(math.hypot(*self.person_velocity) * prediction_sec, 1.20)
            obstacle_half_x += dynamic_padding
        return obstacle_half_x

    def _person_lattice_fixed_offset(self) -> float:
        one_lane_shift = max(float(self.get_parameter('person_avoidance_offset').value), 0.0)
        two_lane_shift = max(
            float(self.get_parameter('person_avoidance_two_lane_offset').value),
            one_lane_shift,
        )
        sign = -1.0 if bool(self.get_parameter('person_avoidance_prefer_right').value) else 1.0
        return float(np.clip(sign * one_lane_shift, -two_lane_shift, two_lane_shift))

    def _lattice_plan(
        self,
        center_path: Sequence[Point],
        local_obstacles: Sequence[LocalObstacle],
        use_fixed_offset: bool = False,
        fixed_offset: float = 0.0,
        allow_return: bool = False,
        force_obs_x: float = -100.0,
        force_obs_half_x: float = 2.0,
    ) -> Tuple[bool, List[Point], bool, float, float]:
        should_stop = False
        selected_offset = 0.0
        recommended_speed = 0.0
        if len(center_path) < 8:
            return False, [], True, selected_offset, recommended_speed

        found_blocking = False
        blocking = LocalObstacle()
        best_obs_x = float('inf')
        blocking_half_x = 2.0

        for obs in local_obstacles:
            if obs.x < -3.0 or obs.x > 42.0:
                continue

            pedestrian_like = obs.type_id == 1.0
            vehicle_dynamic = obs.type_id == 2.0 and obs.motion_state == 2.0
            dynamic_like = pedestrian_like or vehicle_dynamic
            gate = 2.8 if dynamic_like else 2.2
            if self._point_to_path_distance(obs.x, obs.y, center_path) > gate:
                continue

            if dynamic_like and not use_fixed_offset and -0.5 < obs.x < 16.0:
                should_stop = True
                return False, [], should_stop, selected_offset, recommended_speed

            if not dynamic_like and obs.x > 1.0 and obs.x < best_obs_x:
                best_obs_x = obs.x
                blocking = obs
                blocking_half_x = max(2.0, obs.size_x * 0.5 + 1.0)
                found_blocking = True

        if use_fixed_offset:
            one_lane_shift = max(float(self.get_parameter('person_avoidance_offset').value), 0.0)
            two_lane_shift = max(
                float(self.get_parameter('person_avoidance_two_lane_offset').value),
                one_lane_shift,
            )
            safe_offset = fixed_offset
            if abs(safe_offset) < 0.5:
                if found_blocking:
                    safe_offset = -one_lane_shift if blocking.y >= 0.0 else one_lane_shift
                else:
                    safe_offset = one_lane_shift
            safe_offset = float(np.clip(safe_offset, -two_lane_shift, two_lane_shift))

            ref_x = force_obs_x if force_obs_x > -50.0 else (blocking.x if found_blocking else 18.0)
            ref_half_x = force_obs_half_x if force_obs_x > -50.0 else (
                blocking_half_x if found_blocking else 2.0)
            candidate = self._build_lattice_lane_change_path(
                center_path, safe_offset, ref_x, ref_half_x, allow_return)
            if len(candidate) >= 8:
                selected_offset = safe_offset
                return True, candidate, should_stop, selected_offset, recommended_speed

        if not found_blocking:
            return True, list(center_path), should_stop, selected_offset, recommended_speed

        stop_close_x = max(float(self.get_parameter('person_avoidance_stop_close_x').value), 0.0)
        if not use_fixed_offset and blocking.x < stop_close_x:
            should_stop = True
            return False, [], should_stop, selected_offset, recommended_speed

        return False, [], should_stop, selected_offset, recommended_speed

    def _build_lattice_lane_change_path(
        self,
        center_path: Sequence[Point],
        target_offset: float,
        obstacle_x: float,
        obstacle_half_x: float,
        allow_return: bool,
    ) -> List[Point]:
        if len(center_path) < 2:
            return []

        arc_path = self._build_arc_path(center_path, spacing=0.30)
        if len(arc_path) < 8:
            return []

        total_s = arc_path[-1][3]
        obs_s = float(np.clip(obstacle_x, 0.0, max(total_s - 8.0, 0.0)))
        shift_start = float(np.clip(
            self.get_parameter('person_avoidance_shift_start').value,
            0.0,
            max(total_s - 0.20, 0.0),
        ))
        shift_len = max(float(self.get_parameter('person_avoidance_shift_length').value), 0.80)
        shift_end = float(np.clip(
            obs_s - max(5.0, obstacle_half_x + 2.5),
            shift_start + 5.0,
            shift_start + shift_len,
        ))

        hold_len = max(float(self.get_parameter('person_avoidance_hold_length').value), 0.0)
        hold_end = min(total_s - 4.0, obs_s + max(hold_len, obstacle_half_x + 6.0))
        return_len = max(float(self.get_parameter('person_avoidance_return_length').value), 0.80)
        return_end = min(total_s, hold_end + return_len)

        candidate: List[Point] = []
        for x, y, yaw, s in arc_path:
            offset = self._avoidance_offset_profile(
                s, target_offset, shift_start, shift_end, hold_end, return_end, allow_return)
            nx = -math.sin(yaw)
            ny = math.cos(yaw)
            candidate.append((x + offset * nx, y + offset * ny))

        candidate = self._smooth_path(candidate, iterations=4)
        return self._sanitize_forward_path(candidate)

    @staticmethod
    def _point_to_path_distance(px: float, py: float, path: Sequence[Point]) -> float:
        if not path:
            return float('inf')
        return min(math.hypot(point[0] - px, point[1] - py) for point in path)

    def _person_avoidance_obstacle_x(self, total_s: float) -> float:
        default_x = float(self.get_parameter('person_avoidance_obstacle_x').value)
        obstacle_x = default_x

        if self.person_predicted_point is not None:
            obstacle_x = min(obstacle_x, max(self.person_predicted_point[0], 0.80))

        if self.person_fusion_distance is not None:
            obstacle_x = min(obstacle_x, max(self.person_fusion_distance, 0.80))

        if self.image is not None and self.cached_yolo_person_box is not None:
            image_height, _ = self.image.shape[:2]
            _, y0, _, y1 = self.cached_yolo_person_box
            height_ratio = max(y1 - y0, 0) / float(max(image_height, 1))
            if height_ratio >= 0.55:
                obstacle_x = min(obstacle_x, 1.80)
            elif height_ratio >= 0.35:
                obstacle_x = min(obstacle_x, 2.60)
            elif height_ratio >= 0.20:
                obstacle_x = min(obstacle_x, 3.20)

        return float(np.clip(obstacle_x, 0.0, max(total_s, 0.0)))

    def _person_avoidance_offset_sign(self) -> float:
        return -1.0 if bool(self.get_parameter('person_avoidance_prefer_right').value) else 1.0

    @staticmethod
    def _build_arc_path(path: Sequence[Point], spacing: float) -> List[Tuple[float, float, float, float]]:
        if len(path) < 2:
            return []

        dense: List[Point] = [path[0]]
        for p0, p1 in zip(path[:-1], path[1:]):
            seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            steps = max(1, int(math.ceil(seg_len / max(spacing, 0.10))))
            for step in range(1, steps + 1):
                t = step / steps
                dense.append((p0[0] + (p1[0] - p0[0]) * t,
                              p0[1] + (p1[1] - p0[1]) * t))

        arc_path: List[Tuple[float, float, float, float]] = []
        distance = 0.0
        for idx, point in enumerate(dense):
            if idx > 0:
                prev = dense[idx - 1]
                distance += math.hypot(point[0] - prev[0], point[1] - prev[1])

            i0 = max(idx - 1, 0)
            i1 = min(idx + 1, len(dense) - 1)
            yaw = math.atan2(dense[i1][1] - dense[i0][1],
                             dense[i1][0] - dense[i0][0] + 1e-6)
            arc_path.append((point[0], point[1], yaw, distance))

        return arc_path

    @staticmethod
    def _avoidance_offset_profile(
        s: float,
        target_offset: float,
        shift_start: float,
        shift_end: float,
        hold_end: float,
        return_end: float,
        allow_return: bool,
    ) -> float:
        if s <= shift_start:
            return 0.0
        if s < shift_end:
            t = (s - shift_start) / max(shift_end - shift_start, 1e-6)
            return target_offset * TrackDriverNode._quintic_blend(t)
        if not allow_return or s <= hold_end:
            return target_offset
        if s < return_end:
            t = (s - hold_end) / max(return_end - hold_end, 1e-6)
            return target_offset * (1.0 - TrackDriverNode._quintic_blend(t))
        return 0.0

    @staticmethod
    def _quintic_blend(t: float) -> float:
        t = float(np.clip(t, 0.0, 1.0))
        return 6.0 * t ** 5 - 15.0 * t ** 4 + 10.0 * t ** 3

    @staticmethod
    def _sanitize_forward_path(path: Sequence[Point]) -> List[Point]:
        out: List[Point] = []
        last_x = -float('inf')
        for point in path:
            if point[0] < 0.0:
                continue
            if out and point[0] < last_x - 0.10:
                continue
            if out and math.hypot(point[0] - out[-1][0], point[1] - out[-1][1]) < 0.10:
                continue
            out.append(point)
            last_x = point[0]
        return out

    def _filter_track_cones(self, cones: Sequence[Point]) -> List[Point]:
        if len(cones) < 2:
            return []

        left, right = self._split_cones_into_boundaries(cones)
        min_points = int(self.get_parameter('cone_boundary_min_points').value)

        if len(left) < min_points:
            left = []
        if len(right) < min_points:
            right = []

        if left and right:
            centerline = self._centerline_from_boundaries(left, right)
            if len(centerline) >= 2:
                return self._unique_points(list(left) + list(right))

        if left:
            return self._unique_points(left)
        if right:
            return self._unique_points(right)
        return []

    @staticmethod
    def _unique_points(points: Sequence[Point]) -> List[Point]:
        unique: List[Point] = []
        seen = set()
        for point in points:
            key = TrackDriverNode._point_key(point)
            if key in seen:
                continue
            seen.add(key)
            unique.append(point)
        return sorted(unique, key=lambda p: p[0])

    def _build_lane_center_path(self, image: Optional[np.ndarray]) -> Optional[List[Point]]:
        if image is None:
            return None

        height, width = image.shape[:2]
        roi_top = int(height * float(self.get_parameter('lane_roi_top_ratio').value))
        roi = image[roi_top:height, :]
        if roi.size == 0:
            return None

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, np.array([0, 0, 175]), np.array([180, 65, 255]))
        mask = white_mask

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        ys, xs = np.nonzero(mask)
        if len(xs) < int(self.get_parameter('white_lane_min_pixels').value):
            return None

        mid_x = width * 0.5
        bottom_cut = int(mask.shape[0] * 0.70)
        lower_roi = ys > mask.shape[0] * 0.12
        left_mask = (xs < mid_x) & lower_roi
        right_mask = (xs >= mid_x) & lower_roi
        left = xs[left_mask]
        left_y = ys[left_mask]
        right = xs[right_mask]
        right_y = ys[right_mask]

        center_pixels: List[Tuple[float, float]] = []
        expected_width_px = float(self.get_parameter('white_lane_expected_width_px').value)
        for yq in np.linspace(bottom_cut, mask.shape[0] - 1, 6):
            lx = self._median_x_near_y(left, left_y, yq)
            rx = self._median_x_near_y(right, right_y, yq)

            if lx is not None and rx is not None:
                cx = 0.5 * (lx + rx)
            elif lx is not None:
                cx = lx + 0.5 * expected_width_px
            elif rx is not None:
                cx = rx - 0.5 * expected_width_px
            else:
                continue

            center_pixels.append((float(yq), float(cx)))

        if len(center_pixels) < 2:
            return None

        center_pixels.sort(key=lambda p: p[0], reverse=True)
        bottom_center = center_pixels[0][1]
        upper_center = center_pixels[-1][1]

        center_error_px = bottom_center - mid_x
        heading_error_px = bottom_center - upper_center
        lateral_scale = float(self.get_parameter('camera_lateral_scale').value)
        forward_scale = float(self.get_parameter('camera_forward_scale').value)
        center_gain = float(self.get_parameter('lane_center_gain').value)
        heading_gain = float(self.get_parameter('lane_heading_gain').value)

        lateral_error = center_error_px * lateral_scale * center_gain
        heading_error = heading_error_px * lateral_scale * heading_gain

        path: List[Point] = []
        for x in self._path_x_samples():
            curvature_bias = 0.09 * heading_error * x * x
            y = lateral_error + heading_error * forward_scale * x + curvature_bias
            path.append((x, float(np.clip(y, -2.0, 2.0))))

        self.last_white_lane_path = list(path)
        self.last_white_lane_seen_sec = time.monotonic()
        return path

    def _build_school_zone_center_path(self, image: Optional[np.ndarray]) -> Optional[List[Point]]:
        if (
            image is None
            or not self.school_zone_active
            or not bool(self.get_parameter('school_zone_follow_yellow_centerline').value)
        ):
            return None

        height, width = image.shape[:2]
        top_ratio = float(np.clip(self.get_parameter('school_zone_roi_top_ratio').value, 0.0, 0.95))
        y0 = int(height * top_ratio)
        roi = image[y0:height, :]
        if roi.size == 0:
            return None

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(
            hsv,
            np.array([18, 70, 80], dtype=np.uint8),
            np.array([38, 255, 255], dtype=np.uint8),
        )
        kernel = np.ones((3, 3), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        self._mask_school_zone_vehicle_boxes(yellow_mask, y0)
        edge_guard_path = self._build_school_zone_edge_center_path_from_mask(yellow_mask, width)

        center_left = float(np.clip(
            self.get_parameter('school_zone_center_left_ratio').value, 0.05, 0.49))
        center_right = float(np.clip(
            self.get_parameter('school_zone_center_right_ratio').value, 0.51, 0.95))
        x0 = max(int(width * center_left), 0)
        x1 = min(int(width * center_right), width)
        if x1 <= x0:
            return None

        center_mask = np.zeros_like(yellow_mask)
        center_mask[:, x0:x1] = yellow_mask[:, x0:x1]

        kernel = np.ones((3, 3), np.uint8)
        center_mask = cv2.morphologyEx(center_mask, cv2.MORPH_OPEN, kernel)
        center_mask = cv2.morphologyEx(center_mask, cv2.MORPH_CLOSE, kernel)
        vertical_close = max(9, int(center_mask.shape[0] * 0.055))
        if vertical_close % 2 == 0:
            vertical_close += 1
        center_mask = cv2.morphologyEx(
            center_mask,
            cv2.MORPH_CLOSE,
            np.ones((vertical_close, 5), np.uint8),
        )

        ys, xs = np.nonzero(center_mask)
        min_pixels = max(int(self.get_parameter('school_zone_center_min_pixels').value), 1)
        if len(xs) < min_pixels:
            recent = self._recent_school_zone_path()
            return recent if recent is not None else edge_guard_path

        center_pixels: List[Tuple[float, float]] = []
        band_count = max(int(self.get_parameter('school_zone_center_band_count').value), 3)
        band_half_height = max(float(self.get_parameter('school_zone_center_band_half_height').value), 8.0)
        min_band_pixels = max(int(self.get_parameter('school_zone_center_min_band_pixels').value), 1)
        top_cut = int(center_mask.shape[0] * 0.18)
        for yq in np.linspace(top_cut, center_mask.shape[0] - 1, band_count):
            band = np.abs(ys - yq) < band_half_height
            if np.count_nonzero(band) < min_band_pixels:
                continue
            center_pixels.append((float(yq), float(np.median(xs[band]))))

        if len(center_pixels) < 2:
            recent = self._recent_school_zone_path()
            return recent if recent is not None else edge_guard_path

        path = self._path_from_camera_center_pixels(center_pixels, width, curvature_gain=0.06)

        if edge_guard_path is not None:
            edge_weight = float(np.clip(
                self.get_parameter('school_zone_edge_center_weight').value, 0.0, 0.80))
            if edge_weight > 0.0:
                path = self._blend_paths(path, edge_guard_path, edge_weight)

        recent = self._recent_school_zone_path()
        if recent is not None:
            memory_weight = float(np.clip(
                self.get_parameter('school_zone_center_memory_weight').value, 0.0, 0.90))
            if memory_weight > 0.0:
                path = self._blend_paths(path, recent, memory_weight)

        path = self._smooth_path(path, iterations=2)

        if (
            bool(self.get_parameter('school_zone_yellow_path_edge_guard_enabled').value)
            and edge_guard_path is not None
        ):
            limit = max(float(self.get_parameter('school_zone_yellow_path_edge_limit').value), 0.05)
            path = self._clamp_path_to_guard(path, edge_guard_path, limit)

        self._remember_school_zone_path(path)
        return path

    def _recent_school_zone_path(self) -> Optional[List[Point]]:
        if self.last_school_zone_path_seen_sec is None or len(self.last_school_zone_path) < 2:
            return None
        memory_sec = max(float(self.get_parameter('school_zone_center_memory_sec').value), 0.0)
        if time.monotonic() - self.last_school_zone_path_seen_sec > memory_sec:
            return None
        return list(self.last_school_zone_path)

    def _remember_school_zone_path(self, path: Sequence[Point]):
        if len(path) < 2:
            return
        self.last_school_zone_path = list(path)
        self.last_school_zone_path_seen_sec = time.monotonic()

    def _blend_paths(
        self,
        primary_path: Sequence[Point],
        secondary_path: Sequence[Point],
        secondary_weight: float,
    ) -> List[Point]:
        if len(primary_path) < 2 or len(secondary_path) < 2:
            return list(primary_path)
        weight = float(np.clip(secondary_weight, 0.0, 1.0))
        return [
            (x, float((1.0 - weight) * y + weight * self._path_y_at_x(secondary_path, x)))
            for x, y in primary_path
        ]

    def _build_school_zone_edge_center_path_from_mask(
        self,
        yellow_mask: np.ndarray,
        image_width: int,
    ) -> Optional[List[Point]]:
        left_edge_max = float(np.clip(
            self.get_parameter('school_zone_left_edge_max_ratio').value, 0.05, 0.49))
        right_edge_min = float(np.clip(
            self.get_parameter('school_zone_right_edge_min_ratio').value, 0.51, 0.95))
        left_end = max(int(image_width * left_edge_max), 1)
        right_start = min(int(image_width * right_edge_min), image_width - 1)
        if right_start <= left_end:
            return None

        left_y, left_x = np.nonzero(yellow_mask[:, :left_end])
        right_y, right_x_local = np.nonzero(yellow_mask[:, right_start:])
        right_x = right_x_local + right_start
        if len(left_x) < 10 or len(right_x) < 10:
            return None

        bottom_cut = int(yellow_mask.shape[0] * 0.62)
        center_pixels: List[Tuple[float, float]] = []
        for yq in np.linspace(bottom_cut, yellow_mask.shape[0] - 1, 7):
            lx = self._median_x_near_y(left_x, left_y, yq)
            rx = self._median_x_near_y(right_x, right_y, yq)
            if lx is None or rx is None:
                continue
            center_pixels.append((float(yq), 0.5 * (lx + rx)))

        if len(center_pixels) < 2:
            return None

        return self._path_from_camera_center_pixels(center_pixels, image_width, curvature_gain=0.08)

    def _path_from_camera_center_pixels(
        self,
        center_pixels: Sequence[Tuple[float, float]],
        image_width: int,
        curvature_gain: float,
    ) -> List[Point]:
        ordered = sorted(center_pixels, key=lambda p: p[0], reverse=True)
        bottom_center = ordered[0][1]
        upper_center = ordered[-1][1]

        mid_x = image_width * 0.5
        center_error_px = bottom_center - mid_x
        heading_error_px = bottom_center - upper_center
        lateral_scale = float(self.get_parameter('camera_lateral_scale').value)
        forward_scale = float(self.get_parameter('camera_forward_scale').value)
        center_gain = float(self.get_parameter('lane_center_gain').value)
        heading_gain = float(self.get_parameter('lane_heading_gain').value)

        lateral_error = center_error_px * lateral_scale * center_gain
        heading_error = heading_error_px * lateral_scale * heading_gain

        path: List[Point] = []
        for x in self._path_x_samples():
            curvature_bias = curvature_gain * heading_error * x * x
            y = lateral_error + heading_error * forward_scale * x + curvature_bias
            path.append((x, float(np.clip(y, -2.0, 2.0))))
        return path

    @staticmethod
    def _median_x_near_y(xs: np.ndarray, ys: np.ndarray, yq: float) -> Optional[float]:
        if len(xs) == 0:
            return None
        band = np.abs(ys - yq) < 22.0
        if np.count_nonzero(band) < 8:
            return None
        return float(np.median(xs[band]))

    def _build_cone_center_path(self, cones: Sequence[Point]) -> Optional[List[Point]]:
        if len(cones) < 2:
            return None

        left, right = self._split_front_cones_by_side(cones)
        self.current_left_boundary = list(left)
        self.current_right_boundary = list(right)
        if len(left) >= 2:
            self.last_left_boundary = list(left)
        if len(right) >= 2:
            self.last_right_boundary = list(right)

        center_points = self._midpoint_waypoints_from_cone_pairs(left, right)
        self.current_cone_centerline = list(center_points)
        if center_points:
            self.last_cone_centerline = list(center_points)

        if not center_points:
            return None

        center_points = self._sanitize_waypoints(center_points)
        if not center_points:
            return None

        path = self._resample_polyline(
            [(0.0, 0.0)] + center_points,
            spacing=max(float(self.get_parameter('path_spacing').value), 0.10),
            target_length=float(self.get_parameter('path_length').value),
        )

        if len(path) < 3:
            return None

        return self._smooth_path(path, iterations=2)

    def _split_front_cones_by_side(self, cones: Sequence[Point]) -> Tuple[List[Point], List[Point]]:
        side_min_y = float(self.get_parameter('cone_side_min_y').value)
        lateral_limit = float(self.get_parameter('cone_boundary_lateral_limit').value)

        front_cones = [
            p for p in cones
            if p[0] > 0.05 and abs(p[1]) <= lateral_limit
        ]
        left = [p for p in front_cones if p[1] >= side_min_y]
        right = [p for p in front_cones if p[1] <= -side_min_y]

        return (
            sorted(left, key=lambda p: (p[0], math.hypot(p[0], p[1]))),
            sorted(right, key=lambda p: (p[0], math.hypot(p[0], p[1]))),
        )

    def _midpoint_waypoints_from_cone_pairs(
        self,
        left: Sequence[Point],
        right: Sequence[Point],
    ) -> List[Point]:
        if not left or not right:
            return []

        min_width = float(self.get_parameter('cone_pair_min_width').value)
        max_width = float(self.get_parameter('cone_pair_max_width').value)
        max_x_gap = float(self.get_parameter('cone_pair_max_x_gap').value)
        max_pairs = int(self.get_parameter('cone_waypoint_max_pairs').value)

        used_right = set()
        waypoints: List[Point] = []

        for left_cone in left:
            best_idx = None
            best_cost = float('inf')

            for idx, right_cone in enumerate(right):
                if idx in used_right:
                    continue

                x_gap = abs(left_cone[0] - right_cone[0])
                width = math.hypot(left_cone[0] - right_cone[0], left_cone[1] - right_cone[1])
                if x_gap > max_x_gap or not (min_width <= width <= max_width):
                    continue

                cost = x_gap + 0.15 * abs(math.hypot(*left_cone) - math.hypot(*right_cone))
                if cost < best_cost:
                    best_cost = cost
                    best_idx = idx

            if best_idx is None:
                continue

            used_right.add(best_idx)
            right_cone = right[best_idx]
            waypoints.append((
                0.5 * (left_cone[0] + right_cone[0]),
                0.5 * (left_cone[1] + right_cone[1]),
            ))

            if len(waypoints) >= max(max_pairs, 1):
                break

        return sorted(waypoints, key=lambda p: p[0])

    def _split_cones_into_boundaries(self, cones: Sequence[Point]) -> Tuple[List[Point], List[Point]]:
        side_min_y = float(self.get_parameter('cone_side_min_y').value)
        boundary_lateral_limit = float(self.get_parameter('cone_boundary_lateral_limit').value)
        center_ignore_x = float(self.get_parameter('cone_center_ignore_x').value)
        valid = self._sort_cones_by_vehicle_distance([
            p for p in cones
            if p[0] > 0.05 and abs(p[1]) <= boundary_lateral_limit
        ])
        if not valid:
            return [], []

        left_seed = self._select_boundary_seed(valid, side=1)
        right_seed = self._select_boundary_seed(valid, side=-1)

        left: List[Point] = [left_seed] if left_seed is not None else []
        right: List[Point] = [right_seed] if right_seed is not None else []

        used = set()
        if left_seed is not None:
            used.add(self._point_key(left_seed))
        if right_seed is not None:
            used.add(self._point_key(right_seed))

        if not left or not right:
            return (
                self._sort_cones_by_vehicle_distance([p for p in valid if p[1] > side_min_y]),
                self._sort_cones_by_vehicle_distance([p for p in valid if p[1] < -side_min_y]),
            )

        link_max = float(self.get_parameter('cone_boundary_link_max').value)
        for point in valid:
            key = self._point_key(point)
            if key in used:
                continue
            if abs(point[1]) < side_min_y and point[0] < center_ignore_x:
                continue

            left_cost = self._boundary_assign_cost(point, left, link_max)
            right_cost = self._boundary_assign_cost(point, right, link_max)
            if not math.isfinite(left_cost) and not math.isfinite(right_cost):
                continue

            if left_cost < right_cost:
                left.append(point)
            else:
                right.append(point)
            used.add(key)

        return self._sort_boundary(left), self._sort_boundary(right)

    def _select_boundary_seed(self, points: Sequence[Point], side: int) -> Optional[Point]:
        side_min_y = float(self.get_parameter('cone_side_min_y').value)
        neighbor_radius = float(self.get_parameter('cone_seed_neighbor_radius').value)
        if side > 0:
            candidates = [p for p in points if p[1] > side_min_y]
        else:
            candidates = [p for p in points if p[1] < -side_min_y]

        candidates = self._sort_cones_by_vehicle_distance(candidates)
        if not candidates:
            return None

        for candidate in candidates:
            for other in candidates:
                if other == candidate:
                    continue
                if other[0] < candidate[0] - 0.50:
                    continue
                if math.hypot(other[0] - candidate[0], other[1] - candidate[1]) <= neighbor_radius:
                    return candidate

        return candidates[0]

    @staticmethod
    def _point_key(point: Point) -> Tuple[int, int]:
        return (round(point[0] * 100), round(point[1] * 100))

    @staticmethod
    def _sort_boundary(points: Sequence[Point]) -> List[Point]:
        return sorted(points, key=lambda p: (math.hypot(p[0], p[1]), p[0]))

    @staticmethod
    def _boundary_assign_cost(point: Point, boundary: Sequence[Point], link_max: float) -> float:
        if not boundary:
            return float('inf')

        last = boundary[-1]
        link_dist = math.hypot(point[0] - last[0], point[1] - last[1])
        if link_dist > link_max or point[0] < last[0] - 0.80:
            return float('inf')

        heading_penalty = 0.0
        if len(boundary) >= 2:
            prev = boundary[-2]
            h0 = math.atan2(last[1] - prev[1], max(last[0] - prev[0], 1e-3))
            h1 = math.atan2(point[1] - last[1], max(point[0] - last[0], 1e-3))
            heading_penalty = abs(TrackDriverNode._normalize_angle(h1 - h0))

        return link_dist + 0.45 * heading_penalty

    @staticmethod
    def _sort_cones_by_vehicle_distance(points: Sequence[Point]) -> List[Point]:
        return sorted(
            [p for p in points if p[0] > 0.05],
            key=lambda p: (math.hypot(p[0], p[1]), p[0]),
        )

    def _ranked_cone_midpoint_waypoints(self, left: Sequence[Point], right: Sequence[Point]) -> List[Point]:
        if not left or not right:
            return []

        min_width = float(self.get_parameter('cone_pair_min_width').value)
        max_width = float(self.get_parameter('cone_pair_max_width').value)
        max_x_gap = float(self.get_parameter('cone_pair_max_x_gap').value)
        max_pairs = int(self.get_parameter('cone_waypoint_max_pairs').value)

        pair_candidates = []
        for left_idx, left_cone in enumerate(left):
            left_range = math.hypot(left_cone[0], left_cone[1])
            for right_idx, right_cone in enumerate(right):
                right_range = math.hypot(right_cone[0], right_cone[1])
                width = math.hypot(left_cone[0] - right_cone[0], left_cone[1] - right_cone[1])
                x_gap = abs(left_cone[0] - right_cone[0])
                if not (min_width <= width <= max_width) or x_gap > max_x_gap:
                    continue

                midpoint = (
                    0.5 * (left_cone[0] + right_cone[0]),
                    0.5 * (left_cone[1] + right_cone[1]),
                )
                if midpoint[0] < 0.10:
                    continue

                midpoint_range = math.hypot(midpoint[0], midpoint[1])
                range_gap = abs(left_range - right_range)
                rank_gap = abs(left_idx - right_idx)
                score = midpoint_range + 0.55 * range_gap + 0.45 * x_gap + 0.20 * rank_gap
                pair_candidates.append((score, midpoint_range, left_idx, right_idx, midpoint))

        if not pair_candidates:
            return []

        pair_candidates.sort(key=lambda item: item[0])
        used_left = set()
        used_right = set()
        selected = []

        for _, midpoint_range, left_idx, right_idx, midpoint in pair_candidates:
            if left_idx in used_left or right_idx in used_right:
                continue
            used_left.add(left_idx)
            used_right.add(right_idx)
            selected.append((midpoint_range, midpoint))
            if len(selected) >= max(max_pairs, 1):
                break

        selected.sort(key=lambda item: item[0])
        return [midpoint for _, midpoint in selected]

    def _centerline_from_boundaries(self, left: Sequence[Point], right: Sequence[Point]) -> List[Point]:
        if len(left) < 2 or len(right) < 2:
            return []

        left_line = self._sanitize_waypoints(left)
        right_line = self._sanitize_waypoints(right)
        if len(left_line) < 2 or len(right_line) < 2:
            return []

        spacing = max(float(self.get_parameter('path_spacing').value), 0.10)
        target_length = float(self.get_parameter('path_length').value)
        min_width = float(self.get_parameter('cone_pair_min_width').value)
        max_width = float(self.get_parameter('cone_pair_max_width').value)

        left_samples = self._resample_polyline(left_line, spacing=spacing, target_length=target_length)
        right_samples = self._resample_polyline(right_line, spacing=spacing, target_length=target_length)
        sample_count = min(len(left_samples), len(right_samples))

        center_points: List[Point] = []
        for idx in range(sample_count):
            left_pt = left_samples[idx]
            right_pt = right_samples[idx]
            width = math.hypot(left_pt[0] - right_pt[0], left_pt[1] - right_pt[1])
            if not (0.65 * min_width <= width <= 1.45 * max_width):
                continue

            center_points.append((
                0.5 * (left_pt[0] + right_pt[0]),
                0.5 * (left_pt[1] + right_pt[1]),
            ))

        return self._sanitize_waypoints(center_points)

    def _offset_side_to_center(self, side_points: Sequence[Point], side: str, lane_width: float) -> List[Point]:
        points = self._sort_cones_by_vehicle_distance(side_points)
        if len(points) < 2:
            return []

        direction = -1.0 if side == 'left' else 1.0
        half_width = 0.5 * lane_width
        center_points: List[Point] = []

        for i, point in enumerate(points):
            prev_i = max(i - 1, 0)
            next_i = min(i + 1, len(points) - 1)
            dx = points[next_i][0] - points[prev_i][0]
            dy = points[next_i][1] - points[prev_i][1]
            norm = math.hypot(dx, dy)
            if norm < 1e-6:
                continue

            nx = -dy / norm
            ny = dx / norm
            center_points.append((point[0] + direction * half_width * nx,
                                  point[1] + direction * half_width * ny))

        return center_points

    def _update_school_zone_state(self, image: Optional[np.ndarray]):
        self.school_zone_yellow_left_ratio = 0.0
        self.school_zone_yellow_right_ratio = 0.0
        self.school_zone_yellow_pair_row_ratio = 0.0
        self.school_zone_yellow_bottom_pair_row_ratio = 0.0
        self.school_zone_yellow_separation_ratio = 0.0
        self.school_zone_candidate_active = False
        if image is None or not bool(self.get_parameter('school_zone_enabled').value):
            self.school_zone_active = False
            self.school_zone_confirm_count = 0
            self.school_zone_lost_count = 0
            self.school_zone_last_seen_sec = None
            self.school_zone_candidate_last_seen_sec = None
            return
        if (
            bool(self.get_parameter('school_zone_suppress_when_vehicle_visible').value)
            and (self.cached_yolo_vehicle or self._has_yolo_confirmed_vehicle_track())
        ):
            self.school_zone_active = False
            self.school_zone_confirm_count = 0
            self.school_zone_lost_count = 0
            self.school_zone_last_seen_sec = None
            self.school_zone_candidate_last_seen_sec = None
            return

        height, width = image.shape[:2]
        top_ratio = float(np.clip(self.get_parameter('school_zone_roi_top_ratio').value, 0.0, 0.95))
        y0 = int(height * top_ratio)
        roi = image[y0:height, :]
        if roi.size == 0:
            return

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(
            hsv,
            np.array([18, 70, 80], dtype=np.uint8),
            np.array([38, 255, 255], dtype=np.uint8),
        )
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        self._mask_school_zone_vehicle_boxes(yellow_mask, y0)

        left_edge_max = float(np.clip(
            self.get_parameter('school_zone_left_edge_max_ratio').value, 0.05, 0.49))
        right_edge_min = float(np.clip(
            self.get_parameter('school_zone_right_edge_min_ratio').value, 0.51, 0.95))
        left_end = max(int(width * left_edge_max), 1)
        right_start = min(int(width * right_edge_min), width - 1)

        left = yellow_mask[:, :left_end]
        right = yellow_mask[:, right_start:]
        left_pixels = int(np.count_nonzero(left))
        right_pixels = int(np.count_nonzero(right))
        left_area = float(max(left.size, 1))
        right_area = float(max(right.size, 1))
        self.school_zone_yellow_left_ratio = left_pixels / left_area
        self.school_zone_yellow_right_ratio = right_pixels / right_area
        min_row_pixels = max(2, int(width * 0.003))
        max_row_pixels = max(min_row_pixels, int(width * float(np.clip(
            self.get_parameter('school_zone_yellow_row_max_width_ratio').value, 0.01, 0.40))))
        left_row_counts = np.count_nonzero(left, axis=1)
        right_row_counts = np.count_nonzero(right, axis=1)
        left_row_mask = (left_row_counts >= min_row_pixels) & (left_row_counts <= max_row_pixels)
        right_row_mask = (right_row_counts >= min_row_pixels) & (right_row_counts <= max_row_pixels)
        left_row_hits = int(np.count_nonzero(left_row_mask))
        right_row_hits = int(np.count_nonzero(right_row_mask))
        left_row_ratio = left_row_hits / float(max(left.shape[0], 1))
        right_row_ratio = right_row_hits / float(max(right.shape[0], 1))
        min_separation = float(np.clip(
            self.get_parameter('school_zone_yellow_min_separation_ratio').value, 0.10, 0.90))
        paired_rows = np.nonzero(left_row_mask & right_row_mask)[0]
        valid_pair_rows: List[int] = []
        separations: List[float] = []
        for row_idx in paired_rows:
            left_xs = np.flatnonzero(left[row_idx])
            right_xs = np.flatnonzero(right[row_idx])
            if len(left_xs) < min_row_pixels or len(right_xs) < min_row_pixels:
                continue
            left_x = float(np.median(left_xs))
            right_x = float(right_start + np.median(right_xs))
            separation = (right_x - left_x) / float(max(width, 1))
            if separation < min_separation:
                continue
            valid_pair_rows.append(int(row_idx))
            separations.append(separation)

        pair_row_hits = len(valid_pair_rows)
        self.school_zone_yellow_pair_row_ratio = pair_row_hits / float(max(left.shape[0], 1))
        if separations:
            self.school_zone_yellow_separation_ratio = float(np.median(separations))
        bottom_start = int(left.shape[0] * 0.55)
        bottom_pair_hits = sum(1 for row_idx in valid_pair_rows if row_idx >= bottom_start)
        self.school_zone_yellow_bottom_pair_row_ratio = bottom_pair_hits / float(
            max(left.shape[0] - bottom_start, 1)
        )

        ratio_threshold = float(np.clip(
            self.get_parameter('school_zone_yellow_ratio_threshold').value, 0.0, 1.0))
        row_ratio_threshold = float(np.clip(
            self.get_parameter('school_zone_yellow_row_ratio_threshold').value, 0.0, 1.0))
        pair_row_threshold = float(np.clip(
            self.get_parameter('school_zone_yellow_pair_row_ratio_threshold').value, 0.0, 1.0))
        bottom_pair_threshold = float(np.clip(
            self.get_parameter('school_zone_yellow_bottom_pair_row_ratio_threshold').value, 0.0, 1.0))
        min_pixels = max(int(self.get_parameter('school_zone_yellow_min_pixels').value), 1)
        min_pair_rows = max(int(self.get_parameter('school_zone_yellow_min_pair_rows').value), 1)
        preslow_ratio = float(np.clip(
            self.get_parameter('school_zone_preslow_ratio').value, 0.30, 1.00))
        hold_sec = max(float(self.get_parameter('school_zone_hold_sec').value), 0.0)
        now = time.monotonic()
        raw_active = (
            left_pixels >= min_pixels
            and right_pixels >= min_pixels
            and self.school_zone_yellow_left_ratio >= ratio_threshold
            and self.school_zone_yellow_right_ratio >= ratio_threshold
            and left_row_ratio >= row_ratio_threshold
            and right_row_ratio >= row_ratio_threshold
            and pair_row_hits >= min_pair_rows
            and self.school_zone_yellow_pair_row_ratio >= pair_row_threshold
            and self.school_zone_yellow_bottom_pair_row_ratio >= bottom_pair_threshold
            and self.school_zone_yellow_separation_ratio >= min_separation
        )
        candidate_raw = raw_active or (
            left_pixels >= int(min_pixels * preslow_ratio)
            and right_pixels >= int(min_pixels * preslow_ratio)
            and left_row_ratio >= row_ratio_threshold * preslow_ratio
            and right_row_ratio >= row_ratio_threshold * preslow_ratio
            and pair_row_hits >= max(3, int(min_pair_rows * preslow_ratio))
            and self.school_zone_yellow_pair_row_ratio >= pair_row_threshold * preslow_ratio
            and self.school_zone_yellow_bottom_pair_row_ratio >= bottom_pair_threshold * preslow_ratio
            and self.school_zone_yellow_separation_ratio >= min_separation
        )
        if candidate_raw:
            self.school_zone_candidate_active = True
            self.school_zone_candidate_last_seen_sec = now
        elif (
            self.school_zone_candidate_last_seen_sec is not None
            and now - self.school_zone_candidate_last_seen_sec <= hold_sec
        ):
            self.school_zone_candidate_active = True
        else:
            self.school_zone_candidate_active = False

        confirm_frames = max(int(self.get_parameter('school_zone_confirm_frames').value), 1)
        lost_frames = max(int(self.get_parameter('school_zone_lost_frames').value), 0)
        if raw_active:
            self.school_zone_confirm_count += 1
            self.school_zone_lost_count = 0
            self.school_zone_last_seen_sec = now
        else:
            self.school_zone_confirm_count = 0
            self.school_zone_lost_count += 1

        if self.school_zone_confirm_count >= confirm_frames:
            self.school_zone_active = True
        elif (
            self.school_zone_last_seen_sec is not None
            and now - self.school_zone_last_seen_sec <= hold_sec
        ):
            self.school_zone_active = True
        elif self.school_zone_lost_count > lost_frames:
            self.school_zone_active = False

    def _mask_school_zone_vehicle_boxes(self, mask: np.ndarray, roi_y0: int):
        if not bool(self.get_parameter('school_zone_mask_vehicle_boxes').value):
            return

        boxes: List[Tuple[int, int, int, int]] = []
        boxes.extend(box for box, _, _ in self.cached_yolo_vehicle_detections)
        now = time.monotonic()
        yolo_timeout = max(float(self.get_parameter('vehicle_yolo_required_timeout_sec').value), 0.02)
        boxes.extend(
            track.box for track in self.vehicle_tracks.values()
            if track.box is not None and now - track.yolo_last_seen_sec <= yolo_timeout
        )
        if not boxes:
            return

        height, width = mask.shape[:2]
        pad = max(int(width * 0.04), 6)
        for x0, y0, x1, y1 in boxes:
            mx0 = max(int(x0) - pad, 0)
            mx1 = min(int(x1) + pad, width)
            my0 = max(int(y0) - int(roi_y0) - pad, 0)
            my1 = min(int(y1) - int(roi_y0) + pad, height)
            if mx1 <= mx0 or my1 <= my0:
                continue
            mask[my0:my1, mx0:mx1] = 0

    @staticmethod
    def _sanitize_waypoints(points: Sequence[Point]) -> List[Point]:
        waypoints: List[Point] = []
        for point in points:
            if point[0] < 0.10:
                continue
            if waypoints and math.hypot(point[0] - waypoints[-1][0], point[1] - waypoints[-1][1]) < 0.25:
                continue
            waypoints.append(point)
        return waypoints

    @staticmethod
    def _order_forward_points(points: Sequence[Point]) -> List[Point]:
        ordered = sorted([p for p in points if p[0] > 0.05], key=lambda p: (p[0], abs(p[1])))
        if not ordered:
            return []

        filtered: List[Point] = []
        for point in ordered:
            if filtered and math.hypot(point[0] - filtered[-1][0], point[1] - filtered[-1][1]) < 0.25:
                continue
            if filtered and point[0] < filtered[-1][0] - 0.35:
                continue
            filtered.append(point)

        return filtered

    @staticmethod
    def _resample_polyline(points: Sequence[Point], spacing: float, target_length: float) -> List[Point]:
        if len(points) < 2:
            return list(points)

        cumulative = [0.0]
        for p0, p1 in zip(points[:-1], points[1:]):
            cumulative.append(cumulative[-1] + math.hypot(p1[0] - p0[0], p1[1] - p0[1]))

        if cumulative[-1] < 1e-6:
            return list(points)

        samples: List[Point] = []
        max_s = max(cumulative[-1], min(target_length, cumulative[-1]))
        s = 0.0
        idx = 1
        while s <= max_s + 1e-6:
            while idx < len(cumulative) - 1 and cumulative[idx] < s:
                idx += 1

            p0 = points[idx - 1]
            p1 = points[idx]
            seg_len = max(cumulative[idx] - cumulative[idx - 1], 1e-6)
            t = (s - cumulative[idx - 1]) / seg_len
            samples.append((p0[0] + t * (p1[0] - p0[0]), p0[1] + t * (p1[1] - p0[1])))
            s += spacing

        extended_length = cumulative[-1]
        while len(samples) >= 2 and extended_length < target_length:
            p0 = samples[-2]
            p1 = samples[-1]
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            norm = math.hypot(dx, dy)
            if norm < 1e-6:
                break
            samples.append((p1[0] + spacing * dx / norm, p1[1] + spacing * dy / norm))
            extended_length += spacing

        return samples

    @staticmethod
    def _path_y_at_x(path: Sequence[Point], x_query: float) -> float:
        if not path:
            return 0.0
        if len(path) == 1:
            return path[0][1]

        ordered = sorted(path, key=lambda p: p[0])
        if x_query <= ordered[0][0]:
            return ordered[0][1]

        for p0, p1 in zip(ordered[:-1], ordered[1:]):
            if p0[0] <= x_query <= p1[0]:
                dx = p1[0] - p0[0]
                if abs(dx) < 1e-6:
                    return p1[1]
                t = (x_query - p0[0]) / dx
                return p0[1] + t * (p1[1] - p0[1])

        return ordered[-1][1]

    @staticmethod
    def _heading_at(path: Sequence[Point], idx: int) -> float:
        if len(path) < 2:
            return 0.0

        idx = max(0, min(idx, len(path) - 1))
        i0 = max(idx - 1, 0)
        i1 = min(idx + 1, len(path) - 1)
        return math.atan2(path[i1][1] - path[i0][1], max(path[i1][0] - path[i0][0], 1e-3))

    @staticmethod
    def _heading_at_arc(path: Sequence[Point], arc_query: float) -> float:
        if len(path) < 2:
            return 0.0

        accum = 0.0
        prev = (0.0, 0.0)
        target_idx = len(path) - 1
        for idx, point in enumerate(path):
            accum += math.hypot(point[0] - prev[0], point[1] - prev[1])
            prev = point
            if accum >= arc_query:
                target_idx = idx
                break

        return TrackDriverNode._heading_at(path, target_idx)

    def _legacy_cone_center_path(self, left: Sequence[Point], right: Sequence[Point]) -> Optional[List[Point]]:
        lane_width = float(self.get_parameter('lane_width').value)
        center_points: List[Point] = []
        if left and right:
            for x in self._path_x_samples():
                ly = self._interp_side_y(left, x)
                ry = self._interp_side_y(right, x)
                if ly is not None and ry is not None:
                    center_points.append((x, 0.5 * (ly + ry)))
        elif left:
            for x in self._path_x_samples():
                ly = self._interp_side_y(left, x)
                if ly is not None:
                    center_points.append((x, ly - 0.5 * lane_width))
        elif right:
            for x in self._path_x_samples():
                ry = self._interp_side_y(right, x)
                if ry is not None:
                    center_points.append((x, ry + 0.5 * lane_width))

        if len(center_points) < 3:
            return None

        return self._smooth_path(center_points, iterations=2)

    @staticmethod
    def _interp_side_y(points: Sequence[Point], x_query: float) -> Optional[float]:
        near = [p for p in points if abs(p[0] - x_query) < 1.5]
        if near:
            weights = [1.0 / max(abs(p[0] - x_query), 0.15) for p in near]
            return float(sum(w * p[1] for w, p in zip(weights, near)) / sum(weights))

        if len(points) == 1:
            return points[0][1]

        for p0, p1 in zip(points[:-1], points[1:]):
            if p0[0] <= x_query <= p1[0] or p1[0] <= x_query <= p0[0]:
                dx = max(abs(p1[0] - p0[0]), 1e-3)
                t = (x_query - p0[0]) / dx
                return float(p0[1] + t * (p1[1] - p0[1]))

        return None

    def _fuse_center_paths(
        self,
        lane_path: Optional[List[Point]],
        cone_path: Optional[List[Point]],
    ) -> List[Point]:
        if lane_path is not None and cone_path is not None:
            cone_weight = float(np.clip(self.get_parameter('cone_path_weight').value, 0.0, 1.0))
            fused = []
            for cone_x, cone_y in cone_path:
                lane_y = self._path_y_at_x(lane_path, cone_x)
                fused.append((cone_x, (1.0 - cone_weight) * lane_y + cone_weight * cone_y))
            self.last_center_path = self._smooth_path(fused, iterations=1)
            self.last_mode = 'lane+cone'
            return self.last_center_path

        if cone_path is not None:
            self.last_center_path = cone_path
            self.last_mode = 'cone'
            return cone_path

        if lane_path is not None:
            self.last_center_path = lane_path
            self.last_mode = 'lane'
            return lane_path

        self.last_mode = 'memory'
        return self.last_center_path if len(self.last_center_path) >= 2 else self._straight_path()

    def _make_lattice_candidates(self, center_path: Sequence[Point]) -> List[PathCandidate]:
        offset_param = 'cone_lattice_offsets' if 'cone' in self.last_mode else 'lattice_offsets'
        offsets = [float(v) for v in self.get_parameter(offset_param).value]
        candidates: List[PathCandidate] = []

        for offset in offsets:
            shifted = self._offset_path(center_path, offset)
            if len(shifted) < 2:
                continue
            candidates.append(PathCandidate(offset=offset, path=shifted, cost=0.0, min_clearance=float('inf')))

        return candidates

    def _select_best_candidate(
        self,
        candidates: Sequence[PathCandidate],
        cones: Sequence[Point],
    ) -> Optional[PathCandidate]:
        if not candidates:
            return None

        collision_radius = float(self.get_parameter('cone_collision_radius').value)
        clearance_weight = float(self.get_parameter('cone_clearance_weight').value)
        offset_weight = float(self.get_parameter('offset_weight').value)
        if 'cone' in self.last_mode:
            offset_weight = float(self.get_parameter('cone_offset_weight').value)
        continuity_weight = float(self.get_parameter('continuity_weight').value)
        curvature_weight = float(self.get_parameter('curvature_weight').value)

        best: Optional[PathCandidate] = None
        best_cost = float('inf')

        for candidate in candidates:
            min_clearance = self._min_distance_to_points(candidate.path, cones)
            candidate.min_clearance = min_clearance

            clearance_cost = 0.0 if not cones else clearance_weight / max(min_clearance, 0.05)
            near_cone_penalty = 0.0
            if min_clearance < collision_radius:
                near_cone_penalty = 20.0 * (collision_radius - min_clearance)
            offset_cost = offset_weight * abs(candidate.offset)
            continuity_cost = continuity_weight * abs(candidate.offset - self.prev_offset)
            curvature_cost = curvature_weight * self._path_curvature_cost(candidate.path)

            candidate.cost = clearance_cost + near_cone_penalty + offset_cost + continuity_cost + curvature_cost
            if candidate.cost < best_cost:
                best_cost = candidate.cost
                best = candidate

        return best

    @staticmethod
    def _min_distance_to_points(path: Sequence[Point], points: Sequence[Point]) -> float:
        if not points:
            return float('inf')
        best = float('inf')
        for point in points:
            for p0, p1 in zip(path[:-1], path[1:]):
                best = min(best, TrackDriverNode._point_to_segment_distance(point, p0, p1))
        return best

    @staticmethod
    def _point_to_segment_distance(point: Point, p0: Point, p1: Point) -> float:
        vx = p1[0] - p0[0]
        vy = p1[1] - p0[1]
        wx = point[0] - p0[0]
        wy = point[1] - p0[1]
        denom = vx * vx + vy * vy
        if denom < 1e-6:
            return math.hypot(point[0] - p0[0], point[1] - p0[1])
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
        proj = (p0[0] + t * vx, p0[1] + t * vy)
        return math.hypot(point[0] - proj[0], point[1] - proj[1])

    @staticmethod
    def _path_curvature_cost(path: Sequence[Point]) -> float:
        if len(path) < 3:
            return 0.0
        total = 0.0
        for i in range(1, len(path) - 1):
            h0 = math.atan2(path[i][1] - path[i - 1][1], path[i][0] - path[i - 1][0])
            h1 = math.atan2(path[i + 1][1] - path[i][1], path[i + 1][0] - path[i][0])
            total += abs(TrackDriverNode._normalize_angle(h1 - h0))
        return total

    def _pure_pursuit_steer(
        self,
        path: Sequence[Point],
        max_steer_override: Optional[float] = None,
        smoothing_override: Optional[float] = None,
        lookahead_scale_override: Optional[float] = None,
    ) -> float:
        if len(path) < 2:
            return 0.0

        lookahead = self._lookahead_distance()
        if lookahead_scale_override is not None:
            lookahead *= float(np.clip(lookahead_scale_override, 0.30, 1.50))
            lookahead = max(lookahead, 0.45)
        target = path[-1]
        target_idx = len(path) - 1
        accum = 0.0
        prev = (0.0, 0.0)

        for idx, point in enumerate(path):
            accum += math.hypot(point[0] - prev[0], point[1] - prev[1])
            prev = point
            if accum >= lookahead:
                target = point
                target_idx = idx
                break

        alpha = math.atan2(target[1], max(target[0], 0.05))
        ld = max(math.hypot(target[0], target[1]), 0.25)
        wheelbase = float(self.get_parameter('wheelbase').value)
        steer_rad = math.atan2(2.0 * wheelbase * math.sin(alpha), ld)
        heading = self._heading_at(path, target_idx)
        curve_heading = self._heading_at_arc(path, min(lookahead * 2.2, 4.0))
        cte = self._path_y_at_x(path, min(lookahead * 0.65, 2.2))
        heading_gain = float(self.get_parameter('heading_steer_gain').value)
        cte_gain = float(self.get_parameter('cross_track_steer_gain').value)
        curve_gain = float(self.get_parameter('curve_preview_gain').value)
        steer_deg = math.degrees(
            steer_rad
            + heading_gain * heading
            + curve_gain * curve_heading
            + math.atan2(cte_gain * cte, max(ld, 0.5))
        )

        max_steer = (
            float(self.get_parameter('max_steer_deg').value)
            if max_steer_override is None
            else max(float(max_steer_override), 1.0)
        )
        steer_deg = float(np.clip(steer_deg, -max_steer, max_steer))
        if bool(self.get_parameter('invert_steering').value):
            steer_deg = -steer_deg

        smoothing = (
            float(self.get_parameter('steer_smoothing').value)
            if smoothing_override is None
            else float(smoothing_override)
        )
        smoothing = float(np.clip(smoothing, 0.0, 0.95))
        return (1.0 - smoothing) * steer_deg + smoothing * self.prev_steer

    def _boost_person_avoidance_steer(self, steer: float, path: Sequence[Point]) -> float:
        gain = max(float(self.get_parameter('person_avoidance_steer_gain').value), 1.0)
        max_steer = max(float(self.get_parameter('person_avoidance_max_steer_deg').value), 1.0)
        min_steer = max(float(self.get_parameter('person_avoidance_min_steer_deg').value), 0.0)

        boosted = steer * gain
        if abs(boosted) < min_steer:
            sign = 1.0 if boosted > 0.0 else -1.0 if boosted < 0.0 else self._person_avoidance_steer_sign(path)
            boosted = sign * min_steer

        return float(np.clip(boosted, -max_steer, max_steer))

    def _person_avoidance_steer_sign(self, path: Sequence[Point]) -> float:
        lookahead = self._lookahead_distance()
        target_y = self._path_y_at_x(path, min(lookahead, 2.0))
        sign = 1.0 if target_y >= 0.0 else -1.0
        if bool(self.get_parameter('invert_steering').value):
            sign = -sign
        return sign

    def _target_speed(self, steer: float, cones: Sequence[Point], min_clearance: float) -> float:
        base_speed = float(self.get_parameter('base_speed').value)
        min_speed = float(self.get_parameter('min_speed').value)
        max_steer = max(float(self.get_parameter('max_steer_deg').value), 1.0)

        steer_ratio = min(abs(steer) / max_steer, 1.0)
        speed = base_speed - (base_speed - min_speed) * steer_ratio

        stop_distance = float(self.get_parameter('stop_distance').value)
        slow_distance = float(self.get_parameter('slow_distance').value)
        front_half_width = float(self.get_parameter('front_obstacle_half_width').value)
        nearest_front = min(
            [p[0] for p in cones if abs(p[1]) < front_half_width],
            default=float('inf'),
        )
        if nearest_front < stop_distance:
            return 0.0
        if nearest_front < slow_distance:
            speed = min(speed, min_speed)
        if min_clearance < 0.55:
            speed = min(speed, min_speed)

        return max(0.0, speed)

    def _lookahead_distance(self) -> float:
        min_ld = float(self.get_parameter('lookahead_min').value)
        max_ld = float(self.get_parameter('lookahead_max').value)
        base_speed = float(self.get_parameter('base_speed').value)
        lookahead = 0.35 * base_speed
        if 'cone' in self.last_mode:
            lookahead *= float(self.get_parameter('cone_lookahead_scale').value)
        return float(np.clip(lookahead, min_ld, max_ld))

    def _offset_path(self, path: Sequence[Point], offset: float) -> List[Point]:
        if len(path) < 2:
            return []

        out: List[Point] = []
        for i, point in enumerate(path):
            prev_i = max(i - 1, 0)
            next_i = min(i + 1, len(path) - 1)
            dx = path[next_i][0] - path[prev_i][0]
            dy = path[next_i][1] - path[prev_i][1]
            norm = math.hypot(dx, dy)
            if norm < 1e-6:
                out.append(point)
                continue

            nx = -dy / norm
            ny = dx / norm
            out.append((point[0] + offset * nx, point[1] + offset * ny))

        return self._smooth_path(out, iterations=1)

    def _straight_path(self) -> List[Point]:
        return [(x, 0.0) for x in self._path_x_samples()]

    def _path_x_samples(self) -> List[float]:
        length = float(self.get_parameter('path_length').value)
        spacing = max(float(self.get_parameter('path_spacing').value), 0.10)
        count = max(3, int(length / spacing) + 1)
        return [i * spacing for i in range(count)]

    @staticmethod
    def _smooth_path(path: Sequence[Point], iterations: int = 2) -> List[Point]:
        if len(path) < 5:
            return list(path)

        smoothed = list(path)
        for _ in range(iterations):
            copy = smoothed[:]
            for i in range(2, len(smoothed) - 2):
                copy[i] = (
                    0.08 * smoothed[i - 2][0]
                    + 0.22 * smoothed[i - 1][0]
                    + 0.40 * smoothed[i][0]
                    + 0.22 * smoothed[i + 1][0]
                    + 0.08 * smoothed[i + 2][0],
                    0.08 * smoothed[i - 2][1]
                    + 0.22 * smoothed[i - 1][1]
                    + 0.40 * smoothed[i][1]
                    + 0.22 * smoothed[i + 1][1]
                    + 0.08 * smoothed[i + 2][1],
                )
            smoothed = copy

        return smoothed

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _publish_debug_visualization(
        self,
        raw_cones: Sequence[Point],
        track_cones: Sequence[Point],
        lane_path: Optional[Sequence[Point]],
        cone_path: Optional[Sequence[Point]],
        center_path: Sequence[Point],
        selected_path: Optional[Sequence[Point]],
    ):
        if not bool(self.get_parameter('publish_debug_visualization').value):
            return

        frame_id = str(self.get_parameter('viz_frame_id').value)
        lifetime_sec = max(float(self.get_parameter('viz_marker_lifetime_sec').value), 0.0)
        stamp = self.get_clock().now().to_msg()

        markers = MarkerArray()
        self._append_sphere_markers(
            markers, track_cones, 'cones', 1, frame_id, stamp,
            color=(1.00, 0.55, 0.00, 1.00), size=0.16, lifetime_sec=lifetime_sec)
        self._append_sphere_markers(
            markers, self.last_cone_centerline, 'cone_midpoint_waypoints', 100, frame_id, stamp,
            color=(0.00, 0.90, 1.00, 1.00), size=0.18, lifetime_sec=lifetime_sec)
        self._append_line_marker(
            markers, self.last_cone_centerline, 'cone_centerline', 102, frame_id, stamp,
            color=(0.00, 0.90, 1.00, 0.95), width=0.08, lifetime_sec=lifetime_sec)
        marker_id = 200
        marker_id = self._append_person_debug_markers(markers, marker_id, frame_id, stamp, lifetime_sec)
        marker_id = self._append_vehicle_debug_markers(markers, marker_id, frame_id, stamp, lifetime_sec)
        self._append_school_zone_debug_marker(markers, marker_id, frame_id, stamp, lifetime_sec)

        self.marker_pub.publish(markers)

    def _append_school_zone_debug_marker(
        self,
        markers: MarkerArray,
        marker_id: int,
        frame_id: str,
        stamp,
        lifetime_sec: float,
    ) -> int:
        if not self.school_zone_active:
            return marker_id
        return self._append_text_marker(
            markers, (1.0, 0.0), 'school_zone', marker_id, frame_id, stamp,
            f'school zone speed {float(self.get_parameter("school_zone_speed").value):.1f}',
            (1.00, 0.85, 0.05, 1.00), lifetime_sec)

    def _append_person_debug_markers(
        self,
        markers: MarkerArray,
        marker_id: int,
        frame_id: str,
        stamp,
        lifetime_sec: float,
    ) -> int:
        if self.image is None:
            return marker_id

        if self.cached_yolo_person_box is not None:
            point = self.person_fusion_point or self._person_point_from_box(self.image, self.cached_yolo_person_box)
            if point is not None:
                marker_id = self._append_object_marker(
                    markers, point, 'person_yolo', marker_id, frame_id, stamp,
                    color=(1.00, 0.85, 0.05, 0.90), scale=(0.35, 0.35, 1.30),
                    marker_type=Marker.CYLINDER, lifetime_sec=lifetime_sec)
                marker_id = self._append_text_marker(
                    markers, point, 'person_label', marker_id, frame_id, stamp,
                    'person yolo', (1.00, 0.85, 0.05, 1.00), lifetime_sec)

        if self.person_fusion_point is not None:
            point = self.person_fusion_point
            marker_id = self._append_object_marker(
                markers, point, 'person_fused', marker_id, frame_id, stamp,
                color=(0.10, 1.00, 0.15, 0.95), scale=(0.45, 0.45, 1.60),
                marker_type=Marker.CYLINDER, lifetime_sec=lifetime_sec)
            marker_id = self._append_text_marker(
                markers, point, 'person_fused_label', marker_id, frame_id, stamp,
                f'person fused {point[0]:.1f}m', (0.10, 1.00, 0.15, 1.00), lifetime_sec)
            velocity_end = (point[0] + self.person_velocity[0], point[1] + self.person_velocity[1])
            marker_id = self._append_arrow_marker(
                markers, point, velocity_end, 'person_velocity', marker_id, frame_id, stamp,
                color=(0.10, 1.00, 0.15, 0.90), lifetime_sec=lifetime_sec)

        if self.person_predicted_point is not None:
            marker_id = self._append_object_marker(
                markers, self.person_predicted_point, 'person_predicted', marker_id, frame_id, stamp,
                color=(0.10, 0.80, 1.00, 0.80), scale=(0.25, 0.25, 0.25),
                marker_type=Marker.SPHERE, lifetime_sec=lifetime_sec)

        return marker_id

    def _append_vehicle_debug_markers(
        self,
        markers: MarkerArray,
        marker_id: int,
        frame_id: str,
        stamp,
        lifetime_sec: float,
    ) -> int:
        if self.image is not None:
            for box, score, class_id in self.cached_yolo_vehicle_detections:
                point = (
                    self._vehicle_lidar_point_for_camera_box(self.scan_msg, box)
                    or self._vehicle_point_from_box(self.image, box)
                )
                if point is None:
                    continue
                color = self._vehicle_marker_color(class_id, alpha=0.45)
                marker_id = self._append_object_marker(
                    markers, point, 'vehicle_yolo', marker_id, frame_id, stamp,
                    color=color, scale=(0.70, 0.45, 0.45),
                    marker_type=Marker.CUBE, lifetime_sec=lifetime_sec)
                marker_id = self._append_text_marker(
                    markers, point, 'vehicle_yolo_label', marker_id, frame_id, stamp,
                    f'{self._vehicle_class_name(class_id)} yolo {score:.2f}',
                    self._vehicle_marker_color(class_id, alpha=1.0), lifetime_sec)

        for track in sorted(self.vehicle_tracks.values(), key=lambda item: item.track_id):
            point = (track.x, track.y)
            color = self._vehicle_marker_color(track.class_id, alpha=0.95)
            marker_id = self._append_object_marker(
                markers, point, 'vehicle_track', marker_id, frame_id, stamp,
                color=color, scale=(0.80, 0.50, 0.50),
                marker_type=Marker.CUBE, lifetime_sec=lifetime_sec)
            label = (
                f'{self._vehicle_class_name(track.class_id)}#{track.track_id} '
                f'x={track.x:.1f} px={self._predicted_vehicle_x(track):.1f} '
                f'y={track.y:.1f} vx={track.vx:.1f}'
            )
            marker_id = self._append_text_marker(
                markers, point, 'vehicle_track_label', marker_id, frame_id, stamp,
                label, color, lifetime_sec)
            velocity_end = (track.x + track.vx, track.y + track.vy)
            marker_id = self._append_arrow_marker(
                markers, point, velocity_end, 'vehicle_velocity', marker_id, frame_id, stamp,
                color=color, lifetime_sec=lifetime_sec)

        return marker_id

    def _person_point_from_box(
        self,
        image: np.ndarray,
        box: Tuple[int, int, int, int],
    ) -> Optional[Point]:
        image_height, image_width = image.shape[:2]
        x0, y0, x1, y1 = box
        box_height = max(y1 - y0, 1)
        height_ratio = box_height / float(max(image_height, 1))
        if height_ratio <= 0.01:
            return None

        camera_fov = math.radians(max(float(self.get_parameter('person_fusion_camera_fov_deg').value), 1.0))
        center_x = 0.5 * (x0 + x1)
        center_ratio = (center_x - 0.5 * image_width) / max(0.5 * image_width, 1.0)
        angle = -center_ratio * 0.5 * camera_fov
        max_range = max(float(self.get_parameter('person_fusion_max_distance').value), 1.0)
        distance = float(np.clip(0.65 / max(height_ratio, 0.05), 0.50, max_range))
        return distance * math.cos(angle), distance * math.sin(angle)

    @staticmethod
    def _append_sphere_markers(
        markers: MarkerArray,
        points: Sequence[Point],
        namespace: str,
        marker_id: int,
        frame_id: str,
        stamp,
        color: Tuple[float, float, float, float],
        size: float,
        lifetime_sec: float,
    ) -> int:
        for x, y in points:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.08
            marker.pose.orientation.w = 1.0
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            TrackDriverNode._set_marker_lifetime(marker, lifetime_sec)
            markers.markers.append(marker)
            marker_id += 1
        return marker_id

    @staticmethod
    def _append_line_marker(
        markers: MarkerArray,
        points: Sequence[Point],
        namespace: str,
        marker_id: int,
        frame_id: str,
        stamp,
        color: Tuple[float, float, float, float],
        width: float,
        lifetime_sec: float,
    ) -> int:
        if len(points) < 2:
            return marker_id

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        TrackDriverNode._set_marker_lifetime(marker, lifetime_sec)

        for x, y in points:
            point = RosPoint()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.04
            marker.points.append(point)

        markers.markers.append(marker)
        return marker_id + 1

    @staticmethod
    def _append_object_marker(
        markers: MarkerArray,
        point: Point,
        namespace: str,
        marker_id: int,
        frame_id: str,
        stamp,
        color: Tuple[float, float, float, float],
        scale: Tuple[float, float, float],
        marker_type: int,
        lifetime_sec: float,
    ) -> int:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = max(float(scale[2]) * 0.5, 0.05)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        TrackDriverNode._set_marker_lifetime(marker, lifetime_sec)
        markers.markers.append(marker)
        return marker_id + 1

    @staticmethod
    def _append_text_marker(
        markers: MarkerArray,
        point: Point,
        namespace: str,
        marker_id: int,
        frame_id: str,
        stamp,
        text: str,
        color: Tuple[float, float, float, float],
        lifetime_sec: float,
    ) -> int:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = 1.35
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.28
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.text = text
        TrackDriverNode._set_marker_lifetime(marker, lifetime_sec)
        markers.markers.append(marker)
        return marker_id + 1

    @staticmethod
    def _append_arrow_marker(
        markers: MarkerArray,
        start: Point,
        end: Point,
        namespace: str,
        marker_id: int,
        frame_id: str,
        stamp,
        color: Tuple[float, float, float, float],
        lifetime_sec: float,
    ) -> int:
        if math.hypot(end[0] - start[0], end[1] - start[1]) < 0.05:
            return marker_id

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        p0 = RosPoint()
        p0.x = float(start[0])
        p0.y = float(start[1])
        p0.z = 0.30
        p1 = RosPoint()
        p1.x = float(end[0])
        p1.y = float(end[1])
        p1.z = 0.30
        marker.points.append(p0)
        marker.points.append(p1)
        TrackDriverNode._set_marker_lifetime(marker, lifetime_sec)
        markers.markers.append(marker)
        return marker_id + 1

    @staticmethod
    def _vehicle_marker_color(class_id: int, alpha: float = 1.0) -> Tuple[float, float, float, float]:
        if int(class_id) == 0:
            return (0.10, 0.45, 1.00, alpha)
        if int(class_id) == 2:
            return (0.70, 0.25, 1.00, alpha)
        return (1.00, 1.00, 1.00, alpha)

    @staticmethod
    def _set_marker_lifetime(marker: Marker, lifetime_sec: float):
        sec = int(lifetime_sec)
        marker.lifetime.sec = sec
        marker.lifetime.nanosec = int((lifetime_sec - sec) * 1_000_000_000)

    def _log_status(
        self,
        mode: str,
        speed: float,
        steer: float,
        cone_count: int,
        best: Optional[PathCandidate],
        raw_cone_count: Optional[int] = None,
        nearest_obstacle: Optional[float] = None,
    ):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns // 1_000_000_000 == getattr(self, '_last_log_sec', -1):
            return
        self._last_log_sec = now_ns // 1_000_000_000

        if best is None:
            raw_text = f' raw_cones={raw_cone_count}' if raw_cone_count is not None else ''
            nearest_text = self._format_nearest_obstacle(nearest_obstacle)
            speed = self._apply_school_zone_speed_limit(speed)
            zone_text = self._school_zone_log_text()
            vehicle_text = self._vehicle_log_text()
            self.get_logger().info(
                f'mode={mode}{raw_text} track_cones={cone_count} speed={speed:.1f} '
                f'steer={steer:.1f} nearest={nearest_text}{vehicle_text}{zone_text} no_safe_path'
            )
            return

        clearance = best.min_clearance if math.isfinite(best.min_clearance) else 99.0
        raw_text = f'raw_cones={raw_cone_count} ' if raw_cone_count is not None else ''
        nearest_text = self._format_nearest_obstacle(nearest_obstacle)
        speed = self._apply_school_zone_speed_limit(speed)
        zone_text = self._school_zone_log_text()
        vehicle_text = self._vehicle_log_text()
        self.get_logger().info(
            f'mode={mode} {raw_text}track_cones={cone_count} offset={best.offset:+.2f} '
            f'clear={clearance:.2f} nearest={nearest_text}{vehicle_text}{zone_text} '
            f'speed={speed:.1f} steer={steer:.1f}'
        )

    @staticmethod
    def _format_nearest_obstacle(distance: Optional[float]) -> str:
        if distance is None:
            return 'none'
        return f'{distance:.2f}m'

    def _school_zone_log_text(self) -> str:
        if not self.school_zone_active and not self.school_zone_candidate_active:
            return ''
        state = 'active' if self.school_zone_active else 'candidate'
        return (
            f' school_zone={state}({self.school_zone_yellow_left_ratio:.3f},'
            f'{self.school_zone_yellow_right_ratio:.3f},'
            f'pair={self.school_zone_yellow_pair_row_ratio:.3f},'
            f'bottom={self.school_zone_yellow_bottom_pair_row_ratio:.3f},'
            f'sep={self.school_zone_yellow_separation_ratio:.2f})'
        )

    def _vehicle_log_text(self) -> str:
        if not self._vehicle_processing_enabled():
            return ''

        yolo_count = len(self.cached_yolo_vehicle_detections)
        raw_count = int(getattr(self, 'cached_yolo_vehicle_raw_count', 0))
        track_count = len(self.vehicle_tracks)
        now = time.monotonic()
        yolo_timeout = max(float(self.get_parameter('vehicle_yolo_required_timeout_sec').value), 0.02)
        confirmed_tracks = [
            track for track in self.vehicle_tracks.values()
            if now - track.yolo_last_seen_sec <= yolo_timeout
        ]
        confirmed_count = len(confirmed_tracks)
        if yolo_count == 0 and track_count == 0:
            return f' veh=raw{raw_count}/yolo0/track0/valid0'

        lead = None
        active_tracks = [
            track for track in confirmed_tracks
            if now - track.last_seen_sec <= max(float(self.get_parameter('vehicle_track_timeout_sec').value), 0.05)
        ]
        if active_tracks:
            lead = min(active_tracks, key=lambda track: track.x)
        if lead is None:
            return f' veh=raw{raw_count}/yolo{yolo_count}/track{track_count}/valid{confirmed_count}'
        fast_track = (
            self.vehicle_tracks.get(self.vehicle_fast_follow_track_id)
            if self.vehicle_fast_follow_track_id is not None else None
        )
        overtake_track = (
            self.vehicle_tracks.get(self.vehicle_overtake_track_id)
            if self.vehicle_overtake_track_id is not None else None
        )
        active_text = ''
        if fast_track is not None:
            active_text += (
                f' follow={self._vehicle_class_name(fast_track.class_id)}'
                f'({fast_track.x:.2f},{fast_track.y:.2f})'
            )
        if overtake_track is not None:
            active_text += (
                f' ot={self._vehicle_class_name(overtake_track.class_id)}'
                f'({overtake_track.x:.2f},{overtake_track.y:.2f})'
            )
        return (
            f' veh=raw{raw_count}/yolo{yolo_count}/track{track_count}/valid{confirmed_count}'
            f' lead={self._vehicle_class_name(lead.class_id)}'
            f'({lead.x:.2f},{lead.y:.2f},vx={lead.vx:.2f})'
            f'{active_text}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackDriverNode()

    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.drive(angle=0.0, speed=0.0)
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
