from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_topic = LaunchConfiguration('camera_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    motor_topic = LaunchConfiguration('motor_topic')
    model_path = LaunchConfiguration('model_path')
    speed = LaunchConfiguration('speed')
    max_steer_deg = LaunchConfiguration('max_steer_deg')
    ai_enable_topic = LaunchConfiguration('ai_enable_topic')
    ai_speed_limit_topic = LaunchConfiguration('ai_speed_limit_topic')
    hybrid_trigger_topic = LaunchConfiguration('hybrid_trigger_topic')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    publish_light_debug_image = LaunchConfiguration('publish_light_debug_image')
    school_zone_speed = LaunchConfiguration('school_zone_speed')
    school_zone_left_edge_max_ratio = LaunchConfiguration('school_zone_left_edge_max_ratio')
    school_zone_right_edge_min_ratio = LaunchConfiguration('school_zone_right_edge_min_ratio')
    school_zone_yellow_ratio_threshold = LaunchConfiguration('school_zone_yellow_ratio_threshold')
    school_zone_yellow_row_ratio_threshold = LaunchConfiguration('school_zone_yellow_row_ratio_threshold')
    school_zone_yellow_pair_row_ratio_threshold = LaunchConfiguration(
        'school_zone_yellow_pair_row_ratio_threshold')
    school_zone_yellow_bottom_pair_row_ratio_threshold = LaunchConfiguration(
        'school_zone_yellow_bottom_pair_row_ratio_threshold')
    school_zone_yellow_min_pair_rows = LaunchConfiguration('school_zone_yellow_min_pair_rows')
    school_zone_yellow_min_separation_ratio = LaunchConfiguration(
        'school_zone_yellow_min_separation_ratio')
    school_zone_yellow_row_max_width_ratio = LaunchConfiguration('school_zone_yellow_row_max_width_ratio')
    school_zone_yellow_min_pixels = LaunchConfiguration('school_zone_yellow_min_pixels')
    school_zone_preslow_enabled = LaunchConfiguration('school_zone_preslow_enabled')
    school_zone_preslow_ratio = LaunchConfiguration('school_zone_preslow_ratio')
    school_zone_confirm_frames = LaunchConfiguration('school_zone_confirm_frames')
    school_zone_lost_frames = LaunchConfiguration('school_zone_lost_frames')
    school_zone_hold_sec = LaunchConfiguration('school_zone_hold_sec')
    school_zone_mask_vehicle_boxes = LaunchConfiguration('school_zone_mask_vehicle_boxes')
    school_zone_suppress_when_vehicle_visible = LaunchConfiguration('school_zone_suppress_when_vehicle_visible')
    school_zone_follow_yellow_centerline = LaunchConfiguration('school_zone_follow_yellow_centerline')
    school_zone_takeover_enabled = LaunchConfiguration('school_zone_takeover_enabled')
    school_zone_center_left_ratio = LaunchConfiguration('school_zone_center_left_ratio')
    school_zone_center_right_ratio = LaunchConfiguration('school_zone_center_right_ratio')
    school_zone_center_min_pixels = LaunchConfiguration('school_zone_center_min_pixels')
    school_zone_center_memory_sec = LaunchConfiguration('school_zone_center_memory_sec')
    school_zone_center_memory_weight = LaunchConfiguration('school_zone_center_memory_weight')
    school_zone_edge_center_weight = LaunchConfiguration('school_zone_edge_center_weight')
    school_zone_yellow_path_edge_guard_enabled = LaunchConfiguration(
        'school_zone_yellow_path_edge_guard_enabled')
    school_zone_yellow_path_edge_limit = LaunchConfiguration('school_zone_yellow_path_edge_limit')
    school_zone_max_steer_deg = LaunchConfiguration('school_zone_max_steer_deg')
    school_zone_steer_smoothing = LaunchConfiguration('school_zone_steer_smoothing')
    school_zone_lookahead_scale = LaunchConfiguration('school_zone_lookahead_scale')
    stop_on_red_light = LaunchConfiguration('stop_on_red_light')
    stop_on_person = LaunchConfiguration('stop_on_person')
    stop_on_vehicle = LaunchConfiguration('stop_on_vehicle')
    vehicle_overtake_enabled = LaunchConfiguration('vehicle_overtake_enabled')
    vehicle_follow_enabled = LaunchConfiguration('vehicle_follow_enabled')
    vehicle_overtake_speed = LaunchConfiguration('vehicle_overtake_speed')
    vehicle_overtake_max_speed = LaunchConfiguration('vehicle_overtake_max_speed')
    vehicle_overtake_relative_speed_gain = LaunchConfiguration('vehicle_overtake_relative_speed_gain')
    vehicle_overtake_prediction_sec = LaunchConfiguration('vehicle_overtake_prediction_sec')
    vehicle_overtake_speed_hold_gain = LaunchConfiguration('vehicle_overtake_speed_hold_gain')
    vehicle_overtake_speed_pass_gain = LaunchConfiguration('vehicle_overtake_speed_pass_gain')
    vehicle_overtake_offset = LaunchConfiguration('vehicle_overtake_offset')
    vehicle_overtake_shift_length = LaunchConfiguration('vehicle_overtake_shift_length')
    vehicle_overtake_return_to_slow_lane_enabled = LaunchConfiguration(
        'vehicle_overtake_return_to_slow_lane_enabled')
    vehicle_overtake_steer_gain = LaunchConfiguration('vehicle_overtake_steer_gain')
    vehicle_overtake_min_steer_deg = LaunchConfiguration('vehicle_overtake_min_steer_deg')
    vehicle_overtake_lookahead_scale = LaunchConfiguration('vehicle_overtake_lookahead_scale')
    vehicle_overtake_steer_smoothing = LaunchConfiguration('vehicle_overtake_steer_smoothing')
    vehicle_overtake_trigger_distance = LaunchConfiguration('vehicle_overtake_trigger_distance')
    vehicle_overtake_confirm_sec = LaunchConfiguration('vehicle_overtake_confirm_sec')
    vehicle_overtake_min_seen = LaunchConfiguration('vehicle_overtake_min_seen')
    vehicle_overtake_min_time_sec = LaunchConfiguration('vehicle_overtake_min_time_sec')
    vehicle_overtake_safety_distance = LaunchConfiguration('vehicle_overtake_safety_distance')
    vehicle_slow_relative_speed_threshold = LaunchConfiguration('vehicle_slow_relative_speed_threshold')
    vehicle_lidar_fallback_enabled = LaunchConfiguration('vehicle_lidar_fallback_enabled')
    vehicle_lidar_fallback_min_points = LaunchConfiguration('vehicle_lidar_fallback_min_points')
    vehicle_lidar_fallback_min_size = LaunchConfiguration('vehicle_lidar_fallback_min_size')
    vehicle_lidar_fallback_skip_on_person = LaunchConfiguration('vehicle_lidar_fallback_skip_on_person')
    vehicle_lidar_fallback_require_yolo_seed = LaunchConfiguration('vehicle_lidar_fallback_require_yolo_seed')
    vehicle_camera_fallback_enabled = LaunchConfiguration('vehicle_camera_fallback_enabled')
    vehicle_fusion_min_x = LaunchConfiguration('vehicle_fusion_min_x')
    vehicle_fusion_max_distance = LaunchConfiguration('vehicle_fusion_max_distance')
    vehicle_camera_min_distance = LaunchConfiguration('vehicle_camera_min_distance')
    vehicle_yolo_required_timeout_sec = LaunchConfiguration('vehicle_yolo_required_timeout_sec')
    yolo_vehicle_min_box_height_ratio = LaunchConfiguration('yolo_vehicle_min_box_height_ratio')
    yolo_vehicle_min_box_width_ratio = LaunchConfiguration('yolo_vehicle_min_box_width_ratio')
    yolo_vehicle_min_box_bottom_ratio = LaunchConfiguration('yolo_vehicle_min_box_bottom_ratio')
    vehicle_current_lane_lateral_limit = LaunchConfiguration('vehicle_current_lane_lateral_limit')
    vehicle_follow_distance = LaunchConfiguration('vehicle_follow_distance')
    vehicle_follow_min_distance = LaunchConfiguration('vehicle_follow_min_distance')
    vehicle_follow_stop_distance = LaunchConfiguration('vehicle_follow_stop_distance')
    vehicle_follow_close_distance = LaunchConfiguration('vehicle_follow_close_distance')
    vehicle_follow_close_speed = LaunchConfiguration('vehicle_follow_close_speed')
    vehicle_follow_max_speed = LaunchConfiguration('vehicle_follow_max_speed')
    vehicle_follow_closing_gain = LaunchConfiguration('vehicle_follow_closing_gain')
    vehicle_follow_speed_smoothing = LaunchConfiguration('vehicle_follow_speed_smoothing')
    vehicle_follow_target_switch_margin = LaunchConfiguration('vehicle_follow_target_switch_margin')
    vehicle_follow_lateral_gain = LaunchConfiguration('vehicle_follow_lateral_gain')
    vehicle_follow_lateral_smoothing = LaunchConfiguration('vehicle_follow_lateral_smoothing')
    vehicle_follow_lateral_max_step = LaunchConfiguration('vehicle_follow_lateral_max_step')
    vehicle_follow_lateral_deadband = LaunchConfiguration('vehicle_follow_lateral_deadband')
    vehicle_fast_follow_enabled = LaunchConfiguration('vehicle_fast_follow_enabled')
    vehicle_fast_follow_min_sec = LaunchConfiguration('vehicle_fast_follow_min_sec')
    vehicle_follow_steer_gain = LaunchConfiguration('vehicle_follow_steer_gain')
    vehicle_follow_min_steer_deg = LaunchConfiguration('vehicle_follow_min_steer_deg')
    vehicle_follow_max_steer_deg = LaunchConfiguration('vehicle_follow_max_steer_deg')
    vehicle_fast_slow_speed_gap = LaunchConfiguration('vehicle_fast_slow_speed_gap')
    vehicle_lane_edge_follow_enabled = LaunchConfiguration('vehicle_lane_edge_follow_enabled')
    vehicle_outer_wheel_lateral_offset = LaunchConfiguration('vehicle_outer_wheel_lateral_offset')
    vehicle_lane_edge_margin = LaunchConfiguration('vehicle_lane_edge_margin')
    lane_edge_steer_guard_enabled = LaunchConfiguration('lane_edge_steer_guard_enabled')
    lane_edge_steer_guard_start_ratio = LaunchConfiguration('lane_edge_steer_guard_start_ratio')
    lane_edge_steer_guard_max_outward_deg = LaunchConfiguration('lane_edge_steer_guard_max_outward_deg')
    lane_edge_steer_guard_min_outward_deg = LaunchConfiguration('lane_edge_steer_guard_min_outward_deg')
    lane_guard_memory_sec = LaunchConfiguration('lane_guard_memory_sec')
    person_camera_enabled = LaunchConfiguration('person_camera_enabled')
    person_lidar_fallback_enabled = LaunchConfiguration('person_lidar_fallback_enabled')
    person_stop_distance = LaunchConfiguration('person_stop_distance')
    person_lateral_limit = LaunchConfiguration('person_lateral_limit')
    person_fusion_enabled = LaunchConfiguration('person_fusion_enabled')
    person_fusion_camera_fov_deg = LaunchConfiguration('person_fusion_camera_fov_deg')
    person_fusion_angle_margin_deg = LaunchConfiguration('person_fusion_angle_margin_deg')
    person_fusion_max_distance = LaunchConfiguration('person_fusion_max_distance')
    person_fusion_lateral_limit = LaunchConfiguration('person_fusion_lateral_limit')
    person_fusion_lidar_min_points = LaunchConfiguration('person_fusion_lidar_min_points')
    person_yolo_camera_fallback_enabled = LaunchConfiguration('person_yolo_camera_fallback_enabled')
    person_dynamic_enabled = LaunchConfiguration('person_dynamic_enabled')
    person_dynamic_prediction_sec = LaunchConfiguration('person_dynamic_prediction_sec')
    person_dynamic_image_center_deadband = LaunchConfiguration('person_dynamic_image_center_deadband')
    person_dynamic_image_velocity_deadband = LaunchConfiguration('person_dynamic_image_velocity_deadband')
    person_dynamic_crossing_speed = LaunchConfiguration('person_dynamic_crossing_speed')
    person_dynamic_crossing_lateral_limit = LaunchConfiguration('person_dynamic_crossing_lateral_limit')
    person_dynamic_crossing_x_limit = LaunchConfiguration('person_dynamic_crossing_x_limit')
    person_reverse_enabled = LaunchConfiguration('person_reverse_enabled')
    person_reverse_distance = LaunchConfiguration('person_reverse_distance')
    person_reverse_release_distance = LaunchConfiguration('person_reverse_release_distance')
    person_reverse_speed = LaunchConfiguration('person_reverse_speed')
    person_reverse_max_sec = LaunchConfiguration('person_reverse_max_sec')
    person_reverse_steer_deg = LaunchConfiguration('person_reverse_steer_deg')
    person_avoidance_enabled = LaunchConfiguration('person_avoidance_enabled')
    person_wait_release_left_y = LaunchConfiguration('person_wait_release_left_y')
    person_wait_release_image_left_ratio = LaunchConfiguration('person_wait_release_image_left_ratio')
    person_wait_release_confirm_frames = LaunchConfiguration('person_wait_release_confirm_frames')
    person_wait_lost_release_sec = LaunchConfiguration('person_wait_lost_release_sec')
    person_avoidance_offset = LaunchConfiguration('person_avoidance_offset')
    person_avoidance_two_lane_offset = LaunchConfiguration('person_avoidance_two_lane_offset')
    person_avoidance_stop_close_x = LaunchConfiguration('person_avoidance_stop_close_x')
    person_avoidance_plan_length = LaunchConfiguration('person_avoidance_plan_length')
    person_avoidance_prefer_right = LaunchConfiguration('person_avoidance_prefer_right')
    person_avoidance_obstacle_x = LaunchConfiguration('person_avoidance_obstacle_x')
    person_avoidance_obstacle_half_x = LaunchConfiguration('person_avoidance_obstacle_half_x')
    person_avoidance_shift_start = LaunchConfiguration('person_avoidance_shift_start')
    person_avoidance_shift_length = LaunchConfiguration('person_avoidance_shift_length')
    person_avoidance_hold_length = LaunchConfiguration('person_avoidance_hold_length')
    person_avoidance_return_length = LaunchConfiguration('person_avoidance_return_length')
    person_avoidance_allow_return = LaunchConfiguration('person_avoidance_allow_return')
    person_avoidance_speed = LaunchConfiguration('person_avoidance_speed')
    person_avoidance_boost_speed = LaunchConfiguration('person_avoidance_boost_speed')
    person_avoidance_boost_sec = LaunchConfiguration('person_avoidance_boost_sec')
    person_avoidance_close_stop_distance = LaunchConfiguration('person_avoidance_close_stop_distance')
    person_avoidance_hold_sec = LaunchConfiguration('person_avoidance_hold_sec')
    person_avoidance_steer_gain = LaunchConfiguration('person_avoidance_steer_gain')
    person_avoidance_max_steer_deg = LaunchConfiguration('person_avoidance_max_steer_deg')
    person_avoidance_min_steer_deg = LaunchConfiguration('person_avoidance_min_steer_deg')
    person_avoidance_steer_smoothing = LaunchConfiguration('person_avoidance_steer_smoothing')
    yolo_safety_enabled = LaunchConfiguration('yolo_safety_enabled')
    yolo_person_model_path = LaunchConfiguration('yolo_person_model_path')
    yolo_light_model_path = LaunchConfiguration('yolo_light_model_path')
    yolo_person_conf_threshold = LaunchConfiguration('yolo_person_conf_threshold')
    yolo_safety_period_sec = LaunchConfiguration('yolo_safety_period_sec')
    yolo_red_light_period_sec = LaunchConfiguration('yolo_red_light_period_sec')
    yolo_person_min_box_height_ratio = LaunchConfiguration('yolo_person_min_box_height_ratio')
    yolo_person_min_box_bottom_ratio = LaunchConfiguration('yolo_person_min_box_bottom_ratio')
    yolo_light_conf_threshold = LaunchConfiguration('yolo_light_conf_threshold')
    yolo_light_min_box_height_ratio = LaunchConfiguration('yolo_light_min_box_height_ratio')
    yolo_light_max_box_height_ratio = LaunchConfiguration('yolo_light_max_box_height_ratio')
    yolo_light_min_box_area_ratio = LaunchConfiguration('yolo_light_min_box_area_ratio')
    yolo_light_max_box_area_ratio = LaunchConfiguration('yolo_light_max_box_area_ratio')
    yolo_light_max_box_bottom_ratio = LaunchConfiguration('yolo_light_max_box_bottom_ratio')
    red_light_confirm_frames = LaunchConfiguration('red_light_confirm_frames')
    red_light_min_ratio = LaunchConfiguration('red_light_min_ratio')
    red_light_min_dominance = LaunchConfiguration('red_light_min_dominance')
    red_light_go_release_enabled = LaunchConfiguration('red_light_go_release_enabled')
    startup_light_check_enabled = LaunchConfiguration('startup_light_check_enabled')
    startup_light_check_timeout_sec = LaunchConfiguration('startup_light_check_timeout_sec')
    safety_stop_hold_sec = LaunchConfiguration('safety_stop_hold_sec')

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_raw/front'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('motor_topic', default_value='xycar_motor'),
        DeclareLaunchArgument('model_path', default_value='~/cone_il_model_merged_new2/cone_bc_scripted.pt'),
        DeclareLaunchArgument('speed', default_value='10.0'),
        DeclareLaunchArgument('max_steer_deg', default_value='100.0'),
        DeclareLaunchArgument('ai_enable_topic', default_value='/cone_ai/enable'),
        DeclareLaunchArgument('ai_speed_limit_topic', default_value='/cone_ai/speed_limit'),
        DeclareLaunchArgument('hybrid_trigger_topic', default_value='/track_drive/hybrid_trigger'),
        DeclareLaunchArgument('control_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('publish_light_debug_image', default_value='true'),
        DeclareLaunchArgument('school_zone_speed', default_value='6.0'),
        DeclareLaunchArgument('school_zone_left_edge_max_ratio', default_value='0.36'),
        DeclareLaunchArgument('school_zone_right_edge_min_ratio', default_value='0.64'),
        DeclareLaunchArgument('school_zone_yellow_ratio_threshold', default_value='0.006'),
        DeclareLaunchArgument('school_zone_yellow_row_ratio_threshold', default_value='0.14'),
        DeclareLaunchArgument('school_zone_yellow_pair_row_ratio_threshold', default_value='0.12'),
        DeclareLaunchArgument('school_zone_yellow_bottom_pair_row_ratio_threshold', default_value='0.10'),
        DeclareLaunchArgument('school_zone_yellow_min_pair_rows', default_value='6'),
        DeclareLaunchArgument('school_zone_yellow_min_separation_ratio', default_value='0.42'),
        DeclareLaunchArgument('school_zone_yellow_row_max_width_ratio', default_value='0.10'),
        DeclareLaunchArgument('school_zone_yellow_min_pixels', default_value='120'),
        DeclareLaunchArgument('school_zone_preslow_enabled', default_value='true'),
        DeclareLaunchArgument('school_zone_preslow_ratio', default_value='0.60'),
        DeclareLaunchArgument('school_zone_confirm_frames', default_value='2'),
        DeclareLaunchArgument('school_zone_lost_frames', default_value='3'),
        DeclareLaunchArgument('school_zone_hold_sec', default_value='1.0'),
        DeclareLaunchArgument('school_zone_mask_vehicle_boxes', default_value='true'),
        DeclareLaunchArgument('school_zone_suppress_when_vehicle_visible', default_value='true'),
        DeclareLaunchArgument('school_zone_follow_yellow_centerline', default_value='true'),
        DeclareLaunchArgument('school_zone_takeover_enabled', default_value='false'),
        DeclareLaunchArgument('school_zone_center_left_ratio', default_value='0.30'),
        DeclareLaunchArgument('school_zone_center_right_ratio', default_value='0.70'),
        DeclareLaunchArgument('school_zone_center_min_pixels', default_value='24'),
        DeclareLaunchArgument('school_zone_center_memory_sec', default_value='1.20'),
        DeclareLaunchArgument('school_zone_center_memory_weight', default_value='0.45'),
        DeclareLaunchArgument('school_zone_edge_center_weight', default_value='0.20'),
        DeclareLaunchArgument('school_zone_yellow_path_edge_guard_enabled', default_value='true'),
        DeclareLaunchArgument('school_zone_yellow_path_edge_limit', default_value='0.35'),
        DeclareLaunchArgument('school_zone_max_steer_deg', default_value='70.0'),
        DeclareLaunchArgument('school_zone_steer_smoothing', default_value='0.05'),
        DeclareLaunchArgument('school_zone_lookahead_scale', default_value='0.72'),
        DeclareLaunchArgument('stop_on_red_light', default_value='true'),
        DeclareLaunchArgument('stop_on_person', default_value='false'),
        DeclareLaunchArgument('stop_on_vehicle', default_value='false'),
        DeclareLaunchArgument('vehicle_overtake_enabled', default_value='false'),
        DeclareLaunchArgument('vehicle_follow_enabled', default_value='false'),
        DeclareLaunchArgument('vehicle_overtake_speed', default_value='12.0'),
        DeclareLaunchArgument('vehicle_overtake_max_speed', default_value='12.0'),
        DeclareLaunchArgument('vehicle_overtake_relative_speed_gain', default_value='0.0'),
        DeclareLaunchArgument('vehicle_overtake_prediction_sec', default_value='0.90'),
        DeclareLaunchArgument('vehicle_overtake_speed_hold_gain', default_value='1.20'),
        DeclareLaunchArgument('vehicle_overtake_speed_pass_gain', default_value='0.70'),
        DeclareLaunchArgument('vehicle_overtake_offset', default_value='0.30'),
        DeclareLaunchArgument('vehicle_overtake_shift_length', default_value='2.00'),
        DeclareLaunchArgument('vehicle_overtake_return_to_slow_lane_enabled', default_value='true'),
        DeclareLaunchArgument('vehicle_overtake_steer_gain', default_value='1.35'),
        DeclareLaunchArgument('vehicle_overtake_min_steer_deg', default_value='10.0'),
        DeclareLaunchArgument('vehicle_overtake_lookahead_scale', default_value='0.70'),
        DeclareLaunchArgument('vehicle_overtake_steer_smoothing', default_value='0.05'),
        DeclareLaunchArgument('vehicle_overtake_trigger_distance', default_value='5.50'),
        DeclareLaunchArgument('vehicle_overtake_confirm_sec', default_value='0.00'),
        DeclareLaunchArgument('vehicle_overtake_min_seen', default_value='1'),
        DeclareLaunchArgument('vehicle_overtake_min_time_sec', default_value='1.80'),
        DeclareLaunchArgument('vehicle_overtake_safety_distance', default_value='2.20'),
        DeclareLaunchArgument('vehicle_slow_relative_speed_threshold', default_value='0.35'),
        DeclareLaunchArgument('vehicle_lidar_fallback_enabled', default_value='false'),
        DeclareLaunchArgument('vehicle_lidar_fallback_min_points', default_value='2'),
        DeclareLaunchArgument('vehicle_lidar_fallback_min_size', default_value='0.12'),
        DeclareLaunchArgument('vehicle_lidar_fallback_skip_on_person', default_value='true'),
        DeclareLaunchArgument('vehicle_lidar_fallback_require_yolo_seed', default_value='true'),
        DeclareLaunchArgument('vehicle_camera_fallback_enabled', default_value='false'),
        DeclareLaunchArgument('vehicle_fusion_min_x', default_value='0.65'),
        DeclareLaunchArgument('vehicle_fusion_max_distance', default_value='12.0'),
        DeclareLaunchArgument('vehicle_camera_min_distance', default_value='1.20'),
        DeclareLaunchArgument('vehicle_yolo_required_timeout_sec', default_value='0.90'),
        DeclareLaunchArgument('yolo_vehicle_min_box_height_ratio', default_value='0.025'),
        DeclareLaunchArgument('yolo_vehicle_min_box_width_ratio', default_value='0.025'),
        DeclareLaunchArgument('yolo_vehicle_min_box_bottom_ratio', default_value='0.18'),
        DeclareLaunchArgument('vehicle_current_lane_lateral_limit', default_value='1.00'),
        DeclareLaunchArgument('vehicle_follow_distance', default_value='2.80'),
        DeclareLaunchArgument('vehicle_follow_min_distance', default_value='0.60'),
        DeclareLaunchArgument('vehicle_follow_stop_distance', default_value='0.90'),
        DeclareLaunchArgument('vehicle_follow_close_distance', default_value='1.80'),
        DeclareLaunchArgument('vehicle_follow_close_speed', default_value='3.5'),
        DeclareLaunchArgument('vehicle_follow_max_speed', default_value='12.0'),
        DeclareLaunchArgument('vehicle_follow_closing_gain', default_value='3.00'),
        DeclareLaunchArgument('vehicle_follow_speed_smoothing', default_value='0.25'),
        DeclareLaunchArgument('vehicle_follow_target_switch_margin', default_value='0.80'),
        DeclareLaunchArgument('vehicle_follow_lateral_gain', default_value='0.70'),
        DeclareLaunchArgument('vehicle_follow_lateral_smoothing', default_value='0.65'),
        DeclareLaunchArgument('vehicle_follow_lateral_max_step', default_value='0.08'),
        DeclareLaunchArgument('vehicle_follow_lateral_deadband', default_value='0.03'),
        DeclareLaunchArgument('vehicle_fast_follow_enabled', default_value='false'),
        DeclareLaunchArgument('vehicle_fast_follow_min_sec', default_value='2.50'),
        DeclareLaunchArgument('vehicle_follow_steer_gain', default_value='1.15'),
        DeclareLaunchArgument('vehicle_follow_min_steer_deg', default_value='0.0'),
        DeclareLaunchArgument('vehicle_follow_max_steer_deg', default_value='60.0'),
        DeclareLaunchArgument('vehicle_fast_slow_speed_gap', default_value='0.10'),
        DeclareLaunchArgument('vehicle_lane_edge_follow_enabled', default_value='true'),
        DeclareLaunchArgument('vehicle_outer_wheel_lateral_offset', default_value='0.22'),
        DeclareLaunchArgument('vehicle_lane_edge_margin', default_value='0.12'),
        DeclareLaunchArgument('lane_edge_steer_guard_enabled', default_value='true'),
        DeclareLaunchArgument('lane_edge_steer_guard_start_ratio', default_value='0.75'),
        DeclareLaunchArgument('lane_edge_steer_guard_max_outward_deg', default_value='18.0'),
        DeclareLaunchArgument('lane_edge_steer_guard_min_outward_deg', default_value='6.0'),
        DeclareLaunchArgument('lane_guard_memory_sec', default_value='2.00'),
        DeclareLaunchArgument('person_camera_enabled', default_value='false'),
        DeclareLaunchArgument('person_lidar_fallback_enabled', default_value='false'),
        DeclareLaunchArgument('person_stop_distance', default_value='1.15'),
        DeclareLaunchArgument('person_lateral_limit', default_value='0.38'),
        DeclareLaunchArgument('person_fusion_enabled', default_value='true'),
        DeclareLaunchArgument('person_fusion_camera_fov_deg', default_value='62.0'),
        DeclareLaunchArgument('person_fusion_angle_margin_deg', default_value='16.0'),
        DeclareLaunchArgument('person_fusion_max_distance', default_value='6.0'),
        DeclareLaunchArgument('person_fusion_lateral_limit', default_value='1.60'),
        DeclareLaunchArgument('person_fusion_lidar_min_points', default_value='1'),
        DeclareLaunchArgument('person_yolo_camera_fallback_enabled', default_value='true'),
        DeclareLaunchArgument('person_dynamic_enabled', default_value='true'),
        DeclareLaunchArgument('person_dynamic_prediction_sec', default_value='0.80'),
        DeclareLaunchArgument('person_dynamic_image_center_deadband', default_value='0.12'),
        DeclareLaunchArgument('person_dynamic_image_velocity_deadband', default_value='0.20'),
        DeclareLaunchArgument('person_dynamic_crossing_speed', default_value='4.0'),
        DeclareLaunchArgument('person_dynamic_crossing_lateral_limit', default_value='0.55'),
        DeclareLaunchArgument('person_dynamic_crossing_x_limit', default_value='3.00'),
        DeclareLaunchArgument('person_reverse_enabled', default_value='false'),
        DeclareLaunchArgument('person_reverse_distance', default_value='1.05'),
        DeclareLaunchArgument('person_reverse_release_distance', default_value='1.35'),
        DeclareLaunchArgument('person_reverse_speed', default_value='-4.0'),
        DeclareLaunchArgument('person_reverse_max_sec', default_value='1.00'),
        DeclareLaunchArgument('person_reverse_steer_deg', default_value='0.0'),
        DeclareLaunchArgument('person_avoidance_enabled', default_value='false'),
        DeclareLaunchArgument('person_wait_release_left_y', default_value='0.55'),
        DeclareLaunchArgument('person_wait_release_image_left_ratio', default_value='-0.35'),
        DeclareLaunchArgument('person_wait_release_confirm_frames', default_value='2'),
        DeclareLaunchArgument('person_wait_lost_release_sec', default_value='0.50'),
        DeclareLaunchArgument('person_avoidance_offset', default_value='2.80'),
        DeclareLaunchArgument('person_avoidance_two_lane_offset', default_value='5.60'),
        DeclareLaunchArgument('person_avoidance_stop_close_x', default_value='5.50'),
        DeclareLaunchArgument('person_avoidance_plan_length', default_value='30.0'),
        DeclareLaunchArgument('person_avoidance_prefer_right', default_value='false'),
        DeclareLaunchArgument('person_avoidance_obstacle_x', default_value='2.20'),
        DeclareLaunchArgument('person_avoidance_obstacle_half_x', default_value='2.00'),
        DeclareLaunchArgument('person_avoidance_shift_start', default_value='1.00'),
        DeclareLaunchArgument('person_avoidance_shift_length', default_value='15.00'),
        DeclareLaunchArgument('person_avoidance_hold_length', default_value='8.00'),
        DeclareLaunchArgument('person_avoidance_return_length', default_value='20.00'),
        DeclareLaunchArgument('person_avoidance_allow_return', default_value='false'),
        DeclareLaunchArgument('person_avoidance_speed', default_value='10.0'),
        DeclareLaunchArgument('person_avoidance_boost_speed', default_value='14.0'),
        DeclareLaunchArgument('person_avoidance_boost_sec', default_value='0.0'),
        DeclareLaunchArgument('person_avoidance_close_stop_distance', default_value='0.05'),
        DeclareLaunchArgument('person_avoidance_hold_sec', default_value='0.0'),
        DeclareLaunchArgument('person_avoidance_steer_gain', default_value='1.0'),
        DeclareLaunchArgument('person_avoidance_max_steer_deg', default_value='100.0'),
        DeclareLaunchArgument('person_avoidance_min_steer_deg', default_value='0.0'),
        DeclareLaunchArgument('person_avoidance_steer_smoothing', default_value='0.0'),
        DeclareLaunchArgument('yolo_safety_enabled', default_value='true'),
        DeclareLaunchArgument('yolo_person_model_path', default_value='/home/xytron/model/best.onnx'),
        DeclareLaunchArgument('yolo_light_model_path', default_value='/home/xytron/model/light.onnx'),
        DeclareLaunchArgument('yolo_person_conf_threshold', default_value='0.26'),
        DeclareLaunchArgument('yolo_safety_period_sec', default_value='0.02'),
        DeclareLaunchArgument('yolo_red_light_period_sec', default_value='0.10'),
        DeclareLaunchArgument('yolo_person_min_box_height_ratio', default_value='0.035'),
        DeclareLaunchArgument('yolo_person_min_box_bottom_ratio', default_value='0.24'),
        DeclareLaunchArgument('yolo_light_conf_threshold', default_value='0.70'),
        DeclareLaunchArgument('yolo_light_min_box_height_ratio', default_value='0.015'),
        DeclareLaunchArgument('yolo_light_max_box_height_ratio', default_value='0.45'),
        DeclareLaunchArgument('yolo_light_min_box_area_ratio', default_value='0.00008'),
        DeclareLaunchArgument('yolo_light_max_box_area_ratio', default_value='0.080'),
        DeclareLaunchArgument('yolo_light_max_box_bottom_ratio', default_value='0.70'),
        DeclareLaunchArgument('red_light_confirm_frames', default_value='2'),
        DeclareLaunchArgument('red_light_min_ratio', default_value='0.006'),
        DeclareLaunchArgument('red_light_min_dominance', default_value='1.80'),
        DeclareLaunchArgument('red_light_go_release_enabled', default_value='true'),
        DeclareLaunchArgument('startup_light_check_enabled', default_value='false'),
        DeclareLaunchArgument('startup_light_check_timeout_sec', default_value='0.80'),
        DeclareLaunchArgument('safety_stop_hold_sec', default_value='0.35'),

        Node(
            package='track_drive',
            executable='switchable_cone_ai_driver',
            name='cone_ai_direct',
            output='screen',
            parameters=[{
                'image_topic': camera_topic,
                'scan_topic': scan_topic,
                'motor_topic': motor_topic,
                'model_path': model_path,
                'speed': ParameterValue(speed, value_type=float),
                'max_steer_deg': ParameterValue(max_steer_deg, value_type=float),
                'control_rate_hz': ParameterValue(control_rate_hz, value_type=float),
                'enable_topic': ai_enable_topic,
                'speed_limit_topic': ai_speed_limit_topic,
                'start_enabled': True,
                'use_lidar_emergency_stop': False,
                'require_orange_gate': False,
            }],
        ),

        Node(
            package='track_drive',
            executable='track_drive',
            name='driver',
            output='screen',
            parameters=[{
                'camera_topic': camera_topic,
                'scan_topic': scan_topic,
                'motor_topic': motor_topic,
                'control_rate_hz': ParameterValue(control_rate_hz, value_type=float),
                'publish_light_debug_image': ParameterValue(publish_light_debug_image, value_type=bool),
                'school_zone_speed': ParameterValue(school_zone_speed, value_type=float),
                'school_zone_left_edge_max_ratio': ParameterValue(
                    school_zone_left_edge_max_ratio, value_type=float),
                'school_zone_right_edge_min_ratio': ParameterValue(
                    school_zone_right_edge_min_ratio, value_type=float),
                'school_zone_yellow_ratio_threshold': ParameterValue(
                    school_zone_yellow_ratio_threshold, value_type=float),
                'school_zone_yellow_row_ratio_threshold': ParameterValue(
                    school_zone_yellow_row_ratio_threshold, value_type=float),
                'school_zone_yellow_pair_row_ratio_threshold': ParameterValue(
                    school_zone_yellow_pair_row_ratio_threshold, value_type=float),
                'school_zone_yellow_bottom_pair_row_ratio_threshold': ParameterValue(
                    school_zone_yellow_bottom_pair_row_ratio_threshold, value_type=float),
                'school_zone_yellow_min_pair_rows': ParameterValue(
                    school_zone_yellow_min_pair_rows, value_type=int),
                'school_zone_yellow_min_separation_ratio': ParameterValue(
                    school_zone_yellow_min_separation_ratio, value_type=float),
                'school_zone_yellow_row_max_width_ratio': ParameterValue(
                    school_zone_yellow_row_max_width_ratio, value_type=float),
                'school_zone_yellow_min_pixels': ParameterValue(school_zone_yellow_min_pixels, value_type=int),
                'school_zone_preslow_enabled': ParameterValue(school_zone_preslow_enabled, value_type=bool),
                'school_zone_preslow_ratio': ParameterValue(school_zone_preslow_ratio, value_type=float),
                'school_zone_confirm_frames': ParameterValue(school_zone_confirm_frames, value_type=int),
                'school_zone_lost_frames': ParameterValue(school_zone_lost_frames, value_type=int),
                'school_zone_hold_sec': ParameterValue(school_zone_hold_sec, value_type=float),
                'school_zone_mask_vehicle_boxes': ParameterValue(
                    school_zone_mask_vehicle_boxes, value_type=bool),
                'school_zone_suppress_when_vehicle_visible': ParameterValue(
                    school_zone_suppress_when_vehicle_visible, value_type=bool),
                'school_zone_follow_yellow_centerline': ParameterValue(
                    school_zone_follow_yellow_centerline, value_type=bool),
                'school_zone_takeover_enabled': ParameterValue(
                    school_zone_takeover_enabled, value_type=bool),
                'school_zone_center_left_ratio': ParameterValue(
                    school_zone_center_left_ratio, value_type=float),
                'school_zone_center_right_ratio': ParameterValue(
                    school_zone_center_right_ratio, value_type=float),
                'school_zone_center_min_pixels': ParameterValue(school_zone_center_min_pixels, value_type=int),
                'school_zone_center_memory_sec': ParameterValue(
                    school_zone_center_memory_sec, value_type=float),
                'school_zone_center_memory_weight': ParameterValue(
                    school_zone_center_memory_weight, value_type=float),
                'school_zone_edge_center_weight': ParameterValue(
                    school_zone_edge_center_weight, value_type=float),
                'school_zone_yellow_path_edge_guard_enabled': ParameterValue(
                    school_zone_yellow_path_edge_guard_enabled, value_type=bool),
                'school_zone_yellow_path_edge_limit': ParameterValue(
                    school_zone_yellow_path_edge_limit, value_type=float),
                'school_zone_max_steer_deg': ParameterValue(school_zone_max_steer_deg, value_type=float),
                'school_zone_steer_smoothing': ParameterValue(school_zone_steer_smoothing, value_type=float),
                'school_zone_lookahead_scale': ParameterValue(school_zone_lookahead_scale, value_type=float),
                'hybrid_trigger_topic': hybrid_trigger_topic,
                'hybrid_standby_enabled': True,
                'ai_enable_topic': ai_enable_topic,
                'ai_speed_limit_topic': ai_speed_limit_topic,
                'ai_hybrid_enabled': False,
                'ai_passthrough_enabled': False,
                'ai_command_passthrough_enabled': False,
                'hybrid_on_obstacle_enabled': False,
                'stop_on_red_light_enabled': ParameterValue(stop_on_red_light, value_type=bool),
                'stop_on_person_enabled': ParameterValue(stop_on_person, value_type=bool),
                'stop_on_vehicle_enabled': ParameterValue(stop_on_vehicle, value_type=bool),
                'vehicle_overtake_enabled': ParameterValue(vehicle_overtake_enabled, value_type=bool),
                'vehicle_follow_enabled': ParameterValue(vehicle_follow_enabled, value_type=bool),
                'vehicle_overtake_speed': ParameterValue(vehicle_overtake_speed, value_type=float),
                'vehicle_overtake_max_speed': ParameterValue(vehicle_overtake_max_speed, value_type=float),
                'vehicle_overtake_relative_speed_gain': ParameterValue(
                    vehicle_overtake_relative_speed_gain, value_type=float),
                'vehicle_overtake_prediction_sec': ParameterValue(
                    vehicle_overtake_prediction_sec, value_type=float),
                'vehicle_overtake_speed_hold_gain': ParameterValue(
                    vehicle_overtake_speed_hold_gain, value_type=float),
                'vehicle_overtake_speed_pass_gain': ParameterValue(
                    vehicle_overtake_speed_pass_gain, value_type=float),
                'vehicle_overtake_offset': ParameterValue(vehicle_overtake_offset, value_type=float),
                'vehicle_overtake_shift_length': ParameterValue(vehicle_overtake_shift_length, value_type=float),
                'vehicle_overtake_return_to_slow_lane_enabled': ParameterValue(
                    vehicle_overtake_return_to_slow_lane_enabled, value_type=bool),
                'vehicle_overtake_steer_gain': ParameterValue(vehicle_overtake_steer_gain, value_type=float),
                'vehicle_overtake_min_steer_deg': ParameterValue(
                    vehicle_overtake_min_steer_deg, value_type=float),
                'vehicle_overtake_lookahead_scale': ParameterValue(
                    vehicle_overtake_lookahead_scale, value_type=float),
                'vehicle_overtake_steer_smoothing': ParameterValue(
                    vehicle_overtake_steer_smoothing, value_type=float),
                'vehicle_overtake_trigger_distance': ParameterValue(
                    vehicle_overtake_trigger_distance, value_type=float),
                'vehicle_overtake_confirm_sec': ParameterValue(
                    vehicle_overtake_confirm_sec, value_type=float),
                'vehicle_overtake_min_seen': ParameterValue(vehicle_overtake_min_seen, value_type=int),
                'vehicle_overtake_min_time_sec': ParameterValue(
                    vehicle_overtake_min_time_sec, value_type=float),
                'vehicle_overtake_safety_distance': ParameterValue(
                    vehicle_overtake_safety_distance, value_type=float),
                'vehicle_slow_relative_speed_threshold': ParameterValue(
                    vehicle_slow_relative_speed_threshold, value_type=float),
                'vehicle_lidar_fallback_enabled': ParameterValue(
                    vehicle_lidar_fallback_enabled, value_type=bool),
                'vehicle_lidar_fallback_min_points': ParameterValue(
                    vehicle_lidar_fallback_min_points, value_type=int),
                'vehicle_lidar_fallback_min_size': ParameterValue(
                    vehicle_lidar_fallback_min_size, value_type=float),
                'vehicle_lidar_fallback_skip_on_person': ParameterValue(
                    vehicle_lidar_fallback_skip_on_person, value_type=bool),
                'vehicle_lidar_fallback_require_yolo_seed': ParameterValue(
                    vehicle_lidar_fallback_require_yolo_seed, value_type=bool),
                'vehicle_camera_fallback_enabled': ParameterValue(
                    vehicle_camera_fallback_enabled, value_type=bool),
                'vehicle_fusion_min_x': ParameterValue(vehicle_fusion_min_x, value_type=float),
                'vehicle_fusion_max_distance': ParameterValue(vehicle_fusion_max_distance, value_type=float),
                'vehicle_camera_min_distance': ParameterValue(
                    vehicle_camera_min_distance, value_type=float),
                'vehicle_yolo_required_timeout_sec': ParameterValue(
                    vehicle_yolo_required_timeout_sec, value_type=float),
                'yolo_vehicle_min_box_height_ratio': ParameterValue(
                    yolo_vehicle_min_box_height_ratio, value_type=float),
                'yolo_vehicle_min_box_width_ratio': ParameterValue(
                    yolo_vehicle_min_box_width_ratio, value_type=float),
                'yolo_vehicle_min_box_bottom_ratio': ParameterValue(
                    yolo_vehicle_min_box_bottom_ratio, value_type=float),
                'vehicle_current_lane_lateral_limit': ParameterValue(
                    vehicle_current_lane_lateral_limit, value_type=float),
                'vehicle_follow_distance': ParameterValue(vehicle_follow_distance, value_type=float),
                'vehicle_follow_min_distance': ParameterValue(vehicle_follow_min_distance, value_type=float),
                'vehicle_follow_stop_distance': ParameterValue(
                    vehicle_follow_stop_distance, value_type=float),
                'vehicle_follow_close_distance': ParameterValue(
                    vehicle_follow_close_distance, value_type=float),
                'vehicle_follow_close_speed': ParameterValue(vehicle_follow_close_speed, value_type=float),
                'vehicle_follow_max_speed': ParameterValue(vehicle_follow_max_speed, value_type=float),
                'vehicle_follow_closing_gain': ParameterValue(vehicle_follow_closing_gain, value_type=float),
                'vehicle_follow_speed_smoothing': ParameterValue(
                    vehicle_follow_speed_smoothing, value_type=float),
                'vehicle_follow_target_switch_margin': ParameterValue(
                    vehicle_follow_target_switch_margin, value_type=float),
                'vehicle_follow_lateral_gain': ParameterValue(vehicle_follow_lateral_gain, value_type=float),
                'vehicle_follow_lateral_smoothing': ParameterValue(
                    vehicle_follow_lateral_smoothing, value_type=float),
                'vehicle_follow_lateral_max_step': ParameterValue(
                    vehicle_follow_lateral_max_step, value_type=float),
                'vehicle_follow_lateral_deadband': ParameterValue(
                    vehicle_follow_lateral_deadband, value_type=float),
                'vehicle_fast_follow_enabled': ParameterValue(vehicle_fast_follow_enabled, value_type=bool),
                'vehicle_fast_follow_min_sec': ParameterValue(
                    vehicle_fast_follow_min_sec, value_type=float),
                'vehicle_follow_steer_gain': ParameterValue(vehicle_follow_steer_gain, value_type=float),
                'vehicle_follow_min_steer_deg': ParameterValue(
                    vehicle_follow_min_steer_deg, value_type=float),
                'vehicle_follow_max_steer_deg': ParameterValue(
                    vehicle_follow_max_steer_deg, value_type=float),
                'vehicle_fast_slow_speed_gap': ParameterValue(vehicle_fast_slow_speed_gap, value_type=float),
                'vehicle_lane_edge_follow_enabled': ParameterValue(
                    vehicle_lane_edge_follow_enabled, value_type=bool),
                'vehicle_outer_wheel_lateral_offset': ParameterValue(
                    vehicle_outer_wheel_lateral_offset, value_type=float),
                'vehicle_lane_edge_margin': ParameterValue(vehicle_lane_edge_margin, value_type=float),
                'lane_edge_steer_guard_enabled': ParameterValue(
                    lane_edge_steer_guard_enabled, value_type=bool),
                'lane_edge_steer_guard_start_ratio': ParameterValue(
                    lane_edge_steer_guard_start_ratio, value_type=float),
                'lane_edge_steer_guard_max_outward_deg': ParameterValue(
                    lane_edge_steer_guard_max_outward_deg, value_type=float),
                'lane_edge_steer_guard_min_outward_deg': ParameterValue(
                    lane_edge_steer_guard_min_outward_deg, value_type=float),
                'lane_guard_memory_sec': ParameterValue(lane_guard_memory_sec, value_type=float),
                'person_camera_enabled': ParameterValue(person_camera_enabled, value_type=bool),
                'person_lidar_fallback_enabled': ParameterValue(person_lidar_fallback_enabled, value_type=bool),
                'person_stop_distance': ParameterValue(person_stop_distance, value_type=float),
                'person_lateral_limit': ParameterValue(person_lateral_limit, value_type=float),
                'person_fusion_enabled': ParameterValue(person_fusion_enabled, value_type=bool),
                'person_fusion_camera_fov_deg': ParameterValue(person_fusion_camera_fov_deg, value_type=float),
                'person_fusion_angle_margin_deg': ParameterValue(person_fusion_angle_margin_deg, value_type=float),
                'person_fusion_max_distance': ParameterValue(person_fusion_max_distance, value_type=float),
                'person_fusion_lateral_limit': ParameterValue(person_fusion_lateral_limit, value_type=float),
                'person_fusion_lidar_min_points': ParameterValue(person_fusion_lidar_min_points, value_type=int),
                'person_yolo_camera_fallback_enabled': ParameterValue(
                    person_yolo_camera_fallback_enabled, value_type=bool),
                'person_dynamic_enabled': ParameterValue(person_dynamic_enabled, value_type=bool),
                'person_dynamic_prediction_sec': ParameterValue(person_dynamic_prediction_sec, value_type=float),
                'person_dynamic_image_center_deadband': ParameterValue(
                    person_dynamic_image_center_deadband, value_type=float),
                'person_dynamic_image_velocity_deadband': ParameterValue(
                    person_dynamic_image_velocity_deadband, value_type=float),
                'person_dynamic_crossing_speed': ParameterValue(person_dynamic_crossing_speed, value_type=float),
                'person_dynamic_crossing_lateral_limit': ParameterValue(
                    person_dynamic_crossing_lateral_limit, value_type=float),
                'person_dynamic_crossing_x_limit': ParameterValue(person_dynamic_crossing_x_limit, value_type=float),
                'person_reverse_enabled': ParameterValue(person_reverse_enabled, value_type=bool),
                'person_reverse_distance': ParameterValue(person_reverse_distance, value_type=float),
                'person_reverse_release_distance': ParameterValue(
                    person_reverse_release_distance, value_type=float),
                'person_reverse_speed': ParameterValue(person_reverse_speed, value_type=float),
                'person_reverse_max_sec': ParameterValue(person_reverse_max_sec, value_type=float),
                'person_reverse_steer_deg': ParameterValue(person_reverse_steer_deg, value_type=float),
                'person_avoidance_enabled': ParameterValue(person_avoidance_enabled, value_type=bool),
                'person_wait_release_left_y': ParameterValue(person_wait_release_left_y, value_type=float),
                'person_wait_release_image_left_ratio': ParameterValue(
                    person_wait_release_image_left_ratio, value_type=float),
                'person_wait_release_confirm_frames': ParameterValue(
                    person_wait_release_confirm_frames, value_type=int),
                'person_wait_lost_release_sec': ParameterValue(person_wait_lost_release_sec, value_type=float),
                'person_avoidance_offset': ParameterValue(person_avoidance_offset, value_type=float),
                'person_avoidance_two_lane_offset': ParameterValue(
                    person_avoidance_two_lane_offset, value_type=float),
                'person_avoidance_stop_close_x': ParameterValue(person_avoidance_stop_close_x, value_type=float),
                'person_avoidance_plan_length': ParameterValue(person_avoidance_plan_length, value_type=float),
                'person_avoidance_prefer_right': ParameterValue(person_avoidance_prefer_right, value_type=bool),
                'person_avoidance_obstacle_x': ParameterValue(person_avoidance_obstacle_x, value_type=float),
                'person_avoidance_obstacle_half_x': ParameterValue(
                    person_avoidance_obstacle_half_x, value_type=float),
                'person_avoidance_shift_start': ParameterValue(person_avoidance_shift_start, value_type=float),
                'person_avoidance_shift_length': ParameterValue(person_avoidance_shift_length, value_type=float),
                'person_avoidance_hold_length': ParameterValue(person_avoidance_hold_length, value_type=float),
                'person_avoidance_return_length': ParameterValue(person_avoidance_return_length, value_type=float),
                'person_avoidance_allow_return': ParameterValue(person_avoidance_allow_return, value_type=bool),
                'person_avoidance_speed': ParameterValue(person_avoidance_speed, value_type=float),
                'person_avoidance_boost_speed': ParameterValue(person_avoidance_boost_speed, value_type=float),
                'person_avoidance_boost_sec': ParameterValue(person_avoidance_boost_sec, value_type=float),
                'person_avoidance_close_stop_distance': ParameterValue(
                    person_avoidance_close_stop_distance, value_type=float),
                'person_avoidance_hold_sec': ParameterValue(person_avoidance_hold_sec, value_type=float),
                'person_avoidance_steer_gain': ParameterValue(person_avoidance_steer_gain, value_type=float),
                'person_avoidance_max_steer_deg': ParameterValue(
                    person_avoidance_max_steer_deg, value_type=float),
                'person_avoidance_min_steer_deg': ParameterValue(
                    person_avoidance_min_steer_deg, value_type=float),
                'person_avoidance_steer_smoothing': ParameterValue(
                    person_avoidance_steer_smoothing, value_type=float),
                'yolo_safety_enabled': ParameterValue(yolo_safety_enabled, value_type=bool),
                'yolo_person_model_path': yolo_person_model_path,
                'yolo_light_model_path': yolo_light_model_path,
                'yolo_person_conf_threshold': ParameterValue(yolo_person_conf_threshold, value_type=float),
                'yolo_safety_period_sec': ParameterValue(yolo_safety_period_sec, value_type=float),
                'yolo_red_light_period_sec': ParameterValue(yolo_red_light_period_sec, value_type=float),
                'yolo_person_min_box_height_ratio': ParameterValue(
                    yolo_person_min_box_height_ratio, value_type=float),
                'yolo_person_min_box_bottom_ratio': ParameterValue(
                    yolo_person_min_box_bottom_ratio, value_type=float),
                'yolo_light_conf_threshold': ParameterValue(yolo_light_conf_threshold, value_type=float),
                'yolo_light_min_box_height_ratio': ParameterValue(
                    yolo_light_min_box_height_ratio, value_type=float),
                'yolo_light_max_box_height_ratio': ParameterValue(
                    yolo_light_max_box_height_ratio, value_type=float),
                'yolo_light_min_box_area_ratio': ParameterValue(
                    yolo_light_min_box_area_ratio, value_type=float),
                'yolo_light_max_box_area_ratio': ParameterValue(
                    yolo_light_max_box_area_ratio, value_type=float),
                'yolo_light_max_box_bottom_ratio': ParameterValue(yolo_light_max_box_bottom_ratio, value_type=float),
                'red_light_confirm_frames': ParameterValue(red_light_confirm_frames, value_type=int),
                'red_light_min_ratio': ParameterValue(red_light_min_ratio, value_type=float),
                'red_light_min_dominance': ParameterValue(red_light_min_dominance, value_type=float),
                'red_light_go_release_enabled': ParameterValue(
                    red_light_go_release_enabled, value_type=bool),
                'startup_light_check_enabled': ParameterValue(startup_light_check_enabled, value_type=bool),
                'startup_light_check_timeout_sec': ParameterValue(
                    startup_light_check_timeout_sec, value_type=float),
                'safety_stop_hold_sec': ParameterValue(safety_stop_hold_sec, value_type=float),
            }],
        ),
    ])
