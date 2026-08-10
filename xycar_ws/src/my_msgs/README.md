# my_msgs

Xycar 룰베이스 통합 주행에서 사용하는 ROS 2 custom message 패키지입니다.

## 차량 제어 메시지

### `ActuatorCommand`

저수준 액추에이터 명령입니다. 스로틀, 토크, 속도 목표 중 하나를 drive mode로 선택하고 조향, 브레이크, 기어를 함께 전달합니다.

```text
uint8 DRIVE_DISABLED=0
uint8 DRIVE_THROTTLE=1
uint8 DRIVE_TORQUE=2
uint8 DRIVE_SPEED=3
uint8 GEAR_NEUTRAL=0
uint8 GEAR_FORWARD=1
uint8 GEAR_REVERSE=2

std_msgs/Header header
bool enable
float32 steering_target_rad
uint8 drive_mode
float32 throttle_cmd
float32 drive_torque_nm
float32 speed_target_mps
bool brake_engage
uint8 gear
```

### `DriveIntent`

의사결정 또는 상위 제어 계층에서 내려오는 고수준 주행 의도입니다.

```text
std_msgs/Header header
float32 target_speed_mps
float32 target_steering_rad
bool brake_request
bool enable
```

### `VehicleState`

시뮬레이션 플랜트가 publish하는 차량 상태 피드백입니다.

```text
std_msgs/Header header
float32 speed_mps
float32 steering_rad
float32 front_left_steering_rad
float32 front_right_steering_rad
float32 rear_left_wheel_radps
float32 rear_right_wheel_radps
bool brake_engaged
bool estop_active
bool tractive_enabled
bool steering_enabled
uint8 gear
uint8 as_state
```

## 인지 메시지

### `RoadSegment`

차선, 정지선, 횡단보도 관측 polyline입니다.

```text
uint8 TYPE_WHSOL=1
uint8 TYPE_WHDOT=2
uint8 TYPE_YESOL=3
uint8 TYPE_YEDOT=4
uint8 TYPE_STOP_LINE=5
uint8 TYPE_CROSSWALK=6

uint32 detection_id
uint32 track_id
uint8 type
geometry_msgs/Point[] points
float32 confidence
string source
```

### `RoadSegmentArray`

```text
std_msgs/Header header
my_msgs/RoadSegment[] segments
```

### `Centerline`

Ego lane 중심선 추정 결과입니다.

```text
std_msgs/Header header
uint32 detection_id
uint32 track_id
geometry_msgs/Point[] points
float32 confidence
string source
```

### `PerceptionObject`

차량, 보행자, 장애물, 표지판, 신호등 등 객체 관측입니다.

```text
uint8 CLASS_VEHICLE=1
uint8 CLASS_BIKE=2
uint8 CLASS_PEDESTRIAN=3
uint8 CLASS_TRAFFIC_CONE=4
uint8 CLASS_OBSTACLE=5
uint8 CLASS_TRAFFIC_LIGHT=6
uint8 CLASS_SIGN=7

uint32 detection_id
uint32 track_id
uint8 class_id
geometry_msgs/Pose pose
geometry_msgs/Vector3 size
geometry_msgs/Twist twist
float32 confidence
string source
```

### `PerceptionObjectArray`

```text
std_msgs/Header header
my_msgs/PerceptionObject[] objects
```

### `TrafficLightObservation`

신호등 상태 score와 최종 state 관측입니다.

```text
uint8 STATE_UNKNOWN=0
uint8 STATE_RED=1
uint8 STATE_YELLOW=2
uint8 STATE_GREEN=3
uint8 STATE_ARROW=4

uint32 detection_id
uint32 track_id
uint8 state
geometry_msgs/Point position
float32 red_score
float32 yellow_score
float32 green_score
float32 arrow_score
float32 confidence
string source
```

### `TrafficLightObservationArray`

```text
std_msgs/Header header
my_msgs/TrafficLightObservation[] lights
```

## 판단/제어 파이프라인 메시지

### `SceneSummary`

Perception Gateway가 여러 인지 topic을 Main Planning Engine이 바로 쓸 수 있게 요약한 장면 정보입니다.

```text
std_msgs/Header header
bool lane_valid
bool centerline_valid
float32 centerline_quality
float32 centerline_visible_length
geometry_msgs/Point[] centerline_points
bool left_lane_center_valid
geometry_msgs/Point[] left_lane_center_points
bool right_lane_center_valid
geometry_msgs/Point[] right_lane_center_points
float32 lane_width_estimate_m
float32 stopline_distance
uint8 traffic_light_state
float32 traffic_light_confidence
bool obstacle_on_path
float32 front_obstacle_distance
float32 front_obstacle_x
float32 front_obstacle_y
float32 front_obstacle_length
float32 front_obstacle_width
string obstacle_risk_level
string perception_health
float32 oldest_data_age_ms
string[] events
```

### `RouteContext`

Route & Zone Manager가 제공하는 최신 route/zone cache입니다.

```text
std_msgs/Header header
float32 progress_s
string current_zone
string next_zone
string next_maneuver
float32 distance_to_stopline
float32 distance_to_intersection
float32 distance_to_finish
bool route_projection_valid
float32 projection_confidence
float32 cross_track_error
float32 heading_error
geometry_msgs/Point[] local_path_points
```

### `BehaviorDecision`

Main Planning Engine의 우선순위 선택 및 Scenario FSM 결과입니다.

```text
std_msgs/Header header
string safety_state
string active_behavior
string fsm_state
string selected_reason
bool need_stop
float32 target_speed_limit_mps
string path_request
float32 stop_target_distance
string[] constraints
string risk_level
```

### `TargetSpeed`

Planning Core가 path와 같은 planning tick에서 산출한 속도 목표입니다.

```text
std_msgs/Header header
float32 target_speed_mps
float32 speed_limit_mps
bool need_stop
float32 stop_target_distance
string source_behavior
string[] constraints
```

### `AEBState`

AEB Fast Path가 최종 command gate에 전달하는 긴급제동 상태입니다.

```text
std_msgs/Header header
bool active
string state
string reason
float32 front_distance
float32 ttc
float32 object_data_age_ms
```
