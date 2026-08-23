cmake_minimum_required(VERSION 3.8)
project(kaiev26_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorCommand.msg"
  "msg/AEBState.msg"
  "msg/BehaviorDecision.msg"
  "msg/Centerline.msg"
  "msg/DriveIntent.msg"
  "msg/PerceptionObject.msg"
  "msg/PerceptionObjectArray.msg"
  "msg/RoadSegment.msg"
  "msg/RoadSegmentArray.msg"
  "msg/RouteContext.msg"
  "msg/SceneSummary.msg"
  "msg/TargetSpeed.msg"
  "msg/TrafficLightObservation.msg"
  "msg/TrafficLightObservationArray.msg"
  "msg/VehicleState.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
