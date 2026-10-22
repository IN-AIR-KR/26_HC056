cmake_minimum_required(VERSION 3.8)
project(arms_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/DetectionArray.msg"
  "msg/MissionState.msg"
  "msg/CrsfTelemetry.msg"
  "msg/AttitudeTelemetry.msg"
  "msg/BatteryTelemetry.msg"
  "msg/GpsTelemetry.msg"
  "msg/ControlDebug.msg"
  "msg/OperatorInput.msg"
  "msg/FlightCommand.msg"
  "msg/IoStatus.msg"
  "msg/GuidanceGroundTruth.msg"
  "msg/PnDebug.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
