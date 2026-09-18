cmake_minimum_required(VERSION 3.8)
project(chr_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmCommand.msg"
  "msg/ChrDiagnostics.msg"
  "msg/ChrReference.msg"
  "msg/ChrState.msg"
  "msg/FlightDiagnostics.msg"
  "msg/PalletroneCommand.msg"
  DEPENDENCIES geometry_msgs std_msgs)

ament_export_dependencies(rosidl_default_runtime)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
