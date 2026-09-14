cmake_minimum_required(VERSION 3.8)
project(palletrone_flight_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(chr_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(palletrone_flight_controller_node
  src/palletrone_flight_controller_node.cpp)
ament_target_dependencies(palletrone_flight_controller_node chr_msgs rclcpp)

install(TARGETS palletrone_flight_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
