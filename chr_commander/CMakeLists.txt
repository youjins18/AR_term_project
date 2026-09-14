cmake_minimum_required(VERSION 3.8)
project(chr_commander)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(keyboard_commander src/keyboard_commander.cpp)
ament_target_dependencies(keyboard_commander rclcpp)
install(TARGETS keyboard_commander DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
