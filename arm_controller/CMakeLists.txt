cmake_minimum_required(VERSION 3.8)
project(arm_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(chr_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(arm_dynamics SHARED src/arm_dynamics_library.cpp)
target_include_directories(arm_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(arm_controller_node src/arm_controller_node.cpp)
target_link_libraries(arm_controller_node arm_dynamics)
ament_target_dependencies(arm_controller_node chr_msgs rclcpp)

install(TARGETS arm_dynamics arm_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(arm_dynamics)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arm_dynamics test/test_arm_dynamics.cpp)
  target_link_libraries(test_arm_dynamics arm_dynamics)
endif()

ament_package()
