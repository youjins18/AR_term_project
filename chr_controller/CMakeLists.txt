cmake_minimum_required(VERSION 3.8)
project(chr_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(chr_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(chr_dynamics SHARED src/chr_dynamics_library.cpp)
target_include_directories(chr_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(chr_dynamics Eigen3::Eigen)

add_executable(chr_controller_node src/chr_controller_node.cpp)
target_link_libraries(chr_controller_node chr_dynamics Eigen3::Eigen)
ament_target_dependencies(chr_controller_node chr_msgs geometry_msgs rclcpp)

install(TARGETS chr_dynamics chr_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config third_party DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(chr_dynamics)
ament_export_dependencies(Eigen3)
ament_package()
