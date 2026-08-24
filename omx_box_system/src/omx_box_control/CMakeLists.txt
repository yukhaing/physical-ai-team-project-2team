cmake_minimum_required(VERSION 3.10)
project(omx_box_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
add_executable(omx_xyz_controller_node src/omx_xyz_controller_node.cpp)
ament_target_dependencies(
  omx_xyz_controller_node geometry_msgs moveit_ros_planning_interface rclcpp)

add_library(omx_target_panel SHARED
  src/omx_target_panel.cpp
  include/omx_box_control/omx_target_panel.hpp
)
target_include_directories(omx_target_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  omx_target_panel geometry_msgs pluginlib rclcpp rviz_common visualization_msgs)
target_link_libraries(omx_target_panel Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS omx_target_panel omx_xyz_controller_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/box_target_pose_bridge_node.py
  scripts/camera_homography_target_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
