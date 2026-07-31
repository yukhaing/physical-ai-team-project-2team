cmake_minimum_required(VERSION 3.8)
project(omx_target_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

add_library(omx_target_panel SHARED
  src/omx_target_panel.cpp
  include/omx_target_control/omx_target_panel.hpp
)
target_include_directories(omx_target_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  omx_target_panel geometry_msgs pluginlib rclcpp rviz_common visualization_msgs
)
target_link_libraries(omx_target_panel Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS omx_target_panel
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS scripts/box_target_pose_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
