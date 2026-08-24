cmake_minimum_required(VERSION 3.10)
project(omx_box_control)

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
install(TARGETS omx_target_panel
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
)
install(PROGRAMS
  scripts/box_target_pose_bridge_node.py
  scripts/place_keyboard_teach_node.py
  scripts/camera_homography_target_node.py
  scripts/movej_staging_node.py
  scripts/movej_xy_approach_node.py
  scripts/movej_closed_loop_descent_node.py
  scripts/movej_single_ptp_descent_node.py
  scripts/movej_loaded_lift_node.py
  scripts/movej_place_rotate_node.py
  scripts/gripper_open_node.py
  scripts/movej_smooth_relay_node.py
  scripts/pick_coordinator_node.py
  scripts/rule_based_pick_node.py
  scripts/yolo_detection_node.py
  scripts/beagle_adapter_node.py
  scripts/sorting_orchestrator_node.py
  scripts/operations_log_node.py
  scripts/omx_console.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
