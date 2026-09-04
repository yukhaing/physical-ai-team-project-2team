cmake_minimum_required(VERSION 3.8)
project(beagle_slam)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/beagle_base_node.py
  scripts/beagle_mapping_explorer.py
  scripts/save_map.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
