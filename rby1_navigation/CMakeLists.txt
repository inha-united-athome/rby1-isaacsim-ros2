cmake_minimum_required(VERSION 3.14)
project(rby1_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  maps
  scripts
  rviz2
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
