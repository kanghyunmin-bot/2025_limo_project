cmake_minimum_required(VERSION 3.8)
project(map_view_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
