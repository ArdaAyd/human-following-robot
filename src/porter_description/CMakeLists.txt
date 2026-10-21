cmake_minimum_required(VERSION 3.20)
project(porter_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
