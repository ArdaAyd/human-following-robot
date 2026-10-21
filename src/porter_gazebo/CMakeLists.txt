cmake_minimum_required(VERSION 3.20)
project(porter_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
