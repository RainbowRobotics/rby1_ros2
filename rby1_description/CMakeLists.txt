cmake_minimum_required(VERSION 3.8)
project(rby1_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY launch urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
