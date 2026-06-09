cmake_minimum_required(VERSION 3.8)
project(tianji_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
