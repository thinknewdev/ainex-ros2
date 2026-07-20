cmake_minimum_required(VERSION 3.8)
project(ainex_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes worlds config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
