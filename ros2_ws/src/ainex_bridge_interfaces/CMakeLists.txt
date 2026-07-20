cmake_minimum_required(VERSION 3.8)
project(ainex_bridge_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BusServoPosition.msg
  msg/SetBusServosPosition.msg
  srv/GetBusServosPosition.srv
)
ament_package()
