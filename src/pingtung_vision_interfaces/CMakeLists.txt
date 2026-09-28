cmake_minimum_required(VERSION 3.8)
project(pingtung_vision_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AnimalResult.msg"
  "msg/FruitLineHit.msg"
  "msg/PigShitSpatial.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
