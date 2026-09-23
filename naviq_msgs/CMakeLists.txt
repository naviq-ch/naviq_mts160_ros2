cmake_minimum_required(VERSION 3.8)
project(naviq_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Track.msg"
  "msg/TrackDetection.msg"
  "msg/Markers.msg"
  "msg/Navicode.msg"
  "msg/RawTpdo.msg"
  "srv/SelfTest.srv"
  "srv/Zero.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
