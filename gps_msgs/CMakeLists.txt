cmake_minimum_required(VERSION 3.10)
project(gps_msgs)

set(CMAKE_CXX_STANDARD 17)

set(MSG_DEPS std_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(MSG_FILES
  msg/GNSS.msg
  msg/GNSSEcef.msg
  msg/GNSSFix.msg
  msg/GNSSNed.msg
  msg/GNSSSatelliteInfo.msg
  msg/GPSFix.msg
  msg/GPSStatus.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES ${MSG_DEPS}
)

ament_export_dependencies(rosidl_default_runtime)

install(
  FILES ros1_ros2_mapping.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
