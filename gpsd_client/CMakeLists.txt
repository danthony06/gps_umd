cmake_minimum_required(VERSION 3.10)
project(gpsd_client)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Try to find libgps, first with CMake's usual library search method, then by
# querying pkg-config.
find_library(libgps_LIBRARIES NAMES gps)
find_path(libgps_INCLUDE_DIRS NAMES libgpsmm.h gps.h)

if(NOT libgps_LIBRARIES)
  message(STATUS "Checking pkg-config for libgps")
  find_package(PkgConfig)
  if(PkgConfig_FOUND)
    pkg_check_modules(libgps libgps)
  endif()
endif()

if(NOT libgps_LIBRARIES)
  message(FATAL_ERROR "Could not find libgps "
                      "(hint for Debian/Ubuntu: apt install libgps-dev)")
else()
  message(STATUS "Found libgps: ${libgps_LIBRARIES}")
endif()

include_directories(include)

add_library(
  gpsd_client_nodes
    SHARED
      lib/gpsd_client_node.cpp
      lib/gpsd_parser.cpp
      lib/gpsd_parser_impl_v11.cpp
      lib/gpsd_reader_impl.cpp
      lib/gpsd_reader.cpp)

target_compile_definitions(gpsd_client_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL")

target_include_directories(
  gpsd_client_nodes
    PRIVATE
      include
      ${libgps_INCLUDE_DIRS})

target_link_libraries(
  gpsd_client_nodes
    PUBLIC
      rclcpp::rclcpp
      rclcpp_components::component
      ${rclcpp_LIBRARIES}
      ${gps_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
    PRIVATE
      ${libgps_LIBRARIES})

rclcpp_components_register_nodes(gpsd_client_nodes "gpsd_client::GpsdClientNode")
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

add_executable(gpsd_client src/gpsd_client.cpp)
target_link_libraries(
  gpsd_client
    PRIVATE
      gpsd_client_nodes
      class_loader::class_loader
      rclcpp::rclcpp)

#############
## Install ##
#############
install(TARGETS gpsd_client_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS gpsd_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/)

ament_package()
