find_package(ament_cmake REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(gpsd_client_nodes
  SHARED
    src/gpsd_client_node.cpp
)

target_compile_definitions(gpsd_client_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL")

target_include_directories(gpsd_client_nodes
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    #rclcpp::rclcpp
    #rclcpp_components::component
    ${gps_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(gpsd_client_nodes
  PUBLIC
    gpsd_client_reader
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    ${gps_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
)

rclcpp_components_register_nodes(gpsd_client_nodes "gpsd_client::GpsdClientNode")
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

install(TARGETS gpsd_client_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
