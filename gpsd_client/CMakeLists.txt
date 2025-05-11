cmake_minimum_required(VERSION 3.10)
project(gpsd_client)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_subdirectory(libreader)
add_subdirectory(libnodes)

add_executable(gpsd_client src/gpsd_client.cpp)

target_include_directories(gpsd_client
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(gpsd_client
  PUBLIC
    class_loader::class_loader
    rclcpp::rclcpp
    gpsd_client_nodes
)

#############
## Install ##
#############
install(TARGETS gpsd_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/)

ament_package()
