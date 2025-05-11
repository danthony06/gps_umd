# Try to find libgps, first with CMake's usual library search method, then by
# querying pkg-config.
find_library(libgps_LIBRARIES NAMES gps)
find_path(libgps_INCLUDE_DIRS NAMES libgpsmm.h gps.h)

include_directories(include)

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

add_library(gpsd_client_reader
  SHARED
    src/gpsd_reader.cpp
    src/gpsd_reader_impl_v11.cpp
)

target_compile_definitions(gpsd_client_reader
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

target_include_directories(gpsd_client_reader
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${libgps_INCLUDE_DIRS}
)

target_link_libraries(gpsd_client_reader
  PUBLIC
    ${libgps_LIBRARIES}
)

install(TARGETS gpsd_client_reader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
