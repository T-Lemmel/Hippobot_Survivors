cmake_minimum_required(VERSION 3.8)
project(move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

# Include depository
include_directories(include)

# Executables
set(HEADER_FILES
include/boat.h
include/covariances.h
include/gps2enu.h
include/tf_listener.h
)

# C++ executable
ament_auto_add_executable(waypoint_publisher src/waypoint_publisher.cpp)
ament_auto_add_executable(StampingCall src/StampingCall.cpp ${HEADER_FILES})

target_include_directories(StampingCall PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${HEADERS}>
)

target_link_libraries(StampingCall  ${GeographicLib_LIBRARIES})

install(TARGETS
    waypoint_publisher
    StampingCall
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/place_frame.py scripts/tracker.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY scripts/utm
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
        


ament_package()
