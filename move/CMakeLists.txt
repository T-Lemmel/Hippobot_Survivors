cmake_minimum_required(VERSION 3.8)
project(move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)


# C++ executable
add_executable(waypoint_publisher src/waypoint_publisher.cpp)
ament_target_dependencies(waypoint_publisher rclcpp geometry_msgs)

install(TARGETS
    waypoint_publisher
    DESTINATION lib/${PROJECT_NAME}
)


# Python scripts installation
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/place_frame.py scripts/tracker.py
        DESTINATION lib/${PROJECT_NAME})

# Additional installation for non-Python files (if any)
install(DIRECTORY scripts/utm
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
        
ament_package()
