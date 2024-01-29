cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PCL REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_add_executable(DetectAndPublishCentroids src/DetectAndPublishCentroids.cpp)
ament_auto_add_executable(FillMap src/FillMap.cpp)

target_link_libraries(DetectAndPublishCentroids ${PCL_LIBRARIES})
target_link_libraries (FillMap ${PCL_LIBRARIES})

# Add dependencies
ament_target_dependencies(DetectAndPublishCentroids
  geometry_msgs
  rclcpp
)

# Add dependencies
ament_target_dependencies(FillMap
  geometry_msgs
  rclcpp
  nav_msgs
  map_msgs
)

install(DIRECTORY scripts/utm
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()
