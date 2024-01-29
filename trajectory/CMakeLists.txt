cmake_minimum_required(VERSION 3.8)
project(astar_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(include)

# Executables
set(HEADER_FILES
include/astar_path_planner/a_star.h
include/astar_path_planner/point.h
)
add_executable(path_planner_cell_node src/path_planner_cell_node.cpp ${HEADER_FILES})
ament_target_dependencies(path_planner_cell_node rclcpp geometry_msgs nav_msgs)
target_include_directories(path_planner_cell_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${HEADERS}>
  ${OpenCV_INCLUDE_DIRS}  # Include OpenCV directories
)
target_compile_features(path_planner_cell_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS path_planner_cell_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

