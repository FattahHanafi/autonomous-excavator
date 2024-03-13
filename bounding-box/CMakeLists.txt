cmake_minimum_required(VERSION 3.8)
project(bounding-box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(CGAL REQUIRED)

add_executable(bounding_box src/bounding_box.cpp)
target_include_directories(
  bounding_box PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bounding_box rclcpp visualization_msgs geometry_msgs)

add_executable(zed_bounding_box src/zed_bounding_box.cpp)
target_include_directories(
  zed_bounding_box PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)
ament_target_dependencies(zed_bounding_box rclcpp visualization_msgs
                          geometry_msgs zed_interfaces)

add_executable(closest_distance src/closest_distance.cpp)
target_include_directories(
  closest_distance PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  closest_distance
  rclcpp
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  CGAL)

install(TARGETS bounding_box zed_bounding_box closest_distance
        DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
