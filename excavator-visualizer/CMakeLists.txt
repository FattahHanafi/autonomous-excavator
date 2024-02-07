cmake_minimum_required(VERSION 3.8)
project(excavator-visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# se(CMAKE_BUILD_TYPE Debug) set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(joint_publisher src/joint_publisher.cpp)
ament_target_dependencies(joint_publisher rclcpp sensor_msgs)

add_executable(stroke_publisher src/stroke_publisher.cpp)
ament_target_dependencies(stroke_publisher rclcpp sensor_msgs)

add_executable(excavator_tf2_broadcaster src/excavator_tf2_broadcaster.cpp)
ament_target_dependencies(excavator_tf2_broadcaster rclcpp geometry_msgs
                          sensor_msgs tf2 tf2_ros)

# install(TARGETS motion_server camera_server)

install(TARGETS joint_publisher stroke_publisher
                excavator_tf2_broadcaster DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY launch urdf config DESTINATION share/${PROJECT_NAME}/)

ament_package()
