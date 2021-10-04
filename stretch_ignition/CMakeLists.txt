cmake_minimum_required(VERSION 3.10.2)
project(stretch_ignition)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(odom2tf
    src/odom2tf.cpp
)
target_include_directories(odom2tf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
    odom2tf
    geometry_msgs
    nav_msgs
    rclcpp
    tf2
    tf2_ros
)
# Install executable
install(TARGETS odom2tf DESTINATION lib/${PROJECT_NAME})

# Install packages
install(DIRECTORY worlds rviz launch DESTINATION share/${PROJECT_NAME})

ament_package()
