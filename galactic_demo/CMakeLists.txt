cmake_minimum_required(VERSION 3.5)
project(galactic_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS my_sad_server.py DESTINATION lib/${PROJECT_NAME})


# target_compile_definitions(action_client
#   PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")


add_executable(galactic_demo src/galactic_demo.cpp)
target_include_directories(galactic_demo PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  moveit_core
  moveit_ros_planning_interface
  control_msgs
  rclcpp
  rclcpp_components
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS scripts/rock_it DESTINATION lib/${PROJECT_NAME})

install(TARGETS galactic_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
