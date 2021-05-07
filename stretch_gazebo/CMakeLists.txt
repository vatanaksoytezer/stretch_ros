cmake_minimum_required(VERSION 3.10.2)
project(stretch_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf DESTINATION share/${PROJECT_NAME})

ament_package()
