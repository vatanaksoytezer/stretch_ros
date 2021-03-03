cmake_minimum_required(VERSION 3.0.2)
project(stretch_plan_client)

find_package(catkin REQUIRED COMPONENTS moveit_ros_planning_interface roscpp)
find_package(Eigen3)

catkin_package(
    CATKIN_DEPENDS moveit_ros_planning_interface roscpp
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_executable(stretch_plan_client src/stretch_plan_client.cpp)
target_link_libraries(stretch_plan_client ${catkin_LIBRARIES})
install(TARGETS stretch_plan_client
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
