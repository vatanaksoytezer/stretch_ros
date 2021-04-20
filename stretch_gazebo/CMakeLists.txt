cmake_minimum_required(VERSION 3.0.2)
project(stretch_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(FILES config/joints.yaml config/sim.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(FILES launch/gazebo.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES urdf/stretch_gazebo.urdf.xacro
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
