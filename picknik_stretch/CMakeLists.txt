cmake_minimum_required(VERSION 3.0.2)
project(picknik_stretch)

find_package(catkin REQUIRED)

catkin_package()

install(FILES config/view_robot.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(FILES launch/see_stretch.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
