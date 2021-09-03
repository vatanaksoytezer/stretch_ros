FROM osrf/ros:galactic-desktop

MAINTAINER Vatan Aksoy Tezer vatan@picknik.ai

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN \
    # Download moveit source so that we can get necessary dependencies
    git clone https://github.com/PickNikRobotics/stretch_ros.git -b pr-docker && \
    vcs import < stretch_ros/stretch_ros.repos && \
    # Update apt package list as cache is cleared in previous container
    # Usually upgrading involves a few packages only (if container builds became out-of-sync)
    apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    #
    rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh &&\
    colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
            --event-handlers desktop_notification- status-
