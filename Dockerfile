FROM osrf/ros:galactic-desktop

MAINTAINER Vatan Aksoy Tezer vatan@picknik.ai

WORKDIR /root/ws_stretch/src

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
# Download moveit source so that we can get necessary dependencies
RUN git clone https://github.com/vatnaksoytezer/stretch_ros.git -b pr-docker && \
    vcs import < stretch_ros/stretch_ros.repos && \
    # Update apt package list as cache is cleared in previous container
    # Usually upgrading involves a few packages only (if container builds became out-of-sync)
    apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    # Install package dependencies via rosdep
    rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN cd /root/ws_stretch/ && . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
            --event-handlers desktop_notification- status-

# Remove docker clean and apt update

# Add some bashrc shortcuts

# Install latest VS Code and some utilities (nano, vim)

# Potentially setup VS Code setting and install some extensions
