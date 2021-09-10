FROM osrf/ros:galactic-desktop

MAINTAINER Vatan Aksoy Tezer vatan@picknik.ai

WORKDIR /root/ws_stretch/src

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
# Download stretch source so that we can get necessary dependencies
RUN git clone https://github.com/vatanaksoytezer/stretch_ros.git -b pr-docker && \
    vcs import < stretch_ros/stretch_ros.repos

# Update and install dependencies
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade && \
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

# Install latest VS Code and some choice of editors (nano, vim, emacs)
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
    install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/ && \
    sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list' && \
    rm -f packages.microsoft.gpg && \
    apt-get update && \
    apt-get install apt-transport-https nano vim emacs code


# Potentially setup VS Code setting and install some extensions
