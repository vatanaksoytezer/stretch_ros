FROM osrf/ros:galactic-desktop

MAINTAINER Vatan Aksoy Tezer vatan@picknik.ai

# Update and install some common depenccies
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade && apt-get install -y wget python3-pip lsb-release gnupg curl python3-vcstool python3-colcon-common-extensions git && \
    rosdep update

WORKDIR /root/ws_ignition/src

# Download Ignition repository dictionary
RUN wget https://raw.githubusercontent.com/vatanaksoytezer/ign-ci/main/ignition.repos

# Install Ignition Gazebo from source
RUN vcs import < ignition.repos && \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'  && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -  && \
    apt-get -qq update  && \
    cd /root/ws_ignition/ && \
    apt-get install -y $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/ignition\|sdf/d' | tr '\n' ' ') && \
    colcon build --merge-install --cmake-args -DBUILD_TESTING=OFF && \

WORKDIR /root/ws_stretch/src

# Download stretch source so that we can get necessary dependencies
RUN git clone https://github.com/vatanaksoytezer/stretch_ros.git -b pr-docker && \
    vcs import < stretch_ros/stretch_ros.repos

# Update and install dependencies
RUN . /opt/ros/galactic/setup.sh && \
    . /root/ws_ignition/install/setup.sh && \
    export IGNITION_VERSION=edifice && \
    rosdep install -y --from-paths . --ignore-src --rosdistro galactic --as-root=apt:false

# Build the workspace
RUN cd /root/ws_stretch/ && . /opt/ros/galactic/setup.sh && . /root/ws_ignition/install/setup.sh && \
    export IGNITION_VERSION=edifice && \
    colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
            --event-handlers desktop_notification- status-

# Remove docker clean and apt update

# Add some bashrc shortcuts
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc && \
    echo "source /root/ws_ignition/install/setup.bash" >> ~/.bashrc && \
    echo "source /root/ws_stretch/install/setup.bash" >> ~/.bashrc && \
    echo "export IGNITION_VERSION=edifice" >> ~/.bashrc && \
    echo "export IGN_GAZEBO_RESOURCE_PATH=/root/ws_stretch/src/stretch_ros:/root/ws_stretch/src/realsense-ros:/root/ws_stretch/src/aws-robomaker-small-house-world/models" >> ~/.bashrc


# Install latest VS Code and some choice of editors (nano, vim, emacs)
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg && \
    install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/ && \
    sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list' && \
    rm -f packages.microsoft.gpg && \
    apt-get -qq update && \
    apt-get install apt-transport-https nano vim emacs code -y && \
    rm -rf /var/lib/apt/lists/*


# Potentially setup VS Code setting and install some extensions
