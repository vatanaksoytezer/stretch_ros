# Stretch Docker Documentation for ROS World Workshop

Welcome to our ROS World Workshop! This README is a documentation to go through installation steps to install docker and setup your docker environment for Stretch.

## Installation (Linux)

### Prerequisites

#### Install Docker

#### Docker post-install steps

#### Install nvidia-docker2 (Skip if you don't have an nvidia gpu)

### Getting Started with Stretch Docker

We provide a pre-built docker image and a set of scripts to easily get you up and running with Strectch on dockers! To get your docker running with UI and networking enabled:

1) Fetch our docker script

    ```bash
    git clone https://github.com/PickNikRobotics/stretch_ros -b pr-docker 
    ```

2) Start your container from our pre-built image

    ```bash
    chmod +x start_docker.sh && \
    ./start_docker.sh stretch_roscon hello-robot/stretch:roscon
    ```

3) At this point you should be seeing a terminal window with all the necessary source code pre-built for you. 
TODO: Add image

    Test your docker image by bringing up Stretch in Ignition Gazebo by issuing the following set of commands inside your container:
    ```bash
    source /opt/ros/galactic/setup.bash && \
    source /root/ws_stretch/install/setup.bash && \
    ros2 launch stretch_ignition ignition.launch.py
    ```

### Building the docker image from scratch

## Installation (Windows)

## Installation (MacOS)

## Editing the source code during the Workshop

Our docker image comes with a pre-built 