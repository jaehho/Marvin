# Use the official Ubuntu 20.04 image as the base image
FROM ubuntu:20.04

# Set non-interactive mode for APT
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install necessary dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    git

# Setup sources and keys
RUN apt-get update && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic and dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialize rosdep
RUN rosdep init && rosdep update

# Install OpenManipulator-X dependencies
RUN apt-get install -y \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo* \
    ros-noetic-moveit* \
    ros-noetic-industrial-core \
    ros-noetic-dynamixel-sdk \
    ros-noetic-dynamixel-workbench* \
    ros-noetic-robotis-manipulator

# Set up the ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
