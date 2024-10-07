# Dockerfile for OpenManipulator-X with ROS Noetic
FROM osrf/ros:noetic-desktop-full-focal

# Add sourcing to shell startup script
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    udev

# Install OpenManipulator-X packages
RUN apt-get install -y \
    # ros-noetic-ros-controllers \
    # ros-noetic-gazebo* \
    # ros-noetic-moveit* \
    # ros-noetic-industrial-core \
    ros-noetic-dynamixel-sdk \
    ros-noetic-dynamixel-workbench* \
    ros-noetic-robotis-manipulator

# ENTRYPOINT [ "/ros_entrypoint.sh" ]
ENTRYPOINT [ "/bin/bash" ]