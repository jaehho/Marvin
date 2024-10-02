# Dockerfile for OpenManipulator-X with ROS Noetic
FROM osrf/ros:noetic-desktop-focal

# Install ROS Noetic
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full && \
    rm -rf /var/lib/apt/lists/*

# Always source environment in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git

# Install OpenManipulator-X packages
RUN apt-get install -y \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo* \
    ros-noetic-moveit* \
    ros-noetic-industrial-core \
    ros-noetic-dynamixel-sdk \
    ros-noetic-dynamixel-workbench* \
    ros-noetic-robotis-manipulator

# ENTRYPOINT [ "/ros_entrypoint.sh" ]
ENTRYPOINT [ "/bin/bash" ]