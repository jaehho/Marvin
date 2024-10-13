# Dockerfile for OpenManipulator-X with ROS Noetic
FROM osrf/ros:noetic-desktop-full-focal

# Add sourcing to shell startup script
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    udev \
    && apt upgrade -y

# Install OpenManipulator-X packages
RUN apt-get install -y \
    ros-noetic-dynamixel-sdk \
    ros-noetic-dynamixel-workbench* \
    ros-noetic-robotis-manipulator

# COPY ./ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ]