# Dockerfile for Marvin with ROS2 Jazzy
FROM osrf/ros:jazzy-desktop-full-noble

# Add sourcing to shell startup script
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git

WORKDIR /root/ros2_ws

ENTRYPOINT [ "/ros_entrypoint.sh" ]