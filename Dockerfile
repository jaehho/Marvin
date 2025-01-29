# Use the official ROS 2 Humble base image
FROM osrf/ros:jazzy-desktop-full

# # Install additional dependencies if needed
# RUN apt-get update && apt-get upgrade -y && apt-get install -y \
#     python3 \
#     python-is-python3 \
#     python3-pip \
#     && rm -rf /var/lib/apt/lists/*

# ========================
# ENTRYPOINT from ros-core
# ========================
# #!/bin/bash
# set -e

# # setup ros2 environment
# source "/opt/ros/$ROS_DISTRO/setup.bash" --
# exec "$@"
# ------------------------