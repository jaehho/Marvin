# Use the official ROS 2 Humble base image
FROM osrf/ros:humble-desktop-full

# Install additional dependencies if needed
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    python3 \
    python-is-python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /root/marvin/

ENTRYPOINT [ "/ros_entrypoint.sh" ]