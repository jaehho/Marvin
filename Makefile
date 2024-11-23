# Variables
SHELL := /bin/bash
DOCKER_COMPOSE = docker compose
ROS_DISTRO = jazzy
SERVICE_NAME = ros2_${ROS_DISTRO}
WORKSPACE = ./ros2_ws

# Colors
COLOR_GREEN = \\033[0;32m
COLOR_RESET = \\033[0m

.PHONY: default help build up down clean rebuild logs shell ws-build ws-clean

# Default target is `up`
default: up

# Show this help message
help:
	@cat $(MAKEFILE_LIST) | docker run --rm -i xanders/make-help

# Build Docker images
build:
	@echo "$(COLOR_GREEN)Building Docker images...$(COLOR_RESET)"
	${DOCKER_COMPOSE} build

# Create and start containers
up:
	@echo "$(COLOR_GREEN)Starting containers...$(COLOR_RESET)"
	${DOCKER_COMPOSE} up -d

# Stop and remove containers and associated resources
down:
	@echo "$(COLOR_GREEN)Removing containers...$(COLOR_RESET)"
	${DOCKER_COMPOSE} down --remove-orphans

# Clean up Docker environment
clean:
	@echo "$(COLOR_GREEN)Cleaning up Docker environment...$(COLOR_RESET)"
	${DOCKER_COMPOSE} down --remove-orphans
	@docker system prune -a --volumes -f
	@docker rmi -f $(docker images -q) || true
	@docker volume rm $(docker volume ls -q) || true

# Rebuild the Docker image
rebuild: clean build up
	@echo "$(COLOR_GREEN)Rebuilt Docker image and restarted container...$(COLOR_RESET)"

# Follow logs of the container
logs:
	@echo "$(COLOR_GREEN)Viewing logs for ${SERVICE_NAME}...$(COLOR_RESET)"
	${DOCKER_COMPOSE} logs -f ${SERVICE_NAME}

# Access the container shell
shell:
	@echo "$(COLOR_GREEN)Accessing ${SERVICE_NAME} container shell...$(COLOR_RESET)"
	${DOCKER_COMPOSE} exec ${SERVICE_NAME} bash

# Build ROS2 workspace
ws-build:
	@echo "$(COLOR_GREEN)Building ROS2 workspace inside container...$(COLOR_RESET)"
	${DOCKER_COMPOSE} exec ${SERVICE_NAME} bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Clean ROS2 workspace
ws-clean:
	@echo "$(COLOR_GREEN)Cleaning ROS2 workspace build files inside container...$(COLOR_RESET)"
	@rm -rf ${WORKSPACE}/build ${WORKSPACE}/install ${WORKSPACE}/log

##
## Local ROS2
##

source_marvin:
	@echo "$(COLOR_GREEN)Sourcing Marvin workspace...$(COLOR_RESET)"
	. ros2_ws/install/setup.bash

##
## MoveIt2 specific targets
##

# Source MoveIt2 workspace
source_moveit:
	@echo "$(COLOR_GREEN)Sourcing MoveIt2 workspace...$(COLOR_RESET)"
	. ~/ws_moveit/install/setup.bash 

# Launch MoveIt2 setup assistant
launch_setup_assistant:
	@echo "$(COLOR_GREEN)Launching MoveIt2 setup assistant...$(COLOR_RESET)"
	ros2 launch moveit_setup_assistant setup_assistant.launch.py