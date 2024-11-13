# Variables
DOCKER_COMPOSE = docker compose
ROS_DISTRO = jazzy
SERVICE_NAME = ros2_${ROS_DISTRO}
WORKSPACE = ./ros2_ws

.PHONY: default help build up down clean rebuild logs shell ws-build ws-clean

# Default target is `up`
default: up

# Show this help message
help:
	@cat $(MAKEFILE_LIST) | docker run --rm -i xanders/make-help

# Build Docker images
build:
	@echo "Building Docker images..."
	${DOCKER_COMPOSE} build

# Create and start containers
up:
	@echo "Starting containers..."
	${DOCKER_COMPOSE} up -d

# Stop and remove containers and associated resources
down:
	@echo "Removing containers..."
	${DOCKER_COMPOSE} down --remove-orphans

# Clean up Docker environment
clean:
	@echo "Cleaning up Docker environment..."
	${DOCKER_COMPOSE} down --remove-orphans
	@docker system prune -a --volumes -f
	@docker rmi -f $(docker images -q) || true
	@docker volume rm $(docker volume ls -q) || true

# Rebuild the Docker image
rebuild: clean build up
	@echo "Rebuilt Docker image and restarted container..."

# Follow logs of the container
logs:
	@echo "Viewing logs for ${SERVICE_NAME}..."
	${DOCKER_COMPOSE} logs -f ${SERVICE_NAME}

# Access the container shell
shell:
	@echo "Accessing ${SERVICE_NAME} container shell..."
	${DOCKER_COMPOSE} exec ${SERVICE_NAME} bash

# Build ROS2 workspace
ws-build:
	@echo "Building ROS2 workspace inside container..."
	${DOCKER_COMPOSE} exec ${SERVICE_NAME} bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Clean ROS2 workspace
ws-clean:
	@echo "Cleaning ROS2 workspace build files inside container..."
	@rm -rf ${WORKSPACE}/build ${WORKSPACE}/install ${WORKSPACE}/log
