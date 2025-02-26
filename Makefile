.SILENT:
.IGNORE:
.PHONY:

# Variables
DOCKER_COMPOSE = docker compose
ROS_DISTRO = jazzy
CONTAINER = ros2
WORKSPACE = ./ros2_ws

# Colors
COLRO_RED = \033[0;31m
COLOR_GREEN = \033[0;32m
COLOR_YELLOW = \033[0;33m
COLOR_BLUE = \033[0;34m
COLOR_MAGENTA = \033[0;35m
COLOR_CYAN = \033[0;36m
COLOR_WHITE = \033[0;37m
COLOR_RESET = \033[0m

help: ## Show this help message
	echo "Available targets:"
	echo "=================="
	grep -E '(^[a-zA-Z_-]+:.*?## .*$$|^# Section: )' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; \
		     /^# Section:/ {gsub("^# Section: ", ""); print "\n\033[1;35m" $$0 "\033[0m"}; \
		     /^[a-zA-Z_-]+:/ {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

chownme: ## Change ownership of all files in the workspace to the current user
	sudo chown -R $(shell whoami) ./

update_bashrc: ## Update ~/.bashrc with ros2_ws/.bashrc
	./ros2_ws/update_bashrc.sh

# Section: Docker

docker_build: ## Build Docker container(s)
	$(DOCKER_COMPOSE) build

docker_up: ## Start Docker container(s)
	$(DOCKER_COMPOSE) up -d

docker_attach: ## Attach to the container
	$(DOCKER_COMPOSE) exec -it $(CONTAINER) bash

docker_down: ## Stop Docker containers and clean up
	$(DOCKER_COMPOSE) down -t 1 --remove-orphans

# Section: MoveIt2 targets

launch_setup_assistant: ## Launch MoveIt2 setup assistant
	echo "$(COLOR_GREEN)Launching MoveIt2 setup assistant...$(COLOR_RESET)"
	ros2 launch moveit_setup_assistant setup_assistant.launch.py

# Section: OpenManipulatorX targets

launch_open_manipulator_x_controller: ## Launch OpenManipulatorX controller
	ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py

launch_open_manipulator_x_rviz: ## Launch OpenManipulatorX RViz
	ros2 launch open_manipulator_x_description open_manipulator_x_rviz.launch.py 

# Section: Local targets

install-dependencies: ## Check for missing dependencies
	rosdep install -i --from-path ${WORKSPACE}/src --rosdistro ${ROS_DISTRO} -y

clean: ## Clean ROS2 workspace build files
	rm -rf ${WORKSPACE}/build ${WORKSPACE}/install ${WORKSPACE}/log

build: ## Build ROS2 workspace
	cd ${WORKSPACE} && colcon build --symlink-install
