# Variables
DOCKER_COMPOSE = docker compose
ROS_DISTRO = jazzy
CONTAINER = ros2
WORKSPACE = ./ros2_ws

# Colors
COLOR_GREEN = \033[0;32m
COLOR_BLUE = \033[0;34m
COLOR_RESET = \033[0m

help: ## Show this help message
	@echo "Available targets:"
	@echo "=================="
	@grep -E '(^[a-zA-Z_-]+:.*?## .*$$|^# Section: )' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; \
		     /^# Section:/ {gsub("^# Section: ", ""); print "\n\033[1;35m" $$0 "\033[0m"}; \
		     /^[a-zA-Z_-]+:/ {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

chownme: ## Change ownership of all files in the workspace to the current user
	@sudo chown -R $(shell whoami) ./

# Section: MoveIt2 targets

launch_setup_assistant: ## Launch MoveIt2 setup assistant
	@echo "$(COLOR_GREEN)Launching MoveIt2 setup assistant...$(COLOR_RESET)"
	ros2 launch moveit_setup_assistant setup_assistant.launch.py

# Section: OpenManipulatorX targets

launch_open_manipulator_x_controller: ## Launch OpenManipulatorX controller
	ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py

launch_open_manipulator_x_rviz: ## Launch OpenManipulatorX RViz
	ros2 launch open_manipulator_x_description open_manipulator_x_rviz.launch.py 

# Section: Local targets

install-dependencies: ## Check for missing dependencies
	@echo "$(COLOR_GREEN)Checking for missing dependencies...$(COLOR_RESET)"
	rosdep install -i --from-path ${WORKSPACE}/src --rosdistro ${ROS_DISTRO} -y

clean: ## Clean ROS2 workspace build files
	@echo "$(COLOR_GREEN)Cleaning ROS2 workspace build files...$(COLOR_RESET)"
	@rm -rf ${WORKSPACE}/build ${WORKSPACE}/install ${WORKSPACE}/log

build: ## Build ROS2 workspace
	@echo "$(COLOR_GREEN)Building ROS2 workspace...$(COLOR_RESET)"
	@cd ${WORKSPACE} && colcon build --symlink-install

# Section: Docker

docker_build: ## Build Docker container(s)
	@$(DOCKER_COMPOSE) build

docker_up: ## Start Docker container(s)
	@$(DOCKER_COMPOSE) up -d

docker_attach: ## Attach to the container
	@$(DOCKER_COMPOSE) exec -it $(CONTAINER) bash

docker_teardown: ## Stop Docker containers and clean up
	@$(DOCKER_COMPOSE) down -t 1 --remove-orphans

test: ## Test Docker container(s)
	echo $(IS_RUNNING)