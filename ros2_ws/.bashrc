source /opt/ros/jazzy/setup.bash && echo "Sourced Jazzy"

source ~/ws_moveit/install/setup.bash && echo "Sourced moveit"

source ~/jazzy_open_manipulator/install/setup.bash && echo "Sourced open manipulator"

alias openmanipulator_controller="ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py" && echo "Alias openmanipulator_controller set"