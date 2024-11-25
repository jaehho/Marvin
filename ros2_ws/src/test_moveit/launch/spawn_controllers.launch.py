from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("2dof_robot", package_name="test_moveit").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
