from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    #moveit_config = MoveItConfigsBuilder("marvin", package_name="marvin_moveit").to_moveit_configs()
    moveit_config = (
        MoveItConfigsBuilder("marvin").robot_description("ros2_ws/src/marvin/urdf/marvin.urdf").robot_description_semantic("ros2_ws/src/marvin_moveit/config/marvin.srdf").trajectory_executiion("ros2_ws/src/marvin_moveit/config/joint_limits.yaml").to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
