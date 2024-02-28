from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define the paths to the RViz configuration and URDF file
    rviz_config_file = '/home/mili/marvin/ros2_ws/src/marvin/launch/urdf.rviz'
    urdf_file_path = '/home/mili/marvin/ros2_ws/src/marvin/urdf/marvin.urdf'

    # Read the URDF file content to pass to the robot_state_publisher node
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Create the LaunchDescription object
    ld = LaunchDescription()

    # Node to publish the state of the robot to tf2
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Node to launch RViz2 with the specified configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Add the nodes to the LaunchDescription
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld