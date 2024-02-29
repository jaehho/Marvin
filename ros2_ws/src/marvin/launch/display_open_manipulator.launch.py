from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Define the paths to the RViz configuration and URDF file
    rviz_config_file = '/home/mili/marvin/ros2_ws/src/marvin/launch/urdf.rviz'
    urdf_file_path = '/home/mili/marvin/ros2_ws/src/marvin/urdf/open_manipulator.urdf'

    # Declare the 'gui' argument to enable or disable the joint_state_publisher_gui
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

    # Read the URDF file content to pass to the robot_state_publisher node
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Create the LaunchDescription object
    ld = LaunchDescription()

    ld.add_action(gui_arg)  # Add the GUI argument to the launch description

    # Node to publish the state of the robot to tf2
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Conditionally launch the joint_state_publisher_gui based on the 'gui' argument
    def launch_joint_state_publisher_gui(context):
        gui = context.launch_configurations['gui']
        if gui == 'true':
            return [Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen'
            )]
        return []

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
    ld.add_action(OpaqueFunction(function=launch_joint_state_publisher_gui))
    ld.add_action(rviz_node)

    return ld