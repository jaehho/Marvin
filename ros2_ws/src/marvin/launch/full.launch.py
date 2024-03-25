from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'marvin'
    default_model_path = 'urdf/marvin.urdf'
    default_rviz_config_path = PathJoinSubstitution([
        FindPackageShare(package_name), 'rviz', 'urdf.rviz'
    ])

    # Common node parameters
    node_params = {
        'shoulderFlexion': [('left_shoulder_flexion', 'left'), ('right_shoulder_flexion', 'right')],
        'shoulderAdduction': [('left_shoulder_adduction', 'left'), ('right_shoulder_adduction', 'right')],
        'elbowFlexion': [('left_elbow_flexion', 'left'), ('right_elbow_flexion', 'right')]
    }

    ld = LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument('marvin', default_value=package_name, description='Package name'),
        DeclareLaunchArgument('model', default_value=default_model_path, description='Path to the robot description'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path, description='Path to RViz config'),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(
                Command(['xacro ', PathJoinSubstitution([FindPackageShare(package_name), LaunchConfiguration('model')])]),
                value_type=str
            )}]
        ),

        # Pose detection node
        Node(package=package_name, executable='poseDetection', output='screen'),

        # Pose Display node
        Node(package=package_name, executable='poseDisplay', output='screen'),

        # Joint state publisher node
        Node(package=package_name, executable='jointStatePublisher', output='screen'),

        # Display joint states node
        Node(package=package_name, executable='displayJointStates', output='screen'),

        # RViz
        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', LaunchConfiguration('rviz_config')])
    ])

    # Add shoulder flexion, adduction, and elbow flexion nodes
    for executable, names in node_params.items():
        for name, side in names:
            ld.add_action(Node(
                package=package_name,
                executable=executable,
                name=name,
                parameters=[{'side': side}],
                output='screen'
            ))

    return ld
