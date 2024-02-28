from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    
    marvin_path = FindPackageShare('marvin')
    default_model_path = 'urdf/open_manipulator.urdf.xacro' # not PathJoinSubstitution because it will be used in urdf_launch PathJoinSubstitution
    default_rviz_config_path = PathJoinSubstitution([marvin_path, 'rviz', 'urdf.rviz'])

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui'))
    ld.add_action(DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute Path to rviz config file'))
    ld.add_action(DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                        description='Path to robot urdf file relative to urdf_tutorial package'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'marvin',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))

    return ld