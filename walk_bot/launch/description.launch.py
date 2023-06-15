from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare launch arguments
    xacro_path_arg = DeclareLaunchArgument(
        name='xacro_path',
        description='Path to the .xacro file to open.',
        default_value=PathJoinSubstitution(
            [FindPackageShare('walk_bot'), 'urdf', 'walk_bot.urdf']
        )
    )

    # Set up robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                # Run xacro to convert urdf to a string
                Command(['xacro ', LaunchConfiguration('xacro_path')])
            )
        }]
    )

    return LaunchDescription([
        xacro_path_arg,
        robot_state_publisher_node,
    ])
