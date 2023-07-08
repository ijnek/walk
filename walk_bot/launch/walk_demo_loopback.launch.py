from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    period_launch_arg = DeclareLaunchArgument('period', default_value='0.25')

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('walk_bot'), 'launch', 'description.launch.py']
            )
        )
    )

    walk_node = Node(package='walk', executable='walk')
    ik_node = Node(
        package='walk_bot',
        executable='ik',
        remappings=[('joint_command', 'joint_states')])
    phase_provider_node = Node(
        package='walk_bot',
        executable='phase_provider',
        parameters=[{'period': LaunchConfiguration('period')}])

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('walk_bot'), 'rviz', 'walk_demo.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        period_launch_arg,
        description_launch,
        walk_node,
        ik_node,
        phase_provider_node,
        rviz_node,
    ])
