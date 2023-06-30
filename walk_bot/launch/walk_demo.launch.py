from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('walk_bot'), 'launch', 'description.launch.py']
            )
        )
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('walk_bot'), 'launch', 'gazebo.launch.py']
            )
        )
    )

    walk_node = Node(package='walk', executable='walk')
    ik_node = Node(package='walk_bot', executable='ik')

    return LaunchDescription([
      description_launch,
      gazebo_launch,
      walk_node,
      ik_node,
    ])
