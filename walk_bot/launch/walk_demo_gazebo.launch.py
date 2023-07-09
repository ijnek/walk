# Copyright 2023 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    period_launch_arg = DeclareLaunchArgument('period', default_value='0.30')

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
        gazebo_launch,
        walk_node,
        ik_node,
        phase_provider_node,
        rviz_node,
    ])
