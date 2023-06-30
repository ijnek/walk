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

    ik_node = Node(
      package='walk_bot',
      executable='ik',
      remappings=[('joint_command', 'joint_states')])  # This simulates a perfect joint movement
    sole_poses_ims = Node(
      package='sole_poses_ims',
      executable='sole_poses_ims',
      parameters=[{
        'l_sole_default_y': 0.0,
        'l_sole_default_z': -0.4,
        'r_sole_default_y': 0.0,
        'r_sole_default_z': -0.4,
      }],
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('walk_bot'), 'rviz', 'ik.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        description_launch,
        ik_node,
        sole_poses_ims,
        rviz_node,
    ])
