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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Get ROS_DISTRO
    ros_distro = os.environ.get('ROS_DISTRO')

    # Determine Ignition Gazebo version
    # Unfortunately, there seems to be no way to determine the installed gazebo version easily.
    # For now, just assume the gazebo version depending on the ros distro as defined in REP-2000.
    # Ideally, we wouldn't want to infer the gazebo version from the ros distro.
    if ros_distro == 'humble':
        gazebo_version = 'fortress'
    elif ros_distro == 'iron':
        gazebo_version = 'fortress'
    elif ros_distro == 'rolling':
        gazebo_version = 'fortress'
    else:
        print("ERROR: Unknown ros distro, can't evaluate gazebo version")

    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        name='world',
        description='Path to the world file to open.',
        default_value=PathJoinSubstitution(
            [FindPackageShare('walk_bot'), 'world', 'empty.sdf']))

    gui_config_arg = DeclareLaunchArgument(
        name='gui_config',
        description='Path to the GUI config file to open.',
        default_value=PathJoinSubstitution(
            [FindPackageShare('walk_bot'), 'gui_config', 'gui-' + gazebo_version + '.config']))

    xacro_path_arg = DeclareLaunchArgument(
        name='xacro_path',
        description='Path to the .xacro file to open.',
        default_value=PathJoinSubstitution(
            [FindPackageShare('walk_bot'), 'urdf', 'walk_bot.urdf']))

    z_arg = DeclareLaunchArgument(
        name='z',
        description='Initial base_link height in metres',
        default_value='0.3')

    # Gazebo with world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [
                # '-r ',  # Run simulation on start.
                LaunchConfiguration('world'),
                ' --gui-config ',
                LaunchConfiguration('gui_config'),
            ]
        }.items())

    # Create Walk bot model
    # Run xacro to convert walk_bot.urdf to a string containing the robot description
    create_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', Command(['xacro ', LaunchConfiguration('xacro_path')]),
            '-z', LaunchConfiguration('z')])

    # ROS <-> GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                # Clock (GZ -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Joint states (GZ -> ROS2)
                'joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                ],
    )

    # ROS <-> GZ Bridge for Joint Command
    joint_command_bridge = Node(package='walk_bot', executable='gz_bridge')

    return LaunchDescription([
        world_arg,
        gui_config_arg,
        xacro_path_arg,
        z_arg,
        gazebo,
        create_node,
        bridge,
        joint_command_bridge,
    ])
