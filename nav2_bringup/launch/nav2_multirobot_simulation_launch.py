# Copyright (c) 2018 Intel Corporation
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

''' Launch multiple robots on separate namespaces a on simulated environment.
    Intended for use by developers. '''

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Create the launch configuration variables
    num_robots = launch.substitutions.LaunchConfiguration('num_robots')
    world = launch.substitutions.LaunchConfiguration('world')
    simulator = launch.substitutions.LaunchConfiguration('simulator')

    map_yaml_file = launch.substitutions.LaunchConfiguration('map_yaml')
    params_file = launch.substitutions.LaunchConfiguration('params_yaml')

    # Declare the launch arguments
    declare_num_robots_cmd = launch.actions.DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots to add to the system')

    # TODO(orduno) replace with `world_only` model
    declare_world_cmd = launch.actions.DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                                   'worlds/turtlebot3_worlds/waffle.model'),
        description='Full path to world file to load')

    declare_simulator_cmd = launch.actions.DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_map_yaml_cmd = launch.actions.DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.join(launch_dir, 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'params_yaml',
        default_value=[launch.substitutions.ThisLaunchFileDir(), '/nav2_params.yaml'],
        # default_value=os.path.join(launch_dir, 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    start_simulation_instances_cmd = launch.actions.IncludeLaunchDescription(
        # TODO(orduno) Try ThisLaunchFileDir
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'simulations_launch.py')),
        launch_arguments={'num_robots': num_robots,
                          'world': world,
                          'simulator': simulator,
                          'map_yaml_file': map_yaml,
                          'params_file': params_yaml}.items())

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(start_simulation_instances_cmd)

    return ld
