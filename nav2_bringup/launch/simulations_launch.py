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

''' Helper file for launching multiple simulation instances '''

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
# from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Create the launch configuration variables
    num_robots = launch.substitutions.LaunchConfiguration('num_robots')
    world = launch.substitutions.LaunchConfiguration('world')
    simulator = launch.substitutions.LaunchConfiguration('simulator')

    map_yaml_file = launch.substitutions.LaunchConfiguration('map_yaml')
    params_file = launch.substitutions.LaunchConfiguration('params_yaml')

    # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = launch.actions.ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    # Spawn the robots into Gazebo
    spawn_robots_cmd = launch.actions.IncludeLaunchDescription(
        # TODO(orduno) Try ThisLaunchFileDir
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_robots.py')),
        launch_arguments={'num_robots': num_robots}.items())

    # Launch the simulation instances
    start_nav_instances_cmd = launch.actions.IncludeLaunchDescription(
        # TODO(orduno) Try ThisLaunchFileDir
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav_instances_launch')),
        launch_arguments={'num_robots': num_robots,
                          'world': world,
                          'map_yaml_file', map_yaml,
                          'params_file', params_file}.items())

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_robots_cmd)
    ld.add_action(start_simulations_cmd)

    return ld
