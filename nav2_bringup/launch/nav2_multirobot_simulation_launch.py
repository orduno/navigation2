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

''' This is an example on how to spawn multiple robots into Gazebo
    and launch multiple instances of the navigation stack,
    each controlling one robot. Robots co-exist on the same environment. '''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

import launch.actions
import launch_ros.actions


def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Names and poses of the robots
    # TODO(orduno) provide pose as a number not text and use text substitution down
    robots = [
        {'name': 'Robot1', 'x_pose': '-2.0', 'y_pose': '-0.5', 'z_pose': '0.01'},
        {'name': 'Robot2', 'x_pose': '2.0', 'y_pose': '0.5', 'z_pose': '0.01'}]

    # Create the launch configuration variables
    world = launch.substitutions.LaunchConfiguration('world')
    simulator = launch.substitutions.LaunchConfiguration('simulator')

    map_yaml_file = launch.substitutions.LaunchConfiguration('map_yaml')
    params_file = launch.substitutions.LaunchConfiguration('params_yaml')

    # Declare the launch arguments
    declare_world_cmd = launch.actions.DeclareLaunchArgument(
        'world',
        default_value=[ThisLaunchFileDir(), '/world_only.model'],
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
        default_value=[ThisLaunchFileDir(), '/nav2_params.yaml'],
        description='Full path to the ROS2 parameters file to use for all launched nodes')

   # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = launch.actions.ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    # Spawn two robots into Gazebo
    spawn_robots_cmd = []
    for robot in robots:
        spawn_robots_cmd.append(
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_robot_launch.py')),
                launch_arguments={
                    #TODO(orduno) Use text substitution
                                  'x_pose': robot['x_pose'],
                                  'y_pose': robot['y_pose'],
                                  'z_pose': robot['z_pose'],
                                  'robot_name': robot['name']
                                  }.items()
            )
        )

    # TODO(orduno)
    # robot_ns = launch_ros.actions.PushRosNamespace(robot_name)

    # If a robot name is provided, the transforms need to be namespaced
    # Also, several topics where defined with an absolute namespace, i.e. /map

    # Unfortunately, TF2 doesn't provide a way to namespace tranforms
    # https://github.com/ros/geometry2/issues/32
    # The solution for now is to remap the transform topics

    # TODO(orduno) Ideally we'd like to directly obtain the remapping from the parent launch
    #              but there doesn't seem to be a way to do this cleanly in the `launch` pkg

    # remappings = []
    # if IfCondition(remap_transforms):
    #     remappings.append(((robot_name, '/tf'), '/tf'))
    #     remappings.append(((robot_name, '/tf_static'), '/tf_static'))
    #     remappings.append(('/scan', 'scan'))
    #     remappings.append(('/tf', 'tf'))
    #     remappings.append(('/tf_static', 'tf_static'))
    #     # TODO(orduno) change topics to relative namespaces in the stack
    #     remappings.append(('/cmd_vel', 'cmd_vel'))
    #     remappings.append(('/map', 'map'))

    # start_simulation_instances_cmd = launch.actions.IncludeLaunchDescription(
    #     # TODO(orduno) Try ThisLaunchFileDir
    #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'simulations_launch.py')),
    #     launch_arguments={'num_robots': num_robots,
    #                       'world': world,
    #                       'simulator': simulator,
    #                       'map_yaml_file': map_yaml,
    #                       'params_file': params_yaml}.items())

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for spawn_robot in spawn_robots_cmd:
        ld.add_action(spawn_robot)

    # ld.add_action(start_simulation_instances_cmd)

    return ld
