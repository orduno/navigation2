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

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition

import launch.actions
import launch_ros.actions


def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Create the launch configuration variables
    robot_name = launch.substitutions.LaunchConfiguration('robot_name')
    map_yaml_file = launch.substitutions.LaunchConfiguration('map_yaml_file')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    params_file = launch.substitutions.LaunchConfiguration('params_file')
    bt_xml_file = launch.substitutions.LaunchConfiguration('bt_xml_file')
    autostart = launch.substitutions.LaunchConfiguration('autostart')
    nodes_args = launch.substitutions.LaunchConfiguration('nodes_args')
    log_settings = launch.substitutions.LaunchConfiguration('log_settings',
                                                             default='true')

    stdout_linebuf_envvar = launch.actions.SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Declare the launch arguments
    declare_robot_name_cmd = launch.actions.DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Identification name for the robot')

    declare_map_yaml_cmd = launch.actions.DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(launch_dir, 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'params_file',
        default_value=[launch.substitutions.ThisLaunchFileDir(), '/nav2_params.yaml'],
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = launch.actions.DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(
            get_package_prefix('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = launch.actions.DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_nodes_args_cmd = launch.actions.DeclareLaunchArgument(
        'nodes_args', default_value='',
        description='Arguments to pass to all nodes launched by the file')

    # Specify the actions
    start_localization_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_localization_launch.py')),
        launch_arguments={'robot_name': robot_name,
                          'map_yaml_file': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'params_file': params_file,
                          'use_lifecycle_mgr': 'false',
                          'nodes_args': nodes_args}.items())

    start_navigation_cmd = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_navigation_launch.py')),
      launch_arguments={'robot_name': robot_name,
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'params_file': params_file,
                        'bt_xml_file': bt_xml_file,
                        'use_lifecycle_mgr': 'false',
                        'nodes_args': nodes_args}.items())

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        node_executable='lifecycle_manager',
        node_name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart}],
        arguments=[nodes_args])

    log_robot_name_cmd = launch.actions.LogInfo(
        condition=IfCondition(log_settings),
        msg=['Robot name: ', robot_name])

    log_autostart_cmd = launch.actions.LogInfo(
        condition=IfCondition(log_settings),
        msg=['Autostart: ', autostart])

    log_use_sim_time_cmd = launch.actions.LogInfo(
        condition=IfCondition(log_settings),
        msg=['Use sim time: ', use_sim_time])

    log_map_yaml_cmd = launch.actions.LogInfo(
        condition=IfCondition(log_settings),
        msg=['Map yaml: ', map_yaml_file])

    log_params_yaml_cmd = launch.actions.LogInfo(
        condition=IfCondition(log_settings),
        msg=['Params yaml: ', params_file])

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_nodes_args_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_localization_cmd)
    ld.add_action(start_navigation_cmd)

    ld.add_action(log_robot_name_cmd)
    ld.add_action(log_autostart_cmd)
    ld.add_action(log_use_sim_time_cmd)
    ld.add_action(log_map_yaml_cmd)
    ld.add_action(log_params_yaml_cmd)

    return ld
