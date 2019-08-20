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

''' This is all-in-one launch script intended for use by nav2 developers. '''

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

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

    # Launch configuration variables specific to simulation
    rviz_config_file = launch.substitutions.LaunchConfiguration('rviz_config')
    use_simulator = launch.substitutions.LaunchConfiguration('use_simulator')
    simulator = launch.substitutions.LaunchConfiguration('simulator')
    world = launch.substitutions.LaunchConfiguration('world')

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
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'params_file',
        default_value=[launch.substitutions.ThisLaunchFileDir(), '/nav2_params.yaml'],
        # default_value=os.path.join(launch_dir, 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = launch.actions.DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(
            get_package_prefix('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = launch.actions.DeclareLaunchArgument(
        'autostart', default_value='false',
        description='Automatically startup the nav2 stack')

    declare_nodes_args_cmd = launch.actions.DeclareLaunchArgument(
        'nodes_args', default_value='',
        description='Arguments to pass to all nodes launched by the file')

    #TODO(orduno) modify to the default single robot view
    #TODO(orduno) is there a way to pass the robot name so topics in RVIZ are namespaced correctly?
    declare_rviz_config_file_cmd = launch.actions.DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(launch_dir, 'nav2_multi_robot_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = launch.actions.DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_simulator_cmd = launch.actions.DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_world_cmd = launch.actions.DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                                   'worlds/turtlebot3_worlds/waffle.model'),
        description='Full path to world file to load')

    # TODO(orduno) move this to the multi-robot launch script?
    # Instances are separated using different
    # Robot specific settings
    robot_ns = launch_ros.actions.PushRosNamespace(robot_name)

    # Specify the actions
    start_gazebo_cmd = launch.actions.ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=[simulator, '-s', 'libgazebo_ros_init.so', world],
        cwd=[launch_dir], output='screen')

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle.urdf')

    start_robot_state_publisher_cmd = launch_ros.actions.Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf, nodes_args])

    # TODO(orduno) rviz crashing if launched as a node: Unknown option 'ros-args'
    start_rviz_cmd = launch.actions.ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('rviz2'), 'lib/rviz2/rviz2'),
            # TODO(orduno) re-enable
            # ['-d', rviz_config_file],
            ['__ns:=/', robot_name],
            ['/move_base_simple/goal:=move_base_simple/goal'],
            ['/tf:=tf'],
            ['/tf_static:=tf_static'],
            ['/clicked_point:=clicked_point'],
            ['/initialpose:=initialpose'],
            ['/parameter_events:=parameter_events'],
            ['/rosout:=rosout']],
        cwd=[launch_dir], output='screen')

    exit_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=launch.actions.EmitEvent(event=launch.events.Shutdown(reason='rviz exited'))))

    bringup_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_bringup_launch.py')),
        launch_arguments={'robot_name': robot_name,
                          'map_yaml_file': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'bt_xml_file': bt_xml_file,
                          'autostart': autostart,
                          'nodes_args': nodes_args}.items())

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_nodes_args_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions to launch in simulation (conditioned on 'use_simulator')
    ld.add_action(start_gazebo_cmd)

    # Add other nodes and processes we need
    ld.add_action(start_rviz_cmd)
    ld.add_action(exit_event_handler)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(robot_ns)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_cmd)

    return ld
