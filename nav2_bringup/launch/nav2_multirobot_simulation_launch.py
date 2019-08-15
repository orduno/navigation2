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
# from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Create the launch configuration variables
    world = launch.substitutions.LaunchConfiguration('world')
    simulator = launch.substitutions.LaunchConfiguration('simulator')

    map_yaml_file = launch.substitutions.LaunchConfiguration('map')
    params_file = launch.substitutions.LaunchConfiguration('params')

    # Declare the launch arguments
    # TODO(orduno) replace with an `world_only`
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
        'map',
        default_value=os.path.join(launch_dir, 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'params',
        default_value=[launch.substitutions.ThisLaunchFileDir(), '/nav2_params.yaml'],
        # default_value=os.path.join(launch_dir, 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Specify the actions

    # TODO(orduno) Do here PushRosNamespace?

    start_gazebo_cmd = launch.actions.ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    # Spawn 2 robots
    
launch_ros.actions.Node(
            package='mars3d_spawn_tb3',
            node_executable='spawn_turtlebot',
            output='screen',
            arguments=[
                    launch.substitutions.LaunchConfiguration('robot_name'),
                    launch.substitutions.LaunchConfiguration('robot_name'),
                    launch.substitutions.LaunchConfiguration('x_pose'),
                    launch.substitutions.LaunchConfiguration('y_pose'),
                    launch.substitutions.LaunchConfiguration('z_pose')])
                
    # Create 2 instances of navigation



    return LaunchDescription([
        GroupAction([
            # PushRosNamespace('navigation_ns'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/nav2_simulation_launch.py']))
        ])
    ])
