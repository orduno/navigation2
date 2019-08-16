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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch.actions
import launch_ros.actions

def generate_launch_description():
    num_robots = launch.substitutions.LaunchConfiguration('num_robots')

    # TODO(orduno) Remove hardcoded robot number and positions

    # TODO(orduno) Try ThisLaunchFileDir
    launchFile = os.path.join(get_package_share_directory(
        'nav2_bringup'), 'launch', 'spawn_robot_launch.py')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'num_robots', default_value='2'
            description='Number of robots to add to the system')

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launchFile]),
            launch_arguments={'x_pose': '-2.0',
                              'y_pose': '-0.5',
                              'z_pose': '0.01',
                              'robot_name': 'Robot1'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launchFile]),
            launch_arguments={'x_pose': '2.0',
                              'y_pose': '0.5',
                              'z_pose': '0.01',
                              'robot_name': 'Robot2'}.items(),
        ),
    ])
