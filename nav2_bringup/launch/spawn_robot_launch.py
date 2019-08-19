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

from launch import LaunchDescription

import launch.actions
import launch_ros.actions

def generate_launch_description():

    return LaunchDescription([
        launch.actions.LogInfo(
            msg=launch.substitutions.LaunchConfiguration('robot_name')),
        launch_ros.actions.Node(
            package='nav2_bringup',
            node_executable='spawn_turtlebot',
            output='screen',
            arguments=[
                    launch.substitutions.LaunchConfiguration('robot_name'),
                    launch.substitutions.LaunchConfiguration('robot_name'),
                    launch.substitutions.LaunchConfiguration('x_pose'),
                    launch.substitutions.LaunchConfiguration('y_pose'),
                    launch.substitutions.LaunchConfiguration('z_pose')]),
    ])