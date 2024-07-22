# Copyright 2024 RyuYamamoto.
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
# limitations under the License

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    navyu_slam_config = PathJoinSubstitution(
        [FindPackageShare("navyu_slam"), "config", "navyu_slam_params.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="navyu_slam",
                executable="navyu_slam_node",
                name="navyu_slam_node",
                output="screen",
                parameters=[navyu_slam_config],
            ),
        ]
    )
