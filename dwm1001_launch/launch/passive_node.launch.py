# Copyright 2023 The Human and Intelligent Vehicle Ensembles (HIVE) Lab
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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:

    tag_namespace_val = LaunchConfiguration('tag_namespace')
    tag_namespace_arg = DeclareLaunchArgument(
            'tag_namespace',
            default_value=TextSubstitution(text='passive'),
            description='Name of the tag namespace'
        )
    
    config_file_val = LaunchConfiguration('config_file')
    default_config_file_path = PathJoinSubstitution([FindPackageShare('dwm1001_launch'),
                                                     'config',
                                                     'default_passive.yaml'])
    config_file_launch_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file_path,
        description='Configuration file for the active node'
    )

    dwm1001_driver = Node(
        package="dwm1001_driver",
        executable="passive_tag",
        namespace=tag_namespace_val,
        parameters=[PathJoinSubstitution([FindPackageShare('dwm1001_launch'), 'config', config_file_val])],
    )

    return LaunchDescription(
        [tag_namespace_arg, dwm1001_driver]
    )
