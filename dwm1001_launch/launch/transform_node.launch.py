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
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    tag_topic_value = LaunchConfiguration('tag_topic')
    tag_id_arg = DeclareLaunchArgument(
        'tag_topic',
        default_value=TextSubstitution(text='output/DW5188'),
        description="The tag's output topic needed for the transform node."
    )

    dwm_transform = Node(
        package="dwm1001_transform",
        executable="dwm1001_transform",
        name="dwm1001_transform",
        remappings=[("input/tag_position", tag_topic_value)],
    )

    return LaunchDescription(
        [tag_id_arg, dwm_transform]
    )