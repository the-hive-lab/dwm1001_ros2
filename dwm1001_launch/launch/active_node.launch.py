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


def generate_launch_description() -> LaunchDescription:
    dwm1001_driver = Node(
        package="dwm1001_driver",
        executable="active_tag",
        name="dw5188_tag",
        parameters=[{"serial_port": "/dev/ttyACM0"}],
    )

    # map_to_dwm1001_transform = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="map_to_dwm1001_broadcaster",
    #     # x, y, z, roll, pitch, yaw, frame_id, child_frame_id
    #     # arguments=["2.1336", "0", "0", "0", "0", "0", "map", "dwm1001"],
    #     arguments=["0", "0.95", "1.8", "0", "0", "0", "map", "dwm1001"],
    # )

    # anchor_visualizer = Node(
    #     package="dwm1001_visualization",
    #     executable="anchor_visualizer",
    #     name="anchor_visualizer",
    #     parameters=[
    #         {
    #             "initiator_anchor_position": "0.0,0.0,1.8",
    #             "anchor_positions": "0.0,5.02,1.8;4.98,5.96,1.8;9.55,4.85,1.82;9.55,0.84,1.82",
    #             # "initiator_anchor_position": "0.00,0.00,1.03",
    #             # "anchor_positions": "-2.13,2.13,0.94;2.74,1.52,1.04;0.61,3.05,1.02",
    #         }
    #     ],
    # )

    # dwm_transform = Node(
    #     package="dwm1001_transform",
    #     executable="dwm1001_transform",
    #     name="dwm1001_transform",
    #     remappings=[("tag_position", "DW0414")],
    # )

    return LaunchDescription(
        [dwm1001_driver] #map_to_dwm1001_transform, anchor_visualizer, dwm_transform]
    )
