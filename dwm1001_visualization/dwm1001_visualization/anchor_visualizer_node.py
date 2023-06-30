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

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from visualization_msgs.msg import Marker, MarkerArray


class AnchorVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("anchor_visualizer")

        self._declare_parameters()

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "anchor_markers", qos_profile
        )
        self.publisher_timer = self.create_timer(1, self._publish_markers)

    def _declare_parameters(self) -> None:
        initiator_anchor_position_description = ParameterDescriptor(
            description="Initiator anchor position (in meters) for the "
            + "DWM1001 network. Expected format is 'x,y,z'",
            type=ParameterType.PARAMETER_STRING,
            read_only=False,
        )

        self.declare_parameter(
            "initiator_anchor_position", "", initiator_anchor_position_description
        )

        anchor_positions_description = ParameterDescriptor(
            description="Non-initiator anchor positions (in meters) for the "
            + "DWM1001 network. Expected format is 'x,y,z;x,y,z;...;x,y,z'",
            type=ParameterType.PARAMETER_STRING,
            read_only=False,
        )

        self.declare_parameter("anchor_positions", "", anchor_positions_description)

    def _add_anchor_initiator_marker(self, msg: MarkerArray) -> None:
        position_string = self.get_parameter("initiator_anchor_position").value
        if not position_string:
            self.get_logger().warn("No initiator anchor position specified.")
            return

        position = position_string.split(",")
        if len(position) != 3:
            self.get_logger().error(
                "Initiator anchor's position improperly formatted. Not visualizing."
            )
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "dwm1001"

        marker.type = 1  # Cube
        marker.id = 0

        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.g = 1.0
        marker.color.a = 1.0

        msg.markers.append(marker)

        self.get_logger().info("Visualized initiator anchor.")

    def _add_anchor_markers(self, msg: MarkerArray) -> None:
        positions_string = self.get_parameter("anchor_positions").value
        if not positions_string:
            self.get_logger().warn("No non-initiator anchor positions specified.")
            return

        timestamp = self.get_clock().now().to_msg()
        position_strings = positions_string.split(";")
        visualized_markers = 0

        for index, position_string in enumerate(position_strings):
            position = position_string.split(",")
            if len(position) != 3:
                self.get_logger().error(
                    f"Non-initiator anchor {index + 1}'s position improperly "
                    + "formatted. Not visualizing."
                )
                continue

            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = "dwm1001"

            marker.type = 2  # Sphere
            marker.id = index + 1  # Must be unique, initiator anchor is ID 0

            marker.pose.position.x = float(position[0])
            marker.pose.position.y = float(position[1])
            marker.pose.position.z = float(position[2])

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.g = 1.0
            marker.color.a = 1.0

            msg.markers.append(marker)

            visualized_markers += 1

        self.get_logger().info(
            f"Visualized {visualized_markers} non-initiator anchors."
        )

    def _publish_markers(self) -> None:
        msg = MarkerArray()

        self._add_anchor_initiator_marker(msg)
        self._add_anchor_markers(msg)

        self.marker_publisher.publish(msg)

        # We only want to publish once
        self.publisher_timer.destroy()


def main(args=None):
    rclpy.init(args=args)

    anchor_visualizer_node = AnchorVisualizerNode()

    try:
        rclpy.spin(anchor_visualizer_node)
    except KeyboardInterrupt:
        pass
    finally:
        anchor_visualizer_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
