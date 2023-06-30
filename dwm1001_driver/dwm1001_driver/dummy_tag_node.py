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

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import PointStamped, TransformStamped


class DummyTagNode(Node):
    def __init__(self) -> None:
        super().__init__("dummy_tag")

        self._declare_parameters()

        self.tag_id = self.get_parameter("tag_id").value
        if not self.tag_id:
            self._shutdown_fatal("No DWM1001 tag identifier specified.")

        self.get_logger().info("Started position reporting.")

        self.publisher = self.create_publisher(PointStamped, self.tag_id, 1)
        self.timer = self.create_timer(1 / 10, self.timer_callback)

    def _shutdown_fatal(self, message: str) -> None:
        self.get_logger().fatal(message + " Shutting down.")
        exit()

    def _declare_parameters(self):
        tag_id = ParameterDescriptor(
            description="DWM1001 tag identifier",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )

        self.declare_parameter("tag_id", "", tag_id)

    def timer_callback(self):
        time_stamp = self.get_clock().now().to_msg()

        msg = PointStamped()

        msg.header.stamp = time_stamp
        msg.header.frame_id = "dwm1001"

        msg.point.x = 1.2
        msg.point.y = 2.3
        msg.point.z = 3.4

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    dummy_listener = DummyTagNode()
    rclpy.spin(dummy_listener)

    dummy_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
